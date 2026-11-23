/**
 * @file    swing_init.c
 * @brief   回转控制统一初始化入口
 */

#include "swing_control.h"

/* 【2026-11-10 彩标 P2】回转初始化：注册失败=ERR，完成=OK */
#include "../log/log_color.h"
#include "swing_angle.h"
#include "swing_axis.h"
#include "../motor/motor.h"
#include "../motor/motor_config.h"
#include "../motor/drv/hw_encoder.h"      /* H1-A：ABZ 真实计数 */
#include "../angle/mech_cfg.h"            /* mech_div_round64 */
#include "../angle/mech_cfg_store.h"      /* mech_get_rt */
#include "swing_events.h"
#include "../sys_param/sys_param.h"
#include "../config/sys_config.h"
#include "framework/state_engine/sm_assert.h"
#include "../framework/manager/ctrl_manager.h"
#include "../service/input_sim.h"   /* 台架模拟（任务清单 §11） */
#include <rtthread.h>

/* ============================================================
 * 编码器读取函数（H1-A：改为 ABZ 真实计数）
 * ============================================================
 * 旧实现返回 motor->current_position，但该字段的写点被 MARK-13 注释，
 * 恒为 0，导致筒角/限位/过零全部失效。
 * H1-A 直接读 FOC 的 ABZ 计数，用 mech 的 out_num/out_den 换成 0.1°：
 *   angle_x10 = count * out_num / out_den
 * 仍用 swing_dir_polarity 统一“角度增大=右转”的符号。
 * 零点由 t_axis zero 写入 encoder_zero_offset（RAM）。
 * ============================================================ */
static int32_t swing_encoder_getter(void *user_data)
{
    motor_t *motor = (motor_t*)user_data;

    (void)motor;

#if (INPUT_SIM_ENABLE && SWING_ANGLE_SIM_ENABLE)
    /* 台架模拟优先：t_sim swing <x10> 直接给出 raw（0.1°）。 */
    if (input_sim_active(INPUT_SIM_SWING) != 0U) {
        return input_sim_value(INPUT_SIM_SWING);
    }
#endif

    {
        const mech_rt_t *rt = mech_get_rt();
        int32_t x10;

        if ((rt == RT_NULL) || (rt->out_den == 0U)) {
            return 0;
        }

        x10 = mech_div_round64((int64_t)hw_encoder_get_count() *
                               (int64_t)rt->out_num,
                               (int64_t)rt->out_den);
        if (g_swing_dir_polarity != 0U) {
            x10 = -x10;
        }
        return x10;
    }
}

/* ============================================================
 * 解算有效性 → ctx.encoder_fault（2026-11-23 新增）
 * ------------------------------------------------------------
 * 这是 encoder_fault 的**唯一真实生产者**（另一个是台架 t_encfault）。
 * 单一口径：不管来源是解算失效还是将来的 SPI 掉线，都只写这一个字段，
 * 处置也完全一样（停机 + 关阀 + 抱闸 + 拦一键；手动放行，Q30-a）。
 *
 * ⚠【启用 SPI 绝对式编码器前必读】业务.md §13.8.7
 *   SPI 看门狗落地时**不要**直接写 ctx.encoder_fault，必须走同一个入口，
 *   并把 1/0 语义升级为来源位掩码（SOLVE / SPI），否则两源互相覆盖。
 * ============================================================ */
static void swing_angle_fault_to_ctx(uint8_t fault)
{
    g_swing_obj.ctx.encoder_fault = (fault != 0U) ? 1U : 0U;

    LOG_WARN("[Swing] encoder_fault -> %u\n", (unsigned)g_swing_obj.ctx.encoder_fault);
}

/* ============================================================
 * 初始化
 * ============================================================ */
int swing_init(void)
{
    rt_kprintf("[SWING] swing_init()\n");
    rt_err_t ret;
    sys_param_t *sys_param = sys_param_get();

    /* 0. 装载方向极性（2026-11-09 新增）
     *
     * 读取失败时保持 swing_control.c 的静态初值 0（正逻辑），
     * 这是"左转=减小角度"的默认约定，属安全侧默认。
     *
     * ⚠ 本项 flags = CFG_F_REQ_RESTART，运行中改不会立即生效，
     *   因为极性翻转会让正在转动的电机突然反向。
     */
    {
        int32_t v = 0;

        if (sys_config_get_i32(CFG_ID_SWING_DIR_POLARITY, &v) == 0) {
            g_swing_dir_polarity = (v != 0) ? 1U : 0U;
        } else {
            g_swing_dir_polarity = 0U;
        }

        rt_kprintf("[SWING] dir polarity = %u (%s)\n",
                   (unsigned)g_swing_dir_polarity,
                   (g_swing_dir_polarity != 0U) ? "NEG: L+=, R-="
                                                : "POS: L-=, R+=");
    }

    /* 1. 初始化回转角度对象（绑定编码器 + 事件回调） */
    swing_angle_init_with_events(&g_swing_angle,
                                 sys_param->encoder_zero_raw,
                                 swing_encoder_getter,
                                 &g_swing_motor);

    /* 1a. 解算有效性 → ctx.encoder_fault（2026-11-23） */
    g_swing_angle.on_fault_change = swing_angle_fault_to_ctx;

    /* 1b. 【P6 / B-8】手动过零停车开关（TC-MAN-004）
     *  1 = 到 0° 停车 + 锁刹车，必须松开重按才继续（默认，对应用例）；
     *  0 = 手动可直接过零（旧行为，现场可退回）。
     * 只影响手动 LEFT/RIGHT_RUNNING 两个状态；一键走位置环，不进这两个状态。 */
    {
        int32_t v = CFG_DEF_SWING_ZERO_STOP;

        if (sys_config_get_i32(CFG_ID_SWING_ZERO_STOP, &v) != 0) {
            v = CFG_DEF_SWING_ZERO_STOP;
        }
        g_swing_angle.need_zero_stop = (v != 0) ? 1U : 0U;
        rt_kprintf("[SWING] zero stop = %u (%s)\n",
                   (unsigned)g_swing_angle.need_zero_stop,
                   (g_swing_angle.need_zero_stop != 0U) ? "stop at 0deg, re-press"
                                                        : "pass through");
    }

    /* 1c. H1-A 轴适配层 */
    swing_axis_init();

    /* 2. 注册回转控制对象 */
    extern ctrl_manager_t g_ctrl_mgr;
    ret = swing_control_register(&g_ctrl_mgr);
    if (ret != 0) {
        LOG_ERR("[Swing] Register failed\n");
        return -1;
    }

    LOG_OK("[Swing] Initialized.\n");
    return 0;
}
//INIT_DEVICE_EXPORT(swing_init);
