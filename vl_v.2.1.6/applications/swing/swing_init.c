/**
 * @file    swing_init.c
 * @brief   回转控制统一初始化入口
 */

#include "swing_control.h"

/* 【2026-11-10 彩标 P2】回转初始化：注册失败=ERR，完成=OK */
#include "../log/log_color.h"
#include "swing_angle.h"
#include "../motor/motor.h"
#include "../motor/motor_config.h"
#include "swing_events.h"
#include "../sys_param/sys_param.h"
#include "../config/sys_config.h"
#include "framework/state_engine/sm_assert.h"
#include "../framework/manager/ctrl_manager.h"
#include "../service/input_sim.h"   /* 台架模拟（任务清单 §11） */
#include <rtthread.h>

/* ============================================================
 * 编码器读取函数（绑定到 motor_t）
 * ============================================================
 * 【MARK-13 · 2026-11-09 登记：本函数当前恒返回 0】
 *
 * motor->current_position 的唯一写入点在
 *   motor/fb/motor_fb_abs_encoder.c:85  → 已被整段注释
 * 且 motor_config.c:94 启用的 motor_fb_hall_ops.get_position = NULL。
 * ⇒ 该字段恒为 .bss 初值 0 ⇒ 回转角度恒 0.0°。
 *
 * 用户裁定：先记录、后续单独排期，不改动。
 * 排期前提：三编码器大角度解算问题修复后一并处理。
 * 详见 motor_fb_abs_encoder.c 的 MARK-13 说明。
 *
 * ⚠ 在 MARK-13 修复前：
 *   - 方向极性配置（CFG_ID_SWING_DIR_POLARITY）无法实测验证
 *   - 零位/限位判定全部失效
 *   - 序列 SWING_NEAR 在目标非 0 时必然超时
 * ============================================================ */
static int32_t swing_encoder_getter(void *user_data)
{
    motor_t *motor = (motor_t*)user_data;
    SM_ASSERT_PTR(motor);

#if (INPUT_SIM_ENABLE && SWING_ANGLE_SIM_ENABLE)
    /* 台架模拟：t_sim swing <x10> 直接给出编码器 raw（0.1°），
     * 角度 = raw − 零点，与真实路径同一换算。 */
    if (input_sim_active(INPUT_SIM_SWING) != 0U) {
        return input_sim_value(INPUT_SIM_SWING);
    }
#endif

    return motor->current_position;
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
