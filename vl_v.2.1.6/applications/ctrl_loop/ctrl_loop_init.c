/**
 * @file    ctrl_loop_init.c
 * @brief   三环模块的实例创建与周期驱动
 *
 * @note    【为什么用独立线程而不是 bg_scheduler】
 *            bg_sched 线程优先级 15（最低，见 bg_modules.c 注释），
 *            速度环要求 1ms 稳定节拍，会被 finsh / CAN 解析抢占。
 *            独立线程优先级 6，高于背景任务，低于 CAN 解析（4）。
 *
 *          【为什么线程周期用 1ms 而不是更快】
 *            位置环 5ms、速度环 1ms，1ms 是最快环的周期。
 *            再快无意义（tick 分辨率就是 1ms）。
 *            电流环 20kHz 走 PWM ISR 另一条路（MARK-CL-01 预留）。
 */

#include "ctrl_loop.h"
#include "ctrl_loop_cfg.h"
#include "../motor/motor.h"
#include "../motor/motor_config.h"
#include "../angle/motor_speed.h"
#include "../lift/lift_angle.h"
#include "../lift/lift_control.h"
#include "../mode/sys_caps.h"

#include <rtthread.h>
#include <string.h>

/* ============================================================
 * 实例存储（静态分配，不用 malloc）
 * ============================================================ */
static ctrl_loop_t s_loop_swing;    /* 实例 0：回转电机 */
static ctrl_loop_t s_loop_lift;     /* 实例 1：举升机构 */
/* 实例 2、3 预留，未创建 */

/* 线程
 * ⚠ 这三个宏的数值**不要加括号**：
 *   tools/stack/verify.py 用正则 `#define\s+KEY\s+(\d+)` 解析，
 *   写成 (1024) 会解析失败，栈统计表里显示 alloc=None。
 *   工程内 KB_THREAD_STACK / CAN_TX_THREAD_STACK 都是无括号写法，
 *   此处保持一致。 */
#define CL_THREAD_STACK     1024
#define CL_THREAD_PRIO      6
#define CL_THREAD_TICK      5

static rt_thread_t s_cl_thread = RT_NULL;

/* ============================================================
 * 实例 0 的钩子：回转电机
 * ------------------------------------------------------------
 * 反馈源：
 *   位置 → motor->current_position（⚠ 当前恒 0，见 MARK-13）
 *   速度 → motor_speed_get()，返回 0.01 RPM
 *   电流 → motor_sample_current_a10()，返回 0.1A
 *
 * 输出：
 *   速度环输出直接作为占空比（当前六步方波开环模式）
 *   值域 0~1000 对应 0~100%
 * ============================================================ */
static int32_t _swing_get_fb(ctrl_loop_t *self, ctrl_loop_kind_t kind)
{
    motor_t *m = (motor_t *)self->user_data;

    if (m == RT_NULL) { return 0; }

    switch (kind) {
        case CTRL_LOOP_POSITION:
            /* ⚠ MARK-13：current_position 当前恒 0（唯一写入点被注释）。
             * 修好之前位置环不可用，见 motor_fb_abs_encoder.c 的 MARK-13。 */
            return m->current_position;

        case CTRL_LOOP_SPEED: {
            /* 返回输出轴转速，单位 RPM（整数）。
             *
             * motor_speed_get() 的签名是 (int32_t *rpm_out_x100)，
             * 返回 1 表示有效、0 表示尚未算出（窗口未满）。
             * 无效时返回 0 —— PID 会认为是"转速为 0"，
             * 若此时使能速度环会输出满占空比。
             * ⚠ 因此速度环的使能必须由上层在"转速已有效"后才打开，
             *   或在此处返回上一次有效值（当前简化处理为返回 0）。 */
            int32_t rpm_x100 = 0;

            if (motor_speed_get(&rpm_x100) == 0U) {
                return 0;
            }
            return rpm_x100 / 100;
        }

        case CTRL_LOOP_CURRENT:
            /* 0.1A 单位。注意母线电流物理上单向（无符号），
             * 但 PID 内部按有符号处理，故 cast 成 int32 是安全的。 */
            return (int32_t)motor_sample_current_a10(m);

        default:
            return 0;
    }
}

static void _swing_set_out(ctrl_loop_t *self, ctrl_loop_kind_t kind,
                            int32_t out)
{
    motor_t *m = (motor_t *)self->user_data;

    (void)kind;

    if (m == RT_NULL) { return; }

    /* 输出量纲：0~1000 = 0~100% 占空比
     * 直接写 target_output，由 motor_update 里的驱动层换算成 PWM 比较值。 */
    if (out < 0)    { out = 0; }
    if (out > 1000) { out = 1000; }
    m->target_output = out;
}

/* ============================================================
 * 实例 1 的钩子：举升机构
 * ------------------------------------------------------------
 * 举升是液压阀（开关量），不是连续量。
 * 因此位置环输出需要**离散化**：>+阈值 开上升阀，<-阈值 开下降阀。
 *
 * 这正好演示了本模块的通用性：同一个三环模块，
 * 对电机是连续占空比，对液压阀是开关量，差异只在 set_out 里。
 * ============================================================ */
static int32_t _lift_get_fb(ctrl_loop_t *self, ctrl_loop_kind_t kind)
{
    (void)self;

    if (kind == CTRL_LOOP_POSITION) {
        return lift_angle_get_value(&g_lift_angle);
    }
    /* 举升无速度/电流反馈，返回 0 */
    return 0;
}

static void _lift_set_out(ctrl_loop_t *self, ctrl_loop_kind_t kind,
                           int32_t out)
{
    (void)kind;

    /* 输出离散化：死区 ±5（0.5°）防止在目标附近抖动开关阀 */
    lift_context_t *lctx = (lift_context_t *)self->user_data;

    if ((lctx == RT_NULL) || (lctx->solenoid_ptr == RT_NULL)) {
        return;
    }

    if (out > 5) {
        lctx->solenoid_ptr->valve_output |= 0x01;        /* 上升阀 */
        lctx->solenoid_ptr->valve_output &= (uint8_t)~0x02U;
    } else if (out < -5) {
        lctx->solenoid_ptr->valve_output |= 0x02;        /* 下降阀 */
        lctx->solenoid_ptr->valve_output &= (uint8_t)~0x01U;
    } else {
        lctx->solenoid_ptr->valve_output = 0;
    }
}

/* ============================================================
 * 周期线程
 * ============================================================ */
static void _cl_thread_entry(void *param)
{
    uint32_t now;
    uint32_t last = rt_tick_get();

    (void)param;

    while (1) {
        now = rt_tick_get();

        /* 只在线程被实际调度时推进，用真实 elapsed 而非固定值，
         * 这样即使线程被抢占，各环的 dt 也是准确的。 */
        if (now != last) {
            uint8_t i;
            uint8_t n = ctrl_loop_inst_count();

            for (i = 0U; i < n; i++) {
                ctrl_loop_t *l = ctrl_loop_get(i);
                if (l != RT_NULL) {
                    ctrl_loop_tick(l, now);
                }
            }
            last = now;
        }

        rt_thread_mdelay(1);
    }
}

/* ============================================================
 * 初始化
 * ============================================================ */
int ctrl_loop_init_all(void)
{
    int ret;

    /* ---- 1. 校验配置表登记完整性 ----
     * 若 sys_config.c 里的 s_cfg_def[] 与实际参数数不一致，
     * 这里会打印 WARN，避免"读到的全是默认值"这种静默故障。 */
    (void)ctrl_loop_cfg_register();

    /* ---- 2. 创建实例 0：回转 ----
     * user_data 传 motor 对象指针，钩子里直接用。 */
    ret = ctrl_loop_init(&s_loop_swing, "swing",
                         _swing_get_fb, _swing_set_out,
                         (void *)&g_swing_motor);
    if (ret != 0) {
        rt_kprintf("[CtrlLoop] swing init failed\n");
        return -1;
    }
    (void)ctrl_loop_cfg_load(0U, &s_loop_swing);

    /* ---- 3. 创建实例 1：举升 ----
     * user_data 传举升控制上下文，用于 set_out 里操作电磁阀。 */
    ret = ctrl_loop_init(&s_loop_lift, "lift",
                         _lift_get_fb, _lift_set_out,
                         (void *)&g_lift_obj.ctx);
    if (ret != 0) {
        rt_kprintf("[CtrlLoop] lift init failed\n");
        return -1;
    }
    (void)ctrl_loop_cfg_load(1U, &s_loop_lift);

    /* ---- 4. 启动周期线程 ---- */
    s_cl_thread = rt_thread_create("ctrl_loop",
                                   _cl_thread_entry,
                                   RT_NULL,
                                   CL_THREAD_STACK,
                                   CL_THREAD_PRIO,
                                   CL_THREAD_TICK);
    if (s_cl_thread == RT_NULL) {
        rt_kprintf("[CtrlLoop] thread create failed\n");
        return -1;
    }
    rt_thread_startup(s_cl_thread);

    rt_kprintf("[CtrlLoop] %u instance(s), thread prio=%u period=1ms\n",
               (unsigned)ctrl_loop_inst_count(),
               (unsigned)CL_THREAD_PRIO);
    rt_kprintf("[CtrlLoop] NOTE: position feedback is 0 until MARK-13 fixed\n");

    return 0;
}