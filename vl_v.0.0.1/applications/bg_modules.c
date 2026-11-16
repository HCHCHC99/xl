/**
 * @file    bg_modules.c
 * @brief   后台模块注册表实例
 * @note    所有非实时周期性任务在此集中注册
 */

#include "bg_scheduler.h"
#include "device_info/device_info.h"
#include "motor/motor.h"                 /* 【R7】motor_update */
#include "motor/motor_stall.h"           /* 【S1】stall_on_tick */
#include "hw/hw_brake.h"                 /* 【2026-11-10】hw_brake_flush_log */
#include <stdint.h>
#include <stddef.h>

/* ============================================================
 * 后台任务函数声明（外部实现）
 * ============================================================ */
/* bsp_dev_led.c */
extern void led_init(void);
extern void led_task(void);

/* bsp_dev_key.c (按键消抖) */
extern void key_filter_task(void);

/* bsp_dev_wdt.c (看门狗) */
extern void wdt_init(void);
extern void wdt_feed(void);

/* ============================================================
 * 【2026-11-04 / R7】电机周期更新
 * ------------------------------------------------------------
 * 【为什么必须放在这里】
 *   motor_update() 原本全工程零调用者（缺陷 A-16），
 *   导致：
 *     - motor_sample_current_a10() 从不被调用 → 电流恒为初值
 *     - motor_oc_update() 从不执行 → 过流保护完全不工作
 *     - ctrl_ops->update() 从不执行 → PWM 永不写入
 *     - drv_ops->tick() 从不执行 → 强拖/换相状态机不走
 *   一句话：整条电机控制链是断的，电机根本不会转。
 *
 * 【为什么周期写 5ms 而不是 1ms】
 *   motor_update() 内部有自己的节流：
 *       if (elapsed_ms < motor->update_period_ms) return;
 *   而 update_period_ms = MOTOR_UPDATE_PERIOD_MS = 5。
 *   这里填 5 只是告诉调度器"最早 5ms 后可能需要我"，
 *   真正的周期由 motor 自己把关。填 1 会造成 5 倍空转
 *   唤醒（虽然任务本身会立即 return，但唤醒/切换开销是真的）。
 *
 * 【为什么用 SYS_PRIO_HIGH】
 *   控制环属于实时性要求较高的一类。当前模块表里
 *   led 是 SYS_PRIO_MID(2)，motor 放 HIGH(1) 保证先于它执行。
 *
 * 【已知局限 —— 必须记录】
 *   bg_sched 线程优先级 15（最低），因此 motor_update 会被
 *   其它线程抢占，实际周期存在抖动。这在本批（R7/R8）是可接受的，
 *   因为：
 *     ① 当前目的是"把断链接上、能看到电机转"，不是精确控制
 *     ② 真正的过流保护将放在 20kHz ADC ISR 内（S1 批），
 *        不受调度优先级影响
 *   若后续速度环要求严格 5ms，需迁到独立高优先级线程。
 * ============================================================ */
static void motor_periodic_task(void)
{
    /* FOC 模式独占 TMR4_3 时，不再运行旧六步电机对象。
     * 否则 openloop_update → bldc_6step_set_output 会周期性写 OCCR/EMB，
     * 覆盖 focmode 的占空比。 */
    if (!Foc_Mode_PwmOwned()) {
        motor_update(&g_swing_motor);
    }

    /* 【2026-11-10】补打中断里挂起的急停日志。
     *
     * 为什么需要这一步：
     *   hw_brake_emergency_lock() 可被 20kHz 电流环 ISR 调用
     *   （foc_adc_dma_isr -> stall_on_current -> stall_trigger），
     *   而它在 50μs 周期里不能做 vsnprintf + device_write。
     *   中断里只置标志，这里补打。
     *
     * 为什么放在 motor_update 之后：
     *   motor_update 可能刚刚又触发了一次急停（过流路径），
     *   放在后面能保证同一周期内的挂起日志被立即刷出。 */
    hw_brake_flush_log();

    /* 【2026-11-05 / S1】堵转保护周期任务，必须放在 motor_update 之后。
     *
     * 为什么顺序不能反：
     *   motor_update() 内部会刷新 motor->target_output 与
     *   motor->ramp.is_running/mode。stall_on_tick() 用这两个值做门控
     *   （"是否在驱动" / "是否在缓停止"）。若先跑堵转再跑 motor_update，
     *   读到的是上一周期的旧值，门控会滞后 5ms。
     *
     * 为什么放在 5ms 而不是 20kHz：
     *   堵转的电流三档已在 stall_on_current()（20kHz，由 ADC 通路调用）
     *   里完成，那里才是需要高采样率的场合。
     *   本函数只做低速判据（转速源本身就是 5ms 窗口）+ 门控 + 自动清除，
     *   5ms 足够，且避免在 20kHz 里做除法和读多个模块。 */
    stall_on_tick();
}

/* ============================================================
 * 后台模块表
 * ============================================================ */
const bg_module_t g_bg_modules[] = {
      /* 【R7 修复 / 2026-11-04】电机周期更新（5ms 控制环 + 过流保护检查）
       * 位置故意放在最前：同优先级下按数组顺序执行。
       * 详见 motor_periodic_task() 上方的注释。 */
      {"motor",    NULL, motor_periodic_task, SYS_PRIO_HIGH,    5,   0, 0, 0},
      {"led",      NULL, NULL,     SYS_PRIO_MID,    100,   0, 0, 0},
//    {"led",      led_init, led_task,     SYS_PRIO_MID,    100,   0, 0, 0},
//    {"dev_info_report", NULL,     dev_info_task, SYS_PRIO_LOWEST, 10000, 0, 0, 0},

//    /* OTA 状态上报 (5000ms) */
//    {
//        .name         = "ota_report",
//        .init         = NULL,
//        .task         = ota_report_status,
//        .period_ms    = 5000,
//        .last_run_tick = 0
//    },
//    /* 设备信息上报 (10000ms) */
//    {
//        .name         = "dev_info_report",
//        .init         = NULL,
////        .task         = device_info_dump,
//        .period_ms    = 10000,
//        .last_run_tick = 0
//    },
};

/* 模块数量 */
const uint16_t g_bg_module_count = sizeof(g_bg_modules) / sizeof(g_bg_modules[0]);
