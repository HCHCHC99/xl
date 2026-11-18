/**
 * @file    motor_angle_task.c
 * @brief   电机角度/转速的周期驱动 + 诊断命令
 *
 * ============================================================
 * 【为什么用 RT-Thread 软件定时器，而不是 bg_scheduler】
 * ------------------------------------------------------------
 * bg_scheduler 的模块表当前只有 {"led", NULL, NULL} 一项，
 * task == NULL 空转，从未真正验证过调度精度。
 * 而本模块要求 1ms 稳定节拍（过圈余量直接取决于它）。
 *
 * RT-Thread 软定时器由 timer 线程（优先级 0，最高）驱动，
 * 实测抖动在数十 μs 量级，对 1ms 周期足够。
 *
 * 【为什么不用硬件定时器】
 * 1ms 精度不需要硬件定时器；软定时器省一个外设资源，
 * 且回调运行在 timer 线程上下文，可安全调用 SPI/RTT 打印。
 * （硬件 ISR 里禁止这些操作）
 * ============================================================
 */

#include <rtthread.h>
#include <finsh.h>
#include <stdlib.h>

#include "motor_angle.h"
#include "motor_speed.h"
#include "mech_cfg_store.h"
#include "bsp/bsp_pwm.h"            /* bsp_pwm_get_cycle_cnt：20kHz 硬件时基 */
#include "mode/sys_caps.h"

/* 【2026-11-10 彩标 P1】
 * 只给运行时日志上色：解算失败降级=WARN、任务启动=OK。
 * t_mangle / t_mangle_solve 属命令输出，保持单色（Q4 裁定）。 */
#include "../log/log_color.h"

/* ============================================================
 * 周期驱动
 * ============================================================ */
static struct rt_timer s_angle_timer;
static volatile uint32_t s_task_tick;      /* 回调次数，用于检测停摆 */

static void motor_angle_timer_cb(void *param)
{
    uint32_t now_tick;

    (void)param;

    s_task_tick++;

    /* 【2026-11-03 修正】时间源改用 PWM 硬件 tick，不再用 rt_tick(1ms)
     *
     * 原因：5ms 窗口下 rt_tick 的 1ms 分辨率导致 dt 只能是 5 或 6，
     *       量化误差最坏 ±20%，远大于角度量化（±0.08%）。
     *       bsp_pwm_get_cycle_cnt() 是 1 个 PWM 周期分辨率，误差降到 ±0.5%。
     *       tick 频率跟随 cfg_foc_table.h 的 FOC_CFG_PWM_FREQ。
     *
     * 注意：tick 由 PWM 时基中断维护，若该中断未工作，此值恒 0，
     *       motor_speed_update 会因 dt 永不增长而一直不出结果。
     *       诊断见 t_pwmtime。 */
    now_tick = bsp_pwm_get_cycle_cnt();

    /* 1. 角度增量（只读 Z0）*/
    if (motor_angle_update() == MOTOR_ANGLE_OK)
    {
        /* 2. 转速（内部按 tick 分频到 5ms）*/
        motor_speed_update(motor_angle_get_cnt(), now_tick);
    }
    else
    {
        /* 角度更新失败时，转速也不能用陈旧的 cnt 去算。
         * 但要把 tick 基线交出去，否则恢复后 dt 会累积成异常大值。 */
        motor_speed_rebase(motor_angle_get_cnt(), now_tick);
    }
}

/* ============================================================
 * 初始化（由 boot 表调用）
 * ============================================================ */
int motor_angle_task_init(void)
{
    motor_angle_err_t e;

    motor_angle_init();
    motor_speed_init();

    /* ============================================================
     * 【2026-11-03】上电自动解算
     * ------------------------------------------------------------
     * 与用户确认的调试期策略：
     *   解算失败时，motor_angle_solve() 内部会退化为「以当前 Z0 为 0°」
     *   的零基点模式（degenerate=1），使增量/转速仍可工作。
     *   绝对值不可信，但方向、增量、限速趋势全部可用。
     *   落地期改为方案 A（上报故障码 + 禁止运动）。
     *
     * 注意：solve 失败仍返回非 OK，这里只打印提示，不阻断启动。
     * ============================================================ */
    e = motor_angle_solve();
    if (e != MOTOR_ANGLE_OK)
    {
        LOG_WARN("[MAngle] WARN: solve failed -> ZERO-BASE degraded mode\n");
        LOG_WARN("         angle is RELATIVE (0 at boot), absolute unknown\n");
        LOG_WARN("         increments + speed still available for debugging\n");
    }

    rt_timer_init(&s_angle_timer,
                  "mangle",
                  motor_angle_timer_cb,
                  RT_NULL,
                  rt_tick_from_millisecond(MOTOR_ANGLE_PERIOD_MS),
                  RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);

    rt_timer_start(&s_angle_timer);

    LOG_OK("[MAngle] task started, period=%ums (soft timer), inited=%u\n",
               (unsigned)MOTOR_ANGLE_PERIOD_MS,
               (unsigned)((e == MOTOR_ANGLE_OK) ? 1U : 0U));
    return 0;
}

/* 单独暴露解算入口：上电后由启动流程或 MSH 触发。
 * 不放在 timer 里，因为解算要读三颗 + 游标搜索，耗时约 100μs，
 * 且必须在 mech_cfg 就绪后执行。 */
int motor_angle_task_solve(void)
{
    return (int)motor_angle_solve();
}

/* ============================================================
 * 诊断命令
 * ============================================================ */

/* t_mangle : 看虚拟角度 */
static void cmd_mangle(int argc, char **argv)
{
    motor_angle_state_t st;
    int32_t ang = 0;

    if (sys_cap_require(SYS_CAP_DIAG_READ, "t_mangle") == 0U) { return; }

    (void)argc; (void)argv;

    motor_angle_get_state(&st);

    rt_kprintf("[t_mangle] inited=%u updates=%u\n",
               (unsigned)st.inited, (unsigned)st.update_cnt);

    /* 【2026-11-02】把"定时器在跑 vs 全部失败"这两种情况区分开。
     * 以前只看 updates，两者都显示 0，无法定位。
     * still = delta==0 的次数，用于排除"读数完全不变"的假象。 */
    rt_kprintf("  calls=%u  not_init=%u  enc_err=%u  wrap_risk=%u  still=%u  last_err=%u\n",
               (unsigned)st.call_cnt, (unsigned)st.not_init_cnt,
               (unsigned)st.err_cnt, (unsigned)st.wrap_risk_cnt,
               (unsigned)st.still_cnt, (unsigned)st.last_err);
    rt_kprintf("  task tick = %u\n", (unsigned)s_task_tick);

    if (st.inited == 0U)
    {
        rt_kprintf("  NOT SOLVED yet, run 't_mangle_solve' first\n");
        if (st.call_cnt > 0U)
        {
            rt_kprintf("  note: timer IS running (%u calls) but every call early-returns\n",
                       (unsigned)st.call_cnt);
            rt_kprintf("        => angle stays 0 until solve succeeds\n");
        }
        else
        {
            rt_kprintf("  note: timer NOT running (0 calls), check boot table 'mangle' step\n");
        }
        return;
    }

    if (motor_angle_get(&ang) != MOTOR_ANGLE_OK)
    {
        rt_kprintf("  angle unavailable\n");
        return;
    }

    rt_kprintf("  angle = %d mdeg = %d.%03d deg\n",
               (int)ang,
               (int)(ang / 1000),
               (int)((ang % 1000 + 1000) % 1000));

    /* 【2026-11-03】零基点模式必须显著标记，避免被当成真实绝对角 */
    if (st.degenerate != 0U)
    {
        rt_kprintf("  >>> ZERO-BASE MODE (solve failed at boot)\n");
        rt_kprintf("      angle = RELATIVE to boot position, NOT absolute\n");
        rt_kprintf("      base Z0 raw = %d\n", (int)st.z0_raw0);
        rt_kprintf("      increments/speed valid; do NOT use for position control\n");
    }
    else
    {
        rt_kprintf("  base = ABSOLUTE (vernier solved, Z0 raw=%d)\n",
                   (int)st.z0_raw0);
    }
    rt_kprintf("  cnt_accum = %d  (1 cnt = 87.89 mdeg)\n", (int)st.cnt_accum);
    rt_kprintf("  last_raw  = %d\n", (int)st.last_raw);
    rt_kprintf("  max_delta = %u\n", (unsigned)st.max_delta);

    if (st.err_cnt > 0U)
    {
        rt_kprintf("  WARN: %u read failures, check Z0 wiring\n",
                   (unsigned)st.err_cnt);
    }
    if (st.wrap_risk_cnt > 0U)
    {
        rt_kprintf("  WARN: %u wrap-risk events, sampling period may be too long\n",
                   (unsigned)st.wrap_risk_cnt);
    }
}

/* t_mangle_solve : 上电解算 */
static void cmd_mangle_solve(int argc, char **argv)
{
    (void)argc; (void)argv;

    if (sys_cap_require(SYS_CAP_DIAG_READ, "t_mangle_solve") == 0U) { return; }

    if (motor_angle_solve() == MOTOR_ANGLE_OK)
    {
        /* 解算建立了新的计数基线，速度窗口必须重来，
         * 否则会用"解算前"的 delta 算出一个巨大的假转速。 */
        motor_speed_reset();
        LOG_OK("[t_mangle_solve] OK (speed baseline reset)\n");
    }
    else
    {
        LOG_ERR("[t_mangle_solve] FAIL, check encoders / mech_cfg\n");
    }
}

/* t_mspeed : 看转速
 * ------------------------------------------------------------
 * 【2026-11-03 单位变更】
 *   主输出改为 rpm_out_x100（0.01 RPM），因为输出轴实测仅 2.1 RPM 量级，
 *   原来的 x10（0.1 RPM）分辨率只占 4.8%，速度环会有量化死区。
 *   同时新增 omega（0.001 deg/s）与 dt_ticks（50us 粒度）便于判断精度。
 */
static void cmd_mspeed(int argc, char **argv)
{
    motor_speed_state_t st;
    int n = 1;
    int k;
    int ms = 200;

    if (sys_cap_require(SYS_CAP_DIAG_READ, "t_mspeed") == 0U) { return; }

    if (argc >= 2) { n  = atoi(argv[1]); }
    if (argc >= 3) { ms = atoi(argv[2]); }
    if (n <= 0)    { n = 1; }
    if (n > 200)   { n = 200; }
    if (ms < 0)    { ms = 0; }

    rt_kprintf("[t_mspeed] n=%d interval=%dms\n", n, ms);
    rt_kprintf("  k   rpm_out   omega     rpm_z0   delta  dt_tk  ok\n");

    for (k = 0; k < n; k++)
    {
        motor_speed_get_state(&st);

        /* 负数取余在 C 里是向零截断，所以绝对值单独处理，
         * 保证 -2.17 显示成 "-2.17" 而不是 "-2.-17" */
        rt_kprintf("  %3d  %4d.%02d  %6d.%03d  %6d.%d  %5d  %5u   %u\n",
                   k,
                   (int)(st.rpm_out_x100 / 100),
                   (int)((st.rpm_out_x100 < 0 ? -st.rpm_out_x100 : st.rpm_out_x100) % 100),
                   (int)(st.omega_out_mdeg_s / 1000),
                   (int)((st.omega_out_mdeg_s < 0 ? -st.omega_out_mdeg_s : st.omega_out_mdeg_s) % 1000),
                   (int)(st.rpm_z0_x10 / 10),
                   (int)((st.rpm_z0_x10 < 0 ? -st.rpm_z0_x10 : st.rpm_z0_x10) % 10),
                   (int)st.delta_cnt,
                   (unsigned)st.dt_ticks,
                   (unsigned)st.valid);

        if (k < n - 1) { rt_thread_mdelay(ms); }
    }

    rt_kprintf("  ---- summary ----\n");
    rt_kprintf("    updates=%u  dt_reject=%u  rebase=%u  dt_max=%u\n",
               (unsigned)st.update_cnt, (unsigned)st.reject_cnt,
               (unsigned)st.rebase_cnt, (unsigned)st.dt_max_ticks);
    rt_kprintf("    units: rpm_out=x0.01RPM  omega=x0.001deg/s\n");
    rt_kprintf("           rpm_z0=x0.1RPM    dt_tk=x%uus (1 PWM cycle)\n",
               (unsigned)bsp_pwm_cycle_to_us(1U));
    rt_kprintf("    expect dt_tk = %u (5ms @ %u Hz), tolerance +/-5\n",
               (unsigned)MOTOR_SPEED_WINDOW_TICKS,
               (unsigned)(1000000U / bsp_pwm_cycle_to_us(1U)));

    {
        const mech_rt_t *rt = mech_get_rt();
        if (rt != RT_NULL)
        {
            rt_kprintf("    i_total = %d.%02d\n",
                       (int)(rt->i_total_x100 / 100),
                       (int)(rt->i_total_x100 % 100));
        }
        else
        {
            rt_kprintf("    WARN: mech_cfg not loaded, rpm_out unavailable\n");
        }
    }

    /* 分三种情况给结论，避免"一个 0 无法判读" */
    if (st.update_cnt == 0U)
    {
        if (st.rebase_cnt > 0U)
        {
            LOG_ERR("    >>> FAIL: angle not solved (rebase=%u)\n",
                       (unsigned)st.rebase_cnt);
            rt_kprintf("        run 't_mangle_solve' first, then retry\n");
        }
        else if (st.reject_cnt > 0U)
        {
            LOG_ERR("    >>> FAIL: dt abnormal (reject=%u, dt_max=%u)\n",
                       (unsigned)st.reject_cnt, (unsigned)st.dt_max_ticks);
            rt_kprintf("        check t_pwmtime: is 20kHz timebase running?\n");
        }
        else
        {
            rt_kprintf("    >>> no sample yet, wait >= 5ms\n");
        }
    }
    else if (st.dt_max_ticks > (MOTOR_SPEED_WINDOW_TICKS * 2U))
    {
        rt_kprintf("    WARN: dt_max=%u is %ux the window (%u) -> scheduling jitter\n",
                   (unsigned)st.dt_max_ticks,
                   (unsigned)(st.dt_max_ticks / MOTOR_SPEED_WINDOW_TICKS),
                   (unsigned)MOTOR_SPEED_WINDOW_TICKS);
        rt_kprintf("          speed value will be noisy, consider 1ms HW timer\n");
    }
}

MSH_CMD_EXPORT_ALIAS(cmd_mangle,       t_mangle,       motor virtual angle);
MSH_CMD_EXPORT_ALIAS(cmd_mangle_solve, t_mangle_solve, solve motor angle from 3 enc);
MSH_CMD_EXPORT_ALIAS(cmd_mspeed,       t_mspeed,       motor speed from Z0);