/**
 * @file    pwm_time_test.c
 * @brief   FOC 微小时基诊断命令（2026-11-02 新增）
 *
 * ============================================================
 * 【为什么单独做一个诊断命令】
 * 时基是整个 FOC 的地基。如果 20 kHz 计数不准，后面所有
 * 电角度外推、电流环 dt、转速计算全部跟着错，而且错得很隐蔽
 * （表现为"轻微抖动"或"速度有偏差"，很难定位到根因）。
 *
 * 所以必须先独立验证：周期计数的**频率**和**单调性**。
 * ============================================================
 *
 * 【命令清单】
 *   t_pwmtime           打印当前计数值 + 使能状态
 *   t_pwmtime 1000      连续采样 1000 次（每次间隔 1ms），统计
 *                       1ms 内的计数增量，期望 = PWM 频率 / 1000
 * ============================================================
 */

#include <rtthread.h>
#include <finsh.h>
#include <stdlib.h>

#include "hc32_ll.h"                /* __NOP() */
/* 【2026-11-10 彩标 P4】t_pwmtime 诊断：FAIL 上色，数据保持单色 */
#include "../log/log_color.h"
#include "bsp_pwm.h"
#include "../motor/foc/foc_pwm.h"   /* 实际 PWM 频率，跟随 cfg_foc_table.h */
#include "board_config.h"           /* BSP_TMR4_3_ISR_IRQ_NUM */
#include "mode/sys_caps.h"

/* ============================================================
 * t_pwmtime
 * ------------------------------------------------------------
 * 无参数：打印一次快照
 * 带参数：连续采样统计，验证频率
 * ============================================================ */
static void cmd_pwmtime(int argc, char **argv)
{
    uint32_t pwm_freq;

    if (sys_cap_require(SYS_CAP_DIAG_READ, "t_pwmtime") == 0U) { return; }

    pwm_freq = foc_pwm_get_frequency();
    if (pwm_freq == 0U) { pwm_freq = 20000U; }

    /* ---- 无参数：单次快照 ---- */
    if (argc < 2)
    {
        uint32_t cnt = bsp_pwm_get_cycle_cnt();
        uint32_t us  = bsp_pwm_cycle_to_us(cnt);
        uint32_t s[8];
        uint32_t c1, c2;
        int i;

        bsp_pwm_tb_snapshot(s);

        rt_kprintf("[t_pwmtime] timebase %s\n",
                   (bsp_pwm_tb_is_enabled() != 0U) ? "ENABLED" : "DISABLED");

        if (bsp_pwm_tb_is_enabled() == 0U)
        {
            rt_kprintf("  WARN: timebase not enabled, cycle_cnt stuck at 0\n");
            rt_kprintf("        check bsp_pwm_tim4_init() registration\n");
            return;
        }

        rt_kprintf("  cycle_cnt = %u\n", (unsigned)cnt);
        rt_kprintf("  elapsed   = %u us = %u.%03u s\n",
                   (unsigned)us,
                   (unsigned)(us / 1000000U),
                   (unsigned)((us / 1000U) % 1000U));
        rt_kprintf("  expect %u cycles per ms (%u Hz PWM)\n",
                   (unsigned)(pwm_freq / 1000U), (unsigned)pwm_freq);
        rt_kprintf("  wrap in %u hours\n",
                   (unsigned)((0xFFFFFFFFULL / pwm_freq) / 3600ULL));

        /* ---- 寄存器直读：一次就能区分"计数器没跑"和"中断没来" ---- */
        rt_kprintf("  ---- raw registers ----\n");
        rt_kprintf("    CNTR   = %5u   (tri-wave 0..%u, must CHANGE if running)\n",
                   (unsigned)s[1],
                   (unsigned)(foc_pwm_get_period_ticks() - 1U));
        rt_kprintf("    CCSR   = 0x%04X\n", (unsigned)s[2]);
        rt_kprintf("    STOP   = %u   (bit6, 0=running 1=stopped)\n", (unsigned)s[3]);
        rt_kprintf("    IRQZEN = %u   (bit13, valley int enable)\n", (unsigned)s[4]);
        rt_kprintf("    IRQZF  = %u   (bit14, valley flag, self-clearing)\n", (unsigned)s[5]);
        rt_kprintf("    MODE   = %u   (bit5, 1=triangle)\n", (unsigned)s[6]);
        rt_kprintf("    SOE3   = %u   (1=EMB locked, power stage safe)\n", (unsigned)s[7]);

        /* ---- 自动判定 ---- */
        rt_kprintf("  ---- verdict ----\n");

        /* 【2026-11-03 新增】中断链路逐环检查
         * 前三轮反复卡在"CNTR 在跑但 cycle_cnt 为 0"，每次都靠猜。
         * 这里把 INTC 映射 / INTC IER / NVIC ISER / NVIC ISPR / 优先级
         * 一次全部读出并逐环判定，定位到具体哪一环断。 */
        {
            uint32_t q[8];
            int      chain_ok = 1;

            bsp_pwm_tb_irq_snapshot(q);

            rt_kprintf("  ---- irq chain ----\n");
            rt_kprintf("    INTC SEL[%03d] = %u   (expect %u)\n",
                       (int)BSP_TMR4_3_ISR_IRQ_NUM, (unsigned)q[0], (unsigned)q[7]);
            rt_kprintf("    INTC IER      = 0x%08X\n", (unsigned)q[1]);
            rt_kprintf("    NVIC ISER bit = %u   (1=enabled)\n", (unsigned)q[2]);
            rt_kprintf("    NVIC ISPR bit = %u   (1=pending not taken)\n", (unsigned)q[3]);
            rt_kprintf("    NVIC IP       = 0x%02X (priority)\n", (unsigned)q[4]);

            if (q[0] != q[7])
            {
                chain_ok = 0;
                rt_kprintf("    >>> BREAK 1: INTC SEL not written (got %u, want %u)\n",
                           (unsigned)q[0], (unsigned)q[7]);
                rt_kprintf("        IrqSignIn rejected or wrong SEL address\n");
            }
            if (q[2] == 0U)
            {
                chain_ok = 0;
                rt_kprintf("    >>> BREAK 2: NVIC not enabled\n");
            }
            if (q[3] != 0U)
            {
                rt_kprintf("    NOTE: IRQ pending but not taken => masked by higher prio\n");
            }

            /* 事件计数：区分"进了中断但没归类"vs"根本没进" */
            {
                uint32_t e[3];

                bsp_pwm_tb_event_cnt(e);
                rt_kprintf("    ---- event counters ----\n");
                rt_kprintf("      valley   = %u   (expect rising at %u Hz)\n",
                           (unsigned)e[0], (unsigned)pwm_freq);
                rt_kprintf("      peak     = %u\n", (unsigned)e[1]);
                rt_kprintf("      spurious = %u   (should stay 0)\n",
                           (unsigned)e[2]);

                if ((e[0] == 0U) && (e[1] > 0U))
                {
                    rt_kprintf("      >>> ISR fires on PEAK only, valley never\n");
                    rt_kprintf("      => VALLEY int source mismatch (use UDF=359)\n");
                }
                if ((e[0] == 0U) && (e[1] == 0U) && (e[2] == 0U))
                {
                    rt_kprintf("      >>> ISR never entered at all\n");
                }
            }

            if (chain_ok && (bsp_pwm_get_cycle_cnt() == 0U))
            {
                rt_kprintf("    >>> chain configured OK but cycle_cnt==0\n");
                rt_kprintf("        TMR4 int SOURCE never asserts\n");
                rt_kprintf("        check CCSR flag/enable pairing:\n");
                rt_kprintf("          PEAK(IRQPF)   <-> INT_SRC_TMR4_3_OVF(358)\n");
                rt_kprintf("          VALLEY(IRQZF) <-> INT_SRC_TMR4_3_UDF(359)\n");
            }
        }

        if (s[3] != 0U)
        {
            LOG_ERR("    FAIL: counter STOPPED (STOP=1), TMR4 never started\n");
            rt_kprintf("          fix: bsp_pwm_tim4_init() must call TMR4_Start\n");
            return;
        }
        if (s[4] == 0U)
        {
            LOG_ERR("    FAIL: valley interrupt NOT enabled (IRQZEN=0)\n");
            return;
        }

        /* 采样 CNTR 两次：变化=计数器真在跑 */
        c1 = s[1];
        for (i = 0; i < 2000; i++) { __NOP(); }     /* 约 10us @200MHz */
        c2 = (uint32_t)bsp_pwm_tb_get_cntr();

        if (c1 == c2)
        {
            rt_kprintf("    SUSPECT: CNTR unchanged (%u == %u) after 10us\n",
                       (unsigned)c1, (unsigned)c2);
            rt_kprintf("             counter may be stopped or clock gated\n");
        }
        else if (cnt == 0U)
        {
            LOG_ERR("    FAIL: CNTR moving (%u->%u) but cycle_cnt==0\n",
                       (unsigned)c1, (unsigned)c2);
            rt_kprintf("          => ISR never fires. check INTC sign-in / vector number\n");
        }
        else
        {
            rt_kprintf("    OK: counter running (CNTR %u->%u), ISR active (cnt=%u)\n",
                       (unsigned)c1, (unsigned)c2, (unsigned)cnt);
        }
        return;
    }

    /* ---- 带参数：频率验证 ---- */
    {
        int      n = atoi(argv[1]);
        int      k;
        uint32_t prev = 0U;
        uint32_t min_d = 0xFFFFFFFFU;
        uint32_t max_d = 0U;
        uint32_t sum_d = 0U;
        uint32_t samples = 0U;
        uint32_t exp_delta = pwm_freq / 1000U;   /* 1ms 期望计数增量 */
        uint32_t lo_d, hi_d;

        if (exp_delta == 0U) { exp_delta = 1U; }
        lo_d = (exp_delta * 3U) / 4U;
        hi_d = (exp_delta * 5U) / 4U;

        if (n <= 0)    { n = 100; }
        if (n > 5000)  { n = 5000; }

        if (bsp_pwm_tb_is_enabled() == 0U)
        {
            LOG_ERR("[t_pwmtime] FAIL: timebase not enabled\n");
            return;
        }

        rt_kprintf("[t_pwmtime] sampling %d x 1ms, expect ~%u cycles each (%u Hz)\n",
                   n, (unsigned)exp_delta, (unsigned)pwm_freq);
        rt_kprintf("  k    cnt       delta\n");

        prev = bsp_pwm_get_cycle_cnt();

        for (k = 0; k < n; k++)
        {
            uint32_t now;

            rt_thread_mdelay(1);
            now = bsp_pwm_get_cycle_cnt();

            /* 无符号减法自动处理回绕 */
            {
                uint32_t d = now - prev;

                /* 【2026-11-03 修正】统计从第 1 个样本起算，跳过 k==0。
                 *
                 * 原因：k=0 的 delta 是从"发命令前"到"第一次 mdelay 后"，
                 * 中间夹着 rt_kprintf 打印与命令解析，实际间隔不是 1ms。
                 * 实测 k=0 的 delta=2（正常应为 20），
                 * 它会把 min 拉低到 2，从而让 max/min>2 误报 JITTER。
                 * 跳过它，统计才有意义。 */
                if (k > 0)
                {
                    if (d < min_d) { min_d = d; }
                    if (d > max_d) { max_d = d; }
                    sum_d += d;
                    samples++;
                }

                /* 只打印前 10 行和异常的，避免刷屏 */
                if ((k < 10) || (d < lo_d) || (d > hi_d))
                {
                    rt_kprintf("  %3d  %8u  %5u%s\n",
                               k, (unsigned)now, (unsigned)d,
                               ((k == 0) ? "  (first, excluded from stats)"
                                         : (((d < lo_d) || (d > hi_d)) ? "  <-- OUT OF RANGE" : "")));
                }
            }
            prev = now;
        }

        rt_kprintf("  ---- summary ----\n");
        rt_kprintf("    samples = %u\n", (unsigned)samples);

        if (samples > 0U)
        {
            uint32_t avg_x100 = (sum_d * 100U) / samples;
            uint32_t target_x100 = pwm_freq / 10U;   /* freq/1000 * 100 */
            uint32_t tol_x100 = target_x100 / 200U;  /* ±0.5% */

            rt_kprintf("    min = %u   max = %u   avg = %u.%02u cycles/ms\n",
                       (unsigned)min_d, (unsigned)max_d,
                       (unsigned)(avg_x100 / 100U), (unsigned)(avg_x100 % 100U));

            /* 判定：期望 pwm_freq/1000 cycles/ms */
            if ((avg_x100 + tol_x100 >= target_x100) &&
                (avg_x100 <= target_x100 + tol_x100))
            {
                rt_kprintf("    FREQ OK: %u.%02u kHz, matches PWM setting\n",
                           (unsigned)(avg_x100 / 100U),
                           (unsigned)(avg_x100 % 100U));
                rt_kprintf("    (target %u.%02u kHz, tolerance +/-0.5%%)\n",
                           (unsigned)(target_x100 / 100U),
                           (unsigned)(target_x100 % 100U));
            }
            else
            {
                rt_kprintf("    FREQ MISMATCH: expect %u.%02u cycles/ms\n",
                           (unsigned)(target_x100 / 100U),
                           (unsigned)(target_x100 % 100U));
                rt_kprintf("    check: PWM freq = %u Hz ? PCLK1 = 100MHz ?\n",
                           (unsigned)pwm_freq);
            }

            if (max_d > (min_d * 2U))
            {
                rt_kprintf("    JITTER: max/min ratio > 2, ISR may be delayed\n");
                rt_kprintf("    check ISR priority vs other high-rate interrupts\n");
            }
            else
            {
                rt_kprintf("    JITTER OK: max/min ratio <= 2\n");
            }

            /* 区分"计数器没跑"vs"中断没来"——两者都表现为 delta 恒 0 */
            if (max_d == 0U)
            {
                uint32_t a = (uint32_t)bsp_pwm_tb_get_cntr();
                int      i;
                uint32_t b;

                for (i = 0; i < 20000; i++) { __NOP(); }   /* 约 100us */
                b = (uint32_t)bsp_pwm_tb_get_cntr();

                rt_kprintf("    ---- why delta==0 ----\n");
                if (a != b)
                {
                    rt_kprintf("    CNTR moving (%u->%u) => counter OK, ISR NOT firing\n",
                               (unsigned)a, (unsigned)b);
                    rt_kprintf("    check: INTC_IrqSignIn vector, NVIC enable, ISR clears flag\n");
                }
                else
                {
                    rt_kprintf("    CNTR frozen at %u => counter NOT running\n",
                               (unsigned)a);
                    rt_kprintf("    check: CCSR.STOP bit, TMR4_Start(), FCG clock\n");
                }
            }
        }
    }
}

MSH_CMD_EXPORT_ALIAS(cmd_pwmtime, t_pwmtime, FOC pwm timebase check);