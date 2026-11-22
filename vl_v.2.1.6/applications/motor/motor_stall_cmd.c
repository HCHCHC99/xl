/**
 * @file    motor_stall_cmd.c
 * @brief   堵转保护诊断命令（t_stall）
 *
 * ============================================================
 * 【为什么单独一个文件而不是塞进 app_test.c】
 * ------------------------------------------------------------
 * app_test.c 是 hw_pwm / hall / brake 的通用诊断集合，
 * 而堵转诊断需要读 sys_config、motor_speed、foc_adc 三个模块，
 * 依赖面比 app_test.c 大得多。独立成文件可以：
 *   - 只在需要时编入（将来若裁剪堵转功能，删文件即可）
 *   - 不把 app_test.c 的依赖面继续扩大
 * ============================================================
 */

#include <rtthread.h>
#include <finsh.h>
#include <stdlib.h>
#include <string.h>

#include "motor_stall.h"
#include "drv/bsp_adc_foc.h"
#include "../angle/motor_speed.h"      /* motor_speed_get */
#include "../mode/sys_caps.h"

/* 0.1A → 可读字符串（有符号，避免浮点）
 *
 * 【2026-11-09】改为显示符号：电流方向对诊断有价值
 * （电动 vs 回馈），而且能直接看出数据链路是否正常翻转。 */
static void stall_print_a10(const char *tag, int16_t a10)
{
    int32_t v = a10;
    const char *sign = "";

    if (v < 0)
    {
        sign = "-";
        v = -v;
    }
    rt_kprintf("%s=%s%d.%dA", tag, sign,
               (int)(v / 10), (int)(v % 10));
}

/* 原因位 → 字符串 */
static const char *stall_cause_str(uint8_t cause)
{
    if (cause == STALL_CAUSE_NONE)
    {
        return "none";
    }
    if (cause & STALL_CAUSE_OC3)
    {
        return "OC3(55A/10ms)";
    }
    if (cause & STALL_CAUSE_OC2)
    {
        return "OC2(50A/100ms)";
    }
    if (cause & STALL_CAUSE_OC1)
    {
        return "OC1(45A/500ms)";
    }
    if (cause & STALL_CAUSE_LOW_RPM)
    {
        return "LOW_RPM(36rpm/2.5s)";
    }
    return "?";
}

/* ============================================================
 * t_stall
 * ------------------------------------------------------------
 * 无参：打印配置 + 实时状态 + 计数器
 * t_stall clear：手动清故障
 * ============================================================ */
static void cmd_stall(int argc, char **argv)
{
    const motor_stall_ctx_t *ctx;
    motor_stall_cfg_t cfg_copy;
    uint16_t i_now;                 /* 母线电流（单向，无符号） */
    int32_t  rpm_x100 = 0;
    uint8_t  got_speed;

    if (sys_cap_require(SYS_CAP_DIAG_READ, "t_stall") == 0U) { return; }

    /* ---- 子命令：clear ---- */
    if (argc >= 2)
    {
        if (strcmp(argv[1], "clear") == 0)
        {
            if (sys_cap_require(SYS_CAP_DIAG_WRITE, "t_stall clear") == 0U)
            {
                return;
            }
            motor_stall_clear();
            rt_kprintf("[t_stall] fault CLEARED\n");
            return;
        }
        if (strcmp(argv[1], "reload") == 0)
        {
            if (sys_cap_require(SYS_CAP_PARAM_WRITE, "t_stall reload") == 0U)
            {
                return;
            }
            stall_load_cfg();
            rt_kprintf("[t_stall] cfg reloaded from sys_config\n");
            return;
        }
        rt_kprintf("[t_stall] usage: t_stall [clear|reload]\n");
        return;
    }

    ctx = motor_stall_get_ctx();

    /* ---- 当前状态 ---- */
    rt_kprintf("[t_stall] faulted=%u cause=%s triggers=%u\n",
               (unsigned)ctx->faulted,
               stall_cause_str(ctx->cause),
               (unsigned)ctx->trigger_count);

    if (ctx->faulted != 0U)
    {
        rt_kprintf("  snapshot when triggered:\n");
        stall_print_a10("    current", (int16_t)ctx->snap_current_a10);
        rt_kprintf("  tick=%u\n", (unsigned)ctx->snap_tick);
    }

    /* ---- 实时输入 ---- */
    i_now = foc_adc_get_bus_current_a10();
    got_speed = motor_speed_get(&rpm_x100);

    stall_print_a10("  i_bus", (int16_t)i_now);
    rt_kprintf("   ");
    if (got_speed != 0U)
    {
        /* 【2026-11-09 修正负号拼接 bug】
         * 原实现用 rpm_x100/100 与 rpm_x100%100 分别打印整数与小数，
         * 但 C 的整数除法是"向零截断"，负数时两部分符号不一致：
         *   rpm_x100 = -1205
         *     -1205 / 100 = -12      （整数部分）
         *     -1205 % 100 =    -5    （余数，负数！）
         *   拼出来是 "-12.-5" —— 完全错误的显示。
         *
         * 修法：先取绝对值拆成"整数/小数"两段，再单独补负号。 */
        int32_t v = rpm_x100;
        const char *sign = "";

        if (v < 0)
        {
            sign = "-";
            v = -v;
        }
        rt_kprintf("rpm=%s%d.%02d (abs=%d.%02d)\n",
                   sign, (int)(v / 100), (int)(v % 100),
                   (int)(v / 100), (int)(v % 100));
    }
    else
    {
        rt_kprintf("rpm=--- (speed not ready)\n");
    }

    /* ---- 门控 ---- */
    rt_kprintf("  gate: driving=%u current=%u not_ramping=%u  "
               "(all 3 needed for LOW_RPM)\n",
               (unsigned)ctx->gate_driving,
               (unsigned)ctx->gate_current,
               (unsigned)ctx->gate_not_ramping);

    /* ---- 电流三档计数器 ---- */
    /* 注意：这里读不到 s_cfg（static），只能打印当前计数 + 换算倒计时。
     * 为实现这一点，cfg 的副本通过 get_ctx 无法获得，
     * 因此只打印计数；如需看阈值请用 t_cfg get stall_*。 */
    rt_kprintf("  oc counters (20kHz samples):\n");
    rt_kprintf("    oc3 cnt=%5u\n", (unsigned)ctx->oc3_cnt);
    rt_kprintf("    oc2 cnt=%5u\n", (unsigned)ctx->oc2_cnt);
    rt_kprintf("    oc1 cnt=%5u\n", (unsigned)ctx->oc1_cnt);

    /* ---- 低速分段 ---- */
    rt_kprintf("  low-rpm segments:\n");
    rt_kprintf("    seg_ok=%u  (need N consecutive, see boot log)\n",
               (unsigned)ctx->low_seg_ok);
    rt_kprintf("    cur_seg: samples=%u  low_cnt=%u\n",
               (unsigned)ctx->low_seg_samples,
               (unsigned)ctx->low_seg_cnt);

    rt_kprintf("  tip: thresholds via 't_cfg get stall_oc3_a10'\n");
}

MSH_CMD_EXPORT_ALIAS(cmd_stall, t_stall, stall protection diag);