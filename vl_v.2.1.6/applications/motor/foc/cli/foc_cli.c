/**
 *******************************************************************************
 * @file  foc_cli.c
 * @brief FOC msh/调试命令（从 foc_mode.c 拆出，行为不变）
 *
 *        focmode / 短命令 / 参数表 / 占空比与寄存器 dump。
 *        控制核心在 foc/core/foc_run.c，一次性流程在 foc/proc/foc_proc.c。
 *        旧 focmode 数字只在本文件做映射，内部不再使用数字编号。
 *******************************************************************************
 */

#include "../core/foc_run.h"
#include "../proc/foc_proc.h"
#include "foc_current_loop.h"
#include "foc_speed_loop.h"
#include "foc_position_loop.h"
#include "foc_control.h"
#include "foc_protect.h"
#include "encoder_calib.h"
#include "cfg_foc_table.h"
#include "cfg_sampling.h"
#include "hw_tmr4_pwm.h"
#include "foc_eoca_probe.h"

#include <rtthread.h>
#include <finsh.h>
#include <stddef.h>

/*=============================================================================
 * 有符号定点打印（本工程 rt_kprintf 不支持 %f）
 *   原先 fstat 用 (int)x 与 (x-(int)x) 两段拼 "整数.小数"，负数会出现
 *   双负号，例如 iv=-3.-52、spd=-1931.-5（2026-11-20 实测）。
 *   这里统一按 1/10/100/1000 缩放后取绝对值拆分，负号只打一次。
 *   注意：返回静态缓冲区（4 组轮转），仅供单条 rt_kprintf 内使用。
 *=============================================================================*/
static const char *fm_fixed_str(float v, int dec)
{
    static char    s_buf[4][16];
    static uint8_t s_idx = 0U;
    char          *p = s_buf[s_idx & 3U];
    int32_t        mul;
    int32_t        s;
    int32_t        a;
    const char    *sign;

    s_idx++;

    if (dec >= 3)      { mul = 1000; }
    else if (dec == 2) { mul = 100;  }
    else if (dec == 1) { mul = 10;   }
    else               { mul = 1;    }

    s = (int32_t)(v * (float)mul + ((v >= 0.0f) ? 0.5f : -0.5f));
    a = (s < 0) ? -s : s;
    sign = (s < 0) ? "-" : "";

    switch (mul) {
    case 1000:
        rt_snprintf(p, 16, "%s%d.%03d", sign, (int)(a / 1000), (int)(a % 1000));
        break;
    case 100:
        rt_snprintf(p, 16, "%s%d.%02d", sign, (int)(a / 100), (int)(a % 100));
        break;
    case 10:
        rt_snprintf(p, 16, "%s%d.%d", sign, (int)(a / 10), (int)(a % 10));
        break;
    default:
        rt_snprintf(p, 16, "%d", (int)s);
        break;
    }

    return p;
}

/*=============================================================================
 * LOCK 模式 PWM 诊断打印
 *=============================================================================*/
static int foc_mode_pct_x100(float pct)
{
    return (int)(pct * 100.0f + ((pct >= 0.0f) ? 0.5f : -0.5f));
}

static void foc_mode_dump_pwm(float expect_pct)
{
    static const char phase_name[3] = { 'U', 'V', 'W' };
    tmr4_pwm_state_t        st;
    tmr4_adc_trigger_info_t trig;
    int e = foc_mode_pct_x100(expect_pct);
    int i;

    rt_kprintf("[FOC-MODE] cfg[%d] %s\r\n",
               (int)FOC_CFG_SELECT, FOC_CFG_NAME);

    rt_kprintf("[FOC-MODE] freq=%lu Hz, output=%s, sample=%s, "
               "deadtime=%u ns, active=%s\r\n",
               (unsigned long)FOC_CFG_PWM_FREQ,
               (FOC_CFG_OUTPUT_TYPE == TMR4_OUTPUT_COMPLEMENTARY)
                   ? "COMPLEMENTARY" : "SYNC",
               (FOC_CFG_SAMPLE_POINT == CFG_SAMPLE_UPPER)
                   ? "UPPER(peak)" : "LOWER(valley)",
               (unsigned)FOC_CFG_DEADTIME_NS,
               (FOC_CFG_ACTIVE_HIGH) ? "HIGH" : "LOW");

    rt_kprintf("[FOC-MODE] expect U/V/W = %d.%02d / %d.%02d / %d.%02d %%\r\n",
               e / 100, e % 100, e / 100, e % 100, e / 100, e % 100);
    rt_thread_mdelay(10);

    for (i = 0; i < 3; i++) {
        int c, h, l, d;

        hw_tmr4_pwm_get_state((tmr4_pwm_channel_t)i, &st);

        c = foc_mode_pct_x100(st.centerPct);
        h = foc_mode_pct_x100(st.highPct);
        l = foc_mode_pct_x100(st.lowPct);
        d = foc_mode_pct_x100(st.deadPct);

        if (h < 0) { h = 0; }
        if (l < 0) { l = 0; }
        if (d < 0) { d = 0; }

        rt_kprintf("[FOC-MODE] actual  %c: center=%d.%02d high=%d.%02d low=%d.%02d"
                   " dead=%d.%02d %% (cmpH=%u cmpL=%u)\r\n",
                   phase_name[i],
                   c / 100, c % 100, h / 100, h % 100,
                   l / 100, l % 100, d / 100, d % 100,
                   (unsigned)hw_tmr4_pwm_get_occr((tmr4_pwm_channel_t)i, true),
                   (unsigned)hw_tmr4_pwm_get_occr((tmr4_pwm_channel_t)i, false));
    }

    for (i = 0; i < 3; i++) {
        tmr4_pwm_regs_t regs;

        hw_tmr4_pwm_get_regs((tmr4_pwm_channel_t)i, &regs);

        if (FOC_CFG_OUTPUT_TYPE == TMR4_OUTPUT_COMPLEMENTARY) {
            rt_kprintf("[FOC-MODE] REG %c: OCCRH=%u OCCRL=%u PDAR=%u PDBR=%u "
                       "OCER=0x%04X OCSR=0x%04X\r\n",
                       phase_name[i],
                       (unsigned)regs.occrH, (unsigned)regs.occrL,
                       (unsigned)regs.pdar,  (unsigned)regs.pdbr,
                       (unsigned)regs.ocer,  (unsigned)regs.ocsr);
        } else {
            rt_kprintf("[FOC-MODE] REG %c: OCCRH=%u OCCRL=%u "
                       "(SYNC: PDAR/PDBR reg=%u/%u) OCER=0x%04X OCSR=0x%04X\r\n",
                       phase_name[i],
                       (unsigned)regs.occrH, (unsigned)regs.occrL,
                       (unsigned)regs.pdar,  (unsigned)regs.pdbr,
                       (unsigned)regs.ocer,  (unsigned)regs.ocsr);
        }
    }

    if (FOC_CFG_OUTPUT_TYPE == TMR4_OUTPUT_COMPLEMENTARY) {
        rt_kprintf("[FOC-MODE] COMP: LS ON around valley (CNT 0..cmpL), "
                   "HS ON around peak (CNT cmpL..%u)\r\n",
                   (unsigned)(hw_tmr4_pwm_get_period_ticks() - 1U));
        rt_kprintf("[FOC-MODE] COMP: both up/down slopes, "
                   "each edge loses %u dead ticks\r\n",
                   (unsigned)hw_tmr4_pwm_get_dead_ticks());
    } else {
        rt_kprintf("[FOC-MODE] SYNC timing: HS/LS ON together around peak "
                   "(CNT cmp..%u, both slopes), dead offset=%u ticks\r\n",
                   (unsigned)(hw_tmr4_pwm_get_period_ticks() - 1U),
                   (unsigned)hw_tmr4_pwm_get_dead_ticks());
    }
    rt_thread_mdelay(10);

    hw_tmr4_pwm_get_adc_trigger_info(&trig);
    {
        const char *match = "?";
        uint32_t us_x100 = 0U;

        if (trig.timerHz != 0U) {
            us_x100 = (uint32_t)(((uint64_t)trig.compareValue * 100000000ULL) / trig.timerHz);
        }
        if ((trig.matchCond & TMR4_EVT_MATCH_CNT_VALLEY) != 0U) {
            match = "VALLEY";
        } else if ((trig.matchCond & TMR4_EVT_MATCH_CNT_PEAK) != 0U) {
            match = "PEAK";
        }

        rt_kprintf("[FOC-MODE] ADC trig: %s SCCR=%u (%u.%02u us after valley), "
                   "match=%s EVT0/SCM0, period=%u, CNT_now=%u\r\n",
                   trig.highSide ? "UPPER" : "LOWER",
                   (unsigned)trig.compareValue,
                   (unsigned)(us_x100 / 100U), (unsigned)(us_x100 % 100U),
                   match,
                   (unsigned)trig.periodTicks,
                   (unsigned)trig.cntNow);

        if (trig.timerHz != 0U) {
            uint16_t eoca_cnt = foc_eoca_probe_get_cnt();
            uint32_t delta_ticks;

            if (trig.highSide) {
                delta_ticks = (eoca_cnt <= trig.compareValue)
                            ? (uint32_t)(trig.compareValue - eoca_cnt) : 0U;
            } else {
                delta_ticks = (uint32_t)eoca_cnt;
            }

            {
                uint32_t eoca_us_x100 =
                    (uint32_t)(((uint64_t)delta_ticks * 100000000ULL) / trig.timerHz);

                rt_kprintf("[FOC-MODE] EOCA: CNT=%u, %u.%02u us after trigger\r\n",
                           (unsigned)eoca_cnt,
                           (unsigned)(eoca_us_x100 / 100U),
                           (unsigned)(eoca_us_x100 % 100U));
            }
        }
    }
    rt_thread_mdelay(10);
    rt_kprintf("[FOC-MODE] dump end\r\n");
}

/*=============================================================================
 * msh 命令辅助
 *=============================================================================*/
static int foc_mode_parse_float(const char *s, float *out)
{
    float v    = 0.0f;
    float frac = 0.1f;
    int   sign = 1;
    int   any  = 0;

    if ((s == RT_NULL) || (out == RT_NULL)) return -1;

    if (*s == '-') {
        sign = -1;
        s++;
    } else if (*s == '+') {
        s++;
    }

    while ((*s >= '0') && (*s <= '9')) {
        v = v * 10.0f + (float)(*s - '0');
        s++;
        any = 1;
    }

    if (*s == '.') {
        s++;
        while ((*s >= '0') && (*s <= '9')) {
            v += (float)(*s - '0') * frac;
            frac *= 0.1f;
            s++;
            any = 1;
        }
    }

    if ((*s != '\0') || (any == 0)) return -1;

    *out = v * (float)sign;
    return 0;
}

static void foc_mode_print_status(void)
{
    int d10 = (int)(Foc_Run_GetLockDuty() * 10.0f + 0.5f);
    foc_proc_t p = Foc_Proc_GetActive();

    /* 数字一律用兼容映射，保证与旧版日志/fstat 的 0/1/2/4/9/10 一致 */
    rt_kprintf("[FOC-MODE] current=%d (%s), lock duty=%d.%d%%, aligned=%u\r\n",
               Foc_Run_LegacyId(Foc_Run_GetMode()),
               Foc_Run_Name(Foc_Run_GetMode()),
               d10 / 10, d10 % 10,
               (unsigned)g_motor_is_aligned);

    /* 一次性流程不再占用运行模式，单独一行显示 */
    if (p != FOC_PROC_NONE) {
        rt_kprintf("[FOC-PROC] active=%d (%s)\r\n",
                   Foc_Proc_LegacyId(p), Foc_Proc_Name(p));
    }
}

/* 实时打印三相中心占空比：用于确认 IF/电流/速度/位置模式下 PWM 是否在刷新 */
static void foc_mode_print_duties(void)
{
    static const char nm[3] = { 'U', 'V', 'W' };
    tmr4_pwm_state_t st;
    int i;

    for (i = 0; i < 3; i++) {
        int c;

        hw_tmr4_pwm_get_state((tmr4_pwm_channel_t)i, &st);
        c = foc_mode_pct_x100(st.centerPct);
        rt_kprintf("%c=%d.%02d%%%s", nm[i], c / 100, c % 100,
                   (i < 2) ? "  " : "\r\n");
    }
}

/*=============================================================================
 * 运行中可调参数
 *   进入 mode 4/9/10 时打印参数表；
 *   之后发 "rpm 500"（或 "focmode set rpm 500"）可实时修改其中一个参数
 *=============================================================================*/
static void fm_print_num(int v, int dec)
{
    if (v < 0) {
        rt_kprintf("-");
        v = -v;
    }
    if (dec == 0) {
        rt_kprintf("%d", v);
    } else if (dec == 1) {
        rt_kprintf("%d.%d", v / 10, v % 10);
    } else if (dec == 2) {
        rt_kprintf("%d.%02d", v / 100, v % 100);
    } else {
        rt_kprintf("%d.%03d", v / 1000, v % 1000);
    }
}

static int fm_scale(float v, int dec)
{
    float sc = (dec == 0) ? 1.0f
             : (dec == 1) ? 10.0f
             : (dec == 2) ? 100.0f : 1000.0f;

    return (int)(v * sc + ((v >= 0.0f) ? 0.5f : -0.5f));
}

static void fm_param_line(const char *name, const char *desc, const char *unit,
                          float def, float now, int dec)
{
    int sc = fm_scale(def, dec);
    int sn = fm_scale(now, dec);

    rt_kprintf("  %-6s %-24s [%s] def=", name, desc, unit);
    fm_print_num(sc, dec);
    rt_kprintf(" now=");
    fm_print_num(sn, dec);
    rt_kprintf("\r\n");
}

static int fm_ack(const char *name, float v, int dec)
{
    rt_kprintf("[FOC-MODE] set %s -> ", name);
    fm_print_num(fm_scale(v, dec), dec);
    rt_kprintf(" (mode %d)\r\n", Foc_Run_LegacyId(Foc_Run_GetMode()));
    return 0;
}

static void foc_mode_print_params(foc_run_mode_t m)
{
    if (m == FOC_RUN_CURRENT) {
        rt_kprintf("[FOC-MODE] mode 4 CURRENT adjustable params:\r\n");
        fm_param_line("iq",    "target q-axis current",  "A",  0.50f, Foc_Run_GetCurrentCmdIq(), 3);
        fm_param_line("id",    "target d-axis current",  "A",  0.00f, Foc_Run_GetCurrentCmdId(), 3);
        fm_param_line("imax",  "current ref limit",      "A",  CL_IREF_MAX_INIT, g_cl_iref_max, 2);
        fm_param_line("vmax",  "output voltage limit",   "V",  CL_VMAX_INIT, g_cl_vmax_watch, 2);
        rt_kprintf("[FOC-MODE] e.g. iq -1.5          ('-' = reverse torque)\r\n");
        rt_kprintf("[FOC-MODE] e.g. current 0 1.5      (enter mode 4 with id/iq)\r\n");
    } else if (m == FOC_RUN_SPEED) {
        rt_kprintf("[FOC-MODE] mode 9 SPEED adjustable params:\r\n");
        fm_param_line("rpm",   "target speed",           "rpm",   0.0f, g_spd_target_rpm, 1);
        fm_param_line("accel", "speed slew limit",       "rpm/s", SPD_ACCEL_RPM_S_INIT, g_spd_accel_rpm_s, 0);
        fm_param_line("iqlim", "speed loop iq limit",    "mA",    SPD_IQ_LIMIT_MA_INIT, g_spd_iq_limit_ma, 0);
        fm_param_line("kp",    "speed PI kp",            "-",     SPD_KP_INIT, g_spd_kp, 2);
        fm_param_line("ki",    "speed PI ki",            "-",     SPD_KI_INIT, g_spd_ki, 2);
        fm_param_line("kd",    "speed PI kd",            "-",     SPD_KD_INIT, g_spd_kd, 2);
        fm_param_line("iterm", "speed integral limit",   "mA",    SPD_ITERM_MAX_MA_INIT, g_spd_iterm_max_ma, 0);
        fm_param_line("ff",    "speed feedforward",      "mA",    SPD_FF_MA_INIT, g_spd_ff_ma, 0);
        rt_kprintf("[FOC-MODE] e.g. rpm -500          ('-' = reverse)\r\n");
        rt_kprintf("[FOC-MODE] e.g. iqlim 1500        (0=no torque limit change)\r\n");
        rt_kprintf("[FOC-MODE] e.g. speed -500        (enter mode 9 with target)\r\n");
    } else if (m == FOC_RUN_POSITION) {
        rt_kprintf("[FOC-MODE] mode 10 POSITION adjustable params:\r\n");
        fm_param_line("rev",   "target position",        "rev",   0.0f, g_pos_target_rev, 2);
        fm_param_line("vmax",  "speed cmd limit",        "rpm",   POS_V_MAX_RPM_INIT, g_pos_v_max_rpm, 0);
        fm_param_line("iqlim", "speed loop iq limit",    "mA",    POS_IQ_LIMIT_MA_INIT, g_pos_iq_limit_ma, 0);
        fm_param_line("dec",   "planned deceleration",   "rpm/s", POS_DEC_RPM_S_INIT, g_pos_dec_rpm_s, 0);
        fm_param_line("db",    "in-position window",     "rev",   POS_DEADBAND_REV_INIT, g_pos_deadband_rev, 2);
        fm_param_line("kd",    "speed damping",          "rpm/rpm", POS_KD_RPM_PER_RPM_INIT, g_pos_kd_rpm_per_rpm, 2);
        fm_param_line("limit", "soft travel limit",      "rev",   POS_LIMIT_REV_INIT, g_pos_limit_rev, 0);
        rt_kprintf("  %-6s %-24s [-]\r\n", "zero", "set current position as 0");
        rt_kprintf("[FOC-MODE] e.g. rev 2.5            ('-' = reverse)\r\n");
        rt_kprintf("[FOC-MODE] e.g. zero                (set current position as 0)\r\n");
        rt_kprintf("[FOC-MODE] e.g. position 2.5        (enter mode 10 with target)\r\n");
    } else {
        rt_kprintf("[FOC-MODE] mode %d has no live params (use 4/9/10)\r\n",
                   Foc_Run_LegacyId(m));
    }
}

static int foc_mode_set_param(const char *name, float val)
{
    if (Foc_Run_GetMode() == FOC_RUN_CURRENT) {
        if (rt_strcmp(name, "iq") == 0) {
            Foc_Run_SetCurrentCmd(Foc_Run_GetCurrentCmdId(), val);
            return fm_ack("iq", Foc_Run_GetCurrentCmdIq(), 3);
        }
        if (rt_strcmp(name, "id") == 0) {
            Foc_Run_SetCurrentCmd(val, Foc_Run_GetCurrentCmdIq());
            return fm_ack("id", Foc_Run_GetCurrentCmdId(), 3);
        }
        if (rt_strcmp(name, "imax") == 0) {
            if (val < 0.0f) val = 0.0f;
            g_cl_iref_max = val;
            Foc_Run_ReapplyCurrentCmd();
            return fm_ack("imax", g_cl_iref_max, 2);
        }
        if (rt_strcmp(name, "vmax") == 0) {
            if (val < 1.0f) {
                rt_kprintf("[FOC-MODE] vmax must be >= 1.0 V\r\n");
                return -1;
            }
            Foc_CurrentLoop_SetVmax(val);
            g_cl_vmax_watch = val;
            return fm_ack("vmax", val, 2);
        }
    } else if (Foc_Run_GetMode() == FOC_RUN_SPEED) {
        if (rt_strcmp(name, "rpm") == 0) {
            g_spd_target_rpm = val;
            return fm_ack("rpm", g_spd_target_rpm, 1);
        }
        if (rt_strcmp(name, "accel") == 0) {
            if (val < 0.0f) val = 0.0f;
            g_spd_accel_rpm_s = val;
            return fm_ack("accel", g_spd_accel_rpm_s, 0);
        }
        if (rt_strcmp(name, "iqlim") == 0) {
            if (val < 1.0f) val = 1.0f;
            g_spd_iq_limit_ma = val;
            return fm_ack("iqlim", g_spd_iq_limit_ma, 0);
        }
        if (rt_strcmp(name, "kp") == 0) {
            g_spd_kp = val;
            return fm_ack("kp", g_spd_kp, 2);
        }
        if (rt_strcmp(name, "ki") == 0) {
            g_spd_ki = val;
            return fm_ack("ki", g_spd_ki, 2);
        }
        if (rt_strcmp(name, "kd") == 0) {
            g_spd_kd = val;
            return fm_ack("kd", g_spd_kd, 2);
        }
        if (rt_strcmp(name, "iterm") == 0) {
            if (val < 0.0f) val = 0.0f;
            g_spd_iterm_max_ma = val;
            return fm_ack("iterm", g_spd_iterm_max_ma, 0);
        }
        if (rt_strcmp(name, "ff") == 0) {
            if (val < 0.0f) val = 0.0f;
            g_spd_ff_ma = val;
            return fm_ack("ff", g_spd_ff_ma, 0);
        }
    } else if (Foc_Run_GetMode() == FOC_RUN_POSITION) {
        if (rt_strcmp(name, "rev") == 0) {
            g_pos_target_rev = val;
            return fm_ack("rev", g_pos_target_rev, 2);
        }
        if (rt_strcmp(name, "vmax") == 0) {
            if (val < 0.0f) val = 0.0f;
            g_pos_v_max_rpm = val;
            return fm_ack("vmax", g_pos_v_max_rpm, 0);
        }
        if (rt_strcmp(name, "iqlim") == 0) {
            if (val < 1.0f) val = 1.0f;
            g_pos_iq_limit_ma = val;
            return fm_ack("iqlim", g_pos_iq_limit_ma, 0);
        }
        if (rt_strcmp(name, "dec") == 0) {
            if (val < 1.0f) val = 1.0f;
            g_pos_dec_rpm_s = val;
            return fm_ack("dec", g_pos_dec_rpm_s, 0);
        }
        if (rt_strcmp(name, "db") == 0) {
            if (val < 0.0f) val = 0.0f;
            g_pos_deadband_rev = val;
            return fm_ack("db", g_pos_deadband_rev, 2);
        }
        if (rt_strcmp(name, "kd") == 0) {
            if (val < 0.0f) val = 0.0f;
            g_pos_kd_rpm_per_rpm = val;
            return fm_ack("kd", g_pos_kd_rpm_per_rpm, 2);
        }
        if (rt_strcmp(name, "limit") == 0) {
            if (val < 0.0f) val = 0.0f;
            g_pos_limit_rev = val;
            return fm_ack("limit", g_pos_limit_rev, 0);
        }
        if (rt_strcmp(name, "zero") == 0) {
            g_pos_zero_req = 1u;
            rt_kprintf("[FOC-MODE] zero requested, current position becomes 0\r\n");
            return 0;
        }
    } else {
        rt_kprintf("[FOC-MODE] no live params in mode %d\r\n",
                   Foc_Run_LegacyId(Foc_Run_GetMode()));
        return -1;
    }

    rt_kprintf("[FOC-MODE] unknown param '%s' in mode %d\r\n", name,
               Foc_Run_LegacyId(Foc_Run_GetMode()));
    foc_mode_print_params(Foc_Run_GetMode());
    return -1;
}

/*=============================================================================
 * 简化命令：直接输入“参数名 + 数值”，按当前模式自动分发
 *   例如：rev 2.5 / rpm -500 / iq 1.2 / zero
 * 参数名与 focmode set <name> <value> 完全一致。
 *=============================================================================*/
static int foc_param_cmd(const char *name, int argc, char **argv)
{
    float v = 0.0f;

    if (argc < 2) {
        rt_kprintf("[FOC-MODE] usage: %s <value>\r\n", name);
        return -1;
    }
    if (foc_mode_parse_float(argv[1], &v) != 0) {
        rt_kprintf("[FOC-MODE] %s: invalid value '%s'\r\n", name, argv[1]);
        return -1;
    }
    return foc_mode_set_param(name, v);
}

#define FOC_PARAM_CMD(pname)                                          \
static void cmd_foc_##pname(int argc, char **argv)                    \
{                                                                     \
    (void)foc_param_cmd(#pname, argc, argv);                          \
}                                                                     \
MSH_CMD_EXPORT_ALIAS(cmd_foc_##pname, pname, set FOC param)

/* mode 4 / 9 / 10 的可调参数快捷命令 */
FOC_PARAM_CMD(iq);
FOC_PARAM_CMD(id);
FOC_PARAM_CMD(imax);
FOC_PARAM_CMD(vmax);
FOC_PARAM_CMD(rpm);
FOC_PARAM_CMD(accel);
FOC_PARAM_CMD(iqlim);
FOC_PARAM_CMD(kp);
FOC_PARAM_CMD(ki);
FOC_PARAM_CMD(kd);
FOC_PARAM_CMD(iterm);
FOC_PARAM_CMD(ff);
FOC_PARAM_CMD(rev);
FOC_PARAM_CMD(dec);
FOC_PARAM_CMD(db);
FOC_PARAM_CMD(limit);

/* zero 不需要数值：把当前位置设为 0 */
static void cmd_foc_zero(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    (void)foc_mode_set_param("zero", 1.0f);
}
MSH_CMD_EXPORT_ALIAS(cmd_foc_zero, zero, set current position as 0);

/*=============================================================================
 * msh 命令
 *   focmode            打印当前模式
 *   focmode 0          IDLE
 *   focmode 1 [duty]   LOCK（默认 2%）
 *   focmode 2 [频率Hz] [电压V]  IF 开环拖动
 *   focmode 3          编码器电角度对齐（手动，必须先做）
 *   focmode 4 [id] [iq]      电流闭环（A）
 *   focmode 5          扫角测量
 *   focmode 6          一圈计数测量
 *   focmode 7          斜坡对齐
 *   focmode 9 [rpm]    速度闭环
 *   focmode 10 [rev]   位置闭环
 *   focmode 12         无 PWM 电流零偏校准
 *   focmode 13         PWM 50% 零矢量电流零偏校准
 *   focmode stat       关键观测量
 *   focmode set <name> <value>   运行中修改当前模式的参数
 *   短命令：idle/lock/ifrun/current/speed/position/falign/zcal/zcal50/fstat
 *   参数短命令：<name> <value>，如 rpm 500
 *=============================================================================*/
static int foc_mode_msh(int argc, char **argv)
{
    int   m;
    float v1 = 0.0f, v2 = 0.0f;

    if (argc == 1) {
        foc_mode_print_status();
        rt_kprintf("usage: focmode <0..13> [args]\r\n");
        rt_kprintf("  0=IDLE 1=LOCK[duty%%] 2=IF[f,v] 3=CALIB 4=CURRENT[id,iq]\r\n");
        rt_kprintf("  5=SWEEP 6=REV 7=ALIGN7 9=SPEED[rpm] 10=POSITION[rev]\r\n");
        rt_kprintf("  12=OFFCAL_NO_PWM 13=OFFCAL_PWM50\r\n");
        rt_kprintf("  focmode stat | focmode set <name> <value>\r\n");
        rt_kprintf("short: idle lock ifrun current speed position falign zcal zcal50 fstat\r\n");
        rt_kprintf("params: iq id imax vmax rpm accel iqlim kp ki kd iterm ff rev dec db limit zero\r\n");
        return 0;
    }

    if ((argv[1][0] == 's') && (argv[1][1] == 't') && (argv[1][2] == 'a') &&
        (argv[1][3] == 't') && (argv[1][4] == '\0')) {
        foc_mode_print_status();
        rt_kprintf("[FC] isr=%lu dt_us=%lu/%lu, seq=%lu\r\n",
                   (unsigned long)g_fc_isr_cnt,
                   (unsigned long)g_fc_isr_dt_min_us,
                   (unsigned long)g_fc_isr_dt_max_us,
                   (unsigned long)g_fc_sample_seq);
        rt_kprintf("[FC] iu=%s iv=%s iw=%s vbus=%s V\r\n",
                   fm_fixed_str(g_fc_iu, 2), fm_fixed_str(g_fc_iv, 2),
                   fm_fixed_str(g_fc_iw, 2), fm_fixed_str(g_fc_vbus, 1));
        rt_kprintf("[FC] id=%s iq=%s elec=%s deg\r\n",
                   fm_fixed_str(g_fc_id, 2), fm_fixed_str(g_fc_iq, 2),
                   fm_fixed_str(g_fc_enc_elec_deg, 1));
        rt_kprintf("[FC] fp_state=%d fault=0x%08lX fast=%s slow=%s\r\n",
                   (int)g_fp_state, (unsigned long)g_fp_fault,
                   fm_fixed_str(g_fp_fast_max, 2), fm_fixed_str(g_fp_slow_avg, 2));
        rt_kprintf("[FC] spd=%s rpm tgt=%s, iqref=%d mA, pos=%s rev\r\n",
                   fm_fixed_str(g_spd_filt_rpm, 1), fm_fixed_str(g_spd_target_rpm, 1),
                   (int)g_spd_out_ma, fm_fixed_str(g_pos_fb_rev, 3));
        if (g_fm_mode == FOC_RUN_IF) {
            rt_kprintf("[FC] mode=%d theta=%s deg, IF f=%s Hz vq=%s V, align_active=%u\r\n",
                       Foc_Run_LegacyId(g_fm_mode), fm_fixed_str(g_fm_theta_e * 57.29578f, 1),
                       fm_fixed_str(g_fm_freq_hz, 1), fm_fixed_str(g_fm_vq, 1),
                       (unsigned)g_align_active);
        } else {
            /* 非 IF 模式下 f/vq 是上次 IF 残留，打印会误导，直接省略 */
            rt_kprintf("[FC] mode=%d theta=%s deg, align_active=%u\r\n",
                       Foc_Run_LegacyId(g_fm_mode),
                       fm_fixed_str(g_fm_theta_e * 57.29578f, 1),
                       (unsigned)g_align_active);
        }
        rt_kprintf("[FC] duty ");
        foc_mode_print_duties();
        return 0;
    }

    if (rt_strcmp(argv[1], "set") == 0) {
        if (argc < 3) {
            foc_mode_print_params(Foc_Run_GetMode());
            return 0;
        }
        if (argc < 4) {
            rt_kprintf("usage: focmode set <name> <value>\r\n");
            rt_kprintf("   or short: <name> <value>  e.g. rpm 500\r\n");
            foc_mode_print_params(Foc_Run_GetMode());
            return -1;
        }
        if (foc_mode_parse_float(argv[3], &v1) != 0) {
            rt_kprintf("invalid value: %s\r\n", argv[3]);
            return -1;
        }
        return foc_mode_set_param(argv[2], v1);
    }

    {
        const char *p = argv[1];

        m = 0;
        while ((*p >= '0') && (*p <= '9')) {
            m = m * 10 + (int)(*p - '0');
            p++;
        }
        if ((*p != '\0') || (p == argv[1])) {
            rt_kprintf("invalid mode: %s\r\n", argv[1]);
            return -1;
        }
    }

    switch (m) {
    case 1:
        if (argc >= 3) {
            if (foc_mode_parse_float(argv[2], &v1) != 0) {
                rt_kprintf("invalid duty: %s\r\n", argv[2]);
                return -1;
            }
            Foc_Run_SetLockDuty(v1);
        }
        Foc_Run_SetMode(FOC_RUN_LOCK);
        if (Foc_Run_GetMode() == FOC_RUN_LOCK) {
            foc_mode_dump_pwm(Foc_Run_GetLockDuty());
        }
        return 0;

    case 2:
        v1 = 1.0f;    /* 频率 Hz */
        v2 = 1.5f;    /* 电压 V（q 轴） */
        if (argc >= 3) { if (foc_mode_parse_float(argv[2], &v1) != 0) return -1; }
        if (argc >= 4) { if (foc_mode_parse_float(argv[3], &v2) != 0) return -1; }
        Foc_Run_SetOpenloop(v1, v2, 0.0f);   /* IF 只给频率和电压，vd 固定 0 */
        Foc_Run_StartOpenloop();             /* 参考工程 Enter_IF_Mode() 的同一入口 */

        /* 欠压等门禁可能拒绝进入：只有真的进了 IF 才打印 "IF running" */
        if (Foc_Run_GetMode() != FOC_RUN_IF) {
            Foc_Run_SetOpenloop(0.0f, 0.0f, 0.0f);   /* 清掉残留频率，避免 fstat 显示 IF f=1.0 */
            rt_kprintf("[FOC-MODE] IF not entered (mode stays %d)\r\n",
                       Foc_Run_LegacyId(Foc_Run_GetMode()));
            return 0;
        }
        {
            float swing = (v2 >= 0.0f) ? v2 : -v2;

            swing = swing / (CFG_SAMPLING_SIM_VBUS_V * 0.57735027f) * 100.0f;
            rt_kprintf("[FOC-MODE] IF running: f=%d.%d Hz vq=%d.%d V, "
                       "duty swing +/-%d%% around 50%%, use 'fstat' to watch\r\n",
                       (int)v1, (int)((v1 - (float)(int)v1) * 10.0f),
                       (int)v2, (int)((v2 - (float)(int)v2) * 10.0f),
                       (int)(swing + 0.5f));

            if (g_fc_isr_cnt == 0U) {
                rt_kprintf("[FOC-MODE] WARN: 20kHz control ISR not running "
                           "(check foc_adc/foc_ctrl boot log)\r\n");
            }
        }
        return 0;

    case 4:
        v1 = Foc_Run_GetCurrentCmdId();
        v2 = Foc_Run_GetCurrentCmdIq();
        if (argc >= 3) { if (foc_mode_parse_float(argv[2], &v1) != 0) return -1; }
        if (argc >= 4) { if (foc_mode_parse_float(argv[3], &v2) != 0) return -1; }
        if (Foc_Run_GetMode() != FOC_RUN_CURRENT) {
            Foc_Run_SetMode(FOC_RUN_CURRENT);
        }
        if (Foc_Run_GetMode() != FOC_RUN_CURRENT) {
            rt_kprintf("[FOC-MODE] CURRENT not entered (mode stays %d)\r\n",
                       Foc_Run_LegacyId(Foc_Run_GetMode()));
            return 0;
        }
        Foc_Run_SetCurrentCmd(v1, v2);
        foc_mode_print_params(FOC_RUN_CURRENT);
        return 0;

    case 9:
        v1 = g_spd_target_rpm;   /* 不带参数时保持当前目标 */
        if (argc >= 3) { if (foc_mode_parse_float(argv[2], &v1) != 0) return -1; }
        if (Foc_Run_GetMode() != FOC_RUN_SPEED) {
            Foc_Run_SetMode(FOC_RUN_SPEED);
        }
        if (Foc_Run_GetMode() != FOC_RUN_SPEED) {
            rt_kprintf("[FOC-MODE] SPEED not entered (mode stays %d)\r\n",
                       Foc_Run_LegacyId(Foc_Run_GetMode()));
            return 0;
        }
        g_spd_target_rpm = v1;
        foc_mode_print_params(FOC_RUN_SPEED);
        return 0;

    case 10:
        v1 = (Foc_Run_GetMode() == FOC_RUN_POSITION) ? g_pos_target_rev : 0.0f;
        if (argc >= 3) { if (foc_mode_parse_float(argv[2], &v1) != 0) return -1; }
        if (Foc_Run_GetMode() != FOC_RUN_POSITION) {
            Foc_Run_SetMode(FOC_RUN_POSITION);
        }
        if (Foc_Run_GetMode() != FOC_RUN_POSITION) {
            rt_kprintf("[FOC-MODE] POSITION not entered (mode stays %d)\r\n",
                       Foc_Run_LegacyId(Foc_Run_GetMode()));
            return 0;
        }
        g_pos_target_rev = v1;
        foc_mode_print_params(FOC_RUN_POSITION);
        return 0;

    case 0:
        Foc_Run_SetMode(FOC_RUN_IDLE);
        return 0;

    case 3:
        Foc_Proc_Start(FOC_PROC_ENC_ALIGN);
        return 0;

    case 5:
        Foc_Proc_Start(FOC_PROC_SWEEP);
        return 0;

    case 6:
        Foc_Proc_Start(FOC_PROC_REV);
        return 0;

    case 7:
        Foc_Proc_Start(FOC_PROC_ALIGN7);
        return 0;

    case 12:
        Foc_Proc_Start(FOC_PROC_OFFSET_NOPWM);
        return 0;

    case 13:
        Foc_Proc_Start(FOC_PROC_OFFSET_PWM50);
        return 0;

    default:
        rt_kprintf("unsupported mode %d (11/14 not ported)\r\n", m);
        return -1;
    }
}

MSH_CMD_EXPORT_ALIAS(foc_mode_msh, focmode, FOC mode/status/config);

/*=============================================================================
 * 顶层短命令：模式入口
 * ------------------------------------------------------------
 * 只做参数重排，最终仍走 foc_mode_msh()，不复制第二套解析逻辑。
 * 原有 focmode 命令完整保留，两者等价。
 *   旧写法                          新写法
 *   focmode 0                       idle
 *   focmode 1 [duty]                lock [duty]
 *   focmode 2 [freq] [volt]         ifrun [freq] [volt]
 *   focmode 4 [id] [iq]             current [id] [iq]
 *   focmode 9 [rpm]                 speed [rpm]
 *   focmode 10 [rev]                position [rev]
 *   focmode 3                       falign
 *   focmode 12                      zcal
 *   focmode 13                      zcal50
 *   focmode stat                    fstat
 * 参数短命令（iq/id/imax/vmax/rpm/...）已在上面 FOC_PARAM_CMD 单独导出。
 *=============================================================================*/
static void cmd_foc_idle(int argc, char **argv)
{
    char *a[2] = { "focmode", "0" };

    (void)argc;
    (void)argv;
    (void)foc_mode_msh(2, a);
}
MSH_CMD_EXPORT_ALIAS(cmd_foc_idle, idle, FOC idle (focmode 0));

static void cmd_foc_lock(int argc, char **argv)
{
    char *a[3] = { "focmode", "1", RT_NULL };
    int   n = 2;

    if (argc >= 2) {
        a[2] = argv[1];
        n = 3;
    }
    (void)foc_mode_msh(n, a);
}
MSH_CMD_EXPORT_ALIAS(cmd_foc_lock, lock, FOC lock [duty%] (focmode 1));

static void cmd_foc_ifrun(int argc, char **argv)
{
    char *a[4] = { "focmode", "2", RT_NULL, RT_NULL };
    int   n = 2;

    if (argc >= 2) { a[2] = argv[1]; n = 3; }
    if (argc >= 3) { a[3] = argv[2]; n = 4; }
    (void)foc_mode_msh(n, a);
}
MSH_CMD_EXPORT_ALIAS(cmd_foc_ifrun, ifrun, FOC openloop [freq] [volt] (focmode 2));

static void cmd_foc_current(int argc, char **argv)
{
    char *a[4] = { "focmode", "4", RT_NULL, RT_NULL };
    int   n = 2;

    if (argc >= 2) { a[2] = argv[1]; n = 3; }
    if (argc >= 3) { a[3] = argv[2]; n = 4; }
    (void)foc_mode_msh(n, a);
}
MSH_CMD_EXPORT_ALIAS(cmd_foc_current, current, FOC current loop [id] [iq] (focmode 4));

static void cmd_foc_speed(int argc, char **argv)
{
    char *a[3] = { "focmode", "9", RT_NULL };
    int   n = 2;

    if (argc >= 2) {
        a[2] = argv[1];
        n = 3;
    }
    (void)foc_mode_msh(n, a);
}
MSH_CMD_EXPORT_ALIAS(cmd_foc_speed, speed, FOC speed loop [rpm] (focmode 9));

static void cmd_foc_position(int argc, char **argv)
{
    char *a[3] = { "focmode", "10", RT_NULL };
    int   n = 2;

    if (argc >= 2) {
        a[2] = argv[1];
        n = 3;
    }
    (void)foc_mode_msh(n, a);
}
MSH_CMD_EXPORT_ALIAS(cmd_foc_position, position, FOC position loop [rev] (focmode 10));

static void cmd_foc_falign(int argc, char **argv)
{
    char *a[2] = { "focmode", "3" };

    (void)argc;
    (void)argv;
    (void)foc_mode_msh(2, a);
}
MSH_CMD_EXPORT_ALIAS(cmd_foc_falign, falign, FOC encoder align (focmode 3));

static void cmd_foc_zcal(int argc, char **argv)
{
    char *a[2] = { "focmode", "12" };

    (void)argc;
    (void)argv;
    (void)foc_mode_msh(2, a);
}
MSH_CMD_EXPORT_ALIAS(cmd_foc_zcal, zcal, FOC zero offset cal no PWM (focmode 12));

static void cmd_foc_zcal50(int argc, char **argv)
{
    char *a[2] = { "focmode", "13" };

    (void)argc;
    (void)argv;
    (void)foc_mode_msh(2, a);
}
MSH_CMD_EXPORT_ALIAS(cmd_foc_zcal50, zcal50, FOC zero offset cal PWM50 (focmode 13));

static void cmd_foc_fstat(int argc, char **argv)
{
    char *a[2] = { "focmode", "stat" };

    (void)argc;
    (void)argv;
    (void)foc_mode_msh(2, a);
}
MSH_CMD_EXPORT_ALIAS(cmd_foc_fstat, fstat, FOC key observations (focmode stat));
