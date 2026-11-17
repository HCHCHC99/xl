/**
 *******************************************************************************
 * @file  foc_mode.c
 * @brief FOC 模式层实现（IDLE / LOCK / IF / CALIB / CURRENT / SPEED / POSITION）
 *
 *        mode 3/5/6/7 为手动触发的编码器标定：
 *          - 动作由 encoder_calib 状态机执行
 *          - 标定期间 g_align_active=1，20kHz ISR 不碰 PWM
 *******************************************************************************
 */

#include "foc_mode.h"
#include "foc_current_loop.h"
#include "foc_speed_loop.h"
#include "foc_position_loop.h"
#include "encoder_calib.h"
#include "foc_control.h"
#include "foc_protect.h"
#include "cfg_foc_table.h"
#include "cfg_sampling.h"
#include "hw_tmr4_pwm.h"
#include "foc_eoca_probe.h"
#include "../../bsp/bsp_pwm.h"

#include <rtthread.h>
#include <finsh.h>

#include <stddef.h>

/* 20kHz ISR 热路径：Debug 构建是 -O0，闭环时单拍容易超时。只对本文件开 O2。 */
#pragma GCC optimize("O2")

#define FM_TWO_PI          6.28318530f
#define FOC_LOCK_DUTY_PCT  (2.0f)

/*=============================================================================
 * 内部状态
 *=============================================================================*/
static volatile foc_mode_t s_mode     = FOC_MODE_IDLE;
static volatile float      s_theta    = 0.0f;
static volatile float      s_freq     = 0.0f;
static volatile float      s_vq       = 0.0f;
static volatile float      s_vd       = 0.0f;
static volatile float      s_lock_duty = FOC_LOCK_DUTY_PCT;

volatile foc_mode_t g_fm_mode     = FOC_MODE_IDLE;
volatile float      g_fm_theta_e  = 0.0f;
volatile float      g_fm_freq_hz  = 0.0f;
volatile float      g_fm_vq       = 0.0f;
volatile float      g_fm_vd       = 0.0f;
volatile uint32_t   g_fm_step_cnt = 0;

static const char *foc_mode_name(foc_mode_t m)
{
    switch (m) {
    case FOC_MODE_IDLE:     return "IDLE, all low";
    case FOC_MODE_LOCK:     return "LOCK, zero vector";
    case FOC_MODE_IF:       return "IF, open-loop drag";
    case FOC_MODE_CALIB:    return "CALIB, encoder align";
    case FOC_MODE_CURRENT:  return "CURRENT, closed loop";
    case FOC_MODE_SWEEP:    return "SWEEP, angle sweep";
    case FOC_MODE_REV:      return "REV, one-rev count";
    case FOC_MODE_ALIGN7:   return "ALIGN7, ramp align";
    case FOC_MODE_SPEED:    return "SPEED, speed loop";
    case FOC_MODE_POSITION: return "POSITION, position loop";
    default:                return "UNKNOWN";
    }
}

/*=============================================================================
 * 初始化
 *=============================================================================*/
void Foc_Mode_Init(void)
{
    s_mode  = FOC_MODE_IDLE;
    s_theta = 0.0f;
    s_freq  = 0.0f;
    s_vq    = 0.0f;
    s_vd    = 0.0f;
    g_fm_mode = FOC_MODE_IDLE;

    (void)hw_tmr4_pwm_set_all_low();
    bsp_pwm_emergency_brake();

    rt_kprintf("[FOC-MODE] power-on default: mode 0 (IDLE, all low)\r\n");
}

foc_mode_t Foc_Mode_GetMode(void)
{
    return s_mode;
}

/*=============================================================================
 * 模式切换
 *=============================================================================*/
void Foc_Mode_SetMode(foc_mode_t mode)
{
    tmr4_pwm_err_t err;

    /* 任何模式切换先结束上一次标定状态机。
     * 否则 g_align_active 若残留为 1，20kHz ISR 会一直跳过 Foc_Mode_Step，
     * 表现为 IF/CURRENT/SPEED/POSITION 都不刷新占空比（只有 LOCK 能写）。 */
    EncoderCalib_Abort();

    /* ---- 离开旧模式的收尾 ---- */
    if (mode != FOC_MODE_SPEED && mode != FOC_MODE_POSITION) {
        Foc_Speed_Stop();
    }
    if (mode != FOC_MODE_POSITION) {
        Foc_Position_Stop();
    }
    if (mode != FOC_MODE_IF) {
        s_theta = 0.0f;
    }

    /* ---- 进入新模式 ---- */
    switch (mode) {
    case FOC_MODE_IDLE:
        err = hw_tmr4_pwm_set_all_low();
        if (err != TMR4_PWM_OK) {
            rt_kprintf("[FOC-MODE] mode 0 failed: %d\r\n", (int)err);
            return;
        }
        bsp_pwm_emergency_brake();
        break;

    case FOC_MODE_LOCK:
        bsp_pwm_emergency_release();
        err = hw_tmr4_pwm_resume_output();
        if (err != TMR4_PWM_OK) {
            bsp_pwm_emergency_brake();
            rt_kprintf("[FOC-MODE] mode 1 resume failed: %d\r\n", (int)err);
            return;
        }
        err = hw_tmr4_pwm_set_duty3(s_lock_duty, s_lock_duty, s_lock_duty);
        if (err != TMR4_PWM_OK) {
            (void)hw_tmr4_pwm_set_all_low();
            bsp_pwm_emergency_brake();
            rt_kprintf("[FOC-MODE] mode 1 duty failed: %d\r\n", (int)err);
            return;
        }
        break;

    case FOC_MODE_IF:
        bsp_pwm_emergency_release();
        (void)hw_tmr4_pwm_resume_output();
        s_theta = 0.0f;
        break;

    case FOC_MODE_CALIB:
        bsp_pwm_emergency_release();
        EncoderCalib_StartAlign();
        break;

    case FOC_MODE_SWEEP:
        bsp_pwm_emergency_release();
        EncoderCalib_StartSweep();
        break;

    case FOC_MODE_REV:
        bsp_pwm_emergency_release();
        EncoderCalib_StartRevMeasure();
        break;

    case FOC_MODE_ALIGN7:
        bsp_pwm_emergency_release();
        EncoderCalib_StartAlign7();
        break;

    case FOC_MODE_CURRENT:
        bsp_pwm_emergency_release();
        (void)hw_tmr4_pwm_resume_output();
        Foc_CurrentLoop_Reset();
        break;

    case FOC_MODE_SPEED:
        bsp_pwm_emergency_release();
        (void)hw_tmr4_pwm_resume_output();
        Foc_CurrentLoop_Reset();
        Foc_Speed_ConfigCurrentLoop();
        Foc_Speed_Start();
        break;

    case FOC_MODE_POSITION:
        bsp_pwm_emergency_release();
        (void)hw_tmr4_pwm_resume_output();
        Foc_CurrentLoop_Reset();
        Foc_Speed_ConfigCurrentLoop();
        Foc_Speed_Start();
        Foc_Position_Reset();
        break;

    default:
        rt_kprintf("[FOC-MODE] unsupported mode %d\r\n", (int)mode);
        return;
    }

    s_mode = mode;

    rt_kprintf("[FOC-MODE] -> mode %d (%s)\r\n", (int)mode, foc_mode_name(mode));

    if ((mode != FOC_MODE_IDLE) && (mode != FOC_MODE_LOCK) &&
        (mode != FOC_MODE_IF) && (mode != FOC_MODE_CALIB)) {
        rt_kprintf("[FOC-MODE] hint: run 'focmode 3' first if not aligned\r\n");
    }
}

void Foc_Mode_SetOpenloop(float freq_hz, float vq_volts, float vd_volts)
{
    s_freq = freq_hz;
    s_vq   = vq_volts;
    s_vd   = vd_volts;
}

void Foc_Mode_StartOpenloop(void)
{
    /* 与参考工程 Enter_IF_Mode() 一致：
     * 先复位角度，再走完整模式切换（解除 EMB / 恢复 PWM / 清理标定与旧环） */
    Foc_Mode_SetMode(FOC_MODE_IF);
}

void Foc_Mode_Stop(void)
{
    Foc_Mode_SetMode(FOC_MODE_IDLE);
}

/*=============================================================================
 * 每拍
 *=============================================================================*/
void Foc_Mode_Step(float ts, FocCmd_t *out)
{
    if (out == NULL) return;

    switch (s_mode) {
    case FOC_MODE_IF:
        s_theta += FM_TWO_PI * s_freq * ts;
        if (s_theta >= FM_TWO_PI) s_theta -= FM_TWO_PI;
        if (s_theta < 0.0f)       s_theta += FM_TWO_PI;

        out->vd      = s_vd;
        out->vq      = s_vq;
        out->theta_e = s_theta;
        out->enable  = true;
        break;

    case FOC_MODE_CURRENT:
        Foc_CurrentLoop_Step(&out->vd, &out->vq, &out->theta_e);
        out->enable = true;
        break;

    case FOC_MODE_SPEED:
        Foc_Speed_Task20k();
        Foc_CurrentLoop_Step(&out->vd, &out->vq, &out->theta_e);
        out->enable = true;
        break;

    case FOC_MODE_POSITION:
        Foc_Position_Task20k();
        Foc_Speed_Task20k();
        Foc_CurrentLoop_Step(&out->vd, &out->vq, &out->theta_e);
        out->enable = true;
        break;

    case FOC_MODE_CALIB:
    case FOC_MODE_SWEEP:
    case FOC_MODE_REV:
    case FOC_MODE_ALIGN7:
    case FOC_MODE_LOCK:
    case FOC_MODE_IDLE:
    default:
        out->vd = out->vq = 0.0f;
        out->theta_e = 0.0f;
        out->enable  = false;
        break;
    }

    g_fm_mode     = s_mode;
    g_fm_theta_e  = out->theta_e;
    g_fm_freq_hz  = s_freq;
    g_fm_vq       = out->vq;
    g_fm_vd       = out->vd;
    g_fm_step_cnt++;
}

/*=============================================================================
 * LOCK 占空比 / PWM 归属
 *=============================================================================*/
void Foc_Mode_SetLockDuty(float duty_pct)
{
    if (duty_pct < 0.0f)   duty_pct = 0.0f;
    if (duty_pct > 100.0f) duty_pct = 100.0f;

    s_lock_duty = duty_pct;

    if (s_mode == FOC_MODE_LOCK) {
        int d10 = (int)(duty_pct * 10.0f + 0.5f);

        (void)hw_tmr4_pwm_set_duty3(duty_pct, duty_pct, duty_pct);
        rt_kprintf("[FOC-MODE] LOCK duty -> %d.%d%%\r\n", d10 / 10, d10 % 10);
    }
}

float Foc_Mode_GetLockDuty(void)
{
    return s_lock_duty;
}

int Foc_Mode_PwmOwned(void)
{
#if FOC_DISABLE_LEGACY_6STEP
    return 1;
#else
    return (s_mode != FOC_MODE_IDLE) ? 1 : 0;
#endif
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
    int d10 = (int)(Foc_Mode_GetLockDuty() * 10.0f + 0.5f);

    rt_kprintf("[FOC-MODE] current=%d (%s), lock duty=%d.%d%%, aligned=%u\r\n",
               (int)Foc_Mode_GetMode(),
               foc_mode_name(Foc_Mode_GetMode()),
               d10 / 10, d10 % 10,
               (unsigned)g_motor_is_aligned);
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
 *   focmode stat       关键观测量
 *=============================================================================*/
static int foc_mode_msh(int argc, char **argv)
{
    int   m;
    float v1 = 0.0f, v2 = 0.0f;

    if (argc == 1) {
        foc_mode_print_status();
        rt_kprintf("usage: focmode <0..10> [args]\r\n"
                   "  0=IDLE 1=LOCK[duty%%] 2=IF[freq,volt] 3=CALIB 4=CURRENT[id,iq]\r\n"
                   "  5=SWEEP 6=REV 7=ALIGN7 9=SPEED[rpm] 10=POSITION[rev]\r\n"
                   "  focmode stat\r\n");
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
        rt_kprintf("[FC] iu=%d.%02d iv=%d.%02d iw=%d.%02d vbus=%d.%d V\r\n",
                   (int)g_fc_iu, (int)((g_fc_iu - (float)(int)g_fc_iu) * 100.0f),
                   (int)g_fc_iv, (int)((g_fc_iv - (float)(int)g_fc_iv) * 100.0f),
                   (int)g_fc_iw, (int)((g_fc_iw - (float)(int)g_fc_iw) * 100.0f),
                   (int)g_fc_vbus, (int)((g_fc_vbus - (float)(int)g_fc_vbus) * 10.0f));
        rt_kprintf("[FC] id=%d.%02d iq=%d.%02d elec=%d.%d deg\r\n",
                   (int)g_fc_id, (int)((g_fc_id - (float)(int)g_fc_id) * 100.0f),
                   (int)g_fc_iq, (int)((g_fc_iq - (float)(int)g_fc_iq) * 100.0f),
                   (int)g_fc_enc_elec_deg,
                   (int)((g_fc_enc_elec_deg - (float)(int)g_fc_enc_elec_deg) * 10.0f));
        rt_kprintf("[FC] fp_state=%d fault=0x%08lX fast=%d.%02d slow=%d.%02d\r\n",
                   (int)g_fp_state, (unsigned long)g_fp_fault,
                   (int)g_fp_fast_max, (int)((g_fp_fast_max - (float)(int)g_fp_fast_max) * 100.0f),
                   (int)g_fp_slow_avg, (int)((g_fp_slow_avg - (float)(int)g_fp_slow_avg) * 100.0f));
        rt_kprintf("[FC] spd=%d.%d rpm tgt=%d.%d, iqref=%d mA, pos=%d.%03d rev\r\n",
                   (int)g_spd_filt_rpm, (int)((g_spd_filt_rpm - (float)(int)g_spd_filt_rpm) * 10.0f),
                   (int)g_spd_target_rpm, (int)((g_spd_target_rpm - (float)(int)g_spd_target_rpm) * 10.0f),
                   (int)g_spd_out_ma,
                   (int)g_pos_fb_rev, (int)((g_pos_fb_rev - (float)(int)g_pos_fb_rev) * 1000.0f));
        rt_kprintf("[FC] mode=%d theta=%d.%d deg, IF f=%d.%d Hz vq=%d.%d V, align_active=%u\r\n",
                   (int)g_fm_mode,
                   (int)(g_fm_theta_e * 57.29578f),
                   (int)((g_fm_theta_e * 57.29578f -
                          (float)(int)(g_fm_theta_e * 57.29578f)) * 10.0f),
                   (int)g_fm_freq_hz,
                   (int)((g_fm_freq_hz - (float)(int)g_fm_freq_hz) * 10.0f),
                   (int)g_fm_vq,
                   (int)((g_fm_vq - (float)(int)g_fm_vq) * 10.0f),
                   (unsigned)g_align_active);
        rt_kprintf("[FC] duty ");
        foc_mode_print_duties();
        return 0;
    }

    m = (int)(argv[1][0] - '0');
    if ((argv[1][1] != '\0') && !((m == 1) && (argv[1][1] == '0'))) {
        rt_kprintf("invalid mode: %s\r\n", argv[1]);
        return -1;
    }
    if (argv[1][1] == '0') {
        m = 10;
    }

    switch (m) {
    case 1:
        if (argc >= 3) {
            if (foc_mode_parse_float(argv[2], &v1) != 0) {
                rt_kprintf("invalid duty: %s\r\n", argv[2]);
                return -1;
            }
            Foc_Mode_SetLockDuty(v1);
        }
        Foc_Mode_SetMode(FOC_MODE_LOCK);
        if (Foc_Mode_GetMode() == FOC_MODE_LOCK) {
            foc_mode_dump_pwm(Foc_Mode_GetLockDuty());
        }
        return 0;

    case 2:
        v1 = 1.0f;    /* 频率 Hz */
        v2 = 1.5f;    /* 电压 V（q 轴） */
        if (argc >= 3) { if (foc_mode_parse_float(argv[2], &v1) != 0) return -1; }
        if (argc >= 4) { if (foc_mode_parse_float(argv[3], &v2) != 0) return -1; }
        Foc_Mode_SetOpenloop(v1, v2, 0.0f);   /* IF 只给频率和电压，vd 固定 0 */
        Foc_Mode_StartOpenloop();             /* 参考工程 Enter_IF_Mode() 的同一入口 */
        {
            float swing = (v2 >= 0.0f) ? v2 : -v2;

            swing = swing / (CFG_SAMPLING_SIM_VBUS_V * 0.57735027f) * 100.0f;
            rt_kprintf("[FOC-MODE] IF running: f=%d.%d Hz vq=%d.%d V, "
                       "duty swing +/-%d%% around 50%%, use 'focmode stat' to watch\r\n",
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
        v1 = 0.0f;
        v2 = 0.5f;
        if (argc >= 3) { if (foc_mode_parse_float(argv[2], &v1) != 0) return -1; }
        if (argc >= 4) { if (foc_mode_parse_float(argv[3], &v2) != 0) return -1; }
        Foc_Mode_SetMode(FOC_MODE_CURRENT);
        Foc_CurrentLoop_SetRef(v1, v2);
        return 0;

    case 9:
        v1 = 0.0f;
        if (argc >= 3) { if (foc_mode_parse_float(argv[2], &v1) != 0) return -1; }
        Foc_Mode_SetMode(FOC_MODE_SPEED);
        g_spd_target_rpm = v1;
        return 0;

    case 10:
        v1 = 0.0f;
        if (argc >= 3) { if (foc_mode_parse_float(argv[2], &v1) != 0) return -1; }
        Foc_Mode_SetMode(FOC_MODE_POSITION);
        g_pos_target_rev = v1;
        return 0;

    case 0:
        Foc_Mode_SetMode(FOC_MODE_IDLE);
        return 0;

    case 3:
        Foc_Mode_SetMode(FOC_MODE_CALIB);
        return 0;

    case 5:
        Foc_Mode_SetMode(FOC_MODE_SWEEP);
        return 0;

    case 6:
        Foc_Mode_SetMode(FOC_MODE_REV);
        return 0;

    case 7:
        Foc_Mode_SetMode(FOC_MODE_ALIGN7);
        return 0;

    default:
        rt_kprintf("unsupported mode %d (11/14 not ported)\r\n", m);
        return -1;
    }
}

MSH_CMD_EXPORT_ALIAS(foc_mode_msh, focmode, set FOC mode 0..10);
