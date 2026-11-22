/**
 *******************************************************************************
 * @file  foc_speed_loop.c
 * @brief L7 速度外环实现
 *
 *        速度估计：每拍取 g_fc_enc_delta，限幅后压入 100 拍（5ms）环形缓冲，
 *        每 1ms 取窗口和 -> raw_rpm = sum × (200Hz × 60 / CPR) -> EMA
 *******************************************************************************
 */

#include "foc_speed_loop.h"
#include "pid_ctrl.h"
#include "foc_current_loop.h"
#include "foc_control.h"
#include "core/foc_feedback.h"
#include "cfg_motor.h"
#include "cfg_foc_table.h"

#include <rtthread.h>

/*---------------------------------------------------------------------------
 * 编译期常量
 *---------------------------------------------------------------------------*/
#define SPD_ISR_HZ          ((float)FOC_CFG_PWM_FREQ)
#define SPD_DT_S            ((float)SPD_CTRL_DIV / SPD_ISR_HZ)
#define SPD_WIN_HZ          (SPD_ISR_HZ / (float)SPD_WIN_TICKS)
#define SPD_RPM_SCALE       (SPD_WIN_HZ * 60.0f / (float)CFG_ENCODER_CPR)

/*---------------------------------------------------------------------------
 * 可调参数变量
 *---------------------------------------------------------------------------*/
volatile float g_spd_target_rpm    = 0.0f;
volatile float g_spd_kp            = SPD_KP_INIT;
volatile float g_spd_ki            = SPD_KI_INIT;
volatile float g_spd_kd            = SPD_KD_INIT;
volatile float g_spd_accel_rpm_s   = SPD_ACCEL_RPM_S_INIT;
volatile float g_spd_iq_limit_ma   = SPD_IQ_LIMIT_MA_INIT;
volatile float g_spd_iterm_max_ma  = SPD_ITERM_MAX_MA_INIT;
volatile float g_spd_ff_ma         = SPD_FF_MA_INIT;
volatile float g_spd_back_gain     = 0.0f;
volatile float g_spd_fb_alpha      = SPD_FB_ALPHA;
volatile uint32_t g_spd_aw_mask    = SPD_AW_MASK_INIT;
volatile float g_spd_aw_cond_thr   = SPD_AW_COND_THR_INIT;

volatile float g_spd_cur_kp        = SPD_CUR_KP_INIT;
volatile float g_spd_cur_ki        = SPD_CUR_KI_INIT;
volatile float g_spd_cur_kd        = SPD_CUR_KD_INIT;
volatile float g_spd_cur_vmax_v    = SPD_CUR_VMAX_INIT;
volatile float g_spd_cur_iterm_v   = SPD_CUR_ITERM_MAX_INIT;
volatile float g_spd_cur_fb_alpha  = SPD_CUR_FB_ALPHA_INIT;

/*---------------------------------------------------------------------------
 * 观测量
 *---------------------------------------------------------------------------*/
volatile uint8_t g_spd_active    = 0u;
volatile float   g_spd_raw_rpm   = 0.0f;
volatile float   g_spd_filt_rpm  = 0.0f;
volatile float   g_spd_ramp_rpm  = 0.0f;
volatile float   g_spd_err_rpm   = 0.0f;
volatile float   g_spd_out_ma    = 0.0f;
volatile float   g_spd_iterm_ma  = 0.0f;

/*---------------------------------------------------------------------------
 * 内部状态
 *---------------------------------------------------------------------------*/
static struct pid_dev  s_pid_spd;
static struct pid_priv s_priv_spd;

static bool     s_ready    = false;
static uint16_t s_div_cnt  = 0u;
static float    s_raw_rpm  = 0.0f;
static float    s_filt_rpm = 0.0f;
static float    s_ramp_rpm = 0.0f;

static int16_t  s_win[SPD_WIN_TICKS];
static uint16_t s_win_idx  = 0u;
static int32_t  s_win_sum  = 0;
static uint16_t s_win_fill = 0u;

static void spd_win_clear(void)
{
    uint16_t i;

    for (i = 0u; i < (uint16_t)SPD_WIN_TICKS; i++) {
        s_win[i] = 0;
    }
    s_win_idx  = 0u;
    s_win_sum  = 0;
    s_win_fill = 0u;
}

static float spd_ramp(float target, float current, float step)
{
    if (step <= 0.0f) {
        return target;
    }
    if (target > current + step) return current + step;
    if (target < current - step) return current - step;
    return target;
}

/*=============================================================================
 * 初始化
 *=============================================================================*/
void Foc_Speed_Init(void)
{
    if (s_ready) {
        return;
    }

    if (pid_dev_init(&s_pid_spd, &s_priv_spd, PID_MODE_POSITION) != 0) {
        rt_kprintf("[SPD] pid_dev_init failed\r\n");
        return;
    }

    (void)s_pid_spd.ops->set_param(&s_priv_spd, SPD_KP_INIT, SPD_KI_INIT, SPD_KD_INIT);
    (void)s_pid_spd.ops->set_limit(&s_priv_spd,
                                   -SPD_IQ_LIMIT_MA_INIT,
                                    SPD_IQ_LIMIT_MA_INIT, 0.0f);
    (void)s_pid_spd.ops->set_iterm_limit(&s_priv_spd, SPD_ITERM_MAX_MA_INIT);

    s_div_cnt  = 0u;
    spd_win_clear();
    s_raw_rpm  = 0.0f;
    s_filt_rpm = 0.0f;
    s_ramp_rpm = 0.0f;

    g_spd_active   = 0u;
    g_spd_raw_rpm  = 0.0f;
    g_spd_filt_rpm = 0.0f;
    g_spd_ramp_rpm = 0.0f;
    g_spd_err_rpm  = 0.0f;
    g_spd_out_ma   = 0.0f;
    g_spd_iterm_ma = 0.0f;

    s_ready = true;

    rt_kprintf("[SPD] speed loop ready: div=%u kp_x10000=%ld ki_x10000=%ld "
               "iq_lim_mA=%ld accel_rpm_s=%ld\r\n",
               (unsigned)SPD_CTRL_DIV,
               (long)(g_spd_kp * 10000.0f),
               (long)(g_spd_ki * 10000.0f),
               (long)g_spd_iq_limit_ma,
               (long)g_spd_accel_rpm_s);
}

/*=============================================================================
 * 启停
 *=============================================================================*/
void Foc_Speed_Start(void)
{
    g_spd_target_rpm = 0.0f;

    if (!s_ready) {
        return;
    }

    s_pid_spd.ops->reset(&s_priv_spd);

    s_div_cnt  = 0u;
    spd_win_clear();
    s_raw_rpm  = 0.0f;
    s_filt_rpm = 0.0f;
    s_ramp_rpm = 0.0f;

    g_spd_active   = 0u;
    g_spd_raw_rpm  = 0.0f;
    g_spd_filt_rpm = 0.0f;
    g_spd_ramp_rpm = 0.0f;
    g_spd_err_rpm  = 0.0f;
    g_spd_out_ma   = 0.0f;
    g_spd_iterm_ma = 0.0f;

    Foc_CurrentLoop_SetRefDirect(0.0f, 0.0f);
}

void Foc_Speed_Stop(void)
{
    g_spd_target_rpm = 0.0f;

    if (!s_ready) {
        return;
    }

    s_pid_spd.ops->reset(&s_priv_spd);

    s_div_cnt  = 0u;
    spd_win_clear();
    s_raw_rpm  = 0.0f;
    s_filt_rpm = 0.0f;
    s_ramp_rpm = 0.0f;

    g_spd_active   = 0u;
    g_spd_raw_rpm  = 0.0f;
    g_spd_filt_rpm = 0.0f;
    g_spd_ramp_rpm = 0.0f;
    g_spd_err_rpm  = 0.0f;
    g_spd_out_ma   = 0.0f;
    g_spd_iterm_ma = 0.0f;

    Foc_CurrentLoop_SetRefDirect(0.0f, 0.0f);
}

void Foc_Speed_ResetIntegral(void)
{
    if (!s_ready) {
        return;
    }
    s_pid_spd.ops->reset(&s_priv_spd);
    g_spd_iterm_ma = 0.0f;
}

/*=============================================================================
 * 每拍任务（滑窗每拍滚动，内部 1ms 抽稀）
 *=============================================================================*/
void Foc_Speed_Task20k(void)
{
    int32_t d;
    float   alpha;
    float   target;
    float   step;
    float   iq_lim;
    float   iterm;
    float   bg;
    float   ff;
    float   out_ma;

    if (!s_ready) {
        return;
    }

    d = g_fc_enc_delta;
    if (d >  SPD_ENC_DELTA_CLAMP) d =  SPD_ENC_DELTA_CLAMP;
    if (d < -SPD_ENC_DELTA_CLAMP) d = -SPD_ENC_DELTA_CLAMP;

    s_win_sum -= (int32_t)s_win[s_win_idx];
    s_win[s_win_idx] = (int16_t)d;
    s_win_sum += d;
    s_win_idx++;
    if (s_win_idx >= (uint16_t)SPD_WIN_TICKS) {
        s_win_idx = 0u;
    }
    if (s_win_fill < (uint16_t)SPD_WIN_TICKS) {
        s_win_fill++;
    }

    s_div_cnt++;
    if (s_div_cnt < SPD_CTRL_DIV) {
        return;
    }
    s_div_cnt = 0u;

    if (s_win_fill < (uint16_t)SPD_WIN_TICKS) {
        s_raw_rpm      = 0.0f;
        s_filt_rpm     = 0.0f;
        g_spd_raw_rpm  = 0.0f;
        g_spd_filt_rpm = 0.0f;
        return;
    }

    alpha = g_spd_fb_alpha;
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;

    s_raw_rpm = (float)s_win_sum * (float)Foc_Feedback_GetDir() * SPD_RPM_SCALE;
    s_filt_rpm += alpha * (s_raw_rpm - s_filt_rpm);

    g_spd_raw_rpm  = s_raw_rpm;
    g_spd_filt_rpm = s_filt_rpm;

    if (!Foc_CurrentLoop_IsActive()) {
        s_ramp_rpm     = 0.0f;
        g_spd_ramp_rpm = 0.0f;
        g_spd_err_rpm  = 0.0f;
        g_spd_out_ma   = 0.0f;
        g_spd_iterm_ma = 0.0f;
        g_spd_active   = 0u;
        Foc_CurrentLoop_SetRefDirect(0.0f, 0.0f);
        return;
    }
    g_spd_active = 1u;

    /* 参数在线同步（Watch 改动下一拍生效） */
    iq_lim = g_spd_iq_limit_ma;
    if (iq_lim < 1.0f) {
        iq_lim = 1.0f;
    }
    iterm = g_spd_iterm_max_ma;
    if (iterm < 0.0f) {
        iterm = 0.0f;
    }
    bg = g_spd_back_gain;
    if (bg < 0.0f) {
        bg = 0.0f;
    }
    if (bg > 1.0f) {
        bg = 1.0f;
    }

    (void)s_pid_spd.ops->set_param(&s_priv_spd, g_spd_kp, g_spd_ki, g_spd_kd);
    (void)s_pid_spd.ops->set_limit(&s_priv_spd, -iq_lim, iq_lim, bg);
    (void)s_pid_spd.ops->set_iterm_limit(&s_priv_spd, iterm);
    (void)s_pid_spd.ops->set_aw_mode(&s_priv_spd, g_spd_aw_mask);
    (void)s_pid_spd.ops->set_aw_cond_thr(&s_priv_spd, g_spd_aw_cond_thr);

    target = g_spd_target_rpm;
    if (target >  SPD_TARGET_MAX_RPM) target =  SPD_TARGET_MAX_RPM;
    if (target < -SPD_TARGET_MAX_RPM) target = -SPD_TARGET_MAX_RPM;

    step = g_spd_accel_rpm_s * SPD_DT_S;
    s_ramp_rpm = spd_ramp(target, s_ramp_rpm, step);

    out_ma = s_pid_spd.ops->calc(&s_priv_spd, s_filt_rpm, s_ramp_rpm,
                                 0.0f, 0.0f, SPD_DT_S);

    ff = g_spd_ff_ma;
    if (ff < 0.0f) {
        ff = 0.0f;
    }
    if (s_ramp_rpm > SPD_FF_DEADBAND_RPM) {
        out_ma += ff;
    } else if (s_ramp_rpm < -SPD_FF_DEADBAND_RPM) {
        out_ma -= ff;
    }
    if (out_ma >  iq_lim) out_ma =  iq_lim;
    if (out_ma < -iq_lim) out_ma = -iq_lim;

    g_spd_ramp_rpm = s_ramp_rpm;
    g_spd_err_rpm  = s_ramp_rpm - s_filt_rpm;
    g_spd_out_ma   = out_ma;
    g_spd_iterm_ma = g_spd_ki * s_priv_spd.integral;

    Foc_CurrentLoop_SetRefDirect(0.0f, out_ma * 0.001f);
}

/*=============================================================================
 * 速度模式电流环参数
 *=============================================================================*/
void Foc_Speed_ConfigCurrentLoop(void)
{
    Foc_CurrentLoop_ApplyParams(g_spd_cur_kp, g_spd_cur_ki, g_spd_cur_kd,
                                g_spd_cur_vmax_v, g_spd_cur_iterm_v,
                                g_spd_cur_fb_alpha);
    Foc_CurrentLoop_SetFF(0u);
}
