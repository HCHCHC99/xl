/*=============================================================================
 * foc_speed_loop.c  —  L7 速度外环实现
 *
 * 调度：Foc_Mode_Step 的 FOC_MODE_SPEED 分支，每拍调用 Foc_Speed_Task20k，
 *       内部每 100 拍（5ms）执行一次完整的速度环计算。
 *
 * 速度估计：
 *   每拍取 g_fc_enc_delta（电流环 ISR 已算好的编码器增量），限幅 ±36 后
 *   按 g_enc_elec_dir 统一到 FOC 正方向，100 拍累加一次。
 *   raw_rpm = sum_counts × (200Hz × 60 / CPR) = sum × 2.9296875（CPR=4096）
 *   再 EMA α=0.25 得到 PI 反馈。
 *
 * 激活门控：
 *   电流环预热 + 在线零偏校准完成前（Foc_CurrentLoop_IsActive()=false），
 *   速度环冻结斜坡、不积分、iq_ref 恒为 0，避免预热线里把积分顶到限幅。
 *=============================================================================*/

#include "foc_speed_loop.h"
#include "pid_ctrl.h"
#include "foc_current_loop.h"
#include "foc_control.h"      /* g_fc_enc_delta */
#include "hw_encoder.h"       /* g_enc_elec_dir */
#include "cfg_motor.h"        /* CFG_ENCODER_CPR */
#include "rtt_log.h"

/*---------------------------------------------------------------------------
 * 编译期常量
 *---------------------------------------------------------------------------*/
#define SPD_ISR_HZ          (20000.0f)
#define SPD_DT_S            ((float)SPD_CTRL_DIV / SPD_ISR_HZ)          /* 0.005s */
#define SPD_SAMPLE_HZ       (SPD_ISR_HZ / (float)SPD_CTRL_DIV)          /* 200Hz */
/* counts/5ms -> rpm = sum × 200 × 60 / CPR */
#define SPD_RPM_SCALE       (SPD_SAMPLE_HZ * 60.0f / (float)CFG_ENCODER_CPR)

/*---------------------------------------------------------------------------
 * 可调参数变量
 *---------------------------------------------------------------------------*/
volatile float g_spd_target_rpm    = 0.0f;
volatile float g_spd_kp            = SPD_KP_INIT;
volatile float g_spd_ki            = SPD_KI_INIT;
volatile float g_spd_accel_rpm_s   = SPD_ACCEL_RPM_S_INIT;
volatile float g_spd_iq_limit_ma   = SPD_IQ_LIMIT_MA_INIT;
volatile float g_spd_iterm_max_ma  = SPD_ITERM_MAX_MA_INIT;
volatile float g_spd_back_gain     = 0.0f;

volatile float g_spd_cur_kp        = SPD_CUR_KP_INIT;
volatile float g_spd_cur_ki        = SPD_CUR_KI_INIT;
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
static int32_t  s_cnt_sum  = 0;
static float    s_raw_rpm  = 0.0f;
static float    s_filt_rpm = 0.0f;
static float    s_ramp_rpm = 0.0f;

/* 线性斜坡：每 5ms 向 target 靠近，步长不超 step；step <= 0 视为无斜坡 */
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
        return;   /* 幂等 */
    }

    if (pid_dev_init(&s_pid_spd, &s_priv_spd, PID_MODE_POSITION) != 0) {
        MAIN_E("[SPD] pid_dev_init failed");
        return;
    }

    (void)s_pid_spd.ops->set_param(&s_priv_spd, SPD_KP_INIT, SPD_KI_INIT, 0.0f);
    (void)s_pid_spd.ops->set_limit(&s_priv_spd,
                                   -SPD_IQ_LIMIT_MA_INIT,
                                    SPD_IQ_LIMIT_MA_INIT, 0.0f);
    (void)s_pid_spd.ops->set_iterm_limit(&s_priv_spd, SPD_ITERM_MAX_MA_INIT);

    s_div_cnt  = 0u;
    s_cnt_sum  = 0;
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

    MAIN_D("[SPD] speed loop ready: div=%u dt_us=%ld kp_x10000=%ld ki_x10000=%ld "
           "iq_lim_mA=%ld iterm_mA=%ld accel_rpm_s=%ld cpr=%lu",
           (unsigned)SPD_CTRL_DIV,
           (long)(SPD_DT_S * 1e6f),
           (long)(g_spd_kp * 10000.0f),
           (long)(g_spd_ki * 10000.0f),
           (long)g_spd_iq_limit_ma,
           (long)g_spd_iterm_max_ma,
           (long)g_spd_accel_rpm_s,
           (unsigned long)CFG_ENCODER_CPR);
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
    s_cnt_sum  = 0;
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
    s_cnt_sum  = 0;
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

/*=============================================================================
 * 每拍任务（内部 5ms 抽稀）
 *=============================================================================*/
void Foc_Speed_Task20k(void)
{
    int32_t d;
    float   target;
    float   step;
    float   iq_lim;
    float   iterm;
    float   bg;
    float   out_ma;

    if (!s_ready) {
        return;
    }

    /* ---- 5ms 速度累加：单拍增量限幅后统一到 FOC 正方向 ---- */
    d = g_fc_enc_delta;
    if (d >  SPD_ENC_DELTA_CLAMP) d =  SPD_ENC_DELTA_CLAMP;
    if (d < -SPD_ENC_DELTA_CLAMP) d = -SPD_ENC_DELTA_CLAMP;
    s_cnt_sum += d * (int32_t)g_enc_elec_dir;

    s_div_cnt++;
    if (s_div_cnt < SPD_CTRL_DIV) {
        return;
    }
    s_div_cnt = 0u;

    s_raw_rpm = (float)s_cnt_sum * SPD_RPM_SCALE;
    s_cnt_sum = 0;
    s_filt_rpm += SPD_FB_ALPHA * (s_raw_rpm - s_filt_rpm);

    g_spd_raw_rpm  = s_raw_rpm;
    g_spd_filt_rpm = s_filt_rpm;

    /* ---- 电流环未 active：冻结斜坡、不积分、iq_ref 归零 ---- */
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

    /* ---- 参数在线同步（Watch 改动下一拍生效）---- */
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

    (void)s_pid_spd.ops->set_param(&s_priv_spd, g_spd_kp, g_spd_ki, 0.0f);
    (void)s_pid_spd.ops->set_limit(&s_priv_spd, -iq_lim, iq_lim, bg);
    (void)s_pid_spd.ops->set_iterm_limit(&s_priv_spd, iterm);

    /* ---- 目标限幅 + 加速度斜坡 ---- */
    target = g_spd_target_rpm;
    if (target >  SPD_TARGET_MAX_RPM) target =  SPD_TARGET_MAX_RPM;
    if (target < -SPD_TARGET_MAX_RPM) target = -SPD_TARGET_MAX_RPM;

    step = g_spd_accel_rpm_s * SPD_DT_S;
    s_ramp_rpm = spd_ramp(target, s_ramp_rpm, step);

    /* ---- 速度 PI：输出 mA ---- */
    out_ma = s_pid_spd.ops->calc(&s_priv_spd, s_filt_rpm, s_ramp_rpm,
                                 0.0f, 0.0f, SPD_DT_S);

    g_spd_ramp_rpm = s_ramp_rpm;
    g_spd_err_rpm  = s_ramp_rpm - s_filt_rpm;
    g_spd_out_ma   = out_ma;
    g_spd_iterm_ma = g_spd_ki * s_priv_spd.integral;

    /* ---- 下发 iq_ref，id_ref 恒为 0 ---- */
    Foc_CurrentLoop_SetRefDirect(0.0f, out_ma * 0.001f);
}

/*=============================================================================
 * 速度模式电流环参数
 *=============================================================================*/
void Foc_Speed_ConfigCurrentLoop(void)
{
    Foc_CurrentLoop_ApplyParams(g_spd_cur_kp, g_spd_cur_ki, 0.0f,
                                g_spd_cur_vmax_v, g_spd_cur_iterm_v,
                                g_spd_cur_fb_alpha);
    Foc_CurrentLoop_SetFF(0u);
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
