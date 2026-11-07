/*=============================================================================
 * foc_current_loop.c  —  L6.5 电流闭环层实现（力矩模式，有感）
 *
 * 数据流（20kHz ISR）：
 *   Foc_Mode_Step (L6)
 *     -> Foc_CurrentLoop_Step()
 *          [预热] 前 N 拍强制 vd=vq=0，等 PWM/反馈稳定
 *          -> g_fc_id / g_fc_iq                    (L5 第3步反馈)
 *          -> hw_encoder_get_elec_angle_rad()      (电角度，已扣 offset)
 *          -> ref 斜坡：目标 -> 实际（限 g_cl_ref_slew）
 *          -> pid_dev.ops->calc()                  (d/q 各一路)
 *          -> 斜率限幅（g_cl_vq_slew） -> *vd / *vq / *theta_e
 *          -> 受限判断：SELF / VOLTAGE / PROTECT
 *     -> L5 走 InvPark -> SVPWM -> SetDuty（与 IF 复用同一分支）
 *
 * 参数初值全部在 foc_current_loop.h 的 *_INIT 宏里，本文件只做逻辑
 *=============================================================================*/

#include "foc_current_loop.h"
#include "pid_ctrl.h"
#include "hw_encoder.h"
#include "cfg_motor.h"
#include "cfg_sampling.h"
#include "clarke_park.h"     /* ONE_DIV_SQRT3_F */
#include "foc_protect.h"     /* g_fp_state, FP_STATE_NORMAL */
#include "motor_sampling.h"  /* 进 mode4 时做 PWM 在线零偏校准 */
#include "rtt_log.h"

#include <math.h>            /* sqrtf */
#include <stddef.h>

/*=============================================================================
 * 在线可调参数定义（初值来自 .h 的 *_INIT 宏）
 *=============================================================================*/
volatile float    g_cl_ref_slew     = CL_REF_SLEW_INIT;
volatile float    g_cl_vq_slew      = CL_VQ_SLEW_INIT;
volatile float    g_cl_fb_alpha     = CL_FB_ALPHA_INIT;
volatile float    g_cl_iterm_max_watch = CL_ITERM_MAX_INIT;
volatile uint16_t g_cl_warmup_ticks = CL_WARMUP_INIT;
volatile float    g_cl_iref_max     = CL_IREF_MAX_INIT;
volatile float    g_cl_vmax_watch   = CL_VMAX_INIT;

/*---------------------------------------------------------------------------
 * 编译期常量（不需要在线调的）
 *---------------------------------------------------------------------------*/
#define CL_DT_S              (1.0f / 20000.0f)   /* 20kHz ISR 周期 */
#define CL_VLIM_RATIO        1.0f                /* PID 限幅 = vmax（参考工程 ±3.5V） */

/* 受限告警阈值：4000 拍 = 200ms */
#define CL_SAT_WARN_TICKS    4000u

/* mode4 进环前在 50/50 零矢量下重校电流零偏的超时（6000 拍 = 300ms） */
#define CL_OFFCAL_TIMEOUT_TICKS  6000u

/*---------------------------------------------------------------------------
 * 静态实例
 *---------------------------------------------------------------------------*/
static struct pid_dev  s_pid_d;
static struct pid_priv s_priv_d;
static struct pid_dev  s_pid_q;
static struct pid_priv s_priv_q;

static bool     s_ready      = false;
static float    s_vmax       = 0.0f;
static float    s_vlim       = 0.0f;

/* 目标 ref（SetRef 写入） */
static volatile float s_id_ref = 0.0f;
static volatile float s_iq_ref = 0.0f;

/* 斜坡后的实际 ref（给 PID 用） */
static float s_id_ref_ramped = 0.0f;
static float s_iq_ref_ramped = 0.0f;

/* 受限状态 */
static volatile cl_limit_reason_t s_limit_reason = CL_LIMIT_NONE;
static volatile uint16_t          s_sat_cnt      = 0u;
static volatile float             s_vmag_raw     = 0.0f;

/* 输出斜率限制上一拍 vq */
static float s_vq_prev = 0.0f;

/* id/iq 反馈 EMA 状态 */
static float s_id_f = 0.0f;
static float s_iq_f = 0.0f;

/* 进入预热计数 */
static uint16_t s_warmup_cnt = 0u;

/* 进 mode4 时的 PWM 在线零偏校准状态 */
static uint8_t  s_offcal_done  = 0u;
static uint32_t s_offcal_ticks = 0u;

/*---------------------------------------------------------------------------
 * L5 反馈镜像
 *---------------------------------------------------------------------------*/
extern volatile float g_fc_id;
extern volatile float g_fc_iq;

/*---------------------------------------------------------------------------
 * 内部工具
 *---------------------------------------------------------------------------*/
static float cl_clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static void cl_refresh_limits(void)
{
    s_vlim = s_vmax * CL_VLIM_RATIO;

    if (!s_ready) {
        return;
    }
    (void)s_pid_d.ops->set_limit(&s_priv_d, -s_vlim, s_vlim, CL_BACK_GAIN_INIT);
    (void)s_pid_q.ops->set_limit(&s_priv_q, -s_vlim, s_vlim, CL_BACK_GAIN_INIT);
}

/* 输出斜率限制：把 v 限制在 prev  slew 内 */
static float cl_slew(float v, float prev, float slew)
{
    if (slew <= 0.0f) {
        return v;
    }
    if (v > prev + slew) return prev + slew;
    if (v < prev - slew) return prev - slew;
    return v;
}

/* ref 线性斜坡：current 每拍向 target 靠近，步长不超 slew */
static float cl_ramp(float target, float current, float slew)
{
    if (slew <= 0.0f) {
        return target;
    }
    if (target > current + slew) return current + slew;
    if (target < current - slew) return current - slew;
    return target;
}

/*=============================================================================
 * 初始化
 *=============================================================================*/
void Foc_CurrentLoop_Init(void)
{
    if (s_ready) {
        return;   /* 幂等 */
    }

    /* vmax 初值：若 Watch 变量被提前改成非法值，回落到默认 */
    if (g_cl_vmax_watch < 1.0f) {
        g_cl_vmax_watch = CL_VMAX_INIT;
    }
    s_vmax = g_cl_vmax_watch;
    s_vlim = s_vmax * CL_VLIM_RATIO;

    if (pid_dev_init(&s_pid_d, &s_priv_d, PID_MODE_POSITION) != 0) {
        MAIN_E("[CL] pid_dev_init d failed");
        return;
    }
    if (pid_dev_init(&s_pid_q, &s_priv_q, PID_MODE_POSITION) != 0) {
        MAIN_E("[CL] pid_dev_init q failed");
        return;
    }

    (void)s_pid_d.ops->set_param(&s_priv_d, CL_KP_INIT, CL_KI_INIT, CL_KD_INIT);
    (void)s_pid_q.ops->set_param(&s_priv_q, CL_KP_INIT, CL_KI_INIT, CL_KD_INIT);

    (void)s_pid_d.ops->set_limit(&s_priv_d, -s_vlim, s_vlim, CL_BACK_GAIN_INIT);
    (void)s_pid_q.ops->set_limit(&s_priv_q, -s_vlim, s_vlim, CL_BACK_GAIN_INIT);

    (void)s_pid_d.ops->set_iterm_limit(&s_priv_d, g_cl_iterm_max_watch);
    (void)s_pid_q.ops->set_iterm_limit(&s_priv_q, g_cl_iterm_max_watch);

    (void)s_pid_d.ops->set_diff_filter(&s_priv_d, CL_DIFF_ALPHA_INIT);
    (void)s_pid_q.ops->set_diff_filter(&s_priv_q, CL_DIFF_ALPHA_INIT);

    (void)s_pid_d.ops->set_feedforward(&s_priv_d, 0.0f, 0.0f, false, false);
    (void)s_pid_q.ops->set_feedforward(&s_priv_q, 0.0f, 0.0f, false, false);

    s_id_ref         = 0.0f;
    s_iq_ref         = 0.0f;
    s_id_ref_ramped  = 0.0f;
    s_iq_ref_ramped  = 0.0f;
    s_limit_reason   = CL_LIMIT_NONE;
    s_sat_cnt        = 0u;
    s_vmag_raw       = 0.0f;
    s_vq_prev        = 0.0f;
    s_id_f           = 0.0f;
    s_iq_f           = 0.0f;
    s_warmup_cnt     = 0u;
    s_offcal_done    = 1u;   /* 未进 CURRENT 前不做在线校准 */
    s_offcal_ticks   = 0u;
    s_ready          = true;

    MAIN_D("[CL] current loop ready: vmax_x100=%ld vlim_x100=%ld dt_us_x10=%ld "
           "kp_x10000=%ld ki_x100=%ld ref_slew_x1e6=%ld vq_slew_x1e6=%ld "
           "iterm_x100=%ld fb_alpha_x100=%ld warmup=%u iref_max_x100=%ld",
           (long)(s_vmax * 100.0f),
           (long)(s_vlim * 100.0f),
           (long)(CL_DT_S * 1e6f * 10.0f),
           (long)(CL_KP_INIT * 10000.0f),
           (long)(CL_KI_INIT * 100.0f),
           (long)(g_cl_ref_slew * 1000000.0f),
           (long)(g_cl_vq_slew  * 1000000.0f),
           (long)(g_cl_iterm_max_watch * 100.0f),
           (long)(g_cl_fb_alpha * 100.0f),
           (unsigned)g_cl_warmup_ticks,
           (long)(g_cl_iref_max * 100.0f));
}

/*=============================================================================
 * 复位
 *=============================================================================*/
void Foc_CurrentLoop_Reset(void)
{
    if (!s_ready) {
        return;
    }
    s_pid_d.ops->reset(&s_priv_d);
    s_pid_q.ops->reset(&s_priv_q);

    s_id_ref_ramped = 0.0f;
    s_iq_ref_ramped = 0.0f;
    s_limit_reason  = CL_LIMIT_NONE;
    s_sat_cnt       = 0u;
    s_vmag_raw      = 0.0f;
    s_vq_prev       = 0.0f;
    s_id_f          = 0.0f;
    s_iq_f          = 0.0f;
    s_warmup_cnt    = 0u;   /* 重新预热，用当前 g_cl_warmup_ticks */

    /* 进 CURRENT 后在 50/50 零矢量窗口重校三相电流零偏，
     * 消除上电（PWM 关闭）校准与运行中采样点之间的偏置差。
     * 转子还在明显转动时不重校，避免把真实电流当成零偏。 */
    s_offcal_done   = 1u;
    s_offcal_ticks  = 0u;
    if (fabsf(hw_encoder_get_speed_rpm()) < 50.0f) {
        s_offcal_done = 0u;
        Motor_Sampling_StartOffsetCal();
    }
}

/*=============================================================================
 * 目标 / 参数 / 上限
 *=============================================================================*/
void Foc_CurrentLoop_SetRef(float id_ref_a, float iq_ref_a)
{
    float lim = g_cl_iref_max;

    if (lim < 0.0f) {
        lim = 0.0f;   /* 防误设负数 */
    }
    s_id_ref = cl_clampf(id_ref_a, -lim, lim);
    s_iq_ref = cl_clampf(iq_ref_a, -lim, lim);
}

void Foc_CurrentLoop_SetGains(float kp, float ki, float kd)
{
    if (!s_ready) {
        return;
    }
    (void)s_pid_d.ops->set_param(&s_priv_d, kp, ki, kd);
    (void)s_pid_q.ops->set_param(&s_priv_q, kp, ki, kd);
}

void Foc_CurrentLoop_SetITermMax(float v)
{
    if (v < 0.0f) {
        v = 0.0f;
    }
    g_cl_iterm_max_watch = v;
    if (!s_ready) {
        return;
    }
    (void)s_pid_d.ops->set_iterm_limit(&s_priv_d, v);
    (void)s_pid_q.ops->set_iterm_limit(&s_priv_q, v);
}

void Foc_CurrentLoop_SetVmax(float vmax_v)
{
    if (vmax_v < 1.0f) {
        return;
    }
    s_vmax = vmax_v;
    cl_refresh_limits();
}

/*=============================================================================
 * 每拍计算
 *=============================================================================*/
void Foc_CurrentLoop_Step(float *vd, float *vq, float *theta_e)
{
    float fb_d, fb_q, th;
    float out_d, out_q;
    float vd_raw, vq_raw;
    cl_limit_reason_t reason = CL_LIMIT_NONE;

    if ((vd == NULL) || (vq == NULL) || (theta_e == NULL)) {
        return;
    }

    if (!s_ready) {
        *vd = 0.0f;
        *vq = 0.0f;
        *theta_e = 0.0f;
        return;
    }

    th = hw_encoder_get_elec_angle_rad(CFG_MOTOR_POLE_PAIRS);

    /* ---- 进入预热：零矢量 + PWM 在线零偏校准 ---- */
    if ((s_warmup_cnt < g_cl_warmup_ticks) || (!s_offcal_done)) {
        if (s_warmup_cnt < g_cl_warmup_ticks) {
            s_warmup_cnt++;
        }
        s_vq_prev = 0.0f;
        *vd = 0.0f;
        *vq = 0.0f;
        *theta_e = th;

        /* 50/50 零矢量期间累计原始采样，满 2000 点后静默更新零偏 */
        if (!s_offcal_done) {
            s_offcal_ticks++;
            if (Motor_Sampling_ApplyOffsetCalSilent()) {
                s_offcal_done = 1u;
            } else if (s_offcal_ticks >= CL_OFFCAL_TIMEOUT_TICKS) {
                s_offcal_done = 1u;   /* 超时保留上电校准值 */
            }
        }
        return;
    }

    /* ---- 反馈 ---- */
    fb_d = g_fc_id;
    fb_q = g_fc_iq;

    /* id/iq 反馈 EMA（参考工程 FOC_CUR_FB_ALPHA=0.3，1 = 关闭） */
    {
        float a = g_cl_fb_alpha;

        if (a < 0.0f) a = 0.0f;
        if (a > 1.0f) a = 1.0f;
        s_id_f += a * (fb_d - s_id_f);
        s_iq_f += a * (fb_q - s_iq_f);
        fb_d = s_id_f;
        fb_q = s_iq_f;
    }

    /* ---- ref 斜坡：目标 -> 实际，防电流阶跃冲击 ---- */
    s_id_ref_ramped = cl_ramp(s_id_ref, s_id_ref_ramped, g_cl_ref_slew);
    s_iq_ref_ramped = cl_ramp(s_iq_ref, s_iq_ref_ramped, g_cl_ref_slew);

    /* ---- PID 计算（用斜坡后的 ref，前馈关闭）---- */
    out_d = s_pid_d.ops->calc(&s_priv_d, fb_d, s_id_ref_ramped, 0.0f, 0.0f, CL_DT_S);
    out_q = s_pid_q.ops->calc(&s_priv_q, fb_q, s_iq_ref_ramped, 0.0f, 0.0f, CL_DT_S);

    /* ---- 未限幅输出 -> 合成幅值 -> 物理电压判断 ---- */
    vd_raw = s_pid_d.ops->get_raw_output(&s_priv_d);
    vq_raw = s_pid_q.ops->get_raw_output(&s_priv_q);
    s_vmag_raw = sqrtf(vd_raw * vd_raw + vq_raw * vq_raw);

    /* ---- 受限判定（优先级：保护 > 电压 > 自身限幅）---- */
    if (g_fp_state != FP_STATE_NORMAL) {
        reason = CL_LIMIT_PROTECT;
    } else if (s_vmag_raw > s_vmax) {
        reason = CL_LIMIT_VOLTAGE;
    } else if ((out_d >= s_vlim) || (out_d <= -s_vlim) ||
               (out_q >= s_vlim) || (out_q <= -s_vlim)) {
        reason = CL_LIMIT_SELF;
    } else {
        reason = CL_LIMIT_NONE;
    }

    s_limit_reason = reason;

    if (reason != CL_LIMIT_NONE) {
        if (s_sat_cnt < 0xFFFFu) {
            s_sat_cnt++;
        }
    } else {
        s_sat_cnt = 0u;
    }

    /* ---- 输出斜率限制（防机械冲击）---- */
    out_q = cl_slew(out_q, s_vq_prev, g_cl_vq_slew);
    s_vq_prev = out_q;

    *vd      = out_d;
    *vq      = out_q;
    *theta_e = th;
}

/*=============================================================================
 * 查询
 *=============================================================================*/
bool Foc_CurrentLoop_IsReady(void)
{
    return s_ready;
}

bool Foc_CurrentLoop_IsLimited(void)
{
    return (s_limit_reason != CL_LIMIT_NONE);
}

cl_limit_reason_t Foc_CurrentLoop_GetLimitReason(void)
{
    return s_limit_reason;
}

uint16_t Foc_CurrentLoop_GetSatCnt(void)
{
    return s_sat_cnt;
}

float Foc_CurrentLoop_GetVmagRaw(void)
{
    return s_vmag_raw;
}

float Foc_CurrentLoop_GetVmax(void)
{
    return s_vmax;
}

float Foc_CurrentLoop_GetIdRefRamped(void)
{
    return s_id_ref_ramped;
}

float Foc_CurrentLoop_GetIqRefRamped(void)
{
    return s_iq_ref_ramped;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
