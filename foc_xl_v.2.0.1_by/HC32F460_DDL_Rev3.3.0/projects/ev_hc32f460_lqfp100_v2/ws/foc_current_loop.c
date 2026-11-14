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
#include "pid_fx.h"
#include "foc_fx.h"
#include "cfg_fixed.h"
#include "hw_encoder.h"
#include "cfg_motor.h"
#include "clarke_park.h"     /* ONE_DIV_SQRT3_F */
#include "foc_protect.h"     /* g_fp_state, FP_STATE_NORMAL */
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
volatile uint8_t  g_cl_ff_enable   = 0u;
volatile float    g_cl_ff_rs_ohm   = CL_FF_RS_OHM_INIT;
volatile float    g_cl_ff_ls_uh    = CL_FF_LS_UH_INIT;
volatile float    g_cl_ff_flux_vs  = CL_FF_FLUX_VS_INIT;
volatile float    g_cl_we_rad_s    = 0.0f;
volatile float    g_cl_vd_ff       = 0.0f;
volatile float    g_cl_vq_ff       = 0.0f;
volatile uint16_t g_cl_warmup_ticks = CL_WARMUP_INIT;
volatile float    g_cl_iref_max     = CL_IREF_MAX_INIT;
volatile float    g_cl_vmax_watch   = CL_VMAX_INIT;
volatile float    g_cl_back_gain    = CL_BACK_GAIN_INIT;
volatile uint32_t g_cl_aw_mask      = CL_AW_MASK_INIT;
volatile float    g_cl_aw_cond_thr  = CL_AW_COND_THR_INIT;

/*---------------------------------------------------------------------------
 * 编译期常量（不需要在线调的）
 *---------------------------------------------------------------------------*/
#define CL_DT_S              (1.0f / 20000.0f)   /* 20kHz ISR 周期 */
#define CL_VLIM_RATIO        1.0f                /* PID 限幅 = vmax（参考工程 ±3.5V） */
#define CL_TWO_PI            6.28318530f

/* 受限告警阈值：4000 拍 = 200ms */
#define CL_SAT_WARN_TICKS    4000u

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

/* 当前是否处于零矢量输出（短路制动） */
static bool s_zero_out = false;

/* id/iq 反馈 EMA 状态 */
static float s_id_f = 0.0f;
static float s_iq_f = 0.0f;

/* 进入预热计数 */
static uint16_t s_warmup_cnt = 0u;

#if (FOC_USE_FIXED != 0)
/*---------------------------------------------------------------------------
 * 定点路径状态
 *---------------------------------------------------------------------------*/
static PidFx_t  s_pidfx_d;
static PidFx_t  s_pidfx_q;

static int16_t  s_id_f15       = 0;
static int16_t  s_iq_f15       = 0;
static int16_t  s_vq_prev15    = 0;
static int16_t  s_we_f15       = 0;   /* 滤波后的电角速度 Q15（1.0 = FX_WE_BASE_RAD_S） */
static int16_t  s_vlim15       = 0;
static int16_t  s_iterm_max15  = 0;

/* 浮点增益缓存，Watch 改完后换算成 Q15 */
static float    s_kp           = CL_KP_INIT;
static float    s_ki           = CL_KI_INIT;
static float    s_kd           = CL_KD_INIT;
static float    s_iterm_max_v  = CL_ITERM_MAX_INIT;
#endif

/*---------------------------------------------------------------------------
 * L5 反馈镜像
 *---------------------------------------------------------------------------*/
extern volatile float g_fc_id;
extern volatile float g_fc_iq;
extern volatile int16_t g_fc_id_q15;
extern volatile int16_t g_fc_iq_q15;
extern volatile int32_t g_fc_enc_delta;

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
    (void)s_pid_d.ops->set_limit(&s_priv_d, -s_vlim, s_vlim, g_cl_back_gain);
    (void)s_pid_q.ops->set_limit(&s_priv_q, -s_vlim, s_vlim, g_cl_back_gain);
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

/* 积分项限幅只作用于 PID 实例，不改写 g_cl_iterm_max_watch */
static void cl_set_iterm(float v)
{
    if (v < 0.0f) {
        v = 0.0f;
    }
    if (!s_ready) {
        return;
    }
    (void)s_pid_d.ops->set_iterm_limit(&s_priv_d, v);
    (void)s_pid_q.ops->set_iterm_limit(&s_priv_q, v);
}

#if (FOC_USE_FIXED != 0)
/*---------------------------------------------------------------------------
 * 定点参数换算
 *   kp: V/A -> 标幺 Q15
 *   ki: V/A/s -> 每拍增量 Q15
 *   kd: V/(A/s) -> 每拍微分 Q15（当前 kd=0）
 *---------------------------------------------------------------------------*/
static void cl_fx_apply_gains(void)
{
    int32_t kp15 = FX_KP_CUR_Q15(s_kp);
    int32_t ki15 = FX_KI_CUR_TICK_Q15(s_ki);
    int32_t kd15 = (int32_t)(s_kd * FX_CTRL_HZ * FX_I_BASE_A / FX_V_BASE_V * 32768.0f);

    PidFx_SetGains(&s_pidfx_d, kp15, ki15, kd15);
    PidFx_SetGains(&s_pidfx_q, kp15, ki15, kd15);
    PidFx_SetDiffAlpha(&s_pidfx_d, (int16_t)(CL_DIFF_ALPHA_INIT * 32768.0f));
    PidFx_SetDiffAlpha(&s_pidfx_q, (int16_t)(CL_DIFF_ALPHA_INIT * 32768.0f));
}

static void cl_fx_apply_limits(void)
{
    s_vlim15      = Fx_Sat16(FX_VOL_TO_Q15(s_vlim));
    s_iterm_max15 = Fx_Sat16(FX_VOL_TO_Q15(s_iterm_max_v));

    PidFx_SetLimits(&s_pidfx_d, (int16_t)(-s_vlim15), s_vlim15);
    PidFx_SetLimits(&s_pidfx_q, (int16_t)(-s_vlim15), s_vlim15);

    if (s_iterm_max15 > 0) {
        PidFx_SetITermMax(&s_pidfx_d, s_iterm_max15);
        PidFx_SetITermMax(&s_pidfx_q, s_iterm_max15);
    } else {
        PidFx_SetITermMax(&s_pidfx_d, 0);
        PidFx_SetITermMax(&s_pidfx_q, 0);
    }
}
#endif

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

    (void)s_pid_d.ops->set_limit(&s_priv_d, -s_vlim, s_vlim, g_cl_back_gain);
    (void)s_pid_q.ops->set_limit(&s_priv_q, -s_vlim, s_vlim, g_cl_back_gain);

    (void)s_pid_d.ops->set_iterm_limit(&s_priv_d, g_cl_iterm_max_watch);
    (void)s_pid_q.ops->set_iterm_limit(&s_priv_q, g_cl_iterm_max_watch);

    (void)s_pid_d.ops->set_aw_mode(&s_priv_d, g_cl_aw_mask);
    (void)s_pid_q.ops->set_aw_mode(&s_priv_q, g_cl_aw_mask);
    (void)s_pid_d.ops->set_aw_cond_thr(&s_priv_d, g_cl_aw_cond_thr);
    (void)s_pid_q.ops->set_aw_cond_thr(&s_priv_q, g_cl_aw_cond_thr);

    (void)s_pid_d.ops->set_diff_filter(&s_priv_d, CL_DIFF_ALPHA_INIT);
    (void)s_pid_q.ops->set_diff_filter(&s_priv_q, CL_DIFF_ALPHA_INIT);

    (void)s_pid_d.ops->set_feedforward(&s_priv_d, 0.0f, 0.0f, false, false);
    (void)s_pid_q.ops->set_feedforward(&s_priv_q, 0.0f, 0.0f, false, false);

#if (FOC_USE_FIXED != 0)
    PidFx_Init(&s_pidfx_d);
    PidFx_Init(&s_pidfx_q);

    s_kp          = CL_KP_INIT;
    s_ki          = CL_KI_INIT;
    s_kd          = CL_KD_INIT;
    s_iterm_max_v = g_cl_iterm_max_watch;
    s_id_f15      = 0;
    s_iq_f15      = 0;
    s_vq_prev15   = 0;
    s_we_f15      = 0;

    cl_fx_apply_gains();
    cl_fx_apply_limits();
#endif

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
    s_zero_out      = false;

#if (FOC_USE_FIXED != 0)
    PidFx_Reset(&s_pidfx_d);
    PidFx_Reset(&s_pidfx_q);
    s_id_f15    = 0;
    s_iq_f15    = 0;
    s_vq_prev15 = 0;
    s_we_f15    = 0;
#endif
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

void Foc_CurrentLoop_SetRefDirect(float id_ref_a, float iq_ref_a)
{
    float lim = g_cl_iref_max;

    if (lim < 0.0f) {
        lim = 0.0f;
    }
    s_id_ref        = cl_clampf(id_ref_a, -lim, lim);
    s_iq_ref        = cl_clampf(iq_ref_a, -lim, lim);
    s_id_ref_ramped = s_id_ref;
    s_iq_ref_ramped = s_iq_ref;
}

void Foc_CurrentLoop_SetGains(float kp, float ki, float kd)
{
    if (!s_ready) {
        return;
    }
    (void)s_pid_d.ops->set_param(&s_priv_d, kp, ki, kd);
    (void)s_pid_q.ops->set_param(&s_priv_q, kp, ki, kd);

#if (FOC_USE_FIXED != 0)
    s_kp = kp;
    s_ki = ki;
    s_kd = kd;
    cl_fx_apply_gains();
#endif
}

void Foc_CurrentLoop_SetITermMax(float v)
{
    cl_set_iterm(v);

#if (FOC_USE_FIXED != 0)
    s_iterm_max_v = (v < 0.0f) ? 0.0f : v;
    cl_fx_apply_limits();
#endif
}

void Foc_CurrentLoop_SetAntiWindup(uint32_t mask, float cond_thr)
{
    if (!s_ready) {
        return;
    }
    (void)s_pid_d.ops->set_aw_mode(&s_priv_d, mask);
    (void)s_pid_q.ops->set_aw_mode(&s_priv_q, mask);
    (void)s_pid_d.ops->set_aw_cond_thr(&s_priv_d, cond_thr);
    (void)s_pid_q.ops->set_aw_cond_thr(&s_priv_q, cond_thr);
}

void Foc_CurrentLoop_SetBackGain(float v)
{
    if (v < 0.0f) {
        v = 0.0f;
    }
    if (v > 1.0f) {
        v = 1.0f;
    }
    g_cl_back_gain = v;
    cl_refresh_limits();
}

void Foc_CurrentLoop_SetFF(uint8_t en)
{
    g_cl_ff_enable = (en != 0u) ? 1u : 0u;
}

void Foc_CurrentLoop_SetVmax(float vmax_v)
{
    if (vmax_v < 1.0f) {
        return;
    }
    s_vmax = vmax_v;
    cl_refresh_limits();

#if (FOC_USE_FIXED != 0)
    cl_fx_apply_limits();
#endif
}

void Foc_CurrentLoop_ApplyParams(float kp, float ki, float kd,
                                 float vmax_v, float iterm_max_v, float fb_alpha)
{
    if (!s_ready) {
        return;
    }

    if (vmax_v < 1.0f) {
        vmax_v = CL_VMAX_INIT;
    }
    s_vmax = vmax_v;
    cl_refresh_limits();

    (void)s_pid_d.ops->set_param(&s_priv_d, kp, ki, kd);
    (void)s_pid_q.ops->set_param(&s_priv_q, kp, ki, kd);

    cl_set_iterm(iterm_max_v);

    if (fb_alpha < 0.0f) fb_alpha = 0.0f;
    if (fb_alpha > 1.0f) fb_alpha = 1.0f;
    g_cl_fb_alpha = fb_alpha;

#if (FOC_USE_FIXED != 0)
    s_kp          = kp;
    s_ki          = ki;
    s_kd          = kd;
    s_iterm_max_v = (iterm_max_v < 0.0f) ? 0.0f : iterm_max_v;
    cl_fx_apply_gains();
    cl_fx_apply_limits();
#endif
}

/*=============================================================================
 * 每拍计算
 *=============================================================================*/
#if (FOC_USE_FIXED != 0)
/*---------------------------------------------------------------------------
 * 定点路径：反馈、PI、mode8 前馈、限幅全部用 Q15
 *   对外仍返回伏特和 rad，保持 L5 接口不变
 *---------------------------------------------------------------------------*/
static void cl_step_fixed(float *vd, float *vq, float *theta_e)
{
    float    th;
    int16_t  fb_d, fb_q;
    int16_t  ref_d, ref_q;
    int32_t  out_d, out_q;
    int32_t  vd_tot, vq_tot;
    int32_t  vd_ff = 0;
    int32_t  vq_ff = 0;
    int32_t  vlim  = (int32_t)s_vlim15;
    uint32_t vmag_sq;
    cl_limit_reason_t reason = CL_LIMIT_NONE;

    if ((vd == NULL) || (vq == NULL) || (theta_e == NULL)) {
        return;
    }

    s_zero_out = false;

    if (!s_ready) {
        *vd = 0.0f;
        *vq = 0.0f;
        *theta_e = 0.0f;
        return;
    }

    th = hw_encoder_get_elec_angle_rad(CFG_MOTOR_POLE_PAIRS);

    /* ---- 预热 ---- */
    if (s_warmup_cnt < g_cl_warmup_ticks) {
        s_warmup_cnt++;
        s_vq_prev15 = 0;
        *vd = 0.0f;
        *vq = 0.0f;
        *theta_e = th;
        return;
    }

    /* ---- 反馈 + EMA ---- */
    fb_d = g_fc_id_q15;
    fb_q = g_fc_iq_q15;
    {
        int32_t a = FX_FLOAT_TO_Q15(g_cl_fb_alpha);

        if (a < 0) a = 0;
        if (a > 32767) a = 32767;

        if (a >= 32767) {
            s_id_f15 = fb_d;
            s_iq_f15 = fb_q;
        } else {
            s_id_f15 = Fx_Sat16((int32_t)s_id_f15 +
                                (((int32_t)fb_d - (int32_t)s_id_f15) * a >> 15));
            s_iq_f15 = Fx_Sat16((int32_t)s_iq_f15 +
                                (((int32_t)fb_q - (int32_t)s_iq_f15) * a >> 15));
        }

        fb_d = s_id_f15;
        fb_q = s_iq_f15;
    }

    /* ---- ref 斜坡（保持浮点，防阶跃冲击），再转 Q15 ---- */
    s_id_ref_ramped = cl_ramp(s_id_ref, s_id_ref_ramped, g_cl_ref_slew);
    s_iq_ref_ramped = cl_ramp(s_iq_ref, s_iq_ref_ramped, g_cl_ref_slew);

    ref_d = Fx_Sat16(FX_CUR_TO_Q15(s_id_ref_ramped));
    ref_q = Fx_Sat16(FX_CUR_TO_Q15(s_iq_ref_ramped));

    /* ---- 定点 PI ---- */
    out_d = PidFx_Calc(&s_pidfx_d, ref_d, fb_d);
    out_q = PidFx_Calc(&s_pidfx_q, ref_q, fb_q);

    /* ---- mode8：dq 解耦 + 反电动势前馈（定点）---- */
    g_cl_we_rad_s = 0.0f;
    g_cl_vd_ff    = 0.0f;
    g_cl_vq_ff    = 0.0f;
    if (g_cl_ff_enable) {
        int32_t we_raw = (int32_t)g_fc_enc_delta * 1200;   /* Q15，1.0 = FX_WE_BASE */
        int16_t we15;
        int32_t rs_q16, ls_q16, fl_q16, t_ls;

        if (we_raw > 32767)  we_raw = 32767;
        if (we_raw < -32768) we_raw = -32768;

        s_we_f15 = (int16_t)((int32_t)s_we_f15 + ((we_raw - (int32_t)s_we_f15) >> 5));
        we15 = s_we_f15;

        rs_q16 = (int32_t)(g_cl_ff_rs_ohm / FX_Z_BASE_OHM * 65536.0f);
        ls_q16 = (int32_t)(g_cl_ff_ls_uh * 1e-6f / FX_L_BASE_H * 65536.0f);
        fl_q16 = (int32_t)(g_cl_ff_flux_vs / FX_FLUX_BASE_VS * 65536.0f);

        t_ls = (int32_t)(((int64_t)we15 * (int64_t)ls_q16) >> 16);   /* Q15: we*Ls */

        vd_ff = Fx_Sat16((int32_t)(-((int64_t)t_ls * (int64_t)fb_q) >> 15));
        vq_ff = Fx_Sat16((int32_t)(((int64_t)t_ls * (int64_t)fb_d) >> 15) +
                         (int32_t)(((int64_t)we15 * (int64_t)fl_q16) >> 16) +
                         (int32_t)(((int64_t)rs_q16 * (int64_t)ref_q) >> 16));

        g_cl_we_rad_s = FX_Q15_TO_FLOAT(we15) * FX_WE_BASE_RAD_S;
        g_cl_vd_ff    = FX_Q15_TO_VOL((int16_t)vd_ff);
        g_cl_vq_ff    = FX_Q15_TO_VOL((int16_t)vq_ff);
    } else {
        s_we_f15 = 0;
    }

    vd_tot = out_d + vd_ff;
    vq_tot = out_q + vq_ff;

    /* 先钳到 Q15 范围，避免平方和溢出 */
    vd_tot = Fx_Sat16(vd_tot);
    vq_tot = Fx_Sat16(vq_tot);

    vmag_sq = (uint32_t)(vd_tot * vd_tot) + (uint32_t)(vq_tot * vq_tot);

    /* 幅值只给 Watch 看：用 FPU sqrtf（约 15 周期），
     * 不再每拍跑一次 16 轮整数 isqrt（约 100+ 周期） */
    s_vmag_raw = sqrtf((float)vmag_sq) * (FX_V_BASE_V / 32768.0f);

    /* ---- 受限判定（保护 > 电压 > 自身限幅）---- */
    if (g_fp_state != FP_STATE_NORMAL) {
        reason = CL_LIMIT_PROTECT;
    } else if (vmag_sq > (uint32_t)(vlim * vlim)) {
        reason = CL_LIMIT_VOLTAGE;
    } else if ((out_d >= vlim) || (out_d <= -vlim) ||
               (out_q >= vlim) || (out_q <= -vlim)) {
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

    /* ---- vq 斜率限制 ---- */
    {
        int32_t slew15 = FX_VOL_TO_Q15(g_cl_vq_slew);

        if (slew15 > 0) {
            if (vq_tot > (int32_t)s_vq_prev15 + slew15) {
                vq_tot = (int32_t)s_vq_prev15 + slew15;
            } else if (vq_tot < (int32_t)s_vq_prev15 - slew15) {
                vq_tot = (int32_t)s_vq_prev15 - slew15;
            }
        }
        s_vq_prev15 = Fx_Sat16(vq_tot);
        vq_tot = s_vq_prev15;
    }

    /* ---- 最终矢量限幅 ---- */
    vmag_sq = (uint32_t)(vd_tot * vd_tot) + (uint32_t)(vq_tot * vq_tot);
    if (vmag_sq > (uint32_t)(vlim * vlim)) {
        uint32_t vmag = Fx_ISqrt32(vmag_sq);

        if (vmag > 0u) {
            int32_t k15 = (vlim << 15) / (int32_t)vmag;

            vd_tot = (vd_tot * k15) >> 15;
            vq_tot = (vq_tot * k15) >> 15;
        }
    }

    *vd      = FX_Q15_TO_VOL(Fx_Sat16(vd_tot));
    *vq      = FX_Q15_TO_VOL(Fx_Sat16(vq_tot));
    *theta_e = th;
}
#endif

void Foc_CurrentLoop_Step(float *vd, float *vq, float *theta_e)
{
#if (FOC_USE_FIXED != 0)
    cl_step_fixed(vd, vq, theta_e);
#else
    float fb_d, fb_q, th;
    float out_d, out_q;
    float vd_tot, vq_tot;
    float vd_ff = 0.0f, vq_ff = 0.0f;
    cl_limit_reason_t reason = CL_LIMIT_NONE;

    if ((vd == NULL) || (vq == NULL) || (theta_e == NULL)) {
        return;
    }

    s_zero_out = false;

    if (!s_ready) {
        *vd = 0.0f;
        *vq = 0.0f;
        *theta_e = 0.0f;
        return;
    }

    th = hw_encoder_get_elec_angle_rad(CFG_MOTOR_POLE_PAIRS);

    /* ---- 进入预热：零矢量，等待 PWM/反馈稳定 ---- */
    if (s_warmup_cnt < g_cl_warmup_ticks) {
        s_warmup_cnt++;
        s_vq_prev = 0.0f;
        *vd = 0.0f;
        *vq = 0.0f;
        *theta_e = th;
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

    /* ---- PID 计算（PID 自身前馈关闭；dq 前馈在下面单独加）---- */
    out_d = s_pid_d.ops->calc(&s_priv_d, fb_d, s_id_ref_ramped, 0.0f, 0.0f, CL_DT_S);
    out_q = s_pid_q.ops->calc(&s_priv_q, fb_q, s_iq_ref_ramped, 0.0f, 0.0f, CL_DT_S);

    /* ---- dq 解耦 + 反电动势前馈（mode8，Watch 可开关）---- */
    g_cl_we_rad_s = 0.0f;
    g_cl_vd_ff    = 0.0f;
    g_cl_vq_ff    = 0.0f;
    if (g_cl_ff_enable) {
        float rpm  = hw_encoder_get_speed_rpm();
        float we   = rpm * (CL_TWO_PI * (float)CFG_MOTOR_POLE_PAIRS / 60.0f);
        float ls   = g_cl_ff_ls_uh * 1e-6f;
        float rs   = g_cl_ff_rs_ohm;
        float flux = g_cl_ff_flux_vs;

        vd_ff = -we * ls * fb_q;
        vq_ff =  we * ls * fb_d + we * flux + rs * s_iq_ref_ramped;

        g_cl_we_rad_s = we;
        g_cl_vd_ff    = vd_ff;
        g_cl_vq_ff    = vq_ff;
    }

    vd_tot = out_d + vd_ff;
    vq_tot = out_q + vq_ff;

    /* ---- 未限幅总输出 -> 合成幅值 -> 物理电压判断 ---- */
    s_vmag_raw = sqrtf(vd_tot * vd_tot + vq_tot * vq_tot);

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

    /* ---- 输出斜率限制 + 最终矢量限幅 ---- */
    vq_tot = cl_slew(vq_tot, s_vq_prev, g_cl_vq_slew);
    s_vq_prev = vq_tot;

    {
        float vmag = sqrtf(vd_tot * vd_tot + vq_tot * vq_tot);

        if (vmag > s_vlim) {
            float k = s_vlim / vmag;

            vd_tot *= k;
            vq_tot *= k;
        }
    }

    *vd      = vd_tot;
    *vq      = vq_tot;
    *theta_e = th;
#endif
}

/*=============================================================================
 * 零矢量输出（短路制动）
 *=============================================================================*/
void Foc_CurrentLoop_OutputZeroVector(float *vd, float *vq, float *theta_e)
{
    if ((vd == NULL) || (vq == NULL) || (theta_e == NULL)) {
        return;
    }

    if (!s_zero_out) {
        s_zero_out = true;
        if (s_ready) {
            s_pid_d.ops->reset(&s_priv_d);
            s_pid_q.ops->reset(&s_priv_q);
            s_id_ref_ramped = 0.0f;
            s_iq_ref_ramped = 0.0f;
            s_vq_prev       = 0.0f;
            s_limit_reason  = CL_LIMIT_NONE;
            s_sat_cnt       = 0u;
            s_vmag_raw      = 0.0f;
        }
    }

    *vd = 0.0f;
    *vq = 0.0f;
    *theta_e = hw_encoder_get_elec_angle_rad(CFG_MOTOR_POLE_PAIRS);
}

/*=============================================================================
 * 查询
 *=============================================================================*/
bool Foc_CurrentLoop_IsReady(void)
{
    return s_ready;
}

bool Foc_CurrentLoop_IsActive(void)
{
    if (!s_ready) {
        return false;
    }
    return (s_warmup_cnt >= g_cl_warmup_ticks);
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
