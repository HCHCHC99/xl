/*=============================================================================
 * encoder_calib.c  —  编码器电角度标定层实现
 *
 * mode 3 单次校准：
 *   StartAlign(): 置 g_align_active/resume_output/给 50/50/50 零矢量切 ZERO
 *   Task20k(): EC_ZERO 时每拍累加 g_fc_iu/iv/iw，skip 200 + avg 400 拍
 *   Task1ms(): BETA/ALPHA 保持计时，到点取值；ALPHA 结束写 offset/dir
 *              置 g_motor_is_aligned=1，evt=DONE_OK
 *
 *   矢量角度约定：
 *     BETA  = 90° 电角度矢量（s_th1 = π/2）
 *     ALPHA = 0°  电角度矢量（s_th2 = 0）
 *     物理：BETA(90°) -> ALPHA(0°)，电角度减小 90°
 *
 *   dir 符号约定（与 hw_encoder.c 注释一致）：
 *     moved = alpha_hw - beta_hw
 *     moved > 0 : 电角度减小对应计数增 -> 计数增=电角度减 -> dir = -1
 *     moved < 0 : 电角度减小对应计数减 -> 计数增=电角度增 -> dir = +1
 *
 * mode 5 扫角：
 *   StartSweep(): 同 StartAlign 但不走 ZERO，直接给第一段矢量
 *   每角度两段（BETA=起始角, ALPHA=起始角-90°，与 mode 3 同为电角度减小）
 *   后记 g_sweep_*，切下一角度，4 个跑完 DONE
 *   两个模式的 moved 符号含义因此统一：moved > 0 => dir = -1
 *
 * 说明：ec_precompute_duty() 不再自行 +90°，s_th1/s_th2 由调用方
 *       给定"两个绝对电角度"，mode3 与 mode5 语义差异全部上移到调用点
 *
 * 本版新增诊断打印：
 *   abort 时打印 fp_state/fault/三相电流/abort 前的状态机状态
 *=============================================================================*/

#include "encoder_calib.h"
#include "hw_encoder.h"
#include "hw_tmr4_pwm.h"
#include "svpwm.h"
#include "foc_control.h"     /* g_align_active, g_fc_iu/iv/iw */
#include "foc_protect.h"     /* g_fp_state */
#include "foc_sincos.h"
#include "cfg_motor.h"
#include "cfg_sampling.h"
#include "rtt_log.h"

/*=============================================================================
 * 观测量 / 调试可写量
 *=============================================================================*/
volatile uint8_t g_motor_is_aligned = 0u;

volatile float    g_align_volt     = 1.3f;
volatile float    g_align_theta    = 1.5707963f;
volatile uint32_t g_align_hold_ms  = 200u;
volatile uint32_t g_align_ramp_ms  = 500u;   /* mode7 斜坡时间 ms */
volatile uint8_t  g_a7_done        = 0u;     /* mode7 完成标志 */

volatile int32_t  g_align_zero_u_ma = 0;
volatile int32_t  g_align_zero_v_ma = 0;
volatile int32_t  g_align_zero_w_ma = 0;

volatile int32_t  g_align_beta_hw   = 0;
volatile int32_t  g_align_alpha_hw  = 0;
volatile int32_t  g_align_moved     = 0;
volatile int32_t  g_align_offset    = 0;

volatile uint8_t  g_align_evt       = EC_EVT_NONE;

volatile int32_t  g_sweep_beta[4]  = { 0, 0, 0, 0 };
volatile int32_t  g_sweep_alpha[4] = { 0, 0, 0, 0 };
volatile int32_t  g_sweep_moved[4] = { 0, 0, 0, 0 };
volatile int32_t  g_sweep_theta[4] = { 0, 1570, 3141, 4712 };
volatile uint8_t  g_sweep_idx      = 0u;
volatile uint8_t  g_sweep_done     = 0u;

/* mode5 每段电流平均值（mA）
 *   [idx][0..2] = BETA  iu, iv, iw
 *   [idx][3..5] = ALPHA iu, iv, iw
 */
volatile int32_t  g_sweep_i_ma[4][6] = { { 0 } };

/* mode6 一圈计数测量（对齐 -> 强拖 pp*360° -> 再对齐） */
volatile float    g_rev_volt       = 1.5f;   /* 强拖电压 V */
volatile float    g_rev_freq_hz    = 1.0f;   /* 强拖电角度频率 Hz */
volatile uint32_t g_rev_align_ms   = 1000u;  /* 首尾对齐保持 ms */
volatile uint32_t g_rev_ramp_ms    = 500u;   /* 起步加速斜坡 ms */
volatile int32_t  g_rev_start_cnt  = 0;
volatile int32_t  g_rev_end_cnt    = 0;
volatile int32_t  g_rev_delta      = 0;
volatile float    g_rev_pp_meas    = 0.0f;   /* 折算极对数 = CPR*pp_cfg/|delta| */
volatile int32_t  g_rev_cyc_min    = 0;      /* 每个电周期计数增量最小值 */
volatile int32_t  g_rev_cyc_max    = 0;      /* 每个电周期计数增量最大值 */
volatile uint8_t  g_rev_done       = 0u;
volatile uint8_t  g_rev_slip       = 0u;     /* 1 = 各电周期增量差异过大，疑似失步 */

/*=============================================================================
 * 状态机
 *=============================================================================*/
typedef enum {
    EC_IDLE = 0,
    EC_ZERO,
    EC_BETA,
    EC_ALPHA,
    EC_REV_ALIGN0,
    EC_REV_DRAG,
    EC_REV_ALIGN1,
    EC_A7_RAMP_UP,
    EC_A7_HOLD_BETA,
    EC_A7_RAMP_DN,
    EC_A7_HOLD_ALPHA,
    EC_DONE,
} ec_state_t;

static volatile ec_state_t s_ec_state = EC_IDLE;
static volatile uint8_t    s_align_ok = 0u;
static volatile uint8_t    s_sweep_en = 0u;
static volatile uint8_t    s_rev_en   = 0u;
static volatile uint8_t    s_a7_en    = 0u;

#define EC_TWO_PI   6.28318530f
#define EC_PI_2     1.5707963f

/* mode7 斜坡对齐状态 */
static float    s_a7_theta     = 0.0f;
static uint32_t s_a7_ramp_cnt  = 0u;
static uint32_t s_a7_ramp_ms_l = 500u;
static uint32_t s_a7_hold_ms_l = 500u;
static float    s_a7_volt_l    = 1.3f;

/* mode6 强拖状态 */
static float    s_rev_volt_l    = 1.5f;
static float    s_rev_f_l       = 1.0f;
static float    s_rev_theta     = 0.0f;
static float    s_rev_total     = 0.0f;
static uint32_t s_rev_ramp_cnt  = 0u;
static int32_t  s_rev_last_cnt  = 0;
static uint32_t s_rev_cyc_idx   = 0u;
static uint8_t  s_rev_cyc_n     = 0u;
static int32_t  s_rev_cyc_min_l = 0;
static int32_t  s_rev_cyc_max_l = 0;

/* ZERO 采样 */
#define EC_ZERO_SKIP   200u
#define EC_ZERO_AVG    400u
static uint32_t s_zero_cnt = 0u;
static int64_t  s_zero_sum_u = 0;
static int64_t  s_zero_sum_v = 0;
static int64_t  s_zero_sum_w = 0;

/* BETA/ALPHA 计时与取值 */
static volatile uint32_t s_hold_cnt = 0u;
static uint32_t s_hold_ms_l = 200u;

/* mode5 每段电流统计：20kHz 累加，前 100ms 跳过（等电流/转子稳定） */
#define EC_SEG_SKIP_TICKS   2000u
static float    s_seg_sum_u = 0.0f;
static float    s_seg_sum_v = 0.0f;
static float    s_seg_sum_w = 0.0f;
static uint32_t s_seg_cnt   = 0u;
static uint32_t s_seg_tick  = 0u;

static int32_t s_beta_hw_l  = 0;
static int32_t s_alpha_hw_l = 0;

/* 预计算 duty */
static float s_beta_du  = 0.5f, s_beta_dv  = 0.5f, s_beta_dw  = 0.5f;
static float s_alpha_du = 0.5f, s_alpha_dv = 0.5f, s_alpha_dw = 0.5f;

/* 两个绝对电角度：s_th1 = BETA 角，s_th2 = ALPHA 角
 *   mode3: s_th1=90°, s_th2=0°
 *   mode5: s_th1=起始角, s_th2=起始角-90°（与 mode3 同为电角度减小）
 *   由调用方在调 ec_precompute_duty() 之前设好 */
static float s_th1 = 0.0f;
static float s_th2 = 0.0f;

static const float s_sweep_th[4] = {
    0.0f, 1.5707963f, 3.1415926f, 4.7123890f
};

/*=============================================================================
 * 内部工具
 *=============================================================================*/

/* 按调用方给定的 s_th1 / s_th2 两个绝对电角度预计算占空比 */
static void ec_precompute_duty(float align_volt)
{
    float      vbus = CFG_SAMPLING_SIM_VBUS_V;
    SvpwmCtx_t ctx;

    ctx.valpha = align_volt * Foc_Cos(s_th1);
    ctx.vbeta  = align_volt * Foc_Sin(s_th1);
    ctx.vbus   = vbus;
    SVPWM_Calc(&ctx);
    s_beta_du = ctx.duty_u;
    s_beta_dv = ctx.duty_v;
    s_beta_dw = ctx.duty_w;

    ctx.valpha = align_volt * Foc_Cos(s_th2);
    ctx.vbeta  = align_volt * Foc_Sin(s_th2);
    ctx.vbus   = vbus;
    SVPWM_Calc(&ctx);
    s_alpha_du = ctx.duty_u;
    s_alpha_dv = ctx.duty_v;
    s_alpha_dw = ctx.duty_w;
}

static void ec_output_zero_vector(void)
{
    hw_tmr4_pwm_set_duty3(50.0f, 50.0f, 50.0f);
}

static void ec_output_first_segment(void)
{
    hw_tmr4_pwm_set_duty3(s_beta_du * 100.0f,
                          s_beta_dv * 100.0f,
                          s_beta_dw * 100.0f);
}

static void ec_output_second_segment(void)
{
    hw_tmr4_pwm_set_duty3(s_alpha_du * 100.0f,
                          s_alpha_dv * 100.0f,
                          s_alpha_dw * 100.0f);
}

/* 按任意电角度输出矢量（mode6 强拖用） */
static void ec_output_vector(float theta, float volt)
{
    SvpwmCtx_t ctx;

    ctx.valpha = volt * Foc_Cos(theta);
    ctx.vbeta  = volt * Foc_Sin(theta);
    ctx.vbus   = CFG_SAMPLING_SIM_VBUS_V;
    SVPWM_Calc(&ctx);

    hw_tmr4_pwm_set_duty3(ctx.duty_u * 100.0f,
                          ctx.duty_v * 100.0f,
                          ctx.duty_w * 100.0f);
}

/* mode5 每段电流统计：清零 / 取平均（输出 mA） */
static void ec_seg_reset(void)
{
    s_seg_sum_u = 0.0f;
    s_seg_sum_v = 0.0f;
    s_seg_sum_w = 0.0f;
    s_seg_cnt   = 0u;
    s_seg_tick  = 0u;
}

static void ec_seg_snapshot(int32_t dst[3])
{
    if (s_seg_cnt == 0u) {
        dst[0] = 0;
        dst[1] = 0;
        dst[2] = 0;
        return;
    }
    dst[0] = (int32_t)(s_seg_sum_u / (float)s_seg_cnt * 1000.0f);
    dst[1] = (int32_t)(s_seg_sum_v / (float)s_seg_cnt * 1000.0f);
    dst[2] = (int32_t)(s_seg_sum_w / (float)s_seg_cnt * 1000.0f);
}

/*=============================================================================
 * 启动
 *=============================================================================*/
void EncoderCalib_StartAlign(void)
{
    float align_volt;

    align_volt = g_align_volt;
    if (align_volt < 0.05f)  align_volt = 0.05f;
    if (align_volt > 5.0f)   align_volt = 5.0f;

    s_sweep_en = 0u;
    s_rev_en   = 0u;
    s_a7_en    = 0u;
    g_sweep_done = 0u;
    g_rev_done   = 0u;
    g_a7_done    = 0u;

    /* mode 3：BETA=90°，ALPHA=0°（两个绝对电角度） */
    s_th1 = 1.5707963f;
    s_th2 = 0.0f;

    s_hold_ms_l = g_align_hold_ms;
    if (s_hold_ms_l < 20u)    s_hold_ms_l = 20u;
    if (s_hold_ms_l > 20000u) s_hold_ms_l = 20000u;

    ec_precompute_duty(align_volt);

    g_align_evt       = EC_EVT_NONE;
    g_align_beta_hw   = 0;
    g_align_alpha_hw  = 0;
    g_align_moved     = 0;
    g_align_offset    = 0;
    g_align_zero_u_ma = 0;
    g_align_zero_v_ma = 0;
    g_align_zero_w_ma = 0;
    s_align_ok        = 0u;

    g_align_active = 1u;
    hw_tmr4_pwm_resume_output();

    /* ZERO：零矢量 50/50/50，20kHz ISR 采样 */
    s_zero_cnt   = 0u;
    s_zero_sum_u = 0;
    s_zero_sum_v = 0;
    s_zero_sum_w = 0;
    ec_output_zero_vector();

    s_hold_cnt = 0u;
    s_ec_state = EC_ZERO;

    MAIN_D("[EC] StartAlign: volt_x100=%ld hold_ms=%u th1_x1000=%ld th2_x1000=%ld",
           (long)(align_volt * 100.0f),
           (unsigned)s_hold_ms_l,
           (long)(s_th1 * 1000.0f),
           (long)(s_th2 * 1000.0f));
}

/*=============================================================================
 * mode7：ZERO -> 0°斜坡到90° -> 保持 -> 斜坡回0° -> 保持
 *   与 mode3 相同的 dir/offset 计算，但用慢速斜坡避免单步跳变激起振荡/卡齿槽
 *=============================================================================*/
void EncoderCalib_StartAlign7(void)
{
    float    align_volt = g_align_volt;
    uint32_t ramp_ms    = g_align_ramp_ms;
    uint32_t hold_ms    = g_align_hold_ms;

    if (align_volt < 0.05f) align_volt = 0.05f;
    if (align_volt > 5.0f)  align_volt = 5.0f;
    if (ramp_ms < 20u)      ramp_ms = 20u;
    if (ramp_ms > 5000u)    ramp_ms = 5000u;
    if (hold_ms < 100u)     hold_ms = 100u;
    if (hold_ms > 5000u)    hold_ms = 5000u;

    s_a7_volt_l    = align_volt;
    s_a7_ramp_ms_l = ramp_ms;
    s_a7_hold_ms_l = hold_ms;

    s_sweep_en = 0u;
    s_rev_en   = 0u;
    s_a7_en    = 1u;
    s_align_ok = 0u;
    g_sweep_done = 0u;
    g_rev_done   = 0u;
    g_a7_done    = 0u;

    s_a7_theta    = 0.0f;
    s_a7_ramp_cnt = 0u;

    g_align_evt       = EC_EVT_NONE;
    g_align_beta_hw   = 0;
    g_align_alpha_hw  = 0;
    g_align_moved     = 0;
    g_align_offset    = 0;
    g_align_zero_u_ma = 0;
    g_align_zero_v_ma = 0;
    g_align_zero_w_ma = 0;

    g_align_active = 1u;
    hw_tmr4_pwm_resume_output();

    /* ZERO：零矢量 50/50/50，20kHz ISR 采样 */
    s_zero_cnt   = 0u;
    s_zero_sum_u = 0;
    s_zero_sum_v = 0;
    s_zero_sum_w = 0;
    ec_output_zero_vector();

    s_hold_cnt = 0u;
    s_ec_state = EC_ZERO;

    MAIN_D("[CAL7] start: volt_x100=%ld ramp_ms=%lu hold_ms=%lu",
           (long)(align_volt * 100.0f),
           (unsigned long)ramp_ms,
           (unsigned long)hold_ms);
}

void EncoderCalib_StartSweep(void)
{
    float align_volt;

    align_volt = g_align_volt;
    if (align_volt < 0.05f)  align_volt = 0.05f;
    if (align_volt > 5.0f)   align_volt = 5.0f;

    s_sweep_en   = 1u;
    s_rev_en     = 0u;
    s_a7_en      = 0u;
    g_sweep_idx  = 0u;
    g_sweep_done = 0u;
    g_rev_done   = 0u;
    g_a7_done    = 0u;

    /* mode 5：BETA=起始角0，ALPHA=起始角0-90°（与 mode 3 同向） */
    s_th1 = s_sweep_th[0];
    s_th2 = s_th1 - 1.5707963f;

    s_hold_ms_l = g_align_hold_ms;
    if (s_hold_ms_l < 20u)    s_hold_ms_l = 20u;
    if (s_hold_ms_l > 20000u) s_hold_ms_l = 20000u;

    g_align_evt = EC_EVT_NONE;
    s_align_ok  = 0u;

    ec_precompute_duty(align_volt);

    g_align_active = 1u;
    hw_tmr4_pwm_resume_output();
    ec_output_first_segment();
    ec_seg_reset();

    s_hold_cnt = 0u;
    s_ec_state = EC_BETA;

    MAIN_D("[EC] StartSweep: volt_x100=%ld hold_ms=%u th1_x1000=%ld th2_x1000=%ld",
           (long)(align_volt * 100.0f),
           (unsigned)s_hold_ms_l,
           (long)(s_th1 * 1000.0f),
           (long)(s_th2 * 1000.0f));
}

/*=============================================================================
 * mode6：对齐 -> 强拖 pp_cfg*360° -> 再对齐，测一圈编码器计数
 *   delta = end - start（累加计数，可为负）
 *   pp_meas = CPR * pp_cfg / |delta|
 *=============================================================================*/
void EncoderCalib_StartRevMeasure(void)
{
    float volt = g_rev_volt;
    float f    = g_rev_freq_hz;
    uint32_t ams = g_rev_align_ms;

    if (volt < 0.05f) volt = 0.05f;
    if (volt > 5.0f)  volt = 5.0f;
    if (f < 0.1f)     f = 0.1f;
    if (f > 5.0f)     f = 5.0f;
    if (ams < 100u)   ams = 100u;
    if (ams > 5000u)  ams = 5000u;

    s_rev_volt_l = volt;
    s_rev_f_l    = f;
    s_hold_ms_l  = ams;

    s_sweep_en = 0u;
    s_rev_en   = 1u;
    s_a7_en    = 0u;
    s_align_ok = 0u;
    g_sweep_done = 0u;
    g_rev_done   = 0u;
    g_a7_done    = 0u;

    s_rev_theta     = 0.0f;
    s_rev_total     = 0.0f;
    s_rev_ramp_cnt  = 0u;
    s_rev_last_cnt  = hw_encoder_get_count();
    s_rev_cyc_idx   = 0u;
    s_rev_cyc_n     = 0u;
    s_rev_cyc_min_l = 0;
    s_rev_cyc_max_l = 0;

    g_rev_start_cnt = 0;
    g_rev_end_cnt   = 0;
    g_rev_delta     = 0;
    g_rev_pp_meas   = 0.0f;
    g_rev_cyc_min   = 0;
    g_rev_cyc_max   = 0;
    g_rev_done      = 0u;
    g_rev_slip      = 0u;
    g_align_evt     = EC_EVT_NONE;

    g_align_active = 1u;
    hw_tmr4_pwm_resume_output();
    ec_output_vector(0.0f, volt);

    s_hold_cnt = 0u;
    s_ec_state = EC_REV_ALIGN0;

    MAIN_D("[REV] start: volt_x100=%ld f_x100=%ld align_ms=%lu ramp_ms=%lu revs=%u",
           (long)(volt * 100.0f),
           (long)(f * 100.0f),
           (unsigned long)ams,
           (unsigned long)g_rev_ramp_ms,
           (unsigned)CFG_MOTOR_POLE_PAIRS);
}

/*=============================================================================
 * 20kHz ISR：ZERO 阶段零偏采样
 *=============================================================================*/
void EncoderCalib_Task20k(void)
{
    /* mode5：BETA/ALPHA 段累计三相电流（跳过前 100ms，取后段平均） */
    if (s_sweep_en) {
        if ((s_ec_state == EC_BETA) || (s_ec_state == EC_ALPHA)) {
            s_seg_tick++;
            if (s_seg_tick > EC_SEG_SKIP_TICKS) {
                s_seg_sum_u += g_fc_iu;
                s_seg_sum_v += g_fc_iv;
                s_seg_sum_w += g_fc_iw;
                s_seg_cnt++;
            }
        }
        return;   /* sweep 没有 ZERO 阶段 */
    }

    if (s_ec_state != EC_ZERO) {
        return;
    }

    s_zero_cnt++;
    if (s_zero_cnt <= EC_ZERO_SKIP) {
        return;
    }

    s_zero_sum_u += (int64_t)(g_fc_iu * 1000.0f);
    s_zero_sum_v += (int64_t)(g_fc_iv * 1000.0f);
    s_zero_sum_w += (int64_t)(g_fc_iw * 1000.0f);

    if ((s_zero_cnt - EC_ZERO_SKIP) >= EC_ZERO_AVG) {
        g_align_zero_u_ma = (int32_t)(s_zero_sum_u / EC_ZERO_AVG);
        g_align_zero_v_ma = (int32_t)(s_zero_sum_v / EC_ZERO_AVG);
        g_align_zero_w_ma = (int32_t)(s_zero_sum_w / EC_ZERO_AVG);

        g_align_evt = EC_EVT_ZERO_DONE;
        s_hold_cnt  = 0u;

        if (s_a7_en) {
            /* mode7：从 0° 开始斜坡升到 90° */
            s_a7_theta    = 0.0f;
            s_a7_ramp_cnt = 0u;
            s_ec_state    = EC_A7_RAMP_UP;
        } else {
            /* 切 BETA：给 BETA 矢量（s_th1，mode3 = 90°） */
            ec_output_first_segment();
            s_ec_state = EC_BETA;
        }
    }
}

/*=============================================================================
 * 1ms ISR：BETA/ALPHA 保持计时
 *=============================================================================*/
void EncoderCalib_Task1ms(void)
{
    int32_t moved;

    if (g_fp_state != FP_STATE_NORMAL) {
        if ((s_ec_state == EC_ZERO) || (s_ec_state == EC_BETA) ||
            (s_ec_state == EC_ALPHA) ||
            (s_ec_state == EC_REV_ALIGN0) || (s_ec_state == EC_REV_DRAG) ||
            (s_ec_state == EC_REV_ALIGN1) ||
            (s_ec_state == EC_A7_RAMP_UP) || (s_ec_state == EC_A7_HOLD_BETA) ||
            (s_ec_state == EC_A7_RAMP_DN) || (s_ec_state == EC_A7_HOLD_ALPHA)) {
            MAIN_E("[EC] abort: fp_state=%d fault=0x%08lX "
                   "iu_x100=%ld iv_x100=%ld iw_x100=%ld state_before=%d",
                   (int)g_fp_state,
                   (unsigned long)g_fp_fault,
                   (long)(g_fc_iu * 100.0f),
                   (long)(g_fc_iv * 100.0f),
                   (long)(g_fc_iw * 100.0f),
                   (int)s_ec_state);
            g_align_active = 0u;
            s_align_ok     = 0u;
            if (s_sweep_en) {
                g_sweep_done = 1u;
            }
            if (s_rev_en) {
                g_rev_done = 1u;
                g_rev_slip = 1u;
            }
            if (s_a7_en) {
                g_a7_done = 1u;
            }
            g_align_evt = EC_EVT_OC;
            s_ec_state  = EC_DONE;
        }
        return;
    }

    switch (s_ec_state) {
    case EC_REV_ALIGN0:
        s_hold_cnt++;
        if (s_hold_cnt < s_hold_ms_l) {
            return;
        }

        g_rev_start_cnt = hw_encoder_get_count();
        s_rev_last_cnt  = g_rev_start_cnt;
        s_rev_theta     = 0.0f;
        s_rev_total     = 0.0f;
        s_rev_ramp_cnt  = 0u;
        s_rev_cyc_idx   = 0u;
        s_rev_cyc_n     = 0u;
        s_rev_cyc_min_l = 0;
        s_rev_cyc_max_l = 0;

        s_hold_cnt = 0u;
        s_ec_state = EC_REV_DRAG;
        return;

    case EC_REV_DRAG:
    {
        float f;
        float dth;
        float remain;

        f = s_rev_f_l;
        if (s_rev_ramp_cnt < g_rev_ramp_ms) {
            f *= (float)s_rev_ramp_cnt / (float)g_rev_ramp_ms;
            s_rev_ramp_cnt++;
        }

        /* 最后一个电周期线性减速，避免停点时惯性冲过电周期零点 */
        remain = (float)CFG_MOTOR_POLE_PAIRS * EC_TWO_PI - s_rev_total;
        if (remain < EC_TWO_PI) {
            float ff = f * (remain / EC_TWO_PI);
            if (ff < (s_rev_f_l * 0.05f)) {
                ff = s_rev_f_l * 0.05f;
            }
            f = ff;
        }

        dth = EC_TWO_PI * f * 0.001f;
        s_rev_theta += dth;
        s_rev_total += dth;
        while (s_rev_theta >= EC_TWO_PI) {
            s_rev_theta -= EC_TWO_PI;
        }
        ec_output_vector(s_rev_theta, s_rev_volt_l);

        /* 每过一个电周期记录机械计数增量，用于判断是否失步 */
        {
            uint32_t cyc = (uint32_t)(s_rev_total * (1.0f / EC_TWO_PI));

            if (cyc > s_rev_cyc_idx) {
                int32_t d = hw_encoder_get_count() - s_rev_last_cnt;

                if (d < 0) d = -d;
                if (cyc >= 2u) {
                    if (s_rev_cyc_n == 0u) {
                        s_rev_cyc_min_l = d;
                        s_rev_cyc_max_l = d;
                    } else {
                        if (d < s_rev_cyc_min_l) s_rev_cyc_min_l = d;
                        if (d > s_rev_cyc_max_l) s_rev_cyc_max_l = d;
                    }
                    s_rev_cyc_n++;
                }
                s_rev_last_cnt = hw_encoder_get_count();
                s_rev_cyc_idx  = cyc;
            }
        }

        if (s_rev_total >= (float)CFG_MOTOR_POLE_PAIRS * EC_TWO_PI) {
            ec_output_vector(0.0f, s_rev_volt_l);
            s_hold_cnt = 0u;
            s_ec_state = EC_REV_ALIGN1;
        }
        return;
    }

    case EC_REV_ALIGN1:
    {
        int32_t delta;

        s_hold_cnt++;
        if (s_hold_cnt < s_hold_ms_l) {
            return;
        }

        g_rev_end_cnt = hw_encoder_get_count();
        delta = g_rev_end_cnt - g_rev_start_cnt;
        g_rev_delta = delta;
        if (delta != 0) {
            float ad = (delta < 0) ? (float)(-delta) : (float)delta;

            g_rev_pp_meas = ((float)CFG_ENCODER_CPR * (float)CFG_MOTOR_POLE_PAIRS) / ad;
        }
        g_rev_cyc_min = s_rev_cyc_min_l;
        g_rev_cyc_max = s_rev_cyc_max_l;
        g_rev_slip = 0u;
        if (s_rev_cyc_n >= 2u) {
            if ((s_rev_cyc_max_l - s_rev_cyc_min_l) > (s_rev_cyc_max_l / 2)) {
                g_rev_slip = 1u;
            }
        }

        g_rev_done     = 1u;
        s_align_ok     = 0u;
        g_align_evt    = EC_EVT_DONE_OK;
        g_align_active = 0u;
        s_ec_state     = EC_DONE;
        return;
    }

    case EC_A7_RAMP_UP:
        s_a7_ramp_cnt++;
        if (s_a7_ramp_cnt >= s_a7_ramp_ms_l) {
            s_a7_theta = EC_PI_2;
        } else {
            s_a7_theta = (EC_PI_2 * (float)s_a7_ramp_cnt) / (float)s_a7_ramp_ms_l;
        }
        ec_output_vector(s_a7_theta, s_a7_volt_l);
        if (s_a7_ramp_cnt >= s_a7_ramp_ms_l) {
            s_hold_cnt = 0u;
            s_ec_state = EC_A7_HOLD_BETA;
        }
        return;

    case EC_A7_HOLD_BETA:
        s_hold_cnt++;
        if (s_hold_cnt < s_a7_hold_ms_l) {
            return;
        }

        s_beta_hw_l     = hw_encoder_get_count();
        g_align_beta_hw = s_beta_hw_l;

        s_a7_ramp_cnt = 0u;
        s_hold_cnt    = 0u;
        s_ec_state    = EC_A7_RAMP_DN;

        MAIN_D("[CAL7] BETA done: beta_hw=%ld", (long)s_beta_hw_l);
        return;

    case EC_A7_RAMP_DN:
        s_a7_ramp_cnt++;
        if (s_a7_ramp_cnt >= s_a7_ramp_ms_l) {
            s_a7_theta = 0.0f;
        } else {
            s_a7_theta = EC_PI_2
                       - (EC_PI_2 * (float)s_a7_ramp_cnt) / (float)s_a7_ramp_ms_l;
        }
        ec_output_vector(s_a7_theta, s_a7_volt_l);
        if (s_a7_ramp_cnt >= s_a7_ramp_ms_l) {
            s_hold_cnt = 0u;
            s_ec_state = EC_A7_HOLD_ALPHA;
        }
        return;

    case EC_A7_HOLD_ALPHA:
        s_hold_cnt++;
        if (s_hold_cnt < s_a7_hold_ms_l) {
            return;
        }

        s_alpha_hw_l     = hw_encoder_get_count();
        g_align_alpha_hw = s_alpha_hw_l;
        moved            = s_alpha_hw_l - s_beta_hw_l;
        g_align_moved    = moved;

        if (moved > 0) {
            g_enc_elec_dir = -1;
        } else if (moved < 0) {
            g_enc_elec_dir = +1;
        } else {
            s_align_ok  = 0u;
            g_a7_done   = 1u;
            g_align_evt = EC_EVT_DONE_FAIL;
            g_align_active = 0u;
            s_ec_state  = EC_DONE;
            MAIN_E("[CAL7] ALPHA done: moved=0, direction undecidable -> FAIL");
            return;
        }

        {
            int32_t c = s_alpha_hw_l % (int32_t)CFG_ENCODER_CPR;

            if (c < 0) c += (int32_t)CFG_ENCODER_CPR;
            g_enc_elec_offset = c;
            g_align_offset    = c;
        }

        g_motor_is_aligned = 1u;
        s_align_ok         = 1u;
        g_a7_done          = 1u;
        g_align_evt        = EC_EVT_DONE_OK;
        g_align_active     = 0u;
        s_ec_state         = EC_DONE;
        return;

    case EC_BETA:
        s_hold_cnt++;
        if (s_hold_cnt < s_hold_ms_l) {
            return;
        }

        s_beta_hw_l     = hw_encoder_get_count();
        g_align_beta_hw = s_beta_hw_l;

        /* mode5：结算 BETA 段平均电流，随后清零给 ALPHA 段 */
        if (s_sweep_en) {
            int32_t seg_i[3];

            ec_seg_snapshot(seg_i);
            if (g_sweep_idx < 4u) {
                g_sweep_i_ma[g_sweep_idx][0] = seg_i[0];
                g_sweep_i_ma[g_sweep_idx][1] = seg_i[1];
                g_sweep_i_ma[g_sweep_idx][2] = seg_i[2];
            }
            ec_seg_reset();
        }

        /* 切 ALPHA：给 ALPHA 矢量（s_th2，mode3 = 0°） */
        ec_output_second_segment();

        g_align_evt = EC_EVT_BETA_DONE;
        s_hold_cnt  = 0u;
        s_ec_state  = EC_ALPHA;

        MAIN_D("[EC] BETA done: beta_hw=%ld", (long)s_beta_hw_l);
        return;

    case EC_ALPHA:
        s_hold_cnt++;
        if (s_hold_cnt < s_hold_ms_l) {
            return;
        }

        s_alpha_hw_l     = hw_encoder_get_count();
        g_align_alpha_hw = s_alpha_hw_l;
        moved            = s_alpha_hw_l - s_beta_hw_l;
        g_align_moved    = moved;

        if (s_sweep_en) {
            uint8_t idx = g_sweep_idx;
            int32_t seg_i[3];

            /* 结算 ALPHA 段平均电流 */
            ec_seg_snapshot(seg_i);
            if (idx < 4u) {
                g_sweep_i_ma[idx][3] = seg_i[0];
                g_sweep_i_ma[idx][4] = seg_i[1];
                g_sweep_i_ma[idx][5] = seg_i[2];

                g_sweep_beta[idx]  = s_beta_hw_l;
                g_sweep_alpha[idx] = s_alpha_hw_l;
                g_sweep_moved[idx] = moved;
                g_sweep_theta[idx] = (int32_t)(s_th1 * 1000.0f);
            }

            if (idx < 3u) {
                g_sweep_idx = (uint8_t)(idx + 1u);

                /* 下一段：BETA=下一起始角，ALPHA=BETA-90°（与 mode 3 同向） */
                s_th1 = s_sweep_th[g_sweep_idx];
                s_th2 = s_th1 - 1.5707963f;
                ec_precompute_duty(g_align_volt);
                ec_output_first_segment();
                ec_seg_reset();

                g_align_evt = EC_EVT_BETA_DONE;
                s_hold_cnt  = 0u;
                s_ec_state  = EC_BETA;
                return;
            }

            g_sweep_done   = 1u;
            g_align_active = 0u;
            s_ec_state     = EC_DONE;
            g_align_evt    = EC_EVT_DONE_OK;
            return;
        }

        /* mode 3：写 offset/dir
         *   物理：BETA(90°) -> ALPHA(0°)，电角度减小 90°
         *   moved > 0 : 电角度减小对应计数增 -> 计数增=电角度减 -> dir = -1
         *   moved < 0 : 电角度减小对应计数减 -> 计数增=电角度增 -> dir = +1
         *   moved == 0 : 转子未动，方向无法判定 -> 判失败，不写 dir
         */
        if (moved > 0) {
            g_enc_elec_dir = -1;
        } else if (moved < 0) {
            g_enc_elec_dir = +1;
        } else {
            s_align_ok     = 0u;
            g_align_evt    = EC_EVT_DONE_FAIL;
            g_align_active = 0u;
            s_ec_state     = EC_DONE;
            MAIN_E("[EC] ALPHA done: moved=0, direction undecidable -> FAIL");
            return;
        }

        {
            int32_t c = s_alpha_hw_l % (int32_t)CFG_ENCODER_CPR;
            if (c < 0) c += (int32_t)CFG_ENCODER_CPR;
            g_enc_elec_offset = c;
            g_align_offset    = c;
        }

        g_motor_is_aligned = 1u;
        s_align_ok         = 1u;
        g_align_evt        = EC_EVT_DONE_OK;
        g_align_active     = 0u;
        s_ec_state         = EC_DONE;

        MAIN_D("[EC] ALPHA done: alpha_hw=%ld moved=%ld offset=%ld dir=%d",
               (long)s_alpha_hw_l, (long)moved,
               (long)g_enc_elec_offset, (int)g_enc_elec_dir);
        return;

    case EC_ZERO:
    case EC_DONE:
    case EC_IDLE:
    default:
        return;
    }
}

/*=============================================================================
 * 查询与事件消费
 *=============================================================================*/
uint8_t EncoderCalib_IsDone(void)
{
    return (s_ec_state == EC_DONE) ? 1u : 0u;
}

uint8_t EncoderCalib_IsAlignOk(void)
{
    return s_align_ok;
}

uint8_t EncoderCalib_IsSweepDone(void)
{
    return g_sweep_done;
}

uint8_t EncoderCalib_IsRevMeasureDone(void)
{
    return g_rev_done;
}

uint8_t EncoderCalib_IsAlign7Done(void)
{
    return g_a7_done;
}

uint8_t EncoderCalib_ConsumeEvent(void)
{
    uint8_t e = g_align_evt;
    g_align_evt = EC_EVT_NONE;
    return e;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
