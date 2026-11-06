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
 *   每角度两段（BETA=起始角, ALPHA=起始角+90°）后记 g_sweep_*，
 *   切下一角度，4 个跑完 DONE
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

/*=============================================================================
 * 状态机
 *=============================================================================*/
typedef enum {
    EC_IDLE = 0,
    EC_ZERO,
    EC_BETA,
    EC_ALPHA,
    EC_DONE,
} ec_state_t;

static volatile ec_state_t s_ec_state = EC_IDLE;
static volatile uint8_t    s_align_ok = 0u;
static volatile uint8_t    s_sweep_en = 0u;

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

static int32_t s_beta_hw_l  = 0;
static int32_t s_alpha_hw_l = 0;

/* 预计算 duty */
static float s_beta_du  = 0.5f, s_beta_dv  = 0.5f, s_beta_dw  = 0.5f;
static float s_alpha_du = 0.5f, s_alpha_dv = 0.5f, s_alpha_dw = 0.5f;

/* 两个绝对电角度：s_th1 = BETA 角，s_th2 = ALPHA 角
 *   mode3: s_th1=90°, s_th2=0°
 *   mode5: s_th1=起始角, s_th2=起始角+90°
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

void EncoderCalib_StartSweep(void)
{
    float align_volt;

    align_volt = g_align_volt;
    if (align_volt < 0.05f)  align_volt = 0.05f;
    if (align_volt > 5.0f)   align_volt = 5.0f;

    s_sweep_en   = 1u;
    g_sweep_idx  = 0u;
    g_sweep_done = 0u;

    /* mode 5：BETA=起始角0，ALPHA=起始角0+90° */
    s_th1 = s_sweep_th[0];
    s_th2 = s_th1 + 1.5707963f;

    s_hold_ms_l = g_align_hold_ms;
    if (s_hold_ms_l < 20u)    s_hold_ms_l = 20u;
    if (s_hold_ms_l > 20000u) s_hold_ms_l = 20000u;

    g_align_evt = EC_EVT_NONE;
    s_align_ok  = 0u;

    ec_precompute_duty(align_volt);

    g_align_active = 1u;
    hw_tmr4_pwm_resume_output();
    ec_output_first_segment();

    s_hold_cnt = 0u;
    s_ec_state = EC_BETA;

    MAIN_D("[EC] StartSweep: volt_x100=%ld hold_ms=%u th1_x1000=%ld th2_x1000=%ld",
           (long)(align_volt * 100.0f),
           (unsigned)s_hold_ms_l,
           (long)(s_th1 * 1000.0f),
           (long)(s_th2 * 1000.0f));
}

/*=============================================================================
 * 20kHz ISR：ZERO 阶段零偏采样
 *=============================================================================*/
void EncoderCalib_Task20k(void)
{
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

        /* 切 BETA：给 BETA 矢量（s_th1，mode3 = 90°） */
        ec_output_first_segment();

        g_align_evt = EC_EVT_ZERO_DONE;
        s_hold_cnt  = 0u;
        s_ec_state  = EC_BETA;
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
            (s_ec_state == EC_ALPHA)) {
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
            g_align_evt = EC_EVT_OC;
            s_ec_state  = EC_DONE;
        }
        return;
    }

    switch (s_ec_state) {
    case EC_BETA:
        s_hold_cnt++;
        if (s_hold_cnt < s_hold_ms_l) {
            return;
        }

        s_beta_hw_l     = hw_encoder_get_count();
        g_align_beta_hw = s_beta_hw_l;

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
            if (idx < 4u) {
                g_sweep_beta[idx]  = s_beta_hw_l;
                g_sweep_alpha[idx] = s_alpha_hw_l;
                g_sweep_moved[idx] = moved;
                g_sweep_theta[idx] = (int32_t)(s_th1 * 1000.0f);
            }

            if (idx < 3u) {
                g_sweep_idx = (uint8_t)(idx + 1u);

                /* 下一段：BETA=下一起始角，ALPHA=BETA+90° */
                s_th1 = s_sweep_th[g_sweep_idx];
                s_th2 = s_th1 + 1.5707963f;
                ec_precompute_duty(g_align_volt);
                ec_output_first_segment();

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
         *   （moved == 0 的判据留待对齐判据问题单独处理，本次只统一符号）
         */
        if (moved > 0) {
            g_enc_elec_dir = +1;
        } else {
            g_enc_elec_dir = -1;
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

uint8_t EncoderCalib_ConsumeEvent(void)
{
    uint8_t e = g_align_evt;
    g_align_evt = EC_EVT_NONE;
    return e;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
