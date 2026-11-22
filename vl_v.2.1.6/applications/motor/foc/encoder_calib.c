/**
 *******************************************************************************
 * @file  encoder_calib.c
 * @brief 编码器电角度标定层实现
 *
 *        dir 符号约定（与 hw_encoder.c 一致）：
 *          moved = alpha_hw - beta_hw
 *          moved > 0 : 电角度减小对应计数增 -> dir = -1
 *          moved < 0 : 电角度减小对应计数减 -> dir = +1
 *
 *        mode 3 的 ZERO 阶段同时完成电流零偏采样：
 *          前 EC_ZERO_SKIP 拍丢弃，后 EC_ZERO_AVG 拍取均值，
 *          切 BETA 前把均值写入采样层零点。
 *******************************************************************************
 */

#include "encoder_calib.h"
#include "hw_encoder.h"
#include "core/foc_output.h"
#include "svpwm.h"
#include "foc_control.h"
#include "foc_protect.h"
#include "foc_sincos.h"
#include "cfg_motor.h"
#include "cfg_sampling.h"
#include "../drv/bsp_adc_foc.h"

#include <rtthread.h>

/*=============================================================================
 * 观测量 / 调试可写量
 *=============================================================================*/
volatile uint8_t g_motor_is_aligned = 0u;

volatile float    g_align_volt     = 1.3f;
volatile float    g_align_theta    = 1.5707963f;
volatile uint32_t g_align_hold_ms  = 200u;
volatile uint32_t g_align_ramp_ms  = 500u;
volatile uint8_t  g_a7_done        = 0u;

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

volatile int32_t  g_sweep_i_ma[4][6] = { { 0 } };

volatile float    g_rev_volt       = 1.5f;
volatile float    g_rev_freq_hz    = 1.0f;
volatile uint32_t g_rev_align_ms   = 1000u;
volatile uint32_t g_rev_ramp_ms    = 500u;
volatile int32_t  g_rev_start_cnt  = 0;
volatile int32_t  g_rev_end_cnt    = 0;
volatile int32_t  g_rev_delta      = 0;
volatile float    g_rev_pp_meas    = 0.0f;
volatile int32_t  g_rev_cyc_min    = 0;
volatile int32_t  g_rev_cyc_max    = 0;
volatile uint8_t  g_rev_done       = 0u;
volatile uint8_t  g_rev_slip       = 0u;

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
static uint16_t s_zero_wait_ms = 0u;             /* ZERO 阶段看门狗 */
static volatile uint8_t s_zero_apply_req = 0u;   /* 1ms 上下文里应用零偏 */
static volatile uint8_t s_zero_done_req  = 0u;   /* 1ms 上下文里打印 ZERO 计数 */
static int32_t  s_zero_hw_l = 0;                 /* ZERO 结束时的 ABZ 计数 */

/* BETA/ALPHA 计时与取值 */
static volatile uint32_t s_hold_cnt = 0u;
static uint32_t s_hold_ms_l = 200u;

/* mode5 每段电流统计 */
#define EC_SEG_SKIP_TICKS   2000u
static float    s_seg_sum_u = 0.0f;
static float    s_seg_sum_v = 0.0f;
static float    s_seg_sum_w = 0.0f;
static uint32_t s_seg_cnt   = 0u;
static uint32_t s_seg_tick  = 0u;

static int32_t s_beta_hw_l  = 0;
static int32_t s_alpha_hw_l = 0;

static float s_beta_du  = 0.5f, s_beta_dv  = 0.5f, s_beta_dw  = 0.5f;
static float s_alpha_du = 0.5f, s_alpha_dv = 0.5f, s_alpha_dw = 0.5f;

/* 两个绝对电角度：s_th1 = BETA 角，s_th2 = ALPHA 角 */
static float s_th1 = 0.0f;
static float s_th2 = 0.0f;

static const float s_sweep_th[4] = {
    0.0f, 1.5707963f, 3.1415926f, 4.7123890f
};

/*=============================================================================
 * 内部工具
 *=============================================================================*/
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
    Foc_Output_Duty3(50.0f, 50.0f, 50.0f);
}

static void ec_output_first_segment(void)
{
    Foc_Output_Duty3(s_beta_du * 100.0f,
                          s_beta_dv * 100.0f,
                          s_beta_dw * 100.0f);
}

static void ec_output_second_segment(void)
{
    Foc_Output_Duty3(s_alpha_du * 100.0f,
                          s_alpha_dv * 100.0f,
                          s_alpha_dw * 100.0f);
}

static void ec_output_vector(float theta, float volt)
{
    SvpwmCtx_t ctx;

    ctx.valpha = volt * Foc_Cos(theta);
    ctx.vbeta  = volt * Foc_Sin(theta);
    ctx.vbus   = CFG_SAMPLING_SIM_VBUS_V;
    SVPWM_Calc(&ctx);

    Foc_Output_Duty3(ctx.duty_u * 100.0f,
                          ctx.duty_v * 100.0f,
                          ctx.duty_w * 100.0f);
}

static void ec_seg_reset(void)
{
    s_seg_sum_u = 0.0f;
    s_seg_sum_v = 0.0f;
    s_seg_sum_w = 0.0f;
    s_seg_cnt   = 0u;
    s_seg_tick  = 0u;
}

/* 标定结束：交还 PWM 并回到 2% 零矢量，避免停在最后一个标定矢量上 */
static void ec_finish_zero_vector(void)
{
    g_align_active = 0u;
    (void)Foc_Output_Duty3(2.0f, 2.0f, 2.0f);
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

/* ZERO 结束时把零偏均值写入采样层 */
static void ec_apply_zero_offset(void)
{
    uint16_t z[3];

    if (foc_adc_cal_get_result(z) != 0u) {
        foc_adc_apply_offset_raw(z);
        rt_kprintf("[EC] zero offset: raw=%u/%u/%u\r\n",
                   (unsigned)z[0], (unsigned)z[1], (unsigned)z[2]);
    }
    foc_adc_cal_stop();
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

    /* mode 3：BETA=90°，ALPHA=0° */
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
    s_align_ok        = 0u;
    s_zero_hw_l       = 0;
    s_zero_done_req   = 0u;
    s_zero_apply_req  = 0u;

    /* ZERO 准备：先把状态机和标志都置好，再动 PWM。
     * 若先置标志后置状态，ISR 可能看到“标志=1 但状态=IDLE”而清掉标志；
     * 若先写 PWM 后置标志，ISR 又会用旧模式输出覆盖零矢量。 */
    s_zero_cnt = 0u;
    s_hold_cnt = 0u;
    foc_adc_cal_begin();
    s_ec_state    = EC_ZERO;
    g_align_active = 1u;

    Foc_Output_Resume();
    ec_output_zero_vector();

    rt_kprintf("[CAL] StartAlign: volt_x100=%ld hold_ms=%u th1_x1000=%ld th2_x1000=%ld\r\n",
               (long)(align_volt * 100.0f),
               (unsigned)s_hold_ms_l,
               (long)(s_th1 * 1000.0f),
               (long)(s_th2 * 1000.0f));
    rt_kprintf("[CAL] drags are time-driven: BETA 90deg -> ALPHA 0deg, "
               "each %u ms; ABZ only records, not required\r\n",
               (unsigned)s_hold_ms_l);
}

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
    s_zero_hw_l       = 0;
    s_zero_done_req   = 0u;

    s_zero_cnt = 0u;
    s_hold_cnt = 0u;
    s_ec_state = EC_ZERO;
    g_align_active = 1u;

    Foc_Output_Resume();
    ec_output_zero_vector();

    rt_kprintf("[CAL7] start: volt_x100=%ld ramp_ms=%lu hold_ms=%lu\r\n",
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

    /* mode 5：BETA=起始角0，ALPHA=起始角0-90° */
    s_th1 = s_sweep_th[0];
    s_th2 = s_th1 - 1.5707963f;

    s_hold_ms_l = g_align_hold_ms;
    if (s_hold_ms_l < 20u)    s_hold_ms_l = 20u;
    if (s_hold_ms_l > 20000u) s_hold_ms_l = 20000u;

    g_align_evt = EC_EVT_NONE;
    s_align_ok  = 0u;

    ec_precompute_duty(align_volt);

    Foc_Output_Resume();
    ec_output_first_segment();
    ec_seg_reset();

    /* 先状态后标志，避免 ISR 落在两句之间误清 g_align_active */
    s_hold_cnt = 0u;
    s_ec_state = EC_BETA;
    g_align_active = 1u;

    rt_kprintf("[EC] StartSweep: volt_x100=%ld hold_ms=%u th1_x1000=%ld th2_x1000=%ld\r\n",
               (long)(align_volt * 100.0f),
               (unsigned)s_hold_ms_l,
               (long)(s_th1 * 1000.0f),
               (long)(s_th2 * 1000.0f));
}

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

    s_hold_cnt = 0u;
    s_ec_state = EC_REV_ALIGN0;
    g_align_active = 1u;

    Foc_Output_Resume();
    ec_output_vector(0.0f, volt);

    rt_kprintf("[REV] start: volt_x100=%ld f_x100=%ld align_ms=%lu ramp_ms=%lu revs=%u\r\n",
               (long)(volt * 100.0f),
               (long)(f * 100.0f),
               (unsigned long)ams,
               (unsigned long)g_rev_ramp_ms,
               (unsigned)CFG_MOTOR_POLE_PAIRS);
}

/*=============================================================================
 * 20kHz：ZERO 阶段计时 + 扫角电流统计
 *=============================================================================*/
void EncoderCalib_Task20k(void)
{
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
        return;
    }

    if (s_ec_state != EC_ZERO) {
        return;
    }

    s_zero_cnt++;
    if (s_zero_cnt <= EC_ZERO_SKIP) {
        return;
    }

    if ((s_zero_cnt - EC_ZERO_SKIP) >= EC_ZERO_AVG) {
        s_hold_cnt  = 0u;

        /* ZERO 结束：锁存 ABZ 计数，打印放到 1ms 线程上下文（ISR 内不打印） */
        s_zero_hw_l     = hw_encoder_get_count();
        s_zero_done_req = 1u;

        if (s_a7_en) {
            s_a7_theta    = 0.0f;
            s_a7_ramp_cnt = 0u;
            s_ec_state    = EC_A7_RAMP_UP;
        } else {
            /* mode 3：ZERO 结束，零偏换算交给 1ms 线程上下文执行
             * （ISR 内不做除法与 rt_kprintf） */
            s_zero_apply_req = 1u;
            ec_output_first_segment();
            s_ec_state = EC_BETA;
        }
    }
}

/*=============================================================================
 * 1ms：BETA/ALPHA 保持计时
 *=============================================================================*/
void EncoderCalib_Task1ms(void)
{
    int32_t moved;

    if (s_zero_done_req != 0u) {
        s_zero_done_req = 0u;
        rt_kprintf("[CAL] ZERO done: zero_hw=%ld\r\n", (long)s_zero_hw_l);
    }

    if (s_zero_apply_req != 0u) {
        s_zero_apply_req = 0u;
        ec_apply_zero_offset();
    }

    /* ZERO 正常 30ms 结束；超过 100ms 说明 20kHz 没有喂 Task20k */
    if ((s_ec_state == EC_ZERO) && (g_align_active != 0u)) {
        if (++s_zero_wait_ms >= 100u) {
            s_zero_wait_ms = 0u;
            rt_kprintf("[CAL] WARN: ZERO stuck, cnt=%lu active=%u busy=%u isr=%lu\r\n",
                       (unsigned long)s_zero_cnt,
                       (unsigned)g_align_active,
                       (unsigned)EncoderCalib_IsBusy(),
                       (unsigned long)g_fc_isr_cnt);
        }
    } else {
        s_zero_wait_ms = 0u;
    }

    if (g_fp_state != FP_STATE_NORMAL) {
        if ((s_ec_state == EC_ZERO) || (s_ec_state == EC_BETA) ||
            (s_ec_state == EC_ALPHA) ||
            (s_ec_state == EC_REV_ALIGN0) || (s_ec_state == EC_REV_DRAG) ||
            (s_ec_state == EC_REV_ALIGN1) ||
            (s_ec_state == EC_A7_RAMP_UP) || (s_ec_state == EC_A7_HOLD_BETA) ||
            (s_ec_state == EC_A7_RAMP_DN) || (s_ec_state == EC_A7_HOLD_ALPHA)) {
            rt_kprintf("[EC] abort: fp_state=%d fault=0x%08lX "
                       "iu_x100=%ld iv_x100=%ld iw_x100=%ld state_before=%d\r\n",
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
            ec_finish_zero_vector();
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
        ec_finish_zero_vector();
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

        rt_kprintf("[CAL7] BETA done: beta_hw=%ld\r\n", (long)s_beta_hw_l);
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
            ec_finish_zero_vector();
            s_ec_state  = EC_DONE;
            rt_kprintf("[CAL7] ALPHA done: moved=0, direction undecidable -> FAIL\r\n");
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
        ec_finish_zero_vector();
        s_ec_state         = EC_DONE;
        return;

    case EC_BETA:
        s_hold_cnt++;
        if (s_hold_cnt < s_hold_ms_l) {
            return;
        }

        s_beta_hw_l     = hw_encoder_get_count();
        g_align_beta_hw = s_beta_hw_l;

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

        ec_output_second_segment();

        g_align_evt = EC_EVT_BETA_DONE;
        s_hold_cnt  = 0u;
        s_ec_state  = EC_ALPHA;

        rt_kprintf("[CAL] BETA done: beta_hw=%ld zero_hw=%ld delta=%ld th=%ld deg\r\n",
                   (long)s_beta_hw_l,
                   (long)s_zero_hw_l,
                   (long)(s_beta_hw_l - s_zero_hw_l),
                   (long)(s_th1 * 57.29578f));
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
            ec_finish_zero_vector();
            s_ec_state     = EC_DONE;
            g_align_evt    = EC_EVT_DONE_OK;
            return;
        }

        /* mode 3：写 offset/dir */
        if (moved > 0) {
            g_enc_elec_dir = -1;
        } else if (moved < 0) {
            g_enc_elec_dir = +1;
        } else {
            /* 两次强拖已经完整执行（BETA 90° -> ALPHA 0°），
             * 没有 ABZ 时计数不变，moved=0 无法判方向：
             * 只跳过 offset/dir 写入，不回退、不提前中止流程。 */
            s_align_ok     = 0u;
            g_motor_is_aligned = 0u;
            g_align_evt    = EC_EVT_DONE_FAIL;
            ec_finish_zero_vector();
            s_ec_state     = EC_DONE;
            rt_kprintf("[CAL] ALPHA done: beta_hw=%ld alpha_hw=%ld moved=0; "
                       "both drags done, ABZ not connected or rotor not moved, "
                       "offset/dir NOT written\r\n",
                       (long)s_beta_hw_l, (long)s_alpha_hw_l);
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
        ec_finish_zero_vector();
        s_ec_state         = EC_DONE;

        rt_kprintf("[CAL] ALPHA done: beta_hw=%ld alpha_hw=%ld moved=%ld "
                   "offset=%ld dir=%d\r\n",
                   (long)s_beta_hw_l, (long)s_alpha_hw_l, (long)moved,
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

/* 标定状态机是否正在推进（ZERO/保持/强拖等，不含 IDLE/DONE） */
uint8_t EncoderCalib_IsBusy(void)
{
    switch (s_ec_state) {
    case EC_ZERO:
    case EC_BETA:
    case EC_ALPHA:
    case EC_REV_ALIGN0:
    case EC_REV_DRAG:
    case EC_REV_ALIGN1:
    case EC_A7_RAMP_UP:
    case EC_A7_HOLD_BETA:
    case EC_A7_RAMP_DN:
    case EC_A7_HOLD_ALPHA:
        return 1u;
    default:
        return 0u;
    }
}

/* 强制中止标定：切到非标定模式时调用，避免 g_align_active 卡住导致
 * 后续 IF/CURRENT/SPEED/POSITION 全部不刷新 PWM */
void EncoderCalib_Abort(void)
{
    if (g_align_active != 0u) {
        g_align_active = 0u;
    }
    s_sweep_en = 0u;
    s_rev_en   = 0u;
    s_a7_en    = 0u;
    s_align_ok = 0u;
    s_ec_state = EC_IDLE;
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
