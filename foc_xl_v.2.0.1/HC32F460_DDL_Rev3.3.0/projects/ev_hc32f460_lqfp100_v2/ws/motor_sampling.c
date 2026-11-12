/*=============================================================================
 * motor_sampling.c  —  L1 采样层实现
 *
 * 本版改动：
 *   - 电角度对齐（DoAlignBlocking）迁至 encoder_calib.c
 *   - g_motor_is_aligned 迁至 encoder_calib.c
 *   - 清理因此不再需要的 include
 *=============================================================================*/

#include "motor_sampling.h"
#include "hc32_ll.h"
#include "hc32_ll_adc.h"
#include "hc32_ll_aos.h"
#include "hc32_ll_fcg.h"
#include "hc32_ll_gpio.h"
#include "hc32_ll_tmr4.h"
#include "hw_board_pins.h"      /* 通道/引脚宏集中 */
#include "cfg_sampling.h"       /* 模式/标定宏 */
#include "TickTimer.h"          /* tickTimer_GetCount */
#include "rtt_log.h"

/*---------------------------------------------------------------------------
 * 内部状态
 *---------------------------------------------------------------------------*/
static volatile uint16_t s_u16AdcItotal = 0;   /* PA3：母线电流 或 U 相电流 */
static volatile uint16_t s_u16AdcIw     = 0;   /* PA4 */
static volatile uint16_t s_u16AdcIv     = 0;   /* PA5 */
static volatile uint16_t s_u16AdcVbus   = 0;   /* PA6 */
static volatile uint32_t s_u32Seq = 0;
static volatile bool     s_bNewSample = false;

/*---------------------------------------------------------------------------
 * 上电零电流偏置校准
 *   点数由 CFG_SAMPLING_OFFSET_CAL_SAMPLES 决定（默认 2000 点 = 100ms）
 *---------------------------------------------------------------------------*/
#define FP_CAL_SAMPLES   CFG_SAMPLING_OFFSET_CAL_SAMPLES

static volatile uint8_t  s_cal_active     = 0u;   /* 1 = 正在校准 */
static volatile uint32_t s_cal_cnt        = 0u;   /* 已累加点数 */
static volatile uint32_t s_cal_sum_itotal = 0u;
static volatile uint32_t s_cal_sum_iw     = 0u;
static volatile uint32_t s_cal_sum_iv     = 0u;

volatile uint16_t g_cal_offset_itotal = 2048u;
volatile uint16_t g_cal_offset_iw     = 2048u;
volatile uint16_t g_cal_offset_iv     = 2048u;
volatile uint32_t g_cal_sample_cnt    = 0u;

/* 校准状态标志（掉电默认 0） */
volatile uint8_t g_motor_is_offset_calibrated = 0u;

/*---------------------------------------------------------------------------
 * ADC 原始值全局镜像
 *---------------------------------------------------------------------------*/
volatile uint16_t g_adc_raw_itotal = 0;
volatile uint16_t g_adc_raw_iw     = 0;
volatile uint16_t g_adc_raw_iv     = 0;
volatile uint16_t g_adc_raw_vbus   = 0;

/* 默认标定（可被 Motor_Sampling_Init 覆盖） */
static MotorSamplingScale_t s_scale = {
    .current_scale  = CFG_SAMPLING_CURRENT_SCALE,
    .current_offset = { CFG_SAMPLING_CURRENT_OFFSET,
                        CFG_SAMPLING_CURRENT_OFFSET,
                        CFG_SAMPLING_CURRENT_OFFSET },
    .vbus_scale     = CFG_SAMPLING_VBUS_SCALE,
    .vbus_ch        = HW_ADC_CH_VBUS,
};

/*---------------------------------------------------------------------------
 * EOCA 中断回调
 *---------------------------------------------------------------------------*/
void Motor_Sampling_EocaCallback(void)
{
#if (CFG_SAMPLING_BOARD_IU_SENSOR != 0)
    /* 板型 1：PA3 为 iu 直接使用；板型 2：PA3 只采样观测，iu 后续由 -iw-iv 重构 */
    s_u16AdcItotal = ADC_GetValue(CM_ADC1, HW_ADC_CH_IU);
#else
    s_u16AdcItotal = ADC_GetValue(CM_ADC1, HW_ADC_CH_ITOTAL);
#endif
    s_u16AdcIw     = ADC_GetValue(CM_ADC1, HW_ADC_CH_IW);
    s_u16AdcIv     = ADC_GetValue(CM_ADC1, HW_ADC_CH_IV);

#if (CFG_SAMPLING_USE_REAL_ADC != 0)
    if (s_scale.vbus_ch != 0U) {
        s_u16AdcVbus = ADC_GetValue(CM_ADC1, s_scale.vbus_ch);
    }
#else
    (void)s_u16AdcVbus;
#endif

    g_adc_raw_itotal = s_u16AdcItotal;
    g_adc_raw_iw     = s_u16AdcIw;
    g_adc_raw_iv     = s_u16AdcIv;
    g_adc_raw_vbus   = s_u16AdcVbus;

    /* 累加校准数据 */
    if (s_cal_active) {
        s_cal_sum_itotal += s_u16AdcItotal;
        s_cal_sum_iw     += s_u16AdcIw;
        s_cal_sum_iv     += s_u16AdcIv;
        s_cal_cnt++;
        g_cal_sample_cnt = s_cal_cnt;
        if (s_cal_cnt >= FP_CAL_SAMPLES) {
            s_cal_active = 0u;
        }
    }

    s_u32Seq++;
    s_bNewSample = true;
}

/*---------------------------------------------------------------------------
 * ADC 配置
 *---------------------------------------------------------------------------*/
static void Motor_Sampling_AdcCfg(void)
{
    stc_adc_init_t stcAdcInit;

    FCG_Fcg3PeriphClockCmd(FCG3_PERIPH_ADC1, ENABLE);

    (void)ADC_StructInit(&stcAdcInit);
    stcAdcInit.u16ScanMode   = ADC_MD_SEQA_SINGLESHOT;
    stcAdcInit.u16Resolution = ADC_RESOLUTION_12BIT;
    stcAdcInit.u16DataAlign  = ADC_DATAALIGN_RIGHT;
    (void)ADC_Init(CM_ADC1, &stcAdcInit);

    FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_AOS, ENABLE);
    AOS_SetTriggerEventSrc(AOS_ADC1_0, EVT_SRC_TMR4_3_SCMP0);

#if (CFG_SAMPLING_BOARD_IU_SENSOR != 0)
    ADC_ChCmd(CM_ADC1, ADC_SEQ_A, HW_ADC_CH_IU, ENABLE);
#else
    ADC_ChCmd(CM_ADC1, ADC_SEQ_A, HW_ADC_CH_ITOTAL, ENABLE);
#endif
    ADC_ChCmd(CM_ADC1, ADC_SEQ_A, HW_ADC_CH_IW, ENABLE);
    ADC_ChCmd(CM_ADC1, ADC_SEQ_A, HW_ADC_CH_IV, ENABLE);

    /* 设置采样时间 */
#if (CFG_SAMPLING_BOARD_IU_SENSOR != 0)
    ADC_SetSampleTime(CM_ADC1, HW_ADC_CH_IU,     CFG_SAMPLING_ADC_SAMPLE_TIME);
#else
    ADC_SetSampleTime(CM_ADC1, HW_ADC_CH_ITOTAL, CFG_SAMPLING_ADC_SAMPLE_TIME);
#endif
    ADC_SetSampleTime(CM_ADC1, HW_ADC_CH_IW,     CFG_SAMPLING_ADC_SAMPLE_TIME);
    ADC_SetSampleTime(CM_ADC1, HW_ADC_CH_IV,     CFG_SAMPLING_ADC_SAMPLE_TIME);

#if (CFG_SAMPLING_USE_REAL_ADC != 0)
    if (s_scale.vbus_ch != 0U) {
        ADC_ChCmd(CM_ADC1, ADC_SEQ_A, s_scale.vbus_ch, ENABLE);
    }
#endif

    ADC_TriggerConfig(CM_ADC1, ADC_SEQ_A, ADC_HARDTRIG_EVT0);
    ADC_TriggerCmd(CM_ADC1, ADC_SEQ_A, ENABLE);
    ADC_IntCmd(CM_ADC1, ADC_INT_EOCA, ENABLE);
}

/*---------------------------------------------------------------------------
 * 对外：初始化
 *---------------------------------------------------------------------------*/
void Motor_Sampling_Init(const MotorSamplingScale_t *scale)
{
    if (scale != NULL) {
        s_scale = *scale;
    }

    Motor_Sampling_AdcCfg();

    s_u32Seq = 0;
    s_bNewSample = false;
    s_cal_active = 0u;
    s_cal_cnt = 0u;
    g_cal_sample_cnt = 0u;

    MAIN_D("[SAMPLING] ADC1 seqA ready (PA3/4/5/6)");
}

/*---------------------------------------------------------------------------
 * 对外：取最近一帧
 *---------------------------------------------------------------------------*/
void Motor_GetSample(MotorSample_t *s)
{
    float raw;
    float iw, iv;

    if (s == NULL) return;

    raw = (float)s_u16AdcItotal;
#if (CFG_SAMPLING_BOARD_IU_SENSOR == 1)
    s->iu      = (raw - s_scale.current_offset[0]) * s_scale.current_scale;
    s->i_total = 0.0f;   /* 该板没有母线电流传感器 */
#elif (CFG_SAMPLING_BOARD_IU_SENSOR == 2)
    /* PA3 只采样不使用（raw 供观测），iu 由 -iw-iv 重构 */
    s->iu      = 0.0f;
    s->i_total = 0.0f;
#else
    s->i_total = (raw - s_scale.current_offset[0]) * s_scale.current_scale;
#endif

    raw = (float)s_u16AdcIw;
    iw = (raw - s_scale.current_offset[1]) * s_scale.current_scale;
    s->iw = iw;

    raw = (float)s_u16AdcIv;
    iv = (raw - s_scale.current_offset[2]) * s_scale.current_scale;
    s->iv = iv;

#if (CFG_SAMPLING_BOARD_IU_SENSOR == 1)
    /* iu 由 PA3 直接采样，不再重构 */
#else
    /* 板型 0/2：iu = 0 - iv - iw */
    s->iu = 0.0f - iv - iw;
#endif

#if (CFG_SAMPLING_USE_REAL_ADC != 0)
    raw = (float)s_u16AdcVbus;
    s->vbus = raw * s_scale.vbus_scale;
#else
    s->vbus = CFG_SAMPLING_SIM_VBUS_V;
#endif

    s->seq = s_u32Seq;
}

bool Motor_Sampling_HasNew(void)
{
    bool b = s_bNewSample;
    s_bNewSample = false;
    return b;
}

/*=============================================================================
 * 上电零电流偏置校准
 *=============================================================================*/
void Motor_Sampling_StartOffsetCal(void)
{
    s_cal_sum_itotal = 0u;
    s_cal_sum_iw     = 0u;
    s_cal_sum_iv     = 0u;
    s_cal_cnt        = 0u;
    g_cal_sample_cnt = 0u;
    s_cal_active     = 1u;
}

bool Motor_Sampling_IsOffsetCalDone(void)
{
    return (s_cal_active == 0u) && (s_cal_cnt >= FP_CAL_SAMPLES);
}

bool Motor_Sampling_ApplyOffsetCalSilent(void)
{
    uint32_t n;
    uint16_t off_i, off_w, off_v;

    if (!Motor_Sampling_IsOffsetCalDone()) return false;

    n = s_cal_cnt;
    if (n == 0u) return false;

    off_i = (uint16_t)((s_cal_sum_itotal + n / 2u) / n);
    off_w = (uint16_t)((s_cal_sum_iw     + n / 2u) / n);
    off_v = (uint16_t)((s_cal_sum_iv     + n / 2u) / n);

    g_cal_offset_itotal = off_i;
    g_cal_offset_iw     = off_w;
    g_cal_offset_iv     = off_v;

    s_scale.current_offset[0] = (float)off_i;
    s_scale.current_offset[1] = (float)off_w;
    s_scale.current_offset[2] = (float)off_v;

    return true;
}

void Motor_Sampling_ApplyOffsetCal(void)
{
    if (!Motor_Sampling_ApplyOffsetCalSilent()) return;

    MAIN_D("[CAL] done: N=%lu", (unsigned long)s_cal_cnt);
#if (CFG_SAMPLING_BOARD_IU_SENSOR == 1)
    MAIN_D("[CAL]   iu    : raw=%4u", (unsigned)g_cal_offset_itotal);
#elif (CFG_SAMPLING_BOARD_IU_SENSOR == 2)
    MAIN_D("[CAL]   iu(raw,unused): raw=%4u", (unsigned)g_cal_offset_itotal);
#else
    MAIN_D("[CAL]   itotal: raw=%4u", (unsigned)g_cal_offset_itotal);
#endif
    MAIN_D("[CAL]   iw    : raw=%4u", (unsigned)g_cal_offset_iw);
    MAIN_D("[CAL]   iv    : raw=%4u", (unsigned)g_cal_offset_iv);
}

bool Motor_Sampling_DoOffsetCalBlocking(uint32_t timeout_ms)
{
    uint32_t t0;

    Motor_Sampling_StartOffsetCal();
    t0 = tickTimer_GetCount();

    while (!Motor_Sampling_IsOffsetCalDone()) {
        if ((tickTimer_GetCount() - t0) > timeout_ms) {
            return false; /* 超时 */
        }
    }

    Motor_Sampling_ApplyOffsetCal();
    g_motor_is_offset_calibrated = 1u;
    return true; /* 成功 */
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
