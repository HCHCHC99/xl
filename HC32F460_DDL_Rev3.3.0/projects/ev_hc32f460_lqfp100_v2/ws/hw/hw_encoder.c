/**
 *******************************************************************************
 * @file  hw_encoder.c
 * @brief ABZ quadrature encoder via TIMERA_1 hardware quadrature count.
 *******************************************************************************
 */

#include "hw_encoder.h"
#include "hc32_ll.h"
#include "timer6_timebase.h"
#include "hw_board_pins.h"
#include "cfg_motor.h"
#include "rtt_log.h"

#ifndef ENCODER_CPR
#define ENCODER_CPR    CFG_ENCODER_CPR
#endif

#define ENC_SPEED_ALPHA         (0.2f)
#define ENC_DIR_DEADBAND_RPM    (5.0f)
#define ENC_DIR_HYST_RPM        (10.0f)
#define ENC_SPEED_WIN_US        (10000u)
#define ENC_TWO_PI              6.28318530f

/*=============================================================================
 * 可观测变量
 *=============================================================================*/
volatile int32_t  g_enc_count     = 0;
volatile uint8_t  g_enc_dbg_print = 0;
volatile float    g_enc_count_f   = 0.0f;
volatile float    g_enc_angle_deg = 0.0f;
volatile float    g_enc_speed_rpm = 0.0f;
volatile int8_t   g_enc_dir       = 0;
volatile uint32_t g_enc_rev       = 0;
volatile int32_t  g_enc_elec_offset = 0;
volatile int8_t   g_enc_elec_dir    = 1;   /* 默认 +1，由对齐流程按 moved 符号设置 */

/*=============================================================================
 * 内部状态
 *=============================================================================*/
static volatile int32_t  s_count    = 0;
static volatile int32_t  s_prev_hw  = 0;
static volatile uint8_t  s_isr_init = 0;
static volatile int32_t  s_cnt_sum  = 0;
static volatile uint32_t s_us_sum   = 0;
static volatile uint64_t s_last_us  = 0;

static int32_t s_last_pos  = 0;
static float   s_speed_rpm = 0.0f;
static int8_t  s_dir       = 0;
static uint8_t s_inited    = 0;

static void hw_encoder_z_isr(void)
{
    EXTINT_ClearExtIntStatus(HW_ENC_Z_EIRQ);
    g_enc_rev++;
}

void hw_encoder_init(void)
{
    stc_extint_init_t       stcExti;
    stc_irq_signin_config_t stcIrq;
    stc_tmra_init_t         stcTmra;

    if (s_inited) {
        return;
    }

    LL_PERIPH_WE(LL_PERIPH_GPIO);

    stcExti.u32Edge        = EXTINT_TRIG_RISING;
    stcExti.u32Filter      = EXTINT_FILTER_ON;
    stcExti.u32FilterClock = EXTINT_FCLK_DIV64;
    EXTINT_Init(HW_ENC_Z_EIRQ, &stcExti);
    GPIO_ExtIntCmd(HW_PIN_ENC_Z_PORT, HW_PIN_ENC_Z_PIN, ENABLE);

    LL_PERIPH_WP(LL_PERIPH_GPIO);

    stcIrq.enIntSrc    = HW_ENC_Z_IRQ_SRC;
    stcIrq.enIRQn      = HW_ENC_Z_IRQn;
    stcIrq.pfnCallback = &hw_encoder_z_isr;
    if (LL_OK != INTC_IrqSignIn(&stcIrq)) {
        MAIN_D("[HW_ENC] ERROR: INTC_IrqSignIn failed for Z index\r\n");
        return;
    }
    NVIC_ClearPendingIRQ(HW_ENC_Z_IRQn);
    NVIC_SetPriority(HW_ENC_Z_IRQn, DDL_IRQ_PRIO_03);
    NVIC_EnableIRQ(HW_ENC_Z_IRQn);

    FCG_Fcg2PeriphClockCmd(FCG2_PERIPH_TMRA_1, ENABLE);
    TMRA_StructInit(&stcTmra);
    stcTmra.u8CountSrc                = TMRA_CNT_SRC_HW;
    stcTmra.sw_count.u8CountMode      = TMRA_MD_SAWTOOTH;
    stcTmra.sw_count.u8CountDir       = TMRA_DIR_UP;
    stcTmra.hw_count.u16CountUpCond   = TMRA_CNT_UP_COND_INVD;
    stcTmra.hw_count.u16CountDownCond = TMRA_CNT_DOWN_COND_INVD;
    stcTmra.u32PeriodValue            = 0xFFFFu;
    (void)TMRA_Init(CM_TMRA_1, &stcTmra);

    TMRA_HWCountUpCondCmd(CM_TMRA_1, TMRA_CNT_UP_COND_CLKB_LOW_CLKA_RISING,     ENABLE);
    TMRA_HWCountUpCondCmd(CM_TMRA_1, TMRA_CNT_UP_COND_CLKB_HIGH_CLKA_FALLING,   ENABLE);
    TMRA_HWCountUpCondCmd(CM_TMRA_1, TMRA_CNT_UP_COND_CLKA_HIGH_CLKB_RISING,    ENABLE);
    TMRA_HWCountUpCondCmd(CM_TMRA_1, TMRA_CNT_UP_COND_CLKA_LOW_CLKB_FALLING,    ENABLE);

    TMRA_HWCountDownCondCmd(CM_TMRA_1, TMRA_CNT_DOWN_COND_CLKB_HIGH_CLKA_RISING,  ENABLE);
    TMRA_HWCountDownCondCmd(CM_TMRA_1, TMRA_CNT_DOWN_COND_CLKB_LOW_CLKA_FALLING,  ENABLE);
    TMRA_HWCountDownCondCmd(CM_TMRA_1, TMRA_CNT_DOWN_COND_CLKA_LOW_CLKB_RISING,   ENABLE);
    TMRA_HWCountDownCondCmd(CM_TMRA_1, TMRA_CNT_DOWN_COND_CLKA_HIGH_CLKB_FALLING, ENABLE);

    TMRA_SetCountValue(CM_TMRA_1, 0u);
    TMRA_Start(CM_TMRA_1);

    s_count    = 0;
    s_prev_hw  = 0;
    s_isr_init = 0;
    s_last_pos = 0;
    s_cnt_sum  = 0;
    s_us_sum   = 0;
    s_last_us  = 0u;

    g_enc_count   = 0;
    g_enc_count_f = 0.0f;
    g_enc_rev     = 0;
    g_enc_dir     = 0;
    g_enc_speed_rpm = 0.0f;
    g_enc_angle_deg = 0.0f;

    s_inited = 1;

    MAIN_D("[HW_ENC] ABZ init: TMRA_1 quad x4 (CPR=%lu), Z rev-count only",
           (unsigned long)ENCODER_CPR);
}

void hw_encoder_update_count_from_isr(void)
{
    uint16_t hw;
    int16_t  delta;

    if (!s_inited) {
        return;
    }

    hw = (uint16_t)TMRA_GetCountValue(CM_TMRA_1);

    if (!s_isr_init) {
        s_prev_hw  = (int32_t)hw;
        s_isr_init = 1u;
        return;
    }

    delta = (int16_t)(hw - (uint16_t)s_prev_hw);
    s_prev_hw = (int32_t)hw;
    s_count  += (int32_t)delta;

    g_enc_count   = s_count;
    g_enc_count_f = (float)s_count;
}

/* 机械角 [0, 2π) */
float hw_encoder_get_mech_angle_rad(void)
{
    int32_t c = s_count % (int32_t)ENCODER_CPR;

    if (c < 0) {
        c += (int32_t)ENCODER_CPR;
    }
    return (float)c * (ENC_TWO_PI / (float)ENCODER_CPR);
}

/* 电角度 [0, 2π)
 *   1) 先扣 offset，折回 [0, CPR)
 *   2) 乘极对数折成电角度
 *   3) 按 g_enc_elec_dir 翻转（-1 时 e = 2π - e）
 *   4) 折回 [0, 2π)
 *
 * offset 和 dir 由 mode 3 对齐写入：
 *   offset = alpha_hw（ALPHA 0° 时的计数）
 *   dir    = moved > 0 ? -1 : +1
 *     moved > 0：BETA(90°)->ALPHA(0°) 电角度减小，count 增
 *                => 计数增对应电角度减 => dir = -1
 *     moved < 0：count 减 => 计数增对应电角度增 => dir = +1
 */
float hw_encoder_get_elec_angle_rad(uint8_t pole_pairs)
{
    int32_t c = (s_count - g_enc_elec_offset) % (int32_t)ENCODER_CPR;
    float   e;

    if (c < 0) {
        c += (int32_t)ENCODER_CPR;
    }

    e = (float)c * (ENC_TWO_PI / (float)ENCODER_CPR) * (float)pole_pairs;

    /* 方向翻转：-1 时 e -> 2π - e */
    if (g_enc_elec_dir < 0) {
        e = ENC_TWO_PI - e;
    }

    /* 折回 [0, 2π) */
    if (e >= ENC_TWO_PI) {
        e -= ENC_TWO_PI * (float)(int32_t)(e * (1.0f / ENC_TWO_PI));
        if (e >= ENC_TWO_PI) {
            e -= ENC_TWO_PI;
        }
    }
    if (e < 0.0f) {
        e += ENC_TWO_PI;
    }
    return e;
}

void hw_encoder_update(void)
{
    uint64_t now_us;
    int32_t  d_cnt;
    uint32_t d_us;

    if (!s_inited) {
        return;
    }

    Timer6_Timebase_UpdateTimestamp();
    now_us = Timer6_Timebase_GetTimestamp();

    d_cnt = g_enc_count - s_last_pos;
    s_last_pos = g_enc_count;

    d_us = (s_last_us == 0u) ? 1000u : (uint32_t)(now_us - s_last_us);
    if (d_us > 1000000u) {
        d_us = 1000000u;
    }
    s_last_us = now_us;

    s_cnt_sum += d_cnt;
    s_us_sum  += d_us;

    if (s_us_sum >= ENC_SPEED_WIN_US) {
        float rpm = (float)s_cnt_sum * 60000000.0f
                  / ((float)s_us_sum * (float)ENCODER_CPR);
        s_speed_rpm += ENC_SPEED_ALPHA * (rpm - s_speed_rpm);

        if (s_dir == 0) {
            if (s_speed_rpm >  ENC_DIR_DEADBAND_RPM) s_dir =  1;
            else if (s_speed_rpm < -ENC_DIR_DEADBAND_RPM) s_dir = -1;
        } else if (s_dir == 1) {
            if (s_speed_rpm < -ENC_DIR_HYST_RPM) s_dir = -1;
        } else {
            if (s_speed_rpm >  ENC_DIR_HYST_RPM) s_dir =  1;
        }
        s_cnt_sum = 0;
        s_us_sum  = 0;
    }

    g_enc_speed_rpm = s_speed_rpm;
    g_enc_dir       = s_dir;
    g_enc_angle_deg = hw_encoder_get_mech_angle_rad() * (360.0f / ENC_TWO_PI);
}

int32_t  hw_encoder_get_count(void)      { return g_enc_count; }
float    hw_encoder_get_angle_deg(void)  { return g_enc_angle_deg; }
float    hw_encoder_get_speed_rpm(void)  { return g_enc_speed_rpm; }
int8_t   hw_encoder_get_direction(void)  { return g_enc_dir; }
uint32_t hw_encoder_get_rev_count(void)  { return g_enc_rev; }

/*******************************************************************************
 * EOF
 ******************************************************************************/
