/**
 *******************************************************************************
 * @file  hw_encoder.c
 * @brief ABZ quadrature encoder via TMRA1 hardware quadrature count.
 *
 *        A = PA8/TIMA1_CLKA, B = PA9/TIMA1_CLKB, Z = PA10/EXTINT_CH10
 *        1024 lines x 4 = 4096 counts / rev
 *******************************************************************************
 */

#include "hw_encoder.h"
#include "cfg_motor.h"
#include "board_config.h"
#include "hc32_ll.h"

#include <rtthread.h>
#include <finsh.h>

/* 20kHz ISR 每拍读取计数/电角度：只对本文件开 O2。 */
#pragma GCC optimize("O2")

#if !BSP_USING_ABS_ENCODER
/* ------------------------------------------------------------
 * BSP_USING_ABS_ENCODER = 0：保留 SPI 编码器方案，
 * ABZ 接口退化为空实现，保证 FOC 控制层可编译。
 * ------------------------------------------------------------ */
volatile int32_t  g_enc_count     = 0;
volatile uint8_t  g_enc_dbg_print = 0;
volatile float    g_enc_count_f   = 0.0f;
volatile float    g_enc_angle_deg = 0.0f;
volatile float    g_enc_speed_rpm = 0.0f;
volatile int8_t   g_enc_dir       = 0;
volatile uint32_t g_enc_rev       = 0;
volatile int32_t  g_enc_elec_offset = 0;
volatile int8_t   g_enc_elec_dir    = 1;

void hw_encoder_init(void) {}
void hw_encoder_update_count_from_isr(void) {}
void hw_encoder_update(void) {}
float hw_encoder_get_mech_angle_rad(void) { return 0.0f; }
float hw_encoder_get_elec_angle_rad(uint8_t pole_pairs) { (void)pole_pairs; return 0.0f; }
int32_t hw_encoder_get_count(void) { return 0; }
float hw_encoder_get_angle_deg(void) { return 0.0f; }
float hw_encoder_get_speed_rpm(void) { return 0.0f; }
int8_t hw_encoder_get_direction(void) { return 0; }
uint32_t hw_encoder_get_rev_count(void) { return 0u; }

#else  /* BSP_USING_ABS_ENCODER */

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
volatile int8_t   g_enc_elec_dir    = 1;   /* 默认 +1，由 mode 3 对齐写入 */

/*=============================================================================
 * 内部状态
 *=============================================================================*/
static volatile int32_t  s_count    = 0;
static volatile int32_t  s_prev_hw  = 0;
static volatile uint8_t  s_isr_init = 0;
static volatile int32_t  s_cnt_sum  = 0;
static volatile uint32_t s_us_sum   = 0;
static uint32_t s_last_ms = 0u;

static int32_t s_last_pos  = 0;
static float   s_speed_rpm = 0.0f;
static int8_t  s_dir       = 0;
static uint8_t s_inited    = 0;

static struct rt_timer s_enc_timer;

static void hw_encoder_z_isr(void)
{
    EXTINT_ClearExtIntStatus(ENC_Z_EIRQ);
    g_enc_rev++;
}

static void hw_encoder_timer_cb(void *param)
{
    (void)param;
    hw_encoder_update();
}

void hw_encoder_init(void)
{
    stc_gpio_init_t         stcGpio;
    stc_extint_init_t       stcExti;
    stc_irq_signin_config_t stcIrq;
    stc_tmra_init_t         stcTmra;

    if (s_inited) {
        return;
    }

    /* ---- PA8/PA9 复用为 TIMA1_CLKA/CLKB，PA10 数字输入 ---- */
    LL_PERIPH_WE(LL_PERIPH_GPIO);
    GPIO_SetFunc(ENC_A_PORT, ENC_A_PIN, ENC_A_FUNC);
    GPIO_SetFunc(ENC_B_PORT, ENC_B_PIN, ENC_B_FUNC);

    GPIO_StructInit(&stcGpio);
    stcGpio.u16PinDir  = PIN_DIR_IN;
    stcGpio.u16PinAttr = PIN_ATTR_DIGITAL;
    stcGpio.u16ExtInt  = PIN_EXTINT_ON;      /* DDL 3.0.0：EXTINT 挂在 GPIO_Init 里 */
    GPIO_Init(ENC_Z_PORT, ENC_Z_PIN, &stcGpio);
    LL_PERIPH_WP(LL_PERIPH_GPIO);

    /* ---- Z 索引：上升沿中断，只做圈计数 ---- */
    LL_PERIPH_WE(LL_PERIPH_GPIO);
    stcExti.u32Edge        = EXTINT_TRIG_RISING;
    stcExti.u32Filter      = EXTINT_FILTER_ON;
    stcExti.u32FilterClock = EXTINT_FCLK_DIV64;
    EXTINT_Init(ENC_Z_EIRQ, &stcExti);
    LL_PERIPH_WP(LL_PERIPH_GPIO);

    stcIrq.enIntSrc    = ENC_Z_IRQ_SRC;
    stcIrq.enIRQn      = ENC_Z_IRQn;
    stcIrq.pfnCallback = &hw_encoder_z_isr;
    if (LL_OK != INTC_IrqSignIn(&stcIrq)) {
        /* Z 索引不可用不阻断 A/B 正交计数：没接 ABZ 时也要能跑完
         * mode 3 的两次强拖（计数保持 0 即可）。 */
        rt_kprintf("[HW_ENC] WARN: Z index IRQ sign-in failed, quad count only\n");
    } else {
        NVIC_ClearPendingIRQ(ENC_Z_IRQn);
        NVIC_SetPriority(ENC_Z_IRQn, DDL_IRQ_PRIO_03);
        NVIC_EnableIRQ(ENC_Z_IRQn);
    }

    /* ---- TMRA1 正交计数 ---- */
    FCG_Fcg2PeriphClockCmd(FCG2_PERIPH_TMRA_1, ENABLE);
    TMRA_StructInit(&stcTmra);
    stcTmra.u8CountSrc                = TMRA_CNT_SRC_HW;
    stcTmra.sw_count.u16CountMode     = TMRA_MD_SAWTOOTH;
    stcTmra.sw_count.u16CountDir      = TMRA_DIR_UP;
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
    s_last_ms  = 0u;

    g_enc_count   = 0;
    g_enc_count_f = 0.0f;
    g_enc_rev     = 0;
    g_enc_dir     = 0;
    g_enc_speed_rpm = 0.0f;
    g_enc_angle_deg = 0.0f;

    s_inited = 1;

    /* 10ms 诊断测速，FOC 速度环仍用 20kHz 滑窗 */
    rt_timer_init(&s_enc_timer,
                  "abz_enc",
                  hw_encoder_timer_cb,
                  RT_NULL,
                  rt_tick_from_millisecond(10),
                  RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    rt_timer_start(&s_enc_timer);

    rt_kprintf("[HW_ENC] ABZ init: TMRA_1 quad x4 (CPR=%lu), Z rev-count only\n",
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

    if (g_enc_elec_dir < 0) {
        c = (int32_t)ENCODER_CPR - c;
        if (c >= (int32_t)ENCODER_CPR) {
            c -= (int32_t)ENCODER_CPR;
        }
    }

    return (float)c * (ENC_TWO_PI / (float)ENCODER_CPR);
}

/* 电角度 [0, 2π) */
float hw_encoder_get_elec_angle_rad(uint8_t pole_pairs)
{
    int32_t c = (s_count - g_enc_elec_offset) % (int32_t)ENCODER_CPR;
    float   e;

    if (c < 0) {
        c += (int32_t)ENCODER_CPR;
    }

    e = (float)c * (ENC_TWO_PI / (float)ENCODER_CPR) * (float)pole_pairs;

    if (g_enc_elec_dir < 0) {
        e = -e;
    }

    e -= ENC_TWO_PI * (float)(int32_t)(e * (1.0f / ENC_TWO_PI));
    if (e < 0.0f) {
        e += ENC_TWO_PI;
    }
    if (e >= ENC_TWO_PI) {
        e -= ENC_TWO_PI;
    }
    return e;
}

void hw_encoder_update(void)
{
    uint32_t now_ms;
    uint32_t d_ms;
    uint32_t d_us;
    int32_t  d_cnt;

    if (!s_inited) {
        return;
    }

    now_ms = rt_tick_get();
    d_cnt  = g_enc_count - s_last_pos;
    s_last_pos = g_enc_count;

    d_ms = (s_last_ms == 0u) ? 10u : (uint32_t)(now_ms - s_last_ms);
    if (d_ms > 1000u) {
        d_ms = 1000u;
    }
    d_us = (uint32_t)(((uint64_t)d_ms * 1000000ull) / RT_TICK_PER_SECOND);
    s_last_ms = now_ms;

    s_cnt_sum += d_cnt;
    s_us_sum  += d_us;

    if (s_us_sum >= ENC_SPEED_WIN_US) {
        float rpm = (float)s_cnt_sum * 60000000.0f
                  / ((float)s_us_sum * (float)ENCODER_CPR);

        rpm *= (float)g_enc_elec_dir;
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

/*=============================================================================
 * msh 诊断
 *=============================================================================*/
static void cmd_t_abz(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    rt_kprintf("[ABZ] cnt=%d rev=%u mech=%.2f elec=%.2f rpm=%.1f dir=%d\n",
               (int)hw_encoder_get_count(),
               (unsigned)hw_encoder_get_rev_count(),
               hw_encoder_get_mech_angle_rad() * 57.29578f,
               hw_encoder_get_elec_angle_rad(CFG_MOTOR_POLE_PAIRS) * 57.29578f,
               hw_encoder_get_speed_rpm(),
               (int)hw_encoder_get_direction());
}
MSH_CMD_EXPORT_ALIAS(cmd_t_abz, t_abz, ABZ encoder status);
MSH_CMD_EXPORT_ALIAS(cmd_t_abz, abz,   ABZ encoder status);

#endif /* BSP_USING_ABS_ENCODER */
