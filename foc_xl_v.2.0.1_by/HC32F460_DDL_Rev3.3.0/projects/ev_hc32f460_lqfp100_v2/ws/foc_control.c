/*=============================================================================
 * foc_control.c  —  L5 控制层实现（20 kHz EOCA 中断宿主）
 *=============================================================================*/

#include "foc_control.h"
#include "motor_sampling.h"
#include "hw_encoder.h"
#include "hc32_ll.h"
#include "hc32_ll_adc.h"
#include "timer6_timebase.h"
#include "rtt_log.h"
#include "foc_protect.h"

#include "foc_mode.h"
#include "clarke_park.h"
#include "svpwm.h"
#include "hw_tmr4_pwm.h"
#include "cfg_sampling.h"
#include "cfg_fixed.h"
#include "foc_fx.h"
#include "encoder_calib.h"
#include "hw_board_pins.h"

/*---------------------------------------------------------------------------
 * 节拍测量用的常量
 *---------------------------------------------------------------------------*/
#define FC_DT_SANE_MIN_US   10u
#define FC_DT_SANE_MAX_US   200u

/* ISR 节拍 = 20 kHz */
#define FC_ISR_TS_S         (1.0f / 20000.0f)

/* 哨兵：标记上一拍走了 enable=true 分支 */
#define FC_LAST_MODE_IN_IF  ((foc_mode_t)0xFE)

/*=============================================================================
 * Keil Watch 观测量
 *=============================================================================*/
volatile uint32_t g_dbg_cnt_at_isr    = 0;
volatile uint32_t g_fc_isr_cnt        = 0;
volatile uint32_t g_fc_isr_dt_max_us  = 0;
volatile uint32_t g_fc_isr_dt_min_us  = 0xFFFFFFFFu;
volatile uint32_t g_fc_isr_dt_last_us = 0;

volatile int32_t  g_fc_enc_count      = 0;
volatile int32_t  g_fc_enc_delta      = 0;
volatile float    g_fc_enc_mech_deg   = 0.0f;
volatile float    g_fc_enc_elec_deg   = 0.0f;
volatile float    g_fc_enc_speed_rpm  = 0.0f;

volatile float    g_fc_i_total        = 0.0f;
volatile float    g_fc_iu             = 0.0f;
volatile float    g_fc_iv             = 0.0f;
volatile float    g_fc_iw             = 0.0f;
volatile float    g_fc_vbus           = 0.0f;

volatile float    g_fc_id             = 0.0f;
volatile float    g_fc_iq             = 0.0f;

/* Q15 反馈（定点快路径 / 影子对比用） */
volatile int16_t  g_fc_id_q15         = 0;
volatile int16_t  g_fc_iq_q15         = 0;

/* 浮点/Q15 影子对比误差，单位 Q15 LSB（1 个 ADC LSB ≈ 27.5 LSB） */
volatile uint32_t g_fx_id_err_max_lsb = 0;
volatile uint32_t g_fx_iq_err_max_lsb = 0;
volatile int16_t  g_fx_id_err_last_lsb = 0;
volatile int16_t  g_fx_iq_err_last_lsb = 0;

volatile uint32_t g_fc_sample_seq     = 0;

/*---------------------------------------------------------------------------
 * DWT 周期计数：ISR 计算耗时（不含异常出入栈）
 *   HCLK = 200MHz 时 1 count = 5ns
 *   g_dwt_isr_avg_cyc 每 1024 拍更新一次（约 51ms）
 *---------------------------------------------------------------------------*/
volatile uint32_t g_dwt_isr_last_cyc = 0;
volatile uint32_t g_dwt_isr_min_cyc  = 0xFFFFFFFFu;
volatile uint32_t g_dwt_isr_max_cyc  = 0;
volatile uint32_t g_dwt_isr_avg_cyc  = 0;
volatile uint32_t g_dwt_isr_win_cnt  = 0;
static uint32_t   s_dwt_sum_cyc      = 0;
static uint8_t    s_dwt_inited       = 0;

/*---------------------------------------------------------------------------
 * 分段耗时（FOC_DWT_SEG=1 时有效）
 *   [0] 编码器 + 采样 + 过流搬运
 *   [1] Clarke / Park
 *   [2] Foc_Mode_Step（含电流环 + 前馈 + 限幅）
 *   [3] 输出链（InvPark + SVPWM + 写占空比）
 *   每 1024 拍刷新一次平均；对齐期间提前 return，[2][3] 不计入
 *---------------------------------------------------------------------------*/
#if (FOC_DWT_SEG != 0)
volatile uint32_t g_dwt_seg_avg_cyc[4] = { 0u, 0u, 0u, 0u };
static uint32_t   s_dwt_seg_sum[4]     = { 0u, 0u, 0u, 0u };
#endif

/* Mode 3 对齐期间标志 */
volatile uint8_t  g_align_active      = 0u;

/*---------------------------------------------------------------------------
 * 内部状态
 *---------------------------------------------------------------------------*/
static volatile uint8_t  s_enabled     = 0;
static volatile uint16_t s_prev_ts_cnt = 0;
static volatile uint8_t  s_ts_init     = 0;
static volatile int32_t  s_prev_enc    = 0;
static uint32_t          s_ticks_per_us = 1u;

static foc_mode_t        s_last_static_mode = (foc_mode_t)0xFF;
static uint8_t           s_if_resumed = 0u;

/*=============================================================================
 * 节拍采样
 *=============================================================================*/
static void Foc_Control_MeasureDt(void)
{
    uint16_t cnt = (uint16_t)Timer6_Timebase_GetCounter();
    uint16_t d;

    if (!s_ts_init) {
        s_prev_ts_cnt = cnt;
        s_ts_init = 1u;
        return;
    }

    d = (uint16_t)(cnt - s_prev_ts_cnt);
    s_prev_ts_cnt = cnt;

    {
        uint32_t us = (uint32_t)d / s_ticks_per_us;

        if ((us >= FC_DT_SANE_MIN_US) && (us <= FC_DT_SANE_MAX_US)) {
            g_fc_isr_dt_last_us = us;
            if (us > g_fc_isr_dt_max_us) g_fc_isr_dt_max_us = us;
            if (us < g_fc_isr_dt_min_us) g_fc_isr_dt_min_us = us;
        }
    }
}

/*=============================================================================
 * DWT 周期计数：初始化与统计
 *   只统计 ISR 内的软件计算时间，不含 Cortex-M4 异常入栈/出栈
 *=============================================================================*/
static void Foc_Control_DwtInit(void)
{
    if (s_dwt_inited) {
        return;
    }

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0u;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

    s_dwt_inited = 1u;
}

static void Foc_Control_DwtRecord(uint32_t cyc)
{
    g_dwt_isr_last_cyc = cyc;

    if (cyc < g_dwt_isr_min_cyc) g_dwt_isr_min_cyc = cyc;
    if (cyc > g_dwt_isr_max_cyc) g_dwt_isr_max_cyc = cyc;

    s_dwt_sum_cyc += cyc;
    g_dwt_isr_win_cnt++;
    if (g_dwt_isr_win_cnt >= 1024u) {
        g_dwt_isr_avg_cyc = s_dwt_sum_cyc >> 10;   /* /1024 */
        s_dwt_sum_cyc     = 0u;
        g_dwt_isr_win_cnt = 0u;

#if (FOC_DWT_SEG != 0)
        {
            uint32_t i;

            for (i = 0u; i < 4u; i++) {
                g_dwt_seg_avg_cyc[i] = s_dwt_seg_sum[i] >> 10;
                s_dwt_seg_sum[i]     = 0u;
            }
        }
#endif
    }
}

void Foc_Control_Step(void)
{
    MotorSample_t s;
    int32_t enc_now;
    float   mech;
    float   theta_e;
#if (FOC_USE_FIXED == 0)
    float   ialpha, ibeta;
#endif
#if (FOC_DWT_SEG != 0)
    uint32_t ts;
#endif

    g_fc_isr_cnt++;

#if (FOC_DWT_SEG != 0)
    ts = DWT->CYCCNT;
#endif

    Foc_Control_MeasureDt();

    /* ---- 1. 编码器（对齐期间也要更新，保证 DoAlignBlocking 读到最新计数）---- */
    hw_encoder_update_count_from_isr();

    enc_now = hw_encoder_get_count();
    g_fc_enc_delta = enc_now - s_prev_enc;
    s_prev_enc     = enc_now;
    g_fc_enc_count = enc_now;

    mech = hw_encoder_get_mech_angle_rad();
    g_fc_enc_mech_deg = mech * 57.29578f;

    /* 电角度：IDLE/对齐期间也持续更新，保证 g_fc_enc_elec_deg 实时跟踪 */
    theta_e = hw_encoder_get_elec_angle_rad(FOC_CONTROL_POLE_PAIRS);
    g_fc_enc_elec_deg = theta_e * 57.29578f;

    /* ---- 2. 电流采样 ---- */
    Motor_GetSample(&s);
    g_fc_i_total = s.i_total;
    g_fc_iu      = s.iu;
    g_fc_iv      = s.iv;
    g_fc_iw      = s.iw;
    g_fc_vbus    = s.vbus;
    g_fc_sample_seq = s.seq;

    /* ---- 2b. 过流保护搬运 ---- */
    Foc_Protect_PushIsr(g_fc_iu, g_fc_iv, g_fc_iw);

#if (FOC_DWT_SEG != 0)
    s_dwt_seg_sum[0] += DWT->CYCCNT - ts;
    ts = DWT->CYCCNT;
#endif

    /* ---- 3. Clarke & Park ---- */
#if (FOC_USE_FIXED != 0)
    {
        MotorSampleQ15_t sq;
        int16_t  ia15, ib15, id15, iq15;
        uint32_t ph = hw_encoder_get_elec_phase(FOC_CONTROL_POLE_PAIRS);

        Motor_GetSampleQ15(&sq);
        ClarkeQ15(sq.iu, sq.iv, sq.iw, &ia15, &ib15);
        ParkQ15(ia15, ib15, ph, &id15, &iq15);

        g_fc_id_q15 = id15;
        g_fc_iq_q15 = iq15;
        g_fc_id = FX_Q15_TO_CUR(id15);
        g_fc_iq = FX_Q15_TO_CUR(iq15);
    }
#else
    Clarke(g_fc_iu, g_fc_iv, g_fc_iw, &ialpha, &ibeta);
    Park(ialpha, ibeta, theta_e, (float *)&g_fc_id, (float *)&g_fc_iq);
#endif

#if (FOC_USE_FIXED == 0) && (FOC_FX_SHADOW != 0)
    {
        MotorSampleQ15_t sq;
        int16_t  ia15, ib15, id15, iq15;
        uint32_t ph = hw_encoder_get_elec_phase(FOC_CONTROL_POLE_PAIRS);
        int32_t  e;

        Motor_GetSampleQ15(&sq);
        ClarkeQ15(sq.iu, sq.iv, sq.iw, &ia15, &ib15);
        ParkQ15(ia15, ib15, ph, &id15, &iq15);

        g_fc_id_q15 = id15;
        g_fc_iq_q15 = iq15;

        e = (int32_t)id15 - FX_CUR_TO_Q15(g_fc_id);
        g_fx_id_err_last_lsb = (int16_t)e;
        if (e < 0) e = -e;
        if ((uint32_t)e > g_fx_id_err_max_lsb) g_fx_id_err_max_lsb = (uint32_t)e;

        e = (int32_t)iq15 - FX_CUR_TO_Q15(g_fc_iq);
        g_fx_iq_err_last_lsb = (int16_t)e;
        if (e < 0) e = -e;
        if ((uint32_t)e > g_fx_iq_err_max_lsb) g_fx_iq_err_max_lsb = (uint32_t)e;
    }
#endif

#if (FOC_DWT_SEG != 0)
    s_dwt_seg_sum[1] += DWT->CYCCNT - ts;
    ts = DWT->CYCCNT;
#endif

    /* ---- 3b. Mode 3 对齐期间：ISR 不碰 PWM ----
     *   编码器/电流/角度已更新完（第 1~3 步），
     *   这里 return，跳过 Foc_Mode_Step + set_duty3，
     *   让主循环 DoAlignBlocking 的 set_duty3 生效 */
    if (g_align_active) {
        EncoderCalib_Task20k();   /* ZERO 阶段零偏采样，非 ZERO 直接 return */
        return;
    }

    /* ---- 4. 控制链 ---- */
    {
        FocCmd_t  cmd;
#if (FOC_USE_FIXED == 0)
        float     valpha, vbeta;
        SvpwmCtx_t ctx;
#endif

        Foc_Mode_Step(FC_ISR_TS_S, &cmd);

#if (FOC_DWT_SEG != 0)
        s_dwt_seg_sum[2] += DWT->CYCCNT - ts;
        ts = DWT->CYCCNT;
#endif

        if (cmd.enable) {
            if (!s_if_resumed) {
                s_if_resumed = 1u;
                hw_tmr4_pwm_resume_output();
            }

            s_last_static_mode = FC_LAST_MODE_IN_IF;

#if (FOC_USE_FIXED != 0)
            /* 定点输出链：dq 电压 Q15 -> InvParkQ15 -> SVPWM Q15 -> 比较值
             *   相位先乘一半再左移，避开 float 在 2^32 附近的溢出（丢 1 LSB） */
            {
                int16_t  vd15 = Fx_Sat16(FX_VOL_TO_Q15(cmd.vd));
                int16_t  vq15 = Fx_Sat16(FX_VOL_TO_Q15(cmd.vq));
                int16_t  va15, vb15, du15, dv15, dw15;
                uint32_t ph   = (uint32_t)(cmd.theta_e * (FX_PHASE_PER_RAD * 0.5f)) << 1;

                InvParkQ15(vd15, vq15, ph, &va15, &vb15);
                SVPWM_CalcQ15(va15, vb15, &du15, &dv15, &dw15);
                hw_tmr4_pwm_set_duty_q15(du15, dv15, dw15);
            }
#else
            InvPark(cmd.vd, cmd.vq, cmd.theta_e, &valpha, &vbeta);

            ctx.valpha = valpha;
            ctx.vbeta  = vbeta;
            ctx.vbus   = CFG_SAMPLING_SIM_VBUS_V;
            SVPWM_Calc(&ctx);

            hw_tmr4_pwm_set_duty3(ctx.duty_u * 100.0f,
                                  ctx.duty_v * 100.0f,
                                  ctx.duty_w * 100.0f);
#endif
        } else {
            s_if_resumed = 0u;

            if (s_last_static_mode != g_fm_mode) {
                s_last_static_mode = g_fm_mode;

                if (g_fm_mode == FOC_MODE_IDLE) {
                    hw_tmr4_pwm_set_all_low();
                } else {
                    hw_tmr4_pwm_resume_output();
                    hw_tmr4_pwm_set_duty3(2.0f, 2.0f, 2.0f);
                }
            }
        }

#if (FOC_DWT_SEG != 0)
        s_dwt_seg_sum[3] += DWT->CYCCNT - ts;
#endif
    }
}

static void Foc_Control_EocaHandler(void)
{
    uint32_t t0 = DWT->CYCCNT;

    g_dbg_cnt_at_isr = TMR4_GetCountValue(CM_TMR4_3);
    Motor_Sampling_EocaCallback();
    Foc_Control_Step();

    Foc_Control_DwtRecord(DWT->CYCCNT - t0);
}

void Foc_Control_Init(void)
{
    stc_irq_signin_config_t stcIrq;
    uint32_t freq;

    freq = Timer6_Timebase_GetFrequency();
    if (freq < 1000000u) freq = 1000000u;
    s_ticks_per_us = freq / 1000000u;
    if (s_ticks_per_us == 0u) s_ticks_per_us = 1u;

    Foc_Control_DwtInit();
    Foc_Fx_Init();
    Foc_Control_ResetStats();

    stcIrq.enIntSrc    = INT_SRC_ADC1_EOCA;
    stcIrq.enIRQn      = INT116_IRQn;
    stcIrq.pfnCallback = &Foc_Control_EocaHandler;
    if (LL_OK != INTC_IrqSignIn(&stcIrq)) {
        MAIN_E("[FC] INTC_IrqSignIn failed for EOCA");
        return;
    }

    NVIC_ClearPendingIRQ(INT116_IRQn);
    NVIC_SetPriority(INT116_IRQn, FOC_CONTROL_IRQ_PRIO);

    ADC_IntCmd(CM_ADC1, ADC_INT_EOCA, ENABLE);

    s_enabled = 1u;
    NVIC_EnableIRQ(INT116_IRQn);

    MAIN_D("[FC] EOCA 20kHz control layer ready: prio=%u, ticks/us=%lu",
           (unsigned)FOC_CONTROL_IRQ_PRIO, (unsigned long)s_ticks_per_us);
}

void Foc_Control_Enable(void)
{
    if (!s_enabled) {
        s_enabled = 1u;
        NVIC_EnableIRQ(INT116_IRQn);
    }
}

void Foc_Control_Disable(void)
{
    s_enabled = 0u;
    NVIC_DisableIRQ(INT116_IRQn);
}

uint8_t Foc_Control_IsEnabled(void)
{
    return s_enabled;
}

void Foc_Control_ResetStats(void)
{
    NVIC_DisableIRQ(INT116_IRQn);

    g_fc_isr_cnt        = 0;
    g_fc_isr_dt_max_us  = 0;
    g_fc_isr_dt_min_us  = 0xFFFFFFFFu;
    g_fc_isr_dt_last_us = 0;
    g_fc_sample_seq     = 0;

    g_dwt_isr_last_cyc = 0;
    g_dwt_isr_min_cyc  = 0xFFFFFFFFu;
    g_dwt_isr_max_cyc  = 0;
    g_dwt_isr_avg_cyc  = 0;
    g_dwt_isr_win_cnt  = 0;
    s_dwt_sum_cyc      = 0;

    g_fc_id_q15        = 0;
    g_fc_iq_q15        = 0;
    g_fx_id_err_max_lsb = 0;
    g_fx_iq_err_max_lsb = 0;
    g_fx_id_err_last_lsb = 0;
    g_fx_iq_err_last_lsb = 0;

    s_ts_init      = 0u;
    s_prev_ts_cnt  = 0u;
    s_prev_enc     = hw_encoder_get_count();
    s_last_static_mode = (foc_mode_t)0xFF;
    s_if_resumed   = 0u;

    if (s_enabled) {
        NVIC_EnableIRQ(INT116_IRQn);
    }
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
