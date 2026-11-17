/**
 *******************************************************************************
 * @file  foc_control.c
 * @brief L5 控制层实现（挂接 ADC/DMA 采样帧完成回调，20kHz）
 *
 *        数据流（每帧）：
 *          编码器计数 -> 电流采样 -> Clarke/Park -> Foc_Mode_Step
 *          -> InvPark -> SVPWM -> hw_tmr4_pwm_set_duty3
 *
 *        本工程采样帧完成点 = DMA1 CH2 BTC 中断（ADC1 SEQ_A 四通道搬完），
 *        由 bsp_adc_foc.c 在中断里调用 Foc_Control_Step()。
 *******************************************************************************
 */

#include "foc_control.h"
#include "cfg_motor.h"
#include "cfg_sampling.h"
#include "cfg_foc_table.h"
#include "../drv/bsp_adc_foc.h"

#include "hw_encoder.h"
#include "hc32_ll.h"
#include "foc_protect.h"
#include "foc_mode.h"
#include "foc_current_loop.h"
#include "foc_speed_loop.h"
#include "foc_position_loop.h"
#include "clarke_park.h"
#include "svpwm.h"
#include "hw_tmr4_pwm.h"
#include "encoder_calib.h"
#include "foc_sincos.h"

#include <rtthread.h>

/* 20kHz ISR 热路径：Debug 构建是 -O0，闭环时单拍容易超过 50us，
 * 导致 idle 线程得不到调度、msh 输入失效。只对本文件开 O2，
 * 其余模块仍保持 -O0 方便调试。 */
#pragma GCC optimize("O2")

/*---------------------------------------------------------------------------
 * 节拍测量
 *---------------------------------------------------------------------------*/
#define FC_DT_SANE_MIN_US   10u
#define FC_DT_SANE_MAX_US   200u

/* ISR 节拍 = cfg_foc_table.h 的 PWM 频率 */
#define FC_ISR_TS_S         (1.0f / (float)FOC_CFG_PWM_FREQ)

/* 哨兵：标记上一拍走了 enable=true 分支 */
#define FC_LAST_MODE_IN_IF  ((foc_mode_t)0xFE)

/*=============================================================================
 * 观测量
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

volatile uint32_t g_fc_sample_seq     = 0;

volatile uint32_t g_dwt_isr_last_cyc = 0;
volatile uint32_t g_dwt_isr_min_cyc  = 0xFFFFFFFFu;
volatile uint32_t g_dwt_isr_max_cyc  = 0;
volatile uint32_t g_dwt_isr_avg_cyc  = 0;
volatile uint32_t g_dwt_isr_win_cnt  = 0;
static uint32_t   s_dwt_sum_cyc      = 0;
static uint8_t    s_dwt_inited       = 0;

volatile uint8_t  g_align_active      = 0u;

/*---------------------------------------------------------------------------
 * 内部状态
 *---------------------------------------------------------------------------*/
static volatile uint8_t  s_enabled     = 0;
static uint8_t           s_inited      = 0u;
static volatile uint32_t s_prev_cyc    = 0;
static volatile uint8_t  s_cyc_init    = 0;
static volatile int32_t  s_prev_enc    = 0;
static uint32_t          s_ticks_per_us = 1u;

static foc_mode_t        s_last_static_mode = (foc_mode_t)0xFF;
static uint8_t           s_if_resumed = 0u;

static struct rt_timer   s_foc_1ms_timer;

/*=============================================================================
 * 节拍采样（DWT 周期计数，不含异常出入栈）
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

static void Foc_Control_MeasureDt(void)
{
    uint32_t now;

    if (!s_dwt_inited) {
        return;
    }

    now = DWT->CYCCNT;
    if (!s_cyc_init) {
        s_prev_cyc = now;
        s_cyc_init = 1u;
        return;
    }

    {
        uint32_t d  = now - s_prev_cyc;
        uint32_t us = d / s_ticks_per_us;

        s_prev_cyc = now;

        if ((us >= FC_DT_SANE_MIN_US) && (us <= FC_DT_SANE_MAX_US)) {
            g_fc_isr_dt_last_us = us;
            if (us > g_fc_isr_dt_max_us) g_fc_isr_dt_max_us = us;
            if (us < g_fc_isr_dt_min_us) g_fc_isr_dt_min_us = us;
        }
    }
}

static void Foc_Control_DwtRecord(uint32_t cyc)
{
    g_dwt_isr_last_cyc = cyc;

    if (cyc < g_dwt_isr_min_cyc) g_dwt_isr_min_cyc = cyc;
    if (cyc > g_dwt_isr_max_cyc) g_dwt_isr_max_cyc = cyc;

    s_dwt_sum_cyc += cyc;
    g_dwt_isr_win_cnt++;
    if (g_dwt_isr_win_cnt >= 1024u) {
        g_dwt_isr_avg_cyc = s_dwt_sum_cyc >> 10;
        s_dwt_sum_cyc     = 0u;
        g_dwt_isr_win_cnt = 0u;
    }
}

/*=============================================================================
 * 20kHz 控制步（由 bsp_adc_foc 的 DMA BTC 中断调用）
 *=============================================================================*/
void Foc_Control_Step(void)
{
    foc_sample_t s;
    int32_t enc_now;
    float   mech;
    float   theta_e;
    float   ialpha, ibeta;

    if (!s_inited) {
        return;
    }

    g_fc_isr_cnt++;

    Foc_Control_MeasureDt();

    /* 编码器计数已在 bsp_adc_foc 的中断入口同步（与电流同拍） */
    enc_now = hw_encoder_get_count();
    g_fc_enc_delta = enc_now - s_prev_enc;
    s_prev_enc     = enc_now;
    g_fc_enc_count = enc_now;

    mech = hw_encoder_get_mech_angle_rad();
    g_fc_enc_mech_deg = mech * 57.29578f;

    theta_e = hw_encoder_get_elec_angle_rad(FOC_CONTROL_POLE_PAIRS);
    g_fc_enc_elec_deg = theta_e * 57.29578f;
    g_fc_enc_speed_rpm = hw_encoder_get_speed_rpm();

    /* ---- 电流/电压采样 ---- */
    (void)foc_adc_get_sample(&s);
    g_fc_i_total = s.i_total;
    g_fc_iu      = s.iu;
    g_fc_iv      = s.iv;
    g_fc_iw      = s.iw;
    g_fc_vbus    = s.vbus;
    g_fc_sample_seq = s.seq;

    /* ---- 过流保护搬运 ---- */
    Foc_Protect_PushIsr(g_fc_iu, g_fc_iv, g_fc_iw);

    /* ---- Clarke & Park ---- */
    Clarke(g_fc_iu, g_fc_iv, g_fc_iw, &ialpha, &ibeta);
    Park(ialpha, ibeta, theta_e, (float *)&g_fc_id, (float *)&g_fc_iq);

    /* ---- Mode 3 对齐期间：ISR 不碰 PWM ----
     * 注意：这里不能在“标志=1 但状态机尚未置忙”的启动窗口里清标志。
     * Start*() 已经改成先置状态再置标志；本分支只负责喂 20kHz 计时。 */
    if ((g_align_active != 0u) && (EncoderCalib_IsBusy() != 0u)) {
        EncoderCalib_Task20k();
        return;
    }

    /* ---- 控制链 ---- */
    {
        FocCmd_t  cmd;
        float     valpha, vbeta;
        SvpwmCtx_t ctx;

        Foc_Mode_Step(FC_ISR_TS_S, &cmd);

        if (cmd.enable) {
            if (!s_if_resumed) {
                s_if_resumed = 1u;
                hw_tmr4_pwm_resume_output();
            }

            s_last_static_mode = FC_LAST_MODE_IN_IF;

            InvPark(cmd.vd, cmd.vq, cmd.theta_e, &valpha, &vbeta);

            ctx.valpha = valpha;
            ctx.vbeta  = vbeta;
            /* 与 FOC 工程一致：SVPWM 归一化用固定 12V，
             * 真实母线电压仅作观测（标定系数待实测） */
            ctx.vbus   = CFG_SAMPLING_SIM_VBUS_V;
            SVPWM_Calc(&ctx);

            hw_tmr4_pwm_set_duty3(ctx.duty_u * 100.0f,
                                  ctx.duty_v * 100.0f,
                                  ctx.duty_w * 100.0f);
        } else {
            s_if_resumed = 0u;

            if (s_last_static_mode != g_fm_mode) {
                s_last_static_mode = g_fm_mode;

                if (g_fm_mode == FOC_MODE_IDLE) {
                    hw_tmr4_pwm_set_all_low();
                } else if (g_fm_mode != FOC_MODE_LOCK) {
                    /* LOCK 的占空比由 Foc_Mode_SetLockDuty/SetMode 写入，
                     * 这里若再写 2% 会覆盖用户指定的 focmode 1 <duty> */
                    hw_tmr4_pwm_resume_output();
                    hw_tmr4_pwm_set_duty3(2.0f, 2.0f, 2.0f);
                }
            }
        }
    }
}

/*=============================================================================
 * 1ms 任务：过流判断 + 编码器标定状态机
 *=============================================================================*/
static void foc_1ms_timer_cb(void *param)
{
    static uint16_t s_dbg_ms = 0u;

    (void)param;

    Foc_Protect_Task1ms();
    EncoderCalib_Task1ms();

    /* IF 模式心跳：每 500ms 打印一次实际中心占空比，
     * 直接证明角度自增是否真的在改变 PWM */
    if (++s_dbg_ms >= 500u) {
        s_dbg_ms = 0u;
        if (Foc_Mode_GetMode() == FOC_MODE_IF) {
            tmr4_pwm_state_t st;
            int i;
            int c[3];

            for (i = 0; i < 3; i++) {
                hw_tmr4_pwm_get_state((tmr4_pwm_channel_t)i, &st);
                c[i] = (int)(st.centerPct * 100.0f + 0.5f);
            }
            rt_kprintf("[IF] theta=%d.%d deg duty U=%d.%02d V=%d.%02d W=%d.%02d %%\r\n",
                       (int)(g_fm_theta_e * 57.29578f),
                       (int)((g_fm_theta_e * 57.29578f -
                              (float)(int)(g_fm_theta_e * 57.29578f)) * 10.0f),
                       c[0] / 100, c[0] % 100,
                       c[1] / 100, c[1] % 100,
                       c[2] / 100, c[2] % 100);
        } else {
            /* 闭环模式下的 ISR 超时告警：dt 是两次 ISR 入口间隔，
             * 正常应等于 PWM 周期；大于周期说明本拍没跑完就被下一次打断。 */
            uint32_t period_us = 1000000u / FOC_CFG_PWM_FREQ;

            if (g_fc_isr_dt_last_us > (period_us + 2u)) {
                rt_kprintf("[FC] WARN: ISR overrun mode=%d dt=%lu us > period %lu us\r\n",
                           (int)Foc_Mode_GetMode(),
                           (unsigned long)g_fc_isr_dt_last_us,
                           (unsigned long)period_us);
            }
        }
    }
}

/*=============================================================================
 * 初始化
 *=============================================================================*/
void Foc_Control_Init(void)
{
    uint32_t hz;

    if (s_inited) {
        return;
    }

    hz = SystemCoreClock;
    s_ticks_per_us = hz / 1000000u;
    if (s_ticks_per_us == 0u) {
        s_ticks_per_us = 1u;
    }

    Foc_Control_DwtInit();   /* 使能 DWT 周期计数，ISR dt 统计才有效 */

    Foc_SinCos_Init();

    Foc_CurrentLoop_Init();
    Foc_Speed_Init();
    Foc_Position_Init();
    Foc_Protect_Init(RT_NULL);

    s_prev_enc = hw_encoder_get_count();
    s_inited   = 1u;

    /* 采样帧完成回调：DMA 搬完一帧后在中断里调 Foc_Control_Step */
    foc_adc_set_frame_callback(Foc_Control_Step);

    rt_timer_init(&s_foc_1ms_timer,
                  "foc_1ms",
                  foc_1ms_timer_cb,
                  RT_NULL,
                  rt_tick_from_millisecond(1),
                  RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    rt_timer_start(&s_foc_1ms_timer);

    s_enabled = 1u;

    rt_kprintf("[FC] control layer ready: isr=%lu Hz, core=%lu MHz\r\n",
               (unsigned long)FOC_CFG_PWM_FREQ,
               (unsigned long)(hz / 1000000u));

    /* 自检：确认采样帧回调真的在跑。没有这条，IF/CURRENT/SPEED/POSITION
     * 全部表现为"模式切了但占空比不动"。 */
    {
        uint32_t c0 = g_fc_isr_cnt;

        rt_thread_mdelay(5);
        if (g_fc_isr_cnt == c0) {
            rt_kprintf("[FC] WARN: no ADC frame in 5ms, check foc_adc/DMA/trigger\r\n");
        } else {
            rt_kprintf("[FC] frame ok: %lu frames in 5ms\r\n",
                       (unsigned long)(g_fc_isr_cnt - c0));
        }
    }
}

void Foc_Control_Enable(void)
{
    s_enabled = 1u;
}

void Foc_Control_Disable(void)
{
    s_enabled = 0u;
}

uint8_t Foc_Control_IsEnabled(void)
{
    return s_enabled;
}

void Foc_Control_ResetStats(void)
{
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

    s_cyc_init     = 0u;
    s_prev_enc     = hw_encoder_get_count();
    s_last_static_mode = (foc_mode_t)0xFF;
    s_if_resumed   = 0u;
}
