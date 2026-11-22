/**
 *******************************************************************************
 * @file  foc_control.c
 * @brief L5 控制层实现（挂接 ADC/DMA 采样帧完成回调，20kHz）
 *
 *        数据流（每帧）：
 *          编码器计数 -> 电流采样 -> Clarke/Park -> Foc_Run_Step20k
 *          -> InvPark -> SVPWM -> Foc_Output_Duty3
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

#include "core/foc_feedback.h"
#include "core/foc_output.h"
#include "hc32_ll.h"
#include "foc_protect.h"
#include "core/foc_run.h"
#include "proc/foc_proc.h"
#include "foc_current_loop.h"
#include "foc_speed_loop.h"
#include "foc_position_loop.h"
#include "clarke_park.h"
#include "svpwm.h"
#include "hw_tmr4_pwm.h"
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

/* 执行器内部状态：描述桥臂当前**实际**处于什么状态，与运行模式无关。
 * 任何在执行器之外写桥臂的模块（目前只有标定流程）必须把它置为 UNKNOWN，
 * 否则下一拍会跳过静态输出、留下上一段写入的矢量。 */
typedef enum {
    FC_OUT_UNKNOWN = 0,     /* 未知：下一拍必须重设 */
    FC_OUT_ALL_LOW,         /* 全低 + 抱闸 */
    FC_OUT_DRIVING,         /* 恢复正常 PWM 比较输出 */
} fc_out_state_t;

/* Foc_Run_Init() 在板级初始化阶段已置"全低 + 抱闸"，故上电即为 ALL_LOW */
static volatile fc_out_state_t s_out_state = FC_OUT_ALL_LOW;

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
    enc_now = Foc_Feedback_GetCount();
    g_fc_enc_delta = enc_now - s_prev_enc;
    s_prev_enc     = enc_now;
    g_fc_enc_count = enc_now;

    mech = Foc_Feedback_GetMechAngleRad();
    g_fc_enc_mech_deg = mech * 57.29578f;

    theta_e = Foc_Feedback_GetElecAngleRad(FOC_CONTROL_POLE_PAIRS);
    g_fc_enc_elec_deg = theta_e * 57.29578f;
    g_fc_enc_speed_rpm = Foc_Feedback_GetSpeedRpm();

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

    /* ---- 一次性流程（编码器对齐/扫角/测圈/斜坡对齐）期间：ISR 不碰 PWM ----
     * 「标志与状态机同时成立才接管」的判据由 proc 层统一维护，本分支只负责
     * 把 20kHz 计时交给流程并让出控制链。流程自己在执行器之外写桥臂，
     * 所以把输出状态置为未知：流程结束后第一拍会重落运行模式的静态输出。 */
    if (Foc_Proc_Task20k() != 0u) {
        s_out_state = FC_OUT_UNKNOWN;
        return;
    }

    /* ---- 控制链：模式层只产出「本拍输出什么」，本层负责落实 ----
     * set_all_low() / resume_output() 内部要做三相 PWM 重初始化，
     * 因此只在输出状态发生变化时调用一次；duty3 每拍照写（影子装载）。 */
    {
        FocCmd_t  cmd;
        float     valpha, vbeta;
        SvpwmCtx_t ctx;

        Foc_Run_Step20k(FC_ISR_TS_S, &cmd);

        switch (cmd.kind) {
        case FOC_OUT_ALL_LOW:
            if (s_out_state != FC_OUT_ALL_LOW) {
                Foc_Output_SafeStop();          /* 全低 + 抱闸：唯一安全停机入口 */
                s_out_state = FC_OUT_ALL_LOW;
            }
            break;

        case FOC_OUT_DUTY3:
        case FOC_OUT_VECTOR:
            if (s_out_state != FC_OUT_DRIVING) {
                Foc_Output_Release();
                (void)Foc_Output_Resume();
                s_out_state = FC_OUT_DRIVING;
            }

            if (cmd.kind == FOC_OUT_VECTOR) {
                InvPark(cmd.vd, cmd.vq, cmd.theta_e, &valpha, &vbeta);

                ctx.valpha = valpha;
                ctx.vbeta  = vbeta;
                /* 用真实母线电压做调制比（sys_voltage 的 1ms 平均值）。
                 * 低于 12V 时按 12V 处理：上电瞬间或欠压时避免异常占空比，
                 * 欠压保护后续在业务层增加。 */
                ctx.vbus = (g_fc_vbus < 12.0f) ? 12.0f : g_fc_vbus;
                SVPWM_Calc(&ctx);

                (void)Foc_Output_Duty3(ctx.duty_u * 100.0f,
                                      ctx.duty_v * 100.0f,
                                      ctx.duty_w * 100.0f);
            } else {
                (void)Foc_Output_Duty3(cmd.duty_u, cmd.duty_v, cmd.duty_w);
            }
            break;

        case FOC_OUT_NONE:
        default:
            break;
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
    Foc_Proc_Poll1ms();

    /* IF 模式心跳：每 500ms 打印一次实际中心占空比，
     * 直接证明角度自增是否真的在改变 PWM */
    if (++s_dbg_ms >= 500u) {
        s_dbg_ms = 0u;
        if (Foc_Run_GetMode() == FOC_RUN_IF) {
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
                           (int)Foc_Run_LegacyId(Foc_Run_GetMode()),
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

    s_prev_enc = Foc_Feedback_GetCount();
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
    s_prev_enc     = Foc_Feedback_GetCount();
    s_out_state    = FC_OUT_UNKNOWN;   /* 强制下一拍重落一次静态输出 */
}
