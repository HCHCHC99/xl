/*=============================================================================
 * main.c  —  FOC 应用层入口（五模式 + 校准 + 保护 + 电流环 + VOFA）
 *
 * 模式语义：
 *   Mode 0 IDLE    : 全低，安全停机
 *   Mode 1 LOCK    : 50% 零矢量，锁死
 *   Mode 2 IF      : q 轴加压 + 角度自增，开环拖动
 *   Mode 3 CALIB   : 单次正式校准（非阻塞，1ms/20kHz 中断推进）
 *                    ZERO(零矢量采零偏) -> BETA(90) -> ALPHA(0)
 *                    -> 判据 -> 写 offset/dir/g_motor_is_aligned
 *   Mode 4 CURRENT : 有感闭环电流环（硬性要求已对齐 + 零偏已校准）
 *   Mode 5 SWEEP   : 扫角测量（0/90/180/270 四个起始角各跑一遍两点对齐），
 *                    只记录结果，不写 offset/dir，不置 g_motor_is_aligned
 *
 * 本版新增诊断打印：
 *   case 3 入口打印 fp_state/fault/trip_cnt/三相电流
 *   case 4 入口（EntryCheck 之后）打印保护状态
 *=============================================================================*/

#include "main.h"
#include "hc32_ll.h"
#include "hc32_ll_clk.h"
#include "hc32_ll_gpio.h"
#include "hc32_ll_utility.h"
#include "hc32_ll_tmr4.h"
#include "TickTimer.h"

/* ---- 配置层 ---- */
#include "cfg_motor.h"
#include "cfg_sampling.h"
#include "cfg_foc_table.h"

/* ---- 驱动层 ---- */
#include "hw_board_pins.h"
#include "hw_gpio_init.h"
#include "hw_tmr4_pwm.h"
#include "hw_encoder.h"
#include "Timer0_Unit2.h"

/* ---- 算法层 / 采样 ---- */
#include "svpwm.h"
#include "clarke_park.h"
#include "foc_sincos.h"
#include "motor_sampling.h"
#include "encoder_calib.h"

/* ---- 时基 / 控制 ---- */
#include "timer6_timebase.h"
#include "foc_control.h"
#include "foc_mode.h"
#include "foc_protect.h"
#include "foc_current_loop.h"

/* ---- VOFA+ / 日志 ---- */
#include "Usart3_Vofa.h"
#include "rtt_log.h"
#include "SEGGER_RTT.h"

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

/*=============================================================================
 * Watch 可写变量
 *=============================================================================*/
volatile int32_t  g_mode_cmd       = 1;      /* 0=IDLE 1=LOCK 2=IF 3=CALIB 4=CURRENT 5=SWEEP */
volatile int32_t  g_ol_freq_mhz    = 3000;
volatile int32_t  g_ol_vq_mv       = 700;
volatile int32_t  g_vofa_adc_debug = 1;

volatile int32_t  g_cl_id_ref_ma   = 0;
volatile int32_t  g_cl_iq_ref_ma   = 500;
volatile float    g_cl_kp          = 0.1f;
volatile float    g_cl_ki          = 10.0f;
volatile float    g_cl_kd          = 0.0f;

volatile int32_t  g_cl_limit_reason = 0;
volatile float    g_cl_vmag_raw     = 0.0f;
volatile float    g_cl_vmax         = 0.0f;
volatile uint32_t g_cl_sat_cnt      = 0;

volatile int32_t  g_cl_id_ref_ramped_ma = 0;
volatile int32_t  g_cl_iq_ref_ramped_ma = 0;

volatile uint8_t  g_poweron_cal_ok = 0;

/* 非阻塞对齐/扫角运行标志（主循环内部） */
static uint8_t s_align_running = 0;

/*=============================================================================
 * 采样标定
 *=============================================================================*/
static const MotorSamplingScale_t g_scale = {
    .current_scale  = CFG_SAMPLING_CURRENT_SCALE,
    .current_offset = CFG_SAMPLING_CURRENT_OFFSET,
    .vbus_scale     = CFG_SAMPLING_VBUS_SCALE,
    .vbus_ch        = HW_ADC_CH_VBUS,
};

/*=============================================================================
 * VOFA+ 数据帧填充 (16ch)
 *=============================================================================*/
extern volatile float    g_fc_iu;
extern volatile float    g_fc_iv;
extern volatile float    g_fc_iw;
extern volatile float    g_fc_i_total;
extern volatile uint16_t g_adc_raw_itotal;
extern volatile uint16_t g_adc_raw_iv;
extern volatile uint16_t g_adc_raw_iw;
extern volatile float    g_fc_id;
extern volatile float    g_fc_iq;
extern volatile float    g_fc_enc_elec_deg;
extern volatile float    g_fc_enc_mech_deg;
extern volatile int32_t  g_fc_enc_count;

static int Foc_AdcDebug_VofaFill(int32_t *cur)
{
    /* ch0~2: ADC raw (传 raw*1000, 接收端乘 0.001) */
    cur[0] = (int32_t)g_adc_raw_itotal * 1000;
    cur[1] = (int32_t)g_adc_raw_iv     * 1000;
    cur[2] = (int32_t)g_adc_raw_iw     * 1000;

    /* ch3~6: 电流 A (传 mA, 接收端乘 0.001) */
    cur[3] = (int32_t)(g_fc_i_total * 1000.0f);
    cur[4] = (int32_t)(g_fc_iu      * 1000.0f);
    cur[5] = (int32_t)(g_fc_iv      * 1000.0f);
    cur[6] = (int32_t)(g_fc_iw      * 1000.0f);

    /* ch7~8: 真实 iq / id (传 mA) */
    cur[7] = (int32_t)(g_fc_iq      * 1000.0f);
    cur[8] = (int32_t)(g_fc_id      * 1000.0f);

    /* ch9~10: 目标 iq / id (mA 直接给) */
    cur[9]  = g_cl_iq_ref_ma;
    cur[10] = g_cl_id_ref_ma;

    /* ch11~12: 斜坡后 iq / id (mA) */
    cur[11] = g_cl_iq_ref_ramped_ma;
    cur[12] = g_cl_id_ref_ramped_ma;

    /* ch13: 电角度 (0.001 rad) = deg * pi/180 * 1000 */
    cur[13] = (int32_t)(g_fc_enc_elec_deg * 17.4532925f);

    /* ch14: 机械角度 (0.001 rad) */
    cur[14] = (int32_t)(g_fc_enc_mech_deg * 17.4532925f);

    /* ch15: 编码器圈内计数 0~4095，发送值 1000 */
    {
        int32_t c = g_fc_enc_count % (int32_t)CFG_ENCODER_CPR;
        if (c < 0) c += (int32_t)CFG_ENCODER_CPR;
        cur[15] = c * 1000;
    }

    return 16;
}

/*=============================================================================
 * 系统时钟
 *=============================================================================*/
static void SystemClock_Config(void)
{
    Systick_config();
}

/*=============================================================================
 * 进入 IF 模式
 *=============================================================================*/
static void Enter_IF_Mode(void)
{
    if (!g_motor_is_offset_calibrated) {
        MAIN_E("[WARN] Zero offset NOT calibrated, IF current feedback has baseline error.");
    }
    if (!g_motor_is_aligned) {
        MAIN_E("[WARN] Electrical angle NOT aligned, encoder elec angle unreliable. "
               "IF openloop itself is unaffected. Run mode 3 to align.");
    }

    Foc_Mode_SetOpenloop((float)g_ol_freq_mhz / 1000.0f,
                         (float)g_ol_vq_mv    / 1000.0f,
                         0.0f);
    Foc_Mode_StartOpenloop();

    MAIN_D("[IF] start: f=%ld mHz, vq=%ld mV (cal: offset=%u align=%u)",
           (long)g_ol_freq_mhz, (long)g_ol_vq_mv,
           (unsigned)g_motor_is_offset_calibrated,
           (unsigned)g_motor_is_aligned);
}

/*=============================================================================
 * 上电零电流偏置校准（阻塞）
 *=============================================================================*/
static bool PowerOn_OffsetCal(void)
{
    bool ok;

    MAIN_D("[CAL] Power-on zero offset calibration start...");

    (void)hw_tmr4_pwm_set_all_low();
    Foc_Mode_SetMode(FOC_MODE_IDLE);

    ok = Motor_Sampling_DoOffsetCalBlocking(300u);

    if (ok) {
        MAIN_D("[CAL] Power-on offset cal OK: offset=%d LSB",
               (int)g_cal_offset_itotal);
        MAIN_D("[CAL]   itotal=%u iw=%u iv=%u",
               (unsigned)g_cal_offset_itotal,
               (unsigned)g_cal_offset_iw,
               (unsigned)g_cal_offset_iv);
        g_poweron_cal_ok = 1u;
    } else {
        MAIN_E("[CAL] Power-on offset cal TIMEOUT! current feedback unreliable.");
        g_motor_is_offset_calibrated = 0u;
        g_poweron_cal_ok = 0u;
    }

    return ok;
}

/*=============================================================================
 * 电流环入口检查
 *=============================================================================*/
static bool CurrentLoop_EntryCheck(void)
{
    if (!g_motor_is_aligned) {
        MAIN_E("[CL] refuse: elec angle NOT aligned, run Mode 3 first.");
        return false;
    }
    if (!g_motor_is_offset_calibrated) {
        MAIN_E("[CL] refuse: current offset NOT calibrated.");
        return false;
    }
    if (!Foc_CurrentLoop_IsReady()) {
        MAIN_E("[CL] refuse: current loop PID not initialized.");
        return false;
    }
    if (g_fp_state == FP_STATE_LATCHED) {
        MAIN_E("[CL] refuse: protection LATCHED, call Foc_Protect_ClearFault() first.");
        return false;
    }
    if (g_fp_state == FP_STATE_TRIP) {
        MAIN_E("[CL] refuse: protection TRIPPED, wait recover (I < %ldmA).",
               (long)(FP_I_RECOVER_A * 1000.0f));
        return false;
    }
    if (g_fc_vbus < 6.0f) {
        MAIN_E("[CL] refuse: vbus too low (vbus_x100=%ld).",
               (long)(g_fc_vbus * 100.0f));
        return false;
    }

    if (Foc_Mode_GetMode() == FOC_MODE_IF) {
        MAIN_D("[CL] switching from IF: IF vq -> 0 before entering CURRENT.");
        Foc_Mode_SetOpenloop((float)g_ol_freq_mhz / 1000.0f, 0.0f, 0.0f);
    }

    return true;
}

/*=============================================================================
 * main
 *=============================================================================*/
int main(void)
{
    tmr4_pwm_config_t  pwmCfg;
    tmr4_pwm_err_t     pwmErr;
    int32_t            last_mode_cmd = -1;

    /* ---- 1. 基础外设 ---- */
    SystemClock_Config();
    Foc_SinCos_Init();
    hw_gpio_init_all();

    SEGGER_RTT_Init();
    SEGGER_RTT_SetFlagsUpBuffer(0, SEGGER_RTT_MODE_NO_BLOCK_SKIP);

    {
        Usart3_HW_Config_t cfg = USART3_HW_CONFIG_DEFAULT;
        cfg.baudrate = 921600;
        Usart3_Vofa_Init(&cfg);
    }

    MAIN_D("==============================================");
    MAIN_D(" FOC Start: %s, %lu Hz", FOC_CFG_NAME, (unsigned long)FOC_CFG_PWM_FREQ);
    MAIN_D("==============================================");

    /* ---- 2. 采样层 ---- */
    Motor_Sampling_Init(&g_scale);

    /* ---- 3. 时基与定时器 ---- */
    Timer6_Timebase_Init();
    Timer6_Timebase_Start();

    TMR0_Unit2_Init(TMR0_CHANNEL_A_2, 1000UL, TICK_RESET_2);
    TMR0_Unit2_Init(TMR0_CHANNEL_B_2, 1000UL, TICK_RESET_2);

    hw_encoder_init();

    /* ---- 4. TMR4 PWM ---- */
    pwmCfg.outputTypeU = FOC_CFG_OUTPUT_TYPE;
    pwmCfg.outputTypeV = FOC_CFG_OUTPUT_TYPE;
    pwmCfg.outputTypeW = FOC_CFG_OUTPUT_TYPE;
    pwmCfg.freqHz      = FOC_CFG_PWM_FREQ;
    pwmCfg.deadTimeNs  = FOC_CFG_DEADTIME_NS;
    pwmCfg.activeHigh  = FOC_CFG_ACTIVE_HIGH;

    pwmErr = hw_tmr4_pwm_config(&pwmCfg);
    if (pwmErr != TMR4_PWM_OK) {
        MAIN_E("[HW_TMR4] config failed: %d", (int)pwmErr);
        while (1) { }
    }

    pwmErr = hw_tmr4_pwm_start_output();
    if (pwmErr != TMR4_PWM_OK) {
        MAIN_E("[HW_TMR4] start failed: %d", (int)pwmErr);
        while (1) { }
    }

    (void)hw_tmr4_pwm_set_all_low();

    /* ---- 5. 模式 / 保护 / 电流环 ---- */
    Foc_Mode_Init();
    Foc_Protect_Init(NULL);
    Foc_CurrentLoop_Init();

    /* ---- 6. 控制层（EOCA 中断使能）---- */
    Foc_Control_Init();

    /* ---- 7. 上电零偏校准 ---- */
    (void)PowerOn_OffsetCal();

    MAIN_D(" Ready! g_mode_cmd: 0=IDLE 1=LOCK 2=IF 3=CALIB 4=CURRENT 5=SWEEP");
    MAIN_D(" Mode 3 = align (ZERO->BETA->ALPHA), Mode 4 = current, Mode 5 = sweep.");

    /* ---- 主循环 ---- */
    while (1) {
        hw_encoder_update();
        g_fc_enc_speed_rpm = hw_encoder_get_speed_rpm();

        /* ---- 模式切换 ---- */
        if (g_mode_cmd != last_mode_cmd) {
            last_mode_cmd = g_mode_cmd;

            switch (g_mode_cmd) {
            case 0:
                Foc_Mode_SetMode(FOC_MODE_IDLE);
                break;

            case 1:
                Foc_Mode_SetMode(FOC_MODE_LOCK);
                break;

            case 2:
                Enter_IF_Mode();
                break;

            case 3:
                MAIN_D("[CAL] request: fp_state=%d fault=0x%08lX trip_cnt=%lu "
                       "iu_x100=%ld iv_x100=%ld iw_x100=%ld",
                       (int)g_fp_state,
                       (unsigned long)g_fp_fault,
                       (unsigned long)g_fp_trip_cnt,
                       (long)(g_fc_iu * 100.0f),
                       (long)(g_fc_iv * 100.0f),
                       (long)(g_fc_iw * 100.0f));
                MAIN_D("[CAL] Starting Calibration (ZERO->BETA->ALPHA)...");
                Foc_Mode_SetMode(FOC_MODE_IDLE);
                EncoderCalib_StartAlign();
                s_align_running = 1;
                break;

            case 4:
                if (!CurrentLoop_EntryCheck()) {
                    g_mode_cmd    = 0;
                    last_mode_cmd = 0;
                    Foc_Mode_SetMode(FOC_MODE_IDLE);
                    break;
                }

                Foc_CurrentLoop_SetGains(g_cl_kp, g_cl_ki, g_cl_kd);
                Foc_CurrentLoop_SetRef((float)g_cl_id_ref_ma / 1000.0f,
                                       (float)g_cl_iq_ref_ma / 1000.0f);

                MAIN_D("[CL] ---- enter CURRENT, params ----");
                MAIN_D("[CL] PID:   kp_x10000=%ld ki_x100=%ld kd_x10000=%ld",
                       (long)(g_cl_kp * 10000.0f),
                       (long)(g_cl_ki * 100.0f),
                       (long)(g_cl_kd * 10000.0f));
                MAIN_D("[CL] REF:   id_ref_mA=%ld iq_ref_mA=%ld",
                       (long)g_cl_id_ref_ma, (long)g_cl_iq_ref_ma);
                MAIN_D("[CL] RAMP:  ref_slew_x1e6=%ld A/tick, vq_slew_x1e6=%ld V/tick",
                       (long)(g_cl_ref_slew * 1000000.0f),
                       (long)(g_cl_vq_slew  * 1000000.0f));
                MAIN_D("[CL] WARM:  warmup_ticks=%u", (unsigned)g_cl_warmup_ticks);
                MAIN_D("[CL] LIMIT: iref_max_x100=%ld A, vmax_x100=%ld V",
                       (long)(g_cl_iref_max * 100.0f),
                       (long)(g_cl_vmax_watch * 100.0f));
                MAIN_D("[CL] STATE: aligned=%u offset_cal=%u fp_state=%d",
                       (unsigned)g_motor_is_aligned,
                       (unsigned)g_motor_is_offset_calibrated,
                       (int)g_fp_state);
                MAIN_D("[CL] ------------------------------");

                Foc_Mode_SetMode(FOC_MODE_CURRENT);
                MAIN_D("[CL] current loop entered (ref id=%ldmA iq=%ldmA, ramp from 0).",
                       (long)g_cl_id_ref_ma, (long)g_cl_iq_ref_ma);
                break;

            case 5:
                MAIN_D("[SWEEP] request: fp_state=%d fault=0x%08lX trip_cnt=%lu",
                       (int)g_fp_state,
                       (unsigned long)g_fp_fault,
                       (unsigned long)g_fp_trip_cnt);
                MAIN_D("[SWEEP] Starting angle sweep (0/90/180/270)...");
                Foc_Mode_SetMode(FOC_MODE_IDLE);
                EncoderCalib_StartSweep();
                s_align_running = 1;
                break;

            default:
                Foc_Mode_SetMode(FOC_MODE_IDLE);
                break;
            }
        }

        /* ---- 非阻塞对齐/扫角：事件消费 + 收尾 ---- */
        if (s_align_running) {
            uint8_t evt = EncoderCalib_ConsumeEvent();

            if (evt == EC_EVT_ZERO_DONE) {
                MAIN_D("[CAL] zero: iu=%ldmA iv=%ldmA iw=%ldmA",
                       (long)g_align_zero_u_ma,
                       (long)g_align_zero_v_ma,
                       (long)g_align_zero_w_ma);
            } else if (evt == EC_EVT_BETA_DONE) {
                if (EncoderCalib_IsSweepDone() == 0u) {
                    MAIN_D("[CAL] BETA done beta_hw=%ld th=90deg",
                           (long)g_align_beta_hw);
                }
            } else if (evt == EC_EVT_OC) {
                MAIN_E("[CAL] FAULT_OC during align, abort.");
            }

            if (EncoderCalib_IsDone()) {
                s_align_running = 0;

                if (EncoderCalib_IsSweepDone()) {
                    MAIN_D("[SWEEP] pts: "
                           "i0 b=%ld a=%ld m=%ld | i1 b=%ld a=%ld m=%ld | "
                           "i2 b=%ld a=%ld m=%ld | i3 b=%ld a=%ld m=%ld",
                           (long)g_sweep_beta[0], (long)g_sweep_alpha[0], (long)g_sweep_moved[0],
                           (long)g_sweep_beta[1], (long)g_sweep_alpha[1], (long)g_sweep_moved[1],
                           (long)g_sweep_beta[2], (long)g_sweep_alpha[2], (long)g_sweep_moved[2],
                           (long)g_sweep_beta[3], (long)g_sweep_alpha[3], (long)g_sweep_moved[3]);
                    MAIN_D("[SWEEP] done: moved = %ld, %ld, %ld, %ld",
                           (long)g_sweep_moved[0], (long)g_sweep_moved[1],
                           (long)g_sweep_moved[2], (long)g_sweep_moved[3]);
                } else if (EncoderCalib_IsAlignOk()) {
                    MAIN_D("[CAL] done: beta=%ld alpha=%ld moved=%ld offset=%ld dir=%d",
                           (long)g_align_beta_hw, (long)g_align_alpha_hw,
                           (long)g_align_moved, (long)g_align_offset,
                           (int)g_enc_elec_dir);
                    MAIN_D("[CAL] Align OK");
                } else {
                    MAIN_D("[CAL] done: beta=%ld alpha=%ld moved=%ld",
                           (long)g_align_beta_hw, (long)g_align_alpha_hw,
                           (long)g_align_moved);
                    MAIN_E("[CAL] Align failed! Check voltage / hold_ms / pole pairs.");
                }

                g_mode_cmd    = 0;
                last_mode_cmd = 0;
                Foc_Mode_SetMode(FOC_MODE_IDLE);
                (void)hw_tmr4_pwm_set_all_low();
            }
        }

        /* IF 参数实时同步 */
        if (g_mode_cmd == 2) {
            Foc_Mode_SetOpenloop((float)g_ol_freq_mhz / 1000.0f,
                                 (float)g_ol_vq_mv    / 1000.0f,
                                 0.0f);
        }

        /* CURRENT 参数实时同步 */
        if (g_mode_cmd == 4) {
            Foc_CurrentLoop_SetRef((float)g_cl_id_ref_ma / 1000.0f,
                                   (float)g_cl_iq_ref_ma / 1000.0f);
            Foc_CurrentLoop_SetGains(g_cl_kp, g_cl_ki, g_cl_kd);
        }

        /* vmax 在线同步 */
        {
            static float s_last_vmax = 0.0f;
            if (g_cl_vmax_watch != s_last_vmax) {
                s_last_vmax = g_cl_vmax_watch;
                Foc_CurrentLoop_SetVmax(g_cl_vmax_watch);
            }
        }

        /* 受限诊断镜像 */
        g_cl_limit_reason = (int32_t)Foc_CurrentLoop_GetLimitReason();
        g_cl_vmag_raw     = Foc_CurrentLoop_GetVmagRaw();
        g_cl_vmax         = Foc_CurrentLoop_GetVmax();
        g_cl_sat_cnt      = (uint32_t)Foc_CurrentLoop_GetSatCnt();
        g_cl_id_ref_ramped_ma = (int32_t)(Foc_CurrentLoop_GetIdRefRamped() * 1000.0f);
        g_cl_iq_ref_ramped_ma = (int32_t)(Foc_CurrentLoop_GetIqRefRamped() * 1000.0f);

        /* 电流环受限告警 */
        if (g_mode_cmd == 4 && Foc_CurrentLoop_GetSatCnt() > 4000u) {
            static uint32_t s_last_warn_ms = 0;
            uint32_t now_ms = tickTimer_GetCount();

            if ((now_ms - s_last_warn_ms) >= 1000u) {
                s_last_warn_ms = now_ms;
                MAIN_E("[CL] LIMITED(%d): v_raw_x100=%ldV vmax_x100=%ldV "
                       "iq_ref_mA=%ld iq_fb_mA=%ld sat_cnt=%u",
                       (int)Foc_CurrentLoop_GetLimitReason(),
                       (long)(Foc_CurrentLoop_GetVmagRaw() * 100.0f),
                       (long)(Foc_CurrentLoop_GetVmax() * 100.0f),
                       (long)g_cl_iq_ref_ma,
                       (long)(g_fc_iq * 1000.0f),
                       (unsigned)Foc_CurrentLoop_GetSatCnt());
            }
        }

        /* CURRENT 编码器失效监测 */
        if (g_mode_cmd == 4) {
            static int32_t  s_last_enc      = 0;
            static uint32_t s_still_cnt     = 0;
            static bool     s_cl_mon_init   = false;

            if (!s_cl_mon_init) {
                s_last_enc    = g_fc_enc_count;
                s_still_cnt   = 0;
                s_cl_mon_init = true;
            }

            if (g_fc_enc_count == s_last_enc) {
                s_still_cnt++;
            } else {
                s_still_cnt = 0;
                s_last_enc  = g_fc_enc_count;
            }

            if ((s_still_cnt > 100u) &&
                (fabsf((float)g_cl_iq_ref_ma) > 50.0f) &&
                (fabsf(g_fc_iq - (float)g_cl_iq_ref_ma / 1000.0f) > 0.5f)) {
                MAIN_E("[CL] encoder may be dead: cnt still, iq_ref_mA=%ld iq_fb_mA=%ld",
                       (long)g_cl_iq_ref_ma, (long)(g_fc_iq * 1000.0f));
            }
        }

        /* VOFA 数据发送（非阻塞，16 通道） */
        if (!Usart3_Vofa_IsTxBusy() && g_vofa_adc_debug != 0) {
            int32_t cur[16];
            Foc_AdcDebug_VofaFill(cur);
            Usart3_Vofa_SendScaled(cur, 16, USART3_VOFA_SCALE_MILLI);
        }
    }
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
