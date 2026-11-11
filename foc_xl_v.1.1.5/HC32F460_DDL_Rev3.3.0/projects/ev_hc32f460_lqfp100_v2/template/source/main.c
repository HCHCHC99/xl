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
#include "foc_speed_loop.h"
#include "foc_position_loop.h"

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
volatile int32_t  g_mode_cmd       = 0;      /* 0=IDLE 1=LOCK 2=IF 3=CALIB 4=CURRENT 5=SWEEP 6=REV 7=RAMP 8=CURRENT_FF 9=SPEED 10=POSITION */
volatile int32_t  g_ol_freq_mhz    = 700;
volatile int32_t  g_ol_vq_mv       = 700;
volatile int32_t  g_vofa_adc_debug = 1;

volatile int32_t  g_cl_id_ref_ma   = 0;
volatile int32_t  g_cl_iq_ref_ma   = 1500;
volatile float    g_cl_kp          = 0.27f;
volatile float    g_cl_ki          = 630.0f;
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

/* 速度模式电流环参数已下发值（用于 Watch 改动检测） */
static float s_spd_vmax_applied  = 0.0f;
static float s_spd_iterm_applied = 0.0f;

/*=============================================================================
 * 采样标定
 *=============================================================================*/
static const MotorSamplingScale_t g_scale = {
    .current_scale  = CFG_SAMPLING_CURRENT_SCALE,
    .current_offset = { CFG_SAMPLING_CURRENT_OFFSET,
                        CFG_SAMPLING_CURRENT_OFFSET,
                        CFG_SAMPLING_CURRENT_OFFSET },
    .vbus_scale     = CFG_SAMPLING_VBUS_SCALE,
    .vbus_ch        = HW_ADC_CH_VBUS,
};

/*=============================================================================
 * VOFA+ 数据帧填充 (19ch)
 *=============================================================================*/
extern volatile float    g_fc_iu;
extern volatile float    g_fc_iv;
extern volatile float    g_fc_iw;
extern volatile float    g_fc_i_total;
extern volatile float    g_fc_id;
extern volatile float    g_fc_iq;
extern volatile float    g_fc_enc_elec_deg;
extern volatile float    g_fc_enc_mech_deg;
extern volatile int32_t  g_fc_enc_count;

static int Foc_AdcDebug_VofaFill(int32_t *cur)
{
    /* ch0~2: 位置环 最终目标 / 有效给定 / 当前位置（mrev，接收端乘 0.001）
     *   有效给定 = 当前 pos + 限幅/死区后的 err，即 PI 实际跟踪的位置 */
    cur[0] = (int32_t)(g_pos_target_rev * 1000.0f);
    cur[1] = (int32_t)((g_pos_fb_rev + g_pos_err_rev) * 1000.0f);
    cur[2] = (int32_t)(g_pos_fb_rev * 1000.0f);

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

    /* ch16: IF 强拖电角度（0.001 rad），与 ch13 同单位，可直接对比 */
    cur[16] = (int32_t)(g_fm_theta_e * 1000.0f);

    /* ch17~18: 速度环斜坡目标速度 / 实测速度（mRPM，接收端乘 0.001） */
    cur[17] = (int32_t)(g_spd_ramp_rpm * 1000.0f);
    cur[18] = (int32_t)(g_spd_filt_rpm * 1000.0f);

    return 19;
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
        MAIN_D("[CAL]   PA3=%u iw=%u iv=%u",
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
 * 进入 CURRENT：mode4 = 纯 PI；mode8 = PI + dq 解耦/反电动势前馈
 *=============================================================================*/
static bool Enter_CurrentLoop(uint8_t ff_enable)
{
    if (!CurrentLoop_EntryCheck()) {
        g_mode_cmd = 0;
        Foc_Mode_SetMode(FOC_MODE_IDLE);
        return false;
    }

    Foc_CurrentLoop_SetFF(ff_enable);
    Foc_CurrentLoop_SetGains(g_cl_kp, g_cl_ki, g_cl_kd);
    Foc_CurrentLoop_SetRef((float)g_cl_id_ref_ma / 1000.0f,
                           (float)g_cl_iq_ref_ma / 1000.0f);

    /* 从速度模式返回时，恢复 mode 4/8 自己的电压上限 / 积分项限幅 / 反馈 EMA */
    Foc_CurrentLoop_SetVmax(g_cl_vmax_watch);
    Foc_CurrentLoop_SetITermMax(g_cl_iterm_max_watch);
    g_cl_fb_alpha = CL_FB_ALPHA_INIT;

    MAIN_D("[CL] ---- enter CURRENT%s, params ----", ff_enable ? "_FF" : "");
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
    MAIN_D("[CL] LIMIT: iref_max_x100=%ld A, vmax_x100=%ld V, iterm_x100=%ld V",
           (long)(g_cl_iref_max * 100.0f),
           (long)(g_cl_vmax_watch * 100.0f),
           (long)(g_cl_iterm_max_watch * 100.0f));
    if (ff_enable) {
        MAIN_D("[CL] FF:    Rs_x1000=%ld Ls_uh_x10=%ld flux_mVs=%ld",
               (long)(g_cl_ff_rs_ohm * 1000.0f),
               (long)(g_cl_ff_ls_uh * 10.0f),
               (long)(g_cl_ff_flux_vs * 1000000.0f));
    }
    MAIN_D("[CL] STATE: aligned=%u offset_cal=%u fp_state=%d",
           (unsigned)g_motor_is_aligned,
           (unsigned)g_motor_is_offset_calibrated,
           (int)g_fp_state);
    MAIN_D("[CL] ------------------------------");

    Foc_Mode_SetMode(FOC_MODE_CURRENT);
    MAIN_D("[CL] current loop%s entered (ref id=%ldmA iq=%ldmA, ramp from 0).",
           ff_enable ? "_FF" : "",
           (long)g_cl_id_ref_ma, (long)g_cl_iq_ref_ma);
    return true;
}

/*=============================================================================
 * 进入 SPEED：速度外环 + 电流环串级（mode 9）
 *=============================================================================*/
static bool Enter_SpeedLoop(void)
{
    if (!CurrentLoop_EntryCheck()) {
        g_mode_cmd = 0;
        Foc_Mode_SetMode(FOC_MODE_IDLE);
        return false;
    }

    /* 切到参考 mode 40 的电流环参数（0.1/300、6.2V、无反馈 EMA），关 dq 前馈 */
    Foc_Speed_ConfigCurrentLoop();
    s_spd_vmax_applied  = g_spd_cur_vmax_v;
    s_spd_iterm_applied = g_spd_cur_iterm_v;

    Foc_Speed_Start();

    MAIN_D("[SPD] ---- enter SPEED, params ----");
    MAIN_D("[SPD] PI: kp_x10000=%ld ki_x10000=%ld accel_rpm_s=%ld tgt_max_rpm=%ld",
           (long)(g_spd_kp * 10000.0f),
           (long)(g_spd_ki * 10000.0f),
           (long)g_spd_accel_rpm_s,
           (long)SPD_TARGET_MAX_RPM);
    MAIN_D("[SPD] LIMIT: iq_mA=%ld iterm_mA=%ld",
           (long)g_spd_iq_limit_ma, (long)g_spd_iterm_max_ma);
    MAIN_D("[SPD] CUR: kp_x10000=%ld ki_x100=%ld vmax_x100=%ld iterm_x100=%ld "
           "fb_alpha_x100=%ld ff=0",
           (long)(g_spd_cur_kp * 10000.0f),
           (long)(g_spd_cur_ki * 100.0f),
           (long)(g_spd_cur_vmax_v * 100.0f),
           (long)(g_spd_cur_iterm_v * 100.0f),
           (long)(g_spd_cur_fb_alpha * 100.0f));
    MAIN_D("[SPD] ------------------------------");

    Foc_Mode_SetMode(FOC_MODE_SPEED);
    MAIN_D("[SPD] speed loop entered, target reset to 0 (set g_spd_target_rpm in Watch).");
    return true;
}

/*=============================================================================
 * 进入 POSITION：位置环 + 速度环 + 电流环三环串级（mode 10）
 *=============================================================================*/
static bool Enter_PositionLoop(void)
{
    if (!CurrentLoop_EntryCheck()) {
        g_mode_cmd = 0;
        Foc_Mode_SetMode(FOC_MODE_IDLE);
        return false;
    }

    /* 速度内环沿用 mode 9 的电流环参数（0.1/300、6.2V、无反馈 EMA） */
    Foc_Speed_ConfigCurrentLoop();
    s_spd_vmax_applied  = g_spd_cur_vmax_v;
    s_spd_iterm_applied = g_spd_cur_iterm_v;

    /* SetMode 内部会：清速度环 -> 复位电流环 -> 以当前位置为目标复位位置环 */
    Foc_Mode_SetMode(FOC_MODE_POSITION);

    MAIN_D("[POS] ---- enter POSITION, params ----");
    MAIN_D("[POS] hold_pos_x1000=%ld rev", (long)(g_pos_fb_rev * 1000.0f));
    MAIN_D("[POS] PD: kp_x10=%ld ki_x10=%ld kd_x10=%ld spd_lim=%ld iterm=%ld",
           (long)(g_pos_kp * 10.0f),
           (long)(g_pos_ki * 10.0f),
           (long)(g_pos_kd * 10.0f),
           (long)g_pos_speed_limit_rpm,
           (long)g_pos_iterm_max_rpm);
    MAIN_D("[POS] LIMIT: err_max_x1000=%ld db_x1000=%ld trip_rev=%ld back_gain_x100=%ld",
           (long)(g_pos_err_max_rev * 1000.0f),
           (long)(g_pos_deadband_rev * 1000.0f),
           (long)g_pos_limit_rev,
           (long)(g_pos_back_gain * 100.0f));
    MAIN_D("[POS] ------------------------------");
    MAIN_D("[POS] set g_pos_target_rev in Watch; g_pos_zero_req=1 for zero.");
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
        cfg.baudrate = 115200;//921600;
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

    /* ---- 5. 模式 / 保护 / 电流环 / 速度环 / 位置环 ---- */
    Foc_Mode_Init();
    Foc_Protect_Init(NULL);
    Foc_CurrentLoop_Init();
    Foc_Speed_Init();
    Foc_Position_Init();

    /* ---- 6. 控制层（EOCA 中断使能）---- */
    Foc_Control_Init();

    /* ---- 7. 上电零偏校准 ---- */
    (void)PowerOn_OffsetCal();

    MAIN_D(" Ready! g_mode_cmd: 0=IDLE 1=LOCK 2=IF 3=CALIB 4=CURRENT 5=SWEEP 6=REV 7=RAMP 8=CURRENT_FF 9=SPEED 10=POSITION");
    MAIN_D(" Mode 3 = align, Mode 4 = current, Mode 5 = sweep, Mode 6 = rev-count, Mode 7 = ramp-align, Mode 8 = current+FF, Mode 9 = speed+current, Mode 10 = position+speed+current.");

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
                if (!Enter_CurrentLoop(0u)) {
                    last_mode_cmd = 0;
                }
                break;

            case 8:
                MAIN_D("[CL8] request: fp_state=%d fault=0x%08lX trip_cnt=%lu",
                       (int)g_fp_state,
                       (unsigned long)g_fp_fault,
                       (unsigned long)g_fp_trip_cnt);
                if (!Enter_CurrentLoop(1u)) {
                    last_mode_cmd = 0;
                }
                break;

            case 9:
                if (!Enter_SpeedLoop()) {
                    last_mode_cmd = 0;
                }
                break;

            case 10:
                if (!Enter_PositionLoop()) {
                    last_mode_cmd = 0;
                }
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

            case 6:
                MAIN_D("[REV] request: fp_state=%d fault=0x%08lX trip_cnt=%lu",
                       (int)g_fp_state,
                       (unsigned long)g_fp_fault,
                       (unsigned long)g_fp_trip_cnt);
                MAIN_D("[REV] align %lums -> drag %u elec revs -> align %lums",
                       (unsigned long)g_rev_align_ms,
                       (unsigned)CFG_MOTOR_POLE_PAIRS,
                       (unsigned long)g_rev_align_ms);
                Foc_Mode_SetMode(FOC_MODE_IDLE);
                EncoderCalib_StartRevMeasure();
                s_align_running = 1;
                break;

            case 7:
                MAIN_D("[CAL7] request: fp_state=%d fault=0x%08lX trip_cnt=%lu",
                       (int)g_fp_state,
                       (unsigned long)g_fp_fault,
                       (unsigned long)g_fp_trip_cnt);
                MAIN_D("[CAL7] ramp 0->90->0 deg, ramp_ms=%lu hold_ms=%lu",
                       (unsigned long)g_align_ramp_ms,
                       (unsigned long)g_align_hold_ms);
                Foc_Mode_SetMode(FOC_MODE_IDLE);
                EncoderCalib_StartAlign7();
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

                    /* 每段 BETA/ALPHA 的三相平均电流（mA），用于核对相序/极性 */
                    {
                        int i;
                        for (i = 0; i < 4; i++) {
                            MAIN_D("[SWEEP] i%d: BETA th=%lddeg iu=%ld iv=%ld iw=%ld | "
                                   "ALPHA th=%lddeg iu=%ld iv=%ld iw=%ld",
                                   i,
                                   (long)((float)g_sweep_theta[i] * 0.0572958f),
                                   (long)g_sweep_i_ma[i][0],
                                   (long)g_sweep_i_ma[i][1],
                                   (long)g_sweep_i_ma[i][2],
                                   (long)((float)g_sweep_theta[i] * 0.0572958f) - 90,
                                   (long)g_sweep_i_ma[i][3],
                                   (long)g_sweep_i_ma[i][4],
                                   (long)g_sweep_i_ma[i][5]);
                        }
                    }
                } else if (EncoderCalib_IsRevMeasureDone()) {
                    MAIN_D("[REV] start=%ld end=%ld delta=%ld pp_x100=%ld slip=%u "
                           "cyc_min=%ld cyc_max=%ld",
                           (long)g_rev_start_cnt,
                           (long)g_rev_end_cnt,
                           (long)g_rev_delta,
                           (long)(g_rev_pp_meas * 100.0f),
                           (unsigned)g_rev_slip,
                           (long)g_rev_cyc_min,
                           (long)g_rev_cyc_max);
                    MAIN_D("[REV] pp_meas = CPR*pp_cfg/|delta| = %ld.%02ld",
                           (long)(g_rev_pp_meas),
                           (long)((g_rev_pp_meas - (float)(long)g_rev_pp_meas) * 100.0f));
                } else if (EncoderCalib_IsAlign7Done()) {
                    if (EncoderCalib_IsAlignOk()) {
                        MAIN_D("[CAL7] done: beta=%ld alpha=%ld moved=%ld offset=%ld dir=%d",
                               (long)g_align_beta_hw, (long)g_align_alpha_hw,
                               (long)g_align_moved, (long)g_align_offset,
                               (int)g_enc_elec_dir);
                        MAIN_D("[CAL7] Align OK");
                    } else {
                        MAIN_D("[CAL7] failed: beta=%ld alpha=%ld moved=%ld",
                               (long)g_align_beta_hw, (long)g_align_alpha_hw,
                               (long)g_align_moved);
                        MAIN_E("[CAL7] ramp align failed! Check voltage/ramp/hold.");
                    }
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
        if ((g_mode_cmd == 4) || (g_mode_cmd == 8)) {
            Foc_CurrentLoop_SetRef((float)g_cl_id_ref_ma / 1000.0f,
                                   (float)g_cl_iq_ref_ma / 1000.0f);
            Foc_CurrentLoop_SetGains(g_cl_kp, g_cl_ki, g_cl_kd);
        }

        /* SPEED/POSITION 模式电流环参数实时同步（速度环自身参数在 ISR 内每 5ms 同步） */
        if ((g_mode_cmd == 9) || (g_mode_cmd == 10)) {
            Foc_CurrentLoop_SetGains(g_spd_cur_kp, g_spd_cur_ki, 0.0f);
            g_cl_fb_alpha = g_spd_cur_fb_alpha;

            if (g_spd_cur_vmax_v != s_spd_vmax_applied) {
                s_spd_vmax_applied = g_spd_cur_vmax_v;
                Foc_CurrentLoop_SetVmax(g_spd_cur_vmax_v);
            }
            if (g_spd_cur_iterm_v != s_spd_iterm_applied) {
                s_spd_iterm_applied = g_spd_cur_iterm_v;
                Foc_CurrentLoop_SetITermMax(g_spd_cur_iterm_v);
            }
        }

        /* vmax 在线同步 */
        {
            static float s_last_vmax = 0.0f;
            if (g_cl_vmax_watch != s_last_vmax) {
                s_last_vmax = g_cl_vmax_watch;
                Foc_CurrentLoop_SetVmax(g_cl_vmax_watch);
            }
        }

        /* 积分项限幅在线同步 */
        {
            static float s_last_iterm = 0.0f;
            if (g_cl_iterm_max_watch != s_last_iterm) {
                s_last_iterm = g_cl_iterm_max_watch;
                Foc_CurrentLoop_SetITermMax(g_cl_iterm_max_watch);
            }
        }

        /* I 项抗饱和配置在线同步（措施掩码 / 条件积分门限 / 反计算系数） */
        {
            static uint32_t s_last_aw_mask = 0xFFFFFFFFu;
            static float    s_last_aw_thr  = -1.0f;
            static float    s_last_bg      = -1.0f;

            if ((g_cl_aw_mask != s_last_aw_mask) ||
                (g_cl_aw_cond_thr != s_last_aw_thr)) {
                s_last_aw_mask = g_cl_aw_mask;
                s_last_aw_thr  = g_cl_aw_cond_thr;
                Foc_CurrentLoop_SetAntiWindup(g_cl_aw_mask, g_cl_aw_cond_thr);
            }
            if (g_cl_back_gain != s_last_bg) {
                s_last_bg = g_cl_back_gain;
                Foc_CurrentLoop_SetBackGain(g_cl_back_gain);
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
        if (((g_mode_cmd == 4) || (g_mode_cmd == 8)) &&
            Foc_CurrentLoop_GetSatCnt() > 4000u) {
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

        /* CURRENT 关键变量周期打印（10ms 一次，观察过流前的状态） */
        if ((g_mode_cmd == 4) || (g_mode_cmd == 8)) {
            static uint32_t s_last_cl_log_ms = 0;
            uint32_t now_ms = tickTimer_GetCount();

            if ((now_ms - s_last_cl_log_ms) >= 10u) {
                s_last_cl_log_ms = now_ms;
                MAIN_D("[CL] fp=%d flt=0x%08lX trip=%lu "
                       "iq_r=%ld iq=%ld id=%ld iu=%ld iv=%ld iw=%ld "
                       "th=%ld cnt=%ld dcnt=%ld spd=%ld vraw=%ld lim=%ld "
                       "ff=%u we=%ld vffd=%ld vffq=%ld",
                       (int)g_fp_state,
                       (unsigned long)g_fp_fault,
                       (unsigned long)g_fp_trip_cnt,
                       (long)g_cl_iq_ref_ma,
                       (long)(g_fc_iq * 1000.0f),
                       (long)(g_fc_id * 1000.0f),
                       (long)(g_fc_iu * 1000.0f),
                       (long)(g_fc_iv * 1000.0f),
                       (long)(g_fc_iw * 1000.0f),
                       (long)g_fc_enc_elec_deg,
                       (long)g_fc_enc_count,
                       (long)g_fc_enc_delta,
                       (long)g_fc_enc_speed_rpm,
                       (long)(g_cl_vmag_raw * 100.0f),
                       (long)g_cl_limit_reason,
                       (unsigned)g_cl_ff_enable,
                       (long)g_cl_we_rad_s,
                       (long)(g_cl_vd_ff * 1000.0f),
                       (long)(g_cl_vq_ff * 1000.0f));
            }
        }

        /* SPEED 关键变量周期打印（10ms 一次） */
        if (g_mode_cmd == 9) {
            static uint32_t s_last_spd_log_ms = 0;
            uint32_t now_ms = tickTimer_GetCount();

            if ((now_ms - s_last_spd_log_ms) >= 10u) {
                s_last_spd_log_ms = now_ms;
                MAIN_D("[SPD] tgt=%ld ramp=%ld spd=%ld raw=%ld err=%ld "
                       "out=%ld I=%ld iq_r=%ld iq=%ld vraw=%ld lim=%ld itlim=%ld",
                       (long)g_spd_target_rpm,
                       (long)g_spd_ramp_rpm,
                       (long)g_spd_filt_rpm,
                       (long)g_spd_raw_rpm,
                       (long)g_spd_err_rpm,
                       (long)g_spd_out_ma,
                       (long)g_spd_iterm_ma,
                       (long)g_cl_iq_ref_ramped_ma,
                       (long)(g_fc_iq * 1000.0f),
                       (long)(g_cl_vmag_raw * 100.0f),
                       (long)g_cl_limit_reason,
                       (long)g_spd_iterm_max_ma);
            }
        }

        /* POSITION 参数与位置周期打印（500ms 一次） */
        if (g_mode_cmd == 10) {
            static uint32_t s_last_pos_log_ms = 0;
            uint32_t now_ms = tickTimer_GetCount();

            if ((now_ms - s_last_pos_log_ms) >= 500u) {
                s_last_pos_log_ms = now_ms;
                MAIN_D("[POS] kp_x10=%ld ki_x10=%ld kd_x10=%ld spd_lim=%ld iterm=%ld "
                       "err_max_x1000=%ld db_x1000=%ld trip_rev=%ld bg_x100=%ld "
                       "creep=%ld stall=%ld ff=%ld decel_x1000=%ld kpused_x10=%ld",
                       (long)(g_pos_kp * 10.0f),
                       (long)(g_pos_ki * 10.0f),
                       (long)(g_pos_kd * 10.0f),
                       (long)g_pos_speed_limit_rpm,
                       (long)g_pos_iterm_max_rpm,
                       (long)(g_pos_err_max_rev * 1000.0f),
                       (long)(g_pos_deadband_rev * 1000.0f),
                       (long)g_pos_limit_rev,
                       (long)(g_pos_back_gain * 100.0f),
                       (long)g_pos_creep_rpm,
                       (long)g_pos_stall_rpm,
                       (long)g_spd_ff_ma,
                       (long)(g_pos_decel_rev * 1000.0f),
                       (long)(g_pos_kp_used * 10.0f));
                MAIN_D("[POS] seg_x1000=%ld/%ld vh=%ld vm=%ld vl=%ld rpm",
                       (long)(g_pos_seg1_rev * 1000.0f),
                       (long)(g_pos_seg2_rev * 1000.0f),
                       (long)g_pos_spd_high_rpm,
                       (long)g_pos_spd_mid_rpm,
                       (long)g_pos_spd_low_rpm);
                MAIN_D("[POS] tgt_x1000=%ld ref_x1000=%ld pos_x1000=%ld rev, "
                       "vprof=%ld cmd=%ld rpm brake=%u",
                       (long)(g_pos_target_rev * 1000.0f),
                       (long)((g_pos_fb_rev + g_pos_err_rev) * 1000.0f),
                       (long)(g_pos_fb_rev * 1000.0f),
                       (long)g_pos_vprof_rpm,
                       (long)g_pos_cmd_rpm,
                       (unsigned)g_pos_brake);
            }
        }

        /* VOFA 数据发送（非阻塞，19 通道：ch0~2 位置环，ch3~16 原有，ch17~18 速度环） */
        if (!Usart3_Vofa_IsTxBusy() && g_vofa_adc_debug != 0) {
            int32_t cur[19];
            Foc_AdcDebug_VofaFill(cur);
            Usart3_Vofa_SendScaled(cur, 19, USART3_VOFA_SCALE_MILLI);
        }
    }
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
