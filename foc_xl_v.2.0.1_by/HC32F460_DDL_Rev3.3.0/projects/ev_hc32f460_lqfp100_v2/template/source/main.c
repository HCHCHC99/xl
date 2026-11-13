/*=============================================================================
 * main.c  â€? FOC åº”ç”¨å±‚å…¥å£ï¼ˆå¤šæ¨¡å¼?+ æ ¡å‡† + ä¿æŠ¤ + ç”µæµçŽ?+ é€Ÿåº¦çŽ?+ ä½ç½®çŽ?+ VOFAï¼? *
 * æ¨¡å¼è¯­ä¹‰ï¼? *   Mode 0 IDLE    : å…¨ä½Žï¼Œå®‰å…¨åœæœ? *   Mode 1 LOCK    : 50% é›¶çŸ¢é‡ï¼Œé”æ­»
 *   Mode 2 IF      : q è½´åŠ åŽ?+ è§’åº¦è‡ªå¢žï¼Œå¼€çŽ¯æ‹–åŠ? *   Mode 3 CALIB   : å•æ¬¡æ­£å¼æ ¡å‡†ï¼ˆéžé˜»å¡žï¼?ms/20kHz ä¸­æ–­æŽ¨è¿›ï¼? *                    ZERO(é›¶çŸ¢é‡é‡‡é›¶å) -> BETA(90) -> ALPHA(0)
 *                    -> åˆ¤æ® -> å†?offset/dir/g_motor_is_aligned
 *   Mode 4 CURRENT : æœ‰æ„Ÿé—­çŽ¯ç”µæµçŽ¯ï¼ˆç¡¬æ€§è¦æ±‚å·²å¯¹é½ + é›¶åå·²æ ¡å‡†ï¼‰
 *   Mode 5 SWEEP   : æ‰«è§’æµ‹é‡ï¼?/90/180/270 å››ä¸ªèµ·å§‹è§’å„è·‘ä¸€éä¸¤ç‚¹å¯¹é½ï¼‰ï¼? *                    åªè®°å½•ç»“æžœï¼Œä¸å†™ offset/dirï¼Œä¸ç½?g_motor_is_aligned
 *   Mode 6 REV     : ä¸€åœˆè®¡æ•°æµ‹é‡? *   Mode 7 RAMP    : æ–œå¡å¯¹é½ï¼?->90->0ï¼? *   Mode 8 CURRENT_FF : ç”µæµçŽ?+ dq è§£è€?åç”µåŠ¨åŠ¿å‰é¦ˆ
 *   Mode 9 SPEED   : é€Ÿåº¦å¤–çŽ¯ + ç”µæµçŽ¯ä¸²çº? *   Mode 10 POSITION : ä½ç½®å¤–çŽ¯ï¼ˆä¸‰æ®µå¼ï¼?> é€Ÿåº¦çŽ?-> ç”µæµçŽ? *=============================================================================*/

#include "main.h"
#include "hc32_ll.h"
#include "hc32_ll_clk.h"
#include "hc32_ll_gpio.h"
#include "hc32_ll_utility.h"
#include "hc32_ll_tmr4.h"
#include "TickTimer.h"

/* ---- é…ç½®å±?---- */
#include "cfg_motor.h"
#include "cfg_sampling.h"
#include "cfg_foc_table.h"

/* ---- é©±åŠ¨å±?---- */
#include "hw_board_pins.h"
#include "hw_gpio_init.h"
#include "hw_tmr4_pwm.h"
#include "hw_encoder.h"
#include "Timer0_Unit2.h"

/* ---- ç®—æ³•å±?/ é‡‡æ · ---- */
#include "svpwm.h"
#include "clarke_park.h"
#include "foc_sincos.h"
#include "motor_sampling.h"
#include "encoder_calib.h"

/* ---- æ—¶åŸº / æŽ§åˆ¶ ---- */
#include "timer6_timebase.h"
#include "foc_control.h"
#include "foc_mode.h"
#include "foc_protect.h"
#include "foc_current_loop.h"
#include "foc_speed_loop.h"
#include "foc_position_loop.h"
#include "param_ident.h"

/* ---- VOFA+ / æ—¥å¿— ---- */
#include "Usart3_Vofa.h"
#include "rtt_log.h"
#include "SEGGER_RTT.h"

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

/*=============================================================================
 * Watch å¯å†™å˜é‡
 *=============================================================================*/
volatile int32_t  g_mode_cmd       = 0;      /* 0=IDLE 1=LOCK 2=IF 3=CALIB 4=CURRENT 5=SWEEP 6=ALIGN_ALPHA 7=ALIGN_CUR 8=CURRENT_FF 9=SPEED 10=POSITION 11=PARAM_IDENT 12=OFFCAL_NOPWM 13=OFFCAL_PWM50 14=PARAM_IDENT_SCAN */
volatile int32_t  g_ol_freq_mhz    = 700;
volatile int32_t  g_ol_vq_mv       = 700;
volatile uint32_t g_alpha_align_ms = 500u;   /* mode 6 α 轴强拖保持时间 ms */
volatile uint32_t g_alpha_lock_ms  = 300u;   /* mode 6 进入前 2/2/2 零矢量稳定时间 ms */
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

/* éžé˜»å¡žå¯¹é½?æ‰«è§’è¿è¡Œæ ‡å¿—ï¼ˆä¸»å¾ªçŽ¯å†…éƒ¨ï¼?*/
static uint8_t s_align_running = 0;

/* é€Ÿåº¦æ¨¡å¼ç”µæµçŽ¯å‚æ•°å·²ä¸‹å‘å€¼ï¼ˆç”¨äºŽ Watch æ”¹åŠ¨æ£€æµ‹ï¼‰ */
static float s_spd_vmax_applied  = 0.0f;
static float s_spd_iterm_applied = 0.0f;

/*=============================================================================
 * é‡‡æ ·æ ‡å®š
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
 * VOFA+ æ•°æ®å¸§å¡«å…?(19ch)
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
    /* ch0~2: ä½ç½®çŽ?æœ€ç»ˆç›®æ ?/ æœ‰æ•ˆç»™å®š / å½“å‰ä½ç½®ï¼ˆmrevï¼ŒæŽ¥æ”¶ç«¯ä¹?0.001ï¼?     *   æœ‰æ•ˆç»™å®š = å½“å‰ pos + é™å¹…/æ­»åŒºåŽçš„ errï¼Œå³ PI å®žé™…è·Ÿè¸ªçš„ä½ç½?*/
    cur[0] = (int32_t)(g_pos_target_rev * 1000.0f);
    cur[1] = (int32_t)((g_pos_fb_rev + g_pos_err_rev) * 1000.0f);
    cur[2] = (int32_t)(g_pos_fb_rev * 1000.0f);

    /* ch3~6: ç”µæµ A (ä¼?mA, æŽ¥æ”¶ç«¯ä¹˜ 0.001) */
    cur[3] = (int32_t)(g_fc_i_total * 1000.0f);
    cur[4] = (int32_t)(g_fc_iu      * 1000.0f);
    cur[5] = (int32_t)(g_fc_iv      * 1000.0f);
    cur[6] = (int32_t)(g_fc_iw      * 1000.0f);

    /* ch7~8: çœŸå®ž iq / id (ä¼?mA) */
    cur[7] = (int32_t)(g_fc_iq      * 1000.0f);
    cur[8] = (int32_t)(g_fc_id      * 1000.0f);

    /* ch9~10: ç›®æ ‡ iq / id (mA ç›´æŽ¥ç»? */
    cur[9]  = g_cl_iq_ref_ma;
    cur[10] = g_cl_id_ref_ma;

    /* ch11~12: æ–œå¡å?iq / id (mA) */
    cur[11] = g_cl_iq_ref_ramped_ma;
    cur[12] = g_cl_id_ref_ramped_ma;

    /* ch13: ç”µè§’åº?(0.001 rad) = deg * pi/180 * 1000 */
    cur[13] = (int32_t)(g_fc_enc_elec_deg * 17.4532925f);

    /* ch14: æœºæ¢°è§’åº¦ (0.001 rad) */
    cur[14] = (int32_t)(g_fc_enc_mech_deg * 17.4532925f);

    /* ch15: ç¼–ç å™¨åœˆå†…è®¡æ•?0~4095ï¼Œå‘é€å€?Ã—1000 */
    {
        int32_t c = g_fc_enc_count % (int32_t)CFG_ENCODER_CPR;
        if (c < 0) c += (int32_t)CFG_ENCODER_CPR;
        cur[15] = c * 1000;
    }

    /* ch16: IF å¼ºæ‹–ç”µè§’åº¦ï¼ˆ0.001 radï¼‰ï¼Œä¸?ch13 åŒå•ä½ï¼Œå¯ç›´æŽ¥å¯¹æ¯?*/
    cur[16] = (int32_t)(g_fm_theta_e * 1000.0f);

    /* ch17~18: é€Ÿåº¦çŽ¯æ–œå¡ç›®æ ‡é€Ÿåº¦ / å®žæµ‹é€Ÿåº¦ï¼ˆmRPMï¼ŒæŽ¥æ”¶ç«¯ä¹?0.001ï¼?*/
    cur[17] = (int32_t)(g_spd_ramp_rpm * 1000.0f);
    cur[18] = (int32_t)(g_spd_filt_rpm * 1000.0f);

    return 19;
}

/*=============================================================================
 * ç³»ç»Ÿæ—¶é’Ÿ
 *=============================================================================*/
static void SystemClock_Config(void)
{
    Systick_config();
}

/*=============================================================================
 * è¿›å…¥ IF æ¨¡å¼
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
 * mode 6 / 7：一点式对齐
 *   0. 先切 2/2/2 零矢量（LOCK），保持 g_alpha_lock_ms 等输出稳定
 *   1. 读对齐前的编码器计数 x
 *   2. 加压矢量强拖：
 *        align_current = false -> 沿 α 轴（theta_e=0）
 *        align_current = true  -> 沿当前编码器电角度（上电小动作对齐）
 *      幅值都用 mode 2 的 g_ol_vq_mv
 *   3. 保持对齐期间再读一次编码器计数 y
 *   4. RTT 打印 y-x
 *   5. mode 7 额外用 y-x 修正 g_enc_elec_offset（mode 6 只测量不修正）
 *   6. 回 IDLE
 *=============================================================================*/
static void Align_OnePoint(bool align_current)
{
    int32_t  x_cnt, y_cnt, delta;
    int32_t  old_off, new_off;
    float    v;
    float    th;
    uint32_t t0;

    if (Foc_Mode_GetMode() != FOC_MODE_IDLE) {
        MAIN_E("[ALIGN6/7] refuse: mode not IDLE (mode=%d).", (int)Foc_Mode_GetMode());
        return;
    }
    if ((g_align_active != 0u) || (s_align_running != 0u)) {
        MAIN_E("[ALIGN6/7] refuse: encoder calibration in progress.");
        return;
    }
    if (g_fp_state != FP_STATE_NORMAL) {
        MAIN_E("[ALIGN6/7] refuse: protection state=%d.", (int)g_fp_state);
        return;
    }

    v = (float)g_ol_vq_mv / 1000.0f;
    if (v <= 0.0f) {
        MAIN_E("[ALIGN6/7] refuse: invalid align voltage (v_mV=%ld).", (long)g_ol_vq_mv);
        return;
    }

    /* 步骤 0：先切 2/2/2 零矢量（LOCK），等输出稳定后再读 x */
    Foc_Mode_SetMode(FOC_MODE_LOCK);
    t0 = tickTimer_GetCount();
    while ((tickTimer_GetCount() - t0) < g_alpha_lock_ms) {
        hw_encoder_update();
        if (g_fp_state != FP_STATE_NORMAL) {
            break;
        }
    }

    /* 步骤 1：2/2/2 稳住后读对齐前计数 x */
    hw_encoder_update();
    x_cnt = hw_encoder_get_count();

    /* 步骤 2：freq=0 固定角度加压
     *   mode 6：vd 沿 α 轴
     *   mode 7：沿当前编码器电角度（用 vd/vq 合成到 alpha 轴方向） */
    if (align_current) {
        th = hw_encoder_get_elec_angle_rad(CFG_MOTOR_POLE_PAIRS);
        Foc_Mode_SetOpenloop(0.0f, v * sinf(th), v * cosf(th));
    } else {
        Foc_Mode_SetOpenloop(0.0f, 0.0f, v);
    }
    Foc_Mode_StartOpenloop();

    t0 = tickTimer_GetCount();
    while ((tickTimer_GetCount() - t0) < g_alpha_align_ms) {
        hw_encoder_update();
        if (g_fp_state != FP_STATE_NORMAL) {
            break;
        }
    }

    /* 步骤 3：仍保持对齐电压，取第二次计数 */
    hw_encoder_update();
    y_cnt = hw_encoder_get_count();

    delta = y_cnt - x_cnt;

    /* 步骤 4：打印 y-x */
    MAIN_D("%s v_mV=%ld lock_ms=%lu hold_ms=%lu x=%ld y=%ld delta=%ld",
           align_current ? "[ALIGN7]" : "[ALIGN6]",
           (long)g_ol_vq_mv,
           (unsigned long)g_alpha_lock_ms,
           (unsigned long)g_alpha_align_ms,
           (long)x_cnt, (long)y_cnt, (long)delta);

    /* 步骤 5：mode 7 用 y-x 修正电角度零点；mode 6 只测量 */
    if (align_current) {
        if (g_fp_state == FP_STATE_NORMAL) {
            old_off = g_enc_elec_offset;
            new_off = old_off + delta;
            new_off %= (int32_t)CFG_ENCODER_CPR;
            if (new_off < 0) {
                new_off += (int32_t)CFG_ENCODER_CPR;
            }
            g_enc_elec_offset = new_off;
            MAIN_D("[ALIGN7] offset updated: %ld -> %ld (delta=%ld)",
                   (long)old_off, (long)new_off, (long)delta);
        } else {
            MAIN_E("[ALIGN7] offset NOT updated: protection state=%d.",
                   (int)g_fp_state);
        }
    }

    /* 步骤 6：回 IDLE */
    Foc_Mode_SetMode(FOC_MODE_IDLE);
    hw_tmr4_pwm_set_all_low();
}

/*=============================================================================
 * ä¸Šç”µé›¶ç”µæµåç½®æ ¡å‡†ï¼ˆé˜»å¡žï¼? *=============================================================================*/
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
 * mode 13：PWM 零偏校准（50% 零矢量下重校电流零偏）
 *   桥臂状态与辨识/电流环运行点一致，结果覆盖上电无 PWM 校准值
 *=============================================================================*/
static bool OffsetCal_Pwm50(void)
{
    bool ok;

    if ((g_align_active != 0u) || (s_align_running != 0u)) {
        MAIN_E("[CAL13] refuse: encoder calibration in progress.");
        return false;
    }
    if (g_fp_state != FP_STATE_NORMAL) {
        MAIN_E("[CAL13] refuse: protection state=%d, clear fault first.",
               (int)g_fp_state);
        return false;
    }
    if (fabsf(g_fc_enc_speed_rpm) > 50.0f) {
        MAIN_E("[CAL13] refuse: motor moving (spd_x10=%ld).",
               (long)(g_fc_enc_speed_rpm * 10.0f));
        return false;
    }

    MAIN_D("[CAL13] PWM zero-vector offset cal start...");

    /* ISR 不碰 PWM，由本函数独占输出 */
    g_align_active = 1u;
    Foc_Mode_SetMode(FOC_MODE_IDLE);

    (void)hw_tmr4_pwm_resume_output();
    (void)hw_tmr4_pwm_set_duty3(50.0f, 50.0f, 50.0f);

    ok = Motor_Sampling_DoOffsetCalBlocking(300u);

    (void)hw_tmr4_pwm_set_all_low();
    g_align_active = 0u;

    if (ok) {
        MAIN_D("[CAL13] OK: iw=%u iv=%u (PWM 50%% zero vector)",
               (unsigned)g_cal_offset_iw,
               (unsigned)g_cal_offset_iv);
    } else {
        MAIN_E("[CAL13] TIMEOUT! current feedback unreliable.");
        g_motor_is_offset_calibrated = 0u;
    }

    return ok;
}

/*=============================================================================
 * ç”µæµçŽ¯å…¥å£æ£€æŸ? *=============================================================================*/
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
 * è¿›å…¥ CURRENTï¼šmode4 = çº?PIï¼›mode8 = PI + dq è§£è€?åç”µåŠ¨åŠ¿å‰é¦ˆ
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

    /* ä»Žé€Ÿåº¦æ¨¡å¼è¿”å›žæ—¶ï¼Œæ¢å¤ mode 4/8 è‡ªå·±çš„ç”µåŽ‹ä¸Šé™?/ ç§¯åˆ†é¡¹é™å¹?/ åé¦ˆ EMA */
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
 * è¿›å…¥ SPEEDï¼šé€Ÿåº¦å¤–çŽ¯ + ç”µæµçŽ¯ä¸²çº§ï¼ˆmode 9ï¼? *=============================================================================*/
static bool Enter_SpeedLoop(void)
{
    if (!CurrentLoop_EntryCheck()) {
        g_mode_cmd = 0;
        Foc_Mode_SetMode(FOC_MODE_IDLE);
        return false;
    }

    /* åˆ‡åˆ°å‚è€?mode 40 çš„ç”µæµçŽ¯å‚æ•°ï¼?.1/300/6.2V/æ— åé¦?EMAï¼‰ï¼Œå…?dq å‰é¦ˆ */
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
 * è¿›å…¥ POSITIONï¼šä¸‰æ®µå¼ä½ç½®çŽ?+ é€Ÿåº¦çŽ?+ ç”µæµçŽ¯ä¸‰çŽ¯ä¸²çº§ï¼ˆmode 10ï¼? *=============================================================================*/
static bool Enter_PositionLoop(void)
{
    if (!CurrentLoop_EntryCheck()) {
        g_mode_cmd = 0;
        Foc_Mode_SetMode(FOC_MODE_IDLE);
        return false;
    }

    /* é€Ÿåº¦å†…çŽ¯æ²¿ç”¨ mode 9 çš„ç”µæµçŽ¯å‚æ•°ï¼?.1/300/6.2V/æ— åé¦?EMAï¼?*/
    Foc_Speed_ConfigCurrentLoop();
    s_spd_vmax_applied  = g_spd_cur_vmax_v;
    s_spd_iterm_applied = g_spd_cur_iterm_v;

    /* SetMode å†…éƒ¨ä¼šï¼šæ¸…é€Ÿåº¦çŽ?-> å¤ä½ç”µæµçŽ?-> ä»¥å½“å‰ä½ç½®ä¸ºç›®æ ‡å¤ä½ä½ç½®çŽ?*/
    Foc_Mode_SetMode(FOC_MODE_POSITION);

    MAIN_D("[POS] ---- enter POSITION (dec envelope), params ----");
    MAIN_D("[POS] dec=%ld rpm/s kd_x1000=%ld vmax=%ld iq_lim=%ld",
           (long)g_pos_dec_rpm_s,
           (long)(g_pos_kd_rpm_per_rpm * 1000.0f),
           (long)g_pos_v_max_rpm,
           (long)g_pos_iq_limit_ma);
    MAIN_D("[POS] db_x10000=%ld limit_rev=%ld",
           (long)(g_pos_deadband_rev * 10000.0f),
           (long)g_pos_limit_rev);
    MAIN_D("[POS] ------------------------------");
    MAIN_D("[POS] set g_pos_target_rev in Watch; g_pos_zero_req=1 for zero.");
    return true;
}

/*=============================================================================
 * 进入 PARAM_IDENT：离线参数辨识（mode 11）
 *   要求 IDLE、电角度已对齐、电流零偏已校准、保护 NORMAL、转子静止
 *=============================================================================*/
static bool Enter_ParamIdent(void)
{
    if (Foc_Mode_GetMode() != FOC_MODE_IDLE) {
        MAIN_E("[PI] refuse: mode not IDLE (mode=%d).", (int)Foc_Mode_GetMode());
        return false;
    }
    if ((g_align_active != 0u) || (s_align_running != 0u)) {
        MAIN_E("[PI] refuse: encoder calibration in progress, wait until done.");
        return false;
    }
    if (!g_motor_is_aligned) {
        MAIN_E("[PI] refuse: elec angle NOT aligned, run Mode 3 first.");
        return false;
    }
    if (!g_motor_is_offset_calibrated) {
        MAIN_E("[PI] refuse: current offset NOT calibrated.");
        return false;
    }
    if (g_fp_state != FP_STATE_NORMAL) {
        MAIN_E("[PI] refuse: protection state=%d, clear fault first.",
               (int)g_fp_state);
        return false;
    }
    if (fabsf(g_fc_enc_speed_rpm) > 50.0f) {
        MAIN_E("[PI] refuse: motor moving (spd_x10=%ld).",
               (long)(g_fc_enc_speed_rpm * 10.0f));
        return false;
    }

    if (!ParamIdent_Start()) {
        MAIN_E("[PI] start failed: status=%u error=%u",
               (unsigned)g_param_ident.status, (unsigned)g_param_ident.error);
        return false;
    }

    Foc_Mode_SetMode(FOC_MODE_PARAM_IDENT);

    MAIN_D("[PI] ---- enter PARAM_IDENT (Rs/Ld/Lq) ----");
    MAIN_D("[PI] PI: kp_x10000=%ld ki_x100=%ld vmax_x100=%ld iterm_x100=%ld",
           (long)(g_pi_cur_kp * 10000.0f),
           (long)(g_pi_cur_ki * 100.0f),
           (long)(g_pi_cur_vmax_v * 100.0f),
           (long)(g_pi_cur_iterm_v * 100.0f));
    MAIN_D("[PI] EXCITE: Ld_mV=%ld Lq_mV=%ld pol_hold=%u vbus_x100=%ld",
           (long)(g_pi_ld_excite_v * 1000.0f),
           (long)(g_pi_lq_excite_v * 1000.0f),
           (unsigned)g_pi_pol_hold,
           (long)(g_pi_vbus_v * 100.0f));
    MAIN_D("[PI] BIAS: Ld_mA=%ld Lq_mA=%ld | Rs points 300/600/900 mA",
           (long)(g_pi_ld_bias_a * 1000.0f),
           (long)(g_pi_lq_bias_a * 1000.0f));
    MAIN_D("[PI] NOTE: lock rotor for Lq, keep bus = g_pi_vbus_v.");
    MAIN_D("[PI] ------------------------------");

    return true;
}

/*=============================================================================
 * 定点拆分：value*scale 四舍五入后按 digits 位小数拆成符号/整数/小数
 *   仅用于 RTT 打印，避免依赖 printf 浮点支持
 *   注意 scale = 显示单位换算系数 × 10^digits：
 *     Ω  3 位小数 -> 1 × 10^3 = 1000
 *     µH 2 位小数 -> 1e6 × 10^2 = 1e8
 *=============================================================================*/
static void ParamIdent_SplitFixed(float value, float scale, uint8_t digits,
                                  const char **sign, long *ip, long *fp)
{
    float   x = value * scale;
    int32_t d = 1;
    int32_t m;
    uint8_t i;

    for (i = 0u; i < digits; i++) {
        d *= 10;
    }

    if (x >= 0.0f) {
        m = (int32_t)(x + 0.5f);
    } else {
        m = (int32_t)(x - 0.5f);
    }

    if (m < 0) {
        *sign = "-";
        m = -m;
    } else {
        *sign = "";
    }

    *ip = (long)(m / d);
    *fp = (long)(m % d);
}

/*=============================================================================
 * 打印参数辨识结果（只读 g_param_ident / g_pi_*）
 *=============================================================================*/
static void ParamIdent_PrintResult(void)
{
    const char *s_rs;
    const char *s_ld;
    const char *s_lq;
    long rs_i, rs_f;
    long ld_i, ld_f;
    long lq_i, lq_f;

    MAIN_D("[PI] status=%u state=%u progress=%u error=%u sample=%u",
           (unsigned)g_param_ident.status,
           (unsigned)g_param_ident.state,
           (unsigned)g_param_ident.progress,
           (unsigned)g_param_ident.error,
           (unsigned)g_param_ident.sample_count);

    ParamIdent_SplitFixed(g_param_ident.result_Rs, 1000.0f, 3u,
                          &s_rs, &rs_i, &rs_f);
    ParamIdent_SplitFixed(g_param_ident.result_Ld, 1.0e8f, 2u,
                          &s_ld, &ld_i, &ld_f);
    ParamIdent_SplitFixed(g_param_ident.result_Lq, 1.0e8f, 2u,
                          &s_lq, &lq_i, &lq_f);

    MAIN_D("[PI] Rs=%s%ld.%03ld Om | Ld=%s%ld.%02ld uH | Lq=%s%ld.%02ld uH",
           s_rs, rs_i, rs_f, s_ld, ld_i, ld_f, s_lq, lq_i, lq_f);

    MAIN_D("[PI] offset_mV: rs=%ld ld=%ld lq=%ld",
           (long)(g_param_ident.voltage_offset * 1000.0f),
           (long)(g_param_ident.ld_voltage_offset * 1000.0f),
           (long)(g_param_ident.lq_voltage_offset * 1000.0f));

    MAIN_D("[PI] id_avg_mA=%ld,%ld,%ld ud_avg_mV=%ld,%ld,%ld",
           (long)(g_param_ident.id_avg[0] * 1000.0f),
           (long)(g_param_ident.id_avg[1] * 1000.0f),
           (long)(g_param_ident.id_avg[2] * 1000.0f),
           (long)(g_param_ident.ud_avg[0] * 1000.0f),
           (long)(g_param_ident.ud_avg[1] * 1000.0f),
           (long)(g_param_ident.ud_avg[2] * 1000.0f));

    MAIN_D("[PI] cnt: start=%ld end=%ld delta=%ld theta_mrad: start=%ld end=%ld",
           (long)g_pi_start_cnt,
           (long)g_pi_end_cnt,
           (long)(g_pi_end_cnt - g_pi_start_cnt),
           (long)(g_pi_start_theta * 1000.0f),
           (long)(g_pi_end_theta * 1000.0f));
}

/*=============================================================================
 * mode 14：12 个电角度扫描辨识
 *   每个角度先用固定电压矢量把转子拉到目标电角度，再完整跑一次
 *   Rs/Ld/Lq 辨识，结果记入数组，最后打印表格和平均值
 *=============================================================================*/
#define PI_SCAN_N   12u
#define PI_SCAN_ANGLE_STEP_DEG  30u

volatile uint8_t  g_scan_align_en = 1u;        /* 1 = 每个角度先对齐转子；0 = 不动转子只重复测量 */
volatile float    g_scan_align_v  = 0.8f;      /* 对齐矢量电压 V */
volatile uint32_t g_scan_align_ms = 300u;      /* 每个角度对齐保持时间 ms */
volatile float    g_scan_Rs[PI_SCAN_N];
volatile float    g_scan_Ld[PI_SCAN_N];
volatile float    g_scan_Lq[PI_SCAN_N];
volatile float    g_scan_fb_deg[PI_SCAN_N];    /* 对齐后实测电角度 */
volatile int32_t  g_scan_dcnt[PI_SCAN_N];      /* 辨识期间编码器计数变化 */
volatile int32_t  g_scan_dth_mdeg[PI_SCAN_N];  /* 辨识期间电角度变化 mdeg */
volatile uint8_t  g_scan_ok_cnt;
volatile float    g_scan_Rs_avg;
volatile float    g_scan_Ld_avg;
volatile float    g_scan_Lq_avg;
volatile int32_t  g_scan_idx = -1;

/* 输出固定电角度矢量（主循环独占 PWM，期间 g_align_active=1） */
static void IdentScan_OutputVector(float theta_rad, float v)
{
    float valpha;
    float vbeta;
    SvpwmCtx_t ctx;

    InvPark(v, 0.0f, theta_rad, &valpha, &vbeta);

    ctx.valpha = valpha;
    ctx.vbeta  = vbeta;
    ctx.vbus   = CFG_SAMPLING_SIM_VBUS_V;
    SVPWM_Calc(&ctx);

    (void)hw_tmr4_pwm_set_duty3(ctx.duty_u * 100.0f,
                                ctx.duty_v * 100.0f,
                                ctx.duty_w * 100.0f);
}

static void IdentScan_DelayMs(uint32_t ms)
{
    uint32_t t0 = tickTimer_GetCount();

    while ((tickTimer_GetCount() - t0) < ms) {
        hw_encoder_update();
    }
}

static bool IdentScan_WaitIdentDone(uint32_t timeout_ms)
{
    uint32_t t0 = tickTimer_GetCount();

    while (ParamIdent_IsRunning() &&
           (Foc_Mode_GetMode() == FOC_MODE_PARAM_IDENT)) {
        hw_encoder_update();
        if ((tickTimer_GetCount() - t0) > timeout_ms) {
            return false;
        }
    }
    return true;
}

static void IdentScan_PrintRow(uint8_t idx, uint32_t ang_deg)
{
    const char *s_rs;
    const char *s_ld;
    const char *s_lq;
    long rs_i, rs_f;
    long ld_i, ld_f;
    long lq_i, lq_f;

    ParamIdent_SplitFixed(g_scan_Rs[idx], 1000.0f, 3u, &s_rs, &rs_i, &rs_f);
    ParamIdent_SplitFixed(g_scan_Ld[idx], 1.0e8f, 2u, &s_ld, &ld_i, &ld_f);
    ParamIdent_SplitFixed(g_scan_Lq[idx], 1.0e8f, 2u, &s_lq, &lq_i, &lq_f);

    MAIN_D("[SCAN] %2u  %3lu  %s%3ld.%03ld  %s%4ld.%02ld  %s%4ld.%02ld  %6ld  %5ld  %4ld",
           (unsigned)idx,
           (unsigned long)ang_deg,
           s_rs, rs_i, rs_f,
           s_ld, ld_i, ld_f,
           s_lq, lq_i, lq_f,
           (long)g_scan_dcnt[idx],
           (long)g_scan_dth_mdeg[idx],
           (long)g_scan_fb_deg[idx]);
}

static void ParamIdent_ScanRun(void)
{
    uint8_t k;
    uint8_t ok_cnt = 0u;
    float sum_rs = 0.0f;
    float sum_ld = 0.0f;
    float sum_lq = 0.0f;
    const char *s_rs;
    const char *s_ld;
    const char *s_lq;
    long rs_i, rs_f;
    long ld_i, ld_f;
    long lq_i, lq_f;

    if (!g_motor_is_aligned) {
        MAIN_E("[SCAN] refuse: elec angle NOT aligned, run Mode 3 first.");
        return;
    }
    if (!g_motor_is_offset_calibrated) {
        MAIN_E("[SCAN] refuse: current offset NOT calibrated.");
        return;
    }
    if (g_fp_state != FP_STATE_NORMAL) {
        MAIN_E("[SCAN] refuse: protection state=%d, clear fault first.",
               (int)g_fp_state);
        return;
    }
    if (fabsf(g_fc_enc_speed_rpm) > 50.0f) {
        MAIN_E("[SCAN] refuse: motor moving (spd_x10=%ld).",
               (long)(g_fc_enc_speed_rpm * 10.0f));
        return;
    }

    for (k = 0u; k < PI_SCAN_N; k++) {
        g_scan_Rs[k]      = 0.0f;
        g_scan_Ld[k]      = 0.0f;
        g_scan_Lq[k]      = 0.0f;
        g_scan_fb_deg[k]  = 0.0f;
        g_scan_dcnt[k]    = 0;
        g_scan_dth_mdeg[k]= 0;
    }
    g_scan_ok_cnt  = 0u;
    g_scan_Rs_avg  = 0.0f;
    g_scan_Ld_avg  = 0.0f;
    g_scan_Lq_avg  = 0.0f;

    MAIN_D("[SCAN] ===== 12-angle ident: align_v_x100=%ld hold_ms=%lu =====",
           (long)(g_scan_align_v * 100.0f),
           (unsigned long)g_scan_align_ms);
    MAIN_D("[SCAN] id   ang   Rs(Om)        Ld(uH)        Lq(uH)      dCnt    dTh     fb");
    MAIN_D("[SCAN] ----------------------------------------------------------------------");

    for (k = 0u; k < PI_SCAN_N; k++) {
        float    theta   = (float)k * ((float)PI_SCAN_ANGLE_STEP_DEG
                                       * 0.01745329252f);
        uint32_t ang_deg = (uint32_t)k * PI_SCAN_ANGLE_STEP_DEG;

        g_scan_idx = (int32_t)k;

        if (g_scan_align_en != 0u) {
            /* 对齐：ISR 不碰 PWM，主循环独占输出把转子拉到目标电角度 */
            g_align_active = 1u;
            Foc_Mode_SetMode(FOC_MODE_IDLE);
            (void)hw_tmr4_pwm_resume_output();
            IdentScan_OutputVector(theta, g_scan_align_v);

            IdentScan_DelayMs(g_scan_align_ms);

            g_align_active = 0u;
        } else {
            /* 不动转子：确保输出全低、模式回 IDLE 再开始辨识 */
            Foc_Mode_SetMode(FOC_MODE_IDLE);
            (void)hw_tmr4_pwm_set_all_low();
            IdentScan_DelayMs(50u);
        }
        g_scan_fb_deg[k] = g_fc_enc_elec_deg;

        if (!ParamIdent_Start()) {
            MAIN_E("[SCAN] %lu deg: start failed (status=%u error=%u)",
                   (unsigned long)ang_deg,
                   (unsigned)g_param_ident.status,
                   (unsigned)g_param_ident.error);
            break;
        }
        Foc_Mode_SetMode(FOC_MODE_PARAM_IDENT);

        if (!IdentScan_WaitIdentDone(3000u)) {
            ParamIdent_Abort();
            MAIN_E("[SCAN] %lu deg: timeout, abort.", (unsigned long)ang_deg);
            break;
        }

        if (g_param_ident.status != PI_STATUS_SUCCESS) {
            MAIN_E("[SCAN] %lu deg: ident failed (status=%u error=%u)",
                   (unsigned long)ang_deg,
                   (unsigned)g_param_ident.status,
                   (unsigned)g_param_ident.error);
            continue;
        }

        g_scan_Rs[k] = g_param_ident.result_Rs;
        g_scan_Ld[k] = g_param_ident.result_Ld;
        g_scan_Lq[k] = g_param_ident.result_Lq;
        g_scan_dcnt[k] = g_pi_end_cnt - g_pi_start_cnt;
        {
            float dth = (g_pi_end_theta - g_pi_start_theta) * 57.29578f;

            if (dth > 180.0f)  dth -= 360.0f;
            if (dth < -180.0f) dth += 360.0f;
            g_scan_dth_mdeg[k] = (int32_t)(dth * 1000.0f);
        }

        ok_cnt++;
        sum_rs += g_scan_Rs[k];
        sum_ld += g_scan_Ld[k];
        sum_lq += g_scan_Lq[k];

        IdentScan_PrintRow(k, ang_deg);
    }

    MAIN_D("[SCAN] ----------------------------------------------------------------------");

    g_scan_ok_cnt = ok_cnt;
    if (ok_cnt > 0u) {
        g_scan_Rs_avg = sum_rs / (float)ok_cnt;
        g_scan_Ld_avg = sum_ld / (float)ok_cnt;
        g_scan_Lq_avg = sum_lq / (float)ok_cnt;

        ParamIdent_SplitFixed(g_scan_Rs_avg, 1000.0f, 3u, &s_rs, &rs_i, &rs_f);
        ParamIdent_SplitFixed(g_scan_Ld_avg, 1.0e8f, 2u, &s_ld, &ld_i, &ld_f);
        ParamIdent_SplitFixed(g_scan_Lq_avg, 1.0e8f, 2u, &s_lq, &lq_i, &lq_f);

        MAIN_D("[SCAN] AVG(%u/%u)  Rs=%s%ld.%03ld Om  Ld=%s%ld.%02ld uH  Lq=%s%ld.%02ld uH",
               (unsigned)ok_cnt, (unsigned)PI_SCAN_N,
               s_rs, rs_i, rs_f, s_ld, ld_i, ld_f, s_lq, lq_i, lq_f);
    } else {
        MAIN_E("[SCAN] no valid result.");
    }

    (void)hw_tmr4_pwm_set_all_low();
    g_align_active = 0u;
    Foc_Mode_SetMode(FOC_MODE_IDLE);
    g_scan_idx = -1;

    MAIN_D("[SCAN] ===== done =====");
}

/*=============================================================================
 * main
 *=============================================================================*/
int main(void)
{
    tmr4_pwm_config_t  pwmCfg;
    tmr4_pwm_err_t     pwmErr;
    int32_t            last_mode_cmd = -1;

    /* ---- 1. åŸºç¡€å¤–è®¾ ---- */
    SystemClock_Config();
    Foc_SinCos_Init();
    hw_gpio_init_all();

    SEGGER_RTT_Init();
    SEGGER_RTT_SetFlagsUpBuffer(0, SEGGER_RTT_MODE_NO_BLOCK_SKIP);

    {
        Usart3_HW_Config_t cfg = USART3_HW_CONFIG_DEFAULT;
        cfg.baudrate = 115200;
        Usart3_Vofa_Init(&cfg);
    }

    MAIN_D("==============================================");
    MAIN_D(" FOC Start: %s, %lu Hz", FOC_CFG_NAME, (unsigned long)FOC_CFG_PWM_FREQ);
    MAIN_D("==============================================");

    /* ---- 2. é‡‡æ ·å±?---- */
    Motor_Sampling_Init(&g_scale);

    /* ---- 3. æ—¶åŸºä¸Žå®šæ—¶å™¨ ---- */
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

    /* ---- 5. æ¨¡å¼ / ä¿æŠ¤ / ç”µæµçŽ?/ é€Ÿåº¦çŽ?/ ä½ç½®çŽ?---- */
    Foc_Mode_Init();
    Foc_Protect_Init(NULL);
    Foc_CurrentLoop_Init();
    Foc_Speed_Init();
    Foc_Position_Init();
    ParamIdent_Init();

    /* ---- 6. æŽ§åˆ¶å±‚ï¼ˆEOCA ä¸­æ–­ä½¿èƒ½ï¼?--- */
    Foc_Control_Init();

    /* ---- 7. ä¸Šç”µé›¶åæ ¡å‡† ---- */
    (void)PowerOn_OffsetCal();

    MAIN_D(" Ready! g_mode_cmd: 0=IDLE 1=LOCK 2=IF 3=CALIB 4=CURRENT 5=SWEEP 6=ALIGN_ALPHA 7=ALIGN_CUR 8=CURRENT_FF 9=SPEED 10=POSITION 11=PARAM_IDENT 12=OFFCAL_NOPWM 13=OFFCAL_PWM50 14=PARAM_IDENT_SCAN");
    MAIN_D(" Mode 3 = align, Mode 4 = current, Mode 5 = sweep, Mode 6 = one-point alpha align, Mode 7 = one-point current-angle align (+offset update), Mode 8 = current+FF, Mode 9 = speed+current, Mode 10 = position+speed+current, Mode 11 = Rs/Ld/Lq ident, Mode 12 = offset cal (no PWM), Mode 13 = offset cal (PWM 50%), Mode 14 = 12-angle ident scan.");

    /* ---- ä¸»å¾ªçŽ?---- */
    while (1) {
        hw_encoder_update();
        g_fc_enc_speed_rpm = hw_encoder_get_speed_rpm();

        /* ---- æ¨¡å¼åˆ‡æ¢ ---- */
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

            case 11:
                if (!Enter_ParamIdent()) {
                    g_mode_cmd = 0;
                    last_mode_cmd = 0;
                }
                break;

            case 12:
                /* 无 PWM 零偏校准：与上电校准同一套流程，覆盖当前零偏 */
                MAIN_D("[CAL12] request: no-PWM offset calibration.");
                Foc_Mode_SetMode(FOC_MODE_IDLE);
                (void)PowerOn_OffsetCal();
                g_mode_cmd    = 0;
                last_mode_cmd = 0;
                break;

            case 13:
                /* PWM 零偏校准：50% 零矢量下重校，覆盖当前零偏 */
                (void)OffsetCal_Pwm50();
                g_mode_cmd    = 0;
                last_mode_cmd = 0;
                break;

            case 14:
                /* 12 个电角度扫描辨识，表格 + 平均值由函数内部打印 */
                MAIN_D("[SCAN] request: 12-angle ident scan.");
                ParamIdent_ScanRun();
                g_mode_cmd    = 0;
                last_mode_cmd = 0;
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
                /* 一点式对齐（α 轴）：阻塞执行，内部打印 y-x 后回 IDLE */
                MAIN_D("[ALIGN6] request: one-point alpha align.");
                Align_OnePoint(false);
                g_mode_cmd    = 0;
                last_mode_cmd = 0;
                break;

            case 7:
                /* 一点式对齐（当前电角度）：上电小动作对齐 */
                MAIN_D("[ALIGN7] request: one-point current-angle align.");
                Align_OnePoint(true);
                g_mode_cmd    = 0;
                last_mode_cmd = 0;
                break;

            default:
                Foc_Mode_SetMode(FOC_MODE_IDLE);
                break;
            }
        }

        /* ---- éžé˜»å¡žå¯¹é½?æ‰«è§’ï¼šäº‹ä»¶æ¶ˆè´?+ æ”¶å°¾ ---- */
        if (s_align_running) {
            uint8_t evt = EncoderCalib_ConsumeEvent();

            if (evt == EC_EVT_BETA_DONE) {
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

                    /* æ¯æ®µ BETA/ALPHA çš„ä¸‰ç›¸å¹³å‡ç”µæµï¼ˆmAï¼‰ï¼Œç”¨äºŽæ ¸å¯¹ç›¸åº/æžæ€?*/
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

        /* IF å‚æ•°å®žæ—¶åŒæ­¥ */
        if (g_mode_cmd == 2) {
            Foc_Mode_SetOpenloop((float)g_ol_freq_mhz / 1000.0f,
                                 (float)g_ol_vq_mv    / 1000.0f,
                                 0.0f);
        }

        /* CURRENT å‚æ•°å®žæ—¶åŒæ­¥ */
        if ((g_mode_cmd == 4) || (g_mode_cmd == 8)) {
            Foc_CurrentLoop_SetRef((float)g_cl_id_ref_ma / 1000.0f,
                                   (float)g_cl_iq_ref_ma / 1000.0f);
            Foc_CurrentLoop_SetGains(g_cl_kp, g_cl_ki, g_cl_kd);
        }

        /* SPEED/POSITION æ¨¡å¼ç”µæµçŽ¯å‚æ•°å®žæ—¶åŒæ­¥ï¼ˆé€Ÿåº¦çŽ¯è‡ªèº«å‚æ•°åœ¨ ISR å†…æ¯ 5ms åŒæ­¥ï¼?*/
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

        /* vmax åœ¨çº¿åŒæ­¥ */
        {
            static float s_last_vmax = 0.0f;
            if (g_cl_vmax_watch != s_last_vmax) {
                s_last_vmax = g_cl_vmax_watch;
                Foc_CurrentLoop_SetVmax(g_cl_vmax_watch);
            }
        }

        /* ç§¯åˆ†é¡¹é™å¹…åœ¨çº¿åŒæ­?*/
        {
            static float s_last_iterm = 0.0f;
            if (g_cl_iterm_max_watch != s_last_iterm) {
                s_last_iterm = g_cl_iterm_max_watch;
                Foc_CurrentLoop_SetITermMax(g_cl_iterm_max_watch);
            }
        }

        /* I é¡¹æŠ—é¥±å’Œé…ç½®åœ¨çº¿åŒæ­¥ï¼ˆæŽªæ–½æŽ©ç ?/ æ¡ä»¶ç§¯åˆ†é—¨é™ / åè®¡ç®—ç³»æ•°ï¼‰ */
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

        /* å—é™è¯Šæ–­é•œåƒ */
        g_cl_limit_reason = (int32_t)Foc_CurrentLoop_GetLimitReason();
        g_cl_vmag_raw     = Foc_CurrentLoop_GetVmagRaw();
        g_cl_vmax         = Foc_CurrentLoop_GetVmax();
        g_cl_sat_cnt      = (uint32_t)Foc_CurrentLoop_GetSatCnt();
        g_cl_id_ref_ramped_ma = (int32_t)(Foc_CurrentLoop_GetIdRefRamped() * 1000.0f);
        g_cl_iq_ref_ramped_ma = (int32_t)(Foc_CurrentLoop_GetIqRefRamped() * 1000.0f);

        /* ç”µæµçŽ¯å—é™å‘Šè­?*/
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

        /* CURRENT å…³é”®å˜é‡å‘¨æœŸæ‰“å°ï¼?0ms ä¸€æ¬¡ï¼Œè§‚å¯Ÿè¿‡æµå‰çš„çŠ¶æ€ï¼‰ */
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

        /* SPEED å…³é”®å˜é‡å‘¨æœŸæ‰“å°ï¼?0ms ä¸€æ¬¡ï¼‰ */
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

        /* POSITION å‚æ•°ä¸Žä½ç½®å‘¨æœŸæ‰“å°ï¼ˆ500ms ä¸€æ¬¡ï¼‰ */
        if (g_mode_cmd == 10) {
            static uint32_t s_last_pos_log_ms = 0;
            uint32_t now_ms = tickTimer_GetCount();

            if ((now_ms - s_last_pos_log_ms) >= 500u) {
                s_last_pos_log_ms = now_ms;
                MAIN_D("[POS] dec=%ld rpm/s kd_x1000=%ld vmax=%ld iq_lim=%ld db_x10000=%ld",
                       (long)g_pos_dec_rpm_s,
                       (long)(g_pos_kd_rpm_per_rpm * 1000.0f),
                       (long)g_pos_v_max_rpm,
                       (long)g_pos_iq_limit_ma,
                       (long)(g_pos_deadband_rev * 10000.0f));
                MAIN_D("[POS] tgt_x1000=%ld pos_x1000=%ld err_x1000=%ld rev, "
                       "cmd=%ld rpm in_pos=%u",
                       (long)(g_pos_target_rev * 1000.0f),
                       (long)(g_pos_fb_rev * 1000.0f),
                       (long)(g_pos_err_rev * 1000.0f),
                       (long)g_pos_cmd_rpm,
                       (unsigned)g_pos_in_pos);
            }
        }

        /* VOFA æ•°æ®å‘é€ï¼ˆéžé˜»å¡žï¼Œ19 é€šé“ï¼šch0~2 ä½ç½®çŽ¯ï¼Œch3~16 åŽŸæœ‰ï¼Œch17~18 é€Ÿåº¦çŽ¯ï¼‰ */
        /* PARAM_IDENT 进度与结果（RTT） */
        if (g_mode_cmd == 11) {
            static uint32_t s_last_pi_log_ms = 0;
            uint32_t now_ms = tickTimer_GetCount();

            if ((now_ms - s_last_pi_log_ms) >= 100u) {
                const char *s_rs;
                long rs_i, rs_f;

                s_last_pi_log_ms = now_ms;
                ParamIdent_SplitFixed(g_param_ident.result_Rs, 1000.0f, 3u,
                                      &s_rs, &rs_i, &rs_f);

                MAIN_D("[PI] state=%u prog=%u tick=%lu Rs=%s%ld.%03ld Om id=%ld iq=%ld",
                       (unsigned)g_param_ident.state,
                       (unsigned)g_param_ident.progress,
                       (unsigned long)g_pi_tick_cnt,
                       s_rs, rs_i, rs_f,
                       (long)(g_fc_id * 1000.0f),
                       (long)(g_fc_iq * 1000.0f));
            }

            if (!ParamIdent_IsRunning() &&
                (Foc_Mode_GetMode() != FOC_MODE_PARAM_IDENT)) {
                ParamIdent_PrintResult();
                g_mode_cmd    = 0;
                last_mode_cmd = 0;
            }
        }

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
