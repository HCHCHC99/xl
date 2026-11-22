/**
 *******************************************************************************
 * @file  foc_position_loop.c
 * @brief L8 位置外环实现（减速包络）
 *******************************************************************************
 */

#include "foc_position_loop.h"
#include "foc_current_loop.h"
#include "foc_speed_loop.h"
#include "core/foc_feedback.h"
#include "cfg_motor.h"
#include "cfg_foc_table.h"   /* FOC_CFG_PWM_FREQ */

#include <rtthread.h>

#include <math.h>

#define POS_ISR_HZ          ((float)FOC_CFG_PWM_FREQ)
#define POS_DT_MS           ((float)POS_CTRL_DIV * 1000.0f / POS_ISR_HZ)

/* 换向清积分的最小指令幅值，rpm */
#define POS_REV_MIN_RPM     (5.0f)

/*---------------------------------------------------------------------------
 * 可调参数变量
 *---------------------------------------------------------------------------*/
volatile float   g_pos_target_rev        = 0.0f;
volatile uint8_t g_pos_zero_req          = 0u;

volatile float   g_pos_dec_rpm_s        = POS_DEC_RPM_S_INIT;
volatile float   g_pos_kd_rpm_per_rpm    = POS_KD_RPM_PER_RPM_INIT;
volatile float   g_pos_v_max_rpm         = POS_V_MAX_RPM_INIT;
volatile float   g_pos_iq_limit_ma       = POS_IQ_LIMIT_MA_INIT;
volatile float   g_pos_deadband_rev      = POS_DEADBAND_REV_INIT;
volatile float   g_pos_v_inpos_rpm       = POS_V_INPOS_RPM_INIT;
volatile float   g_pos_err_max_rev       = POS_ERR_MAX_REV_INIT;
volatile float   g_pos_limit_rev         = POS_LIMIT_REV_INIT;

/*---------------------------------------------------------------------------
 * 观测量
 *---------------------------------------------------------------------------*/
volatile uint8_t g_pos_active   = 0u;
volatile uint8_t g_pos_in_pos   = 0u;
volatile float   g_pos_fb_rev   = 0.0f;
volatile float   g_pos_err_rev  = 0.0f;
volatile float   g_pos_cmd_rpm  = 0.0f;

/*---------------------------------------------------------------------------
 * 内部状态
 *---------------------------------------------------------------------------*/
static bool     s_ready          = false;
static uint16_t s_div_cnt        = 0u;
static float    s_pos_offset_rev = 0.0f;
static float    s_fb_rev         = 0.0f;
static float    s_vcmd_last      = 0.0f;

/* 当前位置（圈），正方向与速度环一致 */
static float pos_read_rev(void)
{
    float raw = (float)Foc_Feedback_GetCount() * (float)Foc_Feedback_GetDir()
              / (float)CFG_ENCODER_CPR;

    return raw - s_pos_offset_rev;
}

/*=============================================================================
 * 初始化
 *=============================================================================*/
void Foc_Position_Init(void)
{
    if (s_ready) {
        return;
    }

    s_div_cnt        = 0u;
    s_pos_offset_rev = 0.0f;
    s_fb_rev         = 0.0f;
    s_vcmd_last      = 0.0f;

    g_pos_active  = 0u;
    g_pos_in_pos  = 0u;
    g_pos_fb_rev  = 0.0f;
    g_pos_err_rev = 0.0f;
    g_pos_cmd_rpm = 0.0f;

    s_ready = true;

    rt_kprintf("[POS] P-loop ready: div=%u dec=%ld rpm/s vmax=%ld "
               "db_x10000=%ld v_inpos=%ld\r\n",
               (unsigned)POS_CTRL_DIV,
               (long)g_pos_dec_rpm_s,
               (long)g_pos_v_max_rpm,
               (long)(g_pos_deadband_rev * 10000.0f),
               (long)g_pos_v_inpos_rpm);
}

/*=============================================================================
 * 进入 / 离开位置模式
 *=============================================================================*/
void Foc_Position_Reset(void)
{
    if (!s_ready) {
        return;
    }

    s_div_cnt   = 0u;
    s_fb_rev    = pos_read_rev();
    s_vcmd_last = 0.0f;

    g_pos_fb_rev  = s_fb_rev;
    g_pos_err_rev = 0.0f;
    g_pos_cmd_rpm = 0.0f;
    g_pos_active  = 0u;
    g_pos_in_pos  = 1u;

    g_pos_target_rev = s_fb_rev;
}

void Foc_Position_Stop(void)
{
    if (!s_ready) {
        return;
    }

    s_div_cnt   = 0u;
    s_vcmd_last = 0.0f;

    g_pos_err_rev = 0.0f;
    g_pos_cmd_rpm = 0.0f;
    g_pos_active  = 0u;
    g_pos_in_pos  = 0u;
}

/*=============================================================================
 * 每拍任务（内部 2ms 抽稀）
 *=============================================================================*/
void Foc_Position_Task20k(void)
{
    float pos;
    float target;
    float err_raw;
    float lim;
    float db;
    float err_max;
    float dec;
    float kd;
    float vmax;
    float iq_lim;
    float v_inpos;
    float vallow;
    float vcmd;

    if (!s_ready) {
        return;
    }

    s_div_cnt++;
    if (s_div_cnt < POS_CTRL_DIV) {
        return;
    }
    s_div_cnt = 0u;

    /* 零点请求：当前位置设为 0，目标清零 */
    if (g_pos_zero_req != 0u) {
        g_pos_zero_req   = 0u;
        s_pos_offset_rev = (float)Foc_Feedback_GetCount() * (float)Foc_Feedback_GetDir()
                         / (float)CFG_ENCODER_CPR;
        g_pos_target_rev = 0.0f;
    }

    pos = pos_read_rev();
    s_fb_rev = pos;
    g_pos_fb_rev = pos;

    /* 电流环未 active：不输出，速度指令归零 */
    if (!Foc_CurrentLoop_IsActive()) {
        g_pos_err_rev = 0.0f;
        g_pos_cmd_rpm = 0.0f;
        g_pos_active  = 0u;
        g_pos_in_pos  = 0u;
        g_spd_target_rpm = 0.0f;
        return;
    }
    g_pos_active = 1u;

    /* 参数在线同步 + 非法值保护 */
    lim = g_pos_limit_rev;
    if (lim < 0.0f) {
        lim = 0.0f;
    }
    db = g_pos_deadband_rev;
    if (db < 0.0f) {
        db = 0.0f;
    }
    err_max = g_pos_err_max_rev;
    if (err_max < 0.0f) {
        err_max = 0.0f;
    }
    dec = g_pos_dec_rpm_s;
    if (dec < 1.0f) {
        dec = 1.0f;
    }
    /* 规划减速度不得超过速度环目标斜坡，否则减速段会冲过目标 */
    if ((g_spd_accel_rpm_s > 0.0f) && (dec > g_spd_accel_rpm_s)) {
        dec = g_spd_accel_rpm_s;
    }
    kd = g_pos_kd_rpm_per_rpm;
    if (kd < 0.0f) {
        kd = 0.0f;
    }
    vmax = g_pos_v_max_rpm;
    if (vmax < 0.0f) {
        vmax = 0.0f;
    }
    iq_lim = g_pos_iq_limit_ma;
    if (iq_lim < 1.0f) {
        iq_lim = 1.0f;
    }
    v_inpos = g_pos_v_inpos_rpm;
    if (v_inpos < 0.0f) {
        v_inpos = 0.0f;
    }

    /* 目标软行程限位 */
    target = g_pos_target_rev;
    if (target >  lim) target =  lim;
    if (target < -lim) target = -lim;

    /* 减速包络：按 dec 减速恰好停在目标处
     *   d = v²/(120·dec)  ->  v = sqrt(120·dec·|err|) */
    err_raw = target - pos;

    if (fabsf(err_raw) <= db) {
        vcmd = 0.0f;
    } else {
        vallow = sqrtf(120.0f * dec * fabsf(err_raw));
        if (vallow > vmax) {
            vallow = vmax;
        }
        vcmd = (err_raw >= 0.0f) ? vallow : -vallow;

        if (kd > 0.0f) {
            vcmd -= kd * g_spd_filt_rpm;
            if (vcmd >  vmax) vcmd =  vmax;
            if (vcmd < -vmax) vcmd = -vmax;
        }
    }

    /* 换向清速度环积分，避免旧方向积分蓄力顶住制动 */
    if ((fabsf(err_raw) > 2.0f * db) &&
        (((vcmd > POS_REV_MIN_RPM) && (s_vcmd_last < -POS_REV_MIN_RPM)) ||
         ((vcmd < -POS_REV_MIN_RPM) && (s_vcmd_last > POS_REV_MIN_RPM)))) {
        Foc_Speed_ResetIntegral();
    }
    if ((vcmd > POS_REV_MIN_RPM) || (vcmd < -POS_REV_MIN_RPM)) {
        s_vcmd_last = vcmd;
    }

    g_pos_cmd_rpm     = vcmd;
    g_spd_target_rpm  = vcmd;
    g_spd_iq_limit_ma = iq_lim;

    /* 观测：误差钳位 + 到位标志 */
    {
        float err = err_raw;

        if (err >  err_max) err =  err_max;
        if (err < -err_max) err = -err_max;

        g_pos_in_pos = ((fabsf(err_raw) <= db) &&
                        (fabsf(g_spd_filt_rpm) <= v_inpos)) ? 1u : 0u;

        g_pos_err_rev = err;
    }
}
