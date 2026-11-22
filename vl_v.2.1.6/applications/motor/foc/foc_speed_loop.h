/**
 *******************************************************************************
 * @file  foc_speed_loop.h
 * @brief L7 速度外环（5ms 滑窗测速 + 1ms 速度 PI，输出 iq_ref）
 *
 *        20kHz ISR 内每 20 拍（1ms）执行一次；测速用 100 拍滑窗
 *        速度 PI 输出直接写电流环 iq_ref，id_ref 恒为 0
 *******************************************************************************
 */

#ifndef FOC_SPEED_LOOP_H
#define FOC_SPEED_LOOP_H

#include <stdbool.h>
#include <stdint.h>

#include "pid_ctrl.h"

/* 速度环执行周期：20 拍 = 1ms（20kHz） */
#define SPD_CTRL_DIV                (20u)

/* 测速滑动窗口长度：100 拍 = 5ms */
#define SPD_WIN_TICKS               (100u)

#define SPD_KP_INIT                 (5.0f)
#define SPD_KI_INIT                 (100.0f)
#define SPD_KD_INIT                 (0.0f)

#define SPD_AW_MASK_INIT            (PID_AW_CLAMP | PID_AW_BACK_CALC)
#define SPD_AW_COND_THR_INIT        (0.0f)

#define SPD_IQ_LIMIT_MA_INIT        (3400.0f)
#define SPD_ITERM_MAX_MA_INIT       (2000.0f)

#define SPD_FF_MA_INIT              (200.0f)
#define SPD_FF_DEADBAND_RPM         (2.0f)

#define SPD_ACCEL_RPM_S_INIT        (1000.0f)
#define SPD_TARGET_MAX_RPM          (7800.0f)
#define SPD_FB_ALPHA                (0.5f)
#define SPD_ENC_DELTA_CLAMP         (36)

/* 速度模式下的电流环参数初值 */
#define SPD_CUR_KP_INIT             (0.1f)
#define SPD_CUR_KI_INIT             (300.0f)
#define SPD_CUR_KD_INIT             (0.0f)
#define SPD_CUR_VMAX_INIT           (6.2f)
#define SPD_CUR_ITERM_MAX_INIT      (6.0f)
#define SPD_CUR_FB_ALPHA_INIT       (1.0f)

/*=============================================================================
 * 可调参数变量（Watch 可改）
 *=============================================================================*/
extern volatile float    g_spd_target_rpm;
extern volatile float    g_spd_kp;
extern volatile float    g_spd_ki;
extern volatile float    g_spd_kd;
extern volatile float    g_spd_accel_rpm_s;
extern volatile float    g_spd_iq_limit_ma;
extern volatile float    g_spd_iterm_max_ma;
extern volatile float    g_spd_ff_ma;
extern volatile float    g_spd_back_gain;
extern volatile float    g_spd_fb_alpha;
extern volatile uint32_t g_spd_aw_mask;
extern volatile float    g_spd_aw_cond_thr;

extern volatile float    g_spd_cur_kp;
extern volatile float    g_spd_cur_ki;
extern volatile float    g_spd_cur_kd;
extern volatile float    g_spd_cur_vmax_v;
extern volatile float    g_spd_cur_iterm_v;
extern volatile float    g_spd_cur_fb_alpha;

/*=============================================================================
 * 观测量
 *=============================================================================*/
extern volatile uint8_t g_spd_active;
extern volatile float   g_spd_raw_rpm;
extern volatile float   g_spd_filt_rpm;
extern volatile float   g_spd_ramp_rpm;
extern volatile float   g_spd_err_rpm;
extern volatile float   g_spd_out_ma;
extern volatile float   g_spd_iterm_ma;

/*=============================================================================
 * API
 *=============================================================================*/
void Foc_Speed_Init(void);
void Foc_Speed_Start(void);
void Foc_Speed_Stop(void);
void Foc_Speed_ResetIntegral(void);
void Foc_Speed_Task20k(void);
void Foc_Speed_ConfigCurrentLoop(void);

#endif /* FOC_SPEED_LOOP_H */
