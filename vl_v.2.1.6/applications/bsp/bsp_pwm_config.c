/**
 * @file    bsp_pwm_config.c
 * @brief   三相 PWM 配置表
 * @note    改频率、死区、max_duty 只改这个文件
 */

#include "bsp_pwm_config.h"
#include "bsp_pwm.h"
#include "../motor/board_motor_cfg.h"

/* ============================================================
 * 回转轴 PWM 配置
 * ============================================================ */
const hw_pwm_cfg_t g_swing_pwm_cfg = {
    .name        = "swing_pwm",
    .priv        = &g_bsp_pwm_priv,
    .freq_hz     = 20000U,
    .deadtime_ns = 1000U,
    .max_duty    = MOTOR_PWM_MAX_DUTY,
};

