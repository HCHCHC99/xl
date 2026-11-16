/**
 * @file    motor_config.c
 * @brief   电机配置表实例
 */

#include "motor_config.h"
#include "board_motor_cfg.h"
#include "drv/motor_drv_bldc_6step.h"
#include "fb/motor_fb_hall.h"
#include "ctrl/motor_ctrl_openloop.h"
#include <string.h>

/* ============================================================
 * 回转电机 PWM 配置
 * ============================================================ */
static const motor_pwm_cfg_t g_swing_pwm_cfg = {
    .pwm_freq_hz      = MOTOR_PWM_FREQ_HZ,
    .pwm_deadtime_ns  = MOTOR_PWM_DEADTIME_NS,
    .max_duty         = MOTOR_PWM_MAX_DUTY,
};

/* ============================================================
 * 回转电机 Hall 配置
 * ============================================================ */
static const motor_hall_cfg_t g_swing_hall_cfg = {
    .hall_state_table  = MOTOR_HALL_TABLE,
    .hall_active_level = MOTOR_HALL_ACTIVE_LEVEL,
};

/* ============================================================
 * 回转电机软启动/软停配置
 * ============================================================ */
static const soft_ramp_cfg_t g_swing_ramp_cfg = {
    .start_enabled = 1,
    .stop_enabled  = 1,
    .type          = RAMP_TYPE_LINEAR,
    .start_time_ms = 500,
    .stop_time_ms  = 500,
};

/* ============================================================
 * 回转电机过流保护配置
 * ============================================================ */
static const motor_oc_cfg_t g_swing_oc_cfg = {
    .rated_current_a10  = MOTOR_RATED_CURRENT_A10,

    .oc1_current_a10    = MOTOR_OC1_CURRENT_A10,
    .oc1_debounce_ms    = MOTOR_OC1_DEBOUNCE_MS,

    .oc2_current_a10    = MOTOR_OC2_CURRENT_A10,
    .oc2_debounce_ms    = MOTOR_OC2_DEBOUNCE_MS,
    .oc2_deboost_pct    = MOTOR_OC2_DEBOOST_PCT,

    .oc3_current_a10    = MOTOR_OC3_CURRENT_A10,
    .oc3_debounce_ms    = MOTOR_OC3_DEBOUNCE_MS,

    .auto_recover       = 0,        /* 三级过流需手动复位 */
};

/* ============================================================
 * 电机配置表
 * ============================================================ */
const motor_config_t g_motor_configs[] = {
    /* ----- 回转电机：BLDC + 六步换向 + Hall ----- */
    {
        .name = "swing_motor",

        /* D1：电机类型 */
        .type       = MOTOR_TYPE_BLDC,
        .pole_pairs = MOTOR_POLE_PAIRS,

        /* D2：驱动方式 */
        .drv_hw     = MOTOR_DRV_HW_PWM,
        .drv_hw_cfg = &g_swing_pwm_cfg,

        /* D3：换向信号 */
        .comm_type = MOTOR_COMM_HALL,
        .comm_cfg  = &g_swing_hall_cfg,

        /* D4：驱动算法 */
        .default_algo = MOTOR_ALGO_SIX_STEP,

        /* D5：工作模式 */
        .default_mode = MOTOR_MODE_SPEED,

        /* D6：软启动/软停 */
        .ramp_cfg = &g_swing_ramp_cfg,

        /* 安全层：过流保护 */
        .oc_cfg = &g_swing_oc_cfg,

        /* 三层接口 */
        .drv_ops  = &motor_drv_bldc_6step_ops,
        .fb_ops   = &motor_fb_hall_ops,
        .ctrl_ops = &motor_ctrl_openloop_ops,   /* 先用开环 */

        /* 实时性 */
        .update_period_ms     = MOTOR_UPDATE_PERIOD_MS,
        .drv_update_period_us = MOTOR_DRV_PERIOD_US,
    },
};

const uint32_t g_motor_config_count =
    sizeof(g_motor_configs) / sizeof(g_motor_configs[0]);

/* ============================================================
 * 查找 API
 * ============================================================ */
const motor_config_t* motor_config_find(const char *name)
{
    if (name == NULL) return NULL;

    for (uint32_t i = 0; i < g_motor_config_count; i++) {
        if (strcmp(g_motor_configs[i].name, name) == 0) {
            return &g_motor_configs[i];
        }
    }
    return NULL;
}

const motor_config_t* motor_config_default(void)
{
    if (g_motor_config_count == 0) return NULL;
    return &g_motor_configs[0];
}
