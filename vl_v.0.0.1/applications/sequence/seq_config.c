/**
 * @file    seq_config.c
 * @brief   序列配置表实例
 */

#include "seq_config.h"
#include <stddef.h>

/* ============================================================
 * 一键右中序列
 * 步骤：升+25° → 回转至+120° → 升+34°
 * ============================================================ */
static const seq_step_t g_steps_right_middle[] = {
    {
        .name       = "Raise to +25",
        .action     = SEQ_ACT_LIFT_UP,
        .condition  = SEQ_COND_LIFT_GE,
        .target     = 250,
        .timeout_ms = 15000,
    },
    {
        .name       = "Move swing to +120",
        .action     = SEQ_ACT_SWING_MOVE_TO,
        .condition  = SEQ_COND_SWING_NEAR,
        .target     = 1200,
        .timeout_ms = 30000,
    },
    {
        .name       = "Raise to +34",
        .action     = SEQ_ACT_LIFT_UP,
        .condition  = SEQ_COND_LIFT_GE,
        .target     = 340,
        .timeout_ms = 15000,
    },
};

const seq_config_t g_seq_right_middle = {
    .name = "right_middle",
    .type = SEQ_TYPE_RIGHT_MIDDLE,
    .steps = g_steps_right_middle,
    .step_count = sizeof(g_steps_right_middle) / sizeof(g_steps_right_middle[0]),
    .default_timeout_ms = 30000,
};

/* ============================================================
 * 一键右后序列
 * 步骤：升+25° → 回转至+215° → 升+34°
 * ============================================================ */
static const seq_step_t g_steps_right_behind[] = {
    {
        .name       = "Raise to +25",
        .action     = SEQ_ACT_LIFT_UP,
        .condition  = SEQ_COND_LIFT_GE,
        .target     = 250,
        .timeout_ms = 15000,
    },
    {
        .name       = "Move swing to +215",
        .action     = SEQ_ACT_SWING_MOVE_TO,
        .condition  = SEQ_COND_SWING_NEAR,
        .target     = 2150,
        .timeout_ms = 30000,
    },
    {
        .name       = "Raise to +34",
        .action     = SEQ_ACT_LIFT_UP,
        .condition  = SEQ_COND_LIFT_GE,
        .target     = 340,
        .timeout_ms = 15000,
    },
};

const seq_config_t g_seq_right_behind = {
    .name = "right_behind",
    .type = SEQ_TYPE_RIGHT_BEHIND,
    .steps = g_steps_right_behind,
    .step_count = sizeof(g_steps_right_behind) / sizeof(g_steps_right_behind[0]),
    .default_timeout_ms = 30000,
};

/* ============================================================
 * 一键回位序列
 * 步骤：升+25° → 回转归零 → 升+34° → 降至0° → 保持2s
 * ============================================================ */
static const seq_step_t g_steps_home[] = {
    {
        .name       = "Raise to +25",
        .action     = SEQ_ACT_LIFT_UP,
        .condition  = SEQ_COND_LIFT_GE,
        .target     = 250,
        .timeout_ms = 15000,
    },
    {
        .name       = "Move swing to zero",
        .action     = SEQ_ACT_SWING_MOVE_TO,
        .condition  = SEQ_COND_SWING_NEAR,
        .target     = 0,
        .timeout_ms = 30000,
    },
    {
        .name       = "Raise to +34",
        .action     = SEQ_ACT_LIFT_UP,
        .condition  = SEQ_COND_LIFT_GE,
        .target     = 340,
        .timeout_ms = 15000,
    },
    {
        .name       = "Descend to zero",
        .action     = SEQ_ACT_LIFT_DOWN,
        .condition  = SEQ_COND_LIFT_NEAR,
        .target     = 0,
        .timeout_ms = 20000,
    },
    {
        .name       = "Hold 2 seconds",
        .action     = SEQ_ACT_LIFT_HOLD,
        .condition  = SEQ_COND_TIME,
        .time_ms    = 2000,
        .timeout_ms = 5000,
    },
};

const seq_config_t g_seq_home = {
    .name = "home",
    .type = SEQ_TYPE_HOME,
    .steps = g_steps_home,
    .step_count = sizeof(g_steps_home) / sizeof(g_steps_home[0]),
    .default_timeout_ms = 30000,
};

/* ============================================================
 * 按类型查找
 * ============================================================ */
const seq_config_t* seq_config_find(seq_type_t type)
{
    switch (type) {
        case SEQ_TYPE_RIGHT_MIDDLE: return &g_seq_right_middle;
        case SEQ_TYPE_RIGHT_BEHIND: return &g_seq_right_behind;
        case SEQ_TYPE_HOME:         return &g_seq_home;
        default: return NULL;
    }
}
