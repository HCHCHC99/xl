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
        /* 【P7 / B-4】TC-ONE-010：到 0° 后下降阀继续输出 2 s 再失电。
         * 本步正常完成时不关阀，交给下一步「Hold 2 seconds」计时；
         * 那一步结束时才真正关阀（它的 keep_lift 为 0）。 */
        .keep_lift  = 1,
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
 * 一键回位（回转已在零位）—— TC-ONE-011
 * ------------------------------------------------------------
 * 回转已在 0°±2° 时，禁止再"升 +25° → 转零 → 升 +34°"，
 * 否则会为了一个已经在目标位的动作把筒先举起来再放下。
 * 用例要求：仅执行下降，到位后保持 2 s 失电。
 * ============================================================ */
static const seq_step_t g_steps_home_at_zero[] = {
    {
        .name       = "Descend to zero",
        .action     = SEQ_ACT_LIFT_DOWN,
        .condition  = SEQ_COND_LIFT_NEAR,
        .target     = 0,
        .timeout_ms = 20000,
        .keep_lift  = 1,        /* 同 g_steps_home：保持 2 s 靠下一步 */
    },
    {
        .name       = "Hold 2 seconds",
        .action     = SEQ_ACT_LIFT_HOLD,
        .condition  = SEQ_COND_TIME,
        .time_ms    = 2000,
        .timeout_ms = 5000,
    },
};

const seq_config_t g_seq_home_at_zero = {
    .name = "home_at_zero",
    .type = SEQ_TYPE_HOME,
    .steps = g_steps_home_at_zero,
    .step_count = sizeof(g_steps_home_at_zero) / sizeof(g_steps_home_at_zero[0]),
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

/* ============================================================
 * 回位专用：按当前回转角挑配置（TC-ONE-011）
 * ------------------------------------------------------------
 * |angle| <= SEQ_HOME_AT_ZERO_TOL_X10 视为"已在零位"，只做下降 + 保持。
 * 门限与引擎的 ctx->tolerance 默认值（20 = ±2°）保持一致，见
 * seq_engine_init()；这是操作语义上的"到位"，不是控制精度，故不进口参区。
 * TODO(batch4)：若现场需要调整，再随其它阈值一并迁移到 sys_config。
 * ============================================================ */
const seq_config_t* seq_config_find_home(int32_t swing_angle_x10)
{
    if ((swing_angle_x10 >= -SEQ_HOME_AT_ZERO_TOL_X10) &&
        (swing_angle_x10 <=  SEQ_HOME_AT_ZERO_TOL_X10)) {
        return &g_seq_home_at_zero;
    }
    return &g_seq_home;
}
