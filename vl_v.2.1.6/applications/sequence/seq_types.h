/**
 * @file    seq_types.h
 * @brief   序列引擎类型定义
 * @note    定义步骤动作、完成条件、配置表数据结构
 */

#ifndef __SEQ_TYPES_H
#define __SEQ_TYPES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 序列类型枚举
 * ============================================================ */
typedef enum {
    SEQ_TYPE_NONE = 0,
    SEQ_TYPE_RIGHT_MIDDLE,      /* 一键右中 */
    SEQ_TYPE_RIGHT_BEHIND,      /* 一键右后 */
    SEQ_TYPE_HOME,              /* 一键回位 */
    SEQ_TYPE_MAX
} seq_type_t;

/* ============================================================
 * 步骤动作类型
 * ============================================================ */
typedef enum {
    SEQ_ACT_NONE = 0,

    /* 举升相关动作 */
    SEQ_ACT_LIFT_UP,            /* 举升上升（持续输出） */
    SEQ_ACT_LIFT_DOWN,          /* 举升下降（持续输出） */
    SEQ_ACT_LIFT_HOLD,          /* 举升保持（保持当前输出） */
    SEQ_ACT_LIFT_STOP,          /* 举升停止（关闭输出） */

    /* 回转相关动作 */
    SEQ_ACT_SWING_LEFT,          /* 回转左转（持续） */
    SEQ_ACT_SWING_RIGHT,         /* 回转右转（持续） */
    SEQ_ACT_SWING_MOVE_TO,       /* 回转移动到目标角度 */
    SEQ_ACT_SWING_STOP,          /* 回转停止 */

    /* 通用动作 */
    SEQ_ACT_DELAY,              /* 纯延时 */
    SEQ_ACT_CUSTOM,             /* 自定义动作（回调） */

    SEQ_ACT_MAX
} seq_action_t;

/* ============================================================
 * 步骤完成条件类型
 * ============================================================ */
typedef enum {
    SEQ_COND_NONE = 0,          /* 无条件（立即完成，用于延时外的动作）*/
    SEQ_COND_LIFT_GE,           /* 举升角度 ≥ target（0.1°） */
    SEQ_COND_LIFT_LE,           /* 举升角度 ≤ target */
    SEQ_COND_LIFT_NEAR,         /* 举升角度接近 target（±tolerance）*/
    SEQ_COND_SWING_NEAR,         /* 回转角度接近 target（±tolerance）*/
    SEQ_COND_TIME,              /* 时间条件（time_ms 到） */
    SEQ_COND_CUSTOM,            /* 自定义条件（回调） */

    SEQ_COND_MAX
} seq_cond_t;

/* ============================================================
 * 序列步骤配置
 * ============================================================ */
typedef struct {
    const char  *name;          /* 步骤名称（调试用） */
    seq_action_t action;        /* 执行动作 */
    int32_t      target;        /* 目标值（角度，0.1°） */
    uint32_t     time_ms;       /* 时间参数（延时/保持时间） */
    seq_cond_t   condition;     /* 完成条件 */
    uint32_t     timeout_ms;    /* 该步骤超时（0=用序列默认） */

    /* 自定义动作/条件（仅当 action/condition 为 CUSTOM 时使用） */
    int  (*custom_action)(void *ctx);
    int  (*custom_cond)(void *ctx);
} seq_step_t;

/* ============================================================
 * 序列配置表
 * ============================================================ */
typedef struct {
    const char       *name;             /* 序列名称 */
    seq_type_t        type;             /* 序列类型 */
    const seq_step_t *steps;            /* 步骤数组 */
    uint32_t          step_count;       /* 步骤数量 */
    uint32_t          default_timeout_ms;/* 默认超时（单步） */
} seq_config_t;

#ifdef __cplusplus
}
#endif

#endif /* __SEQ_TYPES_H */
