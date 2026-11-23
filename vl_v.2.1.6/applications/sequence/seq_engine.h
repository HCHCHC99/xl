/**
 * @file    seq_engine.h
 * @brief   通用序列执行引擎
 * @note    解释执行配置表定义的序列
 */

#ifndef __SEQ_ENGINE_H
#define __SEQ_ENGINE_H

#include <stdint.h>
#include <rtthread.h>
#include "seq_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 前置声明
 * ============================================================ */
typedef struct seq_context seq_context_t;

/* ============================================================
 * 序列运行状态
 * ============================================================ */
typedef enum {
    SEQ_STATE_IDLE = 0,         /* 空闲 */
    SEQ_STATE_RUNNING,          /* 执行中 */
    SEQ_STATE_HOLDING,          /* 保持中 */
    SEQ_STATE_DONE,             /* 完成 */
    SEQ_STATE_ABORTED,          /* 中止 */
} seq_state_t;

/* ============================================================
 * 序列上下文
 * ============================================================ */
struct seq_context {
    /* ----- 当前配置与状态 ----- */
    const seq_config_t *config;         /* 当前序列配置 */
    seq_type_t          type;           /* 序列类型 */
    seq_state_t         state;          /* 运行状态 */
    uint32_t            current_step;   /* 当前步骤索引 */
    uint32_t            step_start_tick;/* 步骤开始时间 */
    uint8_t             step_entered;   /* 步骤是否已执行 Enter 动作 */

    /* ----- 目标参数 ----- */
    int32_t             target_h_angle; /* 回转目标角度（0.1°） */
    int32_t             target_v_angle; /* 举升目标角度（0.1°） */
    int32_t             tolerance;      /* 到达容差（0.1°） */

    /* ----- 统计 ----- */
    uint32_t            complete_count; /* 完成次数 */
    uint32_t            abort_count;    /* 中止次数 */

    /* ----- 外部引用（由上层设置） ----- */
    void               *swing_ctrl;      /* 回转控制对象指针 */
    void               *vert_ctrl;      /* 举升控制对象指针 */
    void               *swing_angle;     /* 回转角度对象指针 */
    void               *lift_angle;    /* 举升角度对象指针 */

    /* ----- 步骤进入/退出回调（可选） ----- */
    void (*on_step_enter)(seq_context_t *ctx, uint32_t step_idx);
    void (*on_step_exit)(seq_context_t *ctx, uint32_t step_idx);
};

/* ============================================================
 * 引擎 API
 * ============================================================ */

/**
 * @brief 初始化序列引擎
 * @param ctx    序列上下文
 * @param config 序列配置
 * @param type   序列类型
 * @param swing_ctrl 回转控制对象
 * @param vert_ctrl 举升控制对象
 * @param swing_angle 回转角度对象
 * @param lift_angle 举升角度对象
 */
void seq_engine_init(seq_context_t *ctx,
                     const seq_config_t *config,
                     seq_type_t type,
                     void *swing_ctrl,
                     void *vert_ctrl,
                     void *swing_angle,
                     void *lift_angle);

/**
 * @brief 启动序列
 * @return 0 成功
 */
int seq_engine_start(seq_context_t *ctx);

/**
 * @brief 中止序列
 */
void seq_engine_abort(seq_context_t *ctx);

/**
 * @brief 周期更新（由 50ms Tick 调用）
 */
void seq_engine_update(seq_context_t *ctx);

/**
 * @brief 检查是否处于活动状态
 */
uint8_t seq_engine_is_active(const seq_context_t *ctx);

/**
 * @brief 获取当前状态
 */
seq_state_t seq_engine_get_state(const seq_context_t *ctx);

/**
 * @brief 获取当前步骤索引
 */
uint32_t seq_engine_get_step(const seq_context_t *ctx);

/**
 * @brief 台架条件强制开关（P2）
 * @param en 1 = 角度类条件（LIFT_GE/LE/NEAR、SWING_NEAR）直接判满足
 * @note  仅供 `t_seq_auto` 使用；不影响 TIME/NONE/CUSTOM；纯 RAM，不落盘。
 */
void    seq_engine_set_force_cond(uint8_t en);
uint8_t seq_engine_get_force_cond(void);

#ifdef __cplusplus
}
#endif

#endif /* __SEQ_ENGINE_H */
