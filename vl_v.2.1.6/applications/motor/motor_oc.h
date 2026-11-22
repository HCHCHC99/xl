/**
 * @file    motor_oc.h
 * @brief   电机过流保护（安全层）
 * @note    三级过流保护：警告 / 降功率 / 紧急停机
 *          三级过流在驱动层响应，一级/二级在控制层响应
 */

#ifndef __MOTOR_OC_H
#define __MOTOR_OC_H

#include <stdint.h>
#include <rtthread.h>
#include "motor_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 过流保护配置
 * ============================================================ */
typedef struct {
    uint16_t rated_current_a10;

    uint16_t oc1_current_a10;
    uint16_t oc1_debounce_ms;

    uint16_t oc2_current_a10;
    uint16_t oc2_debounce_ms;
    uint8_t  oc2_deboost_pct;

    uint16_t oc3_current_a10;
    uint16_t oc3_debounce_ms;

    uint8_t  auto_recover;
} motor_oc_cfg_t;

/* ============================================================
 * 过流保护上下文
 * ============================================================ */
typedef struct {
    /* ----- 配置引用 ----- */
    const motor_oc_cfg_t *cfg;

    /* ----- 状态 ----- */
    motor_oc_state_t  state;            /* 当前过流级别 */
    uint32_t          state_enter_tick; /* 进入当前状态的时间 */
    uint32_t          trigger_count;    /* 累计触发次数 */

    /* ----- 一级去抖计时 ----- */
    uint32_t          oc1_start_tick;
    uint8_t           oc1_running;

    /* ----- 二级去抖计时 ----- */
    uint32_t          oc2_start_tick;
    uint8_t           oc2_running;

    /* ----- 三级去抖计时 ----- */
    uint32_t          oc3_start_tick;
    uint8_t           oc3_running;

    /* ----- 当前有效电流上限（供工作模式层使用） ----- */
    uint16_t          active_current_limit_a10;
} motor_oc_ctx_t;


/* ============================================================
 * API
 * ============================================================ */

/**
 * @brief 初始化过流保护上下文
 */
void motor_oc_init(motor_oc_ctx_t *ctx, const motor_oc_cfg_t *cfg);

/**
 * @brief 周期更新（由 50ms Tick 调用）
 * @param ctx 过流上下文
 * @param current_a10 当前电流（0.1A）
 * @return 当前过流级别
 */
motor_oc_state_t motor_oc_update(motor_oc_ctx_t *ctx, uint16_t current_a10);

/**
 * @brief 获取当前有效电流上限（供工作模式层限幅）
 */
uint16_t motor_oc_get_limit(const motor_oc_ctx_t *ctx);

/**
 * @brief 手动复位（清除三级过流状态）
 */
rt_err_t motor_oc_reset(motor_oc_ctx_t *ctx);

/**
 * @brief 获取当前状态
 */
motor_oc_state_t motor_oc_get_state(const motor_oc_ctx_t *ctx);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_OC_H */
