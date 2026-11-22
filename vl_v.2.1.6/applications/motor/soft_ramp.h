/**
 * @file    soft_ramp.h
 * @brief   软启动/停止斜坡发生器
 */

#ifndef __SOFT_RAMP_H
#define __SOFT_RAMP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 斜坡类型
 * ============================================================ */
typedef enum {
    RAMP_TYPE_LINEAR = 0,       /* 线性 */
    RAMP_TYPE_S_CURVE,          /* S 型（预留） */
} ramp_type_t;

/* ============================================================
 * 软启动/停止配置
 * ============================================================ */
typedef struct soft_ramp_cfg {
    uint8_t     start_enabled;      /* 1=启用软启动 */
    uint8_t     stop_enabled;       /* 1=启用软停止 */
    ramp_type_t type;               /* 斜坡类型 */
    uint32_t    start_time_ms;      /* 启动时间（如 500ms） */
    uint32_t    stop_time_ms;       /* 停止时间（如 500ms） */
} soft_ramp_cfg_t;

/* ============================================================
 * 斜坡运行时状态
 * ============================================================ */
typedef struct soft_ramp {
    const soft_ramp_cfg_t *cfg;
    int32_t  start_value;           /* 起始值 */
    int32_t  target_value;          /* 目标值 */
    int32_t  current_value;         /* 当前值 */
    uint32_t elapsed_ms;            /* 已用时间 */
    uint8_t  is_running;            /* 是否在斜坡中 */
    uint8_t  mode;                  /* 1=启动斜坡, 0=停止斜坡 */
} soft_ramp_t;

/* ============================================================
 * API
 * ============================================================ */
void    soft_ramp_init(soft_ramp_t *ramp, const soft_ramp_cfg_t *cfg);
void    soft_ramp_start(soft_ramp_t *ramp, int32_t from, int32_t to);
void    soft_ramp_stop(soft_ramp_t *ramp, int32_t from);
int32_t soft_ramp_update(soft_ramp_t *ramp, uint32_t dt_ms);
uint8_t soft_ramp_is_done(const soft_ramp_t *ramp);

#ifdef __cplusplus
}
#endif

#endif /* __SOFT_RAMP_H */
