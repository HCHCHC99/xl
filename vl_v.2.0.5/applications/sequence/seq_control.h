/**
 * @file    seq_control.h
 * @brief   序列控制对象
 */

#ifndef __SEQ_CONTROL_H
#define __SEQ_CONTROL_H

#include <stdint.h>
#include "../framework/core/control_object.h"
#include "../framework/manager/ctrl_manager.h"
#include "seq_engine.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 序列控制上下文
 * ============================================================ */
typedef struct {
    seq_context_t    engine;            /* 序列引擎上下文 */
    uint32_t         request_count;     /* 请求计数 */
    uint32_t         reject_count;      /* 拒绝计数（前置条件不满足） */

    /* 一键按键上升沿检测 */
    uint8_t          prev_middle;
    uint8_t          prev_rear;
    uint8_t          prev_home;

    /* 启车状态 */
    uint8_t          vehicle_started;
} seq_ctrl_context_t;

/* ============================================================
 * 序列控制对象
 * ============================================================ */
typedef struct {
    control_object_t base;
    StateMachine_t   sm_instance;
    seq_ctrl_context_t ctx;
} seq_control_t;

/* ============================================================
 * 外部引用
 * ============================================================ */
extern seq_control_t g_seq_obj;

/* ============================================================
 * API
 * ============================================================ */

int  seq_control_register(ctrl_manager_t *mgr);

void seq_control_update_onekey(uint8_t middle, uint8_t rear, uint8_t home);

void seq_control_set_vehicle_started(uint8_t started);

void seq_control_update(void);

seq_ctrl_context_t* seq_control_get_context(void);

#ifdef __cplusplus
}
#endif

#endif /* __SEQ_CONTROL_H */
