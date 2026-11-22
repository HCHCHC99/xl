/**
 * @file    mode_control.h
 * @brief   模式控制对象 —— 管理工厂/测试/工作模式的切换
 */

#ifndef __MODE_CONTROL_H
#define __MODE_CONTROL_H

#include <stdint.h>
#include "../framework/core/control_object.h"
#include "../framework/manager/ctrl_manager.h"
#include "mode_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 模式状态枚举
 * ============================================================ */
typedef enum {
    MODE_STATE_INIT = 0,        /* 初始化中 */
    MODE_STATE_FACTORY,         /* 工厂模式 */
    MODE_STATE_TEST,            /* 测试模式 */
    MODE_STATE_WORK,            /* 工作模式 */
    MODE_STATE_MAX
} mode_state_t;

/* ============================================================
 * 模式控制上下文
 * ============================================================ */
typedef struct {
    mode_state_t  current_mode;         /* 当前模式 */
    uint32_t      mode_enter_tick;      /* 进入模式的时间戳 */
    mode_config_t config;               /* 配置 */

    /* 统计（调试用） */
    uint32_t      factory_enter_count;
    uint32_t      test_enter_count;
    uint32_t      work_enter_count;
} mode_context_t;

/* ============================================================
 * 模式控制对象
 * ============================================================ */
typedef struct {
    control_object_t base;              /* 基类（必须首位） */
    StateMachine_t   sm_instance;       /* 状态机实例（静态分配） */
    mode_context_t   ctx;               /* 业务上下文 */
} mode_control_t;

/* ============================================================
 * 外部引用
 * ============================================================ */
extern mode_control_t g_mode_obj;

/* ============================================================
 * API
 * ============================================================ */

/**
 * @brief 初始化并注册模式控制对象
 * @param mgr 控制对象管理器
 * @return 0 成功
 */
int mode_control_register(ctrl_manager_t *mgr);

/**
 * @brief 获取模式控制上下文
 */
mode_context_t* mode_control_get_context(void);

/**
 * @brief 设置模式（调试接口）
 */
void mode_control_set_mode(sys_mode_t mode);

#ifdef __cplusplus
}
#endif

#endif /* __MODE_CONTROL_H */
