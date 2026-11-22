/**
 * @file    control_object.h
 * @brief   控制对象兼容层 —— 业务侧看到的仍是 control_object_t
 * @note    **本文件现在是 obj_base 的别名层**（PLAN-OBJ-BASE-V1.0 批次 A）。
 *
 *          设计意图：
 *          1. 业务侧 4 个对象（swing/lift/mode/seq）**一行都不用改**
 *             —— 实测四者的 offsetof(base) 全为 0，别名替换即可
 *          2. 真正的实现已迁至 framework/core/obj_base.{h,c}（零 RTT）
 *          3. 4 处 control_object_init() 调用点通过宏映射到 obj_base_init()
 *
 *          ⚠️ 过渡说明：
 *          - 本文件仍 include <rtthread.h>（业务侧还需要 RT_EOK / rt_kprintf）
 *          - 定义 OBJ_BASE_NO_RTOS 可关闭该 include，用于验证 framework 零 RTT
 *          - RT_EOK 的业务层全量替换属 LEG-1，本批不做
 */

#ifndef __CONTROL_OBJECT_H
#define __CONTROL_OBJECT_H

#include <stdint.h>

#include "obj_base.h"
#include "framework/state_engine/state_engine.h"

/* ============================================================
 * 过渡 include：业务侧依赖 RT_EOK / rt_kprintf
 * 定义 OBJ_BASE_NO_RTOS 时不引入，用于零 RTT 自检
 * ============================================================ */
#ifndef OBJ_BASE_NO_RTOS
    #include <rtthread.h>
#else
    /* 零 RTT 模式下补齐业务侧仍会用到的符号 */
    #ifndef RT_EOK
        #define RT_EOK   OBJ_OK
    #endif
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 类型别名 —— 业务侧零改动
 * ============================================================ */
typedef obj_base_t            control_object_t;
typedef obj_ops_t             ctrl_obj_ops_t;
typedef obj_err_t             ctrl_obj_err_t;
typedef obj_runtime_cfg_t     ctrl_obj_runtime_cfg_t;

/* ============================================================
 * 控制对象类型枚举
 * @note  2026-11-15 C1 批上移到 obj_base.h（零 RTOS 层），本文件经
 *        include obj_base.h 继续向业务侧暴露，**枚举值与类型名均未变**。
 *        目的：framework/core/obj_desc.c 填描述表时可引用 type 而不引入 RTT。
 * ============================================================ */

/* 前置声明（执行器由 ctrl_executor.h 完整定义） */
typedef struct ctrl_executor ctrl_executor_t;

/* ============================================================
 * API 宏映射 —— 4 处 init 调用 + 若干 get/set 零改动
 *
 * 注意 obj_base_init 的签名与 control_object_init 不同：
 *   旧: (obj, name, type, config, context, runtime)              6 参
 *   新: (obj, name, type, config, context, runtime, ops)         7 参
 *
 * 为保持业务侧零改动，提供一个 6 参的原名兼容函数
 * （实现见 control_object.c，内部转调 obj_base_init 并传 ops = NULL）
 * ============================================================ */
obj_err_t control_object_init(control_object_t *obj,
                              const char *name,
                              ctrl_obj_type_t type,
                              const StateMachineConfig_t *config,
                              void *context,
                              const ctrl_obj_runtime_cfg_t *runtime);

/* 其余 API 直接映射（签名完全一致，无需包装） */
#define control_object_process      obj_base_process
#define control_object_reset        obj_base_reset
#define control_object_get_state    obj_base_get_state
#define control_object_get_context  obj_base_get_context
#define control_object_set_mode     obj_base_set_mode
#define control_object_get_mode     obj_base_get_mode
#define control_object_emit         obj_base_emit

#ifdef __cplusplus
}
#endif

#endif /* __CONTROL_OBJECT_H */