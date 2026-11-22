/**
 * @file    control_object.c
 * @brief   兼容层实现 —— 仅保留一个 6 参包装函数
 * @note    真正的实现已迁至 obj_base.c（零 RTT）。
 *          本文件存在的唯一目的：让 4 处业务调用点（6 参形式）零改动。
 */

#include "control_object.h"
#include "framework/executor/ctrl_executor.h"
#include "framework/port/osal.h"

/* ============================================================
 * 6 参兼容包装
 *
 * 与旧 control_object_init 的行为差异（PLAN-OBJ-BASE-V1.0 Q-2）：
 *   旧: 内部硬编码 use_thread ? &g_executor_multi : &g_executor_single
 *   新: obj_base_init 不再选执行器；本包装层代为绑定，
 *       以**保持调用点行为完全不变**（避免本批引入行为回归）
 *
 * 说明：Q-2 的"彻底解耦"目标需业务对象提供 obj_ops_t 并在
 *       ops->init 中自行绑定执行器 —— 那是后续批（G-04）。
 *       本批先把框架改成零 RTT，同时用包装层保持行为等价。
 * ============================================================ */
obj_err_t control_object_init(control_object_t *obj,
                              const char *name,
                              ctrl_obj_type_t type,
                              const StateMachineConfig_t *config,
                              void *context,
                              const ctrl_obj_runtime_cfg_t *runtime)
{
    obj_err_t err;

    /* 1. 走新基类（ops 传 NULL：业务对象本批尚未提供虚表） */
    err = obj_base_init(obj, name, (uint32_t)type, config, context, runtime, NULL);
    if (err != OBJ_OK) {
        return err;
    }

    /* 2. 执行器绑定 —— 保持与原实现完全一致的行为 */
    if (obj->runtime_cfg.use_thread) {
        obj->executor = &g_executor_multi;
    } else {
        obj->executor = &g_executor_single;
    }

    return OBJ_OK;
}