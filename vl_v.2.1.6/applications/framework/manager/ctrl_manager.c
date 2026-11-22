/**
 * @file    ctrl_manager.c
 * @brief   控制对象管理器实现
 */

#include "ctrl_manager.h"
#include "../executor/ctrl_executor.h"
#include "framework/state_engine/sm_assert.h"
#include "framework/port/osal.h"
#include <string.h>


/* ============================================================
 * 全局管理器实例
 * ============================================================ */
ctrl_manager_t g_ctrl_mgr;


/* ============================================================
 * 生命周期
 * ============================================================ */
ctrl_mgr_err_t ctrl_manager_init(ctrl_manager_t *mgr)
{
    SM_ASSERT_PTR(mgr);

    memset(mgr, 0, sizeof(ctrl_manager_t));
    mgr->initialized = 1;

    osal_log("[CtrlMgr] Manager initialized (max=%d)\n\n", CTRL_MGR_MAX_OBJECTS);
    return OBJ_OK;
}

ctrl_mgr_err_t ctrl_manager_start(ctrl_manager_t *mgr)
{
    SM_ASSERT_PTR(mgr);
    SM_ASSERT_MSG(mgr->initialized, "manager not initialized");

    for (uint8_t i = 0; i < mgr->count; i++) {
        control_object_t *obj = mgr->objects[i];

        if (obj->executor && ((ctrl_executor_t *)obj->executor)->init) {
            obj_err_t ret = ((ctrl_executor_t *)obj->executor)->init(obj->executor, obj);
            if (ret != OBJ_OK) {
                osal_log("[CtrlMgr] %s executor init failed!\n", obj->name);
                return ret;
            }
        }
    }

    mgr->running = 1;
    osal_log("[CtrlMgr] All %d objects started.\n\n", mgr->count);
    return OBJ_OK;
}

void ctrl_manager_stop(ctrl_manager_t *mgr)
{
    SM_ASSERT_PTR(mgr);

    for (uint8_t i = 0; i < mgr->count; i++) {
        control_object_t *obj = mgr->objects[i];

        if (obj->executor && ((ctrl_executor_t *)obj->executor)->deinit) {
            ((ctrl_executor_t *)obj->executor)->deinit(obj->executor, obj);
        }
    }

    mgr->running = 0;
}

/* ============================================================
 * 对象管理
 * ============================================================ */
ctrl_mgr_err_t ctrl_manager_register(ctrl_manager_t *mgr, control_object_t *obj)
{
    SM_ASSERT_PTR(mgr);
    SM_ASSERT_PTR(obj);

    if (mgr->count >= CTRL_MGR_MAX_OBJECTS) {
        osal_log("[CtrlMgr] register failed: full\n");
        return -OBJ_EFULL;
    }

    /* 检查重复注册 */
    for (uint8_t i = 0; i < mgr->count; i++) {
        if (mgr->objects[i] == obj) {
            return -CTRL_ERR_EXIST;
        }
    }

    mgr->objects[mgr->count++] = obj;

    osal_log("[CtrlMgr] Registered: %s (type=%d)\n", obj->name, obj->type);
    return OBJ_OK;
}

ctrl_mgr_err_t ctrl_manager_unregister(ctrl_manager_t *mgr, control_object_t *obj)
{
    SM_ASSERT_PTR(mgr);
    SM_ASSERT_PTR(obj);

    for (uint8_t i = 0; i < mgr->count; i++) {
        if (mgr->objects[i] == obj) {
            /* 用最后一个对象填补空缺 */
            mgr->objects[i] = mgr->objects[mgr->count - 1];
            mgr->objects[mgr->count - 1] = NULL;
            mgr->count--;
            return OBJ_OK;
        }
    }
    return -CTRL_ERR_NOT_FOUND;
}

control_object_t* ctrl_manager_find_by_name(ctrl_manager_t *mgr, const char *name)
{
    SM_ASSERT_PTR(mgr);
    SM_ASSERT_PTR(name);

    for (uint8_t i = 0; i < mgr->count; i++) {
        if (strcmp(mgr->objects[i]->name, name) == 0) {
            return mgr->objects[i];
        }
    }
    return NULL;
}

control_object_t* ctrl_manager_find_by_type(ctrl_manager_t *mgr, ctrl_obj_type_t type)
{
    SM_ASSERT_PTR(mgr);

    for (uint8_t i = 0; i < mgr->count; i++) {
        if (mgr->objects[i]->type == type) {
            return mgr->objects[i];
        }
    }
    return NULL;
}

/* ============================================================
 * 事件分发
 * ============================================================ */
ctrl_mgr_err_t ctrl_manager_broadcast(ctrl_manager_t *mgr, app_event_t event)
{
    SM_ASSERT_PTR(mgr);

    for (uint8_t i = 0; i < mgr->count; i++) {
        control_object_t *obj = mgr->objects[i];

        if (obj->executor && ((ctrl_executor_t *)obj->executor)->run) {
            ((ctrl_executor_t *)obj->executor)->run(obj->executor, obj, event);
        }
    }
    return OBJ_OK;
}

ctrl_mgr_err_t ctrl_manager_emit_to(ctrl_manager_t *mgr,
                               ctrl_obj_type_t type,
                               app_event_t event)
{
    control_object_t *obj;

    SM_ASSERT_PTR(mgr);

    obj = ctrl_manager_find_by_type(mgr, type);
    if (obj == NULL) {
        return -CTRL_ERR_NOT_FOUND;
    }

    return control_object_emit(obj, event);
}

/* ============================================================
 * 模式管理
 * ============================================================ */
void ctrl_manager_set_mode(ctrl_manager_t *mgr, uint8_t mode)
{
    SM_ASSERT_PTR(mgr);

    for (uint8_t i = 0; i < mgr->count; i++) {
        control_object_set_mode(mgr->objects[i], mode);
    }

    osal_log("[CtrlMgr] All objects switched to mode %d\n", mode);
}

uint8_t ctrl_manager_get_mode(ctrl_manager_t *mgr, ctrl_obj_type_t type)
{
    control_object_t *obj = ctrl_manager_find_by_type(mgr, type);
    if (obj == NULL) return 0xFF;
    return control_object_get_mode(obj);
}

/* ============================================================
 * 调试
 * ============================================================ */
void ctrl_manager_dump(ctrl_manager_t *mgr)
{
    SM_ASSERT_PTR(mgr);

    osal_log("\n========== Control Manager Dump ==========\n");
    osal_log("count = %d, running = %d\n", mgr->count, mgr->running);
    osal_log("------------------------------------------\n");

    for (uint8_t i = 0; i < mgr->count; i++) {
        control_object_t *obj = mgr->objects[i];

        osal_log("[%2d] %-16s type=%-2d state=%-2d mode=%-2d thread=%d\n",
                   i, obj->name, obj->type,
                   control_object_get_state(obj),
                   obj->current_mode,
                   obj->runtime_cfg.use_thread);
    }

    osal_log("==========================================\n\n");
}

/* ============================================================
 * MSH 命令导出 —— 属 RTOS 交互层，零 RTT 模式下自动关闭
 * @note  这是框架内最后一处 RTT 直接依赖（LEG 项）。
 *        定义 OBJ_BASE_NO_RTOS 时不编译，用于零 RTT 自检。
 * ============================================================ */
#ifndef OBJ_BASE_NO_RTOS
#ifdef RT_USING_FINSH
#include <finsh.h>
MSH_CMD_EXPORT_ALIAS(ctrl_manager_dump, ctrl_dump, dump all control objects);
#endif
#endif /* OBJ_BASE_NO_RTOS */
