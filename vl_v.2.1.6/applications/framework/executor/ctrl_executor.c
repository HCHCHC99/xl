/**
 * @file    ctrl_executor.c
 * @brief   执行器实现（单线程 + 多线程）
 */

#include "ctrl_executor.h"
#include "../core/control_object.h"
#include "framework/state_engine/sm_assert.h"
#include "framework/port/osal.h"
#include <string.h>
#include <stddef.h>

/* ============================================================
 * 【2026-11-23 修复】与 obj_base.c 的 obj_executor_iface_t 布局对齐
 *
 * obj_base.c 为保持零 RTOS 依赖，不能 include 本头文件，只能把 executor
 * 强制解释成 {name, init, emit} 三个指针槽。若执行器结构体在 name 与
 * emit 之间插入/删除字段，obj_base_emit() 会静默调用错误的函数指针。
 * 这里做编译期锁定：emit 必须位于第 3 个指针槽。
 * ============================================================ */
typedef char ctrl_executor_emit_slot_check[
    (offsetof(ctrl_executor_t, emit) == (sizeof(void *) * 2U)) ? 1 : -1
];

/* ============================================================
 * 全局事件队列访问
 * @note  g_app_event_mq 由 applications/ipc_center.c 持有（类型 rt_mq_t）。
 *        为了让本文件与 ctrl_executor.h 在零 RTT 模式下也能编译，
 *        把对它的引用**收敛到这一个函数**，并用条件编译隔离。
 *
 *        OBJ_BASE_NO_RTOS 下返回 NULL（此时业务侧的 ipc_center 不参与编译，
 *        单线程执行器也不会被实际调用，仅用于验证 framework 零 RTT）。
 * ============================================================ */
#ifndef OBJ_BASE_NO_RTOS
#include "ipc_center.h"     /* 提供 g_app_event_mq 的真实类型声明 */

osal_mq_t ctrl_executor_get_event_mq(void)
{
    return (osal_mq_t)g_app_event_mq;
}
#else
osal_mq_t ctrl_executor_get_event_mq(void)
{
    return NULL;
}
#endif

/* ============================================================
 * 单线程执行器
 * ============================================================ */
static obj_err_t single_init(ctrl_executor_t *self, control_object_t *obj)
{
    SM_ASSERT_PTR(self);
    SM_ASSERT_PTR(obj);
    /* 单线程模式无需私有资源 */
    obj->executor_data = NULL;
    return OBJ_OK;
}

static obj_err_t single_emit(ctrl_executor_t *self, control_object_t *obj, app_event_t event)
{
    osal_mq_t mq;

    SM_ASSERT_PTR(self);
    SM_ASSERT_PTR(obj);

    mq = ctrl_executor_get_event_mq();
    SM_ASSERT_PTR(mq);

    /* 投递到全局事件队列 */
    return osal_mq_send(mq, &event, sizeof(app_event_t));
}

static void single_run(ctrl_executor_t *self, control_object_t *obj, app_event_t event)
{
    SM_ASSERT_PTR(self);
    SM_ASSERT_PTR(obj);
    /* 在当前线程（app_main）中同步执行 */
    control_object_process(obj, event);
}

static void single_deinit(ctrl_executor_t *self, control_object_t *obj)
{
    /* 无需释放 */
    (void)self;
    (void)obj;
}

ctrl_executor_t g_executor_single = {
    .name   = "SingleThread",
    .init   = single_init,
    .emit   = single_emit,
    .run    = single_run,
    .deinit = single_deinit,
};

/* ============================================================
 * 多线程执行器
 * ============================================================ */
typedef struct {
    osal_mq_t     mq;
    osal_thread_t thread;
    uint16_t    mq_size;
    uint16_t    stack_size;
    uint8_t     prio;
} multi_executor_data_t;

static obj_err_t multi_init(ctrl_executor_t *self, control_object_t *obj)
{
    multi_executor_data_t *data;

    SM_ASSERT_PTR(self);
    SM_ASSERT_PTR(obj);

    /* 1. 分配私有数据 */
    data = osal_malloc(sizeof(multi_executor_data_t));
    if (data == NULL) {
        return -OBJ_ENOMEM;
    }
    memset(data, 0, sizeof(multi_executor_data_t));

    /* 2. 读取运行时配置 */
    data->mq_size = obj->runtime_cfg.mq_size;
    data->stack_size = obj->runtime_cfg.stack_size;
    data->prio = obj->runtime_cfg.thread_prio;

    SM_ASSERT_MSG(data->mq_size > 0, "mq_size must > 0");
    SM_ASSERT_MSG(data->stack_size > 0, "stack_size must > 0");

    /* 3. 创建私有消息队列 */
    data->mq = osal_mq_create(obj->name, sizeof(app_event_t),
                              data->mq_size, OSAL_IPC_FLAG_FIFO);
    if (data->mq == NULL) {
        osal_free(data);
        return -OBJ_ERROR;
    }

    /* 4. 创建对象线程 */
    data->thread = osal_thread_create(obj->name,
                                      ctrl_obj_thread_entry,
                                      obj,
                                      data->stack_size,
                                      data->prio,
                                      10);
    if (data->thread == NULL) {
        osal_mq_delete(data->mq);
        osal_free(data);
        return -OBJ_ERROR;
    }

    /* 5. 启动线程 */
    osal_thread_startup(data->thread);

    obj->executor_data = data;

    osal_log("[Executor] %s: thread created (prio=%d, mq=%d)\n",
               obj->name, data->prio, data->mq_size);

    return OBJ_OK;
}

static obj_err_t multi_emit(ctrl_executor_t *self, control_object_t *obj, app_event_t event)
{
    multi_executor_data_t *data;

    SM_ASSERT_PTR(self);
    SM_ASSERT_PTR(obj);

    data = (multi_executor_data_t*)obj->executor_data;
    SM_ASSERT_PTR(data);
    SM_ASSERT_PTR(data->mq);

    return osal_mq_send(data->mq, &event, sizeof(app_event_t));
}

static void multi_run(ctrl_executor_t *self, control_object_t *obj, app_event_t event)
{
    SM_ASSERT_PTR(self);
    SM_ASSERT_PTR(obj);
    /* 在对象自己的线程中执行 */
    control_object_process(obj, event);
}

static void multi_deinit(ctrl_executor_t *self, control_object_t *obj)
{
    multi_executor_data_t *data;

    SM_ASSERT_PTR(self);
    SM_ASSERT_PTR(obj);

    data = (multi_executor_data_t*)obj->executor_data;
    if (data == NULL) return;

    if (data->thread != NULL) {
        osal_thread_delete(data->thread);
    }
    if (data->mq != NULL) {
        osal_mq_delete(data->mq);
    }
    osal_free(data);
    obj->executor_data = NULL;
}

ctrl_executor_t g_executor_multi = {
    .name   = "MultiThread",
    .init   = multi_init,
    .emit   = multi_emit,
    .run    = multi_run,
    .deinit = multi_deinit,
};

/* ============================================================
 * 控制对象线程统一入口
 * ============================================================ */
void ctrl_obj_thread_entry(void *param)
{
    control_object_t *obj = (control_object_t*)param;
    multi_executor_data_t *data;
    app_event_t event;

    SM_ASSERT_PTR(obj);

    data = (multi_executor_data_t*)obj->executor_data;
    SM_ASSERT_PTR(data);

    osal_log("[Executor] %s thread running...\n", obj->name);

    while (1) {
        if (osal_mq_recv(data->mq, &event, sizeof(app_event_t), OSAL_WAITING_FOREVER) == OSAL_OK) {
            /* obj->executor 是 void* 不透明句柄，此处强转回执行器接口 */
            ctrl_executor_t *exec = (ctrl_executor_t *)obj->executor;
            if (exec != NULL && exec->run != NULL) {
                exec->run(exec, obj, event);
            }
        }
    }
}
