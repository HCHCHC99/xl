/**
 * @file    ctrl_executor.h
 * @brief   控制对象执行器接口
 * @note    执行器承载"单/多线程"策略
 *          控制对象不感知线程，只通过 executor 投递和运行
 */

#ifndef __CTRL_EXECUTOR_H
#define __CTRL_EXECUTOR_H

#include <stdint.h>
#include "framework/core/control_object.h"      /* 提供 control_object_t（= obj_base_t）*/
#include "framework/state_engine/state_engine.h"/* 提供 Event_t */
#include "app_event.h"
#include "framework/port/osal.h"

/* 过渡：执行器内部仍需 rt_err_t / rt_mq_t（本批 LEG 项）
 * 定义 OBJ_BASE_NO_RTOS 时不引入，用于零 RTT 自检 */
#ifndef OBJ_BASE_NO_RTOS
    #include <rtthread.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 前置声明
 * @note  control_object_t 由 control_object.h 提供（obj_base_t 别名），
 *        此处**不再重复 typedef**，否则会与别名冲突。
 * ============================================================ */
typedef struct ctrl_executor ctrl_executor_t;

/* ============================================================
 * 执行器接口
 * ============================================================ */
struct ctrl_executor {
    const char *name;

    /**
     * @brief 初始化执行器（为对象分配资源）
     * @param self 执行器
     * @param obj  控制对象
     * @return RT_EOK 成功
     */
    obj_err_t (*init)(ctrl_executor_t *self, control_object_t *obj);

    /**
     * @brief 投递事件到对象
     * @param self  执行器
     * @param obj   目标对象
     * @param event 事件
     * @return RT_EOK 成功
     */
    obj_err_t (*emit)(ctrl_executor_t *self, control_object_t *obj, app_event_t event);

    /**
     * @brief 执行对象的事件处理（在正确线程上下文中调用）
     * @param self  执行器
     * @param obj   目标对象
     * @param event 事件
     */
    void (*run)(ctrl_executor_t *self, control_object_t *obj, app_event_t event);

    /**
     * @brief 销毁执行器（释放资源）
     */
    void (*deinit)(ctrl_executor_t *self, control_object_t *obj);
};

/* ============================================================
 * 内置执行器实例
 * ============================================================ */
extern ctrl_executor_t g_executor_single;   /* 单线程执行器 */
extern ctrl_executor_t g_executor_multi;    /* 多线程执行器 */

/* ============================================================
 * 全局事件队列（单线程模式使用）
 * @note  g_app_event_mq 实际由 applications/ipc_center.c 持有，
 *        其声明在 ipc_center.h 中是 rt_mq_t。
 *
 *        这里**不重复声明**该变量（否则零 RTT 模式下 rt_mq_t 不存在会报错），
 *        而是提供一个内部分隔：需要用到它的 .c 通过
 *        ctrl_executor_get_event_mq() 取得，本头文件保持零 RTT 可编译。
 * ============================================================ */
osal_mq_t ctrl_executor_get_event_mq(void);

/* ============================================================
 * 多线程执行器的对象线程入口（框架内部使用）
 * ============================================================ */
void ctrl_obj_thread_entry(void *param);

#ifdef __cplusplus
}
#endif

#endif /* __CTRL_EXECUTOR_H */
