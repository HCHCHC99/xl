/**
 * @file    osal.h
 * @brief   OS 抽象层（OS Abstraction Layer）—— 框架内唯一允许接触 RTOS 的边界
 * @note    本文件**只声明，不 include 任何 RTOS 头文件**。
 *          任何想移植到 FreeRTOS / 裸机的动作，只需重写 osal_xxx.c 一个文件。
 *
 *          设计原则（见方案 PLAN-OBJ-BASE-V1.0 §3.3）：
 *          只封装当前**实际用到**的 6 类能力，不做全能抽象。
 *          当前 4 个业务对象全部 use_thread=0，因此 mutex/sem/event 一律不封。
 */

#ifndef __OSAL_H
#define __OSAL_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 不透明句柄（避免暴露 RTOS 类型）
 * ============================================================ */
typedef void *osal_mq_t;
typedef void *osal_thread_t;

/* ============================================================
 * 返回码（与 obj_err_t 语义一致，保持 0 = 成功）
 * ============================================================ */
#define OSAL_OK         0
#define OSAL_ERROR      (-1)
#define OSAL_EINVAL     (-2)
#define OSAL_ENOMEM     (-3)
#define OSAL_ETIMEOUT   (-4)

/* 永久等待 */
#define OSAL_WAITING_FOREVER    (0xFFFFFFFFu)

/* 空指针（不依赖 stdlib 的 NULL 宏命名习惯） */
#define OSAL_NULL_PTR           ((void *)0)

/* IPC 等待策略（与 RT-Thread 语义对齐，值为 0/1） */
#define OSAL_IPC_FLAG_FIFO      (0x00u)
#define OSAL_IPC_FLAG_PRIO      (0x01u)

/* ============================================================
 * 1) 消息队列
 * ============================================================ */
osal_mq_t osal_mq_create(const char *name, uint32_t msg_size,
                         uint32_t max_msgs, uint32_t flag);
int32_t   osal_mq_delete(osal_mq_t mq);
int32_t   osal_mq_send(osal_mq_t mq, const void *buf, uint32_t size);
int32_t   osal_mq_recv(osal_mq_t mq, void *buf, uint32_t size, uint32_t timeout);

/* ============================================================
 * 2) 线程
 * ============================================================ */
osal_thread_t osal_thread_create(const char *name,
                                 void (*entry)(void *param),
                                 void *param,
                                 uint32_t stack_size,
                                 uint8_t  priority,
                                 uint32_t tick);
int32_t       osal_thread_startup(osal_thread_t thread);
int32_t       osal_thread_delete(osal_thread_t thread);

/* ============================================================
 * 3) 动态内存
 * ============================================================ */
void *osal_malloc(size_t size);
void  osal_free(void *ptr);

/* ============================================================
 * 4) 时间
 * ============================================================ */
uint32_t osal_tick_get(void);

/* ============================================================
 * 5) 日志（可空实现 —— 生产版可整体编译掉）
 * ============================================================ */
void osal_log(const char *fmt, ...);

/* ============================================================
 * 6) 断言（可空实现）
 * ============================================================ */
void osal_assert_fail(const char *expr, const char *file, int line);

#ifdef __cplusplus
}
#endif

#endif /* __OSAL_H */