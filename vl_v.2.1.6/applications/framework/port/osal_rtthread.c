/**
 * @file    osal_rtthread.c
 * @brief   OSAL 的 RT-Thread 后端 —— **全工程唯一允许出现 rt_* 的地方**
 * @note    移植到其他 RTOS 时，只需重写本文件，其余框架代码零改动。
 *          验证方式：定义 OBJ_BASE_NO_RTOS 后编译 framework/，应 0 error。
 */

#include "osal.h"

#ifndef OBJ_BASE_NO_RTOS

#include <rtthread.h>
#include <stdarg.h>

/* ============================================================
 * 1) 消息队列
 * ============================================================ */
osal_mq_t osal_mq_create(const char *name, uint32_t msg_size,
                         uint32_t max_msgs, uint32_t flag)
{
    return (osal_mq_t)rt_mq_create(name, msg_size, max_msgs, (rt_uint8_t)flag);
}

int32_t osal_mq_delete(osal_mq_t mq)
{
    if (mq == OSAL_NULL_PTR) return OSAL_EINVAL;
    return (int32_t)rt_mq_delete((rt_mq_t)mq);
}

int32_t osal_mq_send(osal_mq_t mq, const void *buf, uint32_t size)
{
    if (mq == OSAL_NULL_PTR || buf == OSAL_NULL_PTR) return OSAL_EINVAL;
    return (int32_t)rt_mq_send((rt_mq_t)mq, buf, size);
}

int32_t osal_mq_recv(osal_mq_t mq, void *buf, uint32_t size, uint32_t timeout)
{
    if (mq == OSAL_NULL_PTR || buf == OSAL_NULL_PTR) return OSAL_EINVAL;
    return (int32_t)rt_mq_recv((rt_mq_t)mq, buf, size,
                               (timeout == OSAL_WAITING_FOREVER)
                                   ? RT_WAITING_FOREVER
                                   : (rt_int32_t)timeout);
}

/* ============================================================
 * 2) 线程
 * ============================================================ */
osal_thread_t osal_thread_create(const char *name,
                                 void (*entry)(void *param),
                                 void *param,
                                 uint32_t stack_size,
                                 uint8_t  priority,
                                 uint32_t tick)
{
    return (osal_thread_t)rt_thread_create(name, entry, param,
                                           stack_size, priority, tick);
}

int32_t osal_thread_startup(osal_thread_t thread)
{
    if (thread == OSAL_NULL_PTR) return OSAL_EINVAL;
    return (int32_t)rt_thread_startup((rt_thread_t)thread);
}

int32_t osal_thread_delete(osal_thread_t thread)
{
    if (thread == OSAL_NULL_PTR) return OSAL_EINVAL;
    return (int32_t)rt_thread_delete((rt_thread_t)thread);
}

/* ============================================================
 * 3) 动态内存
 * ============================================================ */
void *osal_malloc(size_t size)
{
    return rt_malloc(size);
}

void osal_free(void *ptr)
{
    if (ptr != OSAL_NULL_PTR) rt_free(ptr);
}

/* ============================================================
 * 4) 时间
 * ============================================================ */
uint32_t osal_tick_get(void)
{
    return (uint32_t)rt_tick_get();
}

/* ============================================================
 * 5) 日志
 * ============================================================ */
void osal_log(const char *fmt, ...)
{
    va_list args;
    char buf[128];

    if (fmt == OSAL_NULL_PTR) return;

    va_start(args, fmt);
    rt_vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    rt_kprintf("%s", buf);
}

/* ============================================================
 * 6) 断言
 * ============================================================ */
void osal_assert_fail(const char *expr, const char *file, int line)
{
    rt_kprintf("[OSAL-ASSERT] %s:%d  (%s)\n", file, line, expr ? expr : "?");
#if (RT_DEBUGING_ASSERT) || defined(RT_DEBUG)
    while (1) { }
#endif
}

#endif /* OBJ_BASE_NO_RTOS */