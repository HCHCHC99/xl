/**
 *******************************************************************************
 * @file    tick_50ms.c
 * @brief   50ms 系统周期 Tick 源实现
 * @note    维护前请阅读 tick_50ms.h 顶部的设计说明
 *
 * ============================================================
 * 【实现要点】
 * ------------------------------------------------------------
 * 1) 使用 RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER
 *    周期模式：由系统 tick 自动重载，无需在回调中重启
 *    软定时器：回调运行在 timer 线程，不在中断上下文
 *
 * 2) 回调内直接 rt_mq_send，而未调用 post_event()
 *    原因：post_event() 丢弃了 rt_mq_send 的返回值，无法观测队列满；
 *          这里保留返回值并累加 drop_count，使「事件丢失」可观测。
 *
 * 3) 回调必须极短
 *    timer 线程优先级为 0（最高），回调耗时直接阻塞全系统。
 *    因此此处只做「计数 + 一次 mq_send」，禁止打印、禁浮点、
 *    禁止任何可能阻塞的调用。
 *
 * 4) 事件队列容量为 32（ipc_center.c:23）
 *    50ms 一条 → 队列最多缓冲 1.6s。若 app_main 停止消费，
 *    队列满后 tick 事件被丢弃，drop_count 递增。
 * ============================================================
 *
 * @version V1.0
 * @date    2026-10-21
 *******************************************************************************
 */

/* ============================================================
 * 头文件
 * ============================================================ */
#include <rtthread.h>
#include <finsh.h>

#include "tick_50ms.h"
#include "app_event.h"
#include "ipc_center.h"
#include "framework/state_engine/sm_assert.h"

/* ============================================================
 * 模块状态
 * ============================================================ */

/** @brief 周期定时器句柄 */
static rt_timer_t        s_tick_timer = RT_NULL;

/** @brief 已产生的 tick 计数（诊断用） */
static volatile uint32_t s_tick_count = 0U;

/** @brief 事件投递失败次数（队列满导致 tick 丢失） */
static volatile uint32_t s_drop_count = 0U;

/* ============================================================
 * 定时器回调
 * ============================================================ */

/**
 * @brief 50ms 周期定时器回调
 * @param [in] param 未使用
 * @note  运行于 timer 线程（优先级 0）
 *        只做「计数 + 一次 mq_send」，禁止打印与阻塞操作
 */
static void _tick_50ms_cb(void *param)
{
    app_event_t evt = (app_event_t)EVT_TICK_50MS;

    (void)param;

    s_tick_count++;

    /* 队列满时 rt_mq_send 返回 -RT_EFULL，
     * 此时本周期 tick 事件被丢弃，仅累加计数不打印 */
    if (rt_mq_send(g_app_event_mq, &evt, sizeof(app_event_t)) != RT_EOK) {
        s_drop_count++;
    }
//    rt_kprintf("[TICK] s_tick_count = %u \n",(unsigned)s_tick_count);
}

/* ============================================================
 * 生命周期实现
 * ============================================================ */

/**
 * @brief  初始化并启动 50ms 周期定时器
 */
int tick_50ms_init(void)
{
    SM_ASSERT_PTR(g_app_event_mq);

    if (s_tick_timer != RT_NULL) {
        rt_kprintf("[TICK] already initialized\n");
        return 0;
    }

    /* 创建周期软定时器
     * 参数：名称、回调、回调入参、超时（tick 单位）、标志 */
    s_tick_timer = rt_timer_create("tick50",
                                   _tick_50ms_cb,
                                   RT_NULL,
                                   rt_tick_from_millisecond(TICK_50MS_PERIOD_MS),
                                   RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    if (s_tick_timer == RT_NULL) {
        rt_kprintf("[TICK] timer create failed\n");
        return -RT_ERROR;
    }

    s_tick_count = 0U;
    s_drop_count = 0U;

    if (rt_timer_start(s_tick_timer) != RT_EOK) {
        rt_kprintf("[TICK] timer start failed\n");
        return -RT_ERROR;
    }

    rt_kprintf("[TICK] 50ms timer started (period=%ums)\n\n",
               (unsigned)TICK_50MS_PERIOD_MS);

    return 0;
}

/**
 * @brief  停止 50ms 周期定时器
 */
int tick_50ms_stop(void)
{
    if (s_tick_timer == RT_NULL) {
        return 0;
    }

    rt_timer_stop(s_tick_timer);
    rt_kprintf("[TICK] stopped (count=%u)\n", (unsigned)s_tick_count);

    return 0;
}

/* ============================================================
 * 诊断实现
 * ============================================================ */

uint32_t tick_50ms_get_count(void)
{
    return s_tick_count;
}

uint32_t tick_50ms_get_drop_count(void)
{
    return s_drop_count;
}

void tick_50ms_clear_count(void)
{
    s_tick_count = 0U;
    s_drop_count = 0U;
}

void tick_50ms_dump(void)
{
    rt_kprintf("\n========== TICK 50MS ==========\n");
    rt_kprintf("period   : %u ms\n", (unsigned)TICK_50MS_PERIOD_MS);
    rt_kprintf("count    : %u  (~%u s)\n",
               (unsigned)s_tick_count,
               (unsigned)(s_tick_count * TICK_50MS_PERIOD_MS / 1000U));
    rt_kprintf("drop     : %u  (queue full)\n", (unsigned)s_drop_count);
    rt_kprintf("state    : %s\n",
               (s_tick_timer != RT_NULL) ? "created" : "not created");
    rt_kprintf("===============================\n\n");
}

/* ============================================================
 * MSH 调试命令
 * ============================================================ */

/**
 * @brief  t_tick —— 查看 tick 计数与丢弃次数
 * @note   验证周期精度：t_tick_clear 后等 10s，再 t_tick，
 *         count 应约为 200（10s / 50ms），drop 应为 0
 */
static void cmd_tick_dump(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    tick_50ms_dump();
}

/**
 * @brief  t_tick_clear —— 清零统计
 */
static void cmd_tick_clear(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    tick_50ms_clear_count();
    rt_kprintf("[TICK] counters cleared\n");
}

MSH_CMD_EXPORT_ALIAS(cmd_tick_dump,  t_tick,       dump 50ms tick state);
MSH_CMD_EXPORT_ALIAS(cmd_tick_clear, t_tick_clear, clear tick counters);
