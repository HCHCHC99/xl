/**
 * @file    app_event.c
 * @brief   事件投递实现
 */

#include "app_event.h"
#include "ipc_center.h"
#include "framework/state_engine/sm_assert.h"
#include <rtthread.h>

/* ============================================================
 * P1（2026-11-23）事件基座
 * ------------------------------------------------------------
 * 1. 普通事件：队列满仍然丢弃（保持历史行为），但计入 drops。
 * 2. 停止类事件：post_event_stop() 保证不丢。队列满时进 8 槽待处理环，
 *    app_event_wait() 每轮优先取环内事件。
 * 3. 只增加统计与旁路，不改变任何业务分支。
 * ============================================================ */

#define STOP_PEND_MAX        8U

static volatile uint32_t s_posted;
static volatile uint32_t s_dropped;
static volatile uint32_t s_stop_posted;
static volatile uint32_t s_stop_deferred;
static volatile uint32_t s_stop_replayed;
static volatile uint32_t s_stop_dup;
static volatile uint32_t s_stop_overflow;

static app_event_t s_pend[STOP_PEND_MAX];
static uint8_t     s_pend_head;
static uint8_t     s_pend_tail;
static uint8_t     s_pend_cnt;
static uint32_t    s_pend_mask;

/* 事件值 → 区间位（bit n 对应基址 n << 12），仅用于 t_evt 显示 */
static uint32_t evt_zone_bit(app_event_t event)
{
    uint32_t zone = ((uint32_t)event >> 12) & 0x1FU;
    return (1UL << zone);
}

static void pend_mask_rebuild(void)
{
    uint32_t i;

    s_pend_mask = 0U;
    for (i = 0U; i < (uint32_t)s_pend_cnt; i++) {
        uint8_t idx = (uint8_t)((s_pend_head + i) % STOP_PEND_MAX);
        s_pend_mask |= evt_zone_bit(s_pend[idx]);
    }
}

/**
 * @brief 投递事件到前台主队列
 * @note  【2026-10-27 · D-58】原为 SM_ASSERT_PTR(g_app_event_mq)。
 *
 *        本函数是全工程最热的路径之一（57 个调用点），keyboard 线程
 *        每次按键扫描、CAN 解析线程每帧分发、各业务模块每次状态变化
 *        都要经过它。而 SM_ASSERT_PTR 在失败时会展开：
 *          rt_assert_handler(32) → rt_kprintf(32) → rt_vsnprintf(80)
 *            → print_number(72) ...  ≈ 600 B
 *        即每个调用者的栈都要为这条"永不发生的分支"预留 600 B。
 *
 *        g_app_event_mq 在 boot 第 2 步 ipc_center_init 就已创建，
 *        早于所有可能调用 post_event 的线程启动，NULL 不可达。
 *        故降级为直接返回：异常时丢一个事件（可接受），
 *        而不是把整机卡死在 rt_assert_handler 的 while(1) 里。
 */
void post_event(app_event_t event)
{
    if (g_app_event_mq == RT_NULL) {
        s_dropped++;
        return;
    }

    if (rt_mq_send(g_app_event_mq, &event, sizeof(app_event_t)) == RT_EOK) {
        s_posted++;
    } else {
        s_dropped++;
    }
}

/**
 * @brief 投递停止类事件（不丢）
 * @note  队列有空位时直接入队；队列满时压入待处理环。
 *        同一事件已在环内则去重（停止指令重复执行是幂等的）。
 *        环满时丢最旧一条并计 stop_overflow —— 8 槽对当前
 *        只有 ACC_OFF / UV_ENTER / SEQ_STOP 几类停止事件足够。
 */
void post_event_stop(app_event_t event)
{
    uint8_t i;
    uint8_t tail;

    if (g_app_event_mq == RT_NULL) {
        s_dropped++;
        return;
    }

    if (rt_mq_send(g_app_event_mq, &event, sizeof(app_event_t)) == RT_EOK) {
        s_stop_posted++;
        return;
    }

    rt_enter_critical();

    for (i = 0U; i < s_pend_cnt; i++) {
        uint8_t idx = (uint8_t)((s_pend_head + i) % STOP_PEND_MAX);

        if (s_pend[idx] == event) {
            s_stop_dup++;
            rt_exit_critical();
            return;
        }
    }

    if (s_pend_cnt >= STOP_PEND_MAX) {
        /* 环满：挤掉最旧一条，保证最新停止指令一定进环 */
        s_pend_head = (uint8_t)((s_pend_head + 1U) % STOP_PEND_MAX);
        s_pend_cnt--;
        s_stop_overflow++;
    }

    tail = s_pend_tail;
    s_pend[tail] = event;
    s_pend_tail = (uint8_t)((s_pend_tail + 1U) % STOP_PEND_MAX);
    s_pend_cnt++;
    s_pend_mask |= evt_zone_bit(event);
    s_stop_deferred++;

    rt_exit_critical();
}

/**
 * @brief 取一个待处理事件（app_main 每轮调用）
 * @note  待处理环优先于主队列：停止类事件最多延迟"当前一条事件的处理时间"。
 */
app_event_t app_event_wait(void)
{
    app_event_t event = 0U;

    if (g_app_event_mq == RT_NULL) {
        return 0U;
    }

    rt_enter_critical();
    if (s_pend_cnt > 0U) {
        event = s_pend[s_pend_head];
        s_pend_head = (uint8_t)((s_pend_head + 1U) % STOP_PEND_MAX);
        s_pend_cnt--;
        s_stop_replayed++;
        pend_mask_rebuild();
        rt_exit_critical();
        return event;
    }
    rt_exit_critical();

    (void)rt_mq_recv(g_app_event_mq, &event, sizeof(app_event_t),
                     RT_WAITING_FOREVER);
    return event;
}

void app_event_get_stat(app_event_stat_t *st)
{
    if (st == RT_NULL) {
        return;
    }

    st->posted        = s_posted;
    st->dropped       = s_dropped;
    st->stop_posted   = s_stop_posted;
    st->stop_deferred = s_stop_deferred;
    st->stop_replayed = s_stop_replayed;
    st->stop_dup      = s_stop_dup;
    st->stop_overflow = s_stop_overflow;

    rt_enter_critical();
    st->pend_count = s_pend_cnt;
    st->pend_mask  = s_pend_mask;
    rt_exit_critical();
}

/* ============================================================
 * 【P2 / 2026-11-23】清零投递统计
 * ------------------------------------------------------------
 * 只清计数，不动队列内容与待处理环，因此可以随时调用。
 * 用途：多轮脚本累积后无法区分"本轮新增"，跑完一组脚本先清零再跑下一组。
 * ============================================================ */
void app_event_clear_stat(void)
{
    rt_enter_critical();
    s_posted        = 0U;
    s_dropped       = 0U;
    s_stop_posted   = 0U;
    s_stop_deferred = 0U;
    s_stop_replayed = 0U;
    s_stop_dup      = 0U;
    s_stop_overflow = 0U;
    rt_exit_critical();
}
