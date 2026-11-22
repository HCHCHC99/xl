/**
 * @file    bg_scheduler.c
 * @brief   后台调度器实现
 */
#include "hw/hw_brake.h"
#include "bg_scheduler.h"
#include "stack_guard.h"
#include "framework/state_engine/sm_assert.h"
#include <rtthread.h>

/* 后台模块表（在 bg_modules.c 中定义） */
extern const bg_module_t g_bg_modules[];
extern const uint16_t g_bg_module_count;

/* 【2026-10-27 · P3】自适应等待的休眠上下限（ms）
 *
 * MIN：下限。理论上 next_delta 算出来是 0 时代表"已到期"，此时不该睡，
 *      但 rt_thread_mdelay(0) 是 yield 语义（会让出 CPU 但仍在就绪态），
 *      在极小周期模块连续到期时可能形成忙等。给 1 ms 作为最小让步，
 *      既能真正阻塞 1 个 tick，也保证不会漏掉高精度模块。
 * MAX：兜底上限。防止两种异常：
 *      ① 没有任何 task 非空的模块 → next_delta 保持初值，不会永不休眠；
 *      ② 某模块 period_ms 被误设成几万 → 不让线程睡过久，保留自愈能力。 */
#define BG_SCHED_MIN_SLEEP_MS     1U

/* 注意：MAX 必须 ≥ 所有模块周期的最大值。当前模块表只有 100 ms，
 * 取 1000 ms 留足余量；若将来加入周期 > 1 s 的模块，须同步上调。 */
#define BG_SCHED_MAX_SLEEP_MS     1000U

/**
 * @brief 初始化后台调度器
 */
int bg_scheduler_init(void)
{
    SM_ASSERT_PTR(g_bg_modules);
    SM_ASSERT_MSG(g_bg_module_count > 0, "BG modules table empty");

    rt_kprintf("[bg_sched] %d modules registered.\n\n", g_bg_module_count);

    return 0;
}

static void brake_tick_handler(void)
{
    extern hw_brake_t g_swing_brake;
    hw_brake_tick(&g_swing_brake, 10U);
}
/**
 * @brief 后台调度线程入口
 */
void bg_scheduler_thread_entry(void *param)
{
    uint32_t now_tick;
    uint16_t i;
    uint8_t  p;
    uint32_t next_delta;    /* 距最近一个模块到期还差多少 tick */
    uint32_t waited;

    rt_kprintf("[bg_sched] Started, polling background tasks...\n\n");

//    /* 执行所有模块的初始化 */
//    for (i = 0; i < g_bg_module_count; i++) {
//        if (g_bg_modules[i].init != NULL) {
//            g_bg_modules[i].init();
//            rt_kprintf("[bg_sched] %s init done.\n", g_bg_modules[i].name);
//        }
//    }

//    while (1) {
//        now_tick = rt_tick_get();
//
//        /* 遍历所有模块，执行周期性任务 */
//        for (i = 0; i < g_bg_module_count; i++) {
//            const bg_module_t *mod = &g_bg_modules[i];
//
//            if (mod->task == NULL) continue;
//
//            if (now_tick - mod->last_run_tick >= mod->period_ms) {
//                ((bg_module_t*)mod)->last_run_tick = now_tick;
//                mod->task();
//            }
//        }
//
//        rt_thread_mdelay(2);
//    }

    /* init 也按 prio 顺序 */
    for (p = 0; p < SYS_PRIO_MAX; p++) {
        for (i = 0; i < g_bg_module_count; i++) {
            if (g_bg_modules[i].prio == p && g_bg_modules[i].init) {
                g_bg_modules[i].init();
            }
        }
    }

    while (1) {
        now_tick = rt_tick_get();
        for (p = 0; p < SYS_PRIO_MAX; p++) {
            for (i = 0; i < g_bg_module_count; i++) {
                bg_module_t *m = (bg_module_t *)&g_bg_modules[i];
                if (m->prio != p || m->task == NULL) continue;
                if (now_tick - m->last_run_tick >= m->period_ms) {
                    uint32_t t0 = rt_tick_get();
                    m->last_run_tick = now_tick;
                    m->task();
                    uint32_t us = (rt_tick_get() - t0) * 1000000U / RT_TICK_PER_SECOND;
                    m->run_count++;
                    if (us > m->max_cost_us) m->max_cost_us = us;
                }
            }
        }

        /* 【2026-10-27 · 用户决定停用栈水位巡检】
         *
         * 原代码在此调用 stack_guard_scan()，按 STACK_GUARD_PERIOD_MS 每秒
         * 扫一次全部线程栈，超阈值打印 [STACK] WARN。
         *
         * 停用原因同 stack_guard.c 顶部注释：用户实测 t_stack 全报 100%
         * 与 ps 矛盾，但那是 11:16 旧固件的行为，修复版从未被实测。
         * 用户明确表示不再纠结此项、优先推进应用功能，故一并摘除。
         *
         * 保留调用语句并注释，便于日后一行恢复。
         * 恢复时需同时把 stack_guard.c 里的 #if 0 改回 #if 1。 */
        // stack_guard_scan();

        /* 【2026-10-27 · P3】固定 mdelay(1) → 自适应等待。
         *
         * 问题：原代码固定 1 ms 醒一次，而现存模块周期是 100 ms，
         * 等于 100:1 空转。虽然当前模块表里唯一一项 task == NULL 不耗 CPU，
         * 但线程唤醒、优先级链扫描、上下文切换本身是实打实的开销，
         * 且会与 finsh 争抢 CPU —— 控制台输入延迟的直接贡献项之一。
         *
         * 算法：遍历所有 task 非空的模块，算出"距下次到期还差多少 tick"，
         * 取最小值作为休眠时长。这样不论模块周期是 100 ms 还是 5 ms，
         * 都能整点唤醒，既无空转也不误期。
         *
         * 三个关键保护：
         *  ① next_delta 初值给 BG_SCHED_MAX_SLEEP_MS 的 tick 数，不是 0 ——
         *     否则"没有任何活跃模块"时永远不休眠，变成纯死循环。
         *  ② 到期或已过期（delta == 0）时把 next_delta 压到 0，立即回到
         *     循环开头重跑一次任务分发，不能睡。
         *  ③ 休眠时长夹在 [BG_SCHED_MIN_SLEEP_MS, BG_SCHED_MAX_SLEEP_MS]，
         *     下限保证加新模块时不会因 delta 计算异常而忙等，上限是兜底，
         *     防止 period_ms 被误设成 0 或极大值时睡死。 */
        next_delta = rt_tick_from_millisecond((rt_int32_t)BG_SCHED_MAX_SLEEP_MS);

        for (i = 0; i < g_bg_module_count; i++) {
            const bg_module_t *m = &g_bg_modules[i];
            uint32_t elapsed;
            uint32_t period_tick;
            uint32_t delta;

            if (m->task == NULL || m->period_ms == 0U) {
                continue;
            }

            elapsed     = now_tick - m->last_run_tick;
            period_tick = rt_tick_from_millisecond((rt_int32_t)m->period_ms);

            if (elapsed >= period_tick) {
                delta = 0U;                 /* 已到期，不能再睡 */
            } else {
                delta = period_tick - elapsed;
            }

            if (delta < next_delta) {
                next_delta = delta;
            }
        }

        waited = next_delta * 1000U / RT_TICK_PER_SECOND;   /* tick → ms，向下取整 */
        if (waited < BG_SCHED_MIN_SLEEP_MS) {
            waited = BG_SCHED_MIN_SLEEP_MS;
        } else if (waited > BG_SCHED_MAX_SLEEP_MS) {
            waited = BG_SCHED_MAX_SLEEP_MS;
        }

        rt_thread_mdelay(waited);
    }
}
