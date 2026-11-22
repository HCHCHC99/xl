/**
 * @file    boot.c
 * @brief   启动序列表驱动执行器（只管一次性启动；周期任务见 bg_scheduler）
 */
#include "boot.h"
#include "boot_table.h"
#include "../mode/sys_caps.h"       /* 能力位门禁（2026-10-27） */
/* 【2026-11-10 彩标 P3】启动日志分级：FAIL/STOPPED=ERR，WARN=WARN，
 * 每步 OK=OK，SKIP=WARN，其余流程信息=INFO（不上色） */
#include "../log/log_color.h"
#include <rtthread.h>
#include <finsh.h>
#include <string.h>

/* 【2026-11-10】原 BOOT_LOG 只走 rt_kprintf，无法分级。
 * 拆为：
 *   BOOT_LOG(fmt,...)    —— 流程信息，默认色（不上色）
 *   BOOT_LOGX(lvl,fmt,..)—— 指定级别，走 LOG_*
 * 为什么保留 BOOT_LOG 而不是全部改成 BOOT_LOGX：
 *   13 处里只有 5 处需要颜色，其余 8 处是表头/流程行。
 *   保持原宏可少改 8 处，降低格式串被打错的风险。 */
#define BOOT_LOG(fmt, ...)   rt_kprintf("[BOOT] " fmt, ##__VA_ARGS__)
#define BOOT_LOGX(lvl, fmt, ...)  LOGF(lvl, "[BOOT] " fmt, ##__VA_ARGS__)

static boot_result_t s_results[BOOT_STEP_MAX];
static uint8_t       s_done      = 0U;
static uint8_t       s_failed_at = 0xFFU;

/* ---------------- 工具 ---------------- */
static int boot_index_of(const char *name)
{
    if (name == NULL) return -1;
    for (uint16_t i = 0; i < g_boot_step_count; i++) {
        if (strcmp(g_boot_steps[i].name, name) == 0) return (int)i;
    }
    return -1;
}

/* 策略 → 是否致命。RETRY 重试耗尽后同样致命 */
static uint8_t boot_is_fatal_policy(uint8_t policy)
{
    return (policy == BOOT_FATAL || policy == BOOT_RETRY) ? 1U : 0U;
}

/* 依赖门禁：前置步骤须「已执行且成功」 */
static int boot_check_deps(const boot_step_t *st)
{
    for (int d = 0; d < BOOT_DEPS_MAX && st->deps[d] != NULL; d++) {
        int idx = boot_index_of(st->deps[d]);
        if (idx < 0) {
            BOOT_LOGX(LOG_LVL_WARN, "WARN %s: unknown dep '%s'\n", st->name, st->deps[d]);
            continue;
        }
        if (s_results[idx].ran == 0U || s_results[idx].result != 0) {
            BOOT_LOGX(LOG_LVL_ERR, "FAIL %s: dep '%s' not ready\n", st->name, st->deps[d]);
            return -1;
        }
    }
    return 0;
}

/* 静态检查 1：同一入口注册两次 */
static void boot_check_dup(void)
{
    for (uint16_t i = 0; i < g_boot_step_count; i++) {
        if (g_boot_steps[i].fn == NULL) continue;
        for (uint16_t j = i + 1; j < g_boot_step_count; j++) {
            if (g_boot_steps[i].fn == g_boot_steps[j].fn) {
                BOOT_LOGX(LOG_LVL_WARN, "WARN dup entry: '%s' == '%s'\n",
                         g_boot_steps[i].name, g_boot_steps[j].name);
            }
        }
    }
}

/* 静态检查 2：level 单调不减 */
static void boot_check_order(void)
{
    for (uint16_t i = 1; i < g_boot_step_count; i++) {
        if (g_boot_steps[i].level < g_boot_steps[i - 1].level) {
            BOOT_LOGX(LOG_LVL_WARN, "WARN order: '%s'(lv%u) after '%s'(lv%u)\n",
                     g_boot_steps[i].name,   (unsigned)g_boot_steps[i].level,
                     g_boot_steps[i - 1].name, (unsigned)g_boot_steps[i - 1].level);
        }
    }
}

/* ---------------- 主流程 ---------------- */
int boot_run_all(void)
{
    const uint16_t n = g_boot_step_count;
    uint32_t t_all;

    memset(s_results, 0, sizeof(s_results));
    s_done      = 0U;
    s_failed_at = 0xFFU;

    t_all = rt_tick_get();

    BOOT_LOG("=== boot start: %u steps ===\n\n", n);
    boot_check_dup();
    boot_check_order();

    for (uint16_t i = 0; i < n; i++) {
        const boot_step_t *st = &g_boot_steps[i];
        boot_result_t     *r  = &s_results[i];
        uint8_t  tries_max;
        uint8_t  tries;
        int      rc = -1;
        uint32_t t0, t1;

        r->ran = 1U;

        /* 1. 依赖门禁 */
        if (boot_check_deps(st) != 0) {
            r->result = -1;
            BOOT_LOG("[%2u/%u] %-16s SKIP (dep not ready)\n", i + 1U, n, st->name);
            if (boot_is_fatal_policy(st->policy)) { s_failed_at = i; goto fatal_stop; }
            continue;
        }

        /* 2. 计算尝试次数：RETRY 用 retry_max，其余 1 次 */
        tries_max = (st->policy == BOOT_RETRY)
                  ? (st->retry_max ? st->retry_max : 1U)
                  : 1U;

        /* 3. 执行（含重试） */
        t0 = rt_tick_get();
        for (tries = 1U; tries <= tries_max; tries++) {
            rc = (st->fn != NULL) ? st->fn() : 0;
            if (rc == 0) break;
            if (tries < tries_max) {
                BOOT_LOG("       retry %u/%u rc=%d\n", tries, tries_max - 1U, rc);
                rt_thread_mdelay(BOOT_RETRY_DELAY_MS);
            }
        }
        t1 = rt_tick_get();

        r->result  = (int8_t)rc;
        r->tries   = tries;
        r->cost_ms = (uint16_t)((t1 - t0) * 1000U / RT_TICK_PER_SECOND);

        /* 4. 逐步骤日志 */
        BOOT_LOG("[%2u/%u] %-16s %s %4ums%s\n",
                 i + 1U, n, st->name,
                 (rc == 0) ? "OK  " : "FAIL",
                 (unsigned)r->cost_ms,
                 (tries > 1U && rc == 0) ? "  (retried)" : "");

        /* 5. 失败处置：FATAL 与 RETRY 都致命 */
        if (rc != 0) {
            if (boot_is_fatal_policy(st->policy)) { s_failed_at = i; goto fatal_stop; }
            BOOT_LOG("       ^ optional, continue (rc=%d)\n", rc);
        }
    }

    s_done = 1U;
    BOOT_LOG("=== boot done: %u ms, %u steps ===\n\n",
             (unsigned)((rt_tick_get() - t_all) * 1000U / RT_TICK_PER_SECOND), n);
    return 0;

fatal_stop:
    BOOT_LOGX(LOG_LVL_ERR, "!!! STOPPED at [%u/%u] %s (rc=%d, tries=%u) !!!\n",
             s_failed_at + 1U, n, g_boot_steps[s_failed_at].name,
             s_results[s_failed_at].result, s_results[s_failed_at].tries);
    BOOT_LOG("!!! %u steps NOT executed: ", n - s_failed_at - 1U);
    for (uint16_t k = s_failed_at + 1U; k < n; k++) {
        rt_kprintf("%s ", g_boot_steps[k].name);
    }
    rt_kprintf("\n");
    return -1;
}

uint8_t              boot_is_done(void)      { return s_done; }
uint8_t              boot_failed_index(void) { return s_failed_at; }

const boot_result_t* boot_get_result(const char *name)
{
    int idx = boot_index_of(name);
    return (idx < 0) ? RT_NULL : &s_results[idx];
}

/* ---------------- 打印 ---------------- */
void boot_dump(void)
{
    static const char *pol[3] = {"FATAL", "OPTION", "RETRY"};

    rt_kprintf("\n=============== BOOT TABLE DUMP ===============\n");
    rt_kprintf("steps=%u  done=%u  failed=%s\n",
               g_boot_step_count, s_done,
               (s_failed_at == 0xFFU) ? "-" : g_boot_steps[s_failed_at].name);
    rt_kprintf("-----------------------------------------------\n");
    rt_kprintf(" # | name             | lv | policy | result | ms\n");
    rt_kprintf("-----------------------------------------------\n");

    for (uint16_t i = 0; i < g_boot_step_count; i++) {
        const boot_step_t   *st = &g_boot_steps[i];
        const boot_result_t *r  = &s_results[i];

        rt_kprintf("%2u | %-16s | %2u | %-6s | %s%3d  | %u\n",
                   i + 1U, st->name, (unsigned)st->level,
                   pol[st->policy & 3U],
                   r->ran ? ((r->result == 0) ? "OK " : "ERR") : "-- ",
                   r->ran ? (int)r->result : 0,
                   (unsigned)r->cost_ms);
    }
    rt_kprintf("==============================================\n\n");
}

/* ---------------- MSH ---------------- */
static void t_boot_dump(int argc, char **argv) { (void)argc; (void)argv; boot_dump(); }
static void t_boot_run (int argc, char **argv) { (void)argc; (void)argv;
                                                 /* 【2026-10-27】重跑启动表会
                                                  * 重复执行硬件初始化（含定时器、
                                                  * CAN、外设复位），运行中调用
                                                  * 会打断正在工作的通道。 */
                                                 if (sys_cap_require(SYS_CAP_DIAG_WRITE, "t_boot_run") == 0U) { return; }
                                                 rt_kprintf("[BOOT] manual re-run\n");
                                                 boot_run_all(); }
MSH_CMD_EXPORT_ALIAS(t_boot_dump, t_boot_dump, dump boot table);
MSH_CMD_EXPORT_ALIAS(t_boot_run,  t_boot_run,  rerun boot sequence);
