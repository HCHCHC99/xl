/**
 * @file    bg_scheduler.h
 * @brief   后台调度器 - 时间轮询，管理非实时周期性任务
 */

#ifndef __BG_SCHEDULER_H
#define __BG_SCHEDULER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


typedef enum {
    SYS_PRIO_HIGHEST = 0,   /* 故障检测、软定时器 */
    SYS_PRIO_HIGH    = 1,   /* 电源、CAN、位置、控制环 */
    SYS_PRIO_MID     = 2,   /* 配置、按键、LED、状态机 */
    SYS_PRIO_LOW     = 3,
    SYS_PRIO_LOWEST  = 4,   /* 机型配置、日志 */
    SYS_PRIO_MAX
} sys_prio_t;


/* ============================================================
 * 后台模块结构
 * ============================================================ */
typedef struct {
    const char *name;           /* 模块名称（调试用） */
    void (*init)(void);         /* 初始化函数（可为 NULL） */
    void (*task)(void);         /* 周期性任务函数 */
    sys_prio_t   prio;          /* 优先级 */
    uint32_t period_ms;         /* 任务执行周期（毫秒） */

    /* 运行时 */
    uint32_t     last_run_tick;     /* 上次运行时间点（运行时更新） */
    uint32_t     run_count;      /* ★ 新增 */
    uint32_t     max_cost_us;    /* ★ 新增 */
} bg_module_t;

/* ============================================================
 * API
 * ============================================================ */

/**
 * @brief 初始化后台调度器
 * @note  在 main.c 中通过 INIT_APP_EXPORT 调用
 */
int bg_scheduler_init(void);

/**
 * @brief 后台调度线程入口
 */
void bg_scheduler_thread_entry(void *param);

#ifdef __cplusplus
}
#endif

#endif /* __BG_SCHEDULER_H */
