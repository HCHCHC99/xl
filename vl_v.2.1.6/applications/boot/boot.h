#ifndef __BOOT_H
#define __BOOT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ================= 启动层级（越小越早） ================= */
typedef enum {
    BOOT_LV_EARLY     = 0,    /* 早期自检：硬件初始化结论校验（无依赖，最先执行） */
    BOOT_LV_IPC       = 5,    /* IPC 中心：mq / sem / mutex */
    BOOT_LV_STORAGE   = 10,   /* 存储：sys_param / device_info / ota */
    BOOT_LV_FRAMEWORK = 20,   /* 框架：ctrl_manager */
    BOOT_LV_HW        = 30,   /* 硬件对象：pwm / brake / hall / motor */
    BOOT_LV_APP       = 40,   /* 应用：mode / lift / swing / seq */
    BOOT_LV_START     = 50,   /* 启动：ctrl_manager_start */
    BOOT_LV_SERVICE   = 60,   /* 服务：bg_sched / can / eeprom */
    BOOT_LV_THREAD    = 70,   /* 线程创建 */
    BOOT_LV_WDT       = 80,   /* 看门狗（最后） */
} boot_level_t;

/* ================= 失败策略 ================= */
typedef enum {
    BOOT_FATAL    = 0,   /* 失败即停 */
    BOOT_OPTIONAL = 1,   /* 失败仅记录，继续 */
    BOOT_RETRY    = 2,   /* 重试 retry_max 次；仍失败 → 按 FATAL 停 */
} boot_policy_t;

#define BOOT_DEPS_MAX        3

/* 【2026-11-10 从 32 扩到 64】
 *
 * 起因：启动表实际已有 54 条，而 boot.c:14 的
 *   static boot_result_t s_results[BOOT_STEP_MAX];   // 只有 32 项
 * 而执行循环用的是 g_boot_step_count（54），
 * 于是 i=32..53 时 s_results[i] 越界写 —— 越界写进 .bss 相邻变量，
 * 把 ctrl_loop 模块的静态实例清零，表现为：
 *   msh > cl list  →  0 instance(s)   （实例注册表被抹）
 * 这是个**静默内存踩踏**，编译器完全不报错。
 *
 * 为什么之前没暴露：ctrl_loop 之前 BOOT_STEP_MAX 恰好够用
 *   （历史条目数 ≤32），本次新增让总数到 54 才踩出来。
 *
 * 扩容到 64：
 *   · 代价 s_results 从 32×~12B=384B 增到 64×~12B=768B（+384B）
 *   · 64 是 2 的幂，便于调试与对齐
 *   · 加编译期断言（boot_table.c 末尾）防止再次静默越界
 * ============================================================ */
#define BOOT_STEP_MAX        64
#define BOOT_RETRY_DELAY_MS  50U   /* RETRY 间隔（ms） */

/* ================= 单条步骤 ================= */
typedef struct {
    const char *name;
    int       (*fn)(void);                 /* 返回 0 为成功 */
    uint8_t     level;                     /* boot_level_t */
    uint8_t     policy;                    /* boot_policy_t */
    uint8_t     retry_max;                 /* 仅 BOOT_RETRY 有效：总尝试次数 */
    const char *deps[BOOT_DEPS_MAX];       /* 前置步骤名（null 结尾） */
} boot_step_t;

/* ================= 执行结果 ================= */
typedef struct {
    uint8_t  ran;        /* 1 = 已执行 */
    int8_t   result;     /* 0 = 成功 */
    uint16_t cost_ms;    /* 耗时（含重试） */
    uint8_t  tries;      /* 实际尝试次数 */
} boot_result_t;

/* ================= API ================= */
int                  boot_run_all(void);
uint8_t              boot_is_done(void);
uint8_t              boot_failed_index(void);   /* 0xFF = 无失败 */
const boot_result_t* boot_get_result(const char *name);
void                 boot_dump(void);

#ifdef __cplusplus
}
#endif
#endif /* __BOOT_H */
