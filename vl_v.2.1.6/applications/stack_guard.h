/**
 * @file    stack_guard.h
 * @brief   线程栈水位巡检
 * @note    周期性扫描各线程栈的填充字节，提前发现即将溢出。
 *          详见 stack_guard.c 顶部的设计说明。
 */

#ifndef __STACK_GUARD_H
#define __STACK_GUARD_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 执行一次栈水位巡检
 * @note  设计为在 bg_sched 线程中周期调用（建议 1 秒一次）。
 *        内部做了启动延迟与告警次数限制，可安全重复调用。
 */
void stack_guard_scan(void);

#ifdef __cplusplus
}
#endif

#endif /* __STACK_GUARD_H */