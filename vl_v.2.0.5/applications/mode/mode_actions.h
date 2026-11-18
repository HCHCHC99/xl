/**
 * @file    mode_actions.h
 * @brief   模式动作函数声明
 */

#ifndef __MODE_ACTIONS_H
#define __MODE_ACTIONS_H

#include "framework/state_engine/state_engine.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Enter 回调 */
void mode_enter_init(StateContext_t ctx);
void mode_enter_factory(StateContext_t ctx);
void mode_enter_test(StateContext_t ctx);
void mode_enter_work(StateContext_t ctx);

/* Exit 回调 */
void mode_exit_init(StateContext_t ctx);
void mode_exit_factory(StateContext_t ctx);
void mode_exit_test(StateContext_t ctx);
void mode_exit_work(StateContext_t ctx);

/* Tick 检查（50ms 周期） */
void mode_tick_check(StateContext_t ctx);

#ifdef __cplusplus
}
#endif

#endif /* __MODE_ACTIONS_H */
