/**
 * @file    swing_actions.h
 * @brief   回转控制动作函数声明
 */

#ifndef __SWING_ACTIONS_H
#define __SWING_ACTIONS_H

#include "framework/state_engine/state_engine.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Enter 回调 */
void swing_act_enter_idle(StateContext_t ctx);
void swing_act_enter_left_running(StateContext_t ctx);
void swing_act_enter_right_running(StateContext_t ctx);
void swing_act_enter_fault(StateContext_t ctx);
void swing_act_enter_unlocking(StateContext_t ctx);
void swing_act_enter_locking(StateContext_t ctx);

/* Exit 回调 */
void swing_act_exit_left_running(StateContext_t ctx);
void swing_act_exit_right_running(StateContext_t ctx);

/* Tick 检查（50ms） */
void swing_act_tick_check(StateContext_t ctx);
void swing_act_tick_unlocking(StateContext_t ctx);
void swing_act_tick_locking(StateContext_t ctx);

#ifdef __cplusplus
}
#endif

#endif /* __SWING_ACTIONS_H */
