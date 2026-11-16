/**
 * @file    lift_actions.h
 * @brief   举升控制动作函数声明
 */

#ifndef __LIFT_ACTIONS_H
#define __LIFT_ACTIONS_H

#include "framework/state_engine/state_engine.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Enter 回调 */
void lift_act_enter_idle(StateContext_t ctx);
void lift_act_valve_on_up(StateContext_t ctx);
void lift_act_valve_on_down(StateContext_t ctx);
void lift_act_valve_off(StateContext_t ctx);
void lift_act_enter_hold(StateContext_t ctx);
void lift_act_exit_hold(StateContext_t ctx);
void lift_act_enter_fault_valve(StateContext_t ctx);
void lift_act_enter_fault_sensor(StateContext_t ctx);

/* Tick 检查 */
void lift_act_tick_check(StateContext_t ctx);

#ifdef __cplusplus
}
#endif

#endif /* __LIFT_ACTIONS_H */
