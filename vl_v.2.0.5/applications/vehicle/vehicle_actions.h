/**
 * @file    vehicle_actions.h
 * @brief   整车四态状态机的状态动作声明（2026-11-14 P2）
 */

#ifndef __VEHICLE_ACTIONS_H
#define __VEHICLE_ACTIONS_H

#include "framework/state_engine/state_engine.h"

#ifdef __cplusplus
extern "C" {
#endif

/* OFF —— 未上电 / 未启车 */
void vehicle_enter_off(StateContext_t context);
void vehicle_exit_off(StateContext_t context);

/* ACC_ON —— 钥匙上电，门 1 成立 */
void vehicle_enter_acc_on(StateContext_t context);
void vehicle_exit_acc_on(StateContext_t context);

/* STARTED —— 已启车，门 2 成立 */
void vehicle_enter_started(StateContext_t context);
void vehicle_exit_started(StateContext_t context);

/* FAULT —— 故障降级，仅手动 */
void vehicle_enter_fault(StateContext_t context);
void vehicle_exit_fault(StateContext_t context);

#ifdef __cplusplus
}
#endif

#endif /* __VEHICLE_ACTIONS_H */
