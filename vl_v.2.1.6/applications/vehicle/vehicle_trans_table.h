/**
 * @file    vehicle_trans_table.h
 * @brief   整车状态机配置的外部引用声明
 */

#ifndef __VEHICLE_TRANS_TABLE_H
#define __VEHICLE_TRANS_TABLE_H

#include "framework/state_engine/state_engine.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 状态机配置（供 vehicle_control.c 的 control_object_init 使用）*/
extern const StateMachineConfig_t g_vehicle_config;

/** 函数表 / 跳转表（调试或单元测试用）*/
extern const StateFuncTable_t g_vehicle_func_table[];
extern const uint16_t         g_vehicle_func_table_size;
extern const StateJumpTable_t g_vehicle_jump_table[];
extern const uint16_t         g_vehicle_jump_table_size;

#ifdef __cplusplus
}
#endif

#endif /* __VEHICLE_TRANS_TABLE_H */