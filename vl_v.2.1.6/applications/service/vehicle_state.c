/**
 * @file    vehicle_state.c
 * @brief   整车状态判定 —— 【兼容层】已迁移至 vehicle/vehicle_control.c
 *
 * 【2026-11-12 对象化】原 189 行独立服务实现已按基类模式重构：
 *   vehicle_control_t（base + sm_instance + ctx）+ 4 态状态机
 *   实现见 applications/vehicle/
 *
 * 本文件保留为空实现，仅为避免既有构建规则引用失效。
 * **不要在此添加逻辑**——所有改动请到 vehicle/vehicle_control.c。
 */

#include "vehicle_state.h"

/* 全部实现已迁移：
 *   vehicle_state_init()  → vehicle/vehicle_control.c
 *   vehicle_poll()        → vehicle/vehicle_control.c
 *   vehicle_is_started()  → vehicle/vehicle_control.c
 *   vehicle_force_reset() → vehicle/vehicle_control.c
 *   g_vehicle_cfg         → vehicle/vehicle_trans_table.c
 *   t_veh / t_veh_clr     → vehicle/vehicle_control.c（调试命令）
 */