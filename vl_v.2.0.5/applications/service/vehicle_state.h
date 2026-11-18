/**
 * @file    vehicle_state.h
 * @brief   整车状态判定 —— 【兼容头】转发到 vehicle/vehicle_control.h
 *
 * 【2026-11-12 对象化】原模块已按基类模式重构为 vehicle_control_t，
 * 本头文件保留以兼容既有 include，内容全部转发。
 *
 * 原 API 签名完全不变 ⇒ app_main.c / boot_table.c 零改动。
 */

#ifndef __VEHICLE_STATE_H
#define __VEHICLE_STATE_H

/* 全部类型与 API 由新头文件提供：
 *   vehicle_cfg_t       —— 判定参数表（原样保留）
 *   g_vehicle_cfg       —— 默认参数表实例
 *   vehicle_state_init()
 *   vehicle_poll()
 *   vehicle_is_started()
 *   vehicle_force_reset()
 */
#include "../vehicle/vehicle_control.h"

#endif /* __VEHICLE_STATE_H */