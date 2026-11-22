/**
 * @file    bsp_brake_config.c
 * @brief   刹车盘配置表
 */

#include "bsp_brake.h"
#include "../hw/hw_brake.h"

/* ============================================================
 * 回转轴刹车盘配置
 * ============================================================ */
const hw_brake_cfg_t g_swing_brake_cfg = {
    .name       = "swing_brake",
    .priv       = &g_bsp_brake_priv,
    .unlock_ms  = 250U,
    .lock_ms    = 100U,
    .timeout_ms = 500U,
};
