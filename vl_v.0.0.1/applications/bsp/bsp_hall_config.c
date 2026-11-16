/**
 * @file    bsp_hall_config.c
 * @brief   Hall 传感器配置表（回转轴）
 */

#include "bsp_hall.h"
#include "../hw/hw_hall.h"

/* ============================================================
 * 回转轴刹车盘配置
 * ============================================================ */
const hw_hall_cfg_t g_swing_hall_cfg = {
    .name = "swing_hall",
    .priv = &g_bsp_hall_priv,
};
