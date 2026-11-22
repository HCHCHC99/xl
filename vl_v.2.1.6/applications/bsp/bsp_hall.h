/**
 * @file    bsp_hall.h
 * @brief   Hall 传感器 BSP 层接口（HC32F452）
 * @note    3 路 Hall 信号：PH2, PC13, PC14
 *          独立 EXTI 通道 + 双边沿触发 + 硬件 RC 滤波
 */

#ifndef __BSP_HALL_H
#define __BSP_HALL_H

#include <stdint.h>
#include <rtthread.h>
#include "../hw/hw_hall.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 私有数据类型 */
typedef struct {
    uint8_t initialized;
    uint8_t active_level;
} bsp_hall_priv_t;

/* 供 config 引用 */
extern const hw_hall_ops_t g_bsp_hall_ops;
extern bsp_hall_priv_t     g_bsp_hall_priv;

/* 应用层持有的 Hall 对象 */
extern hw_hall_t g_swing_hall;
extern const hw_hall_cfg_t g_swing_hall_cfg;

#ifdef __cplusplus
}
#endif

#endif /* __BSP_HALL_H */
