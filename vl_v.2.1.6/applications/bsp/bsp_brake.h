/**
 * @file    bsp_brake.h
 * @brief   刹车盘 BSP 层接口
 * @note    硬件配置：GPIO 直接驱动，通电解锁，断电锁止
 */

#ifndef __BSP_BRAKE_H
#define __BSP_BRAKE_H

#include <stdint.h>
#include <rtthread.h>

#include <stdint.h>
#include "../hw/hw_brake.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 私有数据类型 */
typedef struct {
    uint8_t initialized;
} bsp_brake_priv_t;

extern hw_brake_t g_swing_brake;
/* 供 config 引用 */
extern const hw_brake_ops_t g_bsp_brake_ops;
extern bsp_brake_priv_t     g_bsp_brake_priv;

#ifdef __cplusplus
}
#endif

#endif /* __BSP_BRAKE_H */
