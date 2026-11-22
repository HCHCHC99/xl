/**
 * @file    ut_can_config.h
 * @brief   CAN 设备配置表单 + 路由表类型定义
 */

#ifndef __UT_CAN_CONFIG_H
#define __UT_CAN_CONFIG_H

#include <rtthread.h>
#include "protocol_defs.h"

/* -------------------- 最大通道数（宏定义限制） -------------------- */
#define CAN_CHANNEL_MAX  1   /* 最多支持 1 路 CAN */


/* ============================================================
 * 通道配置结构体
 * ============================================================ */
typedef struct {
    const char *name;        /* 设备名称，如 "can1"、"can2" */
    uint8_t     enabled;     /* 1=启用, 0=禁用 */
    uint8_t     is_primary;  /* 1=主通道（用于发送时的默认选择） */
    /* 未来可扩展字段（如波特率、工作模式等） */
} can_channel_config_t;

/* ============================================================
 * 路由表
 * ============================================================ */
typedef void (*can_rx_callback_t)(const can_frame_t *frame);

typedef struct {
    uint32_t id;
    uint32_t mask;
    can_rx_callback_t callback;
} can_route_entry_t;

/* ============================================================
 * 外部引用：全局配置表
 * ============================================================ */
extern const can_channel_config_t g_can_channels[];
extern const uint8_t g_can_channel_count;

#endif /* __UT_CAN_CONFIG_H */
