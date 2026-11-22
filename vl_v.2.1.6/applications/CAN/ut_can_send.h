/**
 * @file    ut_can_send.h
 * @brief   CAN 发送模块对外接口
 */

#ifndef __UT_CAN_SEND_H
#define __UT_CAN_SEND_H

#include <stdint.h>
#include <rtdevice.h>    /* 提供 struct rt_can_msg */

/* ============================================================
 * 设备管理
 * ============================================================ */
typedef struct {
    rt_device_t dev;                     /* 设备句柄 */
    uint8_t     is_ready;                /* 1=已就绪 */
    uint8_t     is_primary;              /* 1=主通道 */
} can_device_entry_t;


/* -------------------- 模块初始化 -------------------- */
/**
 * @brief 初始化 CAN 发送模块
 * @return 0 成功，-1 失败
 * @note 根据配置表查找并打开 CAN 设备，主通道不可用时直接报错
 */
int can_send_init(void);

/* -------------------- 发送接口 -------------------- */
/**
 * @brief 通过主通道发送 CAN 帧
 * @param id   CAN ID（扩展帧，如 0x18FF11D1）
 * @param data 数据域指针（8字节）
 * @param len  数据长度（0~8）
 * @return 0 成功，-1 设备未就绪，-2 发送失败
 * @note 主通道由配置表中的 is_primary=1 决定
 */
int can_send(uint32_t id, const uint8_t *data, uint8_t len);

/**
 * @brief 通过指定索引的通道发送 CAN 帧
 * @param ch_idx 通道索引（对应配置表中的位置）
 * @param id     CAN ID
 * @param data   数据域指针
 * @param len    数据长度
 * @return 0 成功，-1 设备未就绪，-2 发送失败
 */
int can_send_by_channel(uint8_t ch_idx, uint32_t id, uint8_t *data, uint8_t len);


#ifdef __cplusplus
}
#endif

#endif /* __UT_CAN_SEND_H */
