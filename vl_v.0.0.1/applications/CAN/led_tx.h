/**
 * @file    led_tx.h
 * @brief   0x18FFA4F0 开关面板灯状态帧组包（本机 → 面板，100ms）
 * @note
 *   每键 4bit：bit0=绿 / bit1=黄 / bit2=红 / bit3=闪烁
 *   灯色规则来自 SRS REQ001-FUN003 / REQ003-FUN003：
 *     绿色常亮   = 无故障无输出
 *     黄灯常亮   = 有输出（动作中）
 *     红 1Hz 闪烁 = 编码器异常
 *     红灯常亮   = 电机堵转
 *     优先级：红 > 黄 > 绿
 */

#ifndef __LED_TX_H
#define __LED_TX_H

#include <stdint.h>
#include "can_tx.h"
#include "app_event.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 报文参数
 * ============================================================ */
#define LED_FRAME_ID            (0x18FFA4F0U)
#define LED_FRAME_PERIOD_MS     (100U)
#define LED_FRAME_PHASE_MS      (50U)   /* 与 status(0ms) 错开，避免同拍发送 */

/* ============================================================
 * 4bit 灯的常用组合
 * ============================================================ */
#define LED_OFF                 (0x00U)
#define LED_GREEN               (0x01U)                 /* 绿常亮 */
#define LED_YELLOW              (0x02U)                 /* 黄常亮 */
#define LED_RED                 (0x04U)                 /* 红常亮 */
#define LED_RED_BLINK           (0x04U | 0x08U)         /* 红 1Hz 闪烁 */
#define LED_YELLOW_BLINK        (0x02U | 0x08U)         /* 黄闪烁 */
#define LED_GREEN_BLINK         (0x01U | 0x08U)         /* 绿闪烁 */

/* ============================================================
 * 接口
 * ============================================================ */

/**
 * @brief 注册 0x18FFA4F0 周期发送项 [单线程]
 */
int led_tx_init(void);

/**
 * @brief 灯状态变更事件入口 [线程安全]
 * @param event CAN_EVT_LED_* 事件
 */
void led_tx_on_event(app_event_t event);

/**
 * @brief 组包回调（由 can_tx 发送线程调用）
 */
void led_tx_pack(can_tx_frame_t *frame);

#ifdef __cplusplus
}
#endif

#endif /* __LED_TX_H */
