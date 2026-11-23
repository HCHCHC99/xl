/**
 * @file    led_tx.h
 * @brief   0x18FFA4F0 开关面板灯状态帧组包（本机 → 面板，100ms）
 * @note
 *   本文件只负责"把决策结果塞进 8 字节位域并注册发送"。
 *   **灯色规则在 led_effect.c**（P8 拆分），这里不判断任何业务状态。
 *
 *   拆分原因：台架上没有实物手柄、CAN 对端也没接，若规则混在传输层，
 *   验"编码器故障该亮红闪"就必须先让整条 CAN 链路通。
 *   现在 t_led 直接打印 led_effect 的结果，与是否发送无关。
 */

#ifndef __LED_TX_H
#define __LED_TX_H

#include <stdint.h>
#include "can_tx.h"
#include "app_event.h"
#include "led_effect.h"     /* 灯色编码 + 决策接口 */

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 报文参数
 * ============================================================ */
#define LED_FRAME_ID            (0x18FFA4F0U)
#define LED_FRAME_PERIOD_MS     (100U)
#define LED_FRAME_PHASE_MS      (50U)   /* 与 status(0ms) 错开，避免同拍发送 */

/* 4bit 灯色编码（LED_OFF/LED_GREEN/...）见 led_effect.h */

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
 * @brief 重新计算全部灯色并写入帧缓存（P8）
 * @note  不依赖发送是否开启；t_led 读的就是这份缓存
 */
void led_tx_refresh(void);

/**
 * @brief 组包回调（由 can_tx 发送线程调用）
 */
void led_tx_pack(can_tx_frame_t *frame);

/**
 * @brief 取最近一次决策结果（t_led / 其它诊断用，只读）
 */
const led_effect_t *led_tx_get_effect(void);

/**
 * @brief 打印本帧的 id / 周期 / raw / 逐键灯色与判定依据（t_led 调用）
 */
void led_tx_dump(void);

#ifdef __cplusplus
}
#endif

#endif /* __LED_TX_H */
