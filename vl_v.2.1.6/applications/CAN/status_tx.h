/**
 * @file    status_tx.h
 * @brief   0x18FF11D1 控制器状态反馈帧组包（本机 → HMI，100ms）
 * @note
 *   数据源：各业务模块状态变更后 post_event()，本模块在事件回调中
 *           刷新本地缓存共用体 s_tx，发送线程按周期读取组包。
 *   禁止在 pack 中直接读取其他模块对象（避免跨线程竞态）。
 */

#ifndef __STATUS_TX_H
#define __STATUS_TX_H

#include <stdint.h>
#include "can_tx.h"
#include "app_event.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 报文参数
 * ============================================================ */
#define STATUS_FRAME_ID         (0x18FF11D1U)
#define STATUS_FRAME_PERIOD_MS  (100U)
#define STATUS_FRAME_PHASE_MS   (0U)

/* ============================================================
 * Byte0 bit0-1 电源电压状态
 * ============================================================ */
#define STA_POWER_NORMAL        (0U)    /* 00 正常 */
#define STA_POWER_UNDER         (1U)    /* 01 欠压 */
#define STA_POWER_OVER          (2U)    /* 10 过压 */

/* ============================================================
 * Byte0 bit2-3 控制器校准状态
 * ============================================================ */
#define STA_CALIB_NONE          (0U)    /* 00 无 */
#define STA_CALIB_DONE          (1U)    /* 01 已校准 */
#define STA_CALIB_UNDONE        (2U)    /* 10 未校准 */

/* ============================================================
 * Byte0 bit5-7 控制器工作模式
 * ============================================================ */
#define STA_MODE_STANDBY        (0U)    /* 000 待机 */
#define STA_MODE_MANUAL         (1U)    /* 001 手动状态 */
#define STA_MODE_ONKEY_EXPAND   (2U)    /* 010 一键展开 */
#define STA_MODE_ONKEY_HOME     (3U)    /* 011 一键回位 */

/* ============================================================
 * Byte1 bit1-3 电机工作状态
 * ============================================================ */
#define STA_MOTOR_STOP          (0U)    /* 000 停止 */
#define STA_MOTOR_LEFT          (1U)    /* 001 左转 */
#define STA_MOTOR_RIGHT         (2U)    /* 010 右转 */
#define STA_MOTOR_OPEN          (3U)    /* 011 开路 */
#define STA_MOTOR_STALL         (4U)    /* 100 堵转 */

/* ============================================================
 * Byte1 bit5-7 卸粮筒升降状态
 * ============================================================ */
#define STA_LIFT_STOP           (0U)    /* 000 停止 */
#define STA_LIFT_UP             (1U)    /* 001 上升 */
#define STA_LIFT_DOWN           (2U)    /* 010 下降 */

/* ============================================================
 * Byte2 电磁阀故障码
 * ============================================================ */
#define STA_VALVE_NORMAL        (0U)    /* 00 正常 */
#define STA_VALVE_OPEN          (1U)    /* 01 开路 */
#define STA_VALVE_SHORT         (2U)    /* 10 短路 */

/* ============================================================
 * 接口
 * ============================================================ */

/**
 * @brief 注册 0x18FF11D1 周期发送项 [单线程]
 * @return 0 成功
 */
int status_tx_init(void);

/**
 * @brief 状态变更事件入口 [线程安全]
 * @param event CAN_EVT_STA_* 事件
 * @note  只刷新本地缓存，不直接发帧（周期帧由 can_tx 调度）
 */
void status_tx_on_event(app_event_t event);

/**
 * @brief 组包回调（由 can_tx 发送线程调用）
 */
void status_tx_pack(can_tx_frame_t *frame);

#ifdef __cplusplus
}
#endif

#endif /* __STATUS_TX_H */
