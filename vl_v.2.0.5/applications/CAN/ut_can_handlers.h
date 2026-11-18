/**
 * @file    ut_can_handlers.h
 * @brief   CAN 接收解析对外接口
 */

#ifndef __UT_CAN_HANDLERS_H
#define __UT_CAN_HANDLERS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 读取最近一次发动机转速
 * @return rpm（0 表示尚未收到有效报文）
 * @note  数据源 0x0CF00400，Byte3-4 小端，精度 0.125 → raw >> 3
 */
uint16_t can_get_engine_rpm(void);

/**
 * @brief 发动机转速帧通信是否超时 [线程安全]
 * @return 1 = **曾收到过** 0x0CF00400 且已超过 ENGINE_RPM_TIMEOUT_MS(100ms) 未更新；
 *         0 = 从未收到过，或帧流正常
 * @note  ⚠️ 2026-11-14 16:49 起**不再是 Q-B 的故障判据**：用户 Q-D1 裁定改用
 *        ut_can_scheduler.h 的 can_is_bus_silent()（整条总线 / 解析线程查表心跳，
 *        未收到任何帧也算静默）。本函数保留供诊断与后续条件扩展（Q-D3）复用（R-05 不删）。
 *        只是**只读镜像**既有采样状态，不新建故障源（设计基线 §13A.2）。
 */
uint8_t can_is_engine_rpm_timeout(void);

/* 注：按键相关接口已全部迁移至 keyboard 模块
 *     （applications/keyboard/keyboard_sm.h）：
 *       · 按键解析 / 组合键 / 互斥仲裁 / 超时释放 / 指令生成
 *       · 启车「重新按下」→ kb_request_rebaseline()
 *       · 超时检测        → 模块内 300ms 一次性软件定时器
 */

#ifdef __cplusplus
}
#endif

#endif /* __UT_CAN_HANDLERS_H */
