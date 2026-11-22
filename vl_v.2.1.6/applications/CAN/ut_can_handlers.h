/**
 * @file    ut_can_handlers.h
 * @brief   CAN 接收解析对外接口
 */

#ifndef __UT_CAN_HANDLERS_H
#define __UT_CAN_HANDLERS_H

#include <stdint.h>

#include "../service/input_sim.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 台架模拟开关（任务清单 §11）
 * ------------------------------------------------------------
 * 1 = 允许 t_sim 注入 0x18FF8380（按键）/ 0x0CF00400（转速）/
 *     0x18FF0E63（阀反馈）/ 0x18FF7F18（机型）四类接收帧
 * 0 = 关闭注入入口，只能从真实 CAN 收到
 * 总开关 INPUT_SIM_ENABLE 在 service/input_sim.h
 * ============================================================ */
#ifndef CAN_SIM_ENABLE
#define CAN_SIM_ENABLE      1
#endif

#if (INPUT_SIM_ENABLE && CAN_SIM_ENABLE)
/**
 * @brief 台架注入一帧接收报文（走 can_route_table 的真实路由与回调）
 * @param id   CAN ID（如 0x18FF8380）
 * @param data 数据区（最长 8 字节）
 * @param len  数据长度
 * @note  用于 t_sim；不经过硬件滤波器与 ringbuffer，直接调回调。
 *        参数非法或未命中路由表时静默返回。
 */
void can_sim_inject(uint32_t id, const uint8_t *data, uint8_t len);
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
