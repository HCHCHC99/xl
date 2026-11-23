/**
 * @file    ut_can_scheduler.h
 * @brief   CAN 接收调度器
 */

#ifndef __UT_CAN_SCHEDULER_H
#define __UT_CAN_SCHEDULER_H

#include <stdint.h>

#include "../service/input_sim.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 台架模拟开关（任务清单 §11）
 * ------------------------------------------------------------
 * 1 = 允许 t_sim cansilent 1 强制 can_is_bus_silent() 返回静默
 *     （用于 TC-FLT-015 / 整车 2s FAULT / comm_sta 调试）
 * 0 = 关闭，按真实收帧心跳判定
 * 总开关 INPUT_SIM_ENABLE 在 service/input_sim.h
 * ============================================================ */
#ifndef CAN_SILENT_SIM_ENABLE
#define CAN_SILENT_SIM_ENABLE 1
#endif

void can_scheduler_init(void);
void can_scheduler_push_frame(uint32_t id, uint8_t *data, uint8_t len);
void can_scheduler_start(void);

/**
 * @brief CAN 总线是否静默（未收到任何可通过筛选过滤的报文）[线程安全]
 * @param timeout_ms 静默判定时限；comm_sta/控制器灯用 CFG_ID_CAN_FAULT_MS（300），
 *                   整车 FAULT 用 CFG_ID_VEH_CAN_SILENT_MS（500）
 * @return 1 = 静默；0 = 通信有效
 *
 * 【Q-D1 / 2026-11-14 16:49 裁定】心跳放在**解析线程**、且以「触发查表」为准：
 *   can_parser_thread_entry() 每从 ringbuffer 取出一帧、进入查表 for 之前，
 *   即记一次 s_any_frame_tick —— **不要求命中 can_route_table**。
 *   ⇒ 语义 = 「通过硬件滤波器（当前全收，ut_can_send.c:71-76）的任何非 RTR 帧」。
 *   放在线程侧而非 ISR，避开 D-58「ISR 内禁止断言/打印/阻塞」约束。
 *
 * 【从未收到任何帧】s_any_frame_tick == 0 时直接返回 1（静默），
 *   上电即报通信故障；收到第一帧后按 age 自动恢复。
 */
uint8_t can_is_bus_silent(uint32_t timeout_ms);

/**
 * @brief 距最近一帧的时长（ms）[线程安全]
 * @return 0xFFFFFFFF = 上电至今从未收到任何帧
 */
uint32_t can_last_frame_age_ms(void);

#if (INPUT_SIM_ENABLE && CAN_SILENT_SIM_ENABLE)
/**
 * @brief 台架专用：把"最近一帧时刻"拨到 age_ms 之前（t_comm test/age 用）
 * @note  仅 INPUT_SIM_ENABLE && CAN_SILENT_SIM_ENABLE 时编译；
 *        用于在无真实总线的情况下验证 300 ms 门限边界。
 */
void can_scheduler_set_last_frame_age_ms(uint32_t age_ms);
#endif

#ifdef __cplusplus
}
#endif

#endif /* __UT_CAN_SCHEDULER_H */
