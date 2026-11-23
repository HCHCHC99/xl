/**
 * @file    seq_config.h
 * @brief   序列配置表
 */

#ifndef __SEQ_CONFIG_H
#define __SEQ_CONFIG_H

#include "seq_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 全局序列配置表
 * ============================================================ */
/* 【P7 / B-4】回位序列的"已在零位"门限（0.1° 单位，20 = ±2°）。
 * 与 seq_engine_init() 里 ctx->tolerance 的默认值保持一致。 */
#define SEQ_HOME_AT_ZERO_TOL_X10    20

extern const seq_config_t g_seq_right_middle;
extern const seq_config_t g_seq_right_behind;
extern const seq_config_t g_seq_home;
extern const seq_config_t g_seq_home_at_zero;

/* ============================================================
 * 根据序列类型查找配置
 * ============================================================ */
const seq_config_t* seq_config_find(seq_type_t type);

/* ============================================================
 * 回位专用查找（P7 / B-4，TC-ONE-011）
 * @param swing_angle_x10 当前回转角（0.1°）
 * @return |angle| <= ±2° → g_seq_home_at_zero（仅下降 + 保持 2 s）；
 *         否则         → g_seq_home（升 +25 → 转零 → 升 +34 → 下降 → 保持）
 * ============================================================ */
const seq_config_t* seq_config_find_home(int32_t swing_angle_x10);

#ifdef __cplusplus
}
#endif

#endif /* __SEQ_CONFIG_H */
