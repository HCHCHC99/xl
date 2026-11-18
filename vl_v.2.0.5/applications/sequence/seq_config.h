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
extern const seq_config_t g_seq_right_middle;
extern const seq_config_t g_seq_right_behind;
extern const seq_config_t g_seq_home;

/* ============================================================
 * 根据序列类型查找配置
 * ============================================================ */
const seq_config_t* seq_config_find(seq_type_t type);

#ifdef __cplusplus
}
#endif

#endif /* __SEQ_CONFIG_H */
