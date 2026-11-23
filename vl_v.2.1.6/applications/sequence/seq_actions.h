/**
 * @file    seq_actions.h
 * @brief   序列动作执行函数声明
 */

#ifndef __SEQ_ACTIONS_H
#define __SEQ_ACTIONS_H

#include "seq_types.h"
#include "seq_engine.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 执行步骤动作
 */
void seq_action_execute(seq_context_t *ctx, const seq_step_t *step);

/**
 * @brief 停止当前步骤的所有输出
 */
void seq_action_stop(seq_context_t *ctx);

/**
 * @brief 停止输出（可选择保留举升阀）
 * @param stop_lift 1 = 举升也停（默认/安全）；0 = 保留举升阀输出
 * @note  P7 / B-4：仅当步骤声明 keep_lift 时，引擎才会传 0。
 */
void seq_action_stop_ex(seq_context_t *ctx, uint8_t stop_lift);

#ifdef __cplusplus
}
#endif

#endif /* __SEQ_ACTIONS_H */
