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

#ifdef __cplusplus
}
#endif

#endif /* __SEQ_ACTIONS_H */
