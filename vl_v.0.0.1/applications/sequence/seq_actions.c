/**
 * @file    seq_actions.c
 * @brief   序列动作执行实现
 */

#include "seq_actions.h"
#include "app_event.h"
#include "framework/state_engine/sm_assert.h"
#include <stdlib.h>
#include <rtthread.h>

/* ============================================================
 * 内部辅助：获取角度值
 * ============================================================ */
static int32_t get_swing_angle(seq_context_t *ctx)
{
    if (ctx->swing_angle == NULL) return 0;
    extern int32_t swing_angle_get_value(const void *angle);
    return swing_angle_get_value(ctx->swing_angle);
}

static int32_t get_vert_angle(seq_context_t *ctx)
{
    if (ctx->lift_angle == NULL) return 0;
    extern int32_t lift_angle_get_value(const void *angle);
    return lift_angle_get_value(ctx->lift_angle);
}

/* ============================================================
 * 动作执行
 * ============================================================ */
void seq_action_execute(seq_context_t *ctx, const seq_step_t *step)
{
    SM_ASSERT_PTR(ctx);
    SM_ASSERT_PTR(step);

    switch (step->action) {
        /* ----- 举升动作 ----- */
        case SEQ_ACT_LIFT_UP:
            post_event(LIFT_EVT_SEQ_UP);
            break;

        case SEQ_ACT_LIFT_DOWN:
            post_event(LIFT_EVT_SEQ_DOWN);
            break;

        case SEQ_ACT_LIFT_HOLD:
            /* 保持当前输出，不发送新指令 */
            break;

        case SEQ_ACT_LIFT_STOP:
            post_event(LIFT_EVT_SEQ_STOP);
            break;

        /* ----- 回转动作 ----- */
        case SEQ_ACT_SWING_LEFT:
            post_event(SWING_EVT_SEQ_MOVE_LEFT);
            break;

        case SEQ_ACT_SWING_RIGHT:
            post_event(SWING_EVT_SEQ_MOVE_RIGHT);
            break;

        case SEQ_ACT_SWING_MOVE_TO:
            {
                /* 根据当前位置和目标，自动决定方向 */
                int32_t h_current = get_swing_angle(ctx);
                int32_t h_target = step->target;

                if (h_current < h_target) {
                    post_event(SWING_EVT_SEQ_MOVE_RIGHT);
                } else if (h_current > h_target) {
                    post_event(SWING_EVT_SEQ_MOVE_LEFT);
                } else {
                    /* 已在目标位置 */
                    post_event(SWING_EVT_SEQ_STOP);
                }
            }
            break;

        case SEQ_ACT_SWING_STOP:
            post_event(SWING_EVT_SEQ_STOP);
            break;

        /* ----- 通用动作 ----- */
        case SEQ_ACT_DELAY:
            /* 纯延时，不做动作 */
            break;

        case SEQ_ACT_CUSTOM:
            if (step->custom_action) {
                step->custom_action(ctx);
            }
            break;

        default:
            break;
    }
}

/* ============================================================
 * 停止当前步骤的所有输出
 * ============================================================ */
void seq_action_stop(seq_context_t *ctx)
{
    SM_ASSERT_PTR(ctx);

    /* 停止回转和举升 */
    post_event(SWING_EVT_SEQ_STOP);
    post_event(LIFT_EVT_SEQ_STOP);
}
