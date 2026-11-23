/**
 * @file    seq_actions.c
 * @brief   序列动作执行实现
 */

#include "seq_actions.h"
#include "app_event.h"
#include "framework/state_engine/sm_assert.h"
#include "../lift/lift_control.h"   /* P7：HOLD 步观测举升阀输出 */
#include "../lift/lift_angle.h"     /* lift_angle_t / lift_angle_get_value */
#include "../swing/swing_angle.h"   /* swing_angle_t / swing_angle_get_value */
#include <stdlib.h>
#include <rtthread.h>

/* ============================================================
 * 内部辅助：获取角度值
 * ============================================================ */
static int32_t get_swing_angle(seq_context_t *ctx)
{
    if (ctx->swing_angle == NULL) return 0;
    return swing_angle_get_value((const swing_angle_t*)ctx->swing_angle);
}

static int32_t get_vert_angle(seq_context_t *ctx)
{
    if (ctx->lift_angle == NULL) return 0;
    return lift_angle_get_value((const lift_angle_t*)ctx->lift_angle);
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
            /* 【P7 / B-4】"保持当前输出"不是空动作——前提是上一步声明了
             * keep_lift，引擎才没有在步末关阀。这里只做一件事：把观测打出来，
             * 让人一眼看出"保持的到底是通电还是空等"。
             * 不主动重开阀：若上一步因限位/故障已断电，重开等于绕过保护。 */
            {
                lift_control_t *lift = (lift_control_t*)ctx->vert_ctrl;

                if ((lift != RT_NULL) && (lift->ctx.solenoid_ptr != RT_NULL)) {
                    uint8_t out = lift->ctx.solenoid_ptr->valve_output;

                    rt_kprintf("[Seq] hold: lift valve out=0x%02X %s\n",
                               (unsigned)out,
                               (out != 0U) ? "(still energized)"
                                           : "(NOT energized - hold is empty)");
                }
            }
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
 * ------------------------------------------------------------
 * 【P7 / B-4】stop_lift=0 时保留举升阀输出，供下一步"保持 N 秒"继续吃。
 * 回转始终停止：序列里没有任何一步需要跨步保持回转。
 * ============================================================ */
void seq_action_stop_ex(seq_context_t *ctx, uint8_t stop_lift)
{
    SM_ASSERT_PTR(ctx);

    /* 停止回转 */
    post_event(SWING_EVT_SEQ_STOP);

    /* 停止举升（按调用方要求） */
    if (stop_lift != 0U) {
        post_event(LIFT_EVT_SEQ_STOP);
    }
}

void seq_action_stop(seq_context_t *ctx)
{
    seq_action_stop_ex(ctx, 1U);
}
