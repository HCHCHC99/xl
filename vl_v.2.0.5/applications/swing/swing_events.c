/**
 * @file    swing_events.c
 * @brief   回转位置变化 → 事件投递
 */

#include "swing_events.h"
#include "app_event.h"
#include "framework/state_engine/sm_assert.h"

/* ============================================================
 * 位置变化回调函数
 * ============================================================ */
static void swing_on_position_change(swing_position_t old_pos,
                                    swing_position_t new_pos)
{
    /* 进入零位 */
    if (new_pos == SWING_POS_ZERO) {
        post_event(SWING_EVT_ZERO_REACHED);
        return;
    }

    /* 进入左限位 */
    if (new_pos == SWING_POS_LEFT_LIMIT) {
        post_event(SWING_EVT_LIMIT_LEFT_REACHED);
        return;
    }

    /* 进入右限位 */
    if (new_pos == SWING_POS_RIGHT_LIMIT) {
        post_event(SWING_EVT_LIMIT_RIGHT_REACHED);
        return;
    }

    /* 从限位/零位回到中间位 */
    if (new_pos == SWING_POS_MIDDLE) {
        if (old_pos == SWING_POS_LEFT_LIMIT ||
            old_pos == SWING_POS_RIGHT_LIMIT ||
            old_pos == SWING_POS_ZERO) {
            post_event(SWING_EVT_LIMIT_CLEARED);
        }
        return;
    }
}

/* ============================================================
 * 初始化并绑定事件
 * ============================================================ */
void swing_angle_init_with_events(swing_angle_t *angle,
                                 int32_t encoder_zero_offset,
                                 swing_encoder_getter_t getter,
                                 void *user_data)
{
    swing_angle_init(angle, encoder_zero_offset, getter, user_data);
    angle->on_position_change = swing_on_position_change;
}
