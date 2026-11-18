/**
 * @file    lift_events.c
 * @brief   举升角对象 → 事件投递适配层
 * @note    将位置变化映射为事件投递到消息队列
 */

#include "lift_angle.h"
#include "app_event.h"
#include "framework/state_engine/sm_assert.h"

/* ============================================================
 * 位置变化回调函数
 * ============================================================ */
static void lift_angle_on_position_change(lift_position_t old_pos, lift_position_t new_pos)
{
    /* 位置变化时投递对应事件，由状态机处理 */

    /* 进入中间位（包括 UNKNOWN → MIDDLE） */
    if (new_pos == LIFT_POS_MIDDLE) {
        post_event(LIFT_EVT_LIMIT_CLEARED);
        return;
    }

    /* 进入上限位 */
    if (new_pos == LIFT_POS_UPPER_LIMIT) {
        post_event(LIFT_EVT_LIMIT_REACHED_UPPER);
        return;
    }

    /* 进入下限位 */
    if (new_pos == LIFT_POS_LOWER_LIMIT) {
        post_event(LIFT_EVT_LIMIT_REACHED_LOWER);
        return;
    }

    /* 其他情况（如 UNKNOWN → UNKNOWN）不投递 */
}

/* ============================================================
 * 公开 API
 * ============================================================ */

/**
 * @brief 初始化并绑定事件
 */
void lift_angle_init_with_events(lift_angle_t *angle, lift_sensor_t *sensor)
{
    lift_angle_init(angle, sensor);
    angle->on_position_change = lift_angle_on_position_change;
}

/**
 * @brief 绑定事件回调到已有对象
 */
void lift_angle_init_events(lift_angle_t *angle)
{
    SM_ASSERT_PTR(angle);
    angle->on_position_change = lift_angle_on_position_change;
}
