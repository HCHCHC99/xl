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
    /* 【P6 / B-8 修正】这里**不再**投 SWING_EVT_ZERO_REACHED。
     *
     * 原因（2026-11-23 实测暴露）：过零停车原本有两条触发路径——
     *   ① 本回调：position 变成 ZERO 就投事件（无条件）；
     *   ② swing_angle_update()：符号翻转检测，受 need_zero_stop 控制。
     * 于是 `t_cfg set swing_zero_stop 0` 只关掉了 ②，① 照样把序列停掉，
     * 开关形同虚设（m16 第 24–25 步实测：开关置 0 后过零仍然 LOCKING→IDLE）。
     *
     * 现在统一由 ② 负责：它判"从一侧跨到另一侧"，不依赖落进 ±0.5° 零位带，
     * 快速穿越（一个 50 ms 周期跨 4°）也不会漏；且天然满足
     * "本来就在 0°、向任一侧起步不算过零"。
     * 本回调只保留限位与限位清除两类事件。 */

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
