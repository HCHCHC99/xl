/**
 * @file    swing_events.h
 * @brief   回转位置事件适配层
 */

#ifndef __SWING_EVENTS_H
#define __SWING_EVENTS_H

#include "swing_angle.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化回转角度对象并绑定事件回调
 */
void swing_angle_init_with_events(swing_angle_t *angle,
                                 int32_t encoder_zero_offset,
                                 swing_encoder_getter_t getter,
                                 void *user_data);

#ifdef __cplusplus
}
#endif

#endif /* __SWING_EVENTS_H */
