/**
 * @file    lift_events.h
 * @brief   举升位置事件适配层
 */

#ifndef __LIFT_EVENTS_H
#define __LIFT_EVENTS_H

#include "lift_angle.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化举升角对象并绑定事件回调
 * @param angle   举升角对象
 * @param sensor  传感器对象
 */
void lift_angle_init_with_events(lift_angle_t *angle, lift_sensor_t *sensor);

/**
 * @brief 绑定事件回调到已有对象
 */
void lift_angle_init_events(lift_angle_t *angle);

#ifdef __cplusplus
}
#endif

#endif /* __LIFT_EVENTS_H */
