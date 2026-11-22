/**
 * @file    motor_ctrl_position_pid.h
 * @brief   位置环 PID 控制
 */

#ifndef __MOTOR_CTRL_POSITION_PID_H
#define __MOTOR_CTRL_POSITION_PID_H

#include "../motor.h"
#include "../motor_ctrl.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 位置 PID 配置
 * ============================================================ */
typedef struct {
    float   kp;
    float   ki;
    float   kd;
    int32_t max_output;         /* 输出限幅 */
    int32_t integral_limit;     /* 积分限幅 */
} motor_pos_pid_cfg_t;

extern const motor_ctrl_ops_t motor_ctrl_position_pid_ops;

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_CTRL_POSITION_PID_H */
