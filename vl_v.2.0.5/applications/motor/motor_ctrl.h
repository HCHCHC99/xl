/**
 * @file    motor_ctrl.h
 * @brief   控制层接口（闭环算法）
 */

#ifndef __MOTOR_CTRL_H
#define __MOTOR_CTRL_H

#include <stdint.h>
#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 前置声明
 * ============================================================ */
struct motor;

/* ============================================================
 * 控制层接口
 * ============================================================ */
typedef struct motor_ctrl_ops {
    rt_err_t (*init)(struct motor *self);
    void     (*reset)(struct motor *self);
    void     (*update)(struct motor *self, uint32_t dt_ms);
    void     (*set_target)(struct motor *self, int32_t target);
    void     (*stop)(struct motor *self);
    void     (*deinit)(struct motor *self);
}motor_ctrl_ops_t;

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_CTRL_H */
