/**
 * @file    motor_fb.h
 * @brief   反馈层接口（传感器数据源）
 */

#ifndef __MOTOR_FB_H
#define __MOTOR_FB_H

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
 * 反馈层接口
 * ============================================================ */
typedef struct motor_fb_ops {
    rt_err_t (*init)(struct motor *self);
    void     (*update)(struct motor *self);
    int32_t  (*get_position)(struct motor *self);
    int32_t  (*get_speed)(struct motor *self);
    int32_t  (*get_current)(struct motor *self);
    uint8_t  (*get_fault)(struct motor *self);
    void     (*deinit)(struct motor *self);
}motor_fb_ops_t;

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_FB_H */
