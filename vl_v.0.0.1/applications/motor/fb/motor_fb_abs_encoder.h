/**
 * @file    motor_fb_abs_encoder.h
 * @brief   绝对编码器反馈（3霍尔融合）
 */

#ifndef __MOTOR_FB_ABS_ENCODER_H
#define __MOTOR_FB_ABS_ENCODER_H

#include "../motor.h"
#include "../motor_fb.h"

#ifdef __cplusplus
extern "C" {
#endif

///* ============================================================
// * 绝对编码器配置
// * ============================================================ */
//typedef struct {
//    /* ----- 3 个霍尔芯片的接口 ----- */
//    void     *hall_dev[3];
//    uint8_t   hall_channel[3];
//
//    /* ----- 融合参数 ----- */
//    int32_t   offset_x10;           /* 零点偏移（0.1°） */
//    int32_t   direction;            /* 方向（1 / -1） */
//    int32_t   full_scale_x10;       /* 满量程（0.1°），如 3600 表示 360° */
//} motor_abs_encoder_cfg_t;

/* ============================================================
 * 反馈实现（外部引用）
 * ============================================================ */
extern const motor_fb_ops_t motor_fb_abs_encoder_ops;

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_FB_ABS_ENCODER_H */
