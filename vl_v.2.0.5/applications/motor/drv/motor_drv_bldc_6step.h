/**
 * @file    motor_drv_bldc_6step.h
 * @brief   BLDC 六步换相驱动接口
 */

#ifndef __MOTOR_DRV_BLDC_6STEP_H
#define __MOTOR_DRV_BLDC_6STEP_H

#include "../motor.h"
#include "../motor_drv.h"
#include "../motor_config.h"

#ifdef __cplusplus
extern "C" {
#endif

///* ============================================================
// * BLDC 硬件配置
// * ============================================================ */
//typedef struct {
//    /* ----- 三相 PWM ----- */
//    void     *pwm_dev;
//    uint32_t  pwm_freq_hz;          /* PWM 频率（如 20000） */
//    uint32_t  pwm_deadtime_ns;      /* 死区时间（ns） */
//    uint32_t  max_duty;             /* 最大占空比（如 1000） */
//
//    /* ----- 霍尔传感器（3路） ----- */
//    void     *hall_port[3];
//    uint16_t  hall_pin[3];
//    uint8_t   hall_active_level;    /* 有效电平 */
//
//    /* ----- 隔离电源使能 ----- */
//    void     *enable_port;
//    uint16_t  enable_pin;
//    uint8_t   enable_active_level;  /* 0=低有效，1=高有效 */
//
//    /* ----- 电机参数 ----- */
//    uint8_t   pole_pairs;           /* 极对数（如 4） */
//} motor_bldc_6step_cfg_t;
//

/* ============================================================
 * 驱动实现（外部引用）
 * ============================================================ */
/* 驱动配置使用 motor_pwm_cfg_t（在 motor_config.h 中定义） */
/* 换向配置使用 motor_hall_cfg_t（在 motor_config.h 中定义） */
extern const motor_drv_ops_t motor_drv_bldc_6step_ops;


/**
 * @brief 霍尔中断处理函数（由 GPIO 中断调用）
 */
void motor_bldc_6step_hall_isr(motor_t *motor);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_DRV_BLDC_6STEP_H */
