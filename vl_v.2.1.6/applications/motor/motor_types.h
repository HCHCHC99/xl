/**
 * @file    motor_types.h
 * @brief   电机配置类型定义
 * @note    定义六大配置维度的枚举类型
 */

#ifndef __MOTOR_TYPES_H
#define __MOTOR_TYPES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * D1：电机类型
 * ============================================================ */
typedef enum {
    MOTOR_TYPE_BDC = 0,         /* 有刷直流 */
    MOTOR_TYPE_BLDC,            /* 无刷直流（梯形反电动势） */
    MOTOR_TYPE_PMSM,            /* 永磁同步（正弦反电动势） */
    MOTOR_TYPE_STEPPER,         /* 步进 */
    MOTOR_TYPE_MAX
} motor_type_t;

/* ============================================================
 * D2：驱动方式
 * ============================================================ */
typedef enum {
    MOTOR_DRV_HW_IO = 0,        /* IO 驱动（H桥直接IO） */
    MOTOR_DRV_HW_PWM,           /* PWM 驱动（三相半桥） */
    MOTOR_DRV_HW_SPI_CHIP,      /* SPI 集成芯片 */
    MOTOR_DRV_HW_MAX
} motor_drv_hw_t;

/* ============================================================
 * D3：换向信号
 * ============================================================ */
typedef enum {
    MOTOR_COMM_NONE = 0,        /* 无换向信号（开环） */
    MOTOR_COMM_HALL,            /* 霍尔传感器 */
    MOTOR_COMM_ENCODER,         /* 增量编码器 */
    MOTOR_COMM_ABS_ENCODER,     /* 绝对编码器 */
    MOTOR_COMM_SENSORLESS,      /* 无传感器（BEMF） */
    MOTOR_COMM_MAX
} motor_comm_t;

/* ============================================================
 * D4：驱动算法
 * ============================================================ */
typedef enum {
    MOTOR_ALGO_OPENLOOP = 0,    /* 开环（固定占空比） */
    MOTOR_ALGO_SIX_STEP,        /* 六步换向 */
    MOTOR_ALGO_SVPWM,           /* SVPWM */
    MOTOR_ALGO_FOC,             /* FOC */
    MOTOR_ALGO_MAX
} motor_algo_t;

/* ============================================================
 * D5：工作模式
 * ============================================================ */
typedef enum {
    MOTOR_MODE_TORQUE = 0,      /* 恒转矩（目标电流） */
    MOTOR_MODE_SPEED,           /* 恒转速（目标速度） */
    MOTOR_MODE_POSITION,        /* 定位（目标位置） */
    MOTOR_MODE_MAX
} motor_work_mode_t;

/* ============================================================
 * 安全层：过流保护级别
 * ============================================================ */
typedef enum {
    MOTOR_OC_NONE = 0,          /* 正常 */
    MOTOR_OC_LEVEL1,            /* 一级过流（警告） */
    MOTOR_OC_LEVEL2,            /* 二级过流（降功率） */
    MOTOR_OC_LEVEL3,            /* 三级过流（紧急停机） */
    MOTOR_OC_MAX
} motor_oc_state_t;

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_TYPES_H */
