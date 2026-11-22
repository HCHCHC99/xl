/**
 * @file    lift_angle.h
 * @brief   举升角对象（独立模块）
 * @note    职责：角度计算、位置判断、事件投递
 *          通过函数指针获取电压值，与传感器实现解耦
 *          支持运行时绑定不同的传感器
 */

#ifndef __LIFT_ANGLE_H
#define __LIFT_ANGLE_H

#include <stdint.h>
#include "lift_sensor.h"
#include "lift_adapter.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 位置枚举
 * ============================================================ */
typedef enum {
    LIFT_POS_UNKNOWN = 0,      /* 未知（未初始化或传感器故障） */
    LIFT_POS_UPPER_LIMIT,      /* 上限位附近 (+43° ±2°) */
    LIFT_POS_MIDDLE,           /* 中间位（上下限之间） */
    LIFT_POS_LOWER_LIMIT       /* 下限位附近 (-30° ±2°) */
} lift_position_t;

///* ============================================================
// * 电压获取函数类型（依赖注入接口）
// * @param user_data 用户数据指针（通常指向传感器对象）
// * @return 当前电压值 (mV)
// * ============================================================ */
//typedef uint16_t (*lift_voltage_getter_t)(void *user_data);

/* ============================================================
 * 位置变化回调函数类型
 * @param old_pos 旧位置
 * @param new_pos 新位置
 * ============================================================ */
typedef void (*lift_position_change_cb_t)(lift_position_t old_pos, lift_position_t new_pos);

/* ============================================================
 * 举升角对象结构体
 * ============================================================ */
typedef struct {
    /* ----- 传感器绑定 ----- */
    lift_sensor_t       *sensor;
    lift_adapter_cfg_t   adapter;

    /* ----- 物理量（计算结果） ----- */
    int32_t  current_angle_x10;      /* 当前角度 (0.1°) */
    int32_t  current_raw_value;

//    /* ----- 零位校准 ----- */
//    uint16_t zero_voltage_mv;        /* 零位电压 (mV) */
    uint8_t  is_calibrated;          /* 是否已校准 */

    /* ----- 位置状态 ----- */
    lift_position_t position;       /* 当前位置 */
    lift_position_t last_position;  /* 上一次位置（用于变化检测） */
    uint8_t  is_at_upper_limit;      /* 1=在上限位范围内 */
    uint8_t  is_at_lower_limit;      /* 1=在下限位范围内 */
    uint8_t  is_at_middle;           /* 1=在中间位（非极限） */

    /* ----- 限位常量 ----- */
    int32_t upper_limit_x10;         /* +430 (+43.0°) */
    int32_t lower_limit_x10;         /* -300 (-30.0°) */
//    int32_t middle_center_x10;       /* 0 (0.0°) */
    int32_t limit_tolerance_x10;     /* ±20 (±2°) */
//    int32_t middle_tolerance_x10;    /* ±5 (±0.5°) */

//    /* ----- 依赖注入：传感器适配 ----- */
//    lift_voltage_getter_t get_voltage;   /* 电压获取函数指针 */
//    void *user_data;                      /* 用户数据（指向传感器对象） */

    /* ----- 事件回调 ----- */
    lift_position_change_cb_t on_position_change;  /* 位置变化回调 */

    /* ----- 状态标志 ----- */
    uint8_t is_initialized;              /* 1=已初始化 */
} lift_angle_t;



///* ============================================================
// * 全局举升角对象（由 lift_angle.c 内部定义，供外部引用）
// * ============================================================ */
//extern lift_angle_t g_lift_angle;


/* ============================================================
 * API
 * ============================================================ */

/**
 * @brief 初始化举升角对象
 */
void lift_angle_init(lift_angle_t *angle, lift_sensor_t *sensor);

/**
* @brief 绑定/切换传感器
*/
void lift_angle_bind_sensor(lift_angle_t *angle, lift_sensor_t *sensor);

/**
* @brief 设置转换参数
*/
void lift_angle_set_adapter(lift_angle_t *angle, const lift_adapter_cfg_t *cfg);

/**
* @brief 周期更新（读取传感器 + 转换 + 位置判断）
*/
void lift_angle_update(lift_angle_t *angle);

/**
* @brief 标定零位
*/
void lift_angle_calibrate(lift_angle_t *angle, int32_t raw_zero);

/* ----- 查询接口 ----- */
lift_position_t lift_angle_get_position(const lift_angle_t *angle);
uint8_t lift_angle_is_upper_limit(const lift_angle_t *angle);
uint8_t lift_angle_is_lower_limit(const lift_angle_t *angle);
uint8_t lift_angle_is_middle(const lift_angle_t *angle);
int32_t lift_angle_get_value(const lift_angle_t *angle);
uint8_t lift_angle_is_safe(const lift_angle_t *angle);
uint8_t lift_angle_get_fault(const lift_angle_t *angle);

/* ----- 全局对象 ----- */
extern lift_angle_t g_lift_angle;
lift_angle_t* lift_angle_get(void);

#ifdef __cplusplus
}
#endif

#endif /* __LIFT_ANGLE_H */
