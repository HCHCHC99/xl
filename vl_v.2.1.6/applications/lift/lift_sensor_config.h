/**
 * @file    lift_sensor_config.h
 * @brief   举升传感器配置表
 * @note    通过配置表驱动传感器类型、阈值、适配器参数
 */

#ifndef __LIFT_SENSOR_CONFIG_H
#define __LIFT_SENSOR_CONFIG_H

#include <stdint.h>
#include "lift_sensor.h"
#include "lift_adapter.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 传感器参数联合体
 * ============================================================ */
typedef union {
    /* ----- 电压型参数 ----- */
    struct {
        uint16_t (*read_adc_mv)(void *user_data);
        void    *user_data;
        uint16_t fault_high_mv;       /* 进入（高） */
        uint16_t fault_low_mv;        /* 进入（低） */
        /* 【P5 / B-6】迟滞退出阈值与两个确认时间（0 = 由 voltage_init 兜底） */
        uint16_t fault_high_exit_mv;  /* 恢复（高），典型 4400 */
        uint16_t fault_low_exit_mv;   /* 恢复（低），典型 520  */
        uint16_t confirm_ms;          /* 进入确认，典型 2000 */
        uint16_t recover_ms;          /* 恢复确认，典型 2000 */
    } voltage;

    /* ----- 电流型参数 ----- */
    struct {
        uint16_t (*read_adc_mv)(void *user_data);
        void    *user_data;
        uint16_t sample_resistor_ohm;
        uint16_t fault_high_ma10;
        uint16_t fault_low_ma10;
    } current;

    /* ----- 角度型参数 ----- */
    struct {
        int32_t (*read_angle_x10)(void *user_data);
        void    *user_data;
        uint16_t fault_code;
    } angle;

    /* ----- CAN 型参数 ----- */
    struct {
        void    *can_dev;
        uint32_t can_id;
        uint8_t  data_offset;
    } can;
} lift_sensor_params_t;

/* ============================================================
 * 传感器配置项
 * ============================================================ */
typedef struct lift_sensor_config {
    const char          *name;              /* 配置名称 */
    lift_sensor_type_t  type;              /* 传感器类型 */
    lift_sensor_params_t params;           /* 传感器参数 */
    lift_adapter_cfg_t  adapter;           /* 适配器参数 */
    uint32_t             update_period_ms;  /* 更新周期 */
} lift_sensor_config_t;

/* ============================================================
 * 全局配置表
 * ============================================================ */
extern const lift_sensor_config_t g_lift_sensor_configs[];
extern const uint32_t g_lift_sensor_config_count;

/* ============================================================
 * API
 * ============================================================ */

/**
 * @brief 按名称查找配置
 */
const lift_sensor_config_t* lift_sensor_config_find(const char *name);

/**
 * @brief 获取默认配置（第一个）
 */
const lift_sensor_config_t* lift_sensor_config_default(void);

#ifdef __cplusplus
}
#endif

#endif /* __LIFT_SENSOR_CONFIG_H */
