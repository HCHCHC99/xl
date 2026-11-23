/**
 * @file    lift_sensor.h
 * @brief   举升传感器抽象层
 * @note    支持多种传感器类型：电压型、电流型、角度型、CAN型
 *          通过 ops 接口实现类型无关的访问
 */

#ifndef __LIFT_SENSOR_H
#define __LIFT_SENSOR_H

#include <stdint.h>
#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 前置声明
 * ============================================================ */
typedef struct lift_sensor lift_sensor_t;

/* ============================================================
 * 传感器类型枚举
 * ============================================================ */
typedef enum {
    LIFT_SENSOR_TYPE_VOLTAGE = 0,  /* 电压型（500~4500mV） */
    LIFT_SENSOR_TYPE_CURRENT,      /* 电流型（4~20mA，0.1mA 精度） */
    LIFT_SENSOR_TYPE_ANGLE,        /* 角度型（直接输出 0.1°） */
    LIFT_SENSOR_TYPE_CAN,          /* CAN 型（从 CAN 报文获取） */
    LIFT_SENSOR_TYPE_MAX
} lift_sensor_type_t;

/* ============================================================
 * 传感器操作接口
 * ============================================================ */
typedef struct {
    rt_err_t (*init)(lift_sensor_t *self);
    void     (*update)(lift_sensor_t *self);
    int32_t  (*get_raw)(lift_sensor_t *self);
    uint8_t  (*get_fault)(lift_sensor_t *self);
    void     (*deinit)(lift_sensor_t *self);
} lift_sensor_ops_t;

/* ============================================================
 * 传感器对象
 * ============================================================ */
struct lift_sensor {
    /* ----- 身份标识 ----- */
    const char          *name;
    lift_sensor_type_t  type;

    /* ----- 接口 ----- */
    const lift_sensor_ops_t *ops;
    void                *data;

    /* ----- 运行时 ----- */
    int32_t              raw_value;
    uint8_t              fault;          /* 已确认的故障（带 2s 去抖） */
    uint8_t              fault_latched;  /* 同 fault，保留原名供诊断 */
    uint32_t             fault_elapsed_ms; /* 当前计时已持续（未计时时为 0） */
    uint8_t              fault_timing;   /* 1=正在计时（进入或恢复） */
    uint8_t              is_initialized;
};

/* ============================================================
 * 前向声明：传感器配置
 * ============================================================ */
struct lift_sensor_config;

/* ============================================================
 * 通用 API
 * ============================================================ */

/**
 * @brief 初始化传感器对象
 * @param sensor 传感器对象
 * @param name   传感器名称
 * @param type   传感器类型
 * @param ops    操作接口
 * @param data   私有数据
 * @return RT_EOK 成功
 */
rt_err_t lift_sensor_init(lift_sensor_t *sensor,
                           const char *name,
                           lift_sensor_type_t type,
                           const lift_sensor_ops_t *ops,
                           void *data);

/**
 * @brief 周期更新传感器原始值
 */
void lift_sensor_update(lift_sensor_t *sensor);

/**
 * @brief 获取原始值
 */
int32_t lift_sensor_get_raw(lift_sensor_t *sensor);

/**
 * @brief 获取故障状态
 */
uint8_t lift_sensor_get_fault(lift_sensor_t *sensor);

/**
 * @brief 从配置表初始化传感器
 */
rt_err_t lift_sensor_init_from_config(lift_sensor_t *sensor,
                                        const struct lift_sensor_config *config);

/* ============================================================
 * 电压型传感器接口
 * ============================================================ */

/**
 * @brief 电压型传感器配置
 * @note  raw_value 单位：mV，典型范围 500~4500mV
 *        【P5 / B-6】进/出各一对阈值 + 两个确认时间，构成带迟滞的去抖：
 *          未故障时 mv > hi_enter 或 mv < lo_enter 连续 confirm_ms → 置故障；
 *          已故障时 mv 回到 [lo_exit, hi_exit] 连续 recover_ms → 清故障。
 */
typedef struct {
    /* ----- 配置项（初始化后只读） ----- */
    uint16_t (*read_adc_mv)(void *user_data);  /* ADC 读取函数 */
    void    *user_data;
    uint16_t fault_high_mv;                     /* 进入阈值（高） */
    uint16_t fault_low_mv;                      /* 进入阈值（低） */

    /* ----- P5 新增：迟滞退出阈值与确认时间 ----- */
    uint16_t fault_high_exit_mv;                /* 恢复阈值（高），默认 4400 */
    uint16_t fault_low_exit_mv;                 /* 恢复阈值（低），默认 520  */
    uint16_t confirm_ms;                        /* 进入确认时间，默认 2000 */
    uint16_t recover_ms;                        /* 恢复确认时间，默认 2000 */

    /* ----- 运行时状态（内部维护） ----- */
    uint16_t last_voltage_mv;      /* 上次采样电压 */
    uint32_t timer_tick;           /* 计时起点（rt_tick） */
    uint8_t  timer_active;         /* 1=正在计时 */
} lift_sensor_voltage_cfg_t;

extern const lift_sensor_ops_t lift_sensor_voltage_ops;

/* ============================================================
 * 电流型传感器接口
 * ============================================================ */

/**
 * @brief 电流型传感器配置
 * @note  raw_value 单位：0.1mA（如 4mA = 40, 20mA = 200）
 *        通过采样电阻将电流转为电压：I(0.1mA) = V(mV) * 10 / R(Ω)
 */
typedef struct {
    uint16_t (*read_adc_mv)(void *user_data);  /* ADC 读取函数 */
    void    *user_data;
    uint16_t sample_resistor_ohm;               /* 采样电阻 */
    uint16_t fault_high_ma10;                   /* 过流阈值 */
    uint16_t fault_low_ma10;                    /* 欠流阈值 */
} lift_sensor_current_cfg_t;

extern const lift_sensor_ops_t lift_sensor_current_ops;

/* ============================================================
 * 角度型 / CAN 型（预留）
 * ============================================================ */
extern const lift_sensor_ops_t lift_sensor_angle_ops;
extern const lift_sensor_ops_t lift_sensor_can_ops;

#ifdef __cplusplus
}
#endif

#endif /* __LIFT_SENSOR_H */
