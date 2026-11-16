/**
 * @file    lift_adapter.h
 * @brief   举升原始值 → 角度转换适配器
 * @note    支持不同传感器类型的转换参数配置
 */

#ifndef __LIFT_ADAPTER_H
#define __LIFT_ADAPTER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LIFT_MV_PER_25DEG       910     /* 客户约定值，勿改 */
#define LIFT_ADAPTER_SCALE_NUM  100     /* 分子（0.1° 精度） */
#define LIFT_ADAPTER_SCALE_DEN  364     /* 分母 = 36.4mV/° × 10 */

/* ============================================================
 * 适配器配置
 * ============================================================ */
typedef struct {
    int32_t zero_raw;       /* 零位原始值 */
    int32_t scale_num;      /* 比例分子 */
    int32_t scale_den;      /* 比例分母 */
    int32_t min_angle_x10;  /* 最小角度（限幅） */
    int32_t max_angle_x10;  /* 最大角度（限幅） */
} lift_adapter_cfg_t;

/* ============================================================
 * 转换 API
 * ============================================================ */

/**
 * @brief 原始值 → 角度（0.1°）
 * @note  公式: angle = (raw - zero_raw) * scale_num / scale_den
 */
int32_t lift_adapter_to_angle_x10(int32_t raw, const lift_adapter_cfg_t *cfg);

/**
 * @brief 角度（0.1°）→ 原始值
 */
int32_t lift_adapter_to_raw(int32_t angle_x10, const lift_adapter_cfg_t *cfg);

/* ============================================================
 * 预设配置函数
 * ============================================================ */

/**
 * @brief 配置电压型适配器
 * @param cfg     输出配置
 * @param zero_mv 零位电压（如 2203mV）
 * @note  电压型: 36.4mV = 1° = 10 (0.1°)
 */
void lift_adapter_config_voltage(lift_adapter_cfg_t *cfg, int32_t zero_mv);

/**
 * @brief 配置电流型适配器
 * @param cfg       输出配置
 * @param zero_ma10 零位电流（0.1mA）
 * @param span_ma10 满量程电流跨度（0.1mA）
 * @param span_deg  满量程对应的角度（°）
 */
void lift_adapter_config_current(lift_adapter_cfg_t *cfg,
                                   int32_t zero_ma10,
                                   int32_t span_ma10,
                                   int32_t span_deg);

/**
 * @brief 配置角度型适配器（直通）
 */
void lift_adapter_config_angle(lift_adapter_cfg_t *cfg, int32_t zero_x10);

#ifdef __cplusplus
}
#endif

#endif /* __LIFT_ADAPTER_H */
