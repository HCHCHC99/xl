/**
 * @file    hw_adc.h
 * @brief   ADC 设备标准接口
 * @note    提供统一 ADC 访问接口, 屏蔽底层 BSP 差异
 *          应用层通过 hw_adc_t 对象访问 ADC
 */

#ifndef __HW_ADC_H
#define __HW_ADC_H

#include <stdint.h>
#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 前置声明
 * ============================================================ */
typedef struct hw_adc hw_adc_t;

/* ============================================================
 * 通道枚举
 * ============================================================ */
typedef enum {
    HW_ADC_CH_VOLTAGE = 0,      /* 电压采样通道 */
    HW_ADC_CH_CURRENT,          /* 电流采样通道 */
    HW_ADC_CH_MAX
} hw_adc_channel_t;

/* ============================================================
 * 设备操作接口
 * ============================================================ */
typedef struct {
    rt_err_t (*init)(hw_adc_t *self, const void *cfg);
    rt_err_t (*deinit)(hw_adc_t *self);
    uint16_t (*read_mv)(hw_adc_t *self, uint8_t ch);
    uint16_t (*read_raw)(hw_adc_t *self, uint8_t ch);
} hw_adc_ops_t;

/* ============================================================
 * 设备对象
 * ============================================================ */
struct hw_adc {
    const char        *name;
    const hw_adc_ops_t *ops;
    void              *priv;
    uint8_t            is_ready;
};

/* ============================================================
 * 标准 API
 * ============================================================ */

/**
 * @brief 初始化 ADC 设备 [单线程]
 */
rt_err_t hw_adc_init(hw_adc_t *self,
                     const void *cfg,
                     const hw_adc_ops_t *ops);

/**
 * @brief 反初始化 ADC 设备 [单线程]
 */
rt_err_t hw_adc_deinit(hw_adc_t *self);

/**
 * @brief 读取指定通道电压（mV） [线程安全]
 */
uint16_t hw_adc_read_mv(hw_adc_t *self, uint8_t ch);

/**
 * @brief 读取指定通道原始值 [线程安全]
 */
uint16_t hw_adc_read_raw(hw_adc_t *self, uint8_t ch);

/* ============================================================
 * 全局 ADC 实例（由 BSP 层注册）
 * ============================================================ */
extern hw_adc_t g_hw_adc;

#ifdef __cplusplus
}
#endif

#endif /* __HW_ADC_H */
