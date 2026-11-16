
/**
 * @file    bsp_dev_adc.h
 * @brief   ADC BSP 层通用接口
 * @note    只提供硬件能力，不感知业务语义
 */

#ifndef __BSP_DEV_ADC_H
#define __BSP_DEV_ADC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 *      兼容旧版  开始
 * ============================================================ */
extern rt_adc_device_t    adc_dev_voltage;
extern rt_adc_device_t    adc_dev_current;


float32_t bsp_get_voltage_value(void);
float32_t bsp_get_current_value(void);

rt_int8_t bsp_dev_adc_init(void);
/* ============================================================
 *      兼容旧版  结束
 * ============================================================ */



/* ============================================================
 * ADC 通道枚举（与硬件接线相关）
 * ============================================================ */
typedef enum {
    ADC_CH_LIFT = 0,       /* 举升传感器通道 */
    ADC_CH_SWING,            /* 回转传感器通道（预留） */
    ADC_CH_CURRENT_A,       /* A相电流采样（预留） */
    ADC_CH_CURRENT_B,       /* B相电流采样（预留） */
    ADC_CH_MAX
} adc_channel_t;

/* ============================================================
 * 通用 API
 * ============================================================ */

/**
 * @brief 读取指定通道的 ADC 电压
 * @param channel 通道号
 * @return 电压值（mV）
 */
uint16_t bsp_adc_read_mv(adc_channel_t channel);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_DEV_ADC_H */
