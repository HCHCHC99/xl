/**
 * @file    bsp_adc.h
 * @brief   ADC BSP 层接口
 * @note    只提供硬件能力, 不感知业务语义
 *          实现 hw_adc_ops_t 接口表
 */

#ifndef __BSP_ADC_H
#define __BSP_ADC_H

#include <stdint.h>
#include <rtthread.h>
#include "../hw/hw_adc.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * BSP ADC 初始化
 * ============================================================ */

/**
 * @brief BSP ADC 初始化 [单线程]
 * @param self ADC 设备对象
 * @param cfg  配置参数（当前未使用，预留）
 * @return RT_EOK 成功
 */
rt_err_t bsp_adc_init(hw_adc_t *self, const void *cfg);

/**
 * @brief BSP ADC 反初始化 [单线程]
 */
rt_err_t bsp_adc_deinit(hw_adc_t *self);

/**
 * @brief 读取通道电压（mV） [线程安全]
 */
uint16_t bsp_adc_read_mv(hw_adc_t *self, uint8_t ch);

/**
 * @brief 读取通道原始值 [线程安全]
 */
uint16_t bsp_adc_read_raw(hw_adc_t *self, uint8_t ch);

/**
 * @brief BSP ADC 注册（创建全局实例）
 * @note  由初始化流程调用
 */
void bsp_adc_register(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_ADC_H */
