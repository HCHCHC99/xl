/**
 * @file    foc_eoca_probe.h
 * @brief   ADC1 EOCA 观测探针：PA0 每次 EOCA 翻转一次
 * @note    用于验证 20kHz 采样/转换链路是否正常，可随时移除。
 */

#ifndef __FOC_EOCA_PROBE_H__
#define __FOC_EOCA_PROBE_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  初始化 PA0 为推挽输出，注册并使能 ADC1 EOCA 中断
 * @retval 0 成功；-1 中断注册失败
 * @note   必须在 ADC1 / DMA 初始化完成后调用。
 */
int foc_eoca_probe_init(void);

/**
 * @brief  读取最近一次 EOCA 中断入口时的 TMR4 计数值
 * @retval TMR4 CNT（0..period-1）
 * @note   用于估算 ADC 转换完成（EOCA）相对采样触发点的延迟；
 *         包含中断入口延迟，不是纯硬件 EOCA 时刻。
 */
uint16_t foc_eoca_probe_get_cnt(void);

#ifdef __cplusplus
}
#endif

#endif /* __FOC_EOCA_PROBE_H__ */
