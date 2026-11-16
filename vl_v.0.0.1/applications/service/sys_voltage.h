/**
 * @file    sys_voltage.h
 * @brief   系统（母线）电压采集
 *
 * 【硬件】
 *   AD_VM → PA6 → ADC12_IN6 → ADC2 的 ADC_CH2
 *   分压电阻：上臂 100kΩ / 下臂 3.3kΩ
 *   Vbus = Vpin × (100 + 3.3) / 3.3 = Vpin × 31.03
 *   3.3V 参考下最大可测 ≈ 102.3V，覆盖 12V/24V 系统
 *
 * 【注意】ADC12_IN6 对应的 ADC2 通道号是 2，不是 6！
 *   hc32_ll_adc.h:398  ADC2_PIN_PA6 = (2U)
 *   board_config.h 中的宏名 ADC2_CH6_* 沿用 IN 编号，切勿当作通道号
 *
 * 【与 lift_adc 的关系】
 *   同用 "adc2" 设备、不同通道（2 vs 3），rt_adc_enable 可共存。
 */

#ifndef __SYS_VOLTAGE_H
#define __SYS_VOLTAGE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 硬件参数
 * ============================================================ */
#define SYS_VOLT_ADC_DEV_NAME       "adc2"
#define SYS_VOLT_ADC_CHANNEL        2U      /* ADC12_IN6(PA6) = ADC2 CH2 */
#define SYS_VOLT_ADC_RAW_MAX        4095U   /* 12bit 满量程 */
#define SYS_VOLT_ADC_FULL_SCALE     4096U   /* 12bit 分频基数 */
#define SYS_VOLT_ADC_VREF_MV        3300U   /* VDDA = 3.3V */
#define SYS_VOLT_ADC_SAMPLE_CLK     40U     /* 采样时间（ADCLK 个数），适配高阻分压源 */
#define SYS_VOLT_ADC_OVERSAMPLE     8U      /* 过采样次数（算术平均） */

/**
 * @brief 分压还原系数
 * @note  R_high = 100kΩ，R_low = 3.3kΩ
 *        mv_bus = mv_pin × SYS_VOLT_DIV_NUM / SYS_VOLT_DIV_DEN
 *               = mv_pin × 1033 / 33  ≈ mv_pin × 31.30
 */
#define SYS_VOLT_DIV_NUM            1033U
#define SYS_VOLT_DIV_DEN            33U

/* ============================================================
 * API
 * ============================================================ */

/**
 * @brief 初始化：查找 adc2 设备、设置采样时间、使能通道 [单线程]
 * @return 0 成功；-1 失败（设备缺失或使能失败）
 */
int sys_voltage_init(void);

/**
 * @brief 读取引脚电压（mV，含过采样，未还原分压） [线程安全]
 */
uint16_t sys_voltage_read_pin_mv(void);

/**
 * @brief 读取母线电压（mV，已还原分压） [线程安全]
 */
uint32_t sys_voltage_read_bus_mv(void);

/**
 * @brief 读取母线电压（0.1V 单位，例如 13.0V → 130） [线程安全]
 * @note  供启车判定与状态帧上报使用
 */
uint16_t sys_voltage_get_dv(void);

/**
 * @brief 采集通道是否就绪
 * @return 1 = 可用；0 = 不可用（设备缺失或初始化失败）
 */
uint8_t sys_voltage_is_ready(void);

#ifdef __cplusplus
}
#endif

#endif /* __SYS_VOLTAGE_H */
