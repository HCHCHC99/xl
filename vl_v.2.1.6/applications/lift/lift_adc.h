/**
 * @file    lift_adc.h
 * @brief   举升角 ADC 采集（电压型传感器）
 *
 * 【硬件】
 *   OUT_SENSORR_IN → PA7 → ADC12_IN7 → ADC2 的 ADC_CH3
 *   采样电压 1:1 分压（即不分压）
 *   设备名 "adc2"（board/config/adc_config.h:49）
 *   驱动由 INIT_DEVICE_EXPORT(rt_hw_adc_init) 自动注册（drv_adc.c:270）
 *
 * 【注意】ADC12_IN7 对应的 ADC2 通道号是 3，不是 7！
 *   hc32_ll_adc.h:399  ADC2_PIN_PA7 = (3U)
 *   board_config.h 中的宏名 ADC2_CH7_* 沿用 IN 编号，切勿当作通道号
 *
 * 【不使用 hw_adc / bsp_adc 的原因】
 *   bsp_adc_read_mv() 是恒返回 2203 的桩实现（bsp_adc.c:73），
 *   且 drv_adc 未实现 get_resolution/get_vref，rt_adc_voltage() 恒返回 0，
 *   故此处直接基于 RT-Thread ADC 框架自行换算。
 */
#ifndef __LIFT_ADC_H
#define __LIFT_ADC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 台架模拟开关（任务清单 §11）
 * ------------------------------------------------------------
 * 1 = 允许 t_sim lift <mV> 覆盖传感器电压（举升角/上下限位/传感器
 *     故障阈值一并受影响，2203mV = 0°、36.4mV/°）
 * 0 = 关闭，只读真实 PA7
 * 总开关 INPUT_SIM_ENABLE 在 service/input_sim.h
 * ============================================================ */
#ifndef LIFT_ADC_SIM_ENABLE
#define LIFT_ADC_SIM_ENABLE 1
#endif

/* ============================================================
 * 硬件参数
 * ============================================================ */
#define LIFT_ADC_DEV_NAME    "adc2"    /* adc_config.h:49 */
#define LIFT_ADC_CHANNEL     3U        /* ADC12_IN7(PA7) = ADC2 CH3 */
#define LIFT_ADC_RAW_MAX     4095U     /* 12bit 满量程 */
#define LIFT_ADC_FULL_SCALE  4096U     /* 12bit 分频基数 */

/* 传感器量程（用于校核与限位判断） */
#define LIFT_ADC_SIG_MIN_MV  500U      /* 传感器输出下限 */
#define LIFT_ADC_SIG_MAX_MV  4500U     /* 传感器输出上限 */
#define LIFT_ADC_DEFAULT_ZERO_MV   2203U

/**
 * @brief 参考电压（mV）
 * @note  VDDA = 3.3V。由硬件设计反证：
 *        全量程经 ÷2 后上限 2250mV < 3300mV，正是为适配 3.3V 参考
 */
#define LIFT_ADC_VREF_MV     3300U

/**
 * @brief 硬件分压比 —— 等值双电阻取中点（÷2）
 * @note  传感器 500~4500mV → 引脚 250~2250mV
 *        mv_signal = mv_pin × DIV_DEN / DIV_NUM
 */
#define LIFT_ADC_DIV_NUM     1U
#define LIFT_ADC_DIV_DEN     2U

/**
 * @brief 过采样次数（算术平均，抑制噪声）
 * @note  1 = 关闭；建议 4~16
 */
#define LIFT_ADC_OVERSAMPLE  8U

/* ============================================================
 * API
 * ============================================================ */

/**
 * @brief 初始化：查找设备 + 使能通道 [单线程]
 * @return 0 成功，-1 失败
 */
int lift_adc_init(void);

/**
 * @brief 读取原始值（单次，无滤波） [线程安全]
 */
uint16_t lift_adc_read_raw(void);

/**
 * @brief 读取电压（mV，含过采样与分压还原） [线程安全]
 * @param [in] user_data 未使用（单通道），仅为匹配传感器配置表签名
 */
uint16_t lift_adc_read_mv(void *user_data);

#ifdef __cplusplus
}
#endif
#endif /* __LIFT_ADC_H */
