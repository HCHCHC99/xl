#ifndef __HARDWARE_H__
#define __HARDWARE_H__

#ifdef __cplusplus
extern "C" {
#endif

/* 包含 ADP 层的模块头文件 */
#include "Adapter.h"

/* 硬件初始化函数声明 */
void Hardware_Init(void);

/*==============================================================================
 * 母线电压检测（PA4 / ADC12_IN4 → ADC1_CH4）
 *------------------------------------------------------------------------------
 * 硬件：Vbus ──[R_HIGH]──┬── PA4
 *                        └──[R_LOW ]── GND
 *   Vadc = Vbus × R_LOW / (R_HIGH + R_LOW)
 *   20k/3k 分压：Vadc = Vbus × 3/23 = 0.1304 × Vbus，满量程可测约 25.3V
 *
 * 采样方式：软件触发单次转换（不占用 TMR0 / AOS / DMA，也不影响 20kHz 电流采样）
 *   1) Hardware_Init() 内调用 Vbus_Adc_Init() 完成一次性初始化
 *   2) main loop 内周期性调用 Vbus_Adc_Process() 刷新 g_vbus_mv / g_vbus_v
 *
 * 注意：不要改走 Adp/Adc.c 的框架 —— Adc_Init() 会把 CM_TMR0_1 的 CH_B 重配为
 *       1ms，而那正是 Hardware_Init 里配好的 500us 系统节拍。
 *============================================================================*/

/* PA4 在 ADC1 上默认映射为 ADC_CH4（见 hc32_ll_adc.h 的 ADC1_PIN_PA4） */
#define VBUS_ADC_UNIT           (CM_ADC1)
#define VBUS_ADC_PERIPH_CLK     (FCG3_PERIPH_ADC1)
#define VBUS_ADC_CHANNEL        (ADC_CH4)
#define VBUS_ADC_PORT           (GPIO_PORT_A)
#define VBUS_ADC_PIN            (GPIO_PIN_04)

/* 分压电阻（欧姆）：Vbus 侧 R_HIGH，地侧 R_LOW */
#define VBUS_DIV_R_HIGH_OHM     (20000.0F)
#define VBUS_DIV_R_LOW_OHM      (3000.0F)

/* ADC 参考电压与满量程码值（12bit） */
#define VBUS_ADC_VREF           (3.3F)
#define VBUS_ADC_FULL_SCALE     (4096.0F)

#define VBUS_ADC_SAMPLE_TIME    (255U)      /* 采样窗（ADCLK 周期数），高阻分压取最大窗 */
#define VBUS_ADC_TIMEOUT        (100000UL)  /* 等待转换完成的轮询上限，防止死等 */
#define VBUS_UPDATE_MS          (100U)      /* 刷新周期（ms） */
#define VBUS_FILT_ALPHA         (0.25F)     /* EMA 滤波系数，首拍直通 */

/* Debug switch: 1 = RTT prints on, 0 = off */
#define DEBUG_VBUS   0
#if DEBUG_VBUS
    #define VBUS_DBG(fmt, ...)    MAIN_D("[VBUS] " fmt, ##__VA_ARGS__)
#else
    #define VBUS_DBG(fmt, ...)    ((void)0)
#endif

/* 最新母线电压（Keil Watch 可观察） */
extern volatile uint16_t g_vbus_raw;    /* PA4 原始 ADC 码值（0~4095，未滤波） */
extern volatile uint16_t g_vbus_mv;     /* 单位 mV */
extern volatile float    g_vbus_v;      /* 单位 V */

/* 母线电压检测 API */
void Vbus_Adc_Init(void);               /* 在 Hardware_Init 中调用 */
void Vbus_Adc_Process(void);            /* 在 main loop 中周期性调用 */

/* PA6 ADC 数据处理函数 - 在 main loop 中周期性调用 */
void Pa6_Adc_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* __HARDWARE_H__ */
