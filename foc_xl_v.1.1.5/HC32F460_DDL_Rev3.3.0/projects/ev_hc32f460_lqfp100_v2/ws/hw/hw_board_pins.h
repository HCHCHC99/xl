/*=============================================================================
 * hw_board_pins.h  —  所有硬件引脚/通道/功能号集中定义
 *
 * 换板子换引脚换 ADC 通道换定时器单元，只改这一个文件
 *=============================================================================*/

#ifndef HW_BOARD_PINS_H
#define HW_BOARD_PINS_H

#include "hc32_ll.h"
#include "cfg_sampling.h"

/*=============================================================================
 * PWM 输出（TMR4_3 六路）
 *   PB9 = OUH, PB8 = OUL
 *   PB7 = OVH, PB6 = OVL
 *   PB5 = OWH, PB4 = OWL
 *=============================================================================*/
#define HW_PIN_PWM_UH_PORT       GPIO_PORT_B
#define HW_PIN_PWM_UH_PIN        GPIO_PIN_09
#define HW_PIN_PWM_UH_FUNC       GPIO_FUNC_2

#define HW_PIN_PWM_UL_PORT       GPIO_PORT_B
#define HW_PIN_PWM_UL_PIN        GPIO_PIN_08
#define HW_PIN_PWM_UL_FUNC       GPIO_FUNC_2

#define HW_PIN_PWM_VH_PORT       GPIO_PORT_B
#define HW_PIN_PWM_VH_PIN        GPIO_PIN_07
#define HW_PIN_PWM_VH_FUNC       GPIO_FUNC_2

#define HW_PIN_PWM_VL_PORT       GPIO_PORT_B
#define HW_PIN_PWM_VL_PIN        GPIO_PIN_06
#define HW_PIN_PWM_VL_FUNC       GPIO_FUNC_2

#define HW_PIN_PWM_WH_PORT       GPIO_PORT_B
#define HW_PIN_PWM_WH_PIN        GPIO_PIN_05
#define HW_PIN_PWM_WH_FUNC       GPIO_FUNC_2

#define HW_PIN_PWM_WL_PORT       GPIO_PORT_B
#define HW_PIN_PWM_WL_PIN        GPIO_PIN_04
#define HW_PIN_PWM_WL_FUNC       GPIO_FUNC_2

/*=============================================================================
 * ADC 采样（ADC1 序列A）
 *   PA3 = CH3 (I3)      母线电流 itotal 或 U 相电流 iu（由 CFG_SAMPLING_BOARD_IU_SENSOR 决定）
 *   PA4 = CH4 (iw)      W 相电流
 *   PA5 = CH5 (iv)      V 相电流
 *   PA6 = CH6 (vbus)    母线电压
 *
 *   U 相电流：
 *     CFG_SAMPLING_BOARD_IU_SENSOR=0  PA3 为母线电流，iu = 0 - iw - iv
 *     CFG_SAMPLING_BOARD_IU_SENSOR=1  PA3 直接采样 iu 并使用
 *     CFG_SAMPLING_BOARD_IU_SENSOR=2  PA3 仍采样但只观测，iu = 0 - iw - iv
 *=============================================================================*/
#define HW_PIN_ADC_I3_PORT       GPIO_PORT_A
#define HW_PIN_ADC_I3_PIN        GPIO_PIN_03
#define HW_ADC_CH_I3             ADC_CH3

#if (CFG_SAMPLING_BOARD_IU_SENSOR != 0)
#define HW_ADC_CH_IU             HW_ADC_CH_I3
#else
#define HW_ADC_CH_ITOTAL         HW_ADC_CH_I3
#endif

#define HW_PIN_ADC_IW_PORT       GPIO_PORT_A
#define HW_PIN_ADC_IW_PIN        GPIO_PIN_04
#define HW_ADC_CH_IW             ADC_CH4

#define HW_PIN_ADC_IV_PORT       GPIO_PORT_A
#define HW_PIN_ADC_IV_PIN        GPIO_PIN_05
#define HW_ADC_CH_IV             ADC_CH5

#define HW_PIN_ADC_VBUS_PORT     GPIO_PORT_A
#define HW_PIN_ADC_VBUS_PIN      GPIO_PIN_06
#define HW_ADC_CH_VBUS           ADC_CH6

/*=============================================================================
 * 编码器（TMRA_1 硬件正交 + Z 索引）
 *   PA8 = A 相（TIMA1_CLKA）
 *   PA9 = B 相（TIMA1_CLKB）
 *   PA10 = Z 索引（EXTINT_CH10）
 *=============================================================================*/
#define HW_PIN_ENC_A_PORT        GPIO_PORT_A
#define HW_PIN_ENC_A_PIN         GPIO_PIN_08
#define HW_PIN_ENC_A_FUNC        GPIO_FUNC_4

#define HW_PIN_ENC_B_PORT        GPIO_PORT_A
#define HW_PIN_ENC_B_PIN         GPIO_PIN_09
#define HW_PIN_ENC_B_FUNC        GPIO_FUNC_4

#define HW_PIN_ENC_Z_PORT        GPIO_PORT_A
#define HW_PIN_ENC_Z_PIN         GPIO_PIN_10
#define HW_ENC_Z_EIRQ            EXTINT_CH10
#define HW_ENC_Z_IRQn            INT010_IRQn
#define HW_ENC_Z_IRQ_SRC         INT_SRC_PORT_EIRQ10

/*=============================================================================
 * 调试指示
 *   PC15 = 推挽输出（预留，当前 Foc_Control_Step 内未实际翻转，
 *          可用于示波器观测 20kHz 节拍）
 *=============================================================================*/
#define HW_PIN_DEBUG_TOGGLE_PORT GPIO_PORT_C
#define HW_PIN_DEBUG_TOGGLE_PIN  GPIO_PIN_15

#endif /* HW_BOARD_PINS_H */
