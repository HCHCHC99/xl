#ifndef __ADAPTER_H__
#define __ADAPTER_H__

/* C++兼容 */
#ifdef __cplusplus
extern "C" {
#endif

/*==============================================================================
 * 包含所有硬件适配模块
 *============================================================================*/

/* 基础硬件抽象 - 芯片外设适配 */
#include "Gpio_io.h"     /* 硬件初始化入口 */
#include "Sysclk.h"      /* SysTick时钟适配 */
#include "Timer0_Unit2.h"       /* Timer0周期中断适配 */
#include "Timer0_Unit1.h"
#include "Template_Pwm.h"
#include "Pwm.h"
#include "hc32_ll.h"
#include "Aos.h"
#include "Adc.h"
#include "Motor_hall.h"
#ifdef __cplusplus
}
#endif

#endif /* __ADAPTER_H__ */
