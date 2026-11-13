#ifndef __GPIO_IO_H__
#define __GPIO_IO_H__

#include "hc32_ll.h"

/*******************************************************************************
 * 通用GPIO操作宏
 ******************************************************************************/
#define GPIO_SET(port, pin)     GPIO_SetPins(port, pin)
#define GPIO_RESET(port, pin)   GPIO_ResetPins(port, pin)
#define GPIO_TOGGLE(port, pin)  GPIO_TogglePins(port, pin)
#define GPIO_READ(port, pin)    GPIO_ReadInputPins(port, pin)

/*******************************************************************************
 * 原有引脚定义（供应用层使用）
 ******************************************************************************/
#define PH2_PIN     GPIO_PIN_02
#define PH2_PORT    GPIO_PORT_H

#define PA3_PIN     GPIO_PIN_03
#define PA3_PORT    GPIO_PORT_A

/* 电平状态：PIN_RESET = 0 (低电平), PIN_SET = 1 (高电平)，见 hc32_ll_gpio.h */

/* GPIO输出初始状态枚举 */
typedef enum
{
    GPIO_INIT_LOW  = 0,   /* 初始化为低电平 */
    GPIO_INIT_HIGH = 1    /* 初始化为高电平 */
} en_gpio_init_state_t;

/*******************************************************************************
 * 通用GPIO初始化接口
 ******************************************************************************/

/* 通用GPIO输出初始化 */
void Output_GPIO_Init(uint8_t u8Port, uint16_t u16Pin, en_gpio_init_state_t u8InitState);

/* 通用GPIO输入初始化（enablePullUp: ENABLE=内部上拉, DISABLE=浮空） */
void Input_GPIO_Init(uint8_t u8Port, uint16_t u16Pin, en_functional_state_t enablePullUp);

/*******************************************************************************
 * 按键模块（PB 上拉输入，按下为低电平；主循环轮询消抖，仅短按）
 ******************************************************************************/

/* 按键引脚映射 */
#define KEY_SW1_PORT    GPIO_PORT_B
#define KEY_SW1_PIN     GPIO_PIN_01     /* PB1 -- SW1 */
#define KEY_SW2_PORT    GPIO_PORT_B
#define KEY_SW2_PIN     GPIO_PIN_02     /* PB2 -- SW2 */
#define KEY_SW3_PORT    GPIO_PORT_B
#define KEY_SW3_PIN     GPIO_PIN_00     /* PB0 -- SW3 */

/* 按键编号（与事件位图 bit 对应） */
typedef enum
{
    KEY_ID_SW1 = 0,     /* PB1, bit0 */
    KEY_ID_SW2,         /* PB2, bit1 */
    KEY_ID_SW3,         /* PB0, bit2 */
    KEY_ID_NUM
} key_id_t;

/* 短按事件位图（Keil Watch 可观察）：bit0=SW1, bit1=SW2, bit2=SW3，
 * 置位表示发生过一次完整"按下-松开"（已消抖），读取后需清除 */
extern volatile uint8_t g_key_short_evt;

/* 按键 GPIO 初始化（PB0/PB1/PB2 上拉输入） */
void Key_GPIO_Init(void);

/* 按键扫描 + 消抖，主循环周期调用（20ms 稳定判定，松开时触发短按事件） */
void Key_Scan(void);

/* 取出并清除指定按键的短按事件，1=有短按，0=无 */
uint8_t Key_GetShortPress(key_id_t key);

#endif /* __GPIO_IO_H__ */
