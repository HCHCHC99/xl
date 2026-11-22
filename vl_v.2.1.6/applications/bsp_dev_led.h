/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-05-23     gylocal       the first version
 */
#ifndef APPLICATIONS_BSP_DEV_LED_H_
#define APPLICATIONS_DEV_LED_H_


#define DEV_LED_PIN      GET_PIN(H,2)      // led 引脚  高电平 点亮

typedef enum{
    LED_ID_SYS = 0,
    LED_ID_MAX
}LED_ID_E;

typedef struct{
    rt_base_t  pin;       /* pin 引脚 */
    rt_uint8_t mode;      /* pin 模式*/
    rt_uint8_t status;    /* pin 状态*/
}gpio_led_conf_t;

typedef enum
{
    LED_STATUS_IDLE    = 0x00,
    LED_STATUS_ON,
    LED_STATUS_OFF,
    LED_STATUS_BLINK_1HZ,
    LED_STATUS_BLINK_5HZ
}LED_STATUS_E;

void dev_led_init(void);
void dev_led_on(void);
void dev_led_off(void);

#endif /* APPLICATIONS_BSP_DEV_LED_H_ */
