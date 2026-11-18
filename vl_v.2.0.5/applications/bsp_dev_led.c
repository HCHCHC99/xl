/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-05-23     gylocal       the first version
 */


#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <bsp_dev_led.h>


#include "hc32_ll_def.h"

#define FLAG_STATUS en_flag_status_t

static  gpio_led_conf_t user_led_list[LED_ID_MAX] = {
    [LED_ID_SYS]  ={GET_PIN(H,2)  ,PIN_MODE_OUTPUT_OD,PIN_HIGH}
};

typedef struct
{
    rt_timer_t       timer_led;         /* led 定时器*/
    rt_uint32_t      on_time;           /* led 点亮时间*/
    rt_uint32_t      off_time;          /* led 熄灭时间*/
    rt_uint32_t      count;             /* led 定时器时间片 计数*/
    FLAG_STATUS      stat_on;           /* led 点亮 电平  SET 高电平点亮、RESET 低电平点亮*/
}dev_led_context_t;

static dev_led_context_t dev_led_context[LED_ID_MAX];


static void dev_led_set(rt_uint8_t led_id,FLAG_STATUS sta)
{
    if(led_id < LED_ID_MAX)
    {
        if(RESET == dev_led_context[led_id].stat_on)
        {
            //pin low == on
            if(RESET == sta)
            {
                rt_pin_write(user_led_list[led_id].pin,PIN_LOW);
            }else{
                rt_pin_write(user_led_list[led_id].pin,PIN_HIGH);
            }
        }
        else
        {
            //pin high == on
            if(RESET == sta)
            {
                rt_pin_write(user_led_list[led_id].pin,PIN_HIGH);
            }
            else
            {
                rt_pin_write(user_led_list[led_id].pin,PIN_LOW);
            }
        }
    }
}


/* 定时器超时函数 */
static void timer_led_isp(void* parameter)
{
   rt_uint8_t idx;
   for(idx=0; idx<LED_ID_MAX; idx++)
   {
       if(dev_led_context[idx].count < dev_led_context[idx].on_time)
       {
           //on
           dev_led_set(idx,RESET);
           dev_led_context[idx].count++;
       }else if(dev_led_context[idx].count <
               (dev_led_context[idx].on_time+dev_led_context[idx].off_time))
       {
           //off
           dev_led_set(idx,SET);
           dev_led_context[idx].count++;
       }else
       {
           dev_led_context[idx].count = 0x00;
       }

   }
}

void dev_led_conf(rt_uint8_t chx,rt_uint32_t on_time,rt_uint32_t off_time)
{
    if(chx < LED_ID_MAX)
    {
        if(0x00 != (on_time + off_time))
        {
            on_time = on_time/10 + (0x00==on_time%10 ? 0:1);
            off_time = off_time/10 + (0x00==off_time%10 ? 0:1);

            dev_led_context[chx].on_time = on_time;
            dev_led_context[chx].off_time = off_time;
            dev_led_context[chx].count = dev_led_context[0].count;
        }
    }
}


void dev_led_on(void)
{
    rt_pin_write(DEV_LED_PIN, PIN_HIGH);
}
void dev_led_off(void)
{
    rt_pin_write(DEV_LED_PIN, PIN_LOW);

}


void dev_led_init(void)
{
//    rt_kprintf("dev_led_init is started! >>> \n\n");
//    rt_uint8_t idx = 0;
//    rt_memset(&dev_led_context,0,sizeof(dev_led_context));
//    for(idx=0; idx<LED_ID_MAX; idx++)
//    {
//        rt_pin_mode(user_led_list[idx].pin, user_led_list[idx].mode);
//        rt_pin_write(user_led_list[idx].pin, user_led_list[idx].status);
//        user_led_list[idx].status = LED_STATUS_IDLE;
//    }
//
//    for(idx=0; idx<LED_ID_MAX; idx++)
//    {
//        dev_led_context[idx].stat_on = RESET;
//        dev_led_conf(LED_ID_SYS+idx,0,100);
//    }
//
//    dev_led_conf(LED_ID_SYS,50,50);                                 // 系统运行指示灯
//
//    /* 创建定时器  周期定时器 */
//    dev_led_context[idx].timer_led = rt_timer_create("timer_LED",
//                                                      timer_led_isp,
//                                                      RT_NULL,
//                                                      100,
//                                                      RT_TIMER_FLAG_PERIODIC);
//    /* 启动定时器  */
//    if (dev_led_context[idx].timer_led != RT_NULL)
//    {
//        if ( rt_timer_start(dev_led_context[idx].timer_led) == RT_NULL)
//        {
//            rt_kprintf("timer_LED is started!\r\n\n");
//        }
//    }
    rt_pin_mode(DEV_LED_PIN, PIN_MODE_OUTPUT);

    dev_led_on();
    rt_thread_mdelay(1000);
    dev_led_off();
    rt_thread_mdelay(1000);
    dev_led_on();


}


