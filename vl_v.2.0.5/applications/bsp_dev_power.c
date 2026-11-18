/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-05-29     gylocal       the first version
 */
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <bsp_dev_power.h>

#include "system_parameter.h"



//void get_dev_power_status(void)
//{
//    // 检查 主电源状态
//    rt_uint32_t  count_p=0;
//    rt_uint32_t  count_n=0;
//    rt_uint32_t  count_d=0;
//    rt_int32_t  index=10;
//
//    for( ; index <= 0 ; index-- ){
//        if ( PIN_HIGH == rt_pin_read(POWER_STAUTS_PIN_P) ){
//            count_p++;
//        }
//        if ( PIN_HIGH == rt_pin_read(POWER_STAUTS_PIN_N) ){
//            count_n++;
//        }
//        rt_thread_mdelay(1);
//    }
//
//    if( count_p >= 10 ){
//        JC_FA2_MOTOR.motor_power_status = MOTOR_POWER_FORWARD;      // 电源 正向上电
//    }else if(count_n >= 10){
//        JC_FA2_MOTOR.motor_power_status = MOTOR_POWER_REVERSE;      // 电源 反向上电
//    }else if( count_p <= 2 && count_n <= 2 ){
//        JC_FA2_MOTOR.motor_power_status = MOTOR_POWER_DOWN;         // 电源 掉电
//    }else {
//        JC_FA2_MOTOR.motor_power_status = MOTOR_POWER_NULL;         // 电源 抖动
//    }
//
//}
//
//void dev_power_init(void)
//{
//    rt_pin_mode(POWER_STAUTS_PIN_P, PIN_MODE_INPUT);
//    rt_pin_mode(POWER_STAUTS_PIN_N, PIN_MODE_INPUT);
//}
