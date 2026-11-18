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
#include <bsp_dev_adc.h>

/* ============================================================
 *      兼容旧版  开始
 * ============================================================ */
rt_adc_device_t    adc_dev_current;
rt_adc_device_t    adc_dev_voltage;

#define ADC_DEV_NAME                       "adc2"
#define ADC_DEV_VOLTAGE_CHANNEL            2
#define REFER_VOLTAGE              3300         // 参考电压 3.3v，数据精度 x1000 保留3位小数
#define CONVERT_BITS               (1<<12)      // 转换位数 12 位

#define ADC_GAIN                          (0.8056640625)

#define ADC_DEV_CURRENT_CHANNEL            1


typedef enum{
    ADC_CHANNEL_VOLT = 5,
    ADC_CHANNEL_CURRENT = 6,
    ADC_CHANNEL_MAX
}ADCCHxDef;


#define CC6937
#define ADC_DEV_TYPE               "CC6937"
#define ADC_DEV_0_CURRENT_VOLTAGE  1650         // VCC/2==5000/2
#define ADC_DEV_CURRENT_RANGE      50           // -50A~50A
#define ADC_DEV_mV_mA              0.026400

//#define CC6927
//#define ADC_DEV_TYPE               "CC6927"
//#define ADC_DEV_0_CURRENT_VOLTAGE  1650         // VCC/2==3300/2
//#define ADC_DEV_CURRENT_RANGE      40           // -40A~40A
//#define ADC_DEV_mV_mA              0.026400





/* 采集通道电流-mA */
float32_t bsp_get_current_value(void)
{
    float32_t        value;

    /* 读取 ADC 转换份数  */
    value = rt_adc_read(adc_dev_voltage,ADC_DEV_CURRENT_CHANNEL);

    /* 转换为  0~3300mV */
    value = value * REFER_VOLTAGE / CONVERT_BITS ;

    return value;

}

float32_t bsp_get_voltage_value(void)
{
    float32_t        value;

    /* 读取 ADC 转换份数  */
    value = rt_adc_read(adc_dev_voltage,ADC_DEV_VOLTAGE_CHANNEL);

    /* 转换为  0~3300mV */
    value = value * REFER_VOLTAGE / CONVERT_BITS ;

    return value;
}

rt_int8_t bsp_dev_adc_init(void)
{

    rt_err_t           ret = RT_EOK;

    /* 查找设备 */
    adc_dev_voltage = (rt_adc_device_t)rt_device_find(ADC_DEV_NAME);
    if(adc_dev_voltage == RT_NULL)
    {
        rt_kprintf("adc sample run failed! can't find voltage %s device!\n",ADC_DEV_NAME);
        return RT_ERROR;
    }

    /* 使能设备 */
    ret = rt_adc_enable(adc_dev_voltage,ADC_DEV_VOLTAGE_CHANNEL);


    /* 查找设备 */
    adc_dev_current = (rt_adc_device_t)rt_device_find(ADC_DEV_NAME);
    if(adc_dev_current == RT_NULL)
    {
        rt_kprintf("adc sample run failed! can't find current %s device!\n",ADC_DEV_NAME);
        return RT_ERROR;
    }

    /* 使能设备 */
    ret = rt_adc_enable(adc_dev_current,ADC_DEV_CURRENT_CHANNEL);


    return ret;

//    /* 关闭通道*/
//    ret = re_adc_disable(adc_dev,ADC_DEV_CHANNEL);

}


/* ============================================================
 *      兼容旧版  结束
 * ============================================================ */


/**
 * @file    bsp_dev_adc.c
 * @brief   ADC BSP 层实现
 */

#include "bsp_dev_adc.h"
#include <rtthread.h>

///**
// * @brief 读取指定通道的 ADC 电压
// * @note  TODO: 替换为实际 ADC 采样实现
// */
//uint16_t bsp_adc_read_mv(adc_channel_t channel)
//{
//    uint16_t voltage_mv = 0;
//
//    switch (channel) {
//        case ADC_CH_LIFT:
//            /* TODO: 调用实际 ADC 采样 */
//            voltage_mv = 2203;
//            break;
//
//        case ADC_CH_SWING:
//        case ADC_CH_CURRENT_A:
//        case ADC_CH_CURRENT_B:
//        default:
//            voltage_mv = 0;
//            break;
//    }
//
//    return voltage_mv;
//}
