/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-01-28     gylocal       the first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <bsp_dev_adc.h>
#include <bsp_mod_power.h>

#include "app.h"

#define VOLTAGE_GAIN             (6)
#define VOLTAGE_OFFSET           (370)              // 370mV

#define CURRENT_GAIN             (0.03787878787878787878787878787879)
#define CURRENT_OFFSET           (-1650)            //

#define RATED_VOLTAGE_MV         (12000)            // 12V == 12000mV
#define DWONVOLTAGE_THR          (7000)             //  7V ==  7000mV
#define UNDERVOLTAGE_THR         (9000)             //  9V ==  9000mV
#define OVERVOLTAGE_THR          (18500)            // 18V == 18000mV
#define OVERCURRENT_THR          (40000)            // 40A == 40000mA

#define POWER_CTRL_PIN           GET_PIN(B,15)


/* 创建线程句柄 */
struct rt_thread power_thread;

/* 创建静态线程栈 */
ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t power_thread_stack[1024];



typedef struct{
    uint16_t ADC_sampling_interval;    // 单次 adc 采样间隔   单位：ms
    uint16_t ADC_sampling_quantity;    // 单次 adc 采样数量   单位：ms
}POWER_CONFIG_DEF;


#define ADC_SAMPLING_QUANTITY_MAX    (20)
POWER_CONFIG_DEF power_config_data={
        .ADC_sampling_interval = 10,
        .ADC_sampling_quantity = 20
};

typedef enum{
    POWER_ADC_TYPE_VOLTAGE = 0,
    POWER_ADC_TYPE_CURRENT,
    POWER_ADC_TYPE_MAX
}POWER_ADC_TYPE_DEF;

rt_int32_t power_realtime_data[POWER_ADC_TYPE_MAX][ADC_SAMPLING_QUANTITY_MAX] = {0};

POWER_DATA_DEF power_data = {
        .voltage_state = POWER_VOLTAGE_STATE_NORMAL,
        .current_state = POWER_VOLTAGE_STATE_NORMAL,
        .rated_voltage_mv = RATED_VOLTAGE_MV,
        .downvoltage_thr  = DWONVOLTAGE_THR,
        .undervoltage_thr = UNDERVOLTAGE_THR,
        .overvoltage_thr = OVERVOLTAGE_THR,
        .overcurrent_thr = OVERCURRENT_THR,
        .current_voltage = 0,
        .current_current = 0,
        .voltage_gain = VOLTAGE_GAIN,
        .current_gain = CURRENT_GAIN,
        .voltage_offset = VOLTAGE_OFFSET,
        .current_offset = CURRENT_OFFSET,
        .voltage_error_tick = 0,
        .current_error_tick = 0
};

void set_power_init(){
    rt_pin_mode(POWER_CTRL_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(POWER_CTRL_PIN, PIN_LOW);
}

void set_power_on(){
    rt_pin_write(POWER_CTRL_PIN, PIN_HIGH);
}
void set_power_off(){
    rt_pin_write(POWER_CTRL_PIN, PIN_LOW);
}


float32_t Power_get_voltage_value(void){
    float32_t value;
    value = bsp_get_voltage_value()*power_data.voltage_gain + power_data.voltage_offset;

    return value;
}

float32_t Power_get_current_value(void){
    float32_t value;
    value = (bsp_get_current_value()*power_data.current_gain + power_data.current_offset)*1000;
    return value;
}

void bsp_power_config(void){
    // 读取 系统配置参数


    // 如果 系统配置参数有效，覆盖默认参数

    // 如果 系统配置参数无效，使用默认参数

}



/*
 * 电源状态-正常/过压/欠压
 * UVW 换向
 * CAN 控制命令：左转，右转，停止
 * 事件-过流
 * 绝对角度 信息
 * 目标角度 信息
 *
 * */

// 线程 运行时间 <= 1 tick
static void power_thread_entry(void *parameter )
{

    rt_thread_mdelay(100);


    uint8_t count;
    rt_uint32_t received_e;

    check_tick_data.tick_power_thread = 0;

    uint32_t temp = 0 ;

     // 获取 电源状态信息
    while(1){

        temp = rt_tick_get();

        for ( count = 0; count < power_config_data.ADC_sampling_quantity  ; ++count) {

            power_realtime_data[POWER_ADC_TYPE_VOLTAGE][count] = Power_get_voltage_value();
            power_realtime_data[POWER_ADC_TYPE_CURRENT][count] = Power_get_current_value();
        }

        power_data.current_voltage = 0;
        power_data.current_current = 0;

        for (count = 0; count < (power_config_data.ADC_sampling_quantity ); ++count) {

            power_data.current_voltage += power_realtime_data[POWER_ADC_TYPE_VOLTAGE][count];
            power_data.current_current += power_realtime_data[POWER_ADC_TYPE_CURRENT][count];
        }

        power_data.current_voltage = (power_data.current_voltage / (power_config_data.ADC_sampling_quantity));
        power_data.current_current = (power_data.current_current / (power_config_data.ADC_sampling_quantity));

        power_data.voltage_state = POWER_VOLTAGE_STATE_NORMAL;

        // 检查 电源电压状态
        if ( power_data.current_voltage > power_data.overvoltage_thr) {

            power_data.voltage_state = POWER_VOLTAGE_STATE_OVERVOLTAGE;
            rt_mutex_trytake(&power_mutex_voltage_error);

        } else if ( power_data.current_voltage < power_data.undervoltage_thr) {

            power_data.voltage_state = POWER_VOLTAGE_STATE_UNDERVOLTAGE;
            rt_mutex_trytake(&power_mutex_voltage_error);

        }

        if ( power_data.current_voltage < power_data.downvoltage_thr ) {

            power_data.voltage_state = POWER_VOLTAGE_STATE_DOWNVOLTAGE;

        }

        // 检查 系统 电流状态
        if (power_data.current_current > power_data.overcurrent_thr) {

            power_data.current_state = POWER_CURRENT_STATE_OVERCURRENT;
            rt_mutex_trytake(&power_mutex_current_error);

        } else {
            power_data.current_state = POWER_CURRENT_STATE_NORMAL;
        }


        if ( power_data.voltage_state == POWER_VOLTAGE_STATE_OVERVOLTAGE) {

            power_data.voltage_error_tick = rt_tick_get();
            rt_event_send(&power_voltage_event,POWER_ERROR_VOLTAGE_EVENT);              // 发送 电源 电压 错误 事件
            rt_event_send(&power_voltage_event,POWER_ERROR_OVERVOLTAGE_EVENT);          // 发送 电源 电压 错误 事件 过压

            rt_mq_urgent(&mq_power_voltage_state, &power_data, sizeof(power_data));     // 紧急发送，插入 队列 头部

        }else if (power_data.voltage_state == POWER_VOLTAGE_STATE_UNDERVOLTAGE) {

            power_data.voltage_error_tick = rt_tick_get();
            rt_event_send(&power_voltage_event,POWER_ERROR_VOLTAGE_EVENT);              // 发送 电源 电压 错误 事件
            rt_event_send(&power_voltage_event,POWER_ERROR_UNDERVOLTAGE_EVENT);         // 发送 电源 电压 错误 事件 欠压

            rt_mq_urgent(&mq_power_voltage_state, &power_data, sizeof(power_data));     // 紧急发送，插入 队列 头部

        }else if (power_data.voltage_state == POWER_VOLTAGE_STATE_UNDERVOLTAGE) {

            rt_event_send(&power_voltage_event,POWER_ERROR_VOLTAGE_EVENT);              // 发送 电源 电压 错误 事件
            rt_event_send(&power_voltage_event,POWER_ERROR_DOWNVOLTAGE_EVENT);          // 发送 电源 电压 错误 事件 掉电

            rt_mq_urgent(&mq_power_voltage_state, &power_data, sizeof(power_data));     // 紧急发送，插入 队列 头部

        }else {

            if ( ( rt_tick_get() - power_data.voltage_error_tick) > 60000 ) {
                rt_event_control(&power_voltage_event,RT_IPC_CMD_RESET,RT_NULL);            // 清除 电源错误事件（全部清除 mq_power_voltage_state）
                rt_mq_send(&mq_power_voltage_state, &power_data, sizeof(power_data));       // 正常发送 结构体数据，到队列 尾部
            }
        }

        if( power_data.current_state == POWER_CURRENT_STATE_OVERCURRENT){

            power_data.current_error_tick = rt_tick_get();
            rt_event_send(&power_current_event,POWER_ERROR_CURRENT_EVENT);              // 发送 电源  电流 错误 事件
            rt_mq_urgent(&mq_power_current_state, &power_data, sizeof(power_data));     // 紧急发送，插入 队列 头部

        }else {

            if ( ( rt_tick_get() - power_data.current_error_tick) > 60000 ) {
                rt_event_control(&power_current_event,RT_IPC_CMD_RESET,RT_NULL);            // 清楚 电源错误事件（全部清除 mq_power_current_state）
                rt_mq_send(&mq_power_current_state, &power_data, sizeof(power_data));       // 正常发送 结构体数据，到队列 尾部
            }
        }

        check_tick_data.tick_power_thread = rt_tick_get() - temp;

        rt_thread_mdelay(100);
    }



}


void bsp_power_init(void){

    bsp_power_config();
    set_power_init();
    set_power_on();
    bsp_dev_adc_init();
}


void power_heartbeat_timeout(void* parameter)
{
//    Power_get_voltage_value();
//    Power_get_current_value();
}



void bsp_mod_power_init(void){


    rt_err_t res;
    /********************************  bsp_dev_kth7812_init  初始化 *********************************************/
    bsp_power_init();

    /********************************  power_thread  初始化 *********************************************/

    thread_monitor_info[POWER_THREAD_ID].target_thread   = power_thread;
    thread_monitor_info[POWER_THREAD_ID].name            = "power_thread";
    thread_monitor_info[POWER_THREAD_ID].entry           = power_thread_entry;
    thread_monitor_info[POWER_THREAD_ID].parameter       = RT_NULL;
    thread_monitor_info[POWER_THREAD_ID].stack_start     = power_thread_stack;
    thread_monitor_info[POWER_THREAD_ID].stack_size      = sizeof(power_thread_stack);
    thread_monitor_info[POWER_THREAD_ID].priority        = 6;
    thread_monitor_info[POWER_THREAD_ID].tick            = 3;

//    thread_monitor_info[POWER_THREAD_ID].heartbeat_timer = rt_timer_create("power_heartbeat",
//                                                                         power_heartbeat_timeout,
//                                                                         thread_monitor_info[POWER_THREAD_ID].heartbeat_timer,
////                                                                         (rt_tick_t)power_config_data.ADC_sampling_interval,
//                                                                         100,
//                                                                         RT_TIMER_FLAG_PERIODIC);
//    rt_timer_start(thread_monitor_info[POWER_THREAD_ID].heartbeat_timer);

    res = rt_thread_init( &thread_monitor_info[POWER_THREAD_ID].target_thread,
                           thread_monitor_info[POWER_THREAD_ID].name,
                           thread_monitor_info[POWER_THREAD_ID].entry,
                           thread_monitor_info[POWER_THREAD_ID].parameter,
                           thread_monitor_info[POWER_THREAD_ID].stack_start,
                           thread_monitor_info[POWER_THREAD_ID].stack_size,
                           thread_monitor_info[POWER_THREAD_ID].priority,
                           thread_monitor_info[POWER_THREAD_ID].tick
                           );



    if ( res == RT_EOK )
    {
//        rt_kprintf("go to start power_thread!\n\n");
        rt_thread_startup(&thread_monitor_info[POWER_THREAD_ID].target_thread);
    }
    else
    {
        rt_kprintf("create power_thread failed!\n\n");
    }


}
