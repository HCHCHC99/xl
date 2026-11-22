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


#include "app.h"
#include "bsp_dev_pwm.h"

/* 【2026-10-27 · D-60】dev_pwm 的唯一实体定义。
 * app.h 原先把 `struct rt_device_pwm *dev_pwm;` 直接写在头文件里，
 * 导致每个包含它的 .c 都产生一份定义，全量编译时链接器报
 * multiple definition。现在头文件改为 extern 声明，实体放这里
 * （本文件是唯一给 dev_pwm 赋值的模块）。 */
struct rt_device_pwm *dev_pwm;

#define BRIDGE_UH (1<<5)  // U相上桥臂（bit5）
#define BRIDGE_UL (1<<4)  // U相下桥臂（bit4）

#define BRIDGE_VH (1<<3)  // V相上桥臂（bit3）
#define BRIDGE_VL (1<<2)  // V相下桥臂（bit2）

#define BRIDGE_WH (1<<1)  // W相上桥臂（bit1）
#define BRIDGE_WL (1<<0)  // W相下桥臂（bit0）

/* 全局变量：PWM通道切换表（六步方波标准相序，可根据电机转向调整） */
// 正转切换表（标准六步相序）
const pwm_switch_table_t pwm_table_forward = {
    .raw = {
            0x00,                          // 0x00 (000) - 无效：全关断
            BRIDGE_VH | BRIDGE_WL,         // 0x01 (001)：V上 + W下 → 电流：V→W
            BRIDGE_UH | BRIDGE_WL,         // 0x02 (010)：U上 + W下 → 电流：U→W
            BRIDGE_UH | BRIDGE_VL,         // 0x03 (011)：U上 + V下 → 电流：U→V
            BRIDGE_UH | BRIDGE_VL,         // 0x04 (100)：W上 + V下 → 电流：W→V
            BRIDGE_WH | BRIDGE_UL,         // 0x05 (101)：W上 + U下 → 电流：W→U
            BRIDGE_VH | BRIDGE_UL,         // 0x06 (110)：V上 + U下 → 电流：V→U
            0x00                           // 0x07 (111) - 无效：全关断
    }
};

// 反转切换表（正转表的逆序，核心：霍尔位置对应的桥臂状态反向）
const pwm_switch_table_t pwm_table_reverse = {
    .raw = {
            0x00,                          // 0x00 (000) - 无效：全关断
            BRIDGE_VH | BRIDGE_UL,         // 0x01 (001)：V上 + U下（对应正转0x06）
            BRIDGE_WH | BRIDGE_UL,         // 0x02 (010)：W上 + U下（对应正转0x05）
            BRIDGE_WH | BRIDGE_VL,         // 0x03 (011)：W上 + V下（对应正转0x04）
            BRIDGE_UH | BRIDGE_VL,         // 0x04 (100)：U上 + V下（对应正转0x03）
            BRIDGE_UH | BRIDGE_WL,         // 0x05 (101)：U上 + W下（对应正转0x02）
            BRIDGE_VH | BRIDGE_WL,         // 0x06 (110)：V上 + W下（对应正转0x01）
            0x00                           // 0x07 (111) - 无效：全关断
    }
};

rt_uint8_t dev_pwm_init(void)
{
    //查找设备
    dev_pwm = (struct rt_device_pwm*)rt_device_find(DEV_PWM_NAME);

    if(dev_pwm == RT_NULL)
    {
        rt_kprintf("dev_pwm2 can't find %s device!\n\n", DEV_PWM_NAME);
        return RT_ERROR;
    }
    else
    {
        rt_kprintf("dev_pwm2  find %s device!\n\n", DEV_PWM_NAME);
    }

    //设置参数, 默认输出PWM为0%占空比,即高电平
    rt_pwm_set(dev_pwm,DEV_PWM_CHAN_U_H, MOTOR_PWM_PERIOD, MOTOR_OFF_PULSE);
    rt_pwm_set(dev_pwm,DEV_PWM_CHAN_U_L, MOTOR_PWM_PERIOD, MOTOR_OFF_PULSE);
    rt_pwm_set(dev_pwm,DEV_PWM_CHAN_V_H, MOTOR_PWM_PERIOD, MOTOR_OFF_PULSE);
    rt_pwm_set(dev_pwm,DEV_PWM_CHAN_V_L, MOTOR_PWM_PERIOD, MOTOR_OFF_PULSE);
    rt_pwm_set(dev_pwm,DEV_PWM_CHAN_W_H, MOTOR_PWM_PERIOD, MOTOR_OFF_PULSE);
    rt_pwm_set(dev_pwm,DEV_PWM_CHAN_W_L, MOTOR_PWM_PERIOD, MOTOR_OFF_PULSE);

    //默认 使能PWM设备通道
    rt_pwm_enable(dev_pwm, DEV_PWM_CHAN_U_H);
    rt_pwm_enable(dev_pwm, DEV_PWM_CHAN_U_H);

    rt_pwm_enable(dev_pwm, DEV_PWM_CHAN_V_H);
    rt_pwm_enable(dev_pwm, DEV_PWM_CHAN_V_L);

    rt_pwm_enable(dev_pwm, DEV_PWM_CHAN_W_H);
    rt_pwm_enable(dev_pwm, DEV_PWM_CHAN_W_L);

    return RT_EOK;
}
