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

#include "app.h"
#include "bsp_dev_motor.h"
#include "bsp_dev_pwm.h"
#include "bsp_dev_hall_uvw.h"

//// 先定义正反转选择标识
//typedef enum {
//    MOTOR_DIR_FORWARD = 0,  // 正转
//    MOTOR_DIR_REVERSE = 1   // 反转
//} Motor_Dir_t;

// 全局变量：电机当前转向（可通过外部指令修改，


// 定义PWM使能状态联合体：共享1个字节（8位），作为PWM使能表的元素类型
// 适配：上桥臂PWM，下桥臂直通（可根据需求修改为下桥臂PWM、上桥臂直通）
typedef union {
    // 视角1：按单个PWM配置项访问（位域，可读性强）
    struct {
        // 下桥臂直通使能（bit0-bit2）
        uint8_t u_l_direct : 1;  // bit0：U相下桥臂是否直通（1=导通，0=关断）
        uint8_t v_l_direct : 1;  // bit1：V相下桥臂是否直通（1=导通，0=关断）
        uint8_t w_l_direct : 1;  // bit2：W相下桥臂是否直通（1=导通，0=关断）

        // 上桥臂PWM使能（bit3-bit5）
        uint8_t u_h_pwm : 1;     // bit3：U相上桥臂是否输出PWM（1=使能，0=关断）
        uint8_t v_h_pwm : 1;     // bit4：V相上桥臂是否输出PWM（1=使能，0=关断）
        uint8_t w_h_pwm : 1;     // bit5：W相上桥臂是否输出PWM（1=使能，0=关断）

        // 辅助标识（bit6-bit7）
        uint8_t valid : 1;       // bit6：当前配置是否有效（1=有效，0=无效）
        uint8_t reserved : 1;    // bit7：保留位，凑够1个字节
    } bits;

    // 视角2：按整体编码访问（1个字节，方便存入PWM使能表，快速查表）
    uint8_t code;
} PWM_Enable_t;


// 正转配置宏（上桥臂PWM，下桥臂直通）
#define PWM_FORWARD_U_V  ((PWM_Enable_t){.bits.u_h_pwm=1, .bits.v_l_direct=1, .bits.valid=1})
#define PWM_FORWARD_U_W  ((PWM_Enable_t){.bits.u_h_pwm=1, .bits.w_l_direct=1, .bits.valid=1})
#define PWM_FORWARD_V_W  ((PWM_Enable_t){.bits.v_h_pwm=1, .bits.w_l_direct=1, .bits.valid=1})
#define PWM_FORWARD_V_U  ((PWM_Enable_t){.bits.v_h_pwm=1, .bits.u_l_direct=1, .bits.valid=1})
#define PWM_FORWARD_W_U  ((PWM_Enable_t){.bits.w_h_pwm=1, .bits.u_l_direct=1, .bits.valid=1})
#define PWM_FORWARD_W_V  ((PWM_Enable_t){.bits.w_h_pwm=1, .bits.v_l_direct=1, .bits.valid=1})

// 反转配置宏（颠倒正转顺序，对应反转导通逻辑）
#define PWM_REVERSE_W_V  ((PWM_Enable_t){.bits.w_h_pwm=1, .bits.v_l_direct=1, .bits.valid=1})
#define PWM_REVERSE_W_U  ((PWM_Enable_t){.bits.w_h_pwm=1, .bits.u_l_direct=1, .bits.valid=1})
#define PWM_REVERSE_V_U  ((PWM_Enable_t){.bits.v_h_pwm=1, .bits.u_l_direct=1, .bits.valid=1})
#define PWM_REVERSE_V_W  ((PWM_Enable_t){.bits.v_h_pwm=1, .bits.w_l_direct=1, .bits.valid=1})
#define PWM_REVERSE_U_W  ((PWM_Enable_t){.bits.u_h_pwm=1, .bits.w_l_direct=1, .bits.valid=1})
#define PWM_REVERSE_U_V  ((PWM_Enable_t){.bits.u_h_pwm=1, .bits.v_l_direct=1, .bits.valid=1})

// 步骤2：构建正转PWM使能表（索引=霍尔编码1~6，与之前换相表对应）
const PWM_Enable_t pwm_enable_table_forward[8] = {
    {.code=0x00},  // 索引0：0b000（无效）
    PWM_FORWARD_U_V,  // 索引1：0b001 → U+V-
    PWM_FORWARD_V_W,  // 索引2：0b010 → V+W-
    PWM_FORWARD_U_W,  // 索引3：0b011 → U+W-
    PWM_FORWARD_W_U,  // 索引4：0b100 → W+U-
    PWM_FORWARD_W_V,  // 索引5：0b101 → W+V-
    PWM_FORWARD_V_U,  // 索引6：0b110 → V+U-
    {.code=0x07}   // 索引7：0b111（无效）
};

// 步骤3：构建反转PWM使能表（索引=霍尔编码1~6，颠倒正转顺序）
const PWM_Enable_t pwm_enable_table_reverse[8] = {
    {.code=0x00},  // 索引0：0b000（无效）
    PWM_REVERSE_U_V,  // 索引1：0b001 → 对应正转最后一步
    PWM_REVERSE_V_W,  // 索引2：0b010 → 对应正转倒数第二步
    PWM_REVERSE_U_W,  // 索引3：0b011 → 对应正转倒数第三步
    PWM_REVERSE_W_U,  // 索引4：0b100 → 对应正转第三步
    PWM_REVERSE_W_V,  // 索引5：0b101 → 对应正转第二步
    PWM_REVERSE_V_U,  // 索引6：0b110 → 对应正转第一步
    {.code=0x07}   // 索引7：0b111（无效）
};





MOTOR_STATE_DEF motor_state_date = {
        .motor_direction = MOTOR_DIRECTION_NULL,
        .motor_state = MOTOR_STATE_NULL
};

#ifdef USER_PWM_EN

#else

#define MOTOR_OUH_PIN      GET_PIN(A, 2)                         //
#define MOTOR_OUL_PIN      GET_PIN(A, 2)                         //

#define MOTOR_OVH_PIN      GET_PIN(A, 2)                         //
#define MOTOR_OVL_PIN      GET_PIN(A, 2)                         //

#define MOTOR_OWH_PIN      GET_PIN(A, 2)                         //
#define MOTOR_OWL_PIN      GET_PIN(A, 2)                         //

#endif

#define PWM_CHANG_COUNT_MAX   (8)
#define PWN_START_DUTY        (50)

//rt_uint8_t PWM_chang_count = 0;

rt_timer_t   timer_soft_start = NULL;

rt_uint32_t PWM_duty = PWN_START_DUTY;



//SDH21263输入输出逻辑表
//HIN   LIN           HO     LO
//L       L            L      H
//L       H            L      L
//H       L            L      L
//H       H            H      L


void dev_motor_soft_start(MOTOR_CTRL_HANDLE_DEF  *ctrl_data)
{
#ifdef USER_PWM_EN

    ctrl_data->duty = ctrl_data->soft_start_duty;

//    if (ctrl_data->current_motor_state.motor_direction == MOTOR_DIRECTION_LEFT ) {
//
//        // 读取 UVW hall 位置
//        read_hall_state( &(ctrl_data->hall_state_data) );
//
//        // 使用对应的 pwm 占空比，使能 相应 pwm通道
//        pwm_bridge_switch( ctrl_data );
//
//
//    }else if (ctrl_data->current_motor_state.motor_direction == MOTOR_DIRECTION_LEFT  ){
//
//    }

    // 读取 UVW hall 位置
    read_hall_state( &(ctrl_data->hall_state_data) );

    // 使用对应的 pwm 占空比，使能 相应 pwm通道
    pwm_bridge_switch( ctrl_data );
//    // 启动定时器
//    PWM_duty = PWN_START_DUTY;
//    rt_timer_start(timer_soft_start);
#else
    MOTOR_ENABLE;
#endif
}

void dev_motor_stop(void)
{
#ifdef USER_PWM_EN
    // 关闭电机
//    rt_pwm_set(dev_pwm,DEV_PWM_CHAN_U_H, MOTOR_PWM_PERIOD, MOTOR_ON_PULSE);
//    rt_pwm_set(dev_pwm,DEV_PWM_CHAN_V_H, MOTOR_PWM_PERIOD, MOTOR_OFF_PULSE);
//    rt_pwm_set(dev_pwm,DEV_PWM_CHAN_W_H, MOTOR_PWM_PERIOD, MOTOR_OFF_PULSE);
//
//    rt_pwm_set(dev_pwm,DEV_PWM_CHAN_U_L, MOTOR_PWM_PERIOD, MOTOR_OFF_PULSE);
//    rt_pwm_set(dev_pwm,DEV_PWM_CHAN_V_L, MOTOR_PWM_PERIOD, MOTOR_OFF_PULSE);
//    rt_pwm_set(dev_pwm,DEV_PWM_CHAN_W_L, MOTOR_PWM_PERIOD, MOTOR_OFF_PULSE);

//    rt_pwm_set(dev_pwm,1, MOTOR_PWM_PERIOD, MOTOR_ON_PULSE);
//    rt_pwm_set(dev_pwm,2, MOTOR_PWM_PERIOD, MOTOR_ON_PULSE);
//    rt_pwm_set(dev_pwm,3, MOTOR_PWM_PERIOD, MOTOR_ON_PULSE);
//
//    rt_pwm_set(dev_pwm,4, MOTOR_PWM_PERIOD, MOTOR_ON_PULSE);
//    rt_pwm_set(dev_pwm,5, MOTOR_PWM_PERIOD, MOTOR_ON_PULSE);
//    rt_pwm_set(dev_pwm,6, MOTOR_PWM_PERIOD, MOTOR_ON_PULSE);

    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_UH, 6);
    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_UL, 6);

    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_VH, 6);
    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_VL, 6);

    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_WH, 6);
    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_WL, 6);


#else



    rt_pin_write(MOTOR_OWL_PIN, PIN_HIGH);
#endif
}

#ifdef USER_PWM_EN
/* 定时器超时函数 */
static void soft_start_timeout(void* parameter)
{

//    PWM_duty++;
//    if (PWM_duty <= 100) {
//        rt_pwm_set(dev_pwm,
//                   DEV_PWM2_CHAN_L4,
//                   MOTOR_PWM_PERIOD,
//                   MOTOR_PWM_PERIOD*PWM_duty/100);
//    }else {
//        PWM_duty = PWN_START_DUTY;
//        rt_timer_stop(timer_soft_start);
//    }

}
#endif

void dev_motor_init(void)
{
#ifdef USER_PWM_EN

//    // 初始化 PWM
//    dev_pwm_init();

//    // 创建 周期定时器
//    timer_soft_start = rt_timer_create("motor_soft_start",
//                                        soft_start_timeout,
//                                        RT_NULL,
//                                        20,
//                                        RT_TIMER_FLAG_PERIODIC);

    //    /* 启动TMR4计数，输出PWM */
        TMR4_Start(CM_TMR4_3);

#else
    rt_pin_mode(MOTOR_OUH_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(MOTOR_OUL_PIN, PIN_MODE_OUTPUT);

    rt_pin_mode(MOTOR_OVH_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(MOTOR_OVL_PIN, PIN_MODE_OUTPUT);

    rt_pin_mode(MOTOR_OWH_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(MOTOR_OWL_PIN, PIN_MODE_OUTPUT);

#endif
    dev_motor_stop();
}
