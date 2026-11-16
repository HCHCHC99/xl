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
#include <bsp_dev_hall_uvw.h>
#include <rtdef.h>

#include "app.h"

#define PULSE_COUNT_ENABLE

#ifdef  PULSE_COUNT_ENABLE

#define PULSE_RIGHT_COUNT      ()
#define PULSE_HALF_RIGHT_COUNT ()
#define PULSE_ZERO_COUNT       (0)

typedef struct{
    int32_t pulse_current_count;
}PULSE_COUNT_DEF;

PULSE_COUNT_DEF pulse_count_data={0};

#endif

#define HALL_U_PIN   ( GET_PIN(A,3) )           // 无刷电机 HALL  U相
#define HALL_V_PIN   ( GET_PIN(A,2) )           // 无刷电机 HALL  V相
#define HALL_W_PIN   ( GET_PIN(A,1) )           // 无刷电机 HALL  W相


static void Hall_U_pulse_exit_init(void);
static void Hall_V_pulse_exit_init(void);
static void Hall_W_pulse_exit_init(void);

static void hall_u_a_pulse_cnt(void* parameter);
static void hall_v_a_pulse_cnt(void* parameter);
static void hall_w_a_pulse_cnt(void* parameter);

//// 三相高低侧OC通道定义
//const uint32_t u32Oc_UHCh = TMR4_OC_CH_UH;     // 对应 硬件 通道  MOS_LOU_EN
//const uint32_t u32Oc_ULCh = TMR4_OC_CH_UL;     // 对应 硬件 通道  MOS_HOU_EN
//
//const uint32_t u32Oc_VHCh = TMR4_OC_CH_VH;     // 对应 硬件 通道 MOS_HOV_EN
//const uint32_t u32Oc_VLCh = TMR4_OC_CH_VL;     // 对应 硬件 通道 MOS_LOV_EN
//
//const uint32_t u32Oc_WHCh = TMR4_OC_CH_WH;     // 对应 硬件 通道 MOS_HOW_EN
//const uint32_t u32Oc_WLCh = TMR4_OC_CH_WL;     // 对应 硬件 通道 MOS_LOW_EN


/**
 * @brief  根据霍尔状态和转向切换PWM通道
 * @param  hall_pos: 霍尔位置（0-5）
 * @retval void
 */
void pwm_bridge_switch(MOTOR_CTRL_HANDLE_DEF  *ctrl_data)
{
    float a = 1.0;
    if (ctrl_data->current_motor_state.is_motor_enable) {

        // 根据当前转向选择对应的PWM切换表
        switch (ctrl_data->current_motor_state.motor_direction)
        {
            case MOTOR_DIRECTION_LEFT:
                switch( (ctrl_data->hall_state_data.code) )
                {
                case  0x05:   // W+ -> V-
                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_WH, 300*a);
                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_WL, 300*a);

                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_VH, 6);
                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_VL, 6);

                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_UH, 300*a);
                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_UL, 6);
                    break;
                case  0x04:  // U+ -> V-
                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_UH, 300*a);
                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_UL, 300*a);

                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_VH, 6);
                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_VL, 6);

                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_WH, 300*a);
                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_WL, 6);
                    break;
                case  0x06:  // U+ -> W-
                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_UH, 300*a);
                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_UL, 300*a);

                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_WH, 6);
                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_WL, 6);

                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_VH, 300*a);
                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_VL, 6);
                    break;
                case  0x02:  // V+ -> W-
                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_VH, 300*a);
                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_VL, 300*a);

                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_WH, 6);
                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_WL, 6);

                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_UH, 300*a);
                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_UL, 6);
                    break;
                case  0x03:  // V+ -> U-
                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_VH, 300*a);
                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_VL, 300*a);

                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_UH, 6);
                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_UL, 6);

                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_WH, 300*a);
                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_WL, 6);
                    break;
                case  0x01:  // W+ -> U-
                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_WH, 300*a);
                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_WL, 300*a);

                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_UH, 6);
                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_UL, 6);

                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_VH, 300*a);
                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_VL, 6);
                    break;
                default:
                    break;
                }

                break;
            case MOTOR_DIRECTION_RIGHT:
                switch( (ctrl_data->hall_state_data.code) )
                {
                case  0x05:  // V+ -> W-
                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_VH, 300*a);
                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_VL, 300*a);

                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_WH, 6*a);
                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_WL, 6*a);

                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_UH, 300*a);
                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_UL, 6*a);
                    break;
                case  0x04:  // V+ -> U-
                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_VH, 300*a);
                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_VL, 300*a);

                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_UH, 6*a);
                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_UL, 6*a);

                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_WH, 300*a);
                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_WL, 6*a);
                    break;
                case  0x06:  // W+ -> U-
                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_WH, 300*a);
                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_WL, 300*a);

                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_UH, 6*a);
                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_UL, 6*a);

                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_VH, 300*a);
                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_VL, 6*a);
                    break;
                case  0x02:  // W+ -> V-
                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_WH, 300*a);
                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_WL, 300*a);

                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_VH, 6*a);
                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_VL, 6*a);

                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_UH, 300*a);
                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_UL, 6*a);
                    break;
                case  0x03:  // U+ -> V-
                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_UH, 300*a);
                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_UL, 300*a);

                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_VH, 6*a);
                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_VL, 6*a);

                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_WH, 300*a);
                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_WL, 6*a);
                    break;
                case  0x01:  // U+ -> W-
                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_UH, 300*a);
                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_UL, 300*a);

                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_WH, 6*a);
                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_WL, 6*a);

                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_VH, 300*a);
                    TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_VL, 6*a);
                    break;
                default:
                    break;

                }
                break;
            case MOTOR_DIRECTION_NULL:
            default:
                // 停止状态：关闭所有桥臂
//                uint16_t compare = MOTOR_PWM_PERIOD * 0.02;

                TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_UH, 6);
                TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_UL, 6);

                TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_VH, 6);
                TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_VL, 6);

                TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_WH, 6);
                TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_WL, 6);
                break;
        }
//        rt_uint32_t pulse = (rt_uint32_t)((ctrl_data->duty)*(ctrl_data->period)/100);
    } else {
        TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_UH, 6);
        TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_UL, 6);

        TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_VH, 6);
        TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_VL, 6);

        TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_WH, 6);
        TMR4_OC_SetCompareValue(CM_TMR4_3, TMR4_OC_CH_WL, 6);
    }
}


void read_hall_state(HALL_STATE_DEF *hall){

    hall->hall_bits.hall_u = rt_pin_read(HALL_U_PIN);
    hall->hall_bits.hall_v = rt_pin_read(HALL_V_PIN);
    hall->hall_bits.hall_w = rt_pin_read(HALL_W_PIN);
}


//霍尔脉冲信号输入引脚
void Hall_Pulse_Input_GPIO_Init(void)
{
    Hall_U_pulse_exit_init();
    Hall_V_pulse_exit_init();
    Hall_W_pulse_exit_init();
}


//无刷电机 HALL  U相   初始化
static void Hall_U_pulse_exit_init(void)
{
    rt_pin_mode(HALL_U_PIN, PIN_MODE_INPUT_PULLUP);

    /* 绑定中断，边沿模式，回调函数名为hall_u_a_pulse_cnt */
    rt_pin_attach_irq(HALL_U_PIN, PIN_IRQ_MODE_RISING_FALLING, hall_u_a_pulse_cnt, RT_NULL);
    /* 使能中断 */
    rt_pin_irq_enable(HALL_U_PIN, PIN_IRQ_ENABLE);
}

//无刷电机 HALL  V相   初始化
static void Hall_V_pulse_exit_init(void)
{
    rt_pin_mode(HALL_V_PIN, PIN_MODE_INPUT_PULLUP);

    /* 绑定中断，边沿模式，回调函数名为hall_v_a_pulse_cnt */
    rt_pin_attach_irq(HALL_V_PIN, PIN_IRQ_MODE_RISING_FALLING, hall_v_a_pulse_cnt, RT_NULL);
    /* 使能中断 */
    rt_pin_irq_enable(HALL_V_PIN, PIN_IRQ_ENABLE);
}

//无刷电机 HALL  W相   初始化
static void Hall_W_pulse_exit_init(void)
{
    rt_pin_mode(HALL_W_PIN, PIN_MODE_INPUT_PULLUP);

    /* 绑定中断，边沿模式，回调函数名为hall_w_a_pulse_cnt */
    rt_pin_attach_irq(HALL_W_PIN, PIN_IRQ_MODE_RISING_FALLING, hall_w_a_pulse_cnt, RT_NULL);
    /* 使能中断 */
    rt_pin_irq_enable(HALL_W_PIN, PIN_IRQ_ENABLE);
}

static void hall_u_a_pulse_cnt(void* parameter)
{
//    rt_pin_write(GET_PIN(H,2), PIN_LOW);

    // U相 脉冲 边沿中断（上升 + 下降）
    if (motor_ctrl_data.current_motor_state.motor_state == MOTOR_STATE_STOP ||
        motor_ctrl_data.current_motor_state.motor_state == MOTOR_STATE_SOFT_STOP_LEFT ||
        motor_ctrl_data.current_motor_state.motor_state == MOTOR_STATE_SOFT_STOP_RIGHT ) {

    }else {
        motor_ctrl_data.duty += PWM_DUTY_CHANGE_MIN;

        motor_ctrl_data.duty = (motor_ctrl_data.duty < 98 ?  motor_ctrl_data.duty : 98 );

        read_hall_state( &(motor_ctrl_data.hall_state_data ));
        pwm_bridge_switch( &motor_ctrl_data );
    }

#ifdef  PULSE_COUNT_ENABLE
    if ( motor_ctrl_data.current_motor_state.motor_state == MOTOR_STATE_SOFT_START_LEFT ||
         motor_ctrl_data.current_motor_state.motor_state == MOTOR_STATE_RUNNING_LEFT ||
         motor_ctrl_data.current_motor_state.motor_state == MOTOR_STATE_SOFT_STOP_LEFT ) {

        pulse_count_data.pulse_current_count--;

    }
    if ( motor_ctrl_data.current_motor_state.motor_state == MOTOR_STATE_SOFT_START_RIGHT ||
         motor_ctrl_data.current_motor_state.motor_state == MOTOR_STATE_RUNNING_RIGHT ||
         motor_ctrl_data.current_motor_state.motor_state == MOTOR_STATE_SOFT_STOP_RIGHT ) {

        pulse_count_data.pulse_current_count++;

    }
#endif
}


static void hall_v_a_pulse_cnt(void* parameter)
{

    // V相 脉冲 边沿中断（上升 + 下降）
    if (motor_ctrl_data.current_motor_state.motor_state == MOTOR_STATE_STOP ||
        motor_ctrl_data.current_motor_state.motor_state == MOTOR_STATE_SOFT_STOP_LEFT ||
        motor_ctrl_data.current_motor_state.motor_state == MOTOR_STATE_SOFT_STOP_RIGHT ) {

    }else {
        motor_ctrl_data.duty += PWM_DUTY_CHANGE_MIN;

        motor_ctrl_data.duty = (motor_ctrl_data.duty < 98 ?  motor_ctrl_data.duty : 98 );

        read_hall_state( &(motor_ctrl_data.hall_state_data ));
        pwm_bridge_switch( &motor_ctrl_data );
    }

#ifdef  PULSE_COUNT_ENABLE
    if ( motor_ctrl_data.current_motor_state.motor_state == MOTOR_STATE_SOFT_START_LEFT ||
         motor_ctrl_data.current_motor_state.motor_state == MOTOR_STATE_RUNNING_LEFT ||
         motor_ctrl_data.current_motor_state.motor_state == MOTOR_STATE_SOFT_STOP_LEFT ) {

        pulse_count_data.pulse_current_count--;

    }
    if ( motor_ctrl_data.current_motor_state.motor_state == MOTOR_STATE_SOFT_START_RIGHT ||
         motor_ctrl_data.current_motor_state.motor_state == MOTOR_STATE_RUNNING_RIGHT ||
         motor_ctrl_data.current_motor_state.motor_state == MOTOR_STATE_SOFT_STOP_RIGHT ) {

        pulse_count_data.pulse_current_count++;

    }
#endif
}


static void hall_w_a_pulse_cnt(void* parameter)
{
    rt_pin_write(GET_PIN(H,2), PIN_LOW);
    // W相 脉冲 边沿中断（上升 + 下降）
    if (motor_ctrl_data.current_motor_state.motor_state == MOTOR_STATE_STOP ||
        motor_ctrl_data.current_motor_state.motor_state == MOTOR_STATE_SOFT_STOP_LEFT ||
        motor_ctrl_data.current_motor_state.motor_state == MOTOR_STATE_SOFT_STOP_RIGHT ) {

    }else {
        motor_ctrl_data.duty += PWM_DUTY_CHANGE_MIN;

        motor_ctrl_data.duty = (motor_ctrl_data.duty < 98 ?  motor_ctrl_data.duty : 98 );

        read_hall_state( &(motor_ctrl_data.hall_state_data ));
        pwm_bridge_switch( &motor_ctrl_data );
    }
#ifdef  PULSE_COUNT_ENABLE
    if ( motor_ctrl_data.current_motor_state.motor_state == MOTOR_STATE_SOFT_START_LEFT ||
         motor_ctrl_data.current_motor_state.motor_state == MOTOR_STATE_RUNNING_LEFT ||
         motor_ctrl_data.current_motor_state.motor_state == MOTOR_STATE_SOFT_STOP_LEFT ) {

        pulse_count_data.pulse_current_count--;

    }
    if ( motor_ctrl_data.current_motor_state.motor_state == MOTOR_STATE_SOFT_START_RIGHT ||
         motor_ctrl_data.current_motor_state.motor_state == MOTOR_STATE_RUNNING_RIGHT ||
         motor_ctrl_data.current_motor_state.motor_state == MOTOR_STATE_SOFT_STOP_RIGHT ) {

        pulse_count_data.pulse_current_count++;

    }
#endif

}

#ifdef  PULSE_COUNT_ENABLE

// 电机 -- 4对极 8槽   UVW hall信号， 边沿触发增量，一圈48次边沿信号

// 正转时 -- 角度增加
// 翻转时-- 角度减少
// 电机 --> 摆臂 减速比：136.5x5=682.5
// 摆臂角度 范围  -300°~+300°  误差 ±2°

// 摆臂角度-- 电机角度-- hall值
//   1      682.5   32760
#define PULSE_ANGLE_X  (32760.0f)

float32_t read_hall_angle_increment(void){
    // 相对上电初始化角度后的，角度增量
    return (float32_t)(pulse_count_data.pulse_current_count/PULSE_ANGLE_X);
}

void write_pulse_count(PULSE_COUNT_DEF *data){

}
#endif


void dev_hall_uvw_init(void){

    Hall_U_pulse_exit_init();
    Hall_V_pulse_exit_init();
    Hall_W_pulse_exit_init();

#ifdef  PULSE_COUNT_ENABLE
    pulse_count_data.pulse_current_count = 0 ;
#endif
}
