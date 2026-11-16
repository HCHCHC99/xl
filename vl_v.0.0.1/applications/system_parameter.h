/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-05-23     gylocal       the first version
 */
#ifndef APPLICATIONS_SYSTEM_PARAMETER_H_
#define APPLICATIONS_SYSTEM_PARAMETER_H_


#define CURRENT_OVER_VALUE          33000
#define MOTOR_RUN_DETECION_CURRENT  500

#define POSTION_FILTER_MAX          10

//
//typedef enum
//{
//    MOTOR_POWER_NULL,
//    MOTOR_POWER_FORWARD,
//    MOTOR_POWER_REVERSE,
//    MOTOR_POWER_DOWN
//}MOTOR_POWER;
//
//typedef enum
//{
//    MOTOR_CMD_NULL,
//    MOTOR_CMD_STOP,
//    MOTOR_CMD_FORWARD,
//    MOTOR_CMD_REVERSE
//}MOTOR_CMD;
//
//
//typedef enum
//{
//    MOTOR_ERROR_NULL,
//    MOTOR_ERROR_UNDER_VOLTAGE,
//    MOTOR_ERROR_OVER_VOLTAGE,
//    MOTOR_ERROR_OVER_CURRENT,
//    MOTOR_ERROR_TIMEOUT
//}MOTOR_ERROR;
//
//typedef enum
//{
//    MOTOR_STATUS_NULL,
//    MOTOR_STATUS_STOP,
//    MOTOR_STATUS_FORWARD,
//    MOTOR_STATUS_REVERSE,
//    MOTOR_STATUS_ERROR
//}MOTOR_STATUS;
//
//typedef enum
//{
//    MOTOR_LIMIT_NULL,
//    MOTOR_LIMIT_IS_ON,
//    MOTOR_LIMIT_IS_OFF
//}MOTOR_LIMIT;
//
//typedef enum
//{
//    POSTION_UP,
//    POSTION_DOWN
//}POSTION_NAME;
//typedef enum
//{
//    MOTOR_POSTION_NULL,
//    MOTOR_POSTION_IS_REVERSE_LIMIT,
//    MOTOR_POSTION_IS_FORWARD_LIMIT,
//    MOTOR_POSTION_IS_MIDDLE
//}MOTOR_POSTION;
//
//typedef struct
//{
//    rt_uint8_t      filter ;
//    MOTOR_LIMIT     motor_limit;
//}POSTION_SENSOR;
//
//typedef struct
//{
//    MOTOR_POWER     motor_power_status;      // 主电源 状态 ： 正向给电，反向给电（反向给电包含未供电状态）
//    MOTOR_CMD       motor_cmd;               // 电机控制命令
//    MOTOR_STATUS    motor_status;            // 电机运行状态
//    MOTOR_ERROR     motor_error;             // 电机 错误状态字
//    POSTION_SENSOR  postion_senor[2];        // 位置传感器
//    MOTOR_POSTION   motor_postion;           // 电机运行位置
//    rt_int32_t      motor_current;           // 电机运行电流 --mA
//}FA2_S;
//
//
//extern FA2_S JC_FA2_MOTOR;

#endif /* APPLICATIONS_SYSTEM_PARAMETER_H_ */
