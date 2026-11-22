/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-01-28     gylocal       the first version
 */
#ifndef APPLICATIONS_BSP_DEV_HALL_UVW_H_
#define APPLICATIONS_DEV_HALL_UVW_H_

#include "app.h"

#define  PWM_DUTY_CHANGE_MIN   1



void dev_hall_uvw_init(void);
void read_hall_state(HALL_STATE_DEF *hall);
void Hall_Pulse_Input_GPIO_Init(void);

void pwm_bridge_switch(MOTOR_CTRL_HANDLE_DEF  *ctrl_data);

float32_t read_hall_angle_increment(void);

#endif /* APPLICATIONS_BSP_DEV_HALL_UVW_H_ */
