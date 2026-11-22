/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-05-23     gylocal       the first version
 */
#ifndef APPLICATIONS_BSP_DEV_MOTOR_H_
#define APPLICATIONS_DEV_MOTOR_H_

#include "app.h"
#define USER_PWM_EN

void dev_motor_init(void);
void dev_motor_soft_start(MOTOR_CTRL_HANDLE_DEF  *ctrl_data);

void dev_motor_stop(void);


#endif /* APPLICATIONS_BSP_DEV_MOTOR_H_ */
