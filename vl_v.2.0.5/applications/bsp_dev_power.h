/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-05-29     gylocal       the first version
 */
#ifndef APPLICATIONS_BSP_DEV_POWER_H_
#define APPLICATIONS_DEV_POWER_H_

#define POWER_STAUTS_PIN_P  GET_PIN(B, 9)      // 电源供电状态监测：高电平-正向
#define POWER_STAUTS_PIN_N  GET_PIN(A, 3)      // 电源供电状态监测：高电平-反向

void get_dev_power_status(void);
void dev_power_init(void);
#endif /* APPLICATIONS_BSP_DEV_POWER_H_ */
