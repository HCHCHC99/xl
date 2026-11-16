/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-01-28     gylocal       the first version
 */
#ifndef APPLICATIONS_BSP_DRV_SPI_H_
#define APPLICATIONS_DEV_SPI_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp_dev_kth7812.h"
/* Includes ------------------------------------------------------------------*/

//#include "stm32f4xx_hal_ex.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */


/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

//void MX_SPI1_Init(void);
//void MX_SPI3_Init(void);

#define SPI1_BUS_NAME       "spi1"
#define SPI3_BUS_NAME       "spi3"

//struct rt_spi_device *spi_dev_kth7812_41;
//struct rt_spi_device *spi_dev_kth7812_42;
//struct rt_spi_device *spi_dev_kth7812_43;

extern KTH7812_CONFIG_DEF kth7812_config_data[KTH7812_DEV_MAX];
void bsp_dev_spi_init(void);


//extern void SPI1_SetSpeed(uint8_t SPI_BaudRatePrescaler);
//extern void SPI3_SetSpeed(uint8_t SPI_BaudRatePrescaler);
//
//extern uint8_t SPI1_ReadWriteByte(uint8_t TxData);
//extern uint8_t SPI3_ReadWriteByte(uint8_t TxData);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif


#endif /* APPLICATIONS_BSP_DRV_SPI_H_ */
