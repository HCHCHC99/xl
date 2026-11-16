/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 * Copyright (c) 2022, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-04-28     CDT          first version
 */

#include <rtdevice.h>
#include "board_config.h"

/**
 * The below functions will initialize HC32 board.
 */

#if defined RT_USING_SERIAL
rt_err_t rt_hw_board_uart_init(CM_USART_TypeDef *USARTx)
{
    rt_err_t result = RT_EOK;

    switch ((rt_uint32_t)USARTx)
    {
#if defined(BSP_USING_UART1)
    case (rt_uint32_t)CM_USART1:
        /* Configure USART RX/TX pin. */
        GPIO_SetFunc(USART1_RX_PORT, USART1_RX_PIN, GPIO_FUNC_33);
        GPIO_SetFunc(USART1_TX_PORT, USART1_TX_PIN, GPIO_FUNC_32);
        break;
#endif
#if defined(BSP_USING_UART2)
    case (rt_uint32_t)CM_USART2:
        /* Configure USART RX/TX pin. */
        GPIO_SetFunc(USART2_RX_PORT, USART2_RX_PIN, GPIO_FUNC_37);
        GPIO_SetFunc(USART2_TX_PORT, USART2_TX_PIN, GPIO_FUNC_36);
        break;
#endif
#if defined(BSP_USING_UART3)
    case (rt_uint32_t)CM_USART3:
        /* Configure USART RX/TX pin. */
        GPIO_SetFunc(USART3_RX_PORT, USART3_RX_PIN, GPIO_FUNC_33);
        GPIO_SetFunc(USART3_TX_PORT, USART3_TX_PIN, GPIO_FUNC_32);
        break;
#endif
#if defined(BSP_USING_UART4)
    case (rt_uint32_t)CM_USART4:
        /* Configure USART RX/TX pin. */
        GPIO_SetFunc(USART4_RX_PORT, USART4_RX_PIN, GPIO_FUNC_37);
        GPIO_SetFunc(USART4_TX_PORT, USART4_TX_PIN, GPIO_FUNC_36);
        break;
#endif
    default:
        result = -RT_ERROR;
        break;
    }

    return result;
}
#endif

#if defined(RT_USING_CAN)
void CanPhyEnable(void)
{
#if defined(BSP_USING_CAN1)
#ifdef CAN1_STB_FUNC_ENABLE
    GPIO_ResetPins(CAN_STB_PORT, CAN_STB_PIN);
    GPIO_OutputCmd(CAN_STB_PORT, CAN_STB_PIN, ENABLE);
#endif
#endif
}
rt_err_t rt_hw_board_can_init(CM_CAN_TypeDef *CANx)
{
    rt_err_t result = RT_EOK;

    switch ((rt_uint32_t)CANx)
    {
#if defined(BSP_USING_CAN1)
case (rt_uint32_t)CM_CAN:
    GPIO_SetFunc(CAN1_TX_PORT, CAN1_TX_PIN, CAN1_TX_PIN_FUNC);
    GPIO_SetFunc(CAN1_RX_PORT, CAN1_RX_PIN, CAN1_RX_PIN_FUNC);
    break;
#endif
default:
    result = -RT_ERROR;
    break;
    }

    return result;
}
#endif

#if defined(RT_USING_ADC)
rt_err_t rt_hw_board_adc_init(CM_ADC_TypeDef *ADCx)
{
    rt_err_t result = RT_EOK;
    stc_gpio_init_t stcGpioInit;

    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinAttr = PIN_ATTR_ANALOG;
    switch ((rt_uint32_t)ADCx)
    {
#if defined(BSP_USING_ADC1)
    case (rt_uint32_t)CM_ADC1:
        (void)GPIO_Init(ADC1_CH3_PORT, ADC1_CH3_PIN, &stcGpioInit);  /* PA3 CURRENT_TOTAL */
        (void)GPIO_Init(ADC1_CH4_PORT, ADC1_CH4_PIN, &stcGpioInit);  /* PA4 CURRENT_W     */
        (void)GPIO_Init(ADC1_CH5_PORT, ADC1_CH5_PIN, &stcGpioInit);  /* PA5 CURRENT_V     */
        break;
#endif
#if defined(BSP_USING_ADC2)
    case (rt_uint32_t)CM_ADC2:
        (void)GPIO_Init(ADC2_CH4_PORT, ADC2_CH4_PIN, &stcGpioInit);
        (void)GPIO_Init(ADC2_CH5_PORT, ADC2_CH5_PIN, &stcGpioInit);
        (void)GPIO_Init(ADC2_CH6_PORT, ADC2_CH6_PIN, &stcGpioInit);
        (void)GPIO_Init(ADC2_CH7_PORT, ADC2_CH7_PIN, &stcGpioInit);
        break;
#endif
    default:
        result = -RT_ERROR;
        break;
    }

    return result;
}
#endif


#if defined(RT_USING_SPI)
/* spi port init */
rt_err_t rt_hw_board_spi_init(CM_SPI_TypeDef *CM_SPIx)
{
    rt_err_t result = RT_EOK;

    stc_gpio_init_t stcGpioInit;
    stc_spi_init_t stcSpiInit;
    stc_spi_delay_t stcSpiDelayCfg;

    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinDrv = PIN_HIGH_DRV;

    (void)GPIO_Init(SPI_SCK_PORT, SPI_SCK_PIN, &stcGpioInit);
    (void)GPIO_Init(SPI_MOSI_PORT, SPI_MOSI_PIN, &stcGpioInit);

    switch ((rt_uint32_t)CM_SPIx)
    {
#if defined(BSP_USING_SPI1)
    case (rt_uint32_t)CM_SPI1:
        (void)GPIO_SetFunc(SPI_SCK_PORT,  SPI_SCK_PIN,  SPI_SCK_FUNC);
        (void)GPIO_SetFunc(SPI_MOSI_PORT, SPI_MOSI_PIN, SPI_MOSI_FUNC);
        (void)GPIO_SetFunc(SPI_MISO_PORT, SPI_MISO_PIN, SPI_MISO_FUNC);
        break;
#endif
#if defined(BSP_USING_SPI2)

#endif
    default:
        result = -RT_ERROR;
        break;
    }

    /* Clear initialize structure */
    (void)SPI_StructInit(&stcSpiInit);
    (void)SPI_DelayStructInit(&stcSpiDelayCfg);

    /* Configure peripheral clock */
    FCG_Fcg1PeriphClockCmd(FCG1_PERIPH_SPI1, ENABLE);

    return result;
}
#endif

#if defined(BSP_USING_PWM_TIM4_3)
/**
 * @brief  TIM4_3 三相互补 PWM 引脚配置
 * @param  CM_TMRx 定时器实例（仅支持 CM_TMR4_3）
 * @return RT_EOK 成功；-RT_ERROR 不支持的实例
 * @note   仅配置 GPIO 复用功能，不触碰 TIM4 寄存器（由 bsp_pwm.c 负责）
 *         开关为 BSP_USING_PWM_TIM4_3（本项目自定），
 *         不依赖 RT-Thread PWM 设备框架的 RT_USING_PWM。
 *         [PERF-EVAL-01] 见 board_config.h 说明
 */
rt_err_t rt_hw_board_pwm_init(CM_TMR4_TypeDef *CM_TMRx)
{
    if (CM_TMRx == CM_TMR4_3)
    {
        (void)GPIO_SetFunc(PWM_UH_PORT, PWM_UH_PIN, PWM_UH_PIN_FUNC);
        (void)GPIO_SetFunc(PWM_UL_PORT, PWM_UL_PIN, PWM_UL_PIN_FUNC);

        (void)GPIO_SetFunc(PWM_VH_PORT, PWM_VH_PIN, PWM_VH_PIN_FUNC);
        (void)GPIO_SetFunc(PWM_VL_PORT, PWM_VL_PIN, PWM_VL_PIN_FUNC);

        (void)GPIO_SetFunc(PWM_WH_PORT, PWM_WH_PIN, PWM_WH_PIN_FUNC);
        (void)GPIO_SetFunc(PWM_WL_PORT, PWM_WL_PIN, PWM_WL_PIN_FUNC);

        return RT_EOK;
    }

    return -RT_ERROR;
}
#endif

/**
 * @brief  电磁刹车 GPIO 配置
 * @return RT_EOK
 * @note   仅配置引脚方向与初始电平，不涉及刹车状态机（由 hw_brake 负责）
 *         上电默认输出锁止电平
 *         [PERF-EVAL-03] 见 board_config.h 说明
 */
rt_err_t rt_hw_board_brake_init(void)
{
    stc_gpio_init_t stcGpioInit;

    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinState = (BRAKE_ACTIVE_LOW != 0U) ? PIN_STAT_SET : PIN_STAT_RST;
    stcGpioInit.u16PinDir   = PIN_DIR_OUT;
    (void)GPIO_Init(BRAKE_PORT, BRAKE_PIN, &stcGpioInit);

    return RT_EOK;
}


