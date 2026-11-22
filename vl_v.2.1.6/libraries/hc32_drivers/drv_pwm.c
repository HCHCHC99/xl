/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-02-11     gylocal       the first version
 */


/*
 *    需要自行匹配驱动，先按照 hc32_ll_tim 实现功能，后续嵌入 rtt
 * */
#include <board.h>
#include <drivers/adc.h>
#include <drv_adc.h>
#include <drv_config.h>


#include <rtdbg.h>

#ifdef RT_USING_PWM

//#define DRV_DEBUG
#define LOG_TAG             "drv.pwm"

#define MAX_PERIOD 65535
#define MIN_PERIOD 3
#define MIN_PULSE 2

//enum
//{
//#ifdef BSP_USING_PWM1
//    PWM1_INDEX,
//#endif
//#ifdef BSP_USING_PWM2
//    PWM2_INDEX,
//#endif
//#ifdef BSP_USING_PWM3
//    PWM3_INDEX,
//#endif
//#ifdef BSP_USING_PWM4
//    PWM4_INDEX,
//#endif
//#ifdef BSP_USING_PWM5
//    PWM5_INDEX,
//#endif
//#ifdef BSP_USING_PWM6
//    PWM6_INDEX,
//#endif
//#ifdef BSP_USING_PWM7
//    PWM7_INDEX,
//#endif
//#ifdef BSP_USING_PWM8
//    PWM8_INDEX,
//#endif
//#ifdef BSP_USING_PWM9
//    PWM9_INDEX,
//#endif
//#ifdef BSP_USING_PWM10
//    PWM10_INDEX,
//#endif
//#ifdef BSP_USING_PWM11
//    PWM11_INDEX,
//#endif
//#ifdef BSP_USING_PWM12
//    PWM12_INDEX,
//#endif
//#ifdef BSP_USING_PWM13
//    PWM13_INDEX,
//#endif
//#ifdef BSP_USING_PWM14
//    PWM14_INDEX,
//#endif
//#ifdef BSP_USING_PWM15
//    PWM15_INDEX,
//#endif
//#ifdef BSP_USING_PWM16
//    PWM16_INDEX,
//#endif
//#ifdef BSP_USING_PWM17
//    PWM17_INDEX,
//#endif
//};
//
//typedef struct
//{
//    struct rt_device_pwm rt_pwm;
//    CM_TMR4_TypeDef   *instance;
//    rt_uint8_t channel;
//    char *name;
//} pwm_device;
//
//static pwm_device g_pwm_dev_array[] =
//{
//#ifdef BSP_USING_PWM1
//    PWM1_CONFIG,
//#endif
//
//#ifdef BSP_USING_PWM2
//    PWM2_CONFIG,
//#endif
//
//#ifdef BSP_USING_PWM3
//    PWM3_CONFIG,
//#endif
//
//#ifdef BSP_USING_PWM4
//    PWM4_CONFIG,
//#endif
//
//#ifdef BSP_USING_PWM5
//    PWM5_CONFIG,
//#endif
//
//#ifdef BSP_USING_PWM6
//    PWM6_CONFIG,
//#endif
//
//#ifdef BSP_USING_PWM7
//    PWM7_CONFIG,
//#endif
//
//#ifdef BSP_USING_PWM8
//    PWM8_CONFIG,
//#endif
//
//#ifdef BSP_USING_PWM9
//    PWM9_CONFIG,
//#endif
//
//#ifdef BSP_USING_PWM10
//    PWM10_CONFIG,
//#endif
//
//#ifdef BSP_USING_PWM11
//    PWM11_CONFIG,
//#endif
//
//#ifdef BSP_USING_PWM12
//    PWM12_CONFIG,
//#endif
//
//#ifdef BSP_USING_PWM13
//    PWM13_CONFIG,
//#endif
//
//#ifdef BSP_USING_PWM14
//    PWM14_CONFIG,
//#endif
//
//#ifdef BSP_USING_PWM15
//    PWM15_CONFIG,
//#endif
//
//#ifdef BSP_USING_PWM16
//    PWM16_CONFIG,
//#endif
//
//#ifdef BSP_USING_PWM17
//    PWM17_CONFIG,
//#endif
//};

///* APBx timer clocks frequency doubler state related to APB1CLKDivider value */
//static void pclkx_doubler_get(rt_uint32_t *pclk1_doubler, rt_uint32_t *pclk2_doubler)
//{
//    rt_uint32_t flatency = 0;
//    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
//
//    RT_ASSERT(pclk1_doubler != RT_NULL);
//    RT_ASSERT(pclk1_doubler != RT_NULL);
//
//    HAL_RCC_GetClockConfig(&RCC_ClkInitStruct, &flatency);
//
//    *pclk1_doubler = 1;
//    *pclk2_doubler = 1;
//
//#if defined(SOC_SERIES_STM32MP1)
//    if (RCC_ClkInitStruct.APB1_Div != RCC_APB1_DIV1)
//    {
//        *pclk1_doubler = 2;
//    }
//    if (RCC_ClkInitStruct.APB2_Div != RCC_APB2_DIV1)
//    {
//       *pclk2_doubler = 2;
//    }
//#else
//    if (RCC_ClkInitStruct.APB1CLKDivider != RCC_HCLK_DIV1)
//    {
//         *pclk1_doubler = 2;
//    }
//#if !defined(SOC_SERIES_STM32F0) && !defined(SOC_SERIES_STM32G0)
//    if (RCC_ClkInitStruct.APB2CLKDivider != RCC_HCLK_DIV1)
//    {
//         *pclk2_doubler = 2;
//    }
//#endif
//#endif
//}
//
//static rt_err_t drv_pwm_control(struct rt_device_pwm *device, int cmd, void *arg);
//static struct rt_pwm_ops drv_ops =
//{
//    drv_pwm_control
//};
//
//static rt_err_t drv_pwm_enable(TIM_HandleTypeDef *htim, struct rt_pwm_configuration *configuration, rt_bool_t enable)
//{
//    /* Converts the channel number to the channel number of Hal library */
//    rt_uint32_t channel = 0x04 * (configuration->channel - 1);
//
//    if (!configuration->complementary)
//    {
//        if (!enable)
//        {
//            HAL_TIM_PWM_Stop(htim, channel);
//        }
//        else
//        {
//            HAL_TIM_PWM_Start(htim, channel);
//        }
//    }
//    else if (configuration->complementary)
//    {
//        if (!enable)
//        {
//            HAL_TIMEx_PWMN_Stop(htim, channel);
//        }
//        else
//        {
//            HAL_TIMEx_PWMN_Start(htim, channel);
//        }
//    }
//
//    return RT_EOK;
//}
//
//static rt_err_t drv_pwm_get(TIM_HandleTypeDef *htim, struct rt_pwm_configuration *configuration)
//{
//    /* Converts the channel number to the channel number of Hal library */
//    rt_uint32_t channel = 0x04 * (configuration->channel - 1);
//    rt_uint64_t tim_clock;
//    rt_uint32_t pclk1_doubler, pclk2_doubler;
//
//    pclkx_doubler_get(&pclk1_doubler, &pclk2_doubler);
//
//#if defined(SOC_SERIES_STM32F2) || defined(SOC_SERIES_STM32F4) || defined(SOC_SERIES_STM32F7)
//    if (htim->Instance == TIM9 || htim->Instance == TIM10 || htim->Instance == TIM11)
//#elif defined(SOC_SERIES_STM32L4) || defined(SOC_SERIES_STM32H7)
//    if (htim->Instance == TIM15 || htim->Instance == TIM16 || htim->Instance == TIM17)
//#elif defined(SOC_SERIES_STM32MP1)
//    if (htim->Instance == TIM4)
//#elif defined(SOC_SERIES_STM32F1) || defined(SOC_SERIES_STM32F0) || defined(SOC_SERIES_STM32G0)
//    if (0)
//#endif
//    {
//#if !defined(SOC_SERIES_STM32F0) && !defined(SOC_SERIES_STM32G0)
//        tim_clock = (rt_uint32_t)(HAL_RCC_GetPCLK2Freq() * pclk2_doubler);
//#endif
//    }
//    else
//    {
//        tim_clock = (rt_uint32_t)(HAL_RCC_GetPCLK1Freq() * pclk1_doubler);
//    }
//
//    if (__HAL_TIM_GET_CLOCKDIVISION(htim) == TIM_CLOCKDIVISION_DIV2)
//    {
//        tim_clock = tim_clock / 2;
//    }
//    else if (__HAL_TIM_GET_CLOCKDIVISION(htim) == TIM_CLOCKDIVISION_DIV4)
//    {
//        tim_clock = tim_clock / 4;
//    }
//
//    /* Convert nanosecond to frequency and duty cycle. 1s = 1 * 1000 * 1000 * 1000 ns */
//    tim_clock /= 1000000UL;
//    configuration->period = (__HAL_TIM_GET_AUTORELOAD(htim) + 1) * (htim->Instance->PSC + 1) * 1000UL / tim_clock;
//    configuration->pulse = (__HAL_TIM_GET_COMPARE(htim, channel) + 1) * (htim->Instance->PSC + 1) * 1000UL / tim_clock;
//
//    return RT_EOK;
//}
//
//static rt_err_t drv_pwm_set(TIM_HandleTypeDef *htim, struct rt_pwm_configuration *configuration)
//{
//    rt_uint32_t period, pulse;
//    rt_uint64_t tim_clock, psc;
//    rt_uint32_t pclk1_doubler, pclk2_doubler;
//    /* Converts the channel number to the channel number of Hal library */
//    rt_uint32_t channel = 0x04 * (configuration->channel - 1);
//
//    pclkx_doubler_get(&pclk1_doubler, &pclk2_doubler);
//
//#if defined(SOC_SERIES_STM32F2) || defined(SOC_SERIES_STM32F4) || defined(SOC_SERIES_STM32F7)
//    if (htim->Instance == TIM9 || htim->Instance == TIM10 || htim->Instance == TIM11)
//#elif defined(SOC_SERIES_STM32L4) || defined(SOC_SERIES_STM32H7)|| defined(SOC_SERIES_STM32F3)
//    if (htim->Instance == TIM15 || htim->Instance == TIM16 || htim->Instance == TIM17)
//#elif defined(SOC_SERIES_STM32MP1)
//    if (htim->Instance == TIM4)
//#elif defined(SOC_SERIES_STM32F1) || defined(SOC_SERIES_STM32F0) || defined(SOC_SERIES_STM32G0)
//    if (0)
//#endif
//    {
//#if !defined(SOC_SERIES_STM32F0) && !defined(SOC_SERIES_STM32G0)
//        tim_clock = (rt_uint32_t)(HAL_RCC_GetPCLK2Freq() * pclk2_doubler);
//#endif
//    }
//    else
//    {
//        tim_clock = (rt_uint32_t)(HAL_RCC_GetPCLK1Freq() * pclk1_doubler);
//    }
//
//    /* Convert nanosecond to frequency and duty cycle. 1s = 1 * 1000 * 1000 * 1000 ns */
//    tim_clock /= 1000000UL;
//    period = (unsigned long long)configuration->period * tim_clock / 1000ULL ;
//    psc = period / MAX_PERIOD + 1;
//    period = period / psc;
//    __HAL_TIM_SET_PRESCALER(htim, psc - 1);
//
//    if (period < MIN_PERIOD)
//    {
//        period = MIN_PERIOD;
//    }
//    __HAL_TIM_SET_AUTORELOAD(htim, period - 1);
//
//    pulse = (unsigned long long)configuration->pulse * tim_clock / psc / 1000ULL;
//    if (pulse < MIN_PULSE)
//    {
//        pulse = MIN_PULSE;
//    }
//    else if (pulse > period)
//    {
//        pulse = period;
//    }
//    __HAL_TIM_SET_COMPARE(htim, channel, pulse - 1);
//    __HAL_TIM_SET_COUNTER(htim, 0);
//
//    /* Update frequency value */
//    HAL_TIM_GenerateEvent(htim, TIM_EVENTSOURCE_UPDATE);
//
//    return RT_EOK;
//}
//
//static rt_err_t drv_pwm_control(struct rt_device_pwm *device, int cmd, void *arg)
//{
//    struct rt_pwm_configuration *configuration = (struct rt_pwm_configuration *)arg;
//    TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)device->parent.user_data;
//
//    switch (cmd)
//    {
//    case PWMN_CMD_ENABLE:
//        configuration->complementary = RT_TRUE;
//    case PWM_CMD_ENABLE:
//        return drv_pwm_enable(htim, configuration, RT_TRUE);
//    case PWMN_CMD_DISABLE:
//        configuration->complementary = RT_FALSE;
//    case PWM_CMD_DISABLE:
//        return drv_pwm_enable(htim, configuration, RT_FALSE);
//    case PWM_CMD_SET:
//        return drv_pwm_set(htim, configuration);
//    case PWM_CMD_GET:
//        return drv_pwm_get(htim, configuration);
//    default:
//        return RT_EINVAL;
//    }
//}
//
//static rt_err_t rt_hw_pwm_init_(struct hc32_pwm *device)
//{
//    rt_err_t result = RT_EOK;
//    TIM_HandleTypeDef *tim = RT_NULL;
//    TIM_OC_InitTypeDef oc_config = {0};
//    TIM_MasterConfigTypeDef master_config = {0};
//    TIM_ClockConfigTypeDef clock_config = {0};
//
//    RT_ASSERT(device != RT_NULL);
//
//    tim = (TIM_HandleTypeDef *)&device->tim_handle;
//
//    /* configure the timer to pwm mode */
//    tim->Init.Prescaler = 0;
//    tim->Init.CounterMode = TIM_COUNTERMODE_UP;
//    tim->Init.Period = 0;
//    tim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//#if defined(SOC_SERIES_STM32F1) || defined(SOC_SERIES_STM32L4)
//    tim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//#endif
//
//    if (HAL_TIM_Base_Init(tim) != HAL_OK)
//    {
//        LOG_E("%s pwm init failed", device->name);
//        result = -RT_ERROR;
//        goto __exit;
//    }
//    if (HAL_TIM_PWM_Init(tim) != HAL_OK)
//    {
//        LOG_E("%s pwm init failed", device->name);
//        result = -RT_ERROR;
//        goto __exit;
//    }
//
//    clock_config.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
//    if (HAL_TIM_ConfigClockSource(tim, &clock_config) != HAL_OK)
//    {
//        LOG_E("%s clock init failed", device->name);
//        result = -RT_ERROR;
//        goto __exit;
//    }
//
//    master_config.MasterOutputTrigger = TIM_TRGO_RESET;
//    master_config.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//    if (HAL_TIMEx_MasterConfigSynchronization(tim, &master_config) != HAL_OK)
//    {
//        LOG_E("%s master config failed", device->name);
//        result = -RT_ERROR;
//        goto __exit;
//    }
//
//    oc_config.OCMode = TIM_OCMODE_PWM1;
//    oc_config.Pulse = 0;
////    oc_config.OCPolarity = TIM_OCPOLARITY_HIGH;
//    oc_config.OCPolarity = TIM_OCPOLARITY_LOW;
//    oc_config.OCFastMode = TIM_OCFAST_DISABLE;
//    oc_config.OCNIdleState = TIM_OCNIDLESTATE_RESET;
//    oc_config.OCIdleState  = TIM_OCIDLESTATE_RESET;
//
//    /* config pwm channel */
//    if (device->channel & 0x01)
//    {
//        if (HAL_TIM_PWM_ConfigChannel(tim, &oc_config, TIM_CHANNEL_1) != HAL_OK)
//        {
//            LOG_E("%s channel1 config failed", device->name);
//            result = -RT_ERROR;
//            goto __exit;
//        }
//    }
//
//    if (device->channel & 0x02)
//    {
//        if (HAL_TIM_PWM_ConfigChannel(tim, &oc_config, TIM_CHANNEL_2) != HAL_OK)
//        {
//            LOG_E("%s channel2 config failed", device->name);
//            result = -RT_ERROR;
//            goto __exit;
//        }
//    }
//
//    if (device->channel & 0x04)
//    {
//        if (HAL_TIM_PWM_ConfigChannel(tim, &oc_config, TIM_CHANNEL_3) != HAL_OK)
//        {
//            LOG_E("%s channel3 config failed", device->name);
//            result = -RT_ERROR;
//            goto __exit;
//        }
//    }
//
//    if (device->channel & 0x08)
//    {
//        if (HAL_TIM_PWM_ConfigChannel(tim, &oc_config, TIM_CHANNEL_4) != HAL_OK)
//        {
//            LOG_E("%s channel4 config failed", device->name);
//            result = -RT_ERROR;
//            goto __exit;
//        }
//    }
//
//    /* pwm pin configuration */
//    HAL_TIM_MspPostInit(tim);
//
//    /* enable update request source */
//    __HAL_TIM_URS_ENABLE(tim);
//
//__exit:
//    return result;
//}
//
//static void pwm_get_channel(void)
//{
//#ifdef BSP_USING_PWM1_CH1
//    hc32_pwm_obj[PWM1_INDEX].channel |= 1 << 0;
//#endif
//#ifdef BSP_USING_PWM1_CH2
//    hc32_pwm_obj[PWM1_INDEX].channel |= 1 << 1;
//#endif
//#ifdef BSP_USING_PWM1_CH3
//    hc32_pwm_obj[PWM1_INDEX].channel |= 1 << 2;
//#endif
//#ifdef BSP_USING_PWM1_CH4
//    hc32_pwm_obj[PWM1_INDEX].channel |= 1 << 3;
//#endif
//#ifdef BSP_USING_PWM2_CH1
//    hc32_pwm_obj[PWM2_INDEX].channel |= 1 << 0;
//#endif
//#ifdef BSP_USING_PWM2_CH2
//    hc32_pwm_obj[PWM2_INDEX].channel |= 1 << 1;
//#endif
//#ifdef BSP_USING_PWM2_CH3
//    hc32_pwm_obj[PWM2_INDEX].channel |= 1 << 2;
//#endif
//#ifdef BSP_USING_PWM2_CH4
//    hc32_pwm_obj[PWM2_INDEX].channel |= 1 << 3;
//#endif
//#ifdef BSP_USING_PWM3_CH1
//    hc32_pwm_obj[PWM3_INDEX].channel |= 1 << 0;
//#endif
//#ifdef BSP_USING_PWM3_CH2
//    hc32_pwm_obj[PWM3_INDEX].channel |= 1 << 1;
//#endif
//#ifdef BSP_USING_PWM3_CH3
//    hc32_pwm_obj[PWM3_INDEX].channel |= 1 << 2;
//#endif
//#ifdef BSP_USING_PWM3_CH4
//    hc32_pwm_obj[PWM3_INDEX].channel |= 1 << 3;
//#endif
//#ifdef BSP_USING_PWM4_CH1
//    hc32_pwm_obj[PWM4_INDEX].channel |= 1 << 0;
//#endif
//#ifdef BSP_USING_PWM4_CH2
//    hc32_pwm_obj[PWM4_INDEX].channel |= 1 << 1;
//#endif
//#ifdef BSP_USING_PWM4_CH3
//    hc32_pwm_obj[PWM4_INDEX].channel |= 1 << 2;
//#endif
//#ifdef BSP_USING_PWM4_CH4
//    hc32_pwm_obj[PWM4_INDEX].channel |= 1 << 3;
//#endif
//#ifdef BSP_USING_PWM5_CH1
//    hc32_pwm_obj[PWM5_INDEX].channel |= 1 << 0;
//#endif
//#ifdef BSP_USING_PWM5_CH2
//    hc32_pwm_obj[PWM5_INDEX].channel |= 1 << 1;
//#endif
//#ifdef BSP_USING_PWM5_CH3
//    hc32_pwm_obj[PWM5_INDEX].channel |= 1 << 2;
//#endif
//#ifdef BSP_USING_PWM5_CH4
//    hc32_pwm_obj[PWM5_INDEX].channel |= 1 << 3;
//#endif
//#ifdef BSP_USING_PWM6_CH1
//    hc32_pwm_obj[PWM6_INDEX].channel |= 1 << 0;
//#endif
//#ifdef BSP_USING_PWM6_CH2
//    hc32_pwm_obj[PWM6_INDEX].channel |= 1 << 1;
//#endif
//#ifdef BSP_USING_PWM6_CH3
//    hc32_pwm_obj[PWM6_INDEX].channel |= 1 << 2;
//#endif
//#ifdef BSP_USING_PWM6_CH4
//    hc32_pwm_obj[PWM6_INDEX].channel |= 1 << 3;
//#endif
//#ifdef BSP_USING_PWM7_CH1
//    hc32_pwm_obj[PWM7_INDEX].channel |= 1 << 0;
//#endif
//#ifdef BSP_USING_PWM7_CH2
//    hc32_pwm_obj[PWM7_INDEX].channel |= 1 << 1;
//#endif
//#ifdef BSP_USING_PWM7_CH3
//    hc32_pwm_obj[PWM7_INDEX].channel |= 1 << 2;
//#endif
//#ifdef BSP_USING_PWM7_CH4
//    hc32_pwm_obj[PWM7_INDEX].channel |= 1 << 3;
//#endif
//#ifdef BSP_USING_PWM8_CH1
//    hc32_pwm_obj[PWM8_INDEX].channel |= 1 << 0;
//#endif
//#ifdef BSP_USING_PWM8_CH2
//    hc32_pwm_obj[PWM8_INDEX].channel |= 1 << 1;
//#endif
//#ifdef BSP_USING_PWM8_CH3
//    hc32_pwm_obj[PWM8_INDEX].channel |= 1 << 2;
//#endif
//#ifdef BSP_USING_PWM8_CH4
//    hc32_pwm_obj[PWM8_INDEX].channel |= 1 << 3;
//#endif
//#ifdef BSP_USING_PWM9_CH1
//    hc32_pwm_obj[PWM9_INDEX].channel |= 1 << 0;
//#endif
//#ifdef BSP_USING_PWM9_CH2
//    hc32_pwm_obj[PWM9_INDEX].channel |= 1 << 1;
//#endif
//#ifdef BSP_USING_PWM9_CH3
//    hc32_pwm_obj[PWM9_INDEX].channel |= 1 << 2;
//#endif
//#ifdef BSP_USING_PWM9_CH4
//    hc32_pwm_obj[PWM9_INDEX].channel |= 1 << 3;
//#endif
//#ifdef BSP_USING_PWM12_CH1
//    hc32_pwm_obj[PWM12_INDEX].channel |= 1 << 0;
//#endif
//#ifdef BSP_USING_PWM12_CH2
//    hc32_pwm_obj[PWM12_INDEX].channel |= 1 << 1;
//#endif
//#ifdef BSP_USING_PWM16_CH1
//    hc32_pwm_obj[PWM16_INDEX].channel |= 1 << 0;
//#endif
//#ifdef BSP_USING_PWM17_CH1
//    hc32_pwm_obj[PWM17_INDEX].channel |= 1 << 0;
//#endif
//}
//
//static void _tmr_clock_enable(void)
//{
//#if defined(HC32F4A0)
//#if defined(BSP_USING_TMR0)
//    FCG_Fcg3PeriphClockCmd(FCG3_PERIPH_ADC1, ENABLE);
//#endif
//#if defined(BSP_USING_TMRA)
//    FCG_Fcg3PeriphClockCmd(FCG3_PERIPH_ADC2, ENABLE);
//#endif
//#if defined(BSP_USING_ADC3)
//    FCG_Fcg3PeriphClockCmd(FCG3_PERIPH_ADC3, ENABLE);
//#endif
//#endif
//
//#if defined(HC32F460)
//#if defined(BSP_USING_TMR4)
//    FCG_Fcg2PeriphClockCmd(FCG2_PERIPH_TMR4_1, ENABLE);
//    FCG_Fcg2PeriphClockCmd(FCG2_PERIPH_TMR4_2, ENABLE);
//    FCG_Fcg2PeriphClockCmd(FCG2_PERIPH_TMR4_3, ENABLE);
//#endif
//#if defined(BSP_USING_TMR6)
//    FCG_Fcg3PeriphClockCmd(FCG3_PERIPH_ADC2, ENABLE);
//#endif
//#endif
//}


//extern rt_err_t rt_hw_board_pwm_init(CM_TMR4_TypeDef *CM_TMRx);
static int rt_hw_pwm_init(void)
{
//    int ret, i = 0;
//    stc_tmr4_init_t stcTmr4Init = {0};
//    int32_t ll_ret = 0;
//
//    _tmr_clock_enable();
//    uint32_t dev_cnt = sizeof(g_pwm_dev_array) / sizeof(g_pwm_dev_array[0]);
//
//    for (; i < dev_cnt; i++)
//    {
//        TMR4_DeInit(g_pwm_dev_array[i].instance);
//        stcTmr4Init.u16ClockSrc
//        stcTmr4Init.u16ClockDiv = TMR4_CLK_DIV1024;
//        stcTmr4Init.u16CountMode = TMR4_MD_TRIANGLE;
//        stcTmr4Init.u16PeriodValue = ((uint16_t)(CLK_GetBusClockFreq(CLK_BUS_PCLK1) / (1UL << (uint32_t)(div)) / 4UL) - 1U);
//
//    }
//
//
//
//
//
//
//    int i = 0;
//    int result = RT_EOK;
//
//    pwm_get_channel();
//
//    for (i = 0; i < sizeof(hc32_pwm_obj) / sizeof(hc32_pwm_obj[0]); i++)
//    {
//        /* pwm init */
//        if (rt_hw_pwm_init_(&hc32_pwm_obj[i]) != RT_EOK)
//        {
//            LOG_E("%s init failed", hc32_pwm_obj[i].name);
//            result = -RT_ERROR;
//            goto __exit;
//        }
//        else
//        {
//            LOG_D("%s init success", hc32_pwm_obj[i].name);
//
//            /* register pwm device */
//            if (rt_device_pwm_register(&hc32_pwm_obj[i].pwm_device, hc32_pwm_obj[i].name, &drv_ops, &hc32_pwm_obj[i].tim_handle) == RT_EOK)
//            {
//                LOG_D("%s register success", hc32_pwm_obj[i].name);
//            }
//            else
//            {
//                LOG_E("%s register failed", hc32_pwm_obj[i].name);
//                result = -RT_ERROR;
//            }
//        }
//    }
//
//__exit:
//    return result;


    //  hc32_ll 底层驱动实现 tmr4的初始化，未嵌入式  rtt框架


    // 2. 定义PWM频率相关参数（10kHz）
    #define PWM_FREQ_HZ    (10000UL)
    #define TMR4_CLK_DIV   (TMR4_CLK_DIV16)  // 分频16
    #define DIV_SHIFT      (4UL)             // 1<<4=16，匹配分频系数

    // 3. OC通道到PWM通道的映射宏
    #define TMR4_PWM_CH(x)                  (((x) <= TMR4_OC_CH_UL) ? TMR4_PWM_CH_U : \
                                            (((x) <= TMR4_OC_CH_VL) ? TMR4_PWM_CH_V : TMR4_PWM_CH_W))

    uint32_t u32PwmCh;


//    stc_gpio_init_t stcGpioInit;
//    stc_jtag_cfg_t stcJtagCfg;
//
//    // 1. 使能GPIO和JTAG时钟
//    PWC_Fcg1PeriphClockCmd(PWC_FCG1_PERIPH_GPIO, ENABLE);
//    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_JTAG, ENABLE);
//
//    // 2. 禁用JTAG的nTRST功能（关键步骤）
//    MEM_ZERO_STRUCT(stcJtagCfg);
//    // 配置JTAG模式为：禁用nTRST，仅保留SWD（不影响调试）
//    stcJtagCfg.u8JtagMode = JTAG_MODE_SWD_ONLY;  // 仅开启SWD，关闭JTAG的nTRST/TMS/TCK/TDI
//    // 或单独禁用nTRST：stcJtagCfg.u8JtagNtrst = JTAG_NTRST_DISABLE;
//    JTAG_Config(&stcJtagCfg);

    GPIO_SetDebugPort(GPIO_PIN_TRST|GPIO_PIN_TDI|GPIO_PIN_TDO, DISABLE);



    // 三相高低侧OC通道定义
    const uint32_t u32Oc_UHCh = TMR4_OC_CH_UH;     // 对应 硬件 通道  MOS_LOU_EN
    const uint32_t u32Oc_ULCh = TMR4_OC_CH_UL;     // 对应 硬件 通道  MOS_HOU_EN

    const uint32_t u32Oc_VHCh = TMR4_OC_CH_VH;     // 对应 硬件 通道 MOS_HOV_EN
    const uint32_t u32Oc_VLCh = TMR4_OC_CH_VL;     // 对应 硬件 通道 MOS_LOV_EN

    const uint32_t u32Oc_WHCh = TMR4_OC_CH_WH;     // 对应 硬件 通道 MOS_HOW_EN
    const uint32_t u32Oc_WLCh = TMR4_OC_CH_WL;     // 对应 硬件 通道 MOS_LOW_EN

    // 驱动结构体定义
    stc_tmr4_init_t stcTmr4Init;
    stc_tmr4_oc_init_t stcTmr4OcInit;
    stc_tmr4_pwm_init_t stcTmr4PwmInit;
    un_tmr4_oc_ocmrh_t unTmr4OcOcmrh;
    un_tmr4_oc_ocmrl_t unTmr4OcOcmrl;

    /* 1. 初始化PWM I/O（硬件匹配） */
    GPIO_SetFunc(GPIO_PORT_B, GPIO_PIN_04, GPIO_FUNC_2);   // UL
    GPIO_SetFunc(GPIO_PORT_B, GPIO_PIN_05, GPIO_FUNC_2);   // UH
    GPIO_SetFunc(GPIO_PORT_B, GPIO_PIN_06, GPIO_FUNC_2);   // VL
    GPIO_SetFunc(GPIO_PORT_B, GPIO_PIN_07, GPIO_FUNC_2);   // VH
    GPIO_SetFunc(GPIO_PORT_B, GPIO_PIN_08, GPIO_FUNC_2);   // WH
    GPIO_SetFunc(GPIO_PORT_B, GPIO_PIN_09, GPIO_FUNC_2);   // WL

    /* 2. 使能TMR4_3外设时钟 */
    FCG_Fcg2PeriphClockCmd(FCG2_PERIPH_TMR4_3, ENABLE);

    /************************* 配置TMR4计数器 *************************/
    (void)TMR4_StructInit(&stcTmr4Init);
    stcTmr4Init.u16ClockDiv = TMR4_CLK_DIV;                // 分频16
    stcTmr4Init.u16CountMode = TMR4_MD_TRIANGLE;           // 三角波模式

    // 周期值计算（精准+防溢出）
    uint32_t u32Pclk1Freq = CLK_GetBusClockFreq(CLK_BUS_PCLK1);

    //  u32Period 实时计算值 == 311
    uint32_t u32Period = (u32Pclk1Freq / (1UL << DIV_SHIFT) / PWM_FREQ_HZ /2) - 1U;
    stcTmr4Init.u16PeriodValue = (u32Period > 0xFFFF) ? 0xFFFF : (uint16_t)u32Period;
    (void)TMR4_Init(CM_TMR4_3, &stcTmr4Init);

    /************************* 配置TMR4输出比较 ******************/
    (void)TMR4_OC_StructInit(&stcTmr4OcInit);
    stcTmr4OcInit.u16CompareValue = (stcTmr4Init.u16PeriodValue / 2U);  // 50%占空比
    stcTmr4OcInit.u16CompareValueBufCond = TMR4_OC_BUF_COND_PEAK;       // 峰值更新缓存

    // 初始化所有OC通道
    (void)TMR4_OC_Init(CM_TMR4_3, u32Oc_UHCh, &stcTmr4OcInit);
    (void)TMR4_OC_Init(CM_TMR4_3, u32Oc_ULCh, &stcTmr4OcInit);
    (void)TMR4_OC_Init(CM_TMR4_3, u32Oc_VHCh, &stcTmr4OcInit);
    (void)TMR4_OC_Init(CM_TMR4_3, u32Oc_VLCh, &stcTmr4OcInit);
    (void)TMR4_OC_Init(CM_TMR4_3, u32Oc_WHCh, &stcTmr4OcInit);
    (void)TMR4_OC_Init(CM_TMR4_3, u32Oc_WLCh, &stcTmr4OcInit);

    /* 配置高通道OC模式 */
    unTmr4OcOcmrh.OCMRx_f.OCFDCH = TMR4_OC_OCF_SET; /* bit[0]     1  */
    unTmr4OcOcmrh.OCMRx_f.OCFPKH = TMR4_OC_OCF_SET; /* bit[1]     1  */
    unTmr4OcOcmrh.OCMRx_f.OCFUCH = TMR4_OC_OCF_SET; /* bit[2]     1  */
    unTmr4OcOcmrh.OCMRx_f.OCFZRH = TMR4_OC_OCF_SET; /* bit[3]     1  */

    unTmr4OcOcmrh.OCMRx_f.OPDCH  = TMR4_OC_HIGH;    /* Bit[5:4]   01 */
    unTmr4OcOcmrh.OCMRx_f.OPPKH  = TMR4_OC_HIGH;    /* Bit[7:6]   01 */
    unTmr4OcOcmrh.OCMRx_f.OPUCH  = TMR4_OC_LOW;     /* Bit[9:8]   10 */
    unTmr4OcOcmrh.OCMRx_f.OPZRH  = TMR4_OC_HOLD;    /* Bit[11:10] 00 */

    unTmr4OcOcmrh.OCMRx_f.OPNPKH = TMR4_OC_LOW;     /* Bit[13:12] 10 */
    unTmr4OcOcmrh.OCMRx_f.OPNZRH = TMR4_OC_HOLD;    /* Bit[15:14] 00 */
    TMR4_OC_SetHighChCompareMode(CM_TMR4_3, u32Oc_UHCh, unTmr4OcOcmrh);
    TMR4_OC_SetHighChCompareMode(CM_TMR4_3, u32Oc_VHCh, unTmr4OcOcmrh);
    TMR4_OC_SetHighChCompareMode(CM_TMR4_3, u32Oc_WHCh, unTmr4OcOcmrh);

    /* 配置低通道OC模式（注释完全匹配官方结构体） */
    unTmr4OcOcmrl.OCMRx_f.OCFDCL  = TMR4_OC_OCF_SET; /* bit[0]     1  */
    unTmr4OcOcmrl.OCMRx_f.OCFPKL  = TMR4_OC_OCF_SET; /* bit[1]     1  */
    unTmr4OcOcmrl.OCMRx_f.OCFUCL  = TMR4_OC_OCF_SET; /* bit[2]     1  */
    unTmr4OcOcmrl.OCMRx_f.OCFZRL  = TMR4_OC_OCF_SET; /* bit[3]     1  */

    unTmr4OcOcmrl.OCMRx_f.OPDCL   = TMR4_OC_HIGH;    /* bit[5:4]   01 */
    unTmr4OcOcmrl.OCMRx_f.OPPKL   = TMR4_OC_HIGH;    /* Bit[7:6]   01 */
    unTmr4OcOcmrl.OCMRx_f.OPUCL   = TMR4_OC_LOW;     /* bit[9:8]   10 */
    unTmr4OcOcmrl.OCMRx_f.OPZRL   = TMR4_OC_HOLD;    /* Bit[11:10] 00 */

    unTmr4OcOcmrl.OCMRx_f.OPNPKL  = TMR4_OC_LOW;     /* Bit[13:12] 10 */
    unTmr4OcOcmrl.OCMRx_f.OPNZRL  = TMR4_OC_HOLD;    /* bit[15:14] 00 */

    unTmr4OcOcmrl.OCMRx_f.EOPNDCL = TMR4_OC_HOLD;    /* bit[17:16] 00 */
    unTmr4OcOcmrl.OCMRx_f.EOPNUCL = TMR4_OC_HOLD;    /* bit[19:18] 00 */
    unTmr4OcOcmrl.OCMRx_f.EOPDCL  = TMR4_OC_HIGH;    /* bit[21:20] 01 */
    unTmr4OcOcmrl.OCMRx_f.EOPPKL  = TMR4_OC_HIGH;    /* Bit[23:22] 01 */  // 修正注释
    unTmr4OcOcmrl.OCMRx_f.EOPUCL  = TMR4_OC_LOW;     /* bit[25:24] 10 */
    unTmr4OcOcmrl.OCMRx_f.EOPZRL  = TMR4_OC_HOLD;    /* Bit[27:26] 00 */  // 修正注释
    unTmr4OcOcmrl.OCMRx_f.EOPNPKL = TMR4_OC_LOW;     /* Bit[29:28] 10 */  // 修正注释
    unTmr4OcOcmrl.OCMRx_f.EOPNZRL = TMR4_OC_HOLD;    /* bit[31:30] 00 */
    TMR4_OC_SetLowChCompareMode(CM_TMR4_3, u32Oc_ULCh, unTmr4OcOcmrl);
    TMR4_OC_SetLowChCompareMode(CM_TMR4_3, u32Oc_VLCh, unTmr4OcOcmrl);
    TMR4_OC_SetLowChCompareMode(CM_TMR4_3, u32Oc_WLCh, unTmr4OcOcmrl);

    uint16_t compare = u32Period * 0.02;

    TMR4_OC_SetCompareValue(CM_TMR4_3, u32Oc_UHCh, compare);
    TMR4_OC_SetCompareValue(CM_TMR4_3, u32Oc_ULCh, compare);

    TMR4_OC_SetCompareValue(CM_TMR4_3, u32Oc_VHCh, compare);
    TMR4_OC_SetCompareValue(CM_TMR4_3, u32Oc_VLCh, compare);

    TMR4_OC_SetCompareValue(CM_TMR4_3, u32Oc_WHCh, compare);
    TMR4_OC_SetCompareValue(CM_TMR4_3, u32Oc_WLCh, compare);


    /* 使能所有OC通道 */
    TMR4_OC_Cmd(CM_TMR4_3, u32Oc_UHCh, ENABLE);
    TMR4_OC_Cmd(CM_TMR4_3, u32Oc_ULCh, ENABLE);

    TMR4_OC_Cmd(CM_TMR4_3, u32Oc_VHCh, ENABLE);
    TMR4_OC_Cmd(CM_TMR4_3, u32Oc_VLCh, ENABLE);

    TMR4_OC_Cmd(CM_TMR4_3, u32Oc_WHCh, ENABLE);
    TMR4_OC_Cmd(CM_TMR4_3, u32Oc_WLCh, ENABLE);

    /************************* 配置并使能TMR4 PWM *****************************/
    // 初始化U相PWM
//    u32PwmCh = TMR4_PWM_CH(u32Oc_UHCh);
//    (void)TMR4_PWM_StructInit(&stcTmr4PwmInit);
//    stcTmr4PwmInit.u16Mode = TMR4_PWM_MD_THROUGH;
//    (void)TMR4_PWM_Init(CM_TMR4_3, u32PwmCh, &stcTmr4PwmInit);
//
//    u32PwmCh = TMR4_PWM_CH(u32Oc_ULCh);
//    (void)TMR4_PWM_Init(CM_TMR4_3, u32PwmCh, &stcTmr4PwmInit);

//    // 初始化V相PWM
//    u32PwmCh = TMR4_PWM_CH(u32Oc_VHCh);
//    (void)TMR4_PWM_Init(CM_TMR4_3, u32PwmCh, &stcTmr4PwmInit);
//
//    u32PwmCh = TMR4_PWM_CH(u32Oc_VLCh);
//    (void)TMR4_PWM_Init(CM_TMR4_3, u32PwmCh, &stcTmr4PwmInit);
//
//
//    // 初始化W相PWM
//    u32PwmCh = TMR4_PWM_CH(u32Oc_WHCh);
//    (void)TMR4_PWM_Init(CM_TMR4_3, u32PwmCh, &stcTmr4PwmInit);
//
//    u32PwmCh = TMR4_PWM_CH(u32Oc_WLCh);
//    (void)TMR4_PWM_Init(CM_TMR4_3, u32PwmCh, &stcTmr4PwmInit);


//    /* 启动TMR4计数，输出PWM */
//    TMR4_Start(CM_TMR4_3);

}
INIT_DEVICE_EXPORT(rt_hw_pwm_init);
#endif /* RT_USING_PWM */
