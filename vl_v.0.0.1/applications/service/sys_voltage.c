/**
 * @file    sys_voltage.c
 * @brief   系统（母线）电压采集实现
 */

#include "sys_voltage.h"

#include <rtdevice.h>
#include <rtthread.h>

#include "hc32_ll.h"
#include "hc32_ll_adc.h"

/* ============================================================
 * 模块状态
 * ============================================================ */
static rt_adc_device_t s_dev = RT_NULL;

int sys_voltage_init(void)
{
    rt_err_t ret;

    s_dev = (rt_adc_device_t)rt_device_find(SYS_VOLT_ADC_DEV_NAME);
    if (s_dev == RT_NULL) {
        /* 电压只是启车条件之一，采集不可用不应阻断启动，
         * 由 vehicle_poll 通过 sys_voltage_is_ready() 跳过该条件。 */
        rt_kprintf("[SYS-VOLT] WARN: device '%s' not found, voltage unavailable\n",
                   SYS_VOLT_ADC_DEV_NAME);
        return 0;
    }

    /* 采样时间：复位默认约 4 ADCLK，对 100k/3.3k 高阻分压源偏短 */
    LL_PERIPH_WE(LL_PERIPH_ALL);
    ADC_SetSampleTime(CM_ADC2, SYS_VOLT_ADC_CHANNEL, SYS_VOLT_ADC_SAMPLE_CLK);
    LL_PERIPH_WP(LL_PERIPH_ALL);

    ret = rt_adc_enable(s_dev, SYS_VOLT_ADC_CHANNEL);
    if (ret != RT_EOK) {
        rt_kprintf("[SYS-VOLT] WARN: enable ch%u failed (%d), voltage unavailable\n",
                   (unsigned)SYS_VOLT_ADC_CHANNEL, ret);
        s_dev = RT_NULL;
        return 0;
    }

    rt_kprintf("[SYS-VOLT] %s ch%u enabled (PA6/ADC12_IN6), div=100k/3.3k\n",
               SYS_VOLT_ADC_DEV_NAME, (unsigned)SYS_VOLT_ADC_CHANNEL);
    return 0;
}

uint8_t sys_voltage_is_ready(void)
{
    return (s_dev != RT_NULL) ? 1U : 0U;
}

uint16_t sys_voltage_read_pin_mv(void)
{
    uint32_t sum = 0U;
    uint32_t raw;
    uint32_t n;
    uint32_t mv;

    if (s_dev == RT_NULL) {
        return 0U;
    }

    /* 过采样求平均：抑制单次异常值与随机噪声 */
    n = (SYS_VOLT_ADC_OVERSAMPLE > 0U) ? SYS_VOLT_ADC_OVERSAMPLE : 1U;
    for (uint32_t i = 0U; i < n; i++) {
        uint32_t v = rt_adc_read(s_dev, SYS_VOLT_ADC_CHANNEL);
        if (v > SYS_VOLT_ADC_RAW_MAX) {
            v = SYS_VOLT_ADC_RAW_MAX;       /* 12bit 上限保护 */
        }
        sum += v;
    }
    raw = sum / n;

    mv = (raw * (uint32_t)SYS_VOLT_ADC_VREF_MV) / (uint32_t)SYS_VOLT_ADC_FULL_SCALE;
    if (mv > 65535U) {
        mv = 65535U;
    }
    return (uint16_t)mv;
}

uint32_t sys_voltage_read_bus_mv(void)
{
    uint32_t pin_mv = (uint32_t)sys_voltage_read_pin_mv();

    /* 还原分压：mv_bus = mv_pin × 1033 / 33 */
    return (pin_mv * (uint32_t)SYS_VOLT_DIV_NUM) / (uint32_t)SYS_VOLT_DIV_DEN;
}

uint16_t sys_voltage_get_dv(void)
{
    uint32_t bus_mv = sys_voltage_read_bus_mv();

    return (uint16_t)(bus_mv / 100U);
}
