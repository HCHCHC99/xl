/**
 * @file    lift_adc.c
 * @brief   举升角 ADC 采集实现（基于 RT-Thread ADC 框架）
 *
 * 【为何自行换算而不调用 rt_adc_voltage()】
 *   drv_adc.c:197-201 的 g_adc_ops 只填了 enabled / convert，
 *   get_resolution 与 get_vref 均为 NULL，
 *   导致 misc/adc.c:174/180 的 _adc_control 返回 -RT_EINVAL，
 *   rt_adc_voltage() 必然走 goto _voltage_exit 返回 0（N-09）。
 *
 * 【为何需要过采样】
 *   rt_adc_read() 不检查 convert 的返回值（misc/adc.c:124），
 *   超时会返回未初始化值（N-10）。过采样 + 均值可同时抑制
 *   单次异常值与随机噪声。
 *
 * 【为何需要设采样时间】
 *   本工程从未调用 ADC_SetSampleTime()（N-13），
 *   复位默认采样时间约 4 个 ADCLK，对分压后高阻抗源偏短。
 *   此处显式延长，保证采样充分。
 */
#include "lift_adc.h"
#include "framework/state_engine/sm_assert.h"
#include <rtdevice.h>
#include <rtthread.h>

/* HC32 LL 层：用于设置采样时间 */
#include "hc32_ll.h"
#include "hc32_ll_adc.h"

/* ============================================================
 * 模块状态
 * ============================================================ */
static rt_adc_device_t s_dev = RT_NULL;

/* hc32_ll_adc.c:123  IS_ADC_SAMPLE_TIME(x) → ((x) >= 5U)，无枚举名 */
/* 通道 3 < ADC_SSTR_NUM(16) → 写入 SSTR0+3，单个 8bit 采样时间寄存器 */
#define LIFT_ADC_SAMPLE_TIME_CLK   40U    /* 40 个 ADCLK，适配分压源阻抗 */

/* ============================================================
 * 初始化
 * ============================================================ */
int lift_adc_init(void)
{
    rt_err_t ret;

    /* ---- 1. 查找 ADC 设备（由 drv_adc.c 自动注册） ---- */
    s_dev = (rt_adc_device_t)rt_device_find(LIFT_ADC_DEV_NAME);
    if (s_dev == RT_NULL) {
        rt_kprintf("[LiftADC] device '%s' not found\n", LIFT_ADC_DEV_NAME);
        return -1;
    }

    /* ---- 2. 设置采样时间（缓解 N-13） ---- */
    /* ADC_SAMPLE_TIME 枚举：ADCLK 个数，选较长时间以保证高阻源充分建立 */
    LL_PERIPH_WE(LL_PERIPH_ALL);
    ADC_SetSampleTime(CM_ADC2, LIFT_ADC_CHANNEL, LIFT_ADC_SAMPLE_TIME_CLK);
    LL_PERIPH_WP(LL_PERIPH_ALL);

    /* ---- 3. 使能通道（内部即 ADC_ChCmd(SEQ_A, ch, ENABLE)） ---- */
    ret = rt_adc_enable(s_dev, LIFT_ADC_CHANNEL);
    if (ret != RT_EOK) {
        rt_kprintf("[LiftADC] enable ch%u failed (%d)\n",
                   (unsigned)LIFT_ADC_CHANNEL, ret);
        return -1;
    }

    rt_kprintf("[LiftADC] %s ch%u enabled (PA7/ADC12_IN7), vref=%umV\n",
               LIFT_ADC_DEV_NAME, (unsigned)LIFT_ADC_CHANNEL,
               (unsigned)LIFT_ADC_VREF_MV);
    return 0;
}

/* ============================================================
 * 读原始值（单次）
 * ============================================================ */
uint16_t lift_adc_read_raw(void)
{
    uint32_t raw;

    if (s_dev == RT_NULL) {
        return 0U;
    }

    raw = rt_adc_read(s_dev, LIFT_ADC_CHANNEL);

    /* 12bit 硬件上限保护：rt_adc_read 不校验 convert 返回值（N-10） */
    if (raw > LIFT_ADC_RAW_MAX) {
        raw = LIFT_ADC_RAW_MAX;
    }
    return (uint16_t)raw;
}

/**
 * @brief 读取传感器信号电压（mV，已还原分压）
 * @note  公式：mv_signal = raw × Vref × DIV_DEN / (FULL_SCALE × DIV_NUM)
 *        校核：raw=1367 → 1367×3300×2/(4096×1) = 2203 mV ✅ 落在零位
 */
uint16_t lift_adc_read_mv(void *user_data)
{
    uint32_t sum = 0U;
    uint32_t raw;
    uint32_t mv;
    uint32_t n;

    (void)user_data;    /* 单通道，无需区分数据源 */

    if (s_dev == RT_NULL) {
        return 0U;
    }

    /* ---- 1. 过采样求平均 ---- */
    n = (LIFT_ADC_OVERSAMPLE > 0U) ? LIFT_ADC_OVERSAMPLE : 1U;
    for (uint32_t i = 0U; i < n; i++) {
        sum += (uint32_t)lift_adc_read_raw();
    }
    raw = sum / n;

    /* ---- 2. 换算 mV（含分压还原） ---- */
    mv = (raw * (uint32_t)LIFT_ADC_VREF_MV * (uint32_t)LIFT_ADC_DIV_DEN)
       / ((uint32_t)LIFT_ADC_FULL_SCALE * (uint32_t)LIFT_ADC_DIV_NUM);

    if (mv > 65535U) {
        mv = 65535U;
    }
    return (uint16_t)mv;
}
