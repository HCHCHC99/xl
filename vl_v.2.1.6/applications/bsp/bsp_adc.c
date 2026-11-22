/**
 * @file    bsp_adc.c
 * @brief   ADC BSP 层实现
 * @note    直接操作 MCU ADC 外设, 不感知业务语义
 */

#include "bsp_adc.h"
#include "framework/state_engine/sm_assert.h"
#include <rtthread.h>

/* ============================================================
 * BSP 私有数据
 * ============================================================ */
typedef struct {
    uint8_t  initialized;
    uint16_t voltage_mv_cache;      /* 电压通道缓存 */
    uint16_t current_mv_cache;      /* 电流通道缓存 */
} bsp_adc_priv_t;

static bsp_adc_priv_t g_bsp_adc_priv = {0};

/* ============================================================
 * 内部：通道映射（BSP 硬件相关）
 * ============================================================ */
/* TODO: 根据实际硬件修改 */
#define BSP_ADC_CHANNEL_VOLTAGE     0
#define BSP_ADC_CHANNEL_CURRENT     1

/* ============================================================
 * BSP 操作实现
 * ============================================================ */

rt_err_t bsp_adc_init(hw_adc_t *self, const void *cfg)
{
    SM_ASSERT_PTR(self);
    (void)cfg;

    if (g_bsp_adc_priv.initialized) return RT_EOK;

    /* TODO: 初始化 ADC 外设（HC32F460: LL_ADC_Init） */
    /* 1. 配置 ADC 时钟 */
    /* 2. 配置采样通道 */
    /* 3. 配置采样时间 */
    /* 4. 使能 ADC */

    g_bsp_adc_priv.initialized = 1;
    rt_kprintf("[BSP-ADC] Initialized\n");
    return RT_EOK;
}

rt_err_t bsp_adc_deinit(hw_adc_t *self)
{
    SM_ASSERT_PTR(self);

    if (!g_bsp_adc_priv.initialized) return RT_EOK;

    /* TODO: 关闭 ADC 外设 */

    g_bsp_adc_priv.initialized = 0;
    return RT_EOK;
}

uint16_t bsp_adc_read_mv(hw_adc_t *self, uint8_t ch)
{
    SM_ASSERT_PTR(self);

    uint16_t voltage_mv = 0;

    /* TODO: 根据通道号调用实际 ADC 采样 */
    switch (ch) {
        case HW_ADC_CH_VOLTAGE:
            /* TODO: 调用 LL_ADC 采样 */
            voltage_mv = 2203;      /* 临时桩 */
            break;

        case HW_ADC_CH_CURRENT:
            /* TODO: 调用 LL_ADC 采样 */
            voltage_mv = 0;
            break;

        default:
            break;
    }

    return voltage_mv;
}

uint16_t bsp_adc_read_raw(hw_adc_t *self, uint8_t ch)
{
    SM_ASSERT_PTR(self);
    (void)ch;

    /* TODO: 返回 ADC 原始值（12位: 0~4095） */
    return 0;
}

/* ============================================================
 * BSP ops 表
 * ============================================================ */
const hw_adc_ops_t g_bsp_adc_ops = {
    .init     = bsp_adc_init,
    .deinit   = bsp_adc_deinit,
    .read_mv  = bsp_adc_read_mv,
    .read_raw = bsp_adc_read_raw,
};

/* ============================================================
 * 全局实例注册
 * ============================================================ */
void bsp_adc_register(void)
{
    g_hw_adc.name = "adc0";
    g_hw_adc.ops = &g_bsp_adc_ops;
    g_hw_adc.priv = &g_bsp_adc_priv;
    g_hw_adc.is_ready = 0;
}
