/**
 * @file    foc_eoca_probe.c
 * @brief   ADC1 EOCA 观测探针实现
 * @note    PA0 推挽输出，每次 ADC1 序列 A 转换完成（EOCA）翻转一次。
 *          当前 ADC 由 TMR4 谷值 ISR 软件触发，因此 PA0 波形频率
 *          与 PWM 谷值事件一致（标称 20kHz）。
 */

#include "foc_eoca_probe.h"

#include "hc32_ll.h"
#include "hc32_ll_tmr4.h"

#include <rtthread.h>

#define FOC_EOCA_IRQ_NUM    (INT116_IRQn)

/* 最近一次 EOCA 中断入口时的 TMR4 计数值 */
static volatile uint16_t s_eoca_cnt;

static void foc_eoca_probe_isr(void)
{
    /* 先取 CNT，再清标志/翻转，尽量贴近 EOCA 事件时刻 */
    s_eoca_cnt = TMR4_GetCountValue(CM_TMR4_3);

    /* 先清 EOCA，再翻转，避免退出中断后立刻重入 */
    ADC_ClearStatus(CM_ADC1, ADC_FLAG_EOCA);
    GPIO_TogglePins(GPIO_PORT_A, GPIO_PIN_00);
}

uint16_t foc_eoca_probe_get_cnt(void)
{
    return s_eoca_cnt;
}

int foc_eoca_probe_init(void)
{
    stc_gpio_init_t         gpio;
    stc_irq_signin_config_t irq;

    /* ---- 1. PA0 推挽输出，默认低电平 ---- */
    LL_PERIPH_WE(LL_PERIPH_GPIO);

    (void)GPIO_StructInit(&gpio);
    gpio.u16PinDir   = PIN_DIR_OUT;
    gpio.u16PinState = PIN_STAT_RST;
    gpio.u16PinAttr  = PIN_ATTR_DIGITAL;
    (void)GPIO_Init(GPIO_PORT_A, GPIO_PIN_00, &gpio);

    LL_PERIPH_WP(LL_PERIPH_GPIO);

    /* ---- 2. 注册 ADC1 EOCA 中断 ----
     * INT_SRC_ADC1_EOCA = 448 → 向量允许区间 INT116..INT121，
     * 这里取 INT116。 */
    irq.enIntSrc    = INT_SRC_ADC1_EOCA;
    irq.enIRQn      = FOC_EOCA_IRQ_NUM;
    irq.pfnCallback = &foc_eoca_probe_isr;

    if (INTC_IrqSignIn(&irq) != LL_OK) {
        rt_kprintf("[FOC-EOCA] IrqSignIn failed (src=%d irq=%d)\r\n",
                   (int)irq.enIntSrc, (int)irq.enIRQn);
        return -1;
    }

    NVIC_ClearPendingIRQ(irq.enIRQn);
    NVIC_SetPriority(irq.enIRQn, DDL_IRQ_PRIO_03);
    NVIC_EnableIRQ(irq.enIRQn);

    /* ---- 3. 清残留标志后使能 EOCA 中断 ---- */
    ADC_ClearStatus(CM_ADC1, ADC_FLAG_EOCA);
    ADC_IntCmd(CM_ADC1, ADC_INT_EOCA, ENABLE);

    rt_kprintf("[FOC-EOCA] PA0 probe ready: toggle on ADC1 EOCA\r\n");
    return 0;
}
