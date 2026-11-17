/**
 *******************************************************************************
 * @file  foc_vofa.c
 * @brief VOFA+ JustFloat 5 通道输出（USART3 + DMA2 CH0）
 *
 *        帧格式：5 个 little-endian float + 00 00 80 7F（+Inf 结尾）
 *        ch0~2 = iu/iv/iw (A)
 *        ch3    = 编码器圈内计数 0 ~ CPR-1
 *        ch4    = 圈数（累计计数 / CPR 向下取整）
 *******************************************************************************
 */

#include "foc_vofa.h"

#if FOC_VOFA_ENABLE

#include "foc_control.h"
#include "hw_encoder.h"
#include "cfg_motor.h"
#include "hc32_ll_aos.h"
#include "hc32_ll_dma.h"
#include "hc32_ll_usart.h"

#include <rtthread.h>
#include <finsh.h>
#include <string.h>

/*---------------------------------------------------------------------------
 * 内部状态
 *---------------------------------------------------------------------------*/
static uint8_t s_tx_buf[FOC_VOFA_FRAME_BYTES] __attribute__((aligned(4)));

static volatile uint8_t s_inited;
static volatile uint8_t s_enabled;
static volatile uint8_t s_busy;

static struct rt_timer s_vofa_timer;

/*---------------------------------------------------------------------------
 * DMA 传输完成：只清标志、停通道，不在这里等 TX_CPLT
 * （发送最后 1 字节时 DMA 已结束，USART 会自己把该字节移出去）
 *---------------------------------------------------------------------------*/
static void foc_vofa_dma_tc_isr(void)
{
    if (DMA_GetTransCompleteStatus(FOC_VOFA_DMA_UNIT, FOC_VOFA_DMA_TC_FLAG) == SET) {
        DMA_ClearTransCompleteStatus(FOC_VOFA_DMA_UNIT, FOC_VOFA_DMA_TC_FLAG);
        (void)DMA_ChCmd(FOC_VOFA_DMA_UNIT, FOC_VOFA_DMA_CH, DISABLE);
        s_busy = 0u;
    }
}

/*---------------------------------------------------------------------------
 * 非阻塞发送一帧
 *---------------------------------------------------------------------------*/
static void foc_vofa_send(const float *ch)
{
    uint8_t i;

    if ((s_inited == 0u) || (s_enabled == 0u) || (s_busy != 0u)) {
        return;
    }

    for (i = 0u; i < FOC_VOFA_CH_NUM; i++) {
        (void)memcpy(&s_tx_buf[i * 4u], &ch[i], 4u);
    }
    /* JustFloat 帧尾：0x7F800000 = +Inf，小端 */
    s_tx_buf[FOC_VOFA_FLOAT_BYTES + 0u] = 0x00u;
    s_tx_buf[FOC_VOFA_FLOAT_BYTES + 1u] = 0x00u;
    s_tx_buf[FOC_VOFA_FLOAT_BYTES + 2u] = 0x80u;
    s_tx_buf[FOC_VOFA_FLOAT_BYTES + 3u] = 0x7Fu;

    s_busy = 1u;

    DMA_ClearTransCompleteStatus(FOC_VOFA_DMA_UNIT, FOC_VOFA_DMA_TC_FLAG);
    NVIC_ClearPendingIRQ(FOC_VOFA_DMA_TC_IRQn);

    (void)DMA_SetSrcAddr(FOC_VOFA_DMA_UNIT, FOC_VOFA_DMA_CH,
                         (uint32_t)&s_tx_buf[0]);
    (void)DMA_SetTransCount(FOC_VOFA_DMA_UNIT, FOC_VOFA_DMA_CH,
                            (uint16_t)FOC_VOFA_FRAME_BYTES);
    (void)DMA_ChCmd(FOC_VOFA_DMA_UNIT, FOC_VOFA_DMA_CH, ENABLE);
}

/*---------------------------------------------------------------------------
 * 1ms 采样 + 发送
 *---------------------------------------------------------------------------*/
static void foc_vofa_timer_cb(void *parameter)
{
    float   ch[FOC_VOFA_CH_NUM];
    int32_t c;
    int32_t turns;

    (void)parameter;

    if ((s_inited == 0u) || (s_enabled == 0u) || (s_busy != 0u)) {
        return;
    }

    ch[0] = g_fc_iu;
    ch[1] = g_fc_iv;
    ch[2] = g_fc_iw;

    c = g_enc_count % (int32_t)CFG_ENCODER_CPR;
    if (c < 0) {
        c += (int32_t)CFG_ENCODER_CPR;
    }

    if (g_enc_count >= 0) {
        turns = g_enc_count / (int32_t)CFG_ENCODER_CPR;
    } else {
        turns = -(((-g_enc_count) + (int32_t)CFG_ENCODER_CPR - 1) /
                  (int32_t)CFG_ENCODER_CPR);
    }

    ch[3] = (float)c;
    ch[4] = (float)turns;

    foc_vofa_send(ch);
}

/*---------------------------------------------------------------------------
 * 初始化
 *---------------------------------------------------------------------------*/
int foc_vofa_init(void)
{
    stc_usart_uart_init_t   stcUart;
    stc_dma_init_t          stcDma;
    stc_irq_signin_config_t stcIrq;

    if (s_inited != 0u) {
        return 0;
    }

    LL_PERIPH_WE(LL_PERIPH_ALL);

    /* ---- PB12 RX / PB13 TX ---- */
    (void)GPIO_SetFunc(FOC_VOFA_RX_PORT, FOC_VOFA_RX_PIN, FOC_VOFA_RX_FUNC);
    (void)GPIO_SetFunc(FOC_VOFA_TX_PORT, FOC_VOFA_TX_PIN, FOC_VOFA_TX_FUNC);

    /* ---- USART3 921600 8N1 ---- */
    FCG_Fcg1PeriphClockCmd(FOC_VOFA_FCG_USART, ENABLE);
    (void)USART_UART_StructInit(&stcUart);
    stcUart.u32ClockDiv      = USART_CLK_DIV1;
    stcUart.u32Baudrate      = FOC_VOFA_BAUDRATE;
    stcUart.u32OverSampleBit = USART_OVER_SAMPLE_8BIT;
    (void)USART_UART_Init(FOC_VOFA_USART, &stcUart, NULL);

    /* ---- DMA2 CH0：AOS 由 USART3_TI 触发 ---- */
    FCG_Fcg0PeriphClockCmd(FOC_VOFA_DMA_FCG, ENABLE);
    FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_AOS, ENABLE);

    (void)DMA_StructInit(&stcDma);
    stcDma.u32IntEn       = DMA_INT_ENABLE;
    stcDma.u32BlockSize   = 1UL;
    stcDma.u32TransCount  = 1UL;
    stcDma.u32DataWidth   = DMA_DATAWIDTH_8BIT;
    stcDma.u32SrcAddr     = (uint32_t)&s_tx_buf[0];
    stcDma.u32DestAddr    = (uint32_t)(&FOC_VOFA_USART->DR);
    stcDma.u32SrcAddrInc  = DMA_SRC_ADDR_INC;
    stcDma.u32DestAddrInc = DMA_DEST_ADDR_FIX;
    (void)DMA_Init(FOC_VOFA_DMA_UNIT, FOC_VOFA_DMA_CH, &stcDma);

    AOS_SetTriggerEventSrc(FOC_VOFA_DMA_AOS, FOC_VOFA_DMA_AOS_EVT);

    /* ---- DMA2 CH0 传输完成中断 ---- */
    stcIrq.enIntSrc    = FOC_VOFA_DMA_TC_INT_SRC;
    stcIrq.enIRQn      = FOC_VOFA_DMA_TC_IRQn;
    stcIrq.pfnCallback = &foc_vofa_dma_tc_isr;
    if (LL_OK == INTC_IrqSignIn(&stcIrq)) {
        NVIC_ClearPendingIRQ(FOC_VOFA_DMA_TC_IRQn);
        NVIC_SetPriority(FOC_VOFA_DMA_TC_IRQn, DDL_IRQ_PRIO_DEFAULT);
        NVIC_EnableIRQ(FOC_VOFA_DMA_TC_IRQn);
    } else {
        rt_kprintf("[VOFA] WARN: DMA TC IrqSignIn failed\r\n");
    }

    DMA_Cmd(FOC_VOFA_DMA_UNIT, ENABLE);
    DMA_TransCompleteIntCmd(FOC_VOFA_DMA_UNIT, FOC_VOFA_DMA_TC_INT, ENABLE);
    (void)DMA_ChCmd(FOC_VOFA_DMA_UNIT, FOC_VOFA_DMA_CH, DISABLE);

    USART_FuncCmd(FOC_VOFA_USART, USART_TX, ENABLE);

    LL_PERIPH_WP(LL_PERIPH_ALL);

    s_enabled = 1u;
    s_busy    = 0u;
    s_inited  = 1u;

    rt_timer_init(&s_vofa_timer,
                  "vofa",
                  foc_vofa_timer_cb,
                  RT_NULL,
                  rt_tick_from_millisecond(FOC_VOFA_PERIOD_MS),
                  RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    rt_timer_start(&s_vofa_timer);

    rt_kprintf("[VOFA] USART3 PB12/PB13 %lu bps, 5ch JustFloat, %d ms\r\n",
               (unsigned long)FOC_VOFA_BAUDRATE,
               (int)FOC_VOFA_PERIOD_MS);
    return 0;
}

void foc_vofa_set_enable(uint8_t en)
{
    s_enabled = (en != 0u) ? 1u : 0u;
}

uint8_t foc_vofa_is_enabled(void)
{
    return s_enabled;
}

uint8_t foc_vofa_is_busy(void)
{
    return s_busy;
}

/*---------------------------------------------------------------------------
 * msh：t_vofa [0|1]
 *---------------------------------------------------------------------------*/
static void cmd_t_vofa(int argc, char **argv)
{
    if (argc >= 2) {
        if ((argv[1][0] == '0') && (argv[1][1] == '\0')) {
            foc_vofa_set_enable(0u);
        } else if ((argv[1][0] == '1') && (argv[1][1] == '\0')) {
            foc_vofa_set_enable(1u);
        } else {
            rt_kprintf("usage: t_vofa [0|1]\r\n");
            return;
        }
    }

    rt_kprintf("[VOFA] inited=%u enabled=%u busy=%u baud=%lu period=%d ms ch=%u\r\n",
               (unsigned)s_inited,
               (unsigned)s_enabled,
               (unsigned)s_busy,
               (unsigned long)FOC_VOFA_BAUDRATE,
               (int)FOC_VOFA_PERIOD_MS,
               (unsigned)FOC_VOFA_CH_NUM);
}
MSH_CMD_EXPORT_ALIAS(cmd_t_vofa, t_vofa, VOFA status / on / off);

#else  /* FOC_VOFA_ENABLE */

int foc_vofa_init(void) { return 0; }
void foc_vofa_set_enable(uint8_t en) { (void)en; }
uint8_t foc_vofa_is_enabled(void) { return 0u; }
uint8_t foc_vofa_is_busy(void) { return 0u; }

#endif /* FOC_VOFA_ENABLE */
