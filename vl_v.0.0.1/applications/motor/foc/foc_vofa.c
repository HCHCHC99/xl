/**
 *******************************************************************************
 * @file  foc_vofa.c
 * @brief VOFA+ JustFloat 19 通道输出（USART1 + DMA2 CH0）
 *
 *        引脚：PA1 = USART1_RX(FUNC_33)，PA2 = USART1_TX(FUNC_32)
 *        （PB12/PB13 保留给 CAN1）
 *        帧格式：19 个 little-endian float + 00 00 80 7F（+Inf 结尾）
 *        通道与参考 FOC 工程一致（位置/电流/角度/速度全链路）
 *
 *        发送方式：1ms 软定时器（timer 线程上下文）+ DMA，轮询 TC 完成。
 *******************************************************************************
 */

#include "foc_vofa.h"

#if FOC_VOFA_ENABLE

#include "foc_control.h"
#include "foc_current_loop.h"
#include "hw_encoder.h"
#include "cfg_motor.h"
#include "foc_mode.h"
#include "foc_speed_loop.h"
#include "foc_position_loop.h"
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

static volatile uint8_t  s_inited;
static volatile uint8_t  s_enabled;
static volatile uint8_t  s_busy;
static volatile uint8_t  s_test_pending;  /* t_vofa test 请求，由定时器线程消费 */
static volatile uint16_t s_busy_ms;       /* 本帧已等待的毫秒数 */

/* 诊断计数：t_vofa 可查看 */
static volatile uint32_t s_tx_frames;   /* 发起过的帧数 */
static volatile uint32_t s_tx_done;     /* DMA 完成的帧数 */
static volatile uint32_t s_tx_timeout;  /* 超时/配置失败强制复位的帧数 */
static volatile int32_t  s_cnt_rc;      /* 最近一次 DMA_SetTransCount 返回值 */
static volatile int32_t  s_ch_rc;       /* 最近一次 DMA_ChCmd(ENABLE) 返回值 */

static struct rt_timer s_vofa_timer;

/*---------------------------------------------------------------------------
 * 非阻塞发送一帧
 *
 * 发送顺序（保证 DMA 一定能被触发）：
 *   1. 打开 TX
 *   2. 手动把第 1 个字节写入 DR（此时 TXE 由 1->0）
 *   3. 重装 DMA 计数（剩余 23 字节）并使能通道
 *   4. 第 1 个字节移出后 TXE 由 0->1，产生 TI 事件，DMA 自动搬完剩余字节
 *
 * 所有 DMA 寄存器操作都只在 timer 线程里执行，避免 DDL DMA_ChCmd()
 * 内部静态状态被中断重入。
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

    s_busy    = 1u;
    s_busy_ms = 0u;

    USART_FuncCmd(FOC_VOFA_USART, USART_TX, ENABLE);

    /* 先写第 1 个字节，DMA 负责剩余 23 字节 */
    USART_WriteData(FOC_VOFA_USART, (uint16_t)s_tx_buf[0]);

    DMA_ClearTransCompleteStatus(FOC_VOFA_DMA_UNIT, FOC_VOFA_DMA_TC_FLAG);

    /* 与参考工程一致：每帧都重设一次源地址 */
    (void)DMA_SetSrcAddr(FOC_VOFA_DMA_UNIT, FOC_VOFA_DMA_CH,
                         (uint32_t)&s_tx_buf[1]);

    s_cnt_rc = DMA_SetTransCount(FOC_VOFA_DMA_UNIT, FOC_VOFA_DMA_CH,
                                 (uint16_t)(FOC_VOFA_FRAME_BYTES - 1u));
    if (s_cnt_rc != LL_OK) {
        s_busy = 0u;
        s_tx_timeout++;
        return;
    }

    s_ch_rc = DMA_ChCmd(FOC_VOFA_DMA_UNIT, FOC_VOFA_DMA_CH, ENABLE);
    if (s_ch_rc != LL_OK) {
        s_busy = 0u;
        s_tx_timeout++;
        return;
    }

    s_tx_frames++;
}

/*---------------------------------------------------------------------------
 * DMA 完成轮询（只在 timer 线程调用）
 *   完成条件必须同时满足：
 *     1) DMA TC 置位（剩余字节都已写入 DR）
 *     2) USART TX_CPLT 置位（最后一个字节已完全移出，包括停止位）
 *   确认后关 TX、清标志、关 DMA 通道，避免尾字节被下一帧截断。
 *   异常：超过 FOC_VOFA_TX_TIMEOUT_MS 仍未完成 -> 强制关通道并复位
 *---------------------------------------------------------------------------*/
static void foc_vofa_poll_tc(void)
{
    if (s_busy == 0u) {
        return;
    }

    if ((DMA_GetTransCompleteStatus(FOC_VOFA_DMA_UNIT, FOC_VOFA_DMA_TC_FLAG) == SET) &&
        (USART_GetStatus(FOC_VOFA_USART, USART_FLAG_TX_CPLT) == SET)) {
        USART_FuncCmd(FOC_VOFA_USART, USART_TX, DISABLE);
        USART_ClearStatus(FOC_VOFA_USART,
                          (USART_FLAG_TX_EMPTY | USART_FLAG_TX_CPLT));
        DMA_ClearTransCompleteStatus(FOC_VOFA_DMA_UNIT, FOC_VOFA_DMA_TC_FLAG);
        (void)DMA_ChCmd(FOC_VOFA_DMA_UNIT, FOC_VOFA_DMA_CH, DISABLE);
        s_busy    = 0u;
        s_busy_ms = 0u;
        s_tx_done++;
        return;
    }

    if (++s_busy_ms >= FOC_VOFA_TX_TIMEOUT_MS) {
        (void)DMA_ChCmd(FOC_VOFA_DMA_UNIT, FOC_VOFA_DMA_CH, DISABLE);
        DMA_ClearTransCompleteStatus(FOC_VOFA_DMA_UNIT, FOC_VOFA_DMA_TC_FLAG);
        s_busy    = 0u;
        s_busy_ms = 0u;
        s_tx_timeout++;
    }
}

/*---------------------------------------------------------------------------
 * 1ms 采样 + 发送（timer 线程）
 *---------------------------------------------------------------------------*/
static void foc_vofa_timer_cb(void *parameter)
{
    float   ch[FOC_VOFA_CH_NUM];
    uint8_t i;
    int32_t c;

    (void)parameter;

    if ((s_inited == 0u) || (s_enabled == 0u)) {
        return;
    }

    foc_vofa_poll_tc();          /* 检查上一帧是否完成 */
    if (s_busy != 0u) {
        return;
    }

    if (s_test_pending != 0u) {
        s_test_pending = 0u;
        for (i = 0u; i < FOC_VOFA_CH_NUM; i++) {
            ch[i] = (float)i;
        }
        foc_vofa_send(ch);
        return;
    }

    /* 与参考 FOC 工程 Foc_AdcDebug_VofaFill 的 19 通道一致 */
    ch[0]  = g_pos_target_rev;
    ch[1]  = g_pos_fb_rev + g_pos_err_rev;
    ch[2]  = g_pos_fb_rev;
    ch[3]  = g_fc_i_total;
    ch[4]  = g_fc_iu;
    ch[5]  = g_fc_iv;
    ch[6]  = g_fc_iw;
    ch[7]  = g_fc_iq;
    ch[8]  = g_fc_id;
    ch[9]  = Foc_CurrentLoop_GetIqRef();
    ch[10] = Foc_CurrentLoop_GetIdRef();
    ch[11] = Foc_CurrentLoop_GetIqRefRamped();
    ch[12] = Foc_CurrentLoop_GetIdRefRamped();
    ch[13] = g_fc_enc_elec_deg * 0.0174532925f;   /* 电角度 rad */
    ch[14] = g_fc_enc_mech_deg * 0.0174532925f;   /* 机械角度 rad */

    c = g_enc_count % (int32_t)CFG_ENCODER_CPR;
    if (c < 0) {
        c += (int32_t)CFG_ENCODER_CPR;
    }
    ch[15] = (float)c;                            /* 圈内计数 */

    ch[16] = g_fm_theta_e;                        /* IF 电角度 rad */
    ch[17] = g_spd_ramp_rpm;
    ch[18] = g_spd_filt_rpm;

    foc_vofa_send(ch);
}

/*---------------------------------------------------------------------------
 * 初始化
 *---------------------------------------------------------------------------*/
int foc_vofa_init(void)
{
    stc_usart_uart_init_t   stcUart;
    stc_dma_init_t          stcDma;
    float32_t               f32BaudErr = 0.0f;
    int32_t                 i32UsartRet;

    if (s_inited != 0u) {
        return 0;
    }

    LL_PERIPH_WE(LL_PERIPH_ALL);

    /* ---- PA1 RX / PA2 TX ---- */
    (void)GPIO_SetFunc(FOC_VOFA_RX_PORT, FOC_VOFA_RX_PIN, FOC_VOFA_RX_FUNC);
    (void)GPIO_SetFunc(FOC_VOFA_TX_PORT, FOC_VOFA_TX_PIN, FOC_VOFA_TX_FUNC);

    /* ---- USART1 8N1（波特率由 FOC_VOFA_BAUDRATE 选择） ---- */
    FCG_Fcg1PeriphClockCmd(FOC_VOFA_FCG_USART, ENABLE);
    (void)USART_UART_StructInit(&stcUart);
    stcUart.u32ClockDiv      = FOC_VOFA_USART_CLK_DIV;
    stcUart.u32Baudrate      = FOC_VOFA_BAUDRATE;
    stcUart.u32OverSampleBit = USART_OVER_SAMPLE_8BIT;
    i32UsartRet = USART_UART_Init(FOC_VOFA_USART, &stcUart, &f32BaudErr);

    /* ---- DMA2 CH0：AOS 由 USART1_TI 触发 ---- */
    FCG_Fcg0PeriphClockCmd(FOC_VOFA_DMA_FCG, ENABLE);
    FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_AOS, ENABLE);

    (void)DMA_StructInit(&stcDma);
    stcDma.u32IntEn       = DMA_INT_ENABLE;
    stcDma.u32BlockSize   = 1UL;
    stcDma.u32TransCount  = 1UL;
    stcDma.u32DataWidth   = DMA_DATAWIDTH_8BIT;
    stcDma.u32SrcAddr     = (uint32_t)&s_tx_buf[1];   /* 第 1 字节由 CPU 先写 */
    stcDma.u32DestAddr    = (uint32_t)(&FOC_VOFA_USART->DR);
    stcDma.u32SrcAddrInc  = DMA_SRC_ADDR_INC;
    stcDma.u32DestAddrInc = DMA_DEST_ADDR_FIX;
    (void)DMA_Init(FOC_VOFA_DMA_UNIT, FOC_VOFA_DMA_CH, &stcDma);

    AOS_SetTriggerEventSrc(FOC_VOFA_DMA_AOS, FOC_VOFA_DMA_AOS_EVT);

    /* ---- DMA2 CH0：只轮询 TC 标志，不用传输完成中断 ----
     * 参考工程在中断里调 DMA_ChCmd(DISABLE)，但 DDL 的 DMA_ChCmd 内部
     * 有跨通道共享的静态状态，中断重入时可能静默失败。
     * 这里把所有 DMA 操作收敛到 1ms timer 线程，单上下文。 */
    DMA_Cmd(FOC_VOFA_DMA_UNIT, ENABLE);
    DMA_TransCompleteIntCmd(FOC_VOFA_DMA_UNIT, FOC_VOFA_DMA_TC_INT, ENABLE);
    (void)DMA_ChCmd(FOC_VOFA_DMA_UNIT, FOC_VOFA_DMA_CH, DISABLE);

    /* 初始化时保持 TX 关闭，第一帧发送时再打开 */
    USART_FuncCmd(FOC_VOFA_USART, USART_TX, DISABLE);

    LL_PERIPH_WP(LL_PERIPH_ALL);

    s_enabled      = 1u;
    s_busy         = 0u;
    s_busy_ms      = 0u;
    s_test_pending = 0u;
    s_inited       = 1u;

    rt_timer_init(&s_vofa_timer,
                  "vofa",
                  foc_vofa_timer_cb,
                  RT_NULL,
                  rt_tick_from_millisecond(FOC_VOFA_PERIOD_MS),
                  RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    rt_timer_start(&s_vofa_timer);

    /* 打印 BRR/PR 实配值，便于核对波特率是否真的写进寄存器 */
    rt_kprintf("[VOFA] USART1 PA1(RX)/PA2(TX) %lu bps, %uch JustFloat, %d ms\r\n",
               (unsigned long)FOC_VOFA_BAUDRATE,
               (unsigned)FOC_VOFA_CH_NUM,
               (int)FOC_VOFA_PERIOD_MS);
    rt_kprintf("[VOFA] usart_rc=%ld baud_err_x100=%ld BRR=0x%08lX PR=0x%08lX poll=1\r\n",
               (long)i32UsartRet,
               (long)(f32BaudErr * 100.0f),
               (unsigned long)FOC_VOFA_USART->BRR,
               (unsigned long)FOC_VOFA_USART->PR);
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
 * msh：t_vofa [0|1|test]
 *---------------------------------------------------------------------------*/
static void cmd_t_vofa(int argc, char **argv)
{
    if (argc >= 2) {
        if ((argv[1][0] == '0') && (argv[1][1] == '\0')) {
            foc_vofa_set_enable(0u);
        } else if ((argv[1][0] == '1') && (argv[1][1] == '\0')) {
            foc_vofa_set_enable(1u);
        } else if ((argv[1][0] == 't') && (argv[1][1] == 'e') &&
                   (argv[1][2] == 's') && (argv[1][3] == 't') &&
                   (argv[1][4] == '\0')) {
            s_test_pending = 1u;   /* 由 1ms 定时器线程实际发送 */
            rt_kprintf("[VOFA] test frame queued (0..%u)\r\n",
                       (unsigned)(FOC_VOFA_CH_NUM - 1u));
        } else {
            rt_kprintf("usage: t_vofa [0|1|test]\r\n");
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
    rt_kprintf("[VOFA] tx frames=%lu done=%lu timeout=%lu cnt_rc=%ld ch_rc=%ld\r\n",
               (unsigned long)s_tx_frames,
               (unsigned long)s_tx_done,
               (unsigned long)s_tx_timeout,
               (long)s_cnt_rc,
               (long)s_ch_rc);
    rt_kprintf("[VOFA] TC=%u ch_en=%u\r\n",
               (unsigned)DMA_GetTransCompleteStatus(FOC_VOFA_DMA_UNIT,
                                                    FOC_VOFA_DMA_TC_FLAG),
               (unsigned)((FOC_VOFA_DMA_UNIT->CHEN >> FOC_VOFA_DMA_CH) & 1U));
    rt_kprintf("[VOFA] USART CR1=0x%08lX CR3=0x%08lX SR=0x%08lX BRR=0x%08lX\r\n",
               (unsigned long)FOC_VOFA_USART->CR1,
               (unsigned long)FOC_VOFA_USART->CR3,
               (unsigned long)FOC_VOFA_USART->SR,
               (unsigned long)FOC_VOFA_USART->BRR);
    rt_kprintf("[VOFA] DMA EN=%lu CHEN=0x%08lX SAR=0x%08lX DTCTL=0x%08lX AOS=0x%08lX\r\n",
               (unsigned long)FOC_VOFA_DMA_UNIT->EN,
               (unsigned long)FOC_VOFA_DMA_UNIT->CHEN,
               (unsigned long)FOC_VOFA_DMA_UNIT->SAR0,
               (unsigned long)FOC_VOFA_DMA_UNIT->DTCTL0,
               (unsigned long)(*(volatile uint32_t *)FOC_VOFA_DMA_AOS));
}
MSH_CMD_EXPORT_ALIAS(cmd_t_vofa, t_vofa, VOFA status / on / off / test);

#else  /* FOC_VOFA_ENABLE */

int foc_vofa_init(void) { return 0; }
void foc_vofa_set_enable(uint8_t en) { (void)en; }
uint8_t foc_vofa_is_enabled(void) { return 0u; }
uint8_t foc_vofa_is_busy(void) { return 0u; }

#endif /* FOC_VOFA_ENABLE */
