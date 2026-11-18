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
 *        发送方式：
 *          2026-11-18 起改为「DMA 全负责」，与参考工程 foc_xl_v.2.1.3 的
 *          Usart3_HW_StartTxDma 一致：CPU 不写数据寄存器，整帧 80 字节
 *          由 DMA2 CH0 搬完。启动顺序见 foc_vofa_send() 的注释。
 *          帧通断的确认仍在线程上下文轮询（TC && TX_CPLT），不用 DMA 中断
 *          —— 理由见 foc_vofa_poll_tc() 的注释。
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

/* 波特率自检结果（t_vofa 可查）：
 * 0 = 配置表算出的 BRR 已正确写进寄存器
 * 1 = 写入值与 FOC_VOFA_BRR_EXPECT 不符（时钟配置变了 / 选了不支持的档位）
 * 见 foc_vofa_init() 里的说明：DDL 在算不出 BRR 时**不写寄存器**，
 * 不查就会静默按复位值发乱码。 */
static volatile uint8_t  s_baud_ok;

static struct rt_timer s_vofa_timer;

/*---------------------------------------------------------------------------
 * 非阻塞发送一帧（DMA 全负责）
 *
 * 发送顺序（与参考工程 Usart3_HW_StartTxDma 逐行对应）：
 *   ① 确保 TX 处于关闭态（上一帧收尾时已关，这里是幂等兜底）
 *   ② 清 TC 标志
 *   ③ 源地址 = &s_tx_buf[0]，传输长度 = 80（整帧）
 *   ④ DMA_ChCmd(ENABLE)
 *   ⑤ USART_FuncCmd(TX, ENABLE)
 *   ⑥ 硬件按 TXE 请求逐字节把 80 字节搬到 DR，CPU 全程不碰 DR
 *
 * 【为什么这样能跑通（握手而非盲灌）】
 *   USART1_TI 对应的请求条件是 TXE（发送数据寄存器空）。
 *   每写一次 DR，TXE 立刻由 1→0；该字节移入移位寄存器后 TXE 又回 1，
 *   才产生下一次请求。DMA 是"一次请求搬一个字节"的握手式搬运，
 *   因此不存在"一口气把 80 字节倒进 1 字节深 DR"的情况，
 *   通道提前使能也不会丢数据 —— 它只是停在原地等 TXE 变 1。
 *
 * 【顺序按参考工程来，不做"等价简化"】
 *   DMA_ChCmd(ENABLE) 放在 TX ENABLE 之前，与参考工程逐行一致。
 *   两者差几十个周期，功能上等价；保持一致是为了将来对照排查时
 *   两边代码可以逐行对应，不必再推一遍时序。
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

    /* ① 每帧从 TX 关闭态起步。收尾时已关，这里是幂等兜底：
     * 保证 TXE/TC 处于确定状态，不受上一帧中断或超时路径的残留影响。 */
    USART_FuncCmd(FOC_VOFA_USART, USART_TX, DISABLE);

    /* ② 清 TC 标志 */
    DMA_ClearTransCompleteStatus(FOC_VOFA_DMA_UNIT, FOC_VOFA_DMA_TC_FLAG);

    /* ③ 整帧交给 DMA：源地址指向首字节，长度含帧尾共 80 字节 */
    (void)DMA_SetSrcAddr(FOC_VOFA_DMA_UNIT, FOC_VOFA_DMA_CH,
                         (uint32_t)&s_tx_buf[0]);

    s_cnt_rc = DMA_SetTransCount(FOC_VOFA_DMA_UNIT, FOC_VOFA_DMA_CH,
                                 (uint16_t)FOC_VOFA_FRAME_BYTES);
    if (s_cnt_rc != LL_OK) {
        s_busy = 0u;
        s_tx_timeout++;
        return;
    }

    /* ④ 使能通道（TX 仍关，DMA 等第一个 TXE 请求） */
    s_ch_rc = DMA_ChCmd(FOC_VOFA_DMA_UNIT, FOC_VOFA_DMA_CH, ENABLE);
    if (s_ch_rc != LL_OK) {
        s_busy = 0u;
        s_tx_timeout++;
        return;
    }

    /* ⑤ 最后开 TX，开始逐字节搬运整帧 */
    USART_FuncCmd(FOC_VOFA_USART, USART_TX, ENABLE);

    s_tx_frames++;
}

/*---------------------------------------------------------------------------
 * DMA 完成轮询（只在 timer 线程调用）
 *   完成条件必须同时满足：
 *     1) DMA TC 置位（80 字节都已写入 DR）
 *     2) USART TX_CPLT 置位（最后一个字节已完全移出，包括停止位）
 *   确认后关 TX、清标志、关 DMA 通道，避免尾字节被下一帧截断。
 *   异常：超过 FOC_VOFA_TX_TIMEOUT_MS 仍未完成 -> 强制关通道并复位
 *
 * 【为什么不用 DMA 传输完成中断（这里是刻意不搬参考工程的做法）】
 *   参考工程的 HW_DmaTc_ISR 在中断里这样收尾：
 *       while (TX_CPLT != SET) __NOP();   ← 在 ISR 里自旋
 *   它跑在裸机 main 循环上，自旋最多一个字符时间（@115200 ≈ 87us），
 *   无所谓。但本工程是 RT-Thread + 20kHz FOC 电流环：
 *   DMA2_TC0 与 FOC 的 DMA1_BTC2 同属 DMA 中断源、优先级相近，
 *   在 ISR 里空转 87us 会直接吞掉 1~2 个 20kHz 控制周期（每周期 50us），
 *   表现为电流环抖动甚至失控。
 *   所以这里保持"线程轮询收尾"的结构，并且**关闭该通道的中断请求**
 *   （CHCTL.IE=0 + INTMASK1 屏蔽 TC），让 DMA2_TC0 完全不产生中断，
 *   从根上消除抢占 FOC 电流环的可能。
 *   代价只是收尾最多延后 1ms，帧率不受影响（见下方速率核算）。
 *
 * 【速率核算：帧耗时由波特率决定，配置表里逐档列了实际值】
 *   80 字节 × 10bit = 800bit，每帧耗时 = 800 / 波特率：
 *     @115200  → 6.94ms/帧  > 1ms 周期 ⇒ 物理上不可能 1kHz，
 *                s_busy 会持续为 1，实际帧率被压到约 144Hz。
 *     @576000  → 1.39ms/帧  > 1ms 周期 ⇒ 实际约 720Hz。
 *     @921600  → 0.87ms/帧  < 1ms 周期 ⇒ 约 1kHz，偶发丢帧。
 *   想稳定 1kHz 只有 921600 档能做到。
 *   （boot_table.c 里 vofa 那一行的注释已改为"由 FOC_VOFA_BAUD_SEL 决定"，
 *     原先写的 921600 与头文件默认档 115200 不一致，是遗留笔误。）
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
        /* 超时收尾：必须把 TX 也关掉。
         * 若只关通道不关 TX，下一帧进 foc_vofa_send() 时 TX 可能仍是开的，
         * "关闭态起步"的前提被破坏 ⇒ 复位的这一帧会整帧灌进 DR 丢数据。 */
        USART_FuncCmd(FOC_VOFA_USART, USART_TX, DISABLE);
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

    /* ---- USART1 8N1（波特率/分频/过采样全部由 FOC_VOFA_BAUD_SEL 决定） ---- */
    FCG_Fcg1PeriphClockCmd(FOC_VOFA_FCG_USART, ENABLE);
    (void)USART_UART_StructInit(&stcUart);
    stcUart.u32ClockDiv      = FOC_VOFA_USART_CLK_DIV;
    stcUart.u32Baudrate      = FOC_VOFA_BAUDRATE;
    stcUart.u32OverSampleBit = FOC_VOFA_USART_OVER_SAMPLE;
    i32UsartRet = USART_UART_Init(FOC_VOFA_USART, &stcUart, &f32BaudErr);

    /* ---- 波特率自检：核对 BRR 是否真按配置表写进去了 ----
     * 为什么必须查：hc32_ll_usart.c:1622 的 USART_SetBaudrate 是
     *     if (LL_OK == i32Ret) { 写 BRR; }
     * 一旦 UART_CalculateDivFraction 返回失败（BRR 分数 > 0x7F 或整数 > 0xFF，
     * 例如 500000 @ DIV1+8x 需要分数 128），BRR **完全不写**，保持复位值 0，
     * 但 USART_UART_Init 仍返回错误码而已，外设照样能用 —— 结果就是
     * "帧在发、DMA 在跑、上位机全是乱码"，且没有任何报错。
     * 这里把实配值与配置表里的期望值比对，一次暴露问题。 */
    s_baud_ok = ((i32UsartRet == LL_OK) &&
                 ((FOC_VOFA_USART->BRR & 0x0000FFFFUL) == FOC_VOFA_BRR_EXPECT))
                ? 1u : 0u;

    /* ---- DMA2 CH0：AOS 由 USART1_TI 触发 ---- */
    FCG_Fcg0PeriphClockCmd(FOC_VOFA_DMA_FCG, ENABLE);
    FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_AOS, ENABLE);

    (void)DMA_StructInit(&stcDma);
    stcDma.u32IntEn       = DMA_INT_ENABLE;
    stcDma.u32BlockSize   = 1UL;
    stcDma.u32TransCount  = 1UL;
    stcDma.u32DataWidth   = DMA_DATAWIDTH_8BIT;
    /* 起始地址指向整帧首字节：每次发送都会用 DMA_SetSrcAddr 重设，
     * 这里的初值只是为了初始化后寄存器的值有意义（对齐全 DMA 模式）。 */
    stcDma.u32SrcAddr     = (uint32_t)&s_tx_buf[0];
    stcDma.u32DestAddr    = (uint32_t)(&FOC_VOFA_USART->DR);
    stcDma.u32SrcAddrInc  = DMA_SRC_ADDR_INC;
    stcDma.u32DestAddrInc = DMA_DEST_ADDR_FIX;
    (void)DMA_Init(FOC_VOFA_DMA_UNIT, FOC_VOFA_DMA_CH, &stcDma);

    AOS_SetTriggerEventSrc(FOC_VOFA_DMA_AOS, FOC_VOFA_DMA_AOS_EVT);

    /* ---- DMA2 CH0：只轮询 TC 标志，不用传输完成中断 ----
     * 参考工程在中断里调 DMA_ChCmd(DISABLE)，但 DDL 的 DMA_ChCmd 内部
     * 有跨通道共享的静态状态，中断重入时可能静默失败。
     * 这里把所有 DMA 操作收敛到 1ms timer 线程，单上下文。
     * 通道级中断（CHCTL.IE）保持关闭，避免 DMA2_TC0 中断抢占 FOC 电流环。 */
    DMA_Cmd(FOC_VOFA_DMA_UNIT, ENABLE);
    DMA_TransCompleteIntCmd(FOC_VOFA_DMA_UNIT, FOC_VOFA_DMA_TC_INT, DISABLE);
    (void)DMA_ChCmd(FOC_VOFA_DMA_UNIT, FOC_VOFA_DMA_CH, DISABLE);

    /* 初始化时 TX 保持关闭。这是全 DMA 模式的前提：
     * 每帧靠"关闭态 → 打开"这个动作产生第一个 TI 事件来启动 DMA。
     * 若这里或运行中把 TX 常开，通道一使能就会把整帧灌进 1 字节深的 DR。 */
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
    rt_kprintf("[VOFA] sel=%d div=%lu ov=%s baud_ok=%u usart_rc=%ld err_x100=%ld\r\n",
               (int)FOC_VOFA_BAUD_SEL,
               (unsigned long)FOC_VOFA_USART_CLK_DIV,
               (FOC_VOFA_USART_OVER_SAMPLE == USART_OVER_SAMPLE_8BIT) ? "8x" : "16x",
               (unsigned)s_baud_ok,
               (long)i32UsartRet,
               (long)(f32BaudErr * 100.0f));
    rt_kprintf("[VOFA] BRR=0x%08lX (expect 0x%08lX) PR=0x%08lX dma=full-frame\r\n",
               (unsigned long)FOC_VOFA_USART->BRR,
               (unsigned long)FOC_VOFA_BRR_EXPECT,
               (unsigned long)FOC_VOFA_USART->PR);

    /* 自检失败就大声喊出来：这种情况波形一定不对，别让人去猜硬件 */
    if (s_baud_ok == 0u) {
        rt_kprintf("[VOFA] *** BAUD CONFIG FAILED: BRR not programmed ***\r\n");
        rt_kprintf("[VOFA] 1) check PCLK1 is still 100MHz in board.c\r\n");
        rt_kprintf("[VOFA] 2) check FOC_VOFA_BAUD_SEL matches table in foc_vofa.h\r\n");
    }
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

    rt_kprintf("[VOFA] inited=%u enabled=%u busy=%u baud=%lu sel=%d baud_ok=%u period=%d ms ch=%u\r\n",
               (unsigned)s_inited,
               (unsigned)s_enabled,
               (unsigned)s_busy,
               (unsigned long)FOC_VOFA_BAUDRATE,
               (int)FOC_VOFA_BAUD_SEL,
               (unsigned)s_baud_ok,
               (int)FOC_VOFA_PERIOD_MS,
               (unsigned)FOC_VOFA_CH_NUM);
    rt_kprintf("[VOFA] tx frames=%lu done=%lu timeout=%lu cnt_rc=%ld ch_rc=%ld\r\n",
               (unsigned long)s_tx_frames,
               (unsigned long)s_tx_done,
               (unsigned long)s_tx_timeout,
               (long)s_cnt_rc,
               (long)s_ch_rc);
    rt_kprintf("[VOFA] TC=%u ch_en=%u tx=%u\r\n",
               (unsigned)DMA_GetTransCompleteStatus(FOC_VOFA_DMA_UNIT,
                                                    FOC_VOFA_DMA_TC_FLAG),
               (unsigned)((FOC_VOFA_DMA_UNIT->CHEN >> FOC_VOFA_DMA_CH) & 1U),
               (unsigned)((FOC_VOFA_USART->CR1 & USART_CR1_TE) ? 1U : 0U));
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
MSH_CMD_EXPORT_ALIAS(cmd_t_vofa, vofa,   VOFA status / on / off / test);

#else  /* FOC_VOFA_ENABLE */

int foc_vofa_init(void) { return 0; }
void foc_vofa_set_enable(uint8_t en) { (void)en; }
uint8_t foc_vofa_is_enabled(void) { return 0u; }
uint8_t foc_vofa_is_busy(void) { return 0u; }

#endif /* FOC_VOFA_ENABLE */
