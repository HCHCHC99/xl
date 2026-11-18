/**
 *******************************************************************************
 * @file  foc_vofa.h
 * @brief VOFA+ JustFloat 输出（USART1，PA1/PA2，波特率配置表可选 115200~921600）
 *
 *        接线：
 *          PA1 = USART1_RX = FUNC_33
 *          PA2 = USART1_TX = FUNC_32
 *          TX 走 DMA2 CH0，AOS 由 USART1_TI 触发
 *        （PB12/PB13 是 CAN1，VOFA 不再占用，避免与 CAN 冲突）
 *
 *        发送方式（2026-11-18 改为 DMA 全负责，与参考工程一致）：
 *          每帧顺序（与参考工程 Usart3_HW_StartTxDma 逐行对应）：
 *            ① TX 置关闭态（幂等兜底，收尾时已关）
 *            ② 清 TC 标志 → 设源地址 buf[0] → 设传输长度 80 字节
 *            ③ DMA_ChCmd(ENABLE)
 *            ④ USART_FuncCmd(TX, ENABLE)
 *            ⑤ 硬件按 TXE 请求逐字节把 80 字节搬到 DR，CPU 不碰 DR
 *          帧完成确认在线程上下文轮询（TC && TX_CPLT），不用 DMA 中断
 *          （避免在中断里自旋等 TX_CPLT，抢占 20kHz FOC 电流环）。
 *
 *          原理：USART1_TI 的请求条件是 TXE。DMA 每收到一次请求才搬
 *          一个字节，属握手式搬运，不存在灌满 1 字节深 DR 的问题。
 *
 *        通道（19 路，JustFloat，与参考 FOC 工程一致）：
 *          ch0  位置目标 rev            ch1  有效给定位 (fb+err) rev
 *          ch2  位置反馈 rev            ch3  总电流 A
 *          ch4~6  iu / iv / iw A       ch7~8  iq / id A
 *          ch9~10 iq_ref / id_ref A    ch11~12 斜坡后 iq / id A
 *          ch13 电角度 rad             ch14 机械角度 rad
 *          ch15 圈内计数 0~CPR-1       ch16 IF 电角度 rad
 *          ch17 速度环斜坡 rpm         ch18 速度环实测 rpm
 *******************************************************************************
 */

#ifndef FOC_VOFA_H
#define FOC_VOFA_H

#include "hc32_ll.h"
#include <stdint.h>

/* 总开关：0 = 不初始化 USART1/DMA，也不发帧 */
#ifndef FOC_VOFA_ENABLE
#define FOC_VOFA_ENABLE             1
#endif

/* 发送周期（ms）。
 * 80 字节/帧 × 10bit = 800bit，每帧耗时 = 800 / 波特率：
 *   @115200  → 6.94ms/帧  ⇒ 1ms 周期跑不满，实际约 144Hz
 *   @230400  → 3.47ms/帧  ⇒ 实际约 288Hz
 *   @460800  → 1.74ms/帧  ⇒ 实际约 576Hz
 *   @576000  → 1.39ms/帧  ⇒ 实际约 720Hz
 *   @921600  → 0.87ms/帧  ⇒ 刚好能顶住 1ms 周期，约 1kHz
 * 想稳定 1kHz 只有 921600 档做得到；更低档位实际帧率被波特率限住。 */
#ifndef FOC_VOFA_PERIOD_MS
#define FOC_VOFA_PERIOD_MS          1
#endif

/* ---- 引脚 / 外设（PA1/PA2，USART1） ---- */
#define FOC_VOFA_USART              CM_USART1
#define FOC_VOFA_FCG_USART          FCG1_PERIPH_USART1

#define FOC_VOFA_RX_PORT            (GPIO_PORT_A)
#define FOC_VOFA_RX_PIN             (GPIO_PIN_01)
#define FOC_VOFA_RX_FUNC            (GPIO_FUNC_33)

#define FOC_VOFA_TX_PORT            (GPIO_PORT_A)
#define FOC_VOFA_TX_PIN             (GPIO_PIN_02)
#define FOC_VOFA_TX_FUNC            (GPIO_FUNC_32)

/*===========================================================================
 * 波特率配置表：只改 FOC_VOFA_BAUD_SEL 一行，分频/过采样/超时全部自动跟随
 *
 *   档位                   波特率      分频   过采样   实际波特率      误差
 *   ---------------------  ----------  ------  ------  ------------  ---------
 *   FOC_VOFA_BAUD_115200      115,200  DIV4    16x       115,027.8  -0.1495%
 *   FOC_VOFA_BAUD_230400      230,400  DIV4     8x       230,055.6  -0.1495%
 *   FOC_VOFA_BAUD_460800      460,800  DIV1    16x       460,111.2  -0.1495%
 *   FOC_VOFA_BAUD_576000      576,000  DIV1     8x       574,311.8  -0.2931%
 *   FOC_VOFA_BAUD_921600      921,600  DIV1     8x       920,222.4  -0.1495%
 *
 * 分频与过采样不是随手定的，是按 hc32_ll_usart.c:512 的
 * UART_CalculateDivFraction 公式，把 (PR 分频 × 过采样) 全部组合穷举后，
 * 选「误差最小 且 采样裕量够」的那一组：
 *   · BRR 整数上限 0xFF、分数上限 0x7F（hc32_ll_usart.c:231-232）
 *   · 采样裕量 = C / (B × 8 × (2-OVER8))，即每个过采样点分到几个 PCLK1 时钟。
 *     921600 若用 DIV4+16x 只剩 1.70 个时钟/采样点，故不用；DIV1+8x 有 13.56 个。
 *
 * ⚠ 500000 刻意不在表内：DIV1+8x 需要 BRR 分数 = 128，超上限 0x7F 整一位，
 *   DDL 返回 LL_ERR_INVD_PARAM 且**根本不写 BRR**（静默保持复位值）——
 *   表现是能发帧但全是乱码。真要用 500000，改 DIV4+8x
 *   （BRR = 0x00575，实际 498,453.8，误差 -0.3092%）。
 *
 * 前提：PCLK1 = 100MHz（board/board.c：XTAL 8M → PLL → HCLK 200M → PCLK1_DIV2）。
 * 系统时钟一改，整张表必须重算，否则 BRR 校验会在初始化时直接报错。
 *=========================================================================*/
#define FOC_VOFA_BAUD_115200        1
#define FOC_VOFA_BAUD_230400        2
#define FOC_VOFA_BAUD_460800        3
#define FOC_VOFA_BAUD_576000        4
#define FOC_VOFA_BAUD_921600        5

/* ↓↓↓ 改这一行选档位 ↓↓↓ */
#ifndef FOC_VOFA_BAUD_SEL
#define FOC_VOFA_BAUD_SEL           FOC_VOFA_BAUD_921600
#endif

/* 档位展开表：波特率 / 时钟分频 / 过采样 / 期望 BRR（用于初始化自检） */
#if   (FOC_VOFA_BAUD_SEL == FOC_VOFA_BAUD_115200)
#define FOC_VOFA_BAUDRATE           115200UL
#define FOC_VOFA_USART_CLK_DIV      USART_CLK_DIV4
#define FOC_VOFA_USART_OVER_SAMPLE  USART_OVER_SAMPLE_16BIT
#define FOC_VOFA_BRR_EXPECT         0x00C75UL
#elif (FOC_VOFA_BAUD_SEL == FOC_VOFA_BAUD_230400)
#define FOC_VOFA_BAUDRATE           230400UL
#define FOC_VOFA_USART_CLK_DIV      USART_CLK_DIV4
#define FOC_VOFA_USART_OVER_SAMPLE  USART_OVER_SAMPLE_8BIT
#define FOC_VOFA_BRR_EXPECT         0x00C75UL
#elif (FOC_VOFA_BAUD_SEL == FOC_VOFA_BAUD_460800)
#define FOC_VOFA_BAUDRATE           460800UL
#define FOC_VOFA_USART_CLK_DIV      USART_CLK_DIV1
#define FOC_VOFA_USART_OVER_SAMPLE  USART_OVER_SAMPLE_16BIT
#define FOC_VOFA_BRR_EXPECT         0x00C75UL
#elif (FOC_VOFA_BAUD_SEL == FOC_VOFA_BAUD_576000)
#define FOC_VOFA_BAUDRATE           576000UL
#define FOC_VOFA_USART_CLK_DIV      USART_CLK_DIV1
#define FOC_VOFA_USART_OVER_SAMPLE  USART_OVER_SAMPLE_8BIT
#define FOC_VOFA_BRR_EXPECT         0x01477UL
#elif (FOC_VOFA_BAUD_SEL == FOC_VOFA_BAUD_921600)
#define FOC_VOFA_BAUDRATE           921600UL
#define FOC_VOFA_USART_CLK_DIV      USART_CLK_DIV1
#define FOC_VOFA_USART_OVER_SAMPLE  USART_OVER_SAMPLE_8BIT
#define FOC_VOFA_BRR_EXPECT         0x00C75UL
#else
#error "FOC_VOFA_BAUD_SEL invalid: pick FOC_VOFA_BAUD_115200/230400/460800/576000/921600"
#endif

/* ---- DMA2 CH0：USART1_TI 触发，每触发一次搬一个字节到数据寄存器 ---- */
#define FOC_VOFA_DMA_UNIT           CM_DMA2
#define FOC_VOFA_DMA_CH             DMA_CH0
#define FOC_VOFA_DMA_FCG            FCG0_PERIPH_DMA2
#define FOC_VOFA_DMA_AOS            AOS_DMA2_0
#define FOC_VOFA_DMA_AOS_EVT        EVT_SRC_USART1_TI
#define FOC_VOFA_DMA_TC_FLAG        DMA_FLAG_TC_CH0
#define FOC_VOFA_DMA_TC_INT         DMA_INT_TC_CH0

/* 发送超时（ms）：按帧长和波特率自动计算（10bit/字节 + 5ms 余量）。
 *   115200 → 11ms   230400 → 8ms   460800 → 6ms
 *   576000 →  6ms   921600 → 5ms */
#define FOC_VOFA_TX_TIMEOUT_MS                                  \
    ((FOC_VOFA_FRAME_BYTES * 10U * 1000U) / FOC_VOFA_BAUDRATE + 5U)

/* ---- JustFloat 帧 ---- */
#define FOC_VOFA_CH_NUM             19U
#define FOC_VOFA_FLOAT_BYTES        (FOC_VOFA_CH_NUM * 4U)
#define FOC_VOFA_TAIL_BYTES         4U
#define FOC_VOFA_FRAME_BYTES        (FOC_VOFA_FLOAT_BYTES + FOC_VOFA_TAIL_BYTES)

int     foc_vofa_init(void);
void    foc_vofa_set_enable(uint8_t en);
uint8_t foc_vofa_is_enabled(void);
uint8_t foc_vofa_is_busy(void);

#endif /* FOC_VOFA_H */
