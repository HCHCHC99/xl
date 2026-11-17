/**
 *******************************************************************************
 * @file  foc_vofa.h
 * @brief VOFA+ JustFloat 输出（USART1，PA1/PA2，115200/921600 可选）
 *
 *        接线：
 *          PA1 = USART1_RX = FUNC_33
 *          PA2 = USART1_TX = FUNC_32
 *          TX 走 DMA2 CH0，AOS 由 USART1_TI 触发
 *        （PB12/PB13 是 CAN1，VOFA 不再占用，避免与 CAN 冲突）
 *
 *        通道（5 路，JustFloat）：
 *          ch0 = U 相电流 A
 *          ch1 = V 相电流 A
 *          ch2 = W 相电流 A
 *          ch3 = 编码器圈内计数 0 ~ CPR-1
 *          ch4 = 圈数（累计计数 / CPR，向下取整）
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

/* 发送周期（ms）。1ms = 1kHz 帧率，5 通道 24 字节/帧，约 24KB/s */
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

/* ---- 波特率：直接填 115200 或 921600（也可以填其他标准波特率） ---- */
#define FOC_VOFA_BAUDRATE           115200UL

/* USART 时钟分频自动跟随波特率，与参考工程 HW_GetClkDiv 规则一致：
 *   > 115200 -> DIV1，否则 DIV4 */
#if (FOC_VOFA_BAUDRATE > 115200UL)
#define FOC_VOFA_USART_CLK_DIV      USART_CLK_DIV1
#else
#define FOC_VOFA_USART_CLK_DIV      USART_CLK_DIV4
#endif

/* ---- DMA2 CH0：USART1_TI 触发，每触发一次搬一个字节到数据寄存器 ---- */
#define FOC_VOFA_DMA_UNIT           CM_DMA2
#define FOC_VOFA_DMA_CH             DMA_CH0
#define FOC_VOFA_DMA_FCG            FCG0_PERIPH_DMA2
#define FOC_VOFA_DMA_AOS            AOS_DMA2_0
#define FOC_VOFA_DMA_AOS_EVT        EVT_SRC_USART1_TI
#define FOC_VOFA_DMA_TC_FLAG        DMA_FLAG_TC_CH0
#define FOC_VOFA_DMA_TC_INT         DMA_INT_TC_CH0

/* ---- JustFloat 帧 ---- */
#define FOC_VOFA_CH_NUM             5U
#define FOC_VOFA_FLOAT_BYTES        (FOC_VOFA_CH_NUM * 4U)
#define FOC_VOFA_TAIL_BYTES         4U
#define FOC_VOFA_FRAME_BYTES        (FOC_VOFA_FLOAT_BYTES + FOC_VOFA_TAIL_BYTES)

int     foc_vofa_init(void);
void    foc_vofa_set_enable(uint8_t en);
uint8_t foc_vofa_is_enabled(void);
uint8_t foc_vofa_is_busy(void);

#endif /* FOC_VOFA_H */
