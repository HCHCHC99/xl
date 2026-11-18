/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 * Copyright (c) 2022, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-04-28     CDT          first version
 */

#ifndef __IRQ_CONFIG_H__
#define __IRQ_CONFIG_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BSP_EXTINT0_IRQ_NUM             INT022_IRQn
#define BSP_EXTINT0_IRQ_PRIO            DDL_IRQ_PRIO_DEFAULT
#define BSP_EXTINT1_IRQ_NUM             INT023_IRQn
#define BSP_EXTINT1_IRQ_PRIO            DDL_IRQ_PRIO_DEFAULT
#define BSP_EXTINT2_IRQ_NUM             INT024_IRQn
#define BSP_EXTINT2_IRQ_PRIO            DDL_IRQ_PRIO_DEFAULT
#define BSP_EXTINT3_IRQ_NUM             INT025_IRQn
#define BSP_EXTINT3_IRQ_PRIO            DDL_IRQ_PRIO_DEFAULT
#define BSP_EXTINT4_IRQ_NUM             INT026_IRQn
#define BSP_EXTINT4_IRQ_PRIO            DDL_IRQ_PRIO_DEFAULT
#define BSP_EXTINT5_IRQ_NUM             INT027_IRQn
#define BSP_EXTINT5_IRQ_PRIO            DDL_IRQ_PRIO_DEFAULT
#define BSP_EXTINT6_IRQ_NUM             INT028_IRQn
#define BSP_EXTINT6_IRQ_PRIO            DDL_IRQ_PRIO_DEFAULT
#define BSP_EXTINT7_IRQ_NUM             INT029_IRQn
#define BSP_EXTINT7_IRQ_PRIO            DDL_IRQ_PRIO_DEFAULT
#define BSP_EXTINT8_IRQ_NUM             INT030_IRQn
#define BSP_EXTINT8_IRQ_PRIO            DDL_IRQ_PRIO_DEFAULT
#define BSP_EXTINT9_IRQ_NUM             INT031_IRQn
#define BSP_EXTINT9_IRQ_PRIO            DDL_IRQ_PRIO_DEFAULT
#define BSP_EXTINT10_IRQ_NUM            INT032_IRQn
#define BSP_EXTINT10_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT
#define BSP_EXTINT11_IRQ_NUM            INT033_IRQn
#define BSP_EXTINT11_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT
#define BSP_EXTINT12_IRQ_NUM            INT034_IRQn
#define BSP_EXTINT12_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT
#define BSP_EXTINT13_IRQ_NUM            INT035_IRQn
#define BSP_EXTINT13_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT
#define BSP_EXTINT14_IRQ_NUM            INT036_IRQn
#define BSP_EXTINT14_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT
#define BSP_EXTINT15_IRQ_NUM            INT037_IRQn
#define BSP_EXTINT15_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT

/* DMA1 ch0 */
#define BSP_DMA1_CH0_IRQ_NUM            INT038_IRQn
#define BSP_DMA1_CH0_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT
/* DMA1 ch1 */
#define BSP_DMA1_CH1_IRQ_NUM            INT039_IRQn
#define BSP_DMA1_CH1_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT
/* DMA1 ch2 */
#define BSP_DMA1_CH2_IRQ_NUM            INT040_IRQn
#define BSP_DMA1_CH2_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT
/* DMA1 ch3 */
#define BSP_DMA1_CH3_IRQ_NUM            INT041_IRQn
#define BSP_DMA1_CH3_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT

/* DMA2 ch0 */
#define BSP_DMA2_CH0_IRQ_NUM            INT042_IRQn
#define BSP_DMA2_CH0_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT
/* DMA2 ch1 */
#define BSP_DMA2_CH1_IRQ_NUM            INT043_IRQn
#define BSP_DMA2_CH1_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT
/* DMA2 ch2 */
#define BSP_DMA2_CH2_IRQ_NUM            INT008_IRQn
#define BSP_DMA2_CH2_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT
/* DMA2 ch3 */
#define BSP_DMA2_CH3_IRQ_NUM            INT009_IRQn
#define BSP_DMA2_CH3_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT


#if defined(BSP_USING_UART1)
#define BSP_UART1_RXERR_IRQ_NUM         INT010_IRQn
#define BSP_UART1_RXERR_IRQ_PRIO        DDL_IRQ_PRIO_DEFAULT
#define BSP_UART1_RX_IRQ_NUM            INT083_IRQn
#define BSP_UART1_RX_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT
#define BSP_UART1_TX_IRQ_NUM            INT082_IRQn
#define BSP_UART1_TX_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT

#if defined(BSP_UART1_RX_USING_DMA)
#define BSP_UART1_RXTO_IRQ_NUM          INT006_IRQn
#define BSP_UART1_RXTO_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT
#endif
#if defined(BSP_UART1_TX_USING_DMA)
#define BSP_UART1_TX_CPLT_IRQ_NUM       INT080_IRQn
#define BSP_UART1_TX_CPLT_IRQ_PRIO      DDL_IRQ_PRIO_DEFAULT
#endif
#endif /* BSP_USING_UART1 */

#if defined(BSP_USING_UART2)
#define BSP_UART2_RXERR_IRQ_NUM         INT011_IRQn
#define BSP_UART2_RXERR_IRQ_PRIO        DDL_IRQ_PRIO_DEFAULT
#define BSP_UART2_RX_IRQ_NUM            INT085_IRQn
#define BSP_UART2_RX_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT
#define BSP_UART2_TX_IRQ_NUM            INT084_IRQn
#define BSP_UART2_TX_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT

#if defined(BSP_UART2_RX_USING_DMA)
#define BSP_UART2_RXTO_IRQ_NUM          INT007_IRQn
#define BSP_UART2_RXTO_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT
#endif
#if defined(BSP_UART2_TX_USING_DMA)
#define BSP_UART2_TX_CPLT_IRQ_NUM       INT081_IRQn
#define BSP_UART2_TX_CPLT_IRQ_PRIO      DDL_IRQ_PRIO_DEFAULT
#endif
#endif /* BSP_USING_UART2 */

#if defined(BSP_USING_UART3)
#define BSP_UART3_RXERR_IRQ_NUM         INT012_IRQn
#define BSP_UART3_RXERR_IRQ_PRIO        DDL_IRQ_PRIO_DEFAULT
#define BSP_UART3_RX_IRQ_NUM            INT089_IRQn
#define BSP_UART3_RX_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT
#define BSP_UART3_TX_IRQ_NUM            INT088_IRQn
#define BSP_UART3_TX_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT

#if defined(BSP_UART3_RX_USING_DMA)
#define BSP_UART3_RXTO_IRQ_NUM          INT014_IRQn
#define BSP_UART3_RXTO_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT
#endif
#if defined(BSP_UART3_TX_USING_DMA)
#define BSP_UART3_TX_CPLT_IRQ_NUM       INT086_IRQn
#define BSP_UART3_TX_CPLT_IRQ_PRIO      DDL_IRQ_PRIO_DEFAULT
#endif
#endif /* BSP_USING_UART3 */

#if defined(BSP_USING_UART4)
#define BSP_UART4_RXERR_IRQ_NUM         INT013_IRQn
#define BSP_UART4_RXERR_IRQ_PRIO        DDL_IRQ_PRIO_DEFAULT
#define BSP_UART4_RX_IRQ_NUM            INT091_IRQn
#define BSP_UART4_RX_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT
#define BSP_UART4_TX_IRQ_NUM            INT090_IRQn
#define BSP_UART4_TX_IRQ_PRIO           DDL_IRQ_PRIO_DEFAULT

#if defined(BSP_UART4_RX_USING_DMA)
#define BSP_UART4_RXTO_IRQ_NUM          INT015_IRQn
#define BSP_UART4_RXTO_IRQ_PRIO         DDL_IRQ_PRIO_DEFAULT
#endif
#if defined(BSP_UART4_TX_USING_DMA)
#define BSP_UART4_TX_CPLT_IRQ_NUM       INT087_IRQn
#define BSP_UART4_TX_CPLT_IRQ_PRIO      DDL_IRQ_PRIO_DEFAULT
#endif
#endif /* BSP_USING_UART4 */

#if defined(BSP_USING_CAN1)
#define BSP_CAN1_IRQ_NUM                INT004_IRQn
#define BSP_CAN1_IRQ_PRIO               DDL_IRQ_PRIO_DEFAULT
#endif/* BSP_USING_CAN1 */

/* ============================================================
 * 【2026-11-02】TMR4_3 周期中断 —— FOC 微小时基
 * ------------------------------------------------------------
 * 用途：为 FOC 提供与 PWM 严格同相的 20 kHz 时基。
 *
 * 【硬件事实 1：TMR4 没有 OVF 中断】
 * HC32 TMR4 只有 PEAK / VALLEY 标志：
 *   TMR4_INT_CNT_PEAK   : 三角波到峰值（CNT=2500）触发
 *   TMR4_INT_CNT_VALLEY : 三角波到谷值（CNT=0）触发
 * 三角波走 0→2500→0，完整周期 5000 计数 = 50 μs。
 * 只使能 VALLEY（或只使能 PEAK）→ 每 50 μs 一次 = 20 kHz。
 * 两个都使能会变 40 kHz，这里不需要。
 * 选 VALLEY：该时刻三相下桥刚关断、上桥刚导通，是 FOC 高边
 * 电流采样的标准窗口，后续接 ADC 硬件触发可直接复用。
 * 中断源编号：INT_SRC_TMR4_3_OVF = 358（名为 OVF，实为计数中断）
 *
 * 【硬件事实 2：向量号不能随便选 —— 这是踩过的坑】
 * 实测现象：CNTR 在跑（963→1414）、IRQZEN=1、IRQZF 会置位，
 *           但 ISR 从不进，cycle_cnt 恒 0。
 *
 * 根因：INTC_IrqSignIn() 有一段**硬件分组范围校验**
 *   （hc32_ll_interrupts.c:197-203），公式为：
 *       允许向量 ∈ [ (src/32)*6 + 32 , (src/32)*6 + 37 ]
 *   每组中断源只能映射到 6 个固定向量。
 *
 *   TMR4_3_OVF = 358 → 358/32 = 11 → 允许 INT098..INT103
 *   原先选 INT060 → 校验不通过 → 返回 LL_ERR_INVD_PARAM
 *   → INTC_SEL 未写入、回调未登记
 *   → IRQ060_Handler 调 m_apfnIrqHandler[60]() 是空指针
 *   → 而 DDL 的 IRQ 分发**没有 NULL 保护**，等于跳到 0 地址
 *   （实际表现为中断被丢掉，未观察到 HardFault，但绝不能依赖这点）
 *
 * 校验是 `(void)INTC_IrqSignIn(...)` 丢弃返回值的，
 * 所以失败完全静默 —— 这是本项目"静默失败"倾向的又一例。
 *
 * 【选 INT098 而非 INT103】
 *   两者都在合法窗口内。选窗口下界 INT098：
 *     - 与已用的最大向量 INT091（UART4_RX）最近，便于人工核对不撞车
 *     - 组内从低到高分配，后续 TMR4 其它中断源有 INT099..103 可续
 *   已核对：INT098~INT103 全工程未占用。
 *
 * 【教训】
 *   1) 任何 `(void)INTC_IrqSignIn()` 都必须查返回值，否则映射失败无声无息
 *   2) 中断源 → 向量号不是自由选择，必须按 (src/32)*6+32..+37 计算
 * ============================================================ */
#define BSP_TMR4_3_ISR_IRQ_NUM          INT098_IRQn
#define BSP_TMR4_3_ISR_IRQ_PRIO         DDL_IRQ_PRIO_02   /* 高于 DMA(03)，时基要最稳 */

#ifdef __cplusplus
}
#endif

#endif /* __IRQ_CONFIG_H__ */
