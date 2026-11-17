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


#ifndef __BOARD_CONFIG_H__
#define __BOARD_CONFIG_H__

#include <rtconfig.h>
#include "hc32_ll.h"
#include "drv_config.h"

/* ============================================================
 * 编码器选择
 *   1 = FOC 工程的 ABZ 增量编码器（TMRA1，4096 CPR，PA8/PA9/PA10）
 *   0 = 原 SPI 绝对编码器（KTH7812，SPI1）
 *
 * 注意：为 1 时 SPI 编码器代码仍然编译保留，但 bsp_hw_init 不再初始化它。
 * ============================================================ */
#ifndef BSP_USING_ABS_ENCODER
#define BSP_USING_ABS_ENCODER           1
#endif

#if BSP_USING_ABS_ENCODER
/* ABZ 编码器（FOC 工程）：TMRA1 正交计数 + Z 索引
 *   PA8  = A 相（TIMA1_CLKA）
 *   PA9  = B 相（TIMA1_CLKB）
 *   PA10 = Z 索引（EXTINT_CH10）
 */
#define ENC_A_PORT                      (GPIO_PORT_A)
#define ENC_A_PIN                       (GPIO_PIN_08)
#define ENC_A_FUNC                      (GPIO_FUNC_4)

#define ENC_B_PORT                      (GPIO_PORT_A)
#define ENC_B_PIN                       (GPIO_PIN_09)
#define ENC_B_FUNC                      (GPIO_FUNC_4)

#define ENC_Z_PORT                      (GPIO_PORT_A)
#define ENC_Z_PIN                       (GPIO_PIN_10)
#define ENC_Z_EIRQ                      (EXTINT_CH10)
#define ENC_Z_IRQn                      (INT010_IRQn)
#define ENC_Z_IRQ_SRC                   (INT_SRC_PORT_EIRQ10)
#endif


/************************ USART port **********************/
#if defined(BSP_USING_UART1)
    #define USART1_RX_PORT                  (GPIO_PORT_A)
    #define USART1_RX_PIN                   (GPIO_PIN_01)

    #define USART1_TX_PORT                  (GPIO_PORT_A)
    #define USART1_TX_PIN                   (GPIO_PIN_00)
#endif

#if defined(BSP_USING_UART2)
    #define USART2_RX_PORT                  (GPIO_PORT_A)
    #define USART2_RX_PIN                   (GPIO_PIN_03)

    #define USART2_TX_PORT                  (GPIO_PORT_A)
    #define USART2_TX_PIN                   (GPIO_PIN_02)
#endif

#if defined(BSP_USING_UART3)
    #define USART3_RX_PORT                  (GPIO_PORT_C)
    #define USART3_RX_PIN                   (GPIO_PIN_13)

    #define USART3_TX_PORT                  (GPIO_PORT_H)
    #define USART3_TX_PIN                   (GPIO_PIN_02)
#endif

#if defined(BSP_USING_UART4)
    #define USART4_RX_PORT                  (GPIO_PORT_B)
    #define USART4_RX_PIN                   (GPIO_PIN_09)

    #define USART4_TX_PORT                  (GPIO_PORT_E)
    #define USART4_TX_PIN                   (GPIO_PIN_06)
#endif

/***********  CAN configure *********/
#if defined(BSP_USING_CAN1)
    #define CAN1_TX_PORT                     (GPIO_PORT_B)
    #define CAN1_TX_PIN                      (GPIO_PIN_13)
    #define CAN1_TX_PIN_FUNC                 (GPIO_FUNC_50)

    #define CAN1_RX_PORT                     (GPIO_PORT_B)
    #define CAN1_RX_PIN                      (GPIO_PIN_12)
    #define CAN1_RX_PIN_FUNC                 (GPIO_FUNC_51)

//    #define CAN1_STB_FUNC_ENABLE
    #define CAN_STB_PORT                     (GPIO_PORT_D)
    #define CAN_STB_PIN                      (GPIO_PIN_15)
#endif

/***********  ADC configure *********/
#if defined(BSP_USING_ADC1)
    /* 【2026-10-29 电流采样】用户实测确认的引脚映射：
     *   PA3  ADC1_IN3   CURRENT_TOTAL   母线总电流
     *   PA4  ADC12_IN4  CURRENT_W       W 相（上桥臂 MOS 的 D 端）
     *   PA5  ADC12_IN5  CURRENT_V       V 相（上桥臂 MOS 的 D 端）
     * 三路全在 ADC1，可用单个扫描序列 + 单路 DMA 一次搬完。
     * 注意：不得改用 SPI 的备选引脚 PA4/PA5（见 SPI 段落注释）。 */
    #define ADC1_CH3_PORT                     (GPIO_PORT_A)
    #define ADC1_CH3_PIN                      (GPIO_PIN_03)

    #define ADC1_CH4_PORT                     (GPIO_PORT_A)
    #define ADC1_CH4_PIN                      (GPIO_PIN_04)

    #define ADC1_CH5_PORT                     (GPIO_PORT_A)
    #define ADC1_CH5_PIN                      (GPIO_PIN_05)

//    #define ADC1_CH10_PORT                     (GPIO_PORT_C)
//    #define ADC1_CH10_PIN                      (GPIO_PIN_00)
//
//    #define ADC1_CH12_PORT                     (GPIO_PORT_C)
//    #define ADC1_CH12_PIN                      (GPIO_PIN_02)
//
//    #define ADC1_CH13_PORT                     (GPIO_PORT_C)
//    #define ADC1_CH13_PIN                      (GPIO_PIN_03)
#endif

#if defined(BSP_USING_ADC2)
    //ADC2 has 7 channels CH0-CH7. ADC12_IN4-ADC12_IN11 means ADC2 CH0-CH7

    #define ADC2_CH4_PORT                     (GPIO_PORT_A)
    #define ADC2_CH4_PIN                      (GPIO_PIN_04)

    #define ADC2_CH5_PORT                     (GPIO_PORT_A)
    #define ADC2_CH5_PIN                      (GPIO_PIN_05)

    #define ADC2_CH6_PORT                     (GPIO_PORT_A)
    #define ADC2_CH6_PIN                      (GPIO_PIN_06)

    #define ADC2_CH7_PORT                     (GPIO_PORT_A)
    #define ADC2_CH7_PIN                      (GPIO_PIN_07)


#endif
/***********  SPI configure *********/

#ifdef BSP_USING_SPI1
#ifndef SPI1_BUS_CONFIG
#define SPI1_BUS_CONFIG                             \
    {                                               \
        .Instance = CM_SPI1,                           \
        .bus_name = "spi1",                         \
        .clock = 100000,                           \
    }

    #define SPI_SCK_PORT                      (GPIO_PORT_A)
    #define SPI_SCK_PIN                       (GPIO_PIN_12)
//    #define SPI_SCK_PIN                       (GPIO_PIN_04)

    #define SPI_MOSI_PORT                     (GPIO_PORT_A)
    #define SPI_MOSI_PIN                      (GPIO_PIN_11)
//    #define SPI_MOSI_PIN                      (GPIO_PIN_05)

    #define SPI_MISO_PORT                     (GPIO_PORT_A)
    #define SPI_MISO_PIN                      (GPIO_PIN_10)
//    #define SPI_MISO_PIN                      (GPIO_PIN_06)

    #define SPI_SCK_FUNC                      (GPIO_FUNC_43)
    #define SPI_MOSI_FUNC                     (GPIO_FUNC_40)
    #define SPI_MISO_FUNC                     (GPIO_FUNC_41)


#endif  /* SPI1_BUS_CONFIG */
#endif  /* BSP_USING_SPI1 */

/***********  PWM configure（TIM4_3 三相互补，电机驱动专用） *********/
/*
 * 注意：本组引脚为 TIM4_3 互补 PWM 输出，由 bsp_pwm.c 直接驱动，
 *       不属于 RT-Thread PWM 设备框架。
 *
 * 引脚分配（与 bsp_pwm.c 头注释保持一致）：
 *   U 相：UH-PB9 / UL-PB8
 *   V 相：VH-PB7 / VL-PB6
 *   W 相：WH-PB5 / WL-PB4
 *   复用功能：全部 GPIO_FUNC_2（TIM4_3，与参考 FOC 工程及 drv_pwm.c 一致）
 *
 * [PERF-EVAL-01] 当前使用 hc32_ll GPIO_SetFunc 直接配置。
 *   待评估：若改用 rt_pin 框架，PWM 换相 ISR 内的引脚操作开销
 *   与 EXTI 分发延迟是否可接受。评估完成前不得改动本实现。
 */
#define BSP_USING_PWM_TIM4_3

#define PWM_UH_PORT             (GPIO_PORT_B)
#define PWM_UH_PIN              (GPIO_PIN_09)
#define PWM_UH_PIN_FUNC         (GPIO_FUNC_2)

#define PWM_UL_PORT             (GPIO_PORT_B)
#define PWM_UL_PIN              (GPIO_PIN_08)
#define PWM_UL_PIN_FUNC         (GPIO_FUNC_2)

#define PWM_VH_PORT             (GPIO_PORT_B)
#define PWM_VH_PIN              (GPIO_PIN_07)
#define PWM_VH_PIN_FUNC         (GPIO_FUNC_2)

#define PWM_VL_PORT             (GPIO_PORT_B)
#define PWM_VL_PIN              (GPIO_PIN_06)
#define PWM_VL_PIN_FUNC         (GPIO_FUNC_2)

#define PWM_WH_PORT             (GPIO_PORT_B)
#define PWM_WH_PIN              (GPIO_PIN_05)
#define PWM_WH_PIN_FUNC         (GPIO_FUNC_2)

#define PWM_WL_PORT             (GPIO_PORT_B)
#define PWM_WL_PIN              (GPIO_PIN_04)
#define PWM_WL_PIN_FUNC         (GPIO_FUNC_2)

/***********  BRAKE configure（电磁刹车） *********/
/*
 * 电磁刹车 GPIO：高电平解锁，断电锁止，上电默认锁止。
 * [PERF-EVAL-03] 同 PERF-EVAL-01，评估完成前不得改动。
 */
#define BRAKE_PORT              (GPIO_PORT_B)
#define BRAKE_PIN               (GPIO_PIN_14)
#define BRAKE_ACTIVE_LOW        (0)     /* 0：高电平解锁；1：低电平解锁 */

/***********  PC15 上电使能 *********/
/* 上电后立即配置为推挽输出并输出高电平。 */
#define BSP_PC15_EN_PORT        (GPIO_PORT_C)
#define BSP_PC15_EN_PIN         (GPIO_PIN_15)

/***********  编码器 ABZ 增量信号（Q35：本批仅预留，不实现功能） ***********/
/*
 * 【2026-11-12 用户裁定 Q35 = 预留】
 *   硬件已引出 Z0 编码器的 ABZ 三路信号，用于将来 FOC 电角度解算。
 *   本批【只定义引脚、不做任何初始化、不接任何业务】，
 *   以免占用中断/定时器资源，也避免与现有 SPI 绝对角读取冲突。
 *
 *   引脚（用户 2026-11-12 给定）：
 *     Z0_A → PA0      增量 A 相
 *     Z0_B → PA1      增量 B 相
 *     Z0_Z → PA15     零位 Z 相
 *
 * ⚠️ 将来实现时的注意点（先记下来，免得重踩）：
 *   1) PA15 上电默认为 JTDI（JTAG 数据输入）。若用 SWD 调试，
 *      需先关闭 JTAG 只保留 SWD，否则 PA15 无法作为普通 IO/复用脚。
 *   2) PA0/PA1 在当前工程【未被串口占用】：board_config.h 的
 *      USART1/USART2 引脚块整段位于 `#if defined(BSP_USING_UART1/2)` 内，
 *      而 rtconfig.h 中 BSP_USING_UART 已被注释掉 ⇒ 不生效。
 *      若将来启用 UART1/UART2，PA0/PA1 会与 ABZ 冲突，必须择一。
 *   3) 三路 ABZ 建议走【定时器编码器模式】而非 EXTI 双边沿：
 *      现有 Hall 已占 PH2/PC13/PC14 三个 EXTI 通道，再叠 3 路会显著加重中断负载。
 */
#define ENC_ABZ_Z0_A_PORT       (GPIO_PORT_A)
#define ENC_ABZ_Z0_A_PIN        (GPIO_PIN_00)

#define ENC_ABZ_Z0_B_PORT       (GPIO_PORT_A)
#define ENC_ABZ_Z0_B_PIN        (GPIO_PIN_01)

#define ENC_ABZ_Z0_Z_PORT       (GPIO_PORT_A)
#define ENC_ABZ_Z0_Z_PIN        (GPIO_PIN_15)

/***********  板级引脚初始化函数声明 ***********/
/*
 * 本组函数只做「引脚/复用功能」配置，不初始化外设寄存器。
 * 外设寄存器由 applications/bsp/bsp_*_hw_init() 负责。
 */
#if defined(BSP_USING_PWM_TIM4_3)
rt_err_t rt_hw_board_pwm_init(CM_TMR4_TypeDef *CM_TMRx);
#endif

rt_err_t rt_hw_board_brake_init(void);

#endif /* __BOARD_CONFIG_H__ */
