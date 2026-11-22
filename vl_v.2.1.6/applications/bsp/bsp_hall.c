/**
 * @file    bsp_hall.c
 * @brief   Hall 传感器 BSP 层实现（HC32F452）
 * @note    Hall 引脚：
 *            HALL-A: PH2  → EXTINT_CH2  → INT002_IRQn
 *            HALL-B: PC13 → EXTINT_CH13 → INT013_IRQn
 *            HALL-C: PC14 → EXTINT_CH14 → INT014_IRQn
 *          触发方式：双边沿
 *          滤波：硬件数字滤波（EXTINT_FILTER_ON）
 */

#include "bsp_hall.h"
#include "bsp_hw_init.h"
#include "../motor/board_motor_cfg.h"
#include "framework/state_engine/sm_assert.h"
#include <string.h>
#include <rtthread.h>

/* ============================================================
 * HC32F452 底层驱动
 * ============================================================ */
#include "hc32_ll.h"
#include "hc32_ll_gpio.h"

#include "hc32_ll_interrupts.h"
#include "hc32_ll_fcg.h"

/* ============================================================
 * 引脚定义
 * ============================================================ */

/* ----- HALL-A：PH2 ----- */
#define HALL_A_PORT             (GPIO_PORT_H)
#define HALL_A_PIN              (GPIO_PIN_02)
#define HALL_A_EXTINT_CH        (EXTINT_CH02)
#define HALL_A_INT_SRC          (INT_SRC_PORT_EIRQ2)
#define HALL_A_INT_IRQn         (INT002_IRQn)
#define HALL_A_INT_PRIO         (DDL_IRQ_PRIO_DEFAULT)

/* ----- HALL-B：PC13 ----- */
#define HALL_B_PORT             (GPIO_PORT_C)
#define HALL_B_PIN              (GPIO_PIN_13)
#define HALL_B_EXTINT_CH        (EXTINT_CH13)
#define HALL_B_INT_SRC          (INT_SRC_PORT_EIRQ13)
#define HALL_B_INT_IRQn         (INT013_IRQn)
#define HALL_B_INT_PRIO         (DDL_IRQ_PRIO_DEFAULT)

/* ----- HALL-C：PC14 ----- */
#define HALL_C_PORT             (GPIO_PORT_C)
#define HALL_C_PIN              (GPIO_PIN_14)
#define HALL_C_EXTINT_CH        (EXTINT_CH14)
#define HALL_C_INT_SRC          (INT_SRC_PORT_EIRQ14)
#define HALL_C_INT_IRQn         (INT014_IRQn)
#define HALL_C_INT_PRIO         (DDL_IRQ_PRIO_DEFAULT)


/* ============================================================
 * BSP 私有数据
 * ============================================================ */
bsp_hall_priv_t g_bsp_hall_priv = {0};

/* ============================================================
 * 内部：读取 3 路 Hall 状态
 * ============================================================ */
static uint8_t bsp_hall_read_state_impl(hw_hall_t *self)
{
    (void)self;

    uint8_t state = 0;

    /* 读取 3 路 GPIO，组合为 3 位状态 */
    /* 位序：bit2=HALL_A, bit1=HALL_B, bit0=HALL_C */
    if (GPIO_ReadInputPins(HALL_A_PORT, HALL_A_PIN) != 0) {
        state |= (1 << 2);
    }
    if (GPIO_ReadInputPins(HALL_B_PORT, HALL_B_PIN) != 0) {
        state |= (1 << 1);
    }
    if (GPIO_ReadInputPins(HALL_C_PORT, HALL_C_PIN) != 0) {
        state |= (1 << 0);
    }

    return state;
}

/* ============================================================
 * 中断服务程序
 * ============================================================ */

/**
 * @brief HALL-A 中断回调（PH2）
 * @note  IRQ No.2 in Global IRQ entry No.0~31 is used for EXTINT2
 */
static void EXTINT_HALL_A_IrqCallback(void)
{
    if (SET == EXTINT_GetExtIntStatus(HALL_A_EXTINT_CH)) {
        EXTINT_ClearExtIntStatus(HALL_A_EXTINT_CH);
        hw_hall_on_edge(&g_swing_hall, bsp_hall_read_state_impl(&g_swing_hall));
    }
}

/**
 * @brief HALL-B 中断回调（PC13）
 */
static void EXTINT_HALL_B_IrqCallback(void)
{
    if (SET == EXTINT_GetExtIntStatus(HALL_B_EXTINT_CH)) {
        EXTINT_ClearExtIntStatus(HALL_B_EXTINT_CH);
        hw_hall_on_edge(&g_swing_hall, bsp_hall_read_state_impl(&g_swing_hall));
    }
}

/**
 * @brief HALL-C 中断回调（PC14）
 */
static void EXTINT_HALL_C_IrqCallback(void)
{
    if (SET == EXTINT_GetExtIntStatus(HALL_C_EXTINT_CH)) {
        EXTINT_ClearExtIntStatus(HALL_C_EXTINT_CH);
        hw_hall_on_edge(&g_swing_hall, bsp_hall_read_state_impl(&g_swing_hall));
    }
}

/* ============================================================
 * 内部：Hall 单路初始化（GPIO + EXTINT + IRQ）
 * ============================================================ */
static void bsp_hall_pin_init(uint8_t port, uint16_t pin,
                               uint32_t  extint_ch,
                               en_int_src_t int_src,
                               IRQn_Type irqn,
                               func_ptr_t callback,
                               uint32_t priority)
{
    stc_gpio_init_t stcGpioInit;
    stc_extint_init_t stcExtIntInit;
    stc_irq_signin_config_t stcIrqSignConfig;

    /* ----- 1. GPIO 配置 ----- */
    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16ExtInt = PIN_EXTINT_ON;
    stcGpioInit.u16PullUp = PIN_PU_ON;
    (void)GPIO_Init(port, pin, &stcGpioInit);

    /* ----- 2. EXTINT 配置 ----- */
    (void)EXTINT_StructInit(&stcExtIntInit);
    stcExtIntInit.u32Filter      = EXTINT_FILTER_ON;
    stcExtIntInit.u32FilterClock = EXTINT_FCLK_DIV8;
    stcExtIntInit.u32Edge        = EXTINT_TRIG_BOTH;   /* 双边沿 */
    (void)EXTINT_Init(extint_ch, &stcExtIntInit);

    /* ----- 3. 中断注册 ----- */
    stcIrqSignConfig.enIntSrc    = int_src;
    stcIrqSignConfig.enIRQn      = irqn;
    stcIrqSignConfig.pfnCallback = callback;
    (void)INTC_IrqSignIn(&stcIrqSignConfig);

    /* ----- 4. NVIC 配置 ----- */
    NVIC_ClearPendingIRQ(irqn);
    NVIC_SetPriority(irqn, priority);
    NVIC_EnableIRQ(irqn);
}


/* ============================================================
 * 内部：全部 Hall 初始化
 * ============================================================ */
static void bsp_hall_exti_init(void)
{
    /* 使能外设时钟（参照示例：EFM | FCG | GPIO | PWC_CLK_RMU | SRAM） */
    LL_PERIPH_WE(LL_PERIPH_EFM | LL_PERIPH_FCG | LL_PERIPH_GPIO |
                 LL_PERIPH_PWC_CLK_RMU | LL_PERIPH_SRAM);

    /* ============================================================
     * EXTI 通道 2（PH2 - HALL_A）
     * ============================================================ */
    bsp_hall_pin_init(HALL_A_PORT, HALL_A_PIN,
                       HALL_A_EXTINT_CH,
                       HALL_A_INT_SRC,
                       HALL_A_INT_IRQn,
                       &EXTINT_HALL_A_IrqCallback,
                       HALL_A_INT_PRIO);


    /* ============================================================
     * EXTI 通道 13（PC13 - HALL_B）
     * ============================================================ */
    bsp_hall_pin_init(HALL_B_PORT, HALL_B_PIN,
                       HALL_B_EXTINT_CH,
                       HALL_B_INT_SRC,
                       HALL_B_INT_IRQn,
                       &EXTINT_HALL_B_IrqCallback,
                       HALL_B_INT_PRIO);

    /* ============================================================
     * EXTI 通道 14（PC14 - HALL_C）
     * ============================================================ */
    bsp_hall_pin_init(HALL_C_PORT, HALL_C_PIN,
                       HALL_C_EXTINT_CH,
                       HALL_C_INT_SRC,
                       HALL_C_INT_IRQn,
                       &EXTINT_HALL_C_IrqCallback,
                       HALL_C_INT_PRIO);

    LL_PERIPH_WP(LL_PERIPH_EFM | LL_PERIPH_FCG | LL_PERIPH_GPIO |
                 LL_PERIPH_PWC_CLK_RMU | LL_PERIPH_SRAM);

    rt_kprintf("[BSP-Hall] GPIO + EXTINT configured\n");
}

/* ============================================================
 * BSP ops 实现
 * ============================================================ */

static rt_err_t bsp_hall_init_impl(hw_hall_t *self)
{
    SM_ASSERT_PTR(self);

    /* 硬件由 bsp_hw_init 表统一初始化，此处只校验不改寄存器 */
    if (!g_bsp_hall_priv.initialized) {
        rt_kprintf("[BSP-Hall] %s: hardware not initialised\n", self->cfg->name);
        return -RT_ERROR;
    }

    return RT_EOK;
}

/* ============================================================
 * 硬件初始化（由 bsp_hw_init 表调用）
 * ============================================================
 * Hall 三路 GPIO + EXTI + NVIC 强耦合（EXTI 通道号与引脚号绑定），
 * 故整体保留在本文件，不拆分到 board_config.c。
 * [PERF-EVAL-02] 当前用 hc32_ll INTC_IrqSignIn + 独立 IRQn。
 *   待评估：改用 rt_pin_attach_irq 的统一中断分发是否会增加延迟。
 * ============================================================ */
int bsp_hall_hw_init(void *arg)
{
    (void)arg;

    if (g_bsp_hall_priv.initialized) {
        return 0;   /* 幂等 */
    }

    bsp_hall_exti_init();

    g_bsp_hall_priv.active_level = MOTOR_HALL_ACTIVE_LEVEL;
    g_bsp_hall_priv.initialized  = 1;

    return 0;
}

static rt_err_t bsp_hall_deinit_impl(hw_hall_t *self)
{
    SM_ASSERT_PTR(self);

    if (!g_bsp_hall_priv.initialized) return RT_EOK;

    /* 关闭 3 路 EXTI */
    NVIC_DisableIRQ(HALL_A_INT_IRQn);
    NVIC_DisableIRQ(HALL_B_INT_IRQn);
    NVIC_DisableIRQ(HALL_C_INT_IRQn);

    g_bsp_hall_priv.initialized = 0;
    return RT_EOK;
}

/* ============================================================
 * BSP ops 表
 * ============================================================ */
const hw_hall_ops_t g_bsp_hall_ops = {
    .init       = bsp_hall_init_impl,
    .deinit     = bsp_hall_deinit_impl,
    .read_state = bsp_hall_read_state_impl,
};

///* ============================================================
// * 全局实例注册
// * ============================================================ */
//void bsp_hall_register(void)
//{
//    g_hw_hall.name = "hall";
//    g_hw_hall.ops = &g_bsp_hall_ops;
//    g_hw_hall.priv = &g_bsp_hall_priv;
//    g_hw_hall.is_ready = 0;
//
//    rt_kprintf("[BSP-Hall] Registered\n");
//}
