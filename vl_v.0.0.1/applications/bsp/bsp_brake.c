/**
 * @file    bsp_brake.c
 * @brief   刹车盘 BSP 层实现,（GPIO 直驱）
 * @note    通电解锁，断电锁止，上电默认锁止
 */

#include "bsp_brake.h"
#include "board_config.h"
#include "framework/state_engine/sm_assert.h"
#include <string.h>
#include <rtthread.h>

/* ============================================================
 * HC32F452 底层驱动
 * ============================================================ */
#include "hc32_ll.h"
#include "hc32_ll_gpio.h"

/* ============================================================
 * 引脚定义
 * ============================================================
 * 引脚宏已归一化到 board/board_config.h（BRAKE_PORT / BRAKE_PIN /
 * BRAKE_ACTIVE_LOW），引脚配置由 rt_hw_board_brake_init() 完成。
 * ============================================================ */

/* ============================================================
 * 私有数据
 * ============================================================ */
bsp_brake_priv_t g_bsp_brake_priv = {0};

/* ============================================================
 * 内部辅助
 * ============================================================ */
static inline void bsp_brake_set(uint8_t enable)
{
#if BRAKE_ACTIVE_LOW
    enable = (enable != 0U) ? 0U : 1U;
#endif
    if (enable) {
        GPIO_SetPins(BRAKE_PORT, BRAKE_PIN);
    } else {
        GPIO_ResetPins(BRAKE_PORT, BRAKE_PIN);
    }
}

/* ============================================================
 * Ops 实现
 * ============================================================ */
static rt_err_t bsp_brake_init_impl(hw_brake_t *self)
{
    SM_ASSERT_PTR(self);

    /* 引脚与初始电平由 rt_hw_board_brake_init() 完成
     * 此处再次拉低，确保对象绑定时处于锁止态（幂等） */
    bsp_brake_set(0);
    rt_kprintf("[BSP-BRAKE] %s object bound (locked)\n", self->cfg->name);

    return RT_EOK;
}

static rt_err_t bsp_brake_unlock_impl(hw_brake_t *self)
{
    (void)self;
    bsp_brake_set(1);
    return RT_EOK;
}

static rt_err_t bsp_brake_lock_impl(hw_brake_t *self)
{
    (void)self;
    bsp_brake_set(0);
    return RT_EOK;
}

static rt_err_t bsp_brake_emergency_lock_impl(hw_brake_t *self)
{
    (void)self;
    bsp_brake_set(0);   /* 直接断电，ISR 安全 */
    return RT_EOK;
}

static rt_err_t bsp_brake_clear_fault_impl(hw_brake_t *self)
{
    (void)self;
    return RT_EOK;
}

static rt_err_t bsp_brake_deinit_impl(hw_brake_t *self)
{
    (void)self;
    bsp_brake_set(0);
    return RT_EOK;
}

/* ============================================================
 * Ops 表
 * ============================================================ */
const hw_brake_ops_t g_bsp_brake_ops = {
    .init           = bsp_brake_init_impl,
    .unlock         = bsp_brake_unlock_impl,
    .lock           = bsp_brake_lock_impl,
    .emergency_lock = bsp_brake_emergency_lock_impl,
    .clear_fault    = bsp_brake_clear_fault_impl,
    .deinit         = bsp_brake_deinit_impl,
};

///* ============================================================
// * 全局实例注册
// * ============================================================ */
//void bsp_brake_register(void)
//{
//    hw_brake_init(&g_hw_brake_swing,
//                  &g_bsp_brake_ops,
//                  &g_brake_provider_timer,
//                  NULL,
//                  150,      /* unlock_ms  */
//                  50,       /* lock_ms    */
//                  500);     /* timeout_ms */
//
//    rt_kprintf("[BSP-BRAKE] Registered\n");
//}
