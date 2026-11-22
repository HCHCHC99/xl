/**
 * @file    brake_provider_timer.c
 * @brief   刹车盘状态源：软件计时
 * @note    默认实现，仅根据 elapsed_ms 判定到位
 */

#include "../hw/hw_brake.h"

/* ============================================================
 * 状态源接口实现
 * ============================================================ */
static void timer_reset(hw_brake_t *self)
{
    (void)self;
}

static void timer_tick(hw_brake_t *self, uint32_t dt_ms)
{
    (void)self;
    (void)dt_ms;
}

static uint8_t timer_is_unlock_done(hw_brake_t *self)
{
    return (self->elapsed_ms >= self->cfg->unlock_ms) ? 1U : 0U;
}

static uint8_t timer_is_lock_done(hw_brake_t *self)
{
    return (self->elapsed_ms >= self->cfg->lock_ms) ? 1U : 0U;
}

/* ============================================================
 * 接口表
 * ============================================================ */
const brake_provider_ops_t g_brake_provider_timer = {
    .reset          = timer_reset,
    .tick           = timer_tick,
    .is_unlock_done = timer_is_unlock_done,
    .is_lock_done   = timer_is_lock_done,
};
