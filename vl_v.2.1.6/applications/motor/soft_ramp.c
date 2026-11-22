/**
 * @file    soft_ramp.c
 * @brief   软启动/停止实现
 */

#include "soft_ramp.h"
#include "framework/state_engine/sm_assert.h"
#include <string.h>

/* ============================================================
 * 初始化
 * ============================================================ */
void soft_ramp_init(soft_ramp_t *ramp, const soft_ramp_cfg_t *cfg)
{
    SM_ASSERT_PTR(ramp);

    memset(ramp, 0, sizeof(soft_ramp_t));
    ramp->cfg = cfg;
}

/* ============================================================
 * 启动斜坡
 * ============================================================ */
void soft_ramp_start(soft_ramp_t *ramp, int32_t from, int32_t to)
{
    SM_ASSERT_PTR(ramp);

    if (ramp->cfg == NULL || !ramp->cfg->start_enabled) {
        ramp->current_value = to;
        ramp->is_running = 0;
        return;
    }

    ramp->start_value = from;
    ramp->target_value = to;
    ramp->current_value = from;
    ramp->elapsed_ms = 0;
    ramp->is_running = 1;
    ramp->mode = 1;
}

/* ============================================================
 * 停止斜坡
 * ============================================================ */
void soft_ramp_stop(soft_ramp_t *ramp, int32_t from)
{
    SM_ASSERT_PTR(ramp);

    if (ramp->cfg == NULL || !ramp->cfg->stop_enabled) {
        ramp->current_value = 0;
        ramp->is_running = 0;
        return;
    }

    ramp->start_value = from;
    ramp->target_value = 0;
    ramp->current_value = from;
    ramp->elapsed_ms = 0;
    ramp->is_running = 1;
    ramp->mode = 0;
}

/* ============================================================
 * 周期更新（线性插值）
 * ============================================================ */
int32_t soft_ramp_update(soft_ramp_t *ramp, uint32_t dt_ms)
{
    SM_ASSERT_PTR(ramp);

    if (!ramp->is_running || ramp->cfg == NULL) {
        return ramp->current_value;
    }

    uint32_t total_time;
    int32_t  delta;
    int32_t  step;

    ramp->elapsed_ms += dt_ms;

    total_time = (ramp->mode == 1) ? ramp->cfg->start_time_ms
                                    : ramp->cfg->stop_time_ms;

    if (total_time == 0) {
        ramp->current_value = ramp->target_value;
        ramp->is_running = 0;
        return ramp->current_value;
    }

    /* 线性插值 */
    delta = ramp->target_value - ramp->start_value;

    if (ramp->elapsed_ms >= total_time) {
        /* 斜坡完成 */
        ramp->current_value = ramp->target_value;
        ramp->is_running = 0;
        return ramp->current_value;
    }

    step = delta * (int32_t)ramp->elapsed_ms / (int32_t)total_time;
    ramp->current_value = ramp->start_value + step;

    return ramp->current_value;
}

uint8_t soft_ramp_is_done(const soft_ramp_t *ramp)
{
    SM_ASSERT_PTR(ramp);
    return !ramp->is_running;
}
