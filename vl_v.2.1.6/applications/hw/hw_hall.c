/**
 * @file    hw_hall.c
 * @brief   Hall 传感器设备接口实现
 */

#include "hw_hall.h"
#include "framework/state_engine/sm_assert.h"
#include <rtthread.h>


/* ============================================================
 * 标准 API 实现
 * ============================================================ */
rt_err_t hw_hall_init(hw_hall_t *self,
                      const hw_hall_cfg_t *cfg,
                      const hw_hall_ops_t *ops)
{
    SM_ASSERT_PTR(self);
    SM_ASSERT_PTR(cfg);
    SM_ASSERT_PTR(ops);

    self->cfg               = cfg;
    self->ops               = ops;
    self->is_ready          = 0;
    self->callback          = RT_NULL;
    self->user_data         = RT_NULL;
    self->last_capture_tick = 0;
    self->capture_count     = 0;
    self->last_state        = 0xFF;     /* 强制首次触发 */

    if (ops->init) {
        rt_err_t ret = ops->init(self);
        if (ret != RT_EOK) {
            rt_kprintf("[HW-HALL] %s init failed (%d)\n", cfg->name, ret);
            return ret;
        }
    }

    self->is_ready = 1;

    /* 初始读取一次状态 */
    self->last_state = hw_hall_read_state(self);

    rt_kprintf("[HW-HALL] %s initialized\n", cfg->name);
    return RT_EOK;
}

rt_err_t hw_hall_deinit(hw_hall_t *self)
{
    SM_ASSERT_PTR(self);
    if (!self->is_ready) return RT_EOK;

    rt_err_t ret = RT_EOK;
    if (self->ops->deinit) {
        ret = self->ops->deinit(self);
    }
    self->is_ready = 0;
    return ret;
}

uint8_t hw_hall_read_state(hw_hall_t *self)
{
    SM_ASSERT_PTR(self);
    if (!self->is_ready) return HW_HALL_STATE_INVALID_LOW;
    if (self->ops->read_state) {
        return self->ops->read_state(self);
    }
    return HW_HALL_STATE_INVALID_LOW;
}

rt_err_t hw_hall_set_callback(hw_hall_t *self,
                               hw_hall_callback_t cb,
                               void *user_data)
{
    SM_ASSERT_PTR(self);

    self->callback = cb;
    self->user_data = user_data;
    return RT_EOK;
}

uint32_t hw_hall_get_last_tick(const hw_hall_t *self)
{
    SM_ASSERT_PTR(self);
    return self->last_capture_tick;
}

uint32_t hw_hall_get_count(const hw_hall_t *self)
{
    SM_ASSERT_PTR(self);
    return self->capture_count;
}

/* ============================================================
 * ISR 入口（BSP 层调用）
 * ============================================================ */
void hw_hall_on_edge(hw_hall_t *self, uint8_t new_state)
{
    if (self == RT_NULL || !self->is_ready) return;

    /* ============================================================
     * 1. 去抖：状态未变化，忽略
     * ============================================================ */
    if (new_state == self->last_state) {
        return;
    }

    /* ============================================================
     * 2. 记录捕获信息（用于转速计算）
     * ============================================================ */
    self->last_capture_tick = rt_tick_get();
    self->capture_count++;
    self->last_state = new_state;

    /* ============================================================
     * 3. 调用用户回调（换相处理）
     * ============================================================ */
    if (self->callback) {
        self->callback(self, self->user_data);
    }
}
