/**
 * @file    hw_adc.c
 * @brief   ADC 设备接口实现
 */

#include "hw_adc.h"
#include "framework/state_engine/sm_assert.h"


/* ============================================================
 * 全局 ADC 实例（供外部引用）
 * ============================================================ */
hw_adc_t g_hw_adc = {0};


/* ============================================================
 * 标准 API 实现
 * ============================================================ */

rt_err_t hw_adc_init(hw_adc_t *self,
                     const void *cfg,
                     const hw_adc_ops_t *ops)
{
    SM_ASSERT_PTR(self);
    SM_ASSERT_PTR(ops);
    SM_ASSERT_PTR(ops->init);

    /* ★ ops 必须保存到对象，否则 read_mv/read_raw 中的 self->ops 恒为 NULL */
    self->ops      = ops;
    self->is_ready = 0;

    /* cfg 允许为 NULL：是否必需由具体 ops 决定，抽象层不做强制 */
    rt_err_t ret = ops->init(self, cfg);
    if (ret == RT_EOK) {
        self->is_ready = 1;
    }
    return ret;
}

rt_err_t hw_adc_deinit(hw_adc_t *self)
{
    SM_ASSERT_PTR(self);

    if (!self->is_ready) return RT_EOK;

    rt_err_t ret = RT_EOK;
    if (self->ops && self->ops->deinit) {
        ret = self->ops->deinit(self);
    }
    self->is_ready = 0;
    return ret;
}

uint16_t hw_adc_read_mv(hw_adc_t *self, uint8_t ch)
{
    SM_ASSERT_PTR(self);

    if (!self->is_ready) return 0;
    if (self->ops && self->ops->read_mv) {
        return self->ops->read_mv(self, ch);
    }
    return 0;
}

uint16_t hw_adc_read_raw(hw_adc_t *self, uint8_t ch)
{
    SM_ASSERT_PTR(self);

    if (!self->is_ready) return 0;
    if (self->ops && self->ops->read_raw) {
        return self->ops->read_raw(self, ch);
    }
    return 0;
}
