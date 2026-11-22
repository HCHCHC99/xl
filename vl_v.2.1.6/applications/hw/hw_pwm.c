/**
 * @file    hw_pwm.c
 * @brief   三相 PWM 设备接口实现
 * @note    只做 API 转发和参数校验
 */

#include "hw_pwm.h"
#include "framework/state_engine/sm_assert.h"

/* ============================================================
 * 标准 API 实现
 * ============================================================ */

rt_err_t hw_pwm_init(hw_pwm_t *self,
                     const hw_pwm_cfg_t *cfg,
                     const hw_pwm_ops_t *ops)
{
    SM_ASSERT_PTR(self);
    SM_ASSERT_PTR(cfg);
    SM_ASSERT_PTR(ops);

    if (cfg->max_duty == 0U) return -RT_EINVAL;

    self->cfg      = cfg;   /* ★ 零拷贝，只存指针 */
    self->ops      = ops;
    self->is_ready = 0;

    rt_err_t ret = ops->init(self);
    if (ret != RT_EOK) {
        rt_kprintf("[HW-PWM] %s init failed (%d)\n", cfg->name, ret);
        return ret;
    }

    self->is_ready = 1;
    rt_kprintf("[HW-PWM] %s initialized\n", cfg->name);
    return RT_EOK;
}

rt_err_t hw_pwm_deinit(hw_pwm_t *self)
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

rt_err_t hw_pwm_set_duty(hw_pwm_t *self, uint8_t phase, uint32_t duty)
{
    SM_ASSERT_PTR(self);
    if (!self->is_ready) return -RT_ERROR;

    if (duty > self->cfg->max_duty) duty = self->cfg->max_duty;
    return self->ops->set_duty(self, phase, duty);
}

rt_err_t hw_pwm_enable(hw_pwm_t *self)
{
    SM_ASSERT_PTR(self);
    if (!self->is_ready) return -RT_ERROR;
    return self->ops->enable(self);
}

rt_err_t hw_pwm_disable(hw_pwm_t *self)
{
    SM_ASSERT_PTR(self);
    if (!self->is_ready) return -RT_ERROR;
    return self->ops->disable(self);
}

rt_err_t hw_pwm_brake(hw_pwm_t *self)
{
    SM_ASSERT_PTR(self);
    if (!self->is_ready) return -RT_ERROR;
    return self->ops->brake(self);
}

rt_err_t hw_pwm_brake_release(hw_pwm_t *self)
{
    SM_ASSERT_PTR(self);
    if (!self->is_ready) return -RT_ERROR;
    return self->ops->brake_release(self);
}

