/**
 * @file    motor_ctrl_openloop.c
 * @brief   开环控制实现
 */

#include "motor_ctrl_openloop.h"
//#include "../motor_drv.h"
#include "../soft_ramp.h"
#include "framework/state_engine/sm_assert.h"
#include <string.h>
#include <rtthread.h>

/* ============================================================
 * 控制私有数据
 * ============================================================ */
typedef struct {
    int32_t target_output;      /* 目标输出（PWM 占空比） */
} openloop_data_t;

/* ============================================================
 * 控制层接口实现
 * ============================================================ */
static rt_err_t openloop_init(motor_t *self)
{
    SM_ASSERT_PTR(self);

    openloop_data_t *data = rt_malloc(sizeof(openloop_data_t));
    if (data == RT_NULL) return -RT_ENOMEM;

    memset(data, 0, sizeof(openloop_data_t));
    self->ctrl_data = data;

    return RT_EOK;
}

static void openloop_reset(motor_t *self)
{
    openloop_data_t *data = (openloop_data_t*)self->ctrl_data;
    if (data) data->target_output = 0;
}

static void openloop_update(motor_t *self, uint32_t dt_ms)
{
    openloop_data_t *data = (openloop_data_t*)self->ctrl_data;
    SM_ASSERT_PTR(data);

    /* 计算输出（含软启动/停止） */
    int32_t output;

    if (self->ramp.is_running) {
        output = soft_ramp_update(&self->ramp, dt_ms);
    } else {
        output = data->target_output;
    }

    /* 输出到驱动层 */
    if (self->drv_ops && self->drv_ops->set_output) {
        self->drv_ops->set_output(self, output);
    }
}

static void openloop_set_target(motor_t *self, int32_t target)
{
    openloop_data_t *data = (openloop_data_t*)self->ctrl_data;
    SM_ASSERT_PTR(data);

    data->target_output = target;
    self->target_output = target;

    /* 触发软启动 */
    if (self->ramp.cfg && self->ramp.cfg->start_enabled) {
        soft_ramp_start(&self->ramp, 0, target);
        self->state = MOTOR_STATE_RAMPING;
    } else {
        self->state = MOTOR_STATE_RUNNING;
    }
}

static void openloop_stop(motor_t *self)
{
    openloop_data_t *data = (openloop_data_t*)self->ctrl_data;
    SM_ASSERT_PTR(data);

    /* 触发软停止 */
    if (self->ramp.cfg && self->ramp.cfg->stop_enabled) {
        soft_ramp_stop(&self->ramp, data->target_output);
        data->target_output = 0;
        self->state = MOTOR_STATE_RAMPING;
    } else {
        data->target_output = 0;
        if (self->drv_ops && self->drv_ops->set_output) {
            self->drv_ops->set_output(self, 0);
        }
        self->state = MOTOR_STATE_IDLE;
    }
}

static void openloop_deinit(motor_t *self)
{
    SM_ASSERT_PTR(self);
    if (self->ctrl_data) {
        rt_free(self->ctrl_data);
        self->ctrl_data = NULL;
    }
}

/* ============================================================
 * 接口表
 * ============================================================ */
const motor_ctrl_ops_t motor_ctrl_openloop_ops = {
    .init       = openloop_init,
    .reset      = openloop_reset,
    .update     = openloop_update,
    .set_target = openloop_set_target,
    .stop       = openloop_stop,
    .deinit     = openloop_deinit,
};
