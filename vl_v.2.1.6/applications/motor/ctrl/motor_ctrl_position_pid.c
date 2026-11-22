/**
 * @file    motor_ctrl_position_pid.c
 * @brief   位置环 PID 实现
 */

#include "motor_ctrl_position_pid.h"
#include "../motor_drv.h"
#include "../motor.h"
//#include "../soft_ramp.h"
//#include "../motor_config.h"
#include "framework/state_engine/sm_assert.h"
#include <string.h>
#include <rtthread.h>

/* ============================================================
 * 控制私有数据
 * ============================================================ */
typedef struct {
    const motor_pos_pid_cfg_t *cfg;
    int32_t integral;
    int32_t prev_error;
    int32_t target;
} pos_pid_data_t;

/* ============================================================
 * 控制层接口实现
 * ============================================================ */
static rt_err_t pos_pid_init(motor_t *self)
{
    SM_ASSERT_PTR(self);
    SM_ASSERT_PTR(self->config);

    pos_pid_data_t *data = rt_malloc(sizeof(pos_pid_data_t));
    if (data == RT_NULL) return -RT_ENOMEM;

    memset(data, 0, sizeof(pos_pid_data_t));
    data->cfg = (const motor_pos_pid_cfg_t*)self->config->ctrl_cfg;

    self->ctrl_data = data;

    rt_kprintf("[PosPID] Initialized (kp=%.2f, ki=%.2f, kd=%.2f)\n",
               data->cfg->kp, data->cfg->ki, data->cfg->kd);

    return RT_EOK;
}

static void pos_pid_reset(motor_t *self)
{
    pos_pid_data_t *data = (pos_pid_data_t*)self->ctrl_data;
    if (data) {
        data->integral = 0;
        data->prev_error = 0;
    }
}

static void pos_pid_update(motor_t *self, uint32_t dt_ms)
{
    pos_pid_data_t *data = (pos_pid_data_t*)self->ctrl_data;
    SM_ASSERT_PTR(data);
    SM_ASSERT_PTR(data->cfg);

    /* 1. 计算误差 */
    int32_t current = self->current_position;
    int32_t error = data->target - current;

    /* 2. 积分项 */
    data->integral += error;
    if (data->integral > data->cfg->integral_limit) {
        data->integral = data->cfg->integral_limit;
    } else if (data->integral < -data->cfg->integral_limit) {
        data->integral = -data->cfg->integral_limit;
    }

    /* 3. 微分项 */
    int32_t derivative = error - data->prev_error;
    data->prev_error = error;

    /* 4. PID 运算 */
    float output = data->cfg->kp * error
                 + data->cfg->ki * data->integral
                 + data->cfg->kd * derivative;

    int32_t output_i = (int32_t)output;

    /* 5. 限幅 */
    if (output_i > data->cfg->max_output) output_i = data->cfg->max_output;
    if (output_i < -data->cfg->max_output) output_i = -data->cfg->max_output;

    /* 6. 软启动/停止处理 */
    if (self->ramp.is_running) {
        int32_t ramp_out = soft_ramp_update(&self->ramp, dt_ms);
        /* 在斜坡阶段，输出被限制 */
        if (ramp_out >= 0 && output_i > ramp_out) output_i = ramp_out;
        if (ramp_out < 0 && output_i < ramp_out) output_i = ramp_out;
    }

    /* 7. 输出到驱动层 */
    if (self->drv_ops && self->drv_ops->set_output) {
        self->drv_ops->set_output(self, output_i);
    }
}

static void pos_pid_set_target(motor_t *self, int32_t target)
{
    pos_pid_data_t *data = (pos_pid_data_t*)self->ctrl_data;
    SM_ASSERT_PTR(data);

    data->target = target;
    self->target_position = target;

    /* 触发软启动 */
    if (self->ramp.cfg && self->ramp.cfg->start_enabled) {
        soft_ramp_start(&self->ramp, 0, data->cfg->max_output);
        self->state = MOTOR_STATE_RAMPING;
    } else {
        self->state = MOTOR_STATE_RUNNING;
    }
}

static void pos_pid_stop(motor_t *self)
{
    pos_pid_data_t *data = (pos_pid_data_t*)self->ctrl_data;
    SM_ASSERT_PTR(data);

    /* 触发软停止 */
    if (self->ramp.cfg && self->ramp.cfg->stop_enabled) {
        soft_ramp_stop(&self->ramp, data->cfg->max_output);
        self->state = MOTOR_STATE_RAMPING;
    } else {
        if (self->drv_ops && self->drv_ops->set_output) {
            self->drv_ops->set_output(self, 0);
        }
        self->state = MOTOR_STATE_IDLE;
    }

    data->target = self->current_position;
    data->integral = 0;
}

static void pos_pid_deinit(motor_t *self)
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
const motor_ctrl_ops_t motor_ctrl_position_pid_ops = {
    .init       = pos_pid_init,
    .reset      = pos_pid_reset,
    .update     = pos_pid_update,
    .set_target = pos_pid_set_target,
    .stop       = pos_pid_stop,
    .deinit     = pos_pid_deinit,
};
