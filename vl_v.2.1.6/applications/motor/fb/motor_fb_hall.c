/**
 * @file    motor_fb_hall.c
 * @brief   Hall 反馈实现（转速估计）
 */

#include "motor_fb_hall.h"
#include "../../bsp/bsp_hall.h"
#include "framework/state_engine/sm_assert.h"
#include <string.h>
#include <rtthread.h>

typedef struct {
    uint32_t last_count;
    uint32_t last_tick;
    int32_t  speed_rpm_x10;     /* 速度（0.1 RPM） */
} hall_fb_data_t;

static rt_err_t hall_fb_init(motor_t *self)
{
    SM_ASSERT_PTR(self);
    hall_fb_data_t *data = rt_malloc(sizeof(hall_fb_data_t));
    if (data == RT_NULL) return -RT_ENOMEM;
    memset(data, 0, sizeof(hall_fb_data_t));
    self->fb_data = data;
    return RT_EOK;
}

static void hall_fb_update(motor_t *self)
{
    hall_fb_data_t *data = (hall_fb_data_t*)self->fb_data;
    SM_ASSERT_PTR(data);

    uint32_t now_count = hw_hall_get_count(&g_swing_hall);
    uint32_t now_tick = rt_tick_get();

    uint32_t count_diff = now_count - data->last_count;
    uint32_t tick_diff = now_tick - data->last_tick;

    if (tick_diff > 0 && count_diff > 0) {
        /* 转速计算：
         * 每个电周期有 6 个 Hall 状态变化
         * 电频率 = count_diff / (6 × tick_diff)
         * 机械转速（RPM） = 电频率 × 60 / pole_pairs
         */
        uint32_t pole_pairs = self->pole_pairs;
        if (pole_pairs == 0) pole_pairs = 1;

        /* speed_rpm = (count_diff / (6 × tick_diff)) × 60 / pole_pairs × RT_TICK_PER_SECOND */
        int32_t speed = (int32_t)(count_diff * 60 * RT_TICK_PER_SECOND)
                        / (6 * tick_diff * pole_pairs);

        data->speed_rpm_x10 = speed * 10;
        self->current_speed = data->speed_rpm_x10;

        data->last_count = now_count;
        data->last_tick = now_tick;
    }
}

static int32_t hall_fb_get_speed(motor_t *self)
{
    hall_fb_data_t *data = (hall_fb_data_t*)self->fb_data;
    if (data == NULL) return 0;
    return data->speed_rpm_x10;
}

static void hall_fb_deinit(motor_t *self)
{
    SM_ASSERT_PTR(self);
    if (self->fb_data) {
        rt_free(self->fb_data);
        self->fb_data = NULL;
    }
}

const motor_fb_ops_t motor_fb_hall_ops = {
    .init         = hall_fb_init,
    .update       = hall_fb_update,
    .get_position = NULL,
    .get_speed    = hall_fb_get_speed,
    .get_current  = NULL,
    .get_fault    = NULL,
    .deinit       = hall_fb_deinit,
};
