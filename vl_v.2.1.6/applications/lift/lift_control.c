/**
 * @file    vert_control.c
 * @brief   举升控制状态机初始化 + 定时器管理 + 外部状态更新
 */

#include "lift_control.h"

/* 【2026-11-10 彩标 P0】举升控制注册：失败=ERR，成功=OK */
#include "../log/log_color.h"
#include "lift_actions.h"
#include "lift_angle.h"
#include "../mode/sys_mode.h"
#include "framework/state_engine/sm_assert.h"
#include "app_event.h"
#include "framework/state_engine/state_engine.h"
#include <string.h>
#include <rtthread.h>

/* ============================================================
 * 全局对象
 * ============================================================ */
lift_control_t g_lift_obj;

/* ============================================================
 * 5秒超时定时器（RT-Thread 软件定时器）
 * ============================================================ */
static rt_timer_t g_lift_timeout_timer;

static void lift_timeout_callback(void *param)
{
    post_event(LIFT_EVT_TIMEOUT_5S);
}

/* ============================================================
 * 外部引用
 * ============================================================ */
extern const StateMachineConfig_t g_lift_config;

/* ============================================================
 * 初始化并注册
 * ============================================================ */
int lift_control_register(ctrl_manager_t *mgr)
{
    rt_err_t ret;

    SM_ASSERT_PTR(mgr);

    /* 初始化上下文 */
    memset(&g_lift_obj.ctx, 0, sizeof(lift_context_t));
    g_lift_obj.ctx.solenoid_ptr = &g_solenoid;
    g_lift_obj.ctx.lift_ptr = &g_lift_angle;

    /* 绑定状态机指针 */
    g_lift_obj.base.sm = &g_lift_obj.sm_instance;

    /* 单线程配置 */
    ctrl_obj_runtime_cfg_t runtime = {
        .use_thread = 0,
    };

    /* 初始化控制对象 */
    ret = control_object_init(&g_lift_obj.base,
                              "LiftControl",
                              CTRL_OBJ_TYPE_LIFT,
                              &g_lift_config,
                              &g_lift_obj.ctx,
                              &runtime);
    if (ret != RT_EOK) {
        LOG_ERR("[Lift] control_object_init failed (%d)\n", ret);
        return -1;
    }

    /* 创建超时定时器 */
    g_lift_timeout_timer = rt_timer_create("p_tmo5s",
                                             lift_timeout_callback,
                                             RT_NULL,
                                             5000,
                                             RT_TIMER_FLAG_ONE_SHOT);
    SM_ASSERT_PTR(g_lift_timeout_timer);

    /* 注册到管理器 */
    ret = ctrl_manager_register(mgr, &g_lift_obj.base);
    if (ret != 0) {
        LOG_ERR("[Lift] register failed (%d)\n", ret);
        return -1;
    }

    LOG_OK("[Lift] Control registered.\n");
    return 0;
}


/* ============================================================
 * 公开 API：状态更新
 * ============================================================ */
void lift_control_update_switch(uint8_t up, uint8_t down)
{
    g_lift_obj.ctx.up_sw_pressed = up;
    g_lift_obj.ctx.down_sw_pressed = down;
}

void lift_control_update_valve_fault(uint8_t up_fault, uint8_t down_fault)
{
    uint8_t old_up_fault = g_lift_obj.ctx.valve_fault_up;
    uint8_t old_down_fault = g_lift_obj.ctx.valve_fault_down;

    g_lift_obj.ctx.valve_fault_up = up_fault;
    g_lift_obj.ctx.valve_fault_down = down_fault;

    /* 上升阀故障出现 → 投递事件 */
    if (!old_up_fault && up_fault) {
        post_event(LIFT_EVT_VALVE_FAULT);
    }
    /* 上升阀故障清除 → 投递事件 */
    if (old_up_fault && !up_fault && !g_lift_obj.ctx.valve_fault_down) {
        post_event(LIFT_EVT_VALVE_FAULT_CLEAR);
    }

    /* 下降阀故障出现 → 投递事件 */
    if (!old_down_fault && down_fault) {
        post_event(LIFT_EVT_VALVE_FAULT);
    }
    /* 下降阀故障清除 → 投递事件（仅当所有阀故障清除） */
    if (old_down_fault && !down_fault && !g_lift_obj.ctx.valve_fault_up) {
        post_event(LIFT_EVT_VALVE_FAULT_CLEAR);
    }
}

void lift_control_update_sensor_fault(uint8_t fault)
{
    uint8_t old_fault = g_lift_obj.ctx.sensor_fault;
    g_lift_obj.ctx.sensor_fault = fault;

    if (!old_fault && fault) {
        post_event(LIFT_EVT_SENSOR_FAULT);
    }
    if (old_fault && !fault) {
        post_event(LIFT_EVT_SENSOR_FAULT_CLEAR);
    }
}

void lift_control_update_valve_fb(uint8_t up_fb, uint8_t down_fb)
{
    g_lift_obj.ctx.valve_fb_up = up_fb;
    g_lift_obj.ctx.valve_fb_down = down_fb;
}

/* ============================================================
 * 公开 API：定时器管理
 * ============================================================ */
void lift_control_start_timer(void)
{
    rt_timer_stop(g_lift_timeout_timer);
    rt_timer_start(g_lift_timeout_timer);
}

void lift_control_stop_timer(void)
{
    rt_timer_stop(g_lift_timeout_timer);
}

/* ============================================================
 * 公开 API：获取状态
 * ============================================================ */
lift_state_t  lift_control_get_state(void)
{
    return (lift_state_t)StateMachine_GetState(&g_lift_obj.sm_instance);
}

lift_context_t* lift_control_get_context(void)
{
    return &g_lift_obj.ctx;
}

