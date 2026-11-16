/**
 * @file    swing_control.c
 * @brief   回转控制对象实现
 */

#include "swing_control.h"

/* 【2026-11-10 彩标 P2】回转控制注册：失败=ERR，成功=OK */
#include "../log/log_color.h"
#include "swing_actions.h"
#include "../motor/motor.h"
#include "../motor/motor_config.h"
#include "../mode/sys_mode.h"
#include "framework/state_engine/sm_assert.h"
#include "app_event.h"
#include <string.h>
#include <rtthread.h>

/* ============================================================
 * 全局对象实例
 * ============================================================ */
swing_control_t g_swing_obj;
extern motor_t g_swing_motor;

/* ============================================================
 * 方向极性（默认正逻辑，swing_init 时由 sys_config 覆盖）
 * ============================================================ */
uint8_t g_swing_dir_polarity = 0U;

/* ============================================================
 * 方向翻译：逻辑方向 → 电机转向
 * ------------------------------------------------------------
 * 这是全工程**唯一**的极性生效点。
 *
 * 正逻辑下：
 *   意图减小角度 → 左转 → MOTOR_DIR_CW
 *   意图增大角度 → 右转 → MOTOR_DIR_CCW
 * 负逻辑下：两者互换。
 *
 * 【为什么 in/out 都用"意图"而非"左/右"】
 *   序列按角度目标决策，只关心"要增大还是减小"。
 *   用意图表达后，极性翻转不再需要改序列代码。
 * ============================================================ */
uint8_t swing_dir_to_motor(uint8_t logical_dir)
{
    uint8_t want_increase;

    /* 入参防御：非 SWING_DIR_INCREASE 一律按"减小角度"处理。
     * 为什么不用断言：本条路径在中断/状态机里调用，
     * 断言失败会停机，比"方向按默认走"更危险。
     * 非法值只可能是代码 bug，交给 t_swing 诊断时暴露。 */
    want_increase = (logical_dir == SWING_DIR_INCREASE) ? 1U : 0U;

    /* 负逻辑：意图与实际转向取反 */
    if (g_swing_dir_polarity != 0U) {
        want_increase = (want_increase != 0U) ? 0U : 1U;
    }

    return (want_increase != 0U) ? MOTOR_DIR_CCW : MOTOR_DIR_CW;
}

/* ============================================================
 * 外部引用
 * ============================================================ */
extern const StateMachineConfig_t g_swing_config;

/* ============================================================
 * 初始化并注册
 * ============================================================ */
int swing_control_register(ctrl_manager_t *mgr)
{
    rt_err_t ret;

    SM_ASSERT_PTR(mgr);

    /* 1. 初始化上下文 */
    memset(&g_swing_obj.ctx, 0, sizeof(swing_context_t));
    g_swing_obj.ctx.motor_ptr = &g_swing_motor;
    g_swing_obj.ctx.angle_ptr = &g_swing_angle;

    /* 2. 绑定状态机指针 */
    g_swing_obj.base.sm = &g_swing_obj.sm_instance;

    /* 3. 单线程运行时配置 */
    ctrl_obj_runtime_cfg_t runtime = {
        .use_thread  = 0,
        .thread_prio = 0,
        .stack_size  = 0,
        .mq_size     = 0,
    };

    /* 4. 初始化控制对象 */
    ret = control_object_init(&g_swing_obj.base,
                              "SwingControl",
                              CTRL_OBJ_TYPE_SWING,
                              &g_swing_config,
                              &g_swing_obj.ctx,
                              &runtime);
    if (ret != RT_EOK) {
        LOG_ERR("[Swing] control_object_init failed (%d)\n", ret);
        return -1;
    }

    /* 5. 注册到管理器 */
    ret = ctrl_manager_register(mgr, &g_swing_obj.base);
    if (ret != 0) {
        LOG_ERR("[Swing] ctrl_manager_register failed (%d)\n", ret);
        return -1;
    }

    LOG_OK("[Swing] Control registered.\n");
    return 0;
}

/* ============================================================
 * 开关状态更新（由 CAN 解析层调用）
 * ============================================================ */
void swing_control_update_switch(uint8_t left, uint8_t right)
{
    swing_context_t *hctx = &g_swing_obj.ctx;

    /* 上升沿检测 */
    uint8_t left_rising  = (left  && !hctx->prev_left_sw)  ? 1U : 0U;
    uint8_t right_rising = (right && !hctx->prev_right_sw) ? 1U : 0U;

    /* 更新按键状态 */
    hctx->left_sw_pressed  = left;
    hctx->right_sw_pressed = right;
    hctx->prev_left_sw     = left;
    hctx->prev_right_sw    = right;

    /* 上升沿清除"零位后需重新触发"标志 */
    if (hctx->need_retrigger && ((left_rising != 0U) || (right_rising != 0U))) {
        hctx->need_retrigger = 0;
    }

    /* 方向登记：实际事件由 hdl_panel_switch 统一投递（经互斥量仲裁） */
    if (left_rising != 0U) {
        hctx->pending_dir = 1;
    }
    if (right_rising != 0U) {
        hctx->pending_dir = 2;
    }
}

/* ============================================================
 * 查询接口
 * ============================================================ */
swing_context_t* swing_control_get_context(void)
{
    return &g_swing_obj.ctx;
}

swing_state_t swing_control_get_state(void)
{
    return (swing_state_t)StateMachine_GetState(&g_swing_obj.sm_instance);
}
