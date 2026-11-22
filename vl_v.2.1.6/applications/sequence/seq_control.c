/**
#include <lift/lift_control.h>
 * @file    seq_control.c
 * @brief   序列控制对象实现
 */

#include "seq_control.h"
#include "seq_config.h"
#include "seq_actions.h"
#include "../mode/sys_mode.h"
#include "framework/state_engine/sm_assert.h"
#include "app_event.h"
#include "framework/state_engine/state_engine.h"
#include "../swing/swing_control.h"
#include "../swing/swing_angle.h"
#include "../lift/lift_angle.h"
#include "../lift/lift_control.h"
#include <string.h>
#include <rtthread.h>

/* ============================================================
 * 全局对象实例
 * ============================================================ */
seq_control_t g_seq_obj;

/* ============================================================
 * 序列状态枚举（用于控制对象内部状态机）
 * ============================================================ */
enum {
    SEQ_OBJ_STATE_IDLE = 0,
    SEQ_OBJ_STATE_ACTIVE,
    SEQ_OBJ_STATE_MAX
};

/* ============================================================
 * 前置条件检查
 * ============================================================ */
static uint8_t seq_check_precondition(seq_ctrl_context_t *sctx)
{
    seq_ctrl_context_t *ctx = sctx;

    /* 1. 启车状态 */
    if (!ctx->vehicle_started) {
        rt_kprintf("[Seq] Precondition fail: not started\n");
        return 0;
    }

    /* 2. 模式允许 */
    if (!sys_mode_allows_sequence()) {
        rt_kprintf("[Seq] Precondition fail: mode\n");
        return 0;
    }

    /* 3. 举升无故障 */
    if (g_lift_obj.ctx.sensor_fault) {
        rt_kprintf("[Seq] Precondition fail: sensor fault\n");
        return 0;
    }
    if (g_lift_obj.ctx.valve_fault_up || g_lift_obj.ctx.valve_fault_down) {
        rt_kprintf("[Seq] Precondition fail: valve fault\n");
        return 0;
    }

    /* 4. 回转无故障 */
    if (g_swing_obj.ctx.encoder_fault) {
        rt_kprintf("[Seq] Precondition fail: encoder fault\n");
        return 0;
    }

    /* 5. 无手动输入 */
    if (g_swing_obj.ctx.left_sw_pressed ||
        g_swing_obj.ctx.right_sw_pressed) {
        return 0;
    }
    if (g_lift_obj.ctx.up_sw_pressed || g_lift_obj.ctx.down_sw_pressed) {
        return 0;
    }

    return 1;
}

/* ============================================================
 * 中断检查
 * ============================================================ */
static uint8_t seq_check_interrupt(seq_ctrl_context_t *sctx)
{
    /* 1. 手动按键 */
    if (g_swing_obj.ctx.left_sw_pressed ||
        g_swing_obj.ctx.right_sw_pressed) {
        return 1;
    }
    if (g_lift_obj.ctx.up_sw_pressed || g_lift_obj.ctx.down_sw_pressed) {
        return 1;
    }

    /* 2. 故障 */
    if (g_lift_obj.ctx.sensor_fault ||
        g_lift_obj.ctx.valve_fault_up ||
        g_lift_obj.ctx.valve_fault_down) {
        return 1;
    }

    /* 3. 模式变更 */
    if (!sys_mode_allows_sequence()) {
        return 1;
    }

    return 0;
}

/* ============================================================
 * 启动序列
 * ============================================================ */
static void seq_start_sequence(seq_type_t type)
{
    seq_ctrl_context_t *sctx = &g_seq_obj.ctx;

    /* 1. 前置条件检查 */
    if (!seq_check_precondition(sctx)) {
        sctx->reject_count++;
        return;
    }

    /* 2. 查找配置 */
    const seq_config_t *config = seq_config_find(type);
    if (config == NULL) {
        rt_kprintf("[Seq] Config not found for type %d\n", type);
        return;
    }

    /* 3. 初始化引擎 */
    seq_engine_init(&sctx->engine, config, type,
                    &g_swing_obj, &g_lift_obj,
                    &g_swing_angle, &g_lift_angle);

    /* 4. 启动 */
    if (seq_engine_start(&sctx->engine) == 0) {
        sctx->request_count++;
        /* Q-F3 路 A：改走专用投递接口（铁律 2：状态变更经聚合根） */
        control_object_process(&g_seq_obj.base, SEQ_OBJ_EVT_START);
    }
}

/* ============================================================
 * 状态机配置
 * ============================================================ */
static void seq_obj_enter_idle(StateContext_t ctx)
{
    (void)ctx;
}

static void seq_obj_enter_active(StateContext_t ctx)
{
    (void)ctx;
}

static const StateFuncTable_t g_seq_obj_func_table[] = {
    {SEQ_OBJ_STATE_IDLE,   seq_obj_enter_idle,   NULL},
    {SEQ_OBJ_STATE_ACTIVE, seq_obj_enter_active, NULL},
};

static const StateJumpTable_t g_seq_obj_jump_table[] = {
    /* 【Q-F3 路 A / 2026-11-14 E1】对象级状态机改为真实事件驱动。
     * 只经 control_object_process(&g_seq_obj.base, evt) 同步投递，
     * 不周期调度、不走 post_event（引擎只在匹配到事件时触发一次）。
     * 真正的步骤编排在 sctx->engine（seq_engine 5 态），属**编排层
     * 内部状态机**，不在本表（StateFuncTable_t 无 tick/步骤回调）。 */
    {SEQ_OBJ_STATE_IDLE,   SEQ_OBJ_EVT_START,  SEQ_OBJ_STATE_ACTIVE},
    {SEQ_OBJ_STATE_ACTIVE, SEQ_OBJ_EVT_FINISH, SEQ_OBJ_STATE_IDLE},
    {SEQ_OBJ_STATE_ACTIVE, SEQ_OBJ_EVT_ABORT,  SEQ_OBJ_STATE_IDLE},
};

static const StateMachineConfig_t g_seq_obj_config = {
    .jump_table      = g_seq_obj_jump_table,
    .jump_table_size = sizeof(g_seq_obj_jump_table) / sizeof(g_seq_obj_jump_table[0]),
    .func_table      = g_seq_obj_func_table,
    .func_table_size = sizeof(g_seq_obj_func_table) / sizeof(g_seq_obj_func_table[0]),
    .init_state      = SEQ_OBJ_STATE_IDLE,
    .max_state       = SEQ_OBJ_STATE_MAX,
    .name            = "SeqSM"
};

/* ============================================================
 * 初始化并注册
 * ============================================================ */
int seq_control_register(ctrl_manager_t *mgr)
{
    rt_err_t ret;

    SM_ASSERT_PTR(mgr);

    memset(&g_seq_obj, 0, sizeof(seq_control_t));

    g_seq_obj.base.sm = &g_seq_obj.sm_instance;

    ctrl_obj_runtime_cfg_t runtime = {
        .use_thread  = 0,
        .thread_prio = 0,
        .stack_size  = 0,
        .mq_size     = 0,
    };

    ret = control_object_init(&g_seq_obj.base,
                              "SeqControl",
                              CTRL_OBJ_TYPE_SEQ,
                              &g_seq_obj_config,
                              &g_seq_obj.ctx,
                              &runtime);
    if (ret != RT_EOK) {
        rt_kprintf("[Seq] control_object_init failed (%d)\n", ret);
        return -1;
    }

    ret = ctrl_manager_register(mgr, &g_seq_obj.base);
    if (ret != 0) {
        rt_kprintf("[Seq] register failed (%d)\n", ret);
        return -1;
    }

    rt_kprintf("[Seq] Control registered.\n");
    return 0;
}

/* ============================================================
 * 一键按键更新（上升沿触发）
 * ============================================================ */
void seq_control_update_onekey(uint8_t middle, uint8_t rear, uint8_t home)
{
    seq_ctrl_context_t *sctx = &g_seq_obj.ctx;

    /* 上升沿检测 */
    uint8_t middle_rising = middle && !sctx->prev_middle;
    uint8_t rear_rising   = rear && !sctx->prev_rear;
    uint8_t home_rising   = home && !sctx->prev_home;

    sctx->prev_middle = middle;
    sctx->prev_rear = rear;
    sctx->prev_home = home;

    /* 检查是否有其他序列正在运行 */
    if (seq_engine_is_active(&sctx->engine)) {
        /* 中断当前序列 */
        if (middle_rising || rear_rising || home_rising) {
            rt_kprintf("[Seq] Interrupt current sequence\n");
            seq_engine_abort(&sctx->engine);
            control_object_process(&g_seq_obj.base, SEQ_OBJ_EVT_ABORT);
        }
        return;
    }

    /* 启动新序列 */
    if (middle_rising) {
        rt_kprintf("[Seq] Trigger RIGHT_MIDDLE\n");
        seq_start_sequence(SEQ_TYPE_RIGHT_MIDDLE);
    } else if (rear_rising) {
        rt_kprintf("[Seq] Trigger RIGHT_BEHIND\n");
        seq_start_sequence(SEQ_TYPE_RIGHT_BEHIND);
    } else if (home_rising) {
        rt_kprintf("[Seq] Trigger HOME\n");
        seq_start_sequence(SEQ_TYPE_HOME);
    }
}

/* ============================================================
 * 启车状态设置
 * ============================================================ */
void seq_control_set_vehicle_started(uint8_t started)
{
    g_seq_obj.ctx.vehicle_started = started;
}

/* ============================================================
 * 周期更新（由主循环或控制对象广播调用）
 * ============================================================ */
void seq_control_update(void)
{
    seq_ctrl_context_t *sctx = &g_seq_obj.ctx;

    if (!seq_engine_is_active(&sctx->engine)) {
        return;
    }

    /* 中断检查 */
    if (seq_check_interrupt(sctx)) {
        rt_kprintf("[Seq] Interrupted\n");
        seq_engine_abort(&sctx->engine);
        control_object_process(&g_seq_obj.base, SEQ_OBJ_EVT_ABORT);
        return;
    }

    /* 引擎更新 */
    seq_engine_update(&sctx->engine);

    /* 检查是否完成/中止 */
    seq_state_t state = seq_engine_get_state(&sctx->engine);
    if (state == SEQ_STATE_DONE || state == SEQ_STATE_ABORTED) {
        control_object_process(&g_seq_obj.base, SEQ_OBJ_EVT_FINISH);
    }
}

/* ============================================================
 * 查询接口
 * ============================================================ */
seq_ctrl_context_t* seq_control_get_context(void)
{
    return &g_seq_obj.ctx;
}

/* ============================================================
 * FinSH 调试命令
 * ============================================================ */
#ifdef RT_USING_FINSH
#include <finsh.h>

static void cmd_seq_status(int argc, char **argv)
{
    seq_ctrl_context_t *sctx = &g_seq_obj.ctx;
    rt_kprintf("Sequence state: %d\n", seq_engine_get_state(&sctx->engine));
    rt_kprintf("Current step:   %d\n", seq_engine_get_step(&sctx->engine));
    rt_kprintf("Requests: %u, Rejects: %u\n",
               sctx->request_count, sctx->reject_count);
}
MSH_CMD_EXPORT_ALIAS(cmd_seq_status, seq_status, show sequence status);

static void cmd_seq_abort(int argc, char **argv)
{
    seq_engine_abort(&g_seq_obj.ctx.engine);
    rt_kprintf("Sequence aborted\n");
}
MSH_CMD_EXPORT_ALIAS(cmd_seq_abort, seq_abort, abort current sequence);
#endif
