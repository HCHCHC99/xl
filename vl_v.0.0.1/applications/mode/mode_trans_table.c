/**
 * @file    mode_trans_table.c
 * @brief   模式控制状态机转移表 + 函数表 + 配置
 */

/* ============================================================
 * 【引擎非周期调度 / 用户 2026-11-14 19:08 明确】
 *   本表只在「上游判定匹配到事件、并经 control_object_process()
 *   同步投递」时被查询一次；引擎不跑 tick、不轮询。
 *   50ms 周期逻辑走 app_main.c:131-155 的显式 *_act_tick_check()，
 *   与本表无关。
 *
 * 【自转移行 = 显式忽略 / 用户 18:45 裁定 D2 为设计选择】
 *   表内 target == cur_state 的行（含 EVT_TICK_50MS）表示
 *   「状态未改变，状态机不需要更新」，引擎短路不执行 enter/exit。
 *   这些行不会被周期事件驱动，保留仅为声明意图，请勿当作
 *   「每拍会执行一次」的回调入口。
 * ============================================================ */

#include "mode_control.h"
#include "mode_actions.h"
#include "app_event.h"
#include "framework/state_engine/state_engine.h"
#include <stddef.h>

/* ============================================================
 * 函数表（Enter / Exit 回调）
 * ============================================================ */
const StateFuncTable_t g_mode_func_table[] = {
    {
        .state = MODE_STATE_INIT,
        .enter = mode_enter_init,
        .exit  = mode_exit_init
    },
    {
        .state = MODE_STATE_FACTORY,
        .enter = mode_enter_factory,
        .exit  = mode_exit_factory
    },
    {
        .state = MODE_STATE_TEST,
        .enter = mode_enter_test,
        .exit  = mode_exit_test
    },
    {
        .state = MODE_STATE_WORK,
        .enter = mode_enter_work,
        .exit  = mode_exit_work
    },
};
const uint16_t g_mode_func_table_size = sizeof(g_mode_func_table) / sizeof(g_mode_func_table[0]);

/* ============================================================
 * 跳转表
 * ============================================================ */
const StateJumpTable_t g_mode_jump_table[] = {
    /* ===== INIT 状态 ===== */
    /* 初始化完成后，根据配置进入默认模式（由初始事件触发） */
    {MODE_STATE_INIT, EVT_TICK_50MS, MODE_STATE_INIT},

    /* 【P0 修正 2026-10-27】补 INIT 的三条出口。
     * 原表只有 INIT 自转移，没有出边 ⇒ mode_init() 调 mode_control_set_mode()
     * 投出的 MODE_EVT_ENTER_* 落到 INIT 状态时无迁移匹配，
     * StateMachine_SendEvent() 静默 return（state_engine.c:104-106），
     * mode_enter_factory/test/work 一次都没执行过 ⇒ sys_set_mode() 从未被调用
     * ⇒ g_current_mode 恒等于 sys_mode.c:13 的初值 WORK。
     * 后果：三模式权限形同不存在，flash 里的 default_mode 配置完全不生效。 */
    {MODE_STATE_INIT, MODE_EVT_ENTER_FACTORY, MODE_STATE_FACTORY},
    {MODE_STATE_INIT, MODE_EVT_ENTER_TEST,    MODE_STATE_TEST},
    {MODE_STATE_INIT, MODE_EVT_ENTER_WORK,    MODE_STATE_WORK},
    /* 注意：INIT → 默认模式 的转移在 main 初始化时通过发送事件实现 */

    /* ===== FACTORY 状态 ===== */
    {MODE_STATE_FACTORY, MODE_EVT_ENTER_FACTORY, MODE_STATE_FACTORY},  /* 自转移（更新统计） */
    {MODE_STATE_FACTORY, MODE_EVT_ENTER_TEST,    MODE_STATE_TEST},
    {MODE_STATE_FACTORY, MODE_EVT_ENTER_WORK,    MODE_STATE_WORK},
    {MODE_STATE_FACTORY, EVT_TICK_50MS,          MODE_STATE_FACTORY},

    /* ===== TEST 状态 ===== */
    {MODE_STATE_TEST, MODE_EVT_ENTER_FACTORY, MODE_STATE_FACTORY},
    {MODE_STATE_TEST, MODE_EVT_ENTER_TEST,    MODE_STATE_TEST},
    {MODE_STATE_TEST, MODE_EVT_ENTER_WORK,    MODE_STATE_WORK},
    {MODE_STATE_TEST, MODE_EVT_TEST_TIMEOUT,  MODE_STATE_WORK},        /* 超时退出到工作模式 */
    {MODE_STATE_TEST, EVT_TICK_50MS,          MODE_STATE_TEST},        /* Tick 检查超时 */

    /* ===== WORK 状态 ===== */
    {MODE_STATE_WORK, MODE_EVT_ENTER_FACTORY, MODE_STATE_FACTORY},
    {MODE_STATE_WORK, MODE_EVT_ENTER_TEST,    MODE_STATE_TEST},
    {MODE_STATE_WORK, MODE_EVT_ENTER_WORK,    MODE_STATE_WORK},
    {MODE_STATE_WORK, EVT_TICK_50MS,          MODE_STATE_WORK},
};
const uint16_t g_mode_jump_table_size = sizeof(g_mode_jump_table) / sizeof(g_mode_jump_table[0]);

/* ============================================================
 * 配置
 * ============================================================ */
const StateMachineConfig_t g_mode_config = {
    .jump_table      = g_mode_jump_table,
    .jump_table_size = sizeof(g_mode_jump_table) / sizeof(g_mode_jump_table[0]),
    .func_table      = g_mode_func_table,
    .func_table_size = sizeof(g_mode_func_table) / sizeof(g_mode_func_table[0]),
    .init_state      = MODE_STATE_INIT,
    .max_state       = MODE_STATE_MAX,
    .name            = "ModeSM"
};
