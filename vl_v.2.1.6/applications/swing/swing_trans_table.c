/**
 * @file    swing_trans_table.c
 * @brief   回转控制状态机转移表 + 函数表 + 配置
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

#include "swing_control.h"
#include "swing_actions.h"
#include "app_event.h"
#include "framework/state_engine/state_engine.h"
#include <stddef.h>

/* ============================================================
 * 函数表
 * ============================================================ */
const StateFuncTable_t g_swing_func_table[] = {
    {H_STATE_IDLE,          swing_act_enter_idle,           NULL},
    {H_STATE_UNLOCKING,     swing_act_enter_unlocking,      NULL},
    {H_STATE_LEFT_RUNNING,  swing_act_enter_left_running,   swing_act_exit_left_running},
    {H_STATE_RIGHT_RUNNING, swing_act_enter_right_running,  swing_act_exit_right_running},
    {H_STATE_LOCKING,       swing_act_enter_locking,        NULL},
    {H_STATE_FAULT,         swing_act_enter_fault,          NULL},
};
const uint16_t g_swing_func_table_size = sizeof(g_swing_func_table) / sizeof(g_swing_func_table[0]);

/* ============================================================
 * 跳转表
 * ============================================================ */
const StateJumpTable_t g_swing_jump_table[] = {
    /* ===== IDLE 状态 ===== */
    {H_STATE_IDLE,         SWING_EVT_LEFT_DOWN,            H_STATE_UNLOCKING},
    {H_STATE_IDLE,         SWING_EVT_RIGHT_DOWN,           H_STATE_UNLOCKING},
    {H_STATE_IDLE,         SWING_EVT_ENCODER_FAULT,        H_STATE_FAULT},

    /* ===== UNLOCKING 状态 ===== */
    {H_STATE_UNLOCKING,    SWING_EVT_BRAKE_UNLOCKED_LEFT,  H_STATE_LEFT_RUNNING},
    {H_STATE_UNLOCKING,    SWING_EVT_BRAKE_UNLOCKED_RIGHT, H_STATE_RIGHT_RUNNING},
    {H_STATE_UNLOCKING,    SWING_EVT_BRAKE_FAULT,          H_STATE_FAULT},
    {H_STATE_UNLOCKING,    SWING_EVT_LEFT_UP,              H_STATE_LOCKING},
    {H_STATE_UNLOCKING,    SWING_EVT_RIGHT_UP,             H_STATE_LOCKING},
    {H_STATE_UNLOCKING,    EVT_TICK_50MS,                 H_STATE_UNLOCKING},

    /* ===== LEFT_RUNNING 状态 ===== */
    {H_STATE_LEFT_RUNNING, SWING_EVT_LEFT_UP,              H_STATE_LOCKING},
    {H_STATE_LEFT_RUNNING, SWING_EVT_RIGHT_DOWN,           H_STATE_LOCKING},   /* 互锁 */
    {H_STATE_LEFT_RUNNING, SWING_EVT_ZERO_REACHED,         H_STATE_LOCKING},
    {H_STATE_LEFT_RUNNING, SWING_EVT_LIMIT_LEFT_REACHED,   H_STATE_LOCKING},
    {H_STATE_LEFT_RUNNING, SWING_EVT_ENCODER_FAULT,        H_STATE_FAULT},
    {H_STATE_LEFT_RUNNING, EVT_TICK_50MS,                 H_STATE_LEFT_RUNNING},

    /* ===== RIGHT_RUNNING 状态 ===== */
    {H_STATE_RIGHT_RUNNING, SWING_EVT_RIGHT_UP,            H_STATE_LOCKING},
    {H_STATE_RIGHT_RUNNING, SWING_EVT_LEFT_DOWN,           H_STATE_LOCKING},   /* 互锁 */
    {H_STATE_RIGHT_RUNNING, SWING_EVT_ZERO_REACHED,        H_STATE_LOCKING},
    {H_STATE_RIGHT_RUNNING, SWING_EVT_LIMIT_RIGHT_REACHED, H_STATE_LOCKING},
    {H_STATE_RIGHT_RUNNING, SWING_EVT_ENCODER_FAULT,       H_STATE_FAULT},
    {H_STATE_RIGHT_RUNNING, EVT_TICK_50MS,                H_STATE_RIGHT_RUNNING},

    /* ===== LOCKING 状态 ===== */
    {H_STATE_LOCKING,       SWING_EVT_BRAKE_LOCKED,        H_STATE_IDLE},
    {H_STATE_LOCKING,       SWING_EVT_BRAKE_FAULT,         H_STATE_FAULT},
    {H_STATE_LOCKING,       EVT_TICK_50MS,                H_STATE_LOCKING},

    /* ===== FAULT 状态 =====
     *
     * 【2026-11-10 Q30-a 裁定：报编码器故障但**不影响手动模式**】
     *
     * 问题：原 FAULT 态只能靠 ENCODER_FAULT_CLEAR 回 IDLE，
     *       而该事件在全工程无投递点 ⇒ 一旦进 FAULT 就永久卡死，
     *       手动也动不了。这与"报故障但不影响手动"的裁定冲突。
     *
     * 解决：在 FAULT 态**保留手动方向跳转**。
     *   编码器故障时角度不可信，但用户仍可手动把卸筒转回来。
     *   安全性由两点保证：
     *     ① 手动路径不依赖角度（不判过零、不判限位是否到达，
     *        仅 forbidden_dirs 拦截，见 swing_actions.c）
     *     ② 机械限位兜底 + 堵转保护（motor_stall 已接入 DMA ISR）
     *
     * ⚠ 序列动作（SWING_EVT_SEQ_*）在 FAULT 态**不开放**：
     *   序列依赖角度判定，角度不可信时不应执行。
     *   前检查 seq_check_precondition() 里的 encoder_fault 判定仍有效，
     *   所以序列本就会被拒。
     *
     * ⚠ 进入本状态时 swing_act_enter_fault() 会执行 motor_stop()，
     *   但这是"进入瞬间停一次"，不影响后续手动重新启动。
     */
    {H_STATE_FAULT,         SWING_EVT_ENCODER_FAULT_CLEAR, H_STATE_IDLE},
    {H_STATE_FAULT,         SWING_EVT_BRAKE_FAULT,         H_STATE_FAULT},
    {H_STATE_FAULT,         EVT_TICK_50MS,                H_STATE_FAULT},

    /* 手动方向：允许脱离故障（角度不可信也能手动操作） */
    {H_STATE_FAULT,         SWING_EVT_LEFT_DOWN,           H_STATE_UNLOCKING},
    {H_STATE_FAULT,         SWING_EVT_RIGHT_DOWN,          H_STATE_UNLOCKING},
};
const uint16_t g_swing_jump_table_size = sizeof(g_swing_jump_table) / sizeof(g_swing_jump_table[0]);

/* ============================================================
 * 配置
 * ============================================================ */
const StateMachineConfig_t g_swing_config = {
    .jump_table      = g_swing_jump_table,
    .jump_table_size = sizeof(g_swing_jump_table) / sizeof(g_swing_jump_table[0]),
    .func_table      = g_swing_func_table,
    .func_table_size = sizeof(g_swing_func_table) / sizeof(g_swing_func_table[0]),
    .init_state      = H_STATE_IDLE,
    .max_state       = H_STATE_MAX,
    .name            = "SwingSM"
};
