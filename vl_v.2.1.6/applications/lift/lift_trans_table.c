/**
 * @file    lift_trans_table.c
 * @brief   举升控制状态机转移表 + 函数表 + 配置
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

#include "lift_control.h"
#include "lift_actions.h"
#include <stddef.h>
#include "app_event.h"
#include "framework/state_engine/state_engine.h"

/* ============================================================
 * 函数表（Enter / Exit 回调）
 * ============================================================ */
const StateFuncTable_t g_lift_func_table[] = {
    {P_STATE_IDLE,          lift_act_enter_idle,          NULL},
    {P_STATE_UP_RUNNING,    lift_act_valve_on_up,         lift_act_valve_off},
    {P_STATE_DOWN_RUNNING,  lift_act_valve_on_down,       lift_act_valve_off},
    {P_STATE_LIMIT_HOLDING, lift_act_enter_hold,          lift_act_exit_hold},
    {P_STATE_FAULT_VALVE,   lift_act_enter_fault_valve,   NULL},
    {P_STATE_FAULT_SENSOR,  lift_act_enter_fault_sensor,  NULL},
};

/* ============================================================
 * 跳转表
 * ============================================================ */
const StateJumpTable_t g_lift_jump_table[] = {
    /* ===== IDLE 状态 ===== */
    {P_STATE_IDLE,          LIFT_EVT_UP_DOWN,           P_STATE_UP_RUNNING},
    {P_STATE_IDLE,          LIFT_EVT_DOWN_DOWN,         P_STATE_DOWN_RUNNING},
    {P_STATE_IDLE,          LIFT_EVT_SEQ_UP,            P_STATE_UP_RUNNING},
    {P_STATE_IDLE,          LIFT_EVT_SEQ_DOWN,          P_STATE_DOWN_RUNNING},
    {P_STATE_IDLE,          EVT_TICK_50MS,               P_STATE_IDLE},

    /* IDLE：故障入口 + 限位忽略（原表完全缺失） */
    {P_STATE_IDLE, LIFT_EVT_VALVE_FAULT,        P_STATE_FAULT_VALVE},
    {P_STATE_IDLE, LIFT_EVT_SENSOR_FAULT,       P_STATE_FAULT_SENSOR},
    {P_STATE_IDLE, LIFT_EVT_TIMEOUT_5S,         P_STATE_IDLE},
    {P_STATE_IDLE, LIFT_EVT_LIMIT_REACHED_UPPER,P_STATE_IDLE},
    {P_STATE_IDLE, LIFT_EVT_LIMIT_REACHED_LOWER,P_STATE_IDLE},


    /* ===== UP_RUNNING 状态 ===== */
    {P_STATE_UP_RUNNING,    LIFT_EVT_UP_UP,             P_STATE_IDLE},
    {P_STATE_UP_RUNNING,    LIFT_EVT_DOWN_DOWN,         P_STATE_IDLE},
    {P_STATE_UP_RUNNING,    LIFT_EVT_SEQ_STOP,          P_STATE_IDLE},

    /* 【2026-11-10 Q29 裁定：手动模式只响应动作，限位保持功能先注释掉】
     *
     * 原行为：到限位 → LIMIT_HOLDING（保持 5s，期间可反向脱离）→ IDLE
     * 现行为：到限位 → 直接 IDLE
     *
     * ⚠ 关阀**不依赖** LIMIT_HOLDING 状态，而是靠 UP_RUNNING 的 exit 回调
     *   （本文件 :18 的 lift_act_valve_off）。state_engine 转移时先执行
     *   exit 再执行 enter，所以 UP_RUNNING → IDLE 这条路照常关阀。
     *   ⚠ 绝对不要删 lift_act_valve_off：删了阀不关，液压持续顶死、
     *     溢流阀泄压发热。
     *
     * 保留 P_STATE_LIMIT_HOLDING 枚举值与跳转表条目（下方 :66-73），
     * 以便功能恢复时只需把下面 4 行改回 LIMIT_HOLDING。
     * 保留枚举值也避免 max_state 变化导致历史日志状态号错位。 */
    {P_STATE_UP_RUNNING,    LIFT_EVT_LIMIT_REACHED,     P_STATE_IDLE},
    {P_STATE_UP_RUNNING,    EVT_TICK_50MS,               P_STATE_UP_RUNNING},

    {P_STATE_UP_RUNNING,   LIFT_EVT_LIMIT_REACHED_UPPER, P_STATE_IDLE},
    {P_STATE_UP_RUNNING,   LIFT_EVT_VALVE_FAULT,         P_STATE_FAULT_VALVE},
    {P_STATE_UP_RUNNING,   LIFT_EVT_SENSOR_FAULT,        P_STATE_FAULT_SENSOR},

    /* ===== DOWN_RUNNING 状态 ===== */
    {P_STATE_DOWN_RUNNING,  LIFT_EVT_DOWN_UP,           P_STATE_IDLE},
    {P_STATE_DOWN_RUNNING,  LIFT_EVT_UP_DOWN,           P_STATE_IDLE},
    {P_STATE_DOWN_RUNNING,  LIFT_EVT_SEQ_STOP,          P_STATE_IDLE},

    /* 同上：限位保持先注释，到限位直接回 IDLE（exit 回调照常关阀） */
    {P_STATE_DOWN_RUNNING,  LIFT_EVT_LIMIT_REACHED,     P_STATE_IDLE},
    {P_STATE_DOWN_RUNNING,  EVT_TICK_50MS,               P_STATE_DOWN_RUNNING},

    {P_STATE_DOWN_RUNNING, LIFT_EVT_LIMIT_REACHED_LOWER, P_STATE_IDLE},
    {P_STATE_DOWN_RUNNING, LIFT_EVT_VALVE_FAULT,         P_STATE_FAULT_VALVE},
    {P_STATE_DOWN_RUNNING, LIFT_EVT_SENSOR_FAULT,        P_STATE_FAULT_SENSOR},

    /* ===== LIMIT_HOLDING 状态（2026-11-10 起无入边，保留待恢复）=====
     * 无入边意味着此状态不可达，其跳转条目是死链。
     * 保留原因：① 恢复功能只需改上面 4 行目标状态
     *           ② 枚举值不变，max_state 与历史日志状态号保持不变 */
    {P_STATE_LIMIT_HOLDING, LIFT_EVT_TIMEOUT_5S,        P_STATE_IDLE},
    {P_STATE_LIMIT_HOLDING, EVT_TICK_50MS,               P_STATE_LIMIT_HOLDING},

    /* 保持态：允许反向脱离 + 故障 */
    {P_STATE_LIMIT_HOLDING, LIFT_EVT_UP_DOWN,     P_STATE_UP_RUNNING},
    {P_STATE_LIMIT_HOLDING, LIFT_EVT_DOWN_DOWN,   P_STATE_DOWN_RUNNING},
    {P_STATE_LIMIT_HOLDING, LIFT_EVT_VALVE_FAULT, P_STATE_FAULT_VALVE},
    {P_STATE_LIMIT_HOLDING, LIFT_EVT_SENSOR_FAULT,P_STATE_FAULT_SENSOR},

    /* ===== FAULT_VALVE 状态 ===== */
    {P_STATE_FAULT_VALVE,   LIFT_EVT_VALVE_FAULT_CLEAR, P_STATE_IDLE},

    /* 故障态补全 */
    {P_STATE_FAULT_VALVE,  LIFT_EVT_VALVE_FAULT,       P_STATE_FAULT_VALVE},
    {P_STATE_FAULT_VALVE,  LIFT_EVT_SENSOR_FAULT,      P_STATE_FAULT_SENSOR},
    {P_STATE_FAULT_VALVE,  EVT_TICK_50MS,               P_STATE_FAULT_VALVE},
    {P_STATE_FAULT_SENSOR, LIFT_EVT_VALVE_FAULT,       P_STATE_FAULT_VALVE},
    {P_STATE_FAULT_SENSOR, EVT_TICK_50MS,               P_STATE_FAULT_SENSOR},


    /* ===== FAULT_SENSOR 状态 ===== */
    {P_STATE_FAULT_SENSOR,  LIFT_EVT_UP_DOWN,           P_STATE_FAULT_SENSOR},
    {P_STATE_FAULT_SENSOR,  LIFT_EVT_DOWN_DOWN,         P_STATE_FAULT_SENSOR},
    {P_STATE_FAULT_SENSOR,  LIFT_EVT_SENSOR_FAULT_CLEAR,P_STATE_IDLE},
};

/* ============================================================
 * 配置
 * ============================================================ */
const StateMachineConfig_t g_lift_config = {
    .jump_table      = g_lift_jump_table,
    .jump_table_size = sizeof(g_lift_jump_table) / sizeof(g_lift_jump_table[0]),
    .func_table      = g_lift_func_table,
    .func_table_size = sizeof(g_lift_func_table) / sizeof(g_lift_func_table[0]),
    .init_state      = P_STATE_IDLE,
    .max_state       = P_STATE_MAX,
    .name            = "LiftSM"
};
