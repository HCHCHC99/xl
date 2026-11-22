/**
 * @file    vehicle_trans_table.c
 * @brief   整车四态单机：转移表 + 函数表 + 配置
 *
 * 【2026-11-14 P2】状态模型 OFF / ACC_ON / STARTED / FAULT
 *   OFF      --VEH_EVT_ACC_ON-->  ACC_ON      钥匙上电确认（门 1）
 *   任意态   --VEH_EVT_ACC_OFF--> OFF         钥匙下电确认（B-1 唯一清除途径）
 *   ACC_ON   --VEH_EVT_CONFIRM--> STARTED     Q-A 启车条件连续 500ms
 *   工作态   --VEH_EVT_FAULT-->   FAULT       Q-B 故障连续 2s
 *   FAULT    --VEH_EVT_ACC_OFF--> OFF         Q-B 清除（不自动退出）
 *
 * 【去抖不占状态】rpm_cond_ms / volt_cond_ms / fault_cond_ms 是运行属性（平面状态机），
 *   由 vehicle_poll() 累加；转移表每一行都是真实的已确认跳变。
 *   【V1.9】Q-A 改为两条件独立软件计时，原 start_cond_ms 单一计时器已作废。
 *   例外：VEH_EVT_FORCE_RESET 在 OFF 态是**自转移**（target==cur），
 *   state_engine.c:109 会短路、不执行 enter/exit —— 这是预期行为：
 *   调试命令在已处于 OFF 时不应改变统计计数。其余行均非自转移。
 *
 * 【同步投递】VEH_EVT_* 必须经 control_object_process() 同步驱动，
 *   不得改 post_event（0x9000 段在 app_main 无路由分支，会静默丢弃，V4.0 §8.3）。
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

#include "vehicle_control.h"
#include "vehicle_actions.h"
#include "app_event.h"
#include "framework/state_engine/state_engine.h"
#include <stddef.h>

/* ============================================================
 * 函数表（Enter / Exit 回调）—— 4 态
 * ============================================================ */
const StateFuncTable_t g_vehicle_func_table[] = {
    { VEH_STATE_OFF,      vehicle_enter_off,      vehicle_exit_off      },
    { VEH_STATE_ACC_ON,   vehicle_enter_acc_on,   vehicle_exit_acc_on   },
    { VEH_STATE_STARTED,  vehicle_enter_started,  vehicle_exit_started  },
    { VEH_STATE_FAULT,    vehicle_enter_fault,    vehicle_exit_fault    },
};
const uint16_t g_vehicle_func_table_size =
    sizeof(g_vehicle_func_table) / sizeof(g_vehicle_func_table[0]);

/* ============================================================
 * 跳转表（11 行）
 * ============================================================ */
const StateJumpTable_t g_vehicle_jump_table[] = {
    /* ===== OFF =====
     * 注意：OFF 态在 vehicle_poll() 中必先经 VEH_EVT_ACC_ON 进 ACC_ON，
     *       故障确认只在 ACC_ON / STARTED 判定 ⇒ 此处**没有** OFF→FAULT 行。 */
    {VEH_STATE_OFF,      VEH_EVT_ACC_ON,      VEH_STATE_ACC_ON},
    {VEH_STATE_OFF,      VEH_EVT_FORCE_RESET, VEH_STATE_OFF},

    /* ===== ACC_ON（门 1 成立）===== */
    {VEH_STATE_ACC_ON,   VEH_EVT_ACC_OFF,     VEH_STATE_OFF},
    {VEH_STATE_ACC_ON,   VEH_EVT_CONFIRM,     VEH_STATE_STARTED},   /* Q-A 500ms 到 */
    {VEH_STATE_ACC_ON,   VEH_EVT_FAULT,       VEH_STATE_FAULT},
    {VEH_STATE_ACC_ON,   VEH_EVT_FORCE_RESET, VEH_STATE_OFF},

    /* ===== STARTED（门 2 成立，B-1 只进不退）===== */
    {VEH_STATE_STARTED,  VEH_EVT_ACC_OFF,     VEH_STATE_OFF},
    {VEH_STATE_STARTED,  VEH_EVT_FAULT,       VEH_STATE_FAULT},
    {VEH_STATE_STARTED,  VEH_EVT_FORCE_RESET, VEH_STATE_OFF},

    /* ===== FAULT（仅手动；故障消失不自动退出）===== */
    {VEH_STATE_FAULT,    VEH_EVT_ACC_OFF,     VEH_STATE_OFF},
    {VEH_STATE_FAULT,    VEH_EVT_FORCE_RESET, VEH_STATE_OFF},
};
const uint16_t g_vehicle_jump_table_size =
    sizeof(g_vehicle_jump_table) / sizeof(g_vehicle_jump_table[0]);

/* ============================================================
 * 判定参数表 —— **RAM 运行副本**（R6 / G-01）
 *
 * 此处是 ROM 默认值；vehicle_cfg_load() 在 init 时用 sys_config 表覆盖，
 * 读取失败（BOOT_OPTIONAL）保持默认值，不静默改行为。
 * ⚠️ 旧 exit_rpm / exit_volt_dv / exit_ms 已按 B-1 作废并从本结构删除；
 *    sys_config.c 中 CFG_ID_VEH_EXIT_* 三项保留占位以免改变 Flash 布局，
 *    但业务**不再读取**（登记为历史死项，不新增使用）。
 * ============================================================ */
vehicle_cfg_t g_vehicle_cfg = {
    .enter_rpm     = 400U,      /* Q-A：> 400 rpm */
    .enter_volt_dv = 130U,      /* Q-A：> 13.0 V */
    .enter_ms      = 500U,      /* Q-A：连续 500ms */
    .fault_ms      = 2000U,     /* Q-B：SWR-N-002 的 2s 确认 */
    .poll_ms       = VEHICLE_POLL_MS,
    .can_silent_ms = 500U,      /* Q-D2：解析线程查表心跳的静默门限 */
};

/* ============================================================
 * 状态机配置
 * ============================================================ */
const StateMachineConfig_t g_vehicle_config = {
    .jump_table      = g_vehicle_jump_table,
    .jump_table_size = sizeof(g_vehicle_jump_table) / sizeof(g_vehicle_jump_table[0]),
    .func_table      = g_vehicle_func_table,
    .func_table_size = sizeof(g_vehicle_func_table) / sizeof(g_vehicle_func_table[0]),
    .init_state      = VEH_STATE_OFF,
    .max_state       = VEH_STATE_MAX,
    .name            = "VehicleSM",
};
