/**
 * @file    state_engine.h
 * @brief   通用事件驱动状态机引擎
 * @note    完全独立，不包含任何断言或操作系统相关代码
 *
 * 【设计本质 / 用户 2026-11-14 19:08 明确】
 *   本引擎 **不是周期性调度器**：它不自己跑 tick、不轮询、不加周期任务。
 *   只有在「上游判定匹配到事件」之后，才通过**专用投递接口**
 *   control_object_process()（= obj_base_process，obj_base.c:106）把事件
 *   同步投给状态机，触发一次查表与迁移。
 *   ⇒ 采样/去抖/计时是**上游判定逻辑**的职责（例如 50ms tick 里的
 *     vehicle_poll / *_act_tick_check），条件满足后投一次事件即可；
 *     引擎绝不每周期查表。StateFuncTable_t 也刻意不提供 tick 回调
 *     （见 app_main.c:131-155 的显式周期函数）。
 *
 * 【D1/D2 为设计选择，不是缺陷】（用户 2026-11-14 18:45 裁定）
 *   - 未命中转移：该状态下该事件无定义 ⇒ 静默忽略，不报错、不停机。
 *   - 自转移：状态未改变 ⇒ 状态机不需要更新，不执行 enter/exit。
 *   转移表漏项由**开发期/上电自检的表覆盖校验**保障，不在运行时加诊断。
 */

#ifndef __STATE_ENGINE_H
#define __STATE_ENGINE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 基础类型定义
 * ============================================================ */

typedef uint32_t Event_t;
typedef uint32_t State_t;
typedef void* StateContext_t;

/* ============================================================
 * 回调函数类型
 * ============================================================ */

typedef void (*StateFunc_t)(StateContext_t context);

/* ============================================================
 * 状态机配置表结构
 * ============================================================ */

/**
 * @brief 状态跳转表项
 */
typedef struct {
    State_t cur_state;
    Event_t event;
    State_t target_state;
} StateJumpTable_t;

/**
 * @brief 状态函数表项
 */
typedef struct {
    State_t state;
    StateFunc_t enter;
    StateFunc_t exit;
} StateFuncTable_t;

/**
 * @brief 状态机配置（只读）
 */
typedef struct {
    const StateJumpTable_t *jump_table;
    uint16_t jump_table_size;
    const StateFuncTable_t *func_table;
    uint16_t func_table_size;
    State_t init_state;
    State_t max_state;
    const char *name;
} StateMachineConfig_t;

/* ============================================================
 * 状态机对象
 * ============================================================ */

typedef struct {
    /* ----- 配置引用 ----- */
    const StateJumpTable_t *jump_table;
    uint16_t jump_table_size;
    const StateFuncTable_t *func_table;
    uint16_t func_table_size;
    State_t max_state;
    const char *name;

    /* ----- 运行时状态 ----- */
    State_t cur_state;
    State_t previous_state;
    State_t init_state;
    /* 【Q-F4 / 2026-11-14 裁定：弃用】未维护的预留字段。
     * Init 置 0 后 SendEvent 从不更新，全工程无业务读写者。
     * 去抖/迟滞一律用各对象 ctx 自有的 *_tick 字段（平面状态机 +
     * 显式子状态字段，见 三对象建模-标准设计方案评估 §六/§4.3）。
     * 保留字段仅为不改 StateMachine_t 布局（避免影响 5 个对象）。 */
    uint32_t state_enter_tick;
    StateContext_t context;
    uint8_t is_initialized;
} StateMachine_t;

/* ============================================================
 * API
 * ============================================================ */

void StateMachine_Init(StateMachine_t *sm,
                       const StateMachineConfig_t *cfg,
                       StateContext_t context);

void StateMachine_SendEvent(StateMachine_t *sm, Event_t event);

void StateMachine_Reset(StateMachine_t *sm);

State_t StateMachine_GetState(const StateMachine_t *sm);
State_t StateMachine_GetPreviousState(const StateMachine_t *sm);
StateContext_t StateMachine_GetContext(const StateMachine_t *sm);
uint32_t StateMachine_GetEnterTick(const StateMachine_t *sm);
void StateMachine_SetEnterTick(StateMachine_t *sm, uint32_t tick);
uint8_t StateMachine_IsInitialized(const StateMachine_t *sm);
void StateMachine_SetState(StateMachine_t *sm, State_t state);

#ifdef __cplusplus
}
#endif

#endif /* __STATE_ENGINE_H */
