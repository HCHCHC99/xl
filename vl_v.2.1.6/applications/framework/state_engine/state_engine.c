/**
 * @file    state_engine.c
 * @brief   状态机引擎实现
 * @note    断言配置方式：
 *          1. 用户可在编译选项中定义 SM_ASSERT_CONFIG_FILE
 *             指定自定义断言配置文件路径
 *          2. 否则，引擎内部提供标准 assert 作为默认实现
 */

#include "state_engine.h"
#include "sm_assert.h"
#include <stddef.h>

/* ============================================================
 * 内部函数
 * ============================================================ */

static void state_machine_execute_func(StateMachine_t *sm, State_t state, uint8_t is_enter)
{
    uint16_t i;
    uint8_t found = 0;

    SM_ASSERT_PTR(sm);
    SM_ASSERT_PTR(sm->func_table);
    SM_ASSERT_STATE(state, sm->max_state);

    for (i = 0; i < sm->func_table_size; i++) {
        const StateFuncTable_t *entry = &sm->func_table[i];
        if (entry->state == state) {
            found = 1;
            if (is_enter && entry->enter != NULL) {
                entry->enter(sm->context);
            } else if (!is_enter && entry->exit != NULL) {
                entry->exit(sm->context);
            }
            break;
        }
    }

    SM_ASSERT_MSG(found, "State not found in func_table");
}

static void state_machine_validate_config(const StateMachineConfig_t *cfg)
{
    SM_ASSERT_PTR(cfg);
    SM_ASSERT_PTR(cfg->jump_table);
    SM_ASSERT_MSG(cfg->jump_table_size > 0, "Jump table empty");
    SM_ASSERT_PTR(cfg->func_table);
    SM_ASSERT_MSG(cfg->func_table_size > 0, "Func table empty");
    SM_ASSERT_STATE(cfg->init_state, cfg->max_state);
    SM_ASSERT_MSG(cfg->max_state > 0, "max_state must be > 0");
}

/* ============================================================
 * 公开 API
 * ============================================================ */

void StateMachine_Init(StateMachine_t *sm,
                       const StateMachineConfig_t *cfg,
                       StateContext_t context)
{
    SM_ASSERT_PTR(sm);

    state_machine_validate_config(cfg);

    sm->jump_table = cfg->jump_table;
    sm->jump_table_size = cfg->jump_table_size;
    sm->func_table = cfg->func_table;
    sm->func_table_size = cfg->func_table_size;
    sm->max_state = cfg->max_state;
    sm->name = cfg->name;

    sm->init_state = cfg->init_state;
    sm->cur_state = cfg->init_state;
    sm->previous_state = cfg->init_state;
    sm->state_enter_tick = 0;
    sm->context = context;
    sm->is_initialized = 1;

    state_machine_execute_func(sm, sm->cur_state, 1);
}

void StateMachine_SendEvent(StateMachine_t *sm, Event_t event)
{
    State_t target_state;
    uint16_t i;
    uint8_t matched = 0;

    SM_ASSERT_PTR(sm);
    SM_ASSERT_INITIALIZED(sm);

    target_state = sm->cur_state;

    for (i = 0; i < sm->jump_table_size; i++) {
        const StateJumpTable_t *entry = &sm->jump_table[i];
        if (entry->cur_state == sm->cur_state && entry->event == event) {
            target_state = entry->target_state;
            matched = 1;
            break;
        }
    }

//    SM_ASSERT_MSG(matched, "No transition found for current state and event");
    /* 【D1 设计选择 / 用户 2026-11-14 18:45 裁定，原 SCHEDULED 修复计划作废】
     * 未命中 = 「该状态下该事件无定义」⇒ 静默忽略是预期语义，不是缺陷。
     * 不恢复断言：sm_assert.h 的 rt_assert_handler 会打印后停机。
     * 转移表漏项改由开发期/上电自检的表覆盖校验发现。 */
    if(!matched){
        return;
    }
    SM_ASSERT_STATE(target_state, sm->max_state);

    /* 【D2 设计选择 / 用户 2026-11-14 18:45 裁定，原 SCHEDULED 修复计划作废】
     * 状态未改变 ⇒ 状态机不需要更新；enter/exit 是状态变化的钩子。
     * 另：app_main.c:131-155 的 50ms tick 不投递给状态机（引擎非周期调度），
     *     现有 26 行自转移表项是「显式忽略」的声明，不影响运行时。 */
    if (target_state == sm->cur_state) {
        return;
    }

    state_machine_execute_func(sm, sm->cur_state, 0);

    sm->previous_state = sm->cur_state;
    sm->cur_state = target_state;

    state_machine_execute_func(sm, sm->cur_state, 1);
}

void StateMachine_Reset(StateMachine_t *sm)
{
    SM_ASSERT_PTR(sm);
    SM_ASSERT_INITIALIZED(sm);

    if (sm->cur_state == sm->init_state) {
        state_machine_execute_func(sm, sm->cur_state, 1);
        return;
    }

    state_machine_execute_func(sm, sm->cur_state, 0);

    sm->previous_state = sm->cur_state;
    sm->cur_state = sm->init_state;

    state_machine_execute_func(sm, sm->cur_state, 1);
}

State_t StateMachine_GetState(const StateMachine_t *sm)
{
    SM_ASSERT_PTR(sm);
    SM_ASSERT_INITIALIZED(sm);
    return sm->cur_state;
}

State_t StateMachine_GetPreviousState(const StateMachine_t *sm)
{
    SM_ASSERT_PTR(sm);
    SM_ASSERT_INITIALIZED(sm);
    return sm->previous_state;
}

StateContext_t StateMachine_GetContext(const StateMachine_t *sm)
{
    SM_ASSERT_PTR(sm);
    SM_ASSERT_INITIALIZED(sm);
    return sm->context;
}

uint32_t StateMachine_GetEnterTick(const StateMachine_t *sm)
{
    SM_ASSERT_PTR(sm);
    SM_ASSERT_INITIALIZED(sm);
    return sm->state_enter_tick;
}

void StateMachine_SetEnterTick(StateMachine_t *sm, uint32_t tick)
{
    SM_ASSERT_PTR(sm);
    SM_ASSERT_INITIALIZED(sm);
    sm->state_enter_tick = tick;
}

uint8_t StateMachine_IsInitialized(const StateMachine_t *sm)
{
    if (sm == NULL) return 0;
    return sm->is_initialized;
}

/* 【D3 / Q-F3 路 A，2026-11-14 E1 批整改】
 * 本函数绕过 enter/exit，定位为**调试专用**；业务侧状态变更一律走
 * control_object_process() 投递事件（seq_control.c 的 4 处 SetState 已改）。
 * 原 SCHEDULED「修复引擎」计划作废：不改引擎本体。 */
void StateMachine_SetState(StateMachine_t *sm, State_t state)
{
    SM_ASSERT_PTR(sm);
    SM_ASSERT_MSG(state < sm->max_state, "state out of range");
    sm->cur_state = state;
}
