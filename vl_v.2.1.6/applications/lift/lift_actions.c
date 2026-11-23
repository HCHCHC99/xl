/**
 * @file    lift_actions.c
 * @brief   举升控制状态机动作函数
 */

#include "lift_actions.h"
#include "lift_control.h"
#include "lift_angle.h"
#include "../mode/sys_mode.h"
#include "../log/log_color.h"       /* LOG_WARN（P6：限位拦截打点） */
#include "framework/state_engine/sm_assert.h"
#include "app_event.h"
#include "valve_tx.h"
#include <rtthread.h>

/* 外部函数声明（在 lift_control.c 中实现） */
extern void lift_control_start_timer(void);
extern void lift_control_stop_timer(void);

/* ============================================================
 * 动作函数：进入 IDLE（重置标志）
 * ============================================================ */
void lift_act_enter_idle(StateContext_t ctx)
{
    lift_context_t *lift = (lift_context_t*)ctx;
    SM_ASSERT_PTR(lift);

    lift->hold_start_tick = 0;
    lift->hold_direction = 0;
}

/* ============================================================
 * 动作函数：开启上升阀
 * ============================================================ */
void lift_act_valve_on_up(StateContext_t ctx)
{
    lift_context_t *lift = (lift_context_t*)ctx;
    SM_ASSERT_PTR(lift);
    SM_ASSERT_PTR(lift->solenoid_ptr);

    /* 【2026-10-27】能力位门禁。举升手动三模式均允许（表 F-03），
     * 此处加判定只为与 swing_actions.c 保持一致，避免将来收紧时漏点。 */
    if (!sys_mode_allows_swing_motion()) {
        return;
    }

    /* 互锁检查：如果下降开关已按下，不执行 */
    if (lift->down_sw_pressed) {
        return;
    }

    /* 【P6 / B-7】已在上限就不要再通电：否则"到限位保持 5 s"会变成
     * "在机械限位处再顶 5 s"，对阀和机构都不好。
     * 拒绝后 valve_output 保持 0，enter_hold 会据此判定"没通过电"，直接退出保持。 */
    if (lift_angle_is_upper_limit(lift->lift_ptr)) {
        LOG_WARN("[Lift] Up refused: already at upper limit\n");
        return;
    }

    lift->solenoid_ptr->valve_output |= 0x01;   /* 上升阀得电 */
    lift->solenoid_ptr->valve_output &= ~0x02;  /* 下降阀失电 */
    valve_tx_notify_change();
}

/* ============================================================
 * 动作函数：开启下降阀
 * ============================================================ */
void lift_act_valve_on_down(StateContext_t ctx)
{
    lift_context_t *lift = (lift_context_t*)ctx;
    SM_ASSERT_PTR(lift);
    SM_ASSERT_PTR(lift->solenoid_ptr);

    /* 【2026-10-27】同 valve_on_up，见上方说明 */
    if (!sys_mode_allows_swing_motion()) {
        return;
    }

    /* 互锁检查：如果上升开关已按下，不执行 */
    if (lift->up_sw_pressed) {
        return;
    }

    /* 【P6 / B-7】同 on_up：已在下限不再通电 */
    if (lift_angle_is_lower_limit(lift->lift_ptr)) {
        LOG_WARN("[Lift] Down refused: already at lower limit\n");
        return;
    }

    lift->solenoid_ptr->valve_output |= 0x02;   /* 下降阀得电 */
    lift->solenoid_ptr->valve_output &= ~0x01;  /* 上升阀失电 */
    valve_tx_notify_change();
}

/* ============================================================
 * 动作函数：关闭所有阀
 * ============================================================ */
void lift_act_valve_off(StateContext_t ctx)
{
    lift_context_t *lift = (lift_context_t*)ctx;
    SM_ASSERT_PTR(lift);
    SM_ASSERT_PTR(lift->solenoid_ptr);

    /* 【P6 / B-7】记下"离开运行态时阀是否真的通过电"。
     * enter_hold 据此区分两种情况：
     *   ① 运行中撞到限位  → 阀曾通电 → 继续保持 5 s；
     *   ② 在限位处按同向键 → on_up/on_down 已拒绝、阀从未通电 → 保持无意义，立即退出。 */
    lift->valve_was_on = (lift->solenoid_ptr->valve_output != 0U) ? 1U : 0U;

    lift->solenoid_ptr->valve_output = 0;
    valve_tx_notify_change();
}


/* ============================================================
 * 动作函数：进入限位保持状态
 * ============================================================ */
void lift_act_enter_hold(StateContext_t ctx)
{
    lift_context_t *lift = (lift_context_t*)ctx;
    SM_ASSERT_PTR(lift);
    SM_ASSERT_PTR(lift->solenoid_ptr);

    /* 记录开始时间 */
    lift->hold_start_tick = rt_tick_get();

    /* 【P6 / B-7 关键一步】进入本状态前，UP/DOWN_RUNNING 的 exit 回调已经把阀关了
     * （函数表里两者都是 lift_act_valve_off）。若不在这里按 hold_direction 重新打开，
     * "到限位后继续输出 5 s"就成了空动作——状态是保持，阀却是断的。
     * hold_direction 由 lift_act_tick_check() 在投 LIMIT_REACHED 之前写好：
     *   1 = 上升到位，0 = 下降到位。 */
    if (lift->valve_was_on == 0U) {
        /* 在限位处按同向键：on_up/on_down 已拒绝，阀从未通电。
         * 保持没有意义，直接按超时处理，下一拍回 IDLE（输出始终 0）。 */
        lift->valve_was_on = 1U;      /* 复位，供下次判断 */
        post_event(LIFT_EVT_TIMEOUT_5S);
        return;
    }
    lift->valve_was_on = 1U;

    if (lift->hold_direction != 0U) {
        lift->solenoid_ptr->valve_output |= 0x01;
        lift->solenoid_ptr->valve_output &= (uint8_t)~0x02;
    } else {
        lift->solenoid_ptr->valve_output |= 0x02;
        lift->solenoid_ptr->valve_output &= (uint8_t)~0x01;
    }
    valve_tx_notify_change();

    /* 【2026-11-10 Q29 裁定：限位保持功能先注释掉】
     *
     * 原行为：启动 5s 定时器，到时投 LIFT_EVT_TIMEOUT_5S → IDLE
     * 现状态：P_STATE_LIMIT_HOLDING 已无入边（跳转表改到 IDLE），
     *         本函数不会被执行，定时器也就不需要启动。
     *
     * 定时器对象本身**保留创建**（lift_control.c:71），
     * 因为 stop 调用仍在 lift_act_exit_hold，删除会导致状态不一致。
     * 【P6 / B-7 已恢复】4 行目标已改回 LIMIT_HOLDING，定时器重新启用；
     * 保持时间由 CFG_ID_LIFT_HOLD_MS（默认 5000ms，可 t_cfg 调整，0=立即关阀）。 */
    lift_control_start_timer();
}

/* ============================================================
 * 动作函数：退出限位保持状态
 * ============================================================ */
void lift_act_exit_hold(StateContext_t ctx)
{
    lift_context_t *vctx = (lift_context_t*)ctx;
    SM_ASSERT_PTR(vctx);

    /* 停止定时器 */
    lift_control_stop_timer();

    /* 关闭所有阀 */
    vctx->solenoid_ptr->valve_output = 0;
    valve_tx_notify_change();
}

/* ============================================================
 * 动作函数：进入故障状态（电磁阀故障）
 * ============================================================ */
void lift_act_enter_fault_valve(StateContext_t ctx)
{
    lift_context_t *lift = (lift_context_t*)ctx;
    SM_ASSERT_PTR(lift);
    SM_ASSERT_PTR(lift->solenoid_ptr);

    /* 立即停止所有输出 */
    lift->solenoid_ptr->valve_output = 0;
    valve_tx_notify_change();

    /* 停止定时器 */
    lift_control_stop_timer();

    /* 上报故障信息（可通过日志或CAN报文） */
}

/* ============================================================
 * 动作函数：进入故障状态（传感器故障）
 * ============================================================ */
void lift_act_enter_fault_sensor(StateContext_t ctx)
{
    lift_context_t *lift = (lift_context_t*)ctx;
    SM_ASSERT_PTR(lift);
    SM_ASSERT_PTR(lift->solenoid_ptr);

    /* 立即停止所有输出 */
    lift->solenoid_ptr->valve_output = 0;
    valve_tx_notify_change();

    /* 停止定时器 */
    lift_control_stop_timer();

    /* 上报故障信息 */
}


/* ============================================================
 * 动作函数：50ms Tick 检查（限位检测 + 故障监控）
 * ============================================================ */
void lift_act_tick_check(StateContext_t ctx)
{
    lift_context_t *lift = (lift_context_t*)ctx;
    SM_ASSERT_PTR(lift);
    SM_ASSERT_PTR(lift->lift_ptr);

    /* 获取当前状态（使用全局状态机） */
    State_t cur_state = StateMachine_GetState(&g_lift_obj.sm_instance);

    /* 传感器故障时：限位检测禁用，但手动仍然允许（TC-ANG-013）。
     * 手动松开后状态会落回 IDLE；此时必须把状态推回 FAULT_SENSOR，
     * 否则"传感器故障"这个状态就随手动一起丢了（只剩 ctx 里的标志）。
     * 只在 IDLE 投一次：投完状态机立刻迁移，下一拍已是 FAULT_SENSOR。 */
    if (lift->sensor_fault) {
        if (cur_state == P_STATE_IDLE) {
            post_event(LIFT_EVT_SENSOR_FAULT);
        }
        return;
    }

    /* 运行状态检测限位 */
    if (cur_state == P_STATE_UP_RUNNING) {
        if (lift_angle_is_upper_limit(lift->lift_ptr)) {
            lift->hold_direction = 1;   /* 上升方向触发的限位 */
            post_event(LIFT_EVT_LIMIT_REACHED_UPPER);
        }
    } else if (cur_state == P_STATE_DOWN_RUNNING) {
        if (lift_angle_is_lower_limit(lift->lift_ptr)) {
            lift->hold_direction = 0;   /* 下降方向触发的限位 */
            post_event(LIFT_EVT_LIMIT_REACHED_LOWER);
        }
    }
}

