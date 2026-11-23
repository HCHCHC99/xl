/**
 * @file    swing_actions.c
 * @brief   回转控制动作函数实现
 *
 * @note  2026-11-15 C3 批（Q-C3 按建议代裁定 / 设计基线 §4.2 E-2）：
 *        原 7 处直接 StateMachine_SendEvent(&g_swing_obj.sm_instance, ...)
 *        绕过聚合根，现全部改走**专用投递接口**
 *        control_object_process(&g_swing_obj.base, event)。
 *        语义不变：仍是「上游条件匹配后投一次」，引擎不因此变成周期调度（E-1）。
 *        基类会先校验 base.sm 非空与 is_initialized，再同步触发查表迁移。
 *
 *        保留 StateMachine_GetState(&g_swing_obj.sm_instance) 只读查询
 *        （发生在 swing_act_tick_check 的分状态判定里），它不改变状态，
 *        不属 E-2「禁止直调 SendEvent / SetState」范围，登记为后续可选收口点。
 */

#include "swing_actions.h"

/* 【2026-11-10 彩标 P0】
 * 回转动作分级：
 *   模式禁止 / 限位拦截 / 需重按    -> WARN（操作被拒绝，但无危险）
 *   刹车请求失败 / 刹车 FAULT      -> ERR （功能失效，需排查）
 *   进入 FAULT 态                  -> FAULT（安全事件，红底）
 *   解锁/锁定推进过程              -> INFO（正常流程）*/
#include "../log/log_color.h"
#include "swing_control.h"
#include "swing_angle.h"
#include "../mode/sys_mode.h"
#include "framework/state_engine/sm_assert.h"
#include "app_event.h"
#include "../motor/motor.h"
#include "../hw/hw_brake.h"
#include "../bsp/bsp_brake.h"
#include <rtthread.h>

/* ============================================================
 * 外部引用：刹车盘对象（在 app_hw.c 中定义）
 * ============================================================ */
extern hw_brake_t g_swing_brake;

/* ============================================================
 * 内部辅助：启动电机
 * ============================================================ */
static void swing_motor_start(swing_context_t *hctx, uint8_t dir)
{
    SM_ASSERT_PTR(hctx);
    SM_ASSERT_PTR(hctx->motor_ptr);

    motor_t *motor;
    motor = hctx->motor_ptr;

    /* 【2026-11-09 改造：方向极性配置化】
     *
     * 原实现：
     *   if (dir == 1) { motor->direction = MOTOR_DIR_CW;  }   /* 左转 *\/
     *   else          { motor->direction = MOTOR_DIR_CCW; }   /* 右转 *\/
     *   —— 这是把"左转=MOTOR_DIR_CW"硬编码死，换电机/换编码器安装方向
     *      就必须改代码重编译。
     *
     * 现改为统一走 swing_dir_to_motor()，由配置项
     * CFG_ID_SWING_DIR_POLARITY 决定是否取反。
     *
     * 调用方传入的 dir 语义不变（1=SWING_DIR_DECREASE 左转，
     * 2=SWING_DIR_INCREASE 右转），因此
     * swing_act_enter_left_running/right_running 均无需改动。
     */
    motor->direction = swing_dir_to_motor(dir);

    /* 启动（软启动由电机模块内部处理） */
    motor_start(motor, motor->direction);
}

static void swing_motor_stop(swing_context_t *hctx)
{
    SM_ASSERT_PTR(hctx);
    SM_ASSERT_PTR(hctx->motor_ptr);

    motor_t *motor = hctx->motor_ptr;
    motor_stop(motor);
}

/* ============================================================
 * Enter 回调
 * ============================================================ */
void swing_act_enter_idle(StateContext_t ctx)
{
    swing_context_t *hctx = (swing_context_t*)ctx;

    SM_ASSERT_PTR(hctx);

    /* 停止电机 */
    swing_motor_stop(hctx);

    hctx->from_sequence = 0;
}

/* ============================================================
 * Enter 回调：解锁中
 * ============================================================ */
void swing_act_enter_unlocking(StateContext_t ctx)
{
    swing_context_t *hctx = (swing_context_t*)ctx;

    SM_ASSERT_PTR(hctx);

    /* 模式检查 */
    if (!sys_mode_allows_swing_motion()) {
        LOG_WARN("[Swing] Motion not allowed by mode\n");
        control_object_process(&g_swing_obj.base,
                               SWING_EVT_BRAKE_FAULT);
        return;
    }

    /* 请求解锁 */
    if (hw_brake_unlock(&g_swing_brake) != RT_EOK) {
        LOG_ERR("[Swing] brake unlock request failed\n");
        control_object_process(&g_swing_obj.base,
                               SWING_EVT_BRAKE_FAULT);
        return;
    }

    LOG_INFO("[Swing] Unlocking (dir=%d)...\n", hctx->pending_dir);
}

/* ============================================================
 * Enter 回调：锁止中
 * ============================================================ */
void swing_act_enter_locking(StateContext_t ctx)
{
    swing_context_t *hctx = (swing_context_t*)ctx;

    SM_ASSERT_PTR(hctx);

    /* 停止电机 */
    swing_motor_stop(hctx);

    /* 请求锁止 */
    hw_brake_lock(&g_swing_brake);

    LOG_INFO("[Swing] Locking...\n");
}

/* ============================================================
 * 【P8 / D-1】运行态进入动作被拒绝后的收尾
 * ------------------------------------------------------------
 * 跳转表是"先迁移状态、再执行 enter"（state_engine.c:124-129），
 * 所以 enter 里直接 return 会留下"状态在 RUNNING、电机没转"的假象：
 *   - t_swing 误报 LEFT_RUNNING；
 *   - 状态帧接了真实值后，HMI 会误报"电机在转"。
 *
 * 正确收尾是走一次"松手"：RUNNING → LOCKING → IDLE。
 * 这样刹车也会被重新锁上（拦截时刹车是解锁状态），与正常松手一致。
 *
 * 用同步投递而非 post_event：工程里 enter_brake_check 已有同款用法
 * （投 BRAKE_FAULT 进 FAULT），state_engine 先改 cur_state 再调 enter，
 * 重入时内层迁移不会被外层覆盖。
 * ============================================================ */
static void swing_abort_running(swing_context_t *hctx, app_event_t up_event)
{
    SM_ASSERT_PTR(hctx);
    (void)control_object_process(&g_swing_obj.base, up_event);
}

void swing_act_enter_left_running(StateContext_t ctx)
{
    swing_context_t *hctx = (swing_context_t*)ctx;

    SM_ASSERT_PTR(hctx);

//    if (hw_brake_get_state(&g_swing_brake) != BRAKE_STATE_UNLOCKED) {
//        rt_kprintf("[Swing] Motion not allowed by mode\n");
//        return;
//    }

    /* 模式检查 */
    if (!sys_mode_allows_swing_motion()) {
        LOG_WARN("[Swing] Motion not allowed by mode\n");
        swing_abort_running(hctx, SWING_EVT_LEFT_UP);
        return;
    }

    /* 超限拦截 */
    if (hctx->forbidden_dirs & SWING_FORBID_LEFT) {
        LOG_WARN("[Swing] Left motion forbidden (limit)\n");
        swing_abort_running(hctx, SWING_EVT_LEFT_UP);
        return;
    }

    /* 零位重触发检查 */
    if (hctx->need_retrigger && !hctx->from_sequence) {
        LOG_WARN("[Swing] Need retrigger after zero\n");
        swing_abort_running(hctx, SWING_EVT_LEFT_UP);
        return;
    }

    /* 启动电机左转 */
    swing_motor_start(hctx, 1);
}

void swing_act_enter_right_running(StateContext_t ctx)
{
    swing_context_t *hctx = (swing_context_t*)ctx;

    SM_ASSERT_PTR(hctx);

    /* 模式检查 */
    if (!sys_mode_allows_swing_motion()) {
        LOG_WARN("[Swing] Motion not allowed by mode\n");
        swing_abort_running(hctx, SWING_EVT_RIGHT_UP);
        return;
    }

    /* 超限拦截 */
    if (hctx->forbidden_dirs & SWING_FORBID_RIGHT) {
        LOG_WARN("[Swing] Right motion forbidden (limit)\n");
        swing_abort_running(hctx, SWING_EVT_RIGHT_UP);
        return;
    }

    /* 零位重触发检查 */
    if (hctx->need_retrigger && !hctx->from_sequence) {
        LOG_WARN("[Swing] Need retrigger after zero\n");
        swing_abort_running(hctx, SWING_EVT_RIGHT_UP);
        return;
    }

    /* 启动电机右转 */
    swing_motor_start(hctx, 2);
}

void swing_act_enter_fault(StateContext_t ctx)
{
    swing_context_t *hctx = (swing_context_t*)ctx;

    SM_ASSERT_PTR(hctx);

    /* 立即停止电机 */
    swing_motor_stop(hctx);

    LOG_FAULT("[Swing] Enter FAULT state\n");
}

/* ============================================================
 * Exit 回调
 * ============================================================ */
void swing_act_exit_left_running(StateContext_t ctx)
{
    swing_context_t *hctx = (swing_context_t*)ctx;
    swing_motor_stop(hctx);
}

void swing_act_exit_right_running(StateContext_t ctx)
{
    swing_context_t *hctx = (swing_context_t*)ctx;
    swing_motor_stop(hctx);
}

/* ============================================================
 * 50ms Tick 检查
 * ============================================================ */
void swing_act_tick_check(StateContext_t ctx)
{
    swing_context_t *hctx = (swing_context_t*)ctx;
    swing_angle_t   *angle;
    swing_state_t    cur_state;

    SM_ASSERT_PTR(hctx);
    SM_ASSERT_PTR(hctx->angle_ptr);

    angle = hctx->angle_ptr;

    /* ----- 全局：更新超限拦截掩码 ----- */
    hctx->forbidden_dirs = 0;
    if (swing_angle_is_left_limit(angle)) {
        hctx->forbidden_dirs |= SWING_FORBID_LEFT;
    }
    if (swing_angle_is_right_limit(angle)) {
        hctx->forbidden_dirs |= SWING_FORBID_RIGHT;
    }

    /* ----- 分状态 Tick 处理 ----- */
    cur_state = (swing_state_t)StateMachine_GetState(&g_swing_obj.sm_instance);

    switch (cur_state) {
    case H_STATE_UNLOCKING: {
        hw_brake_state_t st = hw_brake_get_state(&g_swing_brake);
        if (st == BRAKE_STATE_UNLOCKED) {
            /* 按 pending_dir 上报不同事件 */
            if (hctx->pending_dir == 1U) {
                control_object_process(&g_swing_obj.base,
                                       SWING_EVT_BRAKE_UNLOCKED_LEFT);
            } else {
                control_object_process(&g_swing_obj.base,
                                       SWING_EVT_BRAKE_UNLOCKED_RIGHT);
            }
        } else if (st == BRAKE_STATE_FAULT) {
            LOG_ERR("[Swing] brake FAULT during unlock\n");
            control_object_process(&g_swing_obj.base,
                                   SWING_EVT_BRAKE_FAULT);
        }
        break;
    }

    case H_STATE_LOCKING: {
        hw_brake_state_t st = hw_brake_get_state(&g_swing_brake);
        if (st == BRAKE_STATE_LOCKED) {
            control_object_process(&g_swing_obj.base,
                                   SWING_EVT_BRAKE_LOCKED);
        } else if (st == BRAKE_STATE_FAULT) {
            LOG_ERR("[Swing] brake FAULT during lock\n");
            control_object_process(&g_swing_obj.base,
                                   SWING_EVT_BRAKE_FAULT);
        }
        break;
    }

    default:
        break;
    }
}

/* ============================================================
 * Tick 检查：解锁中
 * ============================================================ */
void swing_act_tick_unlocking(StateContext_t ctx)
{
    swing_context_t *hctx = (swing_context_t*)ctx;
    (void)hctx;

    hw_brake_state_t st = hw_brake_get_state(&g_swing_brake);

    if (st == BRAKE_STATE_UNLOCKED) {
        /* 状态机切到 left_running 或 right_running */
    } else if (st == BRAKE_STATE_FAULT) {
        LOG_ERR("[Swing] brake FAULT during unlock\n");
        /* 状态机切到 fault */
    }
}


/* ============================================================
 * Tick 检查：锁止中
 * ============================================================ */
void swing_act_tick_locking(StateContext_t ctx)
{
    swing_context_t *hctx = (swing_context_t*)ctx;
    (void)hctx;

    hw_brake_state_t st = hw_brake_get_state(&g_swing_brake);

    if (st == BRAKE_STATE_LOCKED || st == BRAKE_STATE_FAULT) {
        /* 状态机切到 idle */
    }
}
