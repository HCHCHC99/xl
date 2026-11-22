/**
 * @file    vehicle_actions.c
 * @brief   整车四态状态机的 enter / exit 回调（2026-11-14 P2）
 *
 * 【D-4 / REQ-PWR-06】**不再发送 SYS_EVT_VEHICLE_STOPPED**。
 *   钥匙掉电 ≠ 发动机熄火；B-1 已作废 STOPPING / stop_pending。
 *   序列侧 ctx->vehicle_started 的清除改由 app_main 的 SYS_EVT_ACC_OFF
 *   分支执行（Q-C「确认后禁动作」随本批补）。
 *
 * 【仅真实启车才广播】只有 ACC_ON --CONFIRM--> STARTED 一条路径会发
 *   SYS_EVT_VEHICLE_STARTED；FAULT 不重发、故障恢复也不重发。
 */

#include "vehicle_control.h"
#include "vehicle_actions.h"
#include "app_event.h"
#include "../keyboard/keyboard_sm.h"
#include "framework/port/osal.h"
#include <stddef.h>

/* ============================================================
 * OFF
 * ============================================================ */
void vehicle_enter_off(StateContext_t context)
{
    vehicle_context_t *ctx = (vehicle_context_t *)context;

    if (ctx != NULL) {
        ctx->rpm_cond_ms   = 0U;    /* Q-A V1.9：两个独立计时器都清零 */
        ctx->volt_cond_ms  = 0U;
        ctx->fault_cond_ms = 0U;
        ctx->off_count++;
    }

    osal_log("[VEH] -> OFF (acc_off / force reset; B-1 clear)\n");
    /* 不发 SYS_EVT_VEHICLE_STOPPED —— 见文件头说明 */
}

void vehicle_exit_off(StateContext_t context)
{
    (void)context;
}

/* ============================================================
 * ACC_ON
 * ============================================================ */
void vehicle_enter_acc_on(StateContext_t context)
{
    vehicle_context_t *ctx = (vehicle_context_t *)context;

    if (ctx != NULL) {
        ctx->rpm_cond_ms  = 0U;
        ctx->volt_cond_ms = 0U;
    }

    osal_log("[VEH] -> ACC_ON (key on confirmed; rpm>%u OR volt>%u.%uV, EACH held %ums)\n",
             (unsigned)g_vehicle_cfg.enter_rpm,
             (unsigned)(g_vehicle_cfg.enter_volt_dv / 10U),
             (unsigned)(g_vehicle_cfg.enter_volt_dv % 10U),
             (unsigned)g_vehicle_cfg.enter_ms);
}

void vehicle_exit_acc_on(StateContext_t context)
{
    (void)context;
}

/* ============================================================
 * STARTED
 * ============================================================ */
void vehicle_enter_started(StateContext_t context)
{
    vehicle_context_t *ctx = (vehicle_context_t *)context;

    if (ctx != NULL) {
        ctx->rpm_cond_ms  = 0U;
        ctx->volt_cond_ms = 0U;
        ctx->started_count++;
    }

    osal_log("[VEH] STARTED: rpm=%u, volt=%u.%uV (Q-A V1.9: one independent timer reached 500ms)\n",
             (unsigned)(ctx ? ctx->last_rpm : 0U),
             (unsigned)(ctx ? ctx->last_dv / 10U : 0U),
             (unsigned)(ctx ? ctx->last_dv % 10U : 0U));

    /* 对外广播（语义不变：确认后才发） */
    post_event(SYS_EVT_VEHICLE_STARTED);

    /* 启车后需重新按下：通知键盘模块作废当前按键，须松开再按 */
    kb_request_rebaseline();
}

void vehicle_exit_started(StateContext_t context)
{
    (void)context;
    /* 【D-4】不发 SYS_EVT_VEHICLE_STOPPED；序列侧由 KEY_OFF 清除 */
}

/* ============================================================
 * FAULT —— Q-B ①「仅手动」
 *
 * 降级效果由门禁自然实现：
 *   vehicle_is_started() 在 FAULT 返回 0 ⇒ 一键 / 离合被 app_main 门禁拒绝；
 *   手动四向由门 1（acc_is_on）决定 ⇒ 钥匙上电时仍可动作。
 * 本对象不发布「能不能动作」的许可（铁律 3）。
 * ============================================================ */
void vehicle_enter_fault(StateContext_t context)
{
    vehicle_context_t *ctx = (vehicle_context_t *)context;

    osal_log("[VEH] -> FAULT (word=0x%lX, held %ums); degrade=MANUAL-ONLY\n",
             (unsigned long)(ctx ? ctx->fault_word : 0UL),
             (unsigned)g_vehicle_cfg.fault_ms);
}

void vehicle_exit_fault(StateContext_t context)
{
    (void)context;
}
