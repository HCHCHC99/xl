/**
 * @file    seq_engine.c
 * @brief   通用序列执行引擎实现
 */

#include "seq_engine.h"

/* 【2026-11-10 彩标 P2】
 * 步骤进入/完成=STATE（青），序列完成=OK（绿），
 * 步超时/序列中止=WARN（黄），无法启动=WARN。*/
#include "../log/log_color.h"
#include "seq_actions.h"
#include "app_event.h"
#include "framework/state_engine/sm_assert.h"
#include <string.h>
#include <stdlib.h>
#include <rtthread.h>

/* ============================================================
 * 【P2 / 2026-11-23】台架条件强制开关
 * ------------------------------------------------------------
 * 用于没有人转筒/举升机构时把「角度类完成条件」直接判为满足，
 * 便于一次跑完整条序列观察步骤编排（t_seq_auto 1/0）。
 * 只影响 LIFT_GE / LIFT_LE / LIFT_NEAR / SWING_NEAR 四类角度条件；
 * TIME / NONE / CUSTOM 保持真实语义，避免掩盖时序问题。
 * 默认 0（真实判定），纯 RAM，不落盘，重启即恢复。
 * ============================================================ */
static uint8_t s_force_cond = 0u;

void seq_engine_set_force_cond(uint8_t en)
{
    s_force_cond = (en != 0u) ? 1u : 0u;
}

uint8_t seq_engine_get_force_cond(void)
{
    return s_force_cond;
}

/* ============================================================
 * 内部辅助：检查步骤完成条件
 * ============================================================ */
static uint8_t check_step_condition(seq_context_t *ctx, const seq_step_t *step)
{
    int32_t v_angle = 0, h_angle = 0;

    /* 台架强制：角度类条件直接满足（t_seq_auto 1） */
    if (s_force_cond != 0u) {
        switch (step->condition) {
            case SEQ_COND_LIFT_GE:
            case SEQ_COND_LIFT_LE:
            case SEQ_COND_LIFT_NEAR:
            case SEQ_COND_SWING_NEAR:
                return 1;
            default:
                break;
        }
    }

    /* 从外部对象读取当前角度 */
    if (ctx->lift_angle) {
        extern int32_t lift_angle_get_value(const void *angle);
        v_angle = lift_angle_get_value(ctx->lift_angle);
    }
    if (ctx->swing_angle) {
        extern int32_t swing_angle_get_value(const void *angle);
        h_angle = swing_angle_get_value(ctx->swing_angle);
    }

    switch (step->condition) {
        case SEQ_COND_NONE:
            /* 立即完成 */
            return 1;

        case SEQ_COND_LIFT_GE:
            return (v_angle >= step->target);

        case SEQ_COND_LIFT_LE:
            return (v_angle <= step->target);

        case SEQ_COND_LIFT_NEAR:
            return (abs(v_angle - step->target) <= ctx->tolerance);

        case SEQ_COND_SWING_NEAR:
            return (abs(h_angle - step->target) <= ctx->tolerance);

        case SEQ_COND_TIME:
            return ((rt_tick_get() - ctx->step_start_tick)
                    >= (step->time_ms * RT_TICK_PER_SECOND / 1000));

        case SEQ_COND_CUSTOM:
            if (step->custom_cond) {
                return step->custom_cond(ctx);
            }
            return 1;

        default:
            return 0;
    }
}

/* ============================================================
 * 内部辅助：进入新步骤
 * ============================================================ */
static void enter_step(seq_context_t *ctx, uint32_t step_idx)
{
    SM_ASSERT_PTR(ctx);
    SM_ASSERT_MSG(step_idx < ctx->config->step_count, "step index out of range");

    const seq_step_t *step = &ctx->config->steps[step_idx];

    LOG_STATE("[Seq] Enter step %d: %s\n", step_idx, step->name);

    /* 执行步骤动作 */
    seq_action_execute(ctx, step);

    /* 记录时间 */
    ctx->step_start_tick = rt_tick_get();
    ctx->step_entered = 1;

    /* 步骤进入回调 */
    if (ctx->on_step_enter) {
        ctx->on_step_enter(ctx, step_idx);
    }
}

/* ============================================================
 * 内部辅助：退出当前步骤
 * ------------------------------------------------------------
 * normal_exit = 1：条件满足的正常退出，遵守步骤的 keep_lift
 * normal_exit = 0：超时 / 中止，一律全停（安全方向优先）
 * ============================================================ */
static void exit_current_step(seq_context_t *ctx, uint8_t normal_exit)
{
    SM_ASSERT_PTR(ctx);

    if (!ctx->step_entered) return;

    /* 步骤退出回调 */
    if (ctx->on_step_exit) {
        ctx->on_step_exit(ctx, ctx->current_step);
    }

    /* 停止当前步骤的动作。
     * 【P7 / B-4】keep_lift：本步正常完成后保留举升阀输出，
     * 供紧随其后的"保持 N 秒"步继续吃（home 的 2 s 保持即此用法）。 */
    {
        uint8_t stop_lift = 1U;

        if ((normal_exit != 0U) && (ctx->current_step < ctx->config->step_count)) {
            if (ctx->config->steps[ctx->current_step].keep_lift != 0U) {
                stop_lift = 0U;
                LOG_STATE("[Seq] Step %d keeps lift output (hold next)\n",
                          ctx->current_step);
            }
        }
        seq_action_stop_ex(ctx, stop_lift);
    }

    ctx->step_entered = 0;
}

/* ============================================================
 * 公开 API
 * ============================================================ */
void seq_engine_init(seq_context_t *ctx,
                     const seq_config_t *config,
                     seq_type_t type,
                     void *swing_ctrl,
                     void *vert_ctrl,
                     void *swing_angle,
                     void *lift_angle)
{
    SM_ASSERT_PTR(ctx);
    SM_ASSERT_PTR(config);

    memset(ctx, 0, sizeof(seq_context_t));

    ctx->config = config;
    ctx->type = type;
    ctx->state = SEQ_STATE_IDLE;
    ctx->tolerance = 20;    /* 默认 ±2° */

    ctx->swing_ctrl = swing_ctrl;
    ctx->vert_ctrl = vert_ctrl;
    ctx->swing_angle = swing_angle;
    ctx->lift_angle = lift_angle;

    /* 根据序列类型设置目标 */
    switch (type) {
        case SEQ_TYPE_RIGHT_MIDDLE:
            ctx->target_h_angle = 1200;   /* +120° */
            ctx->target_v_angle = 340;    /* +34° */
            break;
        case SEQ_TYPE_RIGHT_BEHIND:
            ctx->target_h_angle = 2150;   /* +215° */
            ctx->target_v_angle = 340;    /* +34° */
            break;
        case SEQ_TYPE_HOME:
            ctx->target_h_angle = 0;      /* 0° */
            ctx->target_v_angle = 0;      /* 0° */
            break;
        default:
            break;
    }
}

int seq_engine_start(seq_context_t *ctx)
{
    SM_ASSERT_PTR(ctx);
    SM_ASSERT_PTR(ctx->config);

    if (ctx->state != SEQ_STATE_IDLE &&
        ctx->state != SEQ_STATE_DONE &&
        ctx->state != SEQ_STATE_ABORTED) {
        LOG_WARN("[Seq] Cannot start: not idle\n");
        return -1;
    }

    LOG_STATE("[Seq] Starting sequence: %s\n", ctx->config->name);

    ctx->current_step = 0;
    ctx->step_entered = 0;
    ctx->state = SEQ_STATE_RUNNING;

    /* 进入第一个步骤 */
    enter_step(ctx, 0);

    return 0;
}

void seq_engine_abort(seq_context_t *ctx)
{
    SM_ASSERT_PTR(ctx);

    if (ctx->state != SEQ_STATE_RUNNING &&
        ctx->state != SEQ_STATE_HOLDING) {
        return;
    }

    LOG_WARN("[Seq] Aborting sequence\n");

    /* 退出当前步骤（中止：强制全停，不给 keep_lift 留口子） */
    exit_current_step(ctx, 0U);

    ctx->state = SEQ_STATE_ABORTED;
    ctx->abort_count++;
}

void seq_engine_update(seq_context_t *ctx)
{
    SM_ASSERT_PTR(ctx);

    if (ctx->state != SEQ_STATE_RUNNING &&
        ctx->state != SEQ_STATE_HOLDING) {
        return;
    }

    const seq_step_t *step = &ctx->config->steps[ctx->current_step];

    /* 1. 检查步骤完成条件 */
    if (check_step_condition(ctx, step)) {
        LOG_STATE("[Seq] Step %d complete\n", ctx->current_step);

        /* 退出当前步骤（正常完成：遵守 keep_lift） */
        exit_current_step(ctx, 1U);

        /* 进入下一步 */
        if (ctx->current_step + 1 >= ctx->config->step_count) {
            /* 【P7 / B-4】末步误标 keep_lift 时兜底全停——序列结束不该有阀带电 */
            if (ctx->config->steps[ctx->current_step].keep_lift != 0U) {
                LOG_WARN("[Seq] last step has keep_lift, forcing all-stop\n");
                seq_action_stop_ex(ctx, 1U);
            }
            /* 所有步骤完成 */
            ctx->state = SEQ_STATE_DONE;
            ctx->complete_count++;
            LOG_OK("[Seq] Sequence COMPLETE\n");
            return;
        }

        ctx->current_step++;
        enter_step(ctx, ctx->current_step);
        return;
    }

    /* 2. 检查超时 */
    uint32_t timeout = step->timeout_ms ? step->timeout_ms
                                         : ctx->config->default_timeout_ms;
    if (timeout > 0) {
        uint32_t elapsed_ms = (rt_tick_get() - ctx->step_start_tick)
                              * 1000 / RT_TICK_PER_SECOND;
        if (elapsed_ms >= timeout) {
            LOG_ERR("[Seq] Step %d TIMEOUT\n", ctx->current_step);
            exit_current_step(ctx, 0U);
            ctx->state = SEQ_STATE_ABORTED;
            ctx->abort_count++;
            return;
        }
    }
}

uint8_t seq_engine_is_active(const seq_context_t *ctx)
{
    SM_ASSERT_PTR(ctx);
    return (ctx->state == SEQ_STATE_RUNNING ||
            ctx->state == SEQ_STATE_HOLDING);
}

seq_state_t seq_engine_get_state(const seq_context_t *ctx)
{
    SM_ASSERT_PTR(ctx);
    return ctx->state;
}

uint32_t seq_engine_get_step(const seq_context_t *ctx)
{
    SM_ASSERT_PTR(ctx);
    return ctx->current_step;
}
