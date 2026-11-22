/**
 * @file    motor_oc.c
 * @brief   电机过流保护实现
 */

#include "motor_oc.h"
#include "framework/state_engine/sm_assert.h"
#include <string.h>
#include <rtthread.h>

/* 【2026-11-10】彩色日志：三级过流用 FAULT 色（红底白字）*/
#include "../log/log_color.h"

/* ============================================================
 * 初始化
 * ============================================================ */
void motor_oc_init(motor_oc_ctx_t *ctx, const motor_oc_cfg_t *cfg)
{
    SM_ASSERT_PTR(ctx);
    SM_ASSERT_PTR(cfg);

    memset(ctx, 0, sizeof(motor_oc_ctx_t));
    ctx->cfg = cfg;
    ctx->state = MOTOR_OC_NONE;
    ctx->active_current_limit_a10 = cfg->rated_current_a10;
}

/* ============================================================
 * 内部辅助：获取经过时间（ms）
 * ============================================================ */
static uint32_t tick_elapsed_ms(uint32_t start_tick)
{
    return (rt_tick_get() - start_tick) * 1000 / RT_TICK_PER_SECOND;
}

/* ============================================================
 * 过流保护更新
 * ============================================================ */
motor_oc_state_t motor_oc_update(motor_oc_ctx_t *ctx, uint16_t current_a10)
{
    SM_ASSERT_PTR(ctx);
    SM_ASSERT_PTR(ctx->cfg);

    const motor_oc_cfg_t *cfg = ctx->cfg;
    uint32_t now = rt_tick_get();

    /* ============================================================
     * 三级过流检测（最高优先级）
     * ============================================================ */
    if (current_a10 >= cfg->oc3_current_a10) {
        if (!ctx->oc3_running) {
            ctx->oc3_running = 1;
            ctx->oc3_start_tick = now;
        } else if (tick_elapsed_ms(ctx->oc3_start_tick) >= cfg->oc3_debounce_ms) {
            /* ============================================================
             * 【2026-11-09 修复：LEVEL3 每 5ms 重复触发，导致刷屏 + 反复刹车】
             * ------------------------------------------------------------
             * 原实现没有任何"已确认过"的判断，只要电流仍在阈值上、去抖时间
             * 已过，就每次都走这一支：
             *   - rt_kprintf 每 5ms 打一次 → 刷屏，淹掉 MSH 调试输出
             *   - 返回 LEVEL3 → motor_update 每次都重新 brake + emergency_lock
             *     → [HW-BRAKE] emergency_locked 也跟着每 5ms 打一次
             *
             * 实测日志（用户 2026-11-09）就是这两条交替刷屏。
             *
             * 修法：只有"从非 LEVEL3 跃迁到 LEVEL3"才做动作与打印。
             *   已处于 LEVEL3 时直接返回，不动 tick、不重复刹车。
             *
             * 为什么不能简单加打印节流：
             *   打印只是表象，真正的副作用是 motor_update 里每次都重新
             *   brake() + emergency_lock()。重复刹车会不断重置刹车状态机
             *   与 elapsed_ms，属于功能性缺陷，必须从源头断掉。
             * ============================================================ */
            if (ctx->state == MOTOR_OC_LEVEL3) {
                /* 已确认过：静默维持，不重复打印、不重复刹车 */
                return MOTOR_OC_LEVEL3;
            }

            /* 三级过流首次确认：立即停机 */
            ctx->state = MOTOR_OC_LEVEL3;
            ctx->state_enter_tick = now;
            ctx->trigger_count++;
            ctx->active_current_limit_a10 = 0;
            /* 【2026-11-10 彩标】三级过流是"必须立即停机"级事件，
             * 用 LOG_FAULT（红底白字），扫屏时一眼可见。
             * 注意下一行 motor_update 会紧接着打 HW-BRAKE 的
             * emergency_locked，两条同为 FAULT 色，构成一个
             * 完整的"过流 -> 刹车"告警块。 */
            LOG_FAULT("[MOTOR-OC] LEVEL3 emergency stop! I=%d.%dA\n",
                      current_a10 / 10, current_a10 % 10);
            return MOTOR_OC_LEVEL3;
        }
    } else {
        ctx->oc3_running = 0;
    }

    /* ============================================================
     * 二级过流检测（降功率）
     * ============================================================ */
    if (current_a10 >= cfg->oc2_current_a10) {
        if (!ctx->oc2_running) {
            ctx->oc2_running = 1;
            ctx->oc2_start_tick = now;
        } else if (tick_elapsed_ms(ctx->oc2_start_tick) >= cfg->oc2_debounce_ms) {
            ctx->state = MOTOR_OC_LEVEL2;
            ctx->state_enter_tick = now;
            /* 降功率：限制电流上限 */
            ctx->active_current_limit_a10 =
                (cfg->rated_current_a10 * cfg->oc2_deboost_pct) / 100;
            LOG_WARN("[MOTOR-OC] LEVEL2 deboost to %d%%\n", cfg->oc2_deboost_pct);
            return MOTOR_OC_LEVEL2;
        }
    } else {
        ctx->oc2_running = 0;
    }

    /* ============================================================
     * 一级过流检测（警告）
     * ============================================================ */
    if (current_a10 >= cfg->oc1_current_a10) {
        if (!ctx->oc1_running) {
            ctx->oc1_running = 1;
            ctx->oc1_start_tick = now;
        } else if (tick_elapsed_ms(ctx->oc1_start_tick) >= cfg->oc1_debounce_ms) {
            ctx->state = MOTOR_OC_LEVEL1;
            ctx->state_enter_tick = now;
            /* 一级：仅警告，不限流 */
            ctx->active_current_limit_a10 = cfg->rated_current_a10;
            return MOTOR_OC_LEVEL1;
        }
    } else {
        ctx->oc1_running = 0;
        /* 无过流：恢复 */
        if (ctx->state == MOTOR_OC_LEVEL1 || ctx->state == MOTOR_OC_LEVEL2) {
            ctx->state = MOTOR_OC_NONE;
            ctx->active_current_limit_a10 = cfg->rated_current_a10;
        }
    }

    return ctx->state;
}

/* ============================================================
 * 获取当前限流值
 * ============================================================ */
uint16_t motor_oc_get_limit(const motor_oc_ctx_t *ctx)
{
    SM_ASSERT_PTR(ctx);
    return ctx->active_current_limit_a10;
}

/* ============================================================
 * 手动复位
 * ============================================================ */
rt_err_t motor_oc_reset(motor_oc_ctx_t *ctx)
{
    SM_ASSERT_PTR(ctx);
    SM_ASSERT_PTR(ctx->cfg);

    if (ctx->state != MOTOR_OC_LEVEL3) {
        return -RT_ERROR;
    }

    ctx->state = MOTOR_OC_NONE;
    ctx->oc1_running = 0;
    ctx->oc2_running = 0;
    ctx->oc3_running = 0;
    ctx->active_current_limit_a10 = ctx->cfg->rated_current_a10;

    LOG_INFO("[MOTOR-OC] Manual reset\n");
    return RT_EOK;
}

/* ============================================================
 * 查询
 * ============================================================ */
motor_oc_state_t motor_oc_get_state(const motor_oc_ctx_t *ctx)
{
    SM_ASSERT_PTR(ctx);
    return ctx->state;
}
