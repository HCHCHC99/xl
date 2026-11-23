/**
 * @file    mode_actions.c
 * @brief   模式控制动作函数实现
 */

#include "mode_actions.h"

/* 【2026-11-10 彩标 P2】模式切换=STATE；测试模式超时=WARN */
#include "../log/log_color.h"
#include "mode_control.h"
#include "mode_config.h"
#include "sys_mode.h"
#include "sys_caps.h"
#include "app_event.h"
#include "framework/state_engine/sm_assert.h"
#include <rtthread.h>

/* ============================================================
 * 内部辅助：sys_mode_t -> mode_state_t 显式映射
 * ------------------------------------------------------------
 * 【P2 / 2026-11-23】两个枚举不是同一套：
 *   sys_mode_t   { FACTORY=0, MAINT=1, WORK=2 }
 *   mode_state_t { INIT=0, FACTORY=1, TEST=2, WORK=3 }
 * 原实现 `(mode_state_t)mode` 直接强转 ⇒ 进 MAINT 后 current_mode 变成
 * FACTORY(1)，而 mode_tick_check() 判的是 MODE_STATE_TEST(2) ⇒
 * 维护/测试模式的超时判定永远不生效（D-8）。此处改为显式映射。
 * ============================================================ */
static mode_state_t mode_to_state(sys_mode_t m)
{
    switch (m) {
        case SYS_MODE_FACTORY: return MODE_STATE_FACTORY;
        case SYS_MODE_MAINT:   return MODE_STATE_TEST;   /* TEST 与 MAINT 同值 */
        case SYS_MODE_WORK:    return MODE_STATE_WORK;
        default:               return MODE_STATE_INIT;
    }
}

/* ============================================================
 * 内部辅助：切换模式并广播
 * ============================================================ */
static void mode_switch_to(mode_context_t *mctx, sys_mode_t mode)
{
    SM_ASSERT_PTR(mctx);

    /* 更新全局变量 */
    sys_set_mode(mode);

    /* 【2026-10-27】模式变了，立即重算能力掩码。
     * 必须放在 sys_set_mode() 之后：sys_caps_apply() 内部要读 sys_get_mode()。
     * 这是 EVT_MODE_CHANGED 之外的第二条生效路径 —— 即使事件队列满导致
     * 广播丢失，能力掩码也已经同步更新，不存在"模式变了权限没变"的窗口。 */
    sys_caps_apply();

    /* 更新上下文 */
    mctx->current_mode = mode_to_state(mode);
    mctx->mode_enter_tick = rt_tick_get();

    /* 更新统计 */
    switch (mode) {
        case SYS_MODE_FACTORY: mctx->factory_enter_count++; break;
        case SYS_MODE_TEST:    mctx->test_enter_count++;    break;
        case SYS_MODE_WORK:    mctx->work_enter_count++;    break;
        default: break;
    }

    /* 广播模式变更事件（核心机制：其他控制对象自主响应） */
    post_event(EVT_MODE_CHANGED);

    LOG_STATE("[Mode] Switched to %d\n", mode);
}

/* ============================================================
 * Enter 回调
 * ============================================================ */
void mode_enter_init(StateContext_t ctx)
{
    mode_context_t *mctx = (mode_context_t*)ctx;
    SM_ASSERT_PTR(mctx);

    mctx->current_mode = MODE_STATE_INIT;
    mctx->mode_enter_tick = rt_tick_get();

    LOG_STATE("[Mode] Enter INIT\n");
}

void mode_enter_factory(StateContext_t ctx)
{
    mode_context_t *mctx = (mode_context_t*)ctx;
    mode_switch_to(mctx, SYS_MODE_FACTORY);
}

void mode_enter_test(StateContext_t ctx)
{
    mode_context_t *mctx = (mode_context_t*)ctx;
    mode_switch_to(mctx, SYS_MODE_TEST);
}

void mode_enter_work(StateContext_t ctx)
{
    mode_context_t *mctx = (mode_context_t*)ctx;
    mode_switch_to(mctx, SYS_MODE_WORK);
}

/* ============================================================
 * Exit 回调
 * ============================================================ */
void mode_exit_init(StateContext_t ctx)
{
    /* 无操作 */
    (void)ctx;
}

void mode_exit_factory(StateContext_t ctx)
{
    (void)ctx;
}

void mode_exit_test(StateContext_t ctx)
{
    (void)ctx;
}

void mode_exit_work(StateContext_t ctx)
{
    (void)ctx;
}

/* ============================================================
 * 50ms Tick 检查（测试模式超时检测）
 * ============================================================ */
void mode_tick_check(StateContext_t ctx)
{
    mode_context_t *mctx = (mode_context_t*)ctx;
    uint32_t elapsed_ms;

    SM_ASSERT_PTR(mctx);

    /* 仅在测试模式下检查超时 */
    if (mctx->current_mode != MODE_STATE_TEST) {
        return;
    }

    /* 如果配置为不超时，跳过 */
    if (mctx->config.test_timeout_ms == 0) {
        return;
    }

    /* 计算已停留时间 */
    elapsed_ms = (rt_tick_get() - mctx->mode_enter_tick)
                 * 1000 / RT_TICK_PER_SECOND;

    /* 超时检测 */
    if (elapsed_ms >= mctx->config.test_timeout_ms) {
        LOG_WARN("[Mode] Test mode timeout (%ums), exiting...\n", elapsed_ms);
        post_event(MODE_EVT_TEST_TIMEOUT);
    }
}
