/**
 * @file    arbiter.c
 * @brief   业务事件仲裁器实现（P1：影子骨架）
 * @note    零 RTOS 依赖：不 include rtthread.h，不调用 rt_tick_get。
 *          时间戳由调用方在 arb_req_t.tick 填入。
 */

#include "arbiter.h"

/* ============================================================
 * 影子模式下的规则
 * ------------------------------------------------------------
 * P1 只判 GRANT：优先级表尚未启用，任何仲裁类事件都不拦。
 * 真正价值在"对账"——把 GRANT 与 app_main 实际是否投递比对：
 *   GRANT + 未投递  → 链路断（描述表漏行 / 路线缺失 / 门禁拒绝）
 *   非 GRANT + 投递 → 旁路（P2 起才会出现）
 * ============================================================ */

static arb_mode_t   s_mode = ARB_MODE_SHADOW;
static arb_module_t s_owner = ARB_MOD_NONE;

static arb_stat_t s_stat;

static arb_log_t s_last;
static uint8_t   s_await;        /* 1 = 有决策等待 actual 对账 */

static arb_log_t s_ring[ARB_LOG_MAX];
static uint8_t   s_ring_w;
static uint8_t   s_ring_cnt;

/* ============================================================
 * 分类
 * ============================================================ */
arb_class_t arb_classify(app_event_t evt, arb_module_t *module)
{
    if (module != 0) {
        *module = ARB_MOD_NONE;
    }

    switch (evt) {
    /* ---- 停止类：优先级最高，任何情况下都要执行 ---- */
    case SYS_EVT_ACC_OFF:
    case SYS_EVT_UV_ENTER:
        if (module != 0) {
            *module = ARB_MOD_NONE;
        }
        return ARB_CLS_STOP_ALL;

    /* ---- 手动四向「按下」---- */
    case LIFT_EVT_UP_DOWN:
    case LIFT_EVT_DOWN_DOWN:
        if (module != 0) {
            *module = ARB_MOD_LIFT;
        }
        return ARB_CLS_MANUAL;

    case SWING_EVT_LEFT_DOWN:
    case SWING_EVT_RIGHT_DOWN:
        if (module != 0) {
            *module = ARB_MOD_SWING;
        }
        return ARB_CLS_MANUAL;

    /* ---- 一键（SWING / SEQ 两套枚举）---- */
    case SWING_EVT_ONKEY_MIDDLE:
    case SWING_EVT_ONKEY_REAR:
    case SWING_EVT_ONKEY_HOME:
    case SEQ_EVT_ONKEY_MIDDLE:
    case SEQ_EVT_ONKEY_REAR:
    case SEQ_EVT_ONKEY_HOME:
        if (module != 0) {
            *module = ARB_MOD_SEQ;
        }
        return ARB_CLS_ONEKEY;

    /* ---- 校准 ---- */
    case CALIB_EVT_CLICK:
    case CALIB_EVT_DOUBLE_CLICK:
    case CALIB_EVT_LONG_PRESS:
    case CALIB_EVT_KEY_UP:
        if (module != 0) {
            *module = ARB_MOD_CALIB;
        }
        return ARB_CLS_CALIB;

    /* ---- 离合 ---- */
    case CLUTCH_EVT_COMBINE:
    case CLUTCH_EVT_SEPARATE:
        if (module != 0) {
            *module = ARB_MOD_CLUTCH;
        }
        return ARB_CLS_CLUTCH;

    /* ---- 其余全部放行：松开、限位、故障、序列推进、状态刷新 ---- */
    default:
        return ARB_CLS_PASS;
    }
}

/* ============================================================
 * 提交仲裁
 * ============================================================ */
arb_result_t arb_submit(const arb_req_t *req, const arb_snap_t *snap)
{
    arb_result_t res;

    res.dec    = ARB_DEC_GRANT;
    res.reason = "P1-PASS";

    if ((req == 0) || (snap == 0)) {
        res.reason = "BAD-REQ";
        return res;
    }

    if (req->cls == ARB_CLS_PASS) {
        /* 不需要仲裁的事件：不计数、不记日志、不等待对账 */
        s_await = 0U;
        return res;
    }

    if (req->cls == ARB_CLS_STOP_ALL) {
        res.reason = "STOP";
    }

    s_last.event        = req->event;
    s_last.cls          = req->cls;
    s_last.module       = req->module;
    s_last.dec          = res.dec;
    s_last.owner_before = s_owner;
    s_last.delivered    = 0U;
    s_last.tick         = req->tick;
    s_await             = 1U;

    s_stat.submitted++;
    s_stat.grant++;

    return res;
}

/* ============================================================
 * 影子对账
 * ============================================================ */
void arb_shadow_actual(uint8_t delivered)
{
    uint8_t d = (delivered != 0U) ? 1U : 0U;

    if (s_await == 0U) {
        return;
    }
    s_await = 0U;

    s_last.delivered = d;
    s_stat.checked++;

    if ((s_last.dec == ARB_DEC_GRANT) && (d == 0U)) {
        s_stat.mismatch_link++;
    } else if ((s_last.dec != ARB_DEC_GRANT) && (d != 0U)) {
        s_stat.mismatch_bypass++;
    }

    /* owner 只跟随"真实投递"变化，和影子决策分开，避免自证 */
    if (d != 0U) {
        if (s_last.cls == ARB_CLS_STOP_ALL) {
            s_owner = ARB_MOD_NONE;         /* 急停后无 owner */
        } else if (s_last.module != ARB_MOD_NONE) {
            s_owner = s_last.module;
        }
    }

    s_ring[s_ring_w] = s_last;
    s_ring_w = (uint8_t)((s_ring_w + 1U) % ARB_LOG_MAX);
    if (s_ring_cnt < ARB_LOG_MAX) {
        s_ring_cnt++;
    }
}

/* ============================================================
 * 查询 / 控制
 * ============================================================ */
arb_mode_t arb_get_mode(void)
{
    return s_mode;
}

int arb_set_mode(arb_mode_t mode)
{
    /* P1 只允许影子；enforce 在 P2 接入真实拦截后才开放 */
    if (mode != ARB_MODE_SHADOW) {
        return -1;
    }
    s_mode = ARB_MODE_SHADOW;
    return 0;
}

arb_module_t arb_get_owner(void)
{
    return s_owner;
}

/* 【P2 / 2026-11-23】释放 owner（模块回到 IDLE 时由 arb_owner_sync 调用）。
 * 只清"正好是它"的情况，避免把 B 的占用误清成 A。 */
void arb_release_owner(arb_module_t m)
{
    if ((m != ARB_MOD_NONE) && (s_owner == m)) {
        s_owner = ARB_MOD_NONE;
    }
}

void arb_get_stat(arb_stat_t *out)
{
    if (out == 0) {
        return;
    }
    *out = s_stat;
}

uint8_t arb_get_last(arb_log_t *out)
{
    if ((out == 0) || (s_stat.submitted == 0U)) {
        return 0U;
    }
    *out = s_last;
    return 1U;
}

uint8_t arb_get_log(arb_log_t *out, uint8_t max)
{
    uint8_t i;
    uint8_t start;

    if ((out == 0) || (max == 0U)) {
        return 0U;
    }
    if (s_ring_cnt == 0U) {
        return 0U;
    }

    if (max > s_ring_cnt) {
        max = s_ring_cnt;
    }

    /* 只拷贝最新的 max 条，按旧→新输出 */
    start = (uint8_t)((s_ring_w + ARB_LOG_MAX - max) % ARB_LOG_MAX);
    for (i = 0U; i < max; i++) {
        out[i] = s_ring[(start + i) % ARB_LOG_MAX];
    }
    return max;
}

void arb_reset_stats(void)
{
    uint32_t i;

    s_stat.submitted     = 0U;
    s_stat.grant         = 0U;
    s_stat.reject        = 0U;
    s_stat.preempt       = 0U;
    s_stat.mismatch_link = 0U;
    s_stat.mismatch_bypass = 0U;
    s_stat.checked       = 0U;

    s_ring_w   = 0U;
    s_ring_cnt = 0U;
    s_await    = 0U;

    for (i = 0U; i < ARB_LOG_MAX; i++) {
        s_ring[i].event        = 0U;
        s_ring[i].cls          = ARB_CLS_NONE;
        s_ring[i].module       = ARB_MOD_NONE;
        s_ring[i].dec          = ARB_DEC_GRANT;
        s_ring[i].owner_before = ARB_MOD_NONE;
        s_ring[i].delivered    = 0U;
        s_ring[i].tick         = 0U;
    }
}
