/**
 * @file    keyboard_sm.c
 * @brief   L3 键盘指令状态机实现
 */

#include "keyboard_sm.h"
#include "keyboard_scan.h"
#include "keyboard_keymap.h"

#include "../app_event.h"
#include "../lift/lift_control.h"
#include "../swing/swing_control.h"
#include "../sequence/seq_control.h"

#include <string.h>
#include <rtthread.h>
#include <finsh.h>

/* ============================================================
 * 配置
 * ============================================================ */
#define KB_THREAD_PRIO      7
/* 【2026-10-27 · D-59】原 1536 → 按实测重配为 1280。
 *
 * 实测（展开 ELF 调用图，-O0）：
 *   正常路径          488 B
 *   含断言/打印路径   640 B   ← 修复 D-61 前是 1132 B（std assert 链）
 *   叠加切换开销      ~240 B  (硬件压栈104 + 切帧36 + 对齐余量)
 *   → 峰值约 880 B
 *
 * 取 1280：留 1.45 倍余量。
 *
 * 说明：最初按"880 够用"想给 1024，验证脚本算出 880/1024 = 86%，
 * 判定 TIGHT，故上调。keyboard 是唯一在 50ms 内要连续跑完
 * 「按键扫描 → 去抖 → 事件投递 → 控制同步」全链的线程，
 * 且 _kb_sync_context 经函数指针进入 seq_control_update_onekey
 * 等业务模块，不宜压到极限。 */
#define KB_THREAD_STACK     1280U
#define KB_THREAD_TIMESLICE 10

/* ============================================================
 * 模块状态
 * ============================================================ */
static kb_snapshot_t s_snap;
static kb_sm_t       s_sm;

static rt_timer_t  s_tmr  = RT_NULL;
static rt_sem_t    s_wake = RT_NULL;
static rt_thread_t s_tid  = RT_NULL;

/* 「重新按下」请求（跨线程标志） */
static volatile uint8_t s_rebaseline_req = 0U;

/* 上次掩码（用于灯帧刷新通知） */
static uint16_t s_prev_mask = 0U;

void kb_request_rebaseline(void)
{
    s_rebaseline_req = 1U;
    if (s_wake != RT_NULL) {
        rt_sem_release(s_wake);
    }
}

/* ============================================================
 * 超时回调（timer 线程：禁止阻塞与耗时打印）
 * ============================================================ */
static void _kb_timeout_cb(void *param)
{
    (void)param;

    s_snap.mask  = 0U;
    s_snap.valid = 0U;
    s_sm.cnt_timeout++;

    if (s_wake != RT_NULL) {
        rt_sem_release(s_wake);     /* 唤醒键盘线程处理"全释放" */
    }
}

/* ============================================================
 * CAN 帧入口（can_parser 线程）
 * ============================================================ */
void kb_on_can_frame(const uint8_t *data, uint8_t len)
{
    uint16_t mask;

    if (s_tmr == RT_NULL) {
        return;                     /* 未初始化 */
    }

    mask = kb_scan_parse(data, len);

    /* ① 先停定时器：防止已到期的旧超时覆盖新数据 */
    rt_timer_stop(s_tmr);

    /* ② 更新快照（16/32 位对齐写，单写单读无撕裂） */
    s_snap.mask    = mask;
    s_snap.rx_tick = rt_tick_get();
    s_snap.valid   = 1U;
    s_sm.cnt_frames++;

    /* ③ 按键变化 → 通知灯帧刷新（0x18FFA4F0） */
    if (mask != s_prev_mask) {
        s_prev_mask = mask;
        post_event(CAN_EVT_LED_KEY);
    }

    /* ④ 重启 300ms 一次性定时器 */
    rt_timer_start(s_tmr);

    /* ⑤ 唤醒键盘线程 */
    rt_sem_release(s_wake);
}

/* ============================================================
 * 单轴互斥消解
 * ============================================================
 * 【输入】该轴本帧两个方向位（0/1）与上帧记忆
 * 【输出】该轴有效方向位（互斥时两路都为 0 = 视为没按下）
 *
 * 【两阶段判定】
 *   阶段① 方式：是否存在冲突
 *           同帧双按（pos && neg）         → 冲突
 *           前后帧反向（上帧 pos，本帧 neg） → 需再过阶段②
 *   阶段② 条件：
 *           同帧双按                       → 无条件成立
 *           前后帧反向且 Δt ≤ 300ms        → 成立
 *           前后帧反向且 Δt >  300ms       → 视为正常换向，不冲突
 *
 * 【解除条件（四选一）】
 *   1. 从双按变单按（1 1 → 1 0）        → 立刻解除，剩下方向立即生效
 *   2. 单方向与上一帧相同（0 1 → 0 1）  → 解除，该方向生效（交替上报 100ms 后恢复）
 *   3. 全松开（0 0）                     → 解除，无输出
 *   4. CAN 报文丢失                      → 由 _kb_sm_step 清空记忆（见该函数）
 * ============================================================ */
static void _kb_axis_filter(kb_axis_mem_t *m,
                            uint8_t pos, uint8_t neg,
                            rt_tick_t now,
                            uint8_t *out_pos, uint8_t *out_neg)
{
    *out_pos = 0U;
    *out_neg = 0U;

    /* ---------- 阶段① / ②：判定冲突是否成立 ---------- */
    if ((pos != 0U) && (neg != 0U)) {
        /* 同帧双按：无条件成立 */
        m->conflict = 1U;
    } else if ((pos == 0U) && (neg == 0U)) {
        /* 全松开：解除 */
        m->conflict = 0U;
    } else if (m->conflict == 0U) {
        /* 单方向且当前无冲突：检查是否"前后帧反向 + 窗口内" */
        uint8_t prev_dir = (m->last_pos != 0U) ? 1U : ((m->last_neg != 0U) ? 2U : 0U);
        uint8_t cur_dir  = (pos != 0U) ? 1U : 2U;

        if ((prev_dir != 0U) && (cur_dir != prev_dir) &&
            ((now - m->change_tick) <= rt_tick_from_millisecond(KB_MUTEX_WINDOW_MS))) {
            m->conflict = 1U;
        }
    }

    /* ---------- 冲突中：判定是否解除 ---------- */
    if (m->conflict != 0U) {
        if ((pos != 0U) && (neg != 0U)) {
            /* 仍双按 → 保持冲突，两者视为没按 */
        } else if ((pos == 0U) && (neg == 0U)) {
            m->conflict = 0U;
        } else if ((m->last_pos != 0U) && (m->last_neg != 0U)) {
            /* 双按 → 单按：立刻解除，剩下的方向立即生效 */
            m->conflict = 0U;
            *out_pos = pos;
            *out_neg = neg;
        } else if ((pos == m->last_pos) && (neg == m->last_neg)) {
            /* 单方向与上帧一致 → 已稳定，解除并生效 */
            m->conflict = 0U;
            *out_pos = pos;
            *out_neg = neg;
        } else {
            /* 冲突中又换向 → 继续视为没按 */
        }
    } else {
        *out_pos = pos;
        *out_neg = neg;
    }

    /* ---------- 更新记忆 ---------- */
    if ((pos != m->last_pos) || (neg != m->last_neg)) {
        m->change_tick = now;
    }
    m->last_pos = pos;
    m->last_neg = neg;
}

/* ============================================================
 * 有效方向位 → 指令
 * ============================================================
 * 方向位直接来自消解后的结果，不再按 key_code 逐个枚举；
 * key_code 仅用于调试显示（t_key）。
 * ============================================================ */
static void _kb_build_cmd(uint16_t raw_mask,
                          uint8_t up, uint8_t down,
                          uint8_t left, uint8_t right,
                          key_cmd_t *cmd)
{
    memset(cmd, 0, sizeof(*cmd));

    cmd->up    = up;
    cmd->down  = down;
    cmd->left  = left;
    cmd->right = right;

    cmd->calib = ((raw_mask & KB_CALIB) != 0U) ? 1U : 0U;

    /* 一键：只要原始报文里存在手动方向，一键一律拒绝。
     * 用原始报文而非消解结果判断，是为了覆盖「手动互斥被判无效」的情形——
     * 否则上+下冲突瞬间方向位全为 0，一键会趁机动作，与「手动优先」相悖。 */
    if ((raw_mask & KB_MANUAL_MASK) == 0U) {
        cmd->onekey_mid  = ((raw_mask & KB_ONKEY_MID)  != 0U) ? 1U : 0U;
        cmd->onekey_rear = ((raw_mask & KB_ONKEY_REAR) != 0U) ? 1U : 0U;
        cmd->onekey_home = ((raw_mask & KB_ONKEY_HOME) != 0U) ? 1U : 0U;
    }
}

/* ============================================================
 * 指令跳变 → 业务事件
 * ============================================================ */
static void _kb_emit_events(const key_cmd_t *old, const key_cmd_t *now)
{
    /* 举升 */
    if ((now->up   != 0U) && (old->up   == 0U)) { post_event(LIFT_EVT_UP_DOWN);     }
    if ((now->up   == 0U) && (old->up   != 0U)) { post_event(LIFT_EVT_UP_UP);       }
    if ((now->down != 0U) && (old->down == 0U)) { post_event(LIFT_EVT_DOWN_DOWN);   }
    if ((now->down == 0U) && (old->down != 0U)) { post_event(LIFT_EVT_DOWN_UP);     }

    /* 回转 */
    if ((now->left  != 0U) && (old->left  == 0U)) { post_event(SWING_EVT_LEFT_DOWN);  }
    if ((now->left  == 0U) && (old->left  != 0U)) { post_event(SWING_EVT_LEFT_UP);    }
    if ((now->right != 0U) && (old->right == 0U)) { post_event(SWING_EVT_RIGHT_DOWN); }
    if ((now->right == 0U) && (old->right != 0U)) { post_event(SWING_EVT_RIGHT_UP);   }

    /* 一键（触发型，只发按下） */
    if ((now->onekey_mid  != 0U) && (old->onekey_mid  == 0U)) { post_event(SWING_EVT_ONKEY_MIDDLE); }
    if ((now->onekey_rear != 0U) && (old->onekey_rear == 0U)) { post_event(SWING_EVT_ONKEY_REAR);   }
    if ((now->onekey_home != 0U) && (old->onekey_home == 0U)) { post_event(SWING_EVT_ONKEY_HOME);   }

    /* 校准 */
    if ((now->calib != 0U) && (old->calib == 0U)) { post_event(CALIB_EVT_CLICK);  }
    if ((now->calib == 0U) && (old->calib != 0U)) { post_event(CALIB_EVT_KEY_UP); }
}

/* ============================================================
 * 上下文同步（互锁标志 / 序列打断依据）
 * ============================================================ */
static void _kb_sync_context(const key_cmd_t *cmd)
{
    uint8_t up    = cmd->up;
    uint8_t down  = cmd->down;
    uint8_t left  = cmd->left;
    uint8_t right = cmd->right;

    if (cmd->blocked != 0U) {
        /* 锁死时对外呈现"全部松开"，让业务层停止动作 */
        up = down = left = right = 0U;
    }

    lift_control_update_switch(up, down);
    swing_control_update_switch(left, right);
    seq_control_update_onekey(0U, 0U, 0U);      /* 一键由事件驱动，标志无需保持 */
}

/* ============================================================
 * 状态机推进
 * ============================================================ */
static void _kb_sm_step(void)
{
    uint16_t   mask = s_snap.mask;
    uint8_t    up = 0U, down = 0U, left = 0U, right = 0U;
    key_cmd_t  new_cmd;
    key_code_t kc;

    kc = kb_keymap_resolve(mask);

    /* ---------- 启车重按锁死（rebaseline）：必须全部松开才解除 ---------- */
    if (s_sm.state == KSM_BLOCKED) {
        if (mask == 0U) {
            s_sm.state = KSM_IDLE;
            memset(&s_sm.lift_mem, 0, sizeof(s_sm.lift_mem));
            memset(&s_sm.swing_mem,  0, sizeof(s_sm.swing_mem));
        } else {
            memset(&new_cmd, 0, sizeof(new_cmd));
            new_cmd.blocked = 1U;

            s_sm.cur_key = kc;
            s_sm.cmd     = new_cmd;
            _kb_emit_events(&s_sm.prev_cmd, &s_sm.cmd);
            _kb_sync_context(&s_sm.cmd);
            s_sm.prev_cmd = s_sm.cmd;
            return;
        }
    }

    /* ---------- CAN 报文丢失：解除全部冲突与历史记忆 ----------
     * 若不清理 last_pos/last_neg，报文恢复后首帧可能被误判为
     * 「前后帧反向」，导致动作被再次锁住。
     * 注：清空后 mask 仍为 0，故本轮指令为全 0，跳变检测会发出
     *     对应的 *_UP 事件把阀/电机停掉。 */
    if (s_snap.valid == 0U) {
        memset(&s_sm.lift_mem, 0, sizeof(s_sm.lift_mem));
        memset(&s_sm.swing_mem,  0, sizeof(s_sm.swing_mem));
    }

    /* ---------- 逐轴互斥消解（两轴独立，异轴组合不受影响）---------- */
    {
        uint8_t before_lift = s_sm.lift_mem.conflict;
        uint8_t before_swing  = s_sm.swing_mem.conflict;

        _kb_axis_filter(&s_sm.lift_mem,
                        ((mask & KB_UP)   != 0U) ? 1U : 0U,
                        ((mask & KB_DOWN) != 0U) ? 1U : 0U,
                        s_snap.rx_tick, &up, &down);

        _kb_axis_filter(&s_sm.swing_mem,
                        ((mask & KB_LEFT)  != 0U) ? 1U : 0U,
                        ((mask & KB_RIGHT) != 0U) ? 1U : 0U,
                        s_snap.rx_tick, &left, &right);

        if ((before_lift == 0U) && (s_sm.lift_mem.conflict != 0U)) {
            s_sm.cnt_conflict++;
        }
        if ((before_swing == 0U) && (s_sm.swing_mem.conflict != 0U)) {
            s_sm.cnt_conflict++;
        }
    }

    /* ---------- 生成指令 ---------- */
    _kb_build_cmd(mask, up, down, left, right, &new_cmd);

    /* 手动优先：报文中有手动方向且同时有一键 → 一键被拒 */
    if (((mask & KB_MANUAL_MASK) != 0U) && ((mask & KB_ONEKEY_MASK) != 0U)) {
        s_sm.cnt_reject++;
    }

    s_sm.state   = (key_cmd_is_empty(&new_cmd) != 0U) ? KSM_IDLE : KSM_ACTIVE;
    s_sm.cur_key = kc;
    s_sm.cmd     = new_cmd;

    _kb_emit_events(&s_sm.prev_cmd, &s_sm.cmd);
    _kb_sync_context(&s_sm.cmd);

    s_sm.prev_cmd = s_sm.cmd;
}

/* ============================================================
 * 线程
 * ============================================================ */
static void _kb_thread_entry(void *param)
{
    (void)param;

    while (1) {
        if (rt_sem_take(s_wake, RT_WAITING_FOREVER) == RT_EOK) {

            /* 处理「重新按下」请求（启车场景） */
            if (s_rebaseline_req != 0U) {
                s_rebaseline_req = 0U;

                if (s_snap.mask != 0U) {
                    /* 有键正按着 → 锁死，等全部松开后才恢复 */
                    s_sm.state = KSM_BLOCKED;
                    s_sm.cnt_blocked++;
                    memset(&s_sm.cmd, 0, sizeof(s_sm.cmd));
                    s_sm.cmd.blocked = 1U;

                    _kb_emit_events(&s_sm.prev_cmd, &s_sm.cmd);
                    _kb_sync_context(&s_sm.cmd);
                    s_sm.prev_cmd = s_sm.cmd;

                    rt_kprintf("[KB] rebaseline: keys held -> BLOCKED\n");
                    continue;
                }
            }

            _kb_sm_step();
        }
    }
}

/* ============================================================
 * 初始化
 * ============================================================ */
int kb_init(void)
{
    if (s_tmr != RT_NULL) {
        rt_kprintf("[KB] already initialized\n");
        return 0;
    }

    memset(&s_sm, 0, sizeof(s_sm));
    memset(&s_snap, 0, sizeof(s_snap));
    s_sm.state = KSM_IDLE;

    s_wake = rt_sem_create("kb", 0, RT_IPC_FLAG_PRIO);
    if (s_wake == RT_NULL) {
        rt_kprintf("[KB] sem create failed\n");
        return -1;
    }

    /* 一次性定时器：不设 PERIODIC 即为 one-shot */
    s_tmr = rt_timer_create("kb_to", _kb_timeout_cb, RT_NULL,
                            rt_tick_from_millisecond(KB_RX_TIMEOUT_MS),
                            RT_TIMER_FLAG_SOFT_TIMER);
    if (s_tmr == RT_NULL) {
        rt_kprintf("[KB] timer create failed\n");
        return -1;
    }

    s_tid = rt_thread_create("keyboard", _kb_thread_entry, RT_NULL,
                             KB_THREAD_STACK, KB_THREAD_PRIO, KB_THREAD_TIMESLICE);
    if (s_tid == RT_NULL) {
        rt_kprintf("[KB] thread create failed\n");
        return -1;
    }
    rt_thread_startup(s_tid);

    rt_kprintf("[KB] started: timeout=%ums, keymap=%u entries\n",
               (unsigned)KB_RX_TIMEOUT_MS, (unsigned)kb_keymap_entry_count());
    return 0;
}

/* ============================================================
 * 查询 / 调试
 * ============================================================ */
const key_cmd_t *kb_get_cmd(void)
{
    return &s_sm.cmd;
}

ksm_state_t kb_get_state(void)
{
    return s_sm.state;
}

const char *kb_sm_state_name(ksm_state_t s)
{
    switch (s) {
    case KSM_IDLE:    return "IDLE";
    case KSM_ACTIVE:  return "ACTIVE";
    case KSM_BLOCKED: return "BLOCKED";
    default:          return "?";
    }
}

static void t_key(void)
{
    const key_cmd_t *c = &s_sm.cmd;

    rt_kprintf("\n========== KEYBOARD ==========\n");
    rt_kprintf("state     : %s\n", kb_sm_state_name(s_sm.state));
    rt_kprintf("mask      : 0x%03X  (%s)\n",
               (unsigned)s_snap.mask, kb_scan_mask_str(s_snap.mask));
    rt_kprintf("key_code  : %s\n", kb_key_code_name(s_sm.cur_key));
    rt_kprintf("axis      : lift[last %d%d cf%d]  swing[last %d%d cf%d]\n",
               s_sm.lift_mem.last_pos, s_sm.lift_mem.last_neg,
               s_sm.lift_mem.conflict,
               s_sm.swing_mem.last_pos,  s_sm.swing_mem.last_neg,
               s_sm.swing_mem.conflict);
    rt_kprintf("cmd       : U%d D%d L%d R%d | OK[m%d b%d h%d] | calib%d blocked%d\n",
               c->up, c->down, c->left, c->right,
               c->onekey_mid, c->onekey_rear, c->onekey_home,
               c->calib, c->blocked);
    rt_kprintf("valid     : %u   (rx %ums ago)\n",
               (unsigned)s_snap.valid,
               (unsigned)((rt_tick_get() - s_snap.rx_tick) * 1000 / RT_TICK_PER_SECOND));
    rt_kprintf("stats     : frames=%u conflict=%u blocked=%u reject=%u timeout=%u\n",
               (unsigned)s_sm.cnt_frames, (unsigned)s_sm.cnt_conflict,
               (unsigned)s_sm.cnt_blocked, (unsigned)s_sm.cnt_reject,
               (unsigned)s_sm.cnt_timeout);
    rt_kprintf("window    : %ums  (rx timeout %ums)\n",
               (unsigned)KB_MUTEX_WINDOW_MS, (unsigned)KB_RX_TIMEOUT_MS);
    rt_kprintf("==============================\n\n");
}

MSH_CMD_EXPORT_ALIAS(t_key, t_key, dump keyboard state machine);
