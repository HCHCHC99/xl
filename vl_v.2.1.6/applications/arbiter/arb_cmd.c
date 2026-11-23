/**
 * @file    arb_cmd.c
 * @brief   仲裁 / 事件 / 状态机调试命令（P1）
 * @note    本文件只做"读状态 + 打印"，不参与任何业务决策。
 *          命令：t_arb、t_evt、t_sm_dump
 */

#include <rtthread.h>
#include <finsh.h>
#include <string.h>

#include "arbiter.h"
#include "../ipc_center.h"
#include "../framework/state_engine/state_engine.h"

#include "../lift/lift_control.h"
#include "../swing/swing_control.h"
#include "../sequence/seq_control.h"
#include "../vehicle/vehicle_control.h"
#include "../mode/mode_control.h"

/* ============================================================
 * 名称表
 * ============================================================ */
static const char *const s_cls_names[] = {
    "NONE", "STOP_ALL", "CLUTCH", "MANUAL", "ONEKEY", "CALIB", "PASS"
};

static const char *const s_mod_names[] = {
    "NONE", "LIFT", "SWING", "SEQ", "CALIB", "CLUTCH"
};

static const char *name_of_class(arb_class_t c)
{
    if ((uint32_t)c >= (sizeof(s_cls_names) / sizeof(s_cls_names[0]))) {
        return "?";
    }
    return s_cls_names[c];
}

static const char *name_of_module(arb_module_t m)
{
    if ((uint32_t)m >= (sizeof(s_mod_names) / sizeof(s_mod_names[0]))) {
        return "?";
    }
    return s_mod_names[m];
}

static const char *name_of_decision(arb_decision_t d)
{
    switch (d) {
    case ARB_DEC_GRANT:   return "GRANT";
    case ARB_DEC_REJECT:  return "REJECT";
    case ARB_DEC_PREEMPT: return "PREEMPT";
    default:              return "-";
    }
}

/* ============================================================
 * t_arb —— 仲裁器影子状态
 * ============================================================ */
static void arb_dump(void)
{
    arb_stat_t st;
    arb_snap_t snap;
    arb_log_t  last;
    uint32_t   mismatch;

    arb_get_stat(&st);
    arb_snap_fill(&snap);
    mismatch = st.mismatch_link + st.mismatch_bypass;

    rt_kprintf("[ARB] mode=%s owner=%s pending=NONE\n",
               (arb_get_mode() == ARB_MODE_ENFORCE) ? "ENFORCE" : "SHADOW",
               name_of_module(arb_get_owner()));
    rt_kprintf("[ARB] prio STOP_ALL>CLUTCH>MANUAL>ONEKEY>CALIB\n");
    rt_kprintf("[ARB] snap acc=%u start=%u uv=%u fault=0x%lX ang_v=%u ref_v=%u\n",
               (unsigned)snap.acc_on, (unsigned)snap.vehicle_started,
               (unsigned)snap.uv, (unsigned long)snap.fault_word,
               (unsigned)snap.angle_valid, (unsigned)snap.ref_valid);
    rt_kprintf("[ARB] stat grant=%lu reject=%lu preempt=%lu mismatch=%lu\n",
               (unsigned long)st.grant, (unsigned long)st.reject,
               (unsigned long)st.preempt, (unsigned long)mismatch);
    rt_kprintf("[ARB] mism link=%lu bypass=%lu checked=%lu\n",
               (unsigned long)st.mismatch_link,
               (unsigned long)st.mismatch_bypass,
               (unsigned long)st.checked);

    if (arb_get_last(&last) != 0U) {
        rt_kprintf("[ARB] last evt=0x%04X cls=%s mod=%s dec=%s del=%u reason=OK\n",
                   (unsigned)last.event,
                   name_of_class(last.cls),
                   name_of_module(last.module),
                   name_of_decision(last.dec),
                   (unsigned)last.delivered);
    } else {
        rt_kprintf("[ARB] last evt=0x0000 cls=NONE mod=NONE dec=- reason=OK\n");
    }
}

static void arb_log_dump(void)
{
    arb_log_t log[ARB_LOG_MAX];
    uint8_t   n;
    uint8_t   i;

    n = arb_get_log(log, ARB_LOG_MAX);
    if (n == 0U) {
        rt_kprintf("[ARB] log empty\n");
        return;
    }

    for (i = 0U; i < n; i++) {
        rt_kprintf("[ARB] %u evt=0x%04X cls=%s mod=%s dec=%s del=%u own=%s\n",
                   (unsigned)(i + 1U),
                   (unsigned)log[i].event,
                   name_of_class(log[i].cls),
                   name_of_module(log[i].module),
                   name_of_decision(log[i].dec),
                   (unsigned)log[i].delivered,
                   name_of_module(log[i].owner_before));
    }
}

static void t_arb(int argc, char **argv)
{
    const char *sub = (argc >= 2) ? argv[1] : "dump";

    if (strcmp(sub, "dump") == 0) {
        arb_dump();
        return;
    }

    if (strcmp(sub, "log") == 0) {
        arb_log_dump();
        return;
    }

    if (strcmp(sub, "clear") == 0) {
        arb_reset_stats();
        rt_kprintf("[ARB] stats cleared\n");
        return;
    }

    if (strcmp(sub, "mode") == 0) {
        if (argc < 3) {
            rt_kprintf("[ARB] usage: t_arb mode shadow|enforce\n");
            return;
        }
        if (strcmp(argv[2], "shadow") == 0) {
            (void)arb_set_mode(ARB_MODE_SHADOW);
            rt_kprintf("[ARB] mode -> SHADOW\n");
            return;
        }
        if (strcmp(argv[2], "enforce") == 0) {
            if (arb_set_mode(ARB_MODE_ENFORCE) != 0) {
                rt_kprintf("[ARB] enforce not allowed in P1 (still SHADOW)\n");
            } else {
                rt_kprintf("[ARB] mode -> ENFORCE\n");
            }
            return;
        }
        rt_kprintf("[ARB] usage: t_arb mode shadow|enforce\n");
        return;
    }

    rt_kprintf("[ARB] usage: t_arb [dump|log|clear|mode shadow|enforce]\n");
}

MSH_CMD_EXPORT_ALIAS(t_arb, t_arb, arbitration shadow dump: dump|log|clear|mode);

/* ============================================================
 * t_evt —— 事件队列投递统计
 * ============================================================ */
static void t_evt(int argc, char **argv)
{
    app_event_stat_t st;
    rt_uint16_t used = 0U;
    rt_uint16_t max  = 0U;

    /* 【P2】t_evt clear：只清计数，不动队列（多轮脚本后重新取基线） */
    if ((argc >= 2) && (strcmp(argv[1], "clear") == 0)) {
        app_event_clear_stat();
        rt_kprintf("[EVT] counters cleared\n");
        return;
    }

    app_event_get_stat(&st);

    if (g_app_event_mq != RT_NULL) {
        used = g_app_event_mq->entry;
        max  = g_app_event_mq->max_msgs;
    }

    rt_kprintf("[EVT] used=%u/%u drops=%lu hi=%lu pend=0x%08lX\n",
               (unsigned)used, (unsigned)max,
               (unsigned long)st.dropped,
               (unsigned long)st.pend_count,
               (unsigned long)st.pend_mask);
    rt_kprintf("[EVT] post=%lu stop=%lu defer=%lu replay=%lu dup=%lu ovf=%lu\n",
               (unsigned long)st.posted, (unsigned long)st.stop_posted,
               (unsigned long)st.stop_deferred, (unsigned long)st.stop_replayed,
               (unsigned long)st.stop_dup, (unsigned long)st.stop_overflow);
}

MSH_CMD_EXPORT_ALIAS(t_evt, t_evt, event queue counters);

/* ============================================================
 * t_sm_dump —— 五个对象的状态机快照
 * ============================================================ */
static const char *const s_lift_states[] = {
    "IDLE", "UP", "DOWN", "HOLD", "VFAULT", "SFAULT"
};

static const char *const s_swing_states[] = {
    "IDLE", "UNLOCK", "LEFT", "RIGHT", "LOCK", "FAULT"
};

static const char *const s_mode_states[] = {
    "INIT", "FACTORY", "TEST", "WORK"
};

static const char *const s_seq_obj_states[] = {
    "IDLE", "ACTIVE"
};

static const char *const s_seq_eng_states[] = {
    "IDLE", "RUNNING", "HOLDING", "DONE", "ABORTED"
};

static void sm_dump_one(const char *tag, const StateMachine_t *sm,
                        const char *const *names, uint32_t name_cnt)
{
    uint32_t st   = (uint32_t)StateMachine_GetState(sm);
    uint32_t prev = (uint32_t)StateMachine_GetPreviousState(sm);
    const char *sn = (st < name_cnt) ? names[st] : "?";
    const char *pn = (prev < name_cnt) ? names[prev] : "?";

    rt_kprintf("[SM] %-7s %-12s state=%s(%lu) prev=%s(%lu) init=%u\n",
               tag,
               (sm->name != RT_NULL) ? sm->name : "-",
               sn, (unsigned long)st,
               pn, (unsigned long)prev,
               (unsigned)StateMachine_IsInitialized(sm));
}

static void t_sm_dump(int argc, char **argv)
{
    seq_ctrl_context_t *sctx;
    uint32_t            eng;

    (void)argc;
    (void)argv;

    sm_dump_one("lift", &g_lift_obj.sm_instance,
                s_lift_states, sizeof(s_lift_states) / sizeof(s_lift_states[0]));
    sm_dump_one("swing", &g_swing_obj.sm_instance,
                s_swing_states, sizeof(s_swing_states) / sizeof(s_swing_states[0]));

    sm_dump_one("seq", &g_seq_obj.sm_instance,
                s_seq_obj_states, sizeof(s_seq_obj_states) / sizeof(s_seq_obj_states[0]));
    sctx = seq_control_get_context();
    if (sctx != RT_NULL) {
        eng = (uint32_t)seq_engine_get_state(&sctx->engine);
        rt_kprintf("[SM] seq     engine=%s(%lu) step=%lu\n",
                   (eng < (sizeof(s_seq_eng_states) / sizeof(s_seq_eng_states[0])))
                       ? s_seq_eng_states[eng] : "?",
                   (unsigned long)eng,
                   (unsigned long)seq_engine_get_step(&sctx->engine));
    }

    sm_dump_one("mode", &g_mode_obj.sm_instance,
                s_mode_states, sizeof(s_mode_states) / sizeof(s_mode_states[0]));

    {
        uint32_t vst   = (uint32_t)StateMachine_GetState(&g_vehicle_obj.sm_instance);
        uint32_t vprev = (uint32_t)StateMachine_GetPreviousState(&g_vehicle_obj.sm_instance);

        rt_kprintf("[SM] %-7s %-12s state=%s(%lu) prev=%s(%lu) init=%u\n",
                   "vehicle",
                   (g_vehicle_obj.sm_instance.name != RT_NULL)
                       ? g_vehicle_obj.sm_instance.name : "-",
                   vehicle_state_name((vehicle_state_t)vst), (unsigned long)vst,
                   vehicle_state_name((vehicle_state_t)vprev), (unsigned long)vprev,
                   (unsigned)StateMachine_IsInitialized(&g_vehicle_obj.sm_instance));
    }
}

MSH_CMD_EXPORT_ALIAS(t_sm_dump, t_sm_dump, dump 5 control-object state machines);
