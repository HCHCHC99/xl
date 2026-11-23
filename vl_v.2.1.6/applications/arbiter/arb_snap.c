/**
 * @file    arb_snap.c
 * @brief   仲裁状态快照 —— 唯一接触各模块 getter 的文件
 * @note    仲裁规则（arbiter.c）只看 arb_snap_t，不直接 include 业务模块，
 *          避免仲裁层与业务层相互依赖。
 *
 *          P1 只采集已有公开 getter：
 *            acc / 启车 / 欠压 / 整车故障字 / 序列是否活动 / 观察到的 owner
 *          故障位、角度速度等字段留到对应阶段再接入。
 */

#include "arbiter.h"

#include <rtthread.h>

#include "../service/key_state.h"          /* acc_is_on() */
#include "../service/sys_voltage.h"        /* sys_voltage_is_uv() */
#include "../vehicle/vehicle_control.h"    /* vehicle_is_started() / fault word */
#include "../sequence/seq_control.h"       /* seq_control_get_context() */
#include "../lift/lift_control.h"          /* g_lift_obj / obj_base_get_state */
#include "../swing/swing_control.h"        /* g_swing_obj */
#include "../swing/swing_angle.h"          /* swing_angle_is_solve_valid() */

void arb_snap_fill(arb_snap_t *snap)
{
    const seq_ctrl_context_t *sctx;

    if (snap == RT_NULL) {
        return;
    }

    snap->acc_on          = (acc_is_on() != 0U) ? 1U : 0U;
    snap->vehicle_started = (vehicle_is_started() != 0U) ? 1U : 0U;
    snap->uv              = (sys_voltage_is_uv() != 0U) ? 1U : 0U;
    snap->fault_word      = vehicle_fault_word_get();

    sctx = seq_control_get_context();
    snap->seq_active = ((sctx != RT_NULL) &&
                        (seq_engine_is_active(&sctx->engine) != 0U)) ? 1U : 0U;

    /* 【P2】回转角解算有效性：原来 dump 里 ang_v/ref_v 是硬编码 0。
     * ang_v 现在读真实 getter；ref_v 属"筒角参考有效"，批次 4 才实现，恒 0。 */
    snap->angle_valid = (swing_angle_is_solve_valid() != 0U) ? 1U : 0U;
    snap->ref_valid   = 0U;

    snap->owner = arb_get_owner();
    snap->tick  = rt_tick_get();
}

/* ============================================================
 * 【P2 / 2026-11-23】owner 释放
 * ------------------------------------------------------------
 * owner 目前是"最后一条被投递的仲裁类事件所属模块"，模块回到 IDLE 后
 * 必须让出，否则 P3 开 enforce 会一直判"占用中"。
 * 只做释放，不做抢占；CALIB/CLUTCH 尚未建模，先跳过。
 * ============================================================ */
void arb_owner_sync(void)
{
    arb_module_t      own = arb_get_owner();
    const control_object_t *obj = RT_NULL;

    switch (own) {
        case ARB_MOD_LIFT:  obj = &g_lift_obj.base;  break;
        case ARB_MOD_SWING: obj = &g_swing_obj.base; break;
        case ARB_MOD_SEQ:   obj = &g_seq_obj.base;   break;
        default:
            return;         /* NONE / CALIB / CLUTCH：无需处理 */
    }

    /* lift / swing / seq 三个对象的状态枚举都以 0 = IDLE 开头 */
    if (obj_base_get_state(obj) == 0U) {
        arb_release_owner(own);
    }
}

void arb_shadow_probe(app_event_t evt)
{
    arb_req_t  req;
    arb_snap_t snap;

    req.event  = evt;
    req.cls    = arb_classify(evt, &req.module);
    req.tick   = rt_tick_get();

    arb_snap_fill(&snap);

    (void)arb_submit(&req, &snap);
}
