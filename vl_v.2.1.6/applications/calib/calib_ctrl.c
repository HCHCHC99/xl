/**
 * @file    calib_ctrl.c
 * @brief   零点校准状态机实现（P9 / B-10，2026-11-23）
 *
 * ============================================================
 * 状态流（TC-CAL-001~011）
 * ============================================================
 *   IDLE --双击--> PREP(黄5s) --再按(按住)--> ARMED(绿5Hz闪)
 *   ARMED --长按>=2s--> EXEC --对齐成功--> DONE(绿2s) --> IDLE
 *                            --对齐失败--> FAIL(红2s) --> IDLE
 *   ARMED --<2s 松开--> FAIL(红2s) --> IDLE
 *   PREP --5s 超时--> IDLE
 *
 * EXEC = Foc_Proc_Start(FOC_PROC_ENC_ALIGN)（即 focmode 3 两点式）。
 * 成功判据：流程结束且 g_motor_is_aligned==1（encoder_calib.c 在
 * moved==0 时会清 0）。
 *
 * 成功动作（选项 A，业务.md §13.9）：
 *   1. swing_angle_zero_here()      筒角置 0（RAM）
 *   2. sys_param_set_calibrated(1)  is_calibrated=1（同步写 AM 记录）
 *   3. calib_save_fast()            快区立即落盘：offset/dir/tube_x10=0、
 *                                   valid=bit0|bit1、calib_count++
 *      注意：依赖 uv_snapshot_save() 同批改为"保留 tube/bit1"，
 *      否则一次欠压就把筒角清零。
 *
 * 次数口径：按"进入 EXEC"计，每上电周期 2 次（TC-CAL-008/010）。
 * PREP 超时、<2s 松开不消耗次数。
 */

#include <rtthread.h>
#include <string.h>
#include <stdlib.h>

#include "calib_ctrl.h"
#include "app_event.h"
#include "../vehicle/vehicle_control.h"
#include "../sequence/seq_control.h"
#include "../sequence/seq_engine.h"
#include "../sys_param/sys_param.h"
#include "../param/param_store.h"
#include "../param/param_map.h"
#include "../motor/drv/hw_encoder.h"        /* g_enc_elec_offset / g_enc_elec_dir */
#include "../motor/foc/encoder_calib.h"     /* g_motor_is_aligned */
#include "../motor/foc/cfg_motor.h"         /* CFG_ENCODER_CPR / CFG_MOTOR_POLE_PAIRS */
#include "../motor/foc/proc/foc_proc.h"
#include "../swing/swing_angle.h"
#include "../swing/swing_control.h"
#include "../lift/lift_control.h"
#include "../mode/sys_caps.h"

/* ============================================================
 * 常量
 * ============================================================ */
#define CALIB_PREP_WINDOW_MS    5000U   /* 黄灯窗口（TC-CAL-002） */
#define CALIB_EXEC_TIMEOUT_MS   10000U  /* 对齐执行兜底超时 */
#define CALIB_RESULT_HOLD_MS    2000U   /* 成功绿 / 失败红 的停留时间 */
#define CALIB_MAX_PER_BOOT      2U      /* 每上电周期次数上限（TC-CAL-008/010） */

/* ============================================================
 * 内部状态
 * ============================================================ */
static calib_state_t s_state = CALIB_ST_IDLE;
static uint8_t  s_count = 0U;           /* 本上电周期已用次数（进 EXEC 计） */
static uint32_t s_prep_ms = 0U;
static uint32_t s_exec_ms = 0U;
static uint32_t s_hold_ms = 0U;
static uint8_t  s_long_seen = 0U;       /* ARMED 中是否已收到长按 */
static const char *s_reason = "-";

/* ============================================================
 * 前置条件（TC-CAL-001）
 * @return RT_NULL = 通过；否则返回拒绝原因（静态串）
 * ============================================================ */
static const char *calib_check_precondition(void)
{
    const seq_ctrl_context_t *sctx = seq_control_get_context();

    if (s_count >= CALIB_MAX_PER_BOOT) {
        return "count limit (2/boot)";
    }
    if (vehicle_is_started() == 0U) {
        return "not started";
    }
    if (g_swing_obj.ctx.encoder_fault != 0U) {
        return "encoder fault";
    }
    if (g_lift_obj.ctx.sensor_fault != 0U) {
        return "sensor fault";
    }
    if ((g_lift_obj.ctx.valve_fault_up != 0U) ||
        (g_lift_obj.ctx.valve_fault_down != 0U)) {
        return "valve fault";
    }
    if ((g_swing_obj.ctx.left_sw_pressed != 0U) ||
        (g_swing_obj.ctx.right_sw_pressed != 0U) ||
        (g_lift_obj.ctx.up_sw_pressed != 0U) ||
        (g_lift_obj.ctx.down_sw_pressed != 0U)) {
        return "manual keys";
    }
    if ((sctx != RT_NULL) && (seq_engine_is_active(&sctx->engine) != 0U)) {
        return "onekey running";        /* TC-CAL-011：不打断一键 */
    }
    if (Foc_Proc_IsBusy()) {
        return "foc busy";
    }
    return RT_NULL;
}

/* ============================================================
 * 快区落盘（选项 A 的持久化原语）
 * ============================================================ */
static void calib_save_fast(void)
{
    param_fast_payload_t p;
    int32_t off;

    if (param_fast_load(&p) != 0) {
        memset(&p, 0, sizeof(p));
        p.rec_magic = PARAM_FAST_REC_MAGIC;
    }

    off = (int32_t)g_enc_elec_offset % (int32_t)CFG_ENCODER_CPR;
    if (off < 0) {
        off += (int32_t)CFG_ENCODER_CPR;
    }

    p.cpr            = (uint16_t)CFG_ENCODER_CPR;
    p.pole_pairs     = (uint8_t)CFG_MOTOR_POLE_PAIRS;
    p.elec_dir       = g_enc_elec_dir;
    p.offset         = (uint32_t)off;
    p.source         = 0U;                          /* ABZ */
    p.valid          = (uint8_t)(0x01U | 0x02U);    /* bit0 对齐 + bit1 筒角 */
    p.tube_angle_x10 = 0;                           /* 刚置零 */
    p.calib_count++;

    if (param_fast_save_now(&p) != 0) {
        rt_kprintf("[Calib] FAIL: fast snapshot save error\n");
        s_reason = "fast save error";
    } else {
        rt_kprintf("[Calib] fast saved: off=%lu dir=%d tube=0 valid=0x03 "
                   "calib_cnt=%lu snap_seq=%lu\n",
                   (unsigned long)p.offset, (int)p.elec_dir,
                   (unsigned long)p.calib_count, (unsigned long)p.snapshot_seq);
    }
}

/* ============================================================
 * 成功 / 失败
 * ============================================================ */
static void calib_on_success(void)
{
    /* 1. 筒角置 0（§13.3 第 6 条：两点式完成后把筒角置 0） */
    swing_angle_zero_here(&g_swing_angle);

    /* 2. is_calibrated=1（内部同步写 AM 记录，断电保持） */
    sys_param_set_calibrated(1U);

    /* 3. 快区立即落盘（顺带关闭 B15 的"mode 3 只写 RAM"缺口） */
    calib_save_fast();

    s_reason = "ok";
    s_state  = CALIB_ST_DONE;
    s_hold_ms = 0U;
    rt_kprintf("[Calib] SUCCESS: calibrated=1, tube zeroed, fast saved\n");
}

static void calib_on_fail(const char *reason)
{
    s_reason = reason;
    s_state  = CALIB_ST_FAIL;
    s_hold_ms = 0U;
    rt_kprintf("[Calib] FAIL: %s\n", reason);
}

/* ============================================================
 * 进入执行
 * ============================================================ */
static void calib_enter_exec(void)
{
    s_state   = CALIB_ST_EXEC;
    s_exec_ms = 0U;
    s_count++;
    s_long_seen = 0U;

    Foc_Proc_Start(FOC_PROC_ENC_ALIGN);
    if (Foc_Proc_GetActive() != FOC_PROC_ENC_ALIGN) {
        /* FOC 拒绝（欠压 / 编码器无响应等，refuse 打印在 foc_proc 里） */
        calib_on_fail("foc refused");
        return;
    }
    rt_kprintf("[Calib] EXEC: ENC_ALIGN started (attempt %u/%u)\n",
               (unsigned)s_count, (unsigned)CALIB_MAX_PER_BOOT);
}

/* ============================================================
 * 公共 API
 * ============================================================ */
void calib_ctrl_init(void)
{
    s_state  = CALIB_ST_IDLE;
    s_count  = 0U;
    s_reason = "-";
    rt_kprintf("[Calib] init: state machine ready (max %u/boot)\n",
               (unsigned)CALIB_MAX_PER_BOOT);
}

void calib_ctrl_feed(uint16_t event)
{
    switch (s_state) {
    case CALIB_ST_IDLE:
        if (event == CALIB_EVT_DOUBLE_CLICK) {
            const char *r = calib_check_precondition();

            if (r != RT_NULL) {
                rt_kprintf("[Calib] reject: %s\n", r);
                calib_on_fail(r);
            } else {
                s_state   = CALIB_ST_PREP;
                s_prep_ms = 0U;
                rt_kprintf("[Calib] PREP: yellow 5s, press&hold to arm\n");
            }
        }
        /* IDLE 下的单击 = 双击的第一击，忽略 */
        break;

    case CALIB_ST_PREP:
        /* "5s 内再按"：按下即进入 ARMED，灯变绿 5Hz，时长由长按判定 */
        if (event == CALIB_EVT_CLICK) {
            s_state     = CALIB_ST_ARMED;
            s_hold_ms   = 0U;
            s_long_seen = 0U;
            rt_kprintf("[Calib] ARMED: green 5Hz, hold >=2s to run\n");
        }
        break;

    case CALIB_ST_ARMED:
        if (event == CALIB_EVT_LONG_PRESS) {
            s_long_seen = 1U;
            calib_enter_exec();
        } else if (event == CALIB_EVT_KEY_UP) {
            /* 长按事件未到就松手：不足 2s（TC-CAL-004） */
            calib_on_fail("short press (<2s)");
        }
        break;

    case CALIB_ST_EXEC:     /* 执行中按键一律忽略 */
    case CALIB_ST_DONE:     /* 结果展示期忽略 */
    case CALIB_ST_FAIL:
        break;

    default:
        s_state = CALIB_ST_IDLE;
        break;
    }
}

void calib_ctrl_tick_50ms(void)
{
    switch (s_state) {
    case CALIB_ST_PREP:
        s_prep_ms += 50U;
        if (s_prep_ms >= CALIB_PREP_WINDOW_MS) {
            s_state  = CALIB_ST_IDLE;
            s_reason = "prep timeout";
            rt_kprintf("[Calib] PREP timeout (5s), back to IDLE\n");
        }
        break;

    case CALIB_ST_EXEC:
        s_exec_ms += 50U;
        if (!Foc_Proc_IsBusy()) {
            if (g_motor_is_aligned != 0U) {
                calib_on_success();
            } else {
                /* encoder_calib.c 的 moved==0 判无效路径会清 aligned */
                calib_on_fail("align invalid (rotor not moved?)");
            }
        } else if (s_exec_ms >= CALIB_EXEC_TIMEOUT_MS) {
            Foc_Proc_Abort();
            calib_on_fail("exec timeout");
        }
        break;

    case CALIB_ST_DONE:
    case CALIB_ST_FAIL:
        s_hold_ms += 50U;
        if (s_hold_ms >= CALIB_RESULT_HOLD_MS) {
            s_state = CALIB_ST_IDLE;
        }
        break;

    default:
        break;
    }
}

int calib_ctrl_start(void)
{
    const char *r = calib_check_precondition();

    if (r != RT_NULL) {
        rt_kprintf("[Calib] reject: %s\n", r);
        calib_on_fail(r);
        return -1;
    }
    calib_enter_exec();
    return (s_state == CALIB_ST_EXEC) ? 0 : -1;
}

void calib_ctrl_abort(void)
{
    if (s_state == CALIB_ST_EXEC) {
        Foc_Proc_Abort();
    }
    s_state  = CALIB_ST_IDLE;
    s_reason = "aborted";
    rt_kprintf("[Calib] abort -> IDLE\n");
}

uint8_t calib_ctrl_state(void)
{
    return (uint8_t)s_state;
}

uint8_t calib_ctrl_count(void)
{
    return s_count;
}

const char *calib_ctrl_state_name(uint8_t st)
{
    switch (st) {
    case CALIB_ST_PREP: return "PREP";
    case CALIB_ST_ARMED: return "ARMED";
    case CALIB_ST_EXEC: return "EXEC";
    case CALIB_ST_DONE: return "DONE";
    case CALIB_ST_FAIL: return "FAIL";
    case CALIB_ST_IDLE: return "IDLE";
    default: return "?";
    }
}

const char *calib_ctrl_reason(void)
{
    return s_reason;
}

/* ============================================================
 * t_calib —— 校准测试命令
 * ------------------------------------------------------------
 *   t_calib               状态总览
 *   t_calib status        同上
 *   t_calib start         直接进入执行（前置条件照常检查）
 *   t_calib abort         强制回 IDLE（EXEC 中会 abort FOC 流程）
 *   t_calib count         本上电周期已用次数 + is_calibrated
 *   t_calib sim dbl       注入"双击"事件（IDLE → PREP）
 *   t_calib sim click     注入"单击按下"事件（PREP 中 → ARMED）
 *   t_calib sim up        注入"松开"事件（ARMED 中 <2s → FAIL）
 *   t_calib sim long      注入"长按 2s 到"事件（ARMED 中 → EXEC）
 * ============================================================ */
static void t_calib_dump(void)
{
    param_fast_payload_t p;
    int have = (param_fast_load(&p) == 0);

    rt_kprintf("======== CALIB ========\n");
    rt_kprintf("state      : %s (%u)\n",
               calib_ctrl_state_name(s_state), (unsigned)s_state);
    rt_kprintf("count      : %u / %u (per boot)\n",
               (unsigned)s_count, (unsigned)CALIB_MAX_PER_BOOT);
    rt_kprintf("calibrated : %u  (is_calibrated)\n",
               (unsigned)g_sys_params.is_calibrated);
    rt_kprintf("aligned    : %u  (g_motor_is_aligned)\n",
               (unsigned)g_motor_is_aligned);
    rt_kprintf("reason     : %s\n", s_reason);
    rt_kprintf("fast       : %s valid=0x%02X calib_cnt=%lu tube=%ld snap_seq=%lu\n",
               have ? "loaded" : "load fail",
               have ? (unsigned)p.valid : 0U,
               have ? (unsigned long)p.calib_count : 0UL,
               have ? (long)p.tube_angle_x10 : 0L,
               have ? (unsigned long)p.snapshot_seq : 0UL);
    rt_kprintf("=======================\n");
}

static void t_calib(int argc, char **argv)
{
    const char *sub = (argc >= 2) ? argv[1] : "status";

    if (strcmp(sub, "status") == 0) {
        t_calib_dump();
        return;
    }

    if (strcmp(sub, "count") == 0) {
        rt_kprintf("[Calib] count=%u/%u is_calibrated=%u\n",
                   (unsigned)calib_ctrl_count(),
                   (unsigned)2U,
                   (unsigned)g_sys_params.is_calibrated);
        return;
    }

    /* 以下子命令都要诊断写权限 */
    if (sys_cap_require(SYS_CAP_DIAG_WRITE, "t_calib") == 0U) {
        return;
    }

    if (strcmp(sub, "start") == 0) {
        (void)calib_ctrl_start();
        return;
    }
    if (strcmp(sub, "abort") == 0) {
        calib_ctrl_abort();
        return;
    }
    if (strcmp(sub, "sim") == 0) {
        const char *k = (argc >= 3) ? argv[2] : "";

        if (strcmp(k, "dbl") == 0)        { calib_ctrl_feed(CALIB_EVT_DOUBLE_CLICK); }
        else if (strcmp(k, "click") == 0) { calib_ctrl_feed(CALIB_EVT_CLICK); }
        else if (strcmp(k, "up") == 0)    { calib_ctrl_feed(CALIB_EVT_KEY_UP); }
        else if (strcmp(k, "long") == 0)  { calib_ctrl_feed(CALIB_EVT_LONG_PRESS); }
        else {
            rt_kprintf("[Calib] usage: t_calib sim dbl|click|up|long\n");
        }
        return;
    }

    rt_kprintf("[Calib] usage: t_calib [status|start|abort|count|sim dbl|click|up|long]\n");
}
MSH_CMD_EXPORT_ALIAS(t_calib, t_calib, calibration state machine test);
