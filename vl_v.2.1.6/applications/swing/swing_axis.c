/**
 * @file    swing_axis.c
 * @brief   回转轴适配层（H1-A）
 *
 * 关键口径（业务.md §15.19）：
 *   - 手动：位置环增量，每 100ms 目标 = 当前角 + 动态步长
 *   - 每次停→动：等抱闸推定解锁 → α 一点式对齐 200ms → 等到 500ms → 位置环
 *   - 松开：位置环减速到 in_pos/30rpm → FOC IDLE → 物理抱闸锁止
 *   - 限位/零点由本层统一 guard，不依赖 swing 状态机的 50ms 刷新
 *   - 筒角来自 FOC ABZ 计数 + mech 换算，t_axis zero 写 RAM 零点
 */

#include "swing_axis.h"

#include <math.h>
#include <rtthread.h>
#include <finsh.h>
#include <stdlib.h>
#include <string.h>

#include "swing_angle.h"
#include "swing_control.h"              /* g_swing_dir_polarity */
#include "../mode/sys_mode.h"           /* t_axis 也走模式门禁 */
#include "../bsp/bsp_brake.h"
#include "../hw/hw_brake.h"
#include "../motor/drv/hw_encoder.h"    /* g_enc_elec_dir / hw_encoder_get_count */
#include "../motor/foc/core/foc_run.h"
#include "../motor/foc/encoder_calib.h"   /* g_motor_is_aligned */
#include "../motor/foc/proc/foc_proc.h"
#include "../motor/foc/foc_speed_loop.h"
#include "../motor/foc/foc_position_loop.h"
#include "../angle/mech_cfg.h"
#include "../angle/mech_cfg_store.h"

#define AXIS_GATE_MS            500U
#define AXIS_ALIGN_MS           200U
#define AXIS_TICK_MS            50U
#define AXIS_TARGET_DIV         2U      /* 50ms * 2 = 100ms */
#define AXIS_LIMIT_LEFT_X10     (-1000)
#define AXIS_LIMIT_RIGHT_X10    (2150)
#define AXIS_STEP_MIN_X10       10      /* 1.0 deg */
#define AXIS_STEP_MAX_X10       100     /* 10.0 deg */

typedef struct {
    swing_axis_state_t state;
    uint8_t  dir;               /* 1 left / 2 right */
    uint8_t  pending;
    uint8_t  abs_mode;
    uint8_t  zero_valid;
    uint8_t  zero_pending;
    uint8_t  wait_zero;
    uint8_t  need_release;
    uint8_t  forbid_dir;
    uint8_t  stop_req;
    uint8_t  tick_div;
    int32_t  zero_raw_x10;
    int32_t  abs_target_x10;
    int32_t  last_target_x10;
    uint32_t cmd_tick;
    uint32_t stop_tick;
} swing_axis_ctx_t;

static swing_axis_ctx_t s_ax;

/* ============================================================
 * 筒角（本层独立于 abs_angle 解算）
 * ============================================================ */
static int32_t axis_raw_x10(void)
{
    const mech_rt_t *rt = mech_get_rt();
    int32_t x10;

    if ((rt == RT_NULL) || (rt->out_den == 0U)) {
        return 0;
    }

    x10 = mech_div_round64((int64_t)hw_encoder_get_count() *
                           (int64_t)rt->out_num,
                           (int64_t)rt->out_den);
    if (g_swing_dir_polarity != 0U) {
        x10 = -x10;
    }
    return x10;
}

static int32_t axis_tube_x10(void)
{
    if (s_ax.zero_valid == 0U) {
        return axis_raw_x10();
    }
    return axis_raw_x10() - s_ax.zero_raw_x10;
}

/* 位置环方向：dir * polarity；mode3 后 dir=-1 时，角度正方向对应负 rev */
static float axis_angle_to_rev(int32_t angle_x10)
{
    int32_t sign = (g_enc_elec_dir >= 0) ? 1 : -1;

    if (g_swing_dir_polarity != 0U) {
        sign = -sign;
    }
    return (float)angle_x10 * 0.3f * (float)sign;
}

/* 动态步长：1.5 × (rpm/180) × 100ms；1500→1.25°，6500→5.42° */
static int32_t axis_step_x10(void)
{
    float rpm = g_pos_v_max_rpm;
    float step_deg;
    int32_t step_x10;

    if (rpm < 100.0f) {
        rpm = 1500.0f;
    }
    step_deg = 1.5f * (rpm / 180.0f) * 0.1f;
    step_x10 = (int32_t)(step_deg * 10.0f + 0.5f);

    if (step_x10 < AXIS_STEP_MIN_X10) step_x10 = AXIS_STEP_MIN_X10;
    if (step_x10 > AXIS_STEP_MAX_X10) step_x10 = AXIS_STEP_MAX_X10;
    return step_x10;
}

static const char *brake_name(hw_brake_state_t st)
{
    switch (st) {
    case BRAKE_STATE_LOCKED:    return "LOCKED";
    case BRAKE_STATE_UNLOCKING: return "UNLOCKING";
    case BRAKE_STATE_UNLOCKED:  return "UNLOCKED";
    case BRAKE_STATE_LOCKING:   return "LOCKING";
    case BRAKE_STATE_FAULT:     return "FAULT";
    default:                    return "?";
    }
}

const char *swing_axis_state_name(swing_axis_state_t st)
{
    switch (st) {
    case SWING_AXIS_IDLE:       return "IDLE";
    case SWING_AXIS_WAIT_BRAKE: return "WAIT_BRAKE";
    case SWING_AXIS_ALIGN:      return "ALIGN";
    case SWING_AXIS_GATE:       return "GATE";
    case SWING_AXIS_POSITION:   return "POSITION";
    case SWING_AXIS_STOPPING:   return "STOPPING";
    case SWING_AXIS_LOCK_WAIT:  return "LOCK_WAIT";
    case SWING_AXIS_FAULT:      return "FAULT";
    default:                    return "?";
    }
}

/* ============================================================
 * 限位 / 零点 guard
 * ============================================================ */
static int32_t axis_clamp_target(int32_t cur, int32_t target)
{
    if (target > AXIS_LIMIT_RIGHT_X10) {
        target = AXIS_LIMIT_RIGHT_X10;
    }
    if (target < AXIS_LIMIT_LEFT_X10) {
        target = AXIS_LIMIT_LEFT_X10;
    }

    /* 手动过零停车：目标不能从一侧跨到另一侧 */
    if ((g_swing_angle.need_zero_stop != 0U) && (s_ax.abs_mode == 0U)) {
        if (((cur > 0) && (target < 0)) ||
            ((cur < 0) && (target > 0))) {
            target = 0;
            s_ax.need_release = 1U;
            s_ax.stop_req     = 1U;
            rt_kprintf("[Axis] zero-stop: target clamped to 0, release required\n");
        }
    }

    if ((target == AXIS_LIMIT_RIGHT_X10) &&
        (s_ax.dir == SWING_AXIS_DIR_RIGHT)) {
        s_ax.forbid_dir = SWING_AXIS_DIR_RIGHT;
        s_ax.stop_req   = 1U;
    }
    if ((target == AXIS_LIMIT_LEFT_X10) &&
        (s_ax.dir == SWING_AXIS_DIR_LEFT)) {
        s_ax.forbid_dir = SWING_AXIS_DIR_LEFT;
        s_ax.stop_req   = 1U;
    }

    return target;
}

/* ============================================================
 * 初始化 / 状态
 * ============================================================ */
void swing_axis_init(void)
{
    memset(&s_ax, 0, sizeof(s_ax));
    s_ax.state = SWING_AXIS_IDLE;

    /* H1-A 默认参数；客户可运行中改这些变量 */
    g_pos_v_max_rpm     = 1500.0f;
    g_spd_accel_rpm_s   = 1000.0f;
    g_pos_dec_rpm_s     = 1000.0f;

    rt_kprintf("[Axis] init: H1-A manual position-loop jog, gate=%ums\n",
               (unsigned)AXIS_GATE_MS);
}

swing_axis_state_t swing_axis_get_state(void)
{
    return s_ax.state;
}

uint8_t swing_axis_is_idle(void)
{
    return (s_ax.state == SWING_AXIS_IDLE) ? 1U : 0U;
}

/* ============================================================
 * 手动按键
 * ============================================================ */
void swing_axis_prepare(uint8_t dir)
{
    if ((dir != SWING_AXIS_DIR_LEFT) && (dir != SWING_AXIS_DIR_RIGHT)) {
        return;
    }
    if (s_ax.state == SWING_AXIS_FAULT) {
        /* 保留 Q30-a：编码器 FAULT 下仍允许手动把筒转回来 */
        s_ax.state = SWING_AXIS_IDLE;
    }
    if ((s_ax.state != SWING_AXIS_IDLE) &&
        (s_ax.state != SWING_AXIS_LOCK_WAIT)) {
        return;
    }
    if (s_ax.forbid_dir == dir) {
        rt_kprintf("[Axis] prepare refused: limit forbid dir=%u\n", (unsigned)dir);
        return;
    }

    s_ax.forbid_dir = 0U;
    s_ax.dir        = dir;
    s_ax.pending    = 1U;
    s_ax.abs_mode   = 0U;
    s_ax.stop_req   = 0U;
    s_ax.tick_div   = 0U;
    s_ax.cmd_tick   = rt_tick_get();
    s_ax.state      = SWING_AXIS_WAIT_BRAKE;

    rt_kprintf("[Axis] prepare dir=%s t=%lu\n",
               (dir == SWING_AXIS_DIR_LEFT) ? "LEFT" : "RIGHT",
               (unsigned long)s_ax.cmd_tick);
}

uint8_t swing_axis_start(void)
{
    if ((s_ax.dir != SWING_AXIS_DIR_LEFT) &&
        (s_ax.dir != SWING_AXIS_DIR_RIGHT)) {
        rt_kprintf("[Axis] start refused: no direction\n");
        return 0U;
    }
    if (s_ax.zero_valid == 0U) {
        rt_kprintf("[Axis] start refused: run 't_axis zero' first\n");
        return 0U;
    }
    if (!sys_mode_allows_swing_motion()) {
        rt_kprintf("[Axis] start refused: mode not allow swing motion\n");
        return 0U;
    }
    if (Foc_Proc_IsBusy()) {
        rt_kprintf("[Axis] start refused: FOC proc busy\n");
        return 0U;
    }
    if (s_ax.forbid_dir == s_ax.dir) {
        rt_kprintf("[Axis] start refused: limit forbid dir=%u\n", (unsigned)s_ax.dir);
        return 0U;
    }
    if (s_ax.pending == 0U) {
        s_ax.pending  = 1U;
        s_ax.cmd_tick = rt_tick_get();
    }
    if (s_ax.state == SWING_AXIS_IDLE) {
        s_ax.state = SWING_AXIS_WAIT_BRAKE;
    }

    if (hw_brake_get_state(&g_swing_brake) != BRAKE_STATE_UNLOCKED) {
        (void)hw_brake_unlock(&g_swing_brake);
    }
    return 1U;
}

/* ============================================================
 * 位置环
 * ============================================================ */
static void axis_set_initial_target(void)
{
    int32_t target_x10;

    if (s_ax.abs_mode != 0U) {
        target_x10 = s_ax.abs_target_x10;
    } else {
        target_x10 = axis_tube_x10();
    }

    s_ax.last_target_x10 = target_x10;
    g_pos_target_rev = axis_angle_to_rev(target_x10);
}

static void axis_enter_position(void)
{
    if (s_ax.zero_valid == 0U) {
        rt_kprintf("[Axis] position refused: no zero\n");
        swing_axis_abort();
        return;
    }

    Foc_Run_SetMode(FOC_RUN_POSITION);
    if (Foc_Run_GetMode() != FOC_RUN_POSITION) {
        rt_kprintf("[Axis] position refused: FOC mode=%d\n",
                   (int)Foc_Run_GetMode());
        swing_axis_abort();
        return;
    }

    /* t_axis zero 的位置环零点请求，必须在 position 模式里消费一次；
     * 否则紧跟着写的绝对目标会被 zero_req 的 target=0 覆盖。 */
    if (s_ax.zero_pending != 0U) {
        g_pos_zero_req   = 1U;
        s_ax.zero_pending = 0U;
        s_ax.wait_zero    = 1U;
    } else {
        s_ax.wait_zero = 0U;
        axis_set_initial_target();
    }

    s_ax.state    = SWING_AXIS_POSITION;
    s_ax.tick_div = 0U;

    rt_kprintf("[Axis] POSITION: angle_x10=%ld target_x10=%ld rev_x100=%ld zero_wait=%u\n",
               (long)axis_tube_x10(), (long)s_ax.last_target_x10,
               (long)(g_pos_target_rev * 100.0f), (unsigned)s_ax.wait_zero);
}

static void axis_update_target(void)
{
    int32_t cur;
    int32_t target;

    if (s_ax.abs_mode != 0U) {
        return;     /* 绝对目标已设置，守到 stop */
    }

    cur = axis_tube_x10();
    if (s_ax.dir == SWING_AXIS_DIR_LEFT) {
        target = cur - axis_step_x10();
    } else {
        target = cur + axis_step_x10();
    }

    target = axis_clamp_target(cur, target);
    s_ax.last_target_x10 = target;
    g_pos_target_rev = axis_angle_to_rev(target);
}

/* ============================================================
 * 停止 / 故障
 * ============================================================ */
void swing_axis_stop(void)
{
    s_ax.stop_req     = 1U;
    s_ax.need_release = 0U;

    if ((s_ax.state == SWING_AXIS_WAIT_BRAKE) ||
        (s_ax.state == SWING_AXIS_ALIGN) ||
        (s_ax.state == SWING_AXIS_GATE)) {
        Foc_Proc_Abort();
        Foc_Run_SetMode(FOC_RUN_IDLE);
        (void)hw_brake_lock(&g_swing_brake);
        s_ax.state = SWING_AXIS_LOCK_WAIT;
        s_ax.pending = 0U;
        return;
    }

    if (s_ax.state == SWING_AXIS_POSITION) {
        s_ax.state = SWING_AXIS_STOPPING;   /* 走完最后一个增量再锁 */
        s_ax.stop_tick = rt_tick_get();
        return;
    }

    if (s_ax.state == SWING_AXIS_IDLE) {
        s_ax.pending = 0U;
        if (hw_brake_get_state(&g_swing_brake) != BRAKE_STATE_LOCKED) {
            (void)hw_brake_lock(&g_swing_brake);
            s_ax.state = SWING_AXIS_LOCK_WAIT;
        }
    }
}

void swing_axis_abort(void)
{
    Foc_Proc_Abort();
    Foc_Run_SetMode(FOC_RUN_IDLE);
    (void)hw_brake_lock(&g_swing_brake);
    s_ax.state   = SWING_AXIS_FAULT;
    s_ax.pending = 0U;
    s_ax.stop_req = 1U;
    rt_kprintf("[Axis] ABORT: FOC IDLE + brake lock\n");
}

/* ============================================================
 * 50ms tick
 * ============================================================ */
void swing_axis_tick(void)
{
    uint32_t elapsed_ms;
    hw_brake_state_t bst;

    elapsed_ms = (uint32_t)((rt_tick_get() - s_ax.cmd_tick) *
                            1000U / RT_TICK_PER_SECOND);

    switch (s_ax.state) {
    case SWING_AXIS_WAIT_BRAKE:
        bst = hw_brake_get_state(&g_swing_brake);
        if (bst == BRAKE_STATE_UNLOCKED) {
            if (Foc_Proc_IsBusy() == false) {
                Foc_Proc_Start(FOC_PROC_ONE_POINT);
                s_ax.state = SWING_AXIS_ALIGN;
                rt_kprintf("[Axis] align start @%lums\n", (unsigned long)elapsed_ms);
            }
        } else if (bst == BRAKE_STATE_FAULT) {
            rt_kprintf("[Axis] brake FAULT in WAIT_BRAKE\n");
            swing_axis_abort();
        }
        break;

    case SWING_AXIS_ALIGN:
        if (Foc_Proc_IsBusy() == false) {
            if (elapsed_ms >= AXIS_GATE_MS) {
                axis_enter_position();
            } else {
                s_ax.state = SWING_AXIS_GATE;
            }
            rt_kprintf("[Axis] align done @%lums\n", (unsigned long)elapsed_ms);
        }
        break;

    case SWING_AXIS_GATE:
        if (elapsed_ms >= AXIS_GATE_MS) {
            axis_enter_position();
        }
        break;

    case SWING_AXIS_POSITION:
        if (s_ax.stop_req != 0U) {
            s_ax.state = SWING_AXIS_STOPPING;
            break;
        }
        if (s_ax.wait_zero != 0U) {
            if (g_pos_zero_req == 0U) {
                s_ax.wait_zero = 0U;
                axis_set_initial_target();
                rt_kprintf("[Axis] position zero consumed, target=%ld\n",
                           (long)s_ax.last_target_x10);
            }
            break;
        }
        s_ax.tick_div++;
        if (s_ax.tick_div >= AXIS_TARGET_DIV) {
            s_ax.tick_div = 0U;
            axis_update_target();
        }
        break;

    case SWING_AXIS_STOPPING:
        if ((g_pos_in_pos != 0U) &&
            (fabsf(g_spd_filt_rpm) <= g_pos_v_inpos_rpm)) {
            Foc_Run_SetMode(FOC_RUN_IDLE);
            (void)hw_brake_lock(&g_swing_brake);
            s_ax.state = SWING_AXIS_LOCK_WAIT;
            rt_kprintf("[Axis] stopped @%ld x10, brake lock\n",
                       (long)axis_tube_x10());
        } else if ((rt_tick_get() - s_ax.stop_tick) >=
                   rt_tick_from_millisecond(5000U)) {
            /* 位置环没有给出 in_pos（无电流环/编码器异常）：超时强制收口 */
            Foc_Run_SetMode(FOC_RUN_IDLE);
            (void)hw_brake_lock(&g_swing_brake);
            s_ax.state = SWING_AXIS_LOCK_WAIT;
            rt_kprintf("[Axis] stop timeout, force lock @%ld x10\n",
                       (long)axis_tube_x10());
        }
        break;

    case SWING_AXIS_LOCK_WAIT:
        if (hw_brake_get_state(&g_swing_brake) == BRAKE_STATE_LOCKED) {
            s_ax.state   = SWING_AXIS_IDLE;
            s_ax.pending = 0U;
        }
        break;

    case SWING_AXIS_FAULT:
    case SWING_AXIS_IDLE:
    default:
        break;
    }
}

/* ============================================================
 * t_axis zero
 * ============================================================ */
void swing_axis_zero_here(void)
{
    int32_t raw;

    if (s_ax.state != SWING_AXIS_IDLE) {
        rt_kprintf("[Axis] zero refused: state=%s\n",
                   swing_axis_state_name(s_ax.state));
        return;
    }

    raw = axis_raw_x10();
    s_ax.zero_raw_x10 = raw;
    s_ax.zero_valid   = 1U;
    s_ax.zero_pending = 1U;
    s_ax.need_release = 0U;
    s_ax.forbid_dir   = 0U;

    /* 同步 swing_angle（t_swing 也归零），位置环零点在首次进 POSITION 时消费 */
    swing_angle_calibrate(&g_swing_angle, raw);
    g_pos_target_rev = 0.0f;

    rt_kprintf("[Axis] zero: raw=%ld, tube=0.0deg, pos_zero_pending=1\n",
               (long)raw);
}

/* ============================================================
 * t_axis move（H1-B 预留，H1-A 用来验证换算）
 * ============================================================ */
int32_t swing_axis_move_to(int32_t angle_x10)
{
    int32_t cur;

    if (s_ax.zero_valid == 0U) {
        return -1;
    }
    if (!sys_mode_allows_swing_motion()) {
        return -4;
    }
    if ((angle_x10 < AXIS_LIMIT_LEFT_X10) ||
        (angle_x10 > AXIS_LIMIT_RIGHT_X10)) {
        return -2;
    }
    if ((s_ax.state != SWING_AXIS_IDLE) &&
        (s_ax.state != SWING_AXIS_LOCK_WAIT)) {
        return -3;
    }

    cur = axis_tube_x10();
    s_ax.abs_mode      = 1U;
    s_ax.abs_target_x10 = angle_x10;
    s_ax.dir = (angle_x10 >= cur) ? SWING_AXIS_DIR_RIGHT : SWING_AXIS_DIR_LEFT;
    s_ax.pending  = 1U;
    s_ax.stop_req = 0U;
    s_ax.tick_div = 0U;
    s_ax.cmd_tick = rt_tick_get();
    s_ax.state    = SWING_AXIS_WAIT_BRAKE;

    if (hw_brake_get_state(&g_swing_brake) != BRAKE_STATE_UNLOCKED) {
        (void)hw_brake_unlock(&g_swing_brake);
    }
    return 0;
}

/* ============================================================
 * t_axis 命令
 * ============================================================ */
static void axis_print_stat(void)
{
    rt_kprintf("\n======== AXIS (H1-A) ========\n");
    rt_kprintf("state    : %s\n", swing_axis_state_name(s_ax.state));
    rt_kprintf("dir      : %s   pending=%u zero=%u\n",
               (s_ax.dir == SWING_AXIS_DIR_LEFT) ? "LEFT" :
               ((s_ax.dir == SWING_AXIS_DIR_RIGHT) ? "RIGHT" : "-"),
               (unsigned)s_ax.pending, (unsigned)s_ax.zero_valid);
    rt_kprintf("brake    : %s\n", brake_name(hw_brake_get_state(&g_swing_brake)));
    rt_kprintf("angle    : %ld x10 (%ld mdeg) raw=%ld zero=%ld\n",
               (long)axis_tube_x10(), (long)(axis_tube_x10() * 100),
               (long)axis_raw_x10(), (long)s_ax.zero_raw_x10);
    rt_kprintf("target   : last=%ld abs=%ld abs_mode=%u\n",
               (long)s_ax.last_target_x10, (long)s_ax.abs_target_x10,
               (unsigned)s_ax.abs_mode);
    rt_kprintf("pos_rev  : fb_x100=%ld target_x100=%ld in_pos=%u\n",
               (long)(g_pos_fb_rev * 100.0f),
               (long)(g_pos_target_rev * 100.0f), (unsigned)g_pos_in_pos);
    rt_kprintf("loop     : rpm=%ld vmax=%ld step_x10=%ld\n",
               (long)g_spd_filt_rpm, (long)g_pos_v_max_rpm,
               (long)axis_step_x10());
    rt_kprintf("foc      : mode=%d aligned=%u busy=%u gate=%ums\n",
               (int)Foc_Run_GetMode(), (unsigned)g_motor_is_aligned,
               (unsigned)(Foc_Proc_IsBusy() ? 1U : 0U),
               (unsigned)AXIS_GATE_MS);
    rt_kprintf("=============================\n");
}

static void cmd_t_axis(int argc, char **argv)
{
    if (argc < 2) {
        axis_print_stat();
        return;
    }

    if (strcmp(argv[1], "stat") == 0) {
        axis_print_stat();
        return;
    }

    if (strcmp(argv[1], "zero") == 0) {
        swing_axis_zero_here();
        return;
    }

    if (strcmp(argv[1], "stop") == 0) {
        swing_axis_stop();
        rt_kprintf("[Axis] stop requested\n");
        return;
    }

    if ((strcmp(argv[1], "jog") == 0) && (argc >= 4)) {
        uint8_t dir;
        int32_t ms = atoi(argv[3]);
        uint32_t t0;

        if (strcmp(argv[2], "left") == 0) {
            dir = SWING_AXIS_DIR_LEFT;
        } else if (strcmp(argv[2], "right") == 0) {
            dir = SWING_AXIS_DIR_RIGHT;
        } else {
            rt_kprintf("usage: t_axis jog left|right <ms>\n");
            return;
        }
        if (ms < 100) ms = 100;
        if (ms > 30000) ms = 30000;

        swing_axis_prepare(dir);
        if (swing_axis_start() == 0U) {
            s_ax.state   = SWING_AXIS_IDLE;
            s_ax.pending = 0U;
            return;
        }
        t0 = rt_tick_get();
        while ((rt_tick_get() - t0) < rt_tick_from_millisecond((rt_tick_t)ms)) {
            rt_thread_mdelay(50);
        }
        swing_axis_stop();
        {
            uint32_t t1 = rt_tick_get();
            while ((rt_tick_get() - t1) < rt_tick_from_millisecond(8000U)) {
                if (s_ax.state == SWING_AXIS_IDLE) {
                    break;
                }
                rt_thread_mdelay(50);
            }
        }
        rt_kprintf("[Axis] jog %s %ldms done, state=%s angle_x10=%ld\n",
                   (dir == SWING_AXIS_DIR_LEFT) ? "left" : "right", (long)ms,
                   swing_axis_state_name(s_ax.state), (long)axis_tube_x10());
        return;
    }

    if ((strcmp(argv[1], "move") == 0) && (argc >= 3)) {
        int32_t target = atoi(argv[2]);
        int32_t rc = swing_axis_move_to(target);
        uint8_t reached = 0U;

        if (rc != 0) {
            rt_kprintf("[Axis] move %ld refused: rc=%ld\n",
                       (long)target, (long)rc);
            return;
        }

        {
            uint32_t t0 = rt_tick_get();
            while ((rt_tick_get() - t0) < rt_tick_from_millisecond(15000U)) {
                if ((s_ax.state == SWING_AXIS_POSITION) &&
                    (g_pos_in_pos != 0U)) {
                    reached = 1U;
                    break;
                }
                if ((s_ax.state == SWING_AXIS_FAULT) ||
                    (s_ax.state == SWING_AXIS_IDLE)) {
                    break;
                }
                rt_thread_mdelay(50);
            }
        }
        swing_axis_stop();
        {
            uint32_t t1 = rt_tick_get();
            while ((rt_tick_get() - t1) < rt_tick_from_millisecond(8000U)) {
                if (s_ax.state == SWING_AXIS_IDLE) {
                    break;
                }
                rt_thread_mdelay(50);
            }
        }
        rt_kprintf("[Axis] move %ld -> state=%s angle_x10=%ld reached=%u\n",
                   (long)target, swing_axis_state_name(s_ax.state),
                   (long)axis_tube_x10(), (unsigned)reached);
        return;
    }

    rt_kprintf("usage: t_axis [stat|zero|stop|jog left|right <ms>|move <x10>]\n");
}

MSH_CMD_EXPORT_ALIAS(cmd_t_axis, t_axis, H1-A swing axis: stat/zero/jog/move/stop);
