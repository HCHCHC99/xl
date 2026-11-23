/**
 * @file    seq_control.c
 * @brief   序列控制对象实现
 */

#include "seq_control.h"
#include "seq_config.h"
#include "seq_actions.h"
#include "seq_arbit.h"
#include "../mode/sys_mode.h"
#include "../mode/sys_caps.h"
#include "framework/state_engine/sm_assert.h"
#include "app_event.h"
#include "framework/state_engine/state_engine.h"
#include "../swing/swing_control.h"
#include "../swing/swing_angle.h"
#include "../lift/lift_angle.h"
#include "../lift/lift_control.h"
#include "../vehicle/vehicle_control.h"   /* D-11：前置条件读实时启车状态 */
#include <string.h>
#include <stdlib.h>
#include <rtthread.h>

/* ============================================================
 * 全局对象实例
 * ============================================================ */
seq_control_t g_seq_obj;

/* ============================================================
 * 序列状态枚举（用于控制对象内部状态机）
 * ============================================================ */
enum {
    SEQ_OBJ_STATE_IDLE = 0,
    SEQ_OBJ_STATE_ACTIVE,
    SEQ_OBJ_STATE_MAX
};

/* ============================================================
 * 前置条件检查
 * ============================================================ */
static uint8_t seq_check_precondition(seq_ctrl_context_t *sctx)
{
    (void)sctx;     /* 前置条件全部读实时状态；参数保留，便于以后按 ctx 扩展 */

    /* 1. 启车状态（D-11，2026-11-23）
     *
     * 原来读 ctx->vehicle_started 这份"事件副本"：它只在 ACC_OFF 里清 0，
     * 进 FAULT（CAN 静默 / 电压缺失）不清，t_veh_clr 直调整车对象也不走事件，
     * 于是钥匙清故障后整车停在 ACC_ON，前置条件却仍读到旧的 1
     * ⇒ t_seq_run / t_seq_case 能在未启车时启动，测试"过了却没测到前置条件"。
     *
     * 改为直接问整车对象：FAULT 下 vehicle_is_started() 返回 0，
     * 与 app_main 门 2（evt_needs_vehicle_started）用同一判据，不会两处不一致。
     * ctx->vehicle_started 保留为事件侧副本，仅用于诊断，不再参与门禁。 */
    if (vehicle_is_started() == 0U) {
        rt_kprintf("[Seq] Precondition fail: not started\n");
        return 0;
    }

    /* 2. 模式允许 */
    if (!sys_mode_allows_sequence()) {
        rt_kprintf("[Seq] Precondition fail: mode\n");
        return 0;
    }

    /* 3. 举升无故障 */
    if (g_lift_obj.ctx.sensor_fault) {
        rt_kprintf("[Seq] Precondition fail: sensor fault\n");
        return 0;
    }
    if (g_lift_obj.ctx.valve_fault_up || g_lift_obj.ctx.valve_fault_down) {
        rt_kprintf("[Seq] Precondition fail: valve fault\n");
        return 0;
    }

    /* 4. 回转无故障 */
    if (g_swing_obj.ctx.encoder_fault) {
        rt_kprintf("[Seq] Precondition fail: encoder fault\n");
        return 0;
    }

    /* 5. 无手动输入 */
    if (g_swing_obj.ctx.left_sw_pressed ||
        g_swing_obj.ctx.right_sw_pressed) {
        return 0;
    }
    if (g_lift_obj.ctx.up_sw_pressed || g_lift_obj.ctx.down_sw_pressed) {
        return 0;
    }

    return 1;
}

/* ============================================================
 * 中断检查
 * ============================================================ */
static uint8_t seq_check_interrupt(seq_ctrl_context_t *sctx)
{
    /* 1. 手动按键 */
    if (g_swing_obj.ctx.left_sw_pressed ||
        g_swing_obj.ctx.right_sw_pressed) {
        return 1;
    }
    if (g_lift_obj.ctx.up_sw_pressed || g_lift_obj.ctx.down_sw_pressed) {
        return 1;
    }

    /* 2. 故障 */
    if (g_lift_obj.ctx.sensor_fault ||
        g_lift_obj.ctx.valve_fault_up ||
        g_lift_obj.ctx.valve_fault_down) {
        return 1;
    }

    /* 3. 模式变更 */
    if (!sys_mode_allows_sequence()) {
        return 1;
    }

    return 0;
}

/* ============================================================
 * 一键类型映射
 * ============================================================ */
const char *seq_control_onekey_name(uint8_t key)
{
    switch (key) {
        case 1U: return "RIGHT_MIDDLE";
        case 2U: return "RIGHT_BEHIND";
        case 3U: return "HOME";
        default: return "NONE";
    }
}

static seq_type_t seq_onekey_to_type(uint8_t key)
{
    switch (key) {
        case 1U: return SEQ_TYPE_RIGHT_MIDDLE;
        case 2U: return SEQ_TYPE_RIGHT_BEHIND;
        case 3U: return SEQ_TYPE_HOME;
        default: return SEQ_TYPE_RIGHT_MIDDLE;
    }
}

static const char *seq_abort_name(uint8_t reason)
{
    switch (reason) {
        case 0U: return "NONE";
        case 1U: return "INTERRUPT";
        case 2U: return "USER";
        case 3U: return "TIMEOUT";
        default: return "?";
    }
}

/* ============================================================
 * 启动序列
 * @return 0 = 已启动；-1 = 前置条件不满足 / 配置缺失 / 引擎拒绝
 * ============================================================ */
static int seq_start_sequence(seq_type_t type)
{
    seq_ctrl_context_t *sctx = &g_seq_obj.ctx;

    /* 1. 前置条件检查 */
    if (!seq_check_precondition(sctx)) {
        sctx->reject_count++;
        sctx->rej_precond++;
        return -1;
    }

    /* 2. 查找配置
     * 【P7 / B-4】回位要看回转是否已在零位：已在零位只做「下降 + 保持 2 s」
     * （TC-ONE-011），否则走完整五步。这两份配置的 type 都是 SEQ_TYPE_HOME。 */
    const seq_config_t *config;
    if (type == SEQ_TYPE_HOME) {
        int32_t h = swing_angle_get_value(&g_swing_angle);

        config = seq_config_find_home(h);
        if (config == &g_seq_home_at_zero) {
            /* 【P7】不用 %.1f：rt_kprintf 不解析浮点，会把 "%.1f" 原样打出来
             * 并吃掉一个参数，导致后面的 %d 读到错位值（首轮打出 "%f ... 0"）。
             * 直接按 0.1° 整数拆成"整数位.小数位"。 */
            int32_t av = (h < 0) ? -h : h;

            rt_kprintf("[Seq] home: swing at %s%d.%d deg (within +-%d deg), descend only\n",
                       (h < 0) ? "-" : "",
                       (int)(av / 10), (int)(av % 10),
                       (int)SEQ_HOME_AT_ZERO_TOL_X10 / 10);
        }
    } else {
        config = seq_config_find(type);
    }
    if (config == NULL) {
        rt_kprintf("[Seq] Config not found for type %d\n", type);
        sctx->reject_count++;
        sctx->rej_precond++;
        return -1;
    }

    /* 3. 初始化引擎 */
    seq_engine_init(&sctx->engine, config, type,
                    &g_swing_obj, &g_lift_obj,
                    &g_swing_angle, &g_lift_angle);

    /* 4. 启动 */
    if (seq_engine_start(&sctx->engine) != 0) {
        sctx->reject_count++;
        sctx->rej_precond++;
        return -1;
    }

    sctx->request_count++;
    /* Q-F3 路 A：改走专用投递接口（铁律 2：状态变更经聚合根） */
    control_object_process(&g_seq_obj.base, SEQ_OBJ_EVT_START);
    return 0;
}

/* ============================================================
 * 状态机配置
 * ============================================================ */
static void seq_obj_enter_idle(StateContext_t ctx)
{
    (void)ctx;
}

static void seq_obj_enter_active(StateContext_t ctx)
{
    (void)ctx;
}

static const StateFuncTable_t g_seq_obj_func_table[] = {
    {SEQ_OBJ_STATE_IDLE,   seq_obj_enter_idle,   NULL},
    {SEQ_OBJ_STATE_ACTIVE, seq_obj_enter_active, NULL},
};

static const StateJumpTable_t g_seq_obj_jump_table[] = {
    /* 【Q-F3 路 A / 2026-11-14 E1】对象级状态机改为真实事件驱动。
     * 只经 control_object_process(&g_seq_obj.base, evt) 同步投递，
     * 不周期调度、不走 post_event（引擎只在匹配到事件时触发一次）。
     * 真正的步骤编排在 sctx->engine（seq_engine 5 态），属**编排层
     * 内部状态机**，不在本表（StateFuncTable_t 无 tick/步骤回调）。 */
    {SEQ_OBJ_STATE_IDLE,   SEQ_OBJ_EVT_START,  SEQ_OBJ_STATE_ACTIVE},
    {SEQ_OBJ_STATE_ACTIVE, SEQ_OBJ_EVT_FINISH, SEQ_OBJ_STATE_IDLE},
    {SEQ_OBJ_STATE_ACTIVE, SEQ_OBJ_EVT_ABORT,  SEQ_OBJ_STATE_IDLE},
};

static const StateMachineConfig_t g_seq_obj_config = {
    .jump_table      = g_seq_obj_jump_table,
    .jump_table_size = sizeof(g_seq_obj_jump_table) / sizeof(g_seq_obj_jump_table[0]),
    .func_table      = g_seq_obj_func_table,
    .func_table_size = sizeof(g_seq_obj_func_table) / sizeof(g_seq_obj_func_table[0]),
    .init_state      = SEQ_OBJ_STATE_IDLE,
    .max_state       = SEQ_OBJ_STATE_MAX,
    .name            = "SeqSM"
};

/* ============================================================
 * 初始化并注册
 * ============================================================ */
int seq_control_register(ctrl_manager_t *mgr)
{
    rt_err_t ret;

    SM_ASSERT_PTR(mgr);

    memset(&g_seq_obj, 0, sizeof(seq_control_t));

    g_seq_obj.base.sm = &g_seq_obj.sm_instance;

    ctrl_obj_runtime_cfg_t runtime = {
        .use_thread  = 0,
        .thread_prio = 0,
        .stack_size  = 0,
        .mq_size     = 0,
    };

    ret = control_object_init(&g_seq_obj.base,
                              "SeqControl",
                              CTRL_OBJ_TYPE_SEQ,
                              &g_seq_obj_config,
                              &g_seq_obj.ctx,
                              &runtime);
    if (ret != RT_EOK) {
        rt_kprintf("[Seq] control_object_init failed (%d)\n", ret);
        return -1;
    }

    ret = ctrl_manager_register(mgr, &g_seq_obj.base);
    if (ret != 0) {
        rt_kprintf("[Seq] register failed (%d)\n", ret);
        return -1;
    }

    rt_kprintf("[Seq] Control registered.\n");
    return 0;
}

/* ============================================================
 * 一键触发（P3 主入口）
 * ------------------------------------------------------------
 * 判定顺序（先"允不允许"，再"要不要打断"）：
 *   1. 参数非法                  → SEQ_RET_PRECOND
 *   2. 仲裁（多键 / 超限）       → SEQ_RET_MULTI / SEQ_RET_LIMIT
 *   3. 同键重复（序列在跑）      → SEQ_RET_DUP，不打断
 *   4. 异键（序列在跑）          → 中断旧序列后启动新的
 *   5. 无活动序列                → 直接启动
 * 关键点：仲裁在"打断"之前做——被拒的请求不允许把正在跑的序列打掉。
 * ============================================================ */
static int seq_trigger_common(uint8_t key, uint8_t key_count)
{
    seq_ctrl_context_t *sctx = &g_seq_obj.ctx;
    seq_type_t   type;
    seq_arb_t    arb;
    int32_t      angle;

    if ((key < 1U) || (key > 3U)) {
        return SEQ_RET_PRECOND;
    }

    type  = seq_onekey_to_type(key);
    angle = swing_angle_get_value(&g_swing_angle);

    /* ---- 2. 仲裁（多键 / 超限）---- */
    arb = seq_arbit_check_onekey(type, angle, key_count);
    if (arb != SEQ_ARB_OK) {
        if (arb == SEQ_ARB_DENY_MULTI_KEY) {
            sctx->rej_multi++;
        } else {
            sctx->rej_limit++;
        }
        sctx->reject_count++;
        rt_kprintf("[Seq] %s denied: %s (angle=%ld x10)\n",
                   seq_control_onekey_name(key),
                   seq_arbit_reason(arb),
                   (long)angle);
        return (arb == SEQ_ARB_DENY_MULTI_KEY) ? SEQ_RET_MULTI : SEQ_RET_LIMIT;
    }

    /* ---- 3./4. 序列在跑：同键忽略、异键中断 ---- */
    if (seq_engine_is_active(&sctx->engine)) {
        if (sctx->current_onekey == key) {
            /* 同键重复：动作时序仍按第一次执行（TC-ONE-012 / PRI-006） */
            sctx->rej_dup++;
            sctx->reject_count++;
            rt_kprintf("[Seq] %s ignored (same key running)\n",
                       seq_control_onekey_name(key));
            return SEQ_RET_DUP;
        }

        /* 其他一键：立即中断并执行新动作（TC-ONE-013 / PRI-004） */
        rt_kprintf("[Seq] %s interrupts %s\n",
                   seq_control_onekey_name(key),
                   seq_control_onekey_name(sctx->current_onekey));
        sctx->abort_reason = 1U;
        seq_engine_abort(&sctx->engine);
        control_object_process(&g_seq_obj.base, SEQ_OBJ_EVT_ABORT);
    }

    /* ---- 5. 启动 ---- */
    sctx->current_onekey = key;
    sctx->abort_reason   = 0U;

    rt_kprintf("[Seq] Trigger %s\n", seq_control_onekey_name(key));

    if (seq_start_sequence(type) != 0) {
        sctx->current_onekey = 0U;      /* 未真正启动：不占用"当前一键" */
        return SEQ_RET_PRECOND;
    }
    return SEQ_RET_OK;
}

int seq_control_trigger(uint8_t key)
{
    return seq_trigger_common(key, 1U);
}

int seq_control_trigger_multi(uint8_t mid, uint8_t rear, uint8_t home)
{
    uint8_t n   = 0U;
    uint8_t key = 0U;

    if (mid  != 0U) { n++; key = 1U; }
    if (rear != 0U) { n++; if (key == 0U) { key = 2U; } }
    if (home != 0U) { n++; if (key == 0U) { key = 3U; } }

    if (n == 0U) {
        rt_kprintf("[Seq] trigger_multi: no key given\n");
        return SEQ_RET_PRECOND;
    }

    /* n >= 2 时仲裁会直接判 MULTI，这里仍带上"第一个键"只为把事件打出来 */
    return seq_trigger_common(key, n);
}

/* ============================================================
 * 兼容旧接口：由上升沿转到 seq_control_trigger()
 * @note  P2 起键盘直接投 SEQ_EVT_ONKEY_*，本函数保留仅为兼容。
 * ============================================================ */
void seq_control_update_onekey(uint8_t middle, uint8_t rear, uint8_t home)
{
    seq_ctrl_context_t *sctx = &g_seq_obj.ctx;

    uint8_t middle_rising = ((middle != 0U) && (sctx->prev_middle == 0U)) ? 1U : 0U;
    uint8_t rear_rising   = ((rear   != 0U) && (sctx->prev_rear   == 0U)) ? 1U : 0U;
    uint8_t home_rising   = ((home   != 0U) && (sctx->prev_home   == 0U)) ? 1U : 0U;

    sctx->prev_middle = middle;
    sctx->prev_rear   = rear;
    sctx->prev_home   = home;

    if (middle_rising != 0U) {
        (void)seq_control_trigger(1U);
    } else if (rear_rising != 0U) {
        (void)seq_control_trigger(2U);
    } else if (home_rising != 0U) {
        (void)seq_control_trigger(3U);
    }
}

/* ============================================================
 * 序列对象的事件分发（P2）
 * @note  由 app_main 在描述表命中后调用。序列对象的 SM 只有 IDLE/ACTIVE
 *        两态，一键按键不引起状态迁移，故在此补一次"命令分发"。
 * ============================================================ */
void seq_control_on_event(app_event_t event)
{
    switch (event) {
        case SEQ_EVT_ONKEY_MIDDLE: (void)seq_control_trigger(1U); break;
        case SEQ_EVT_ONKEY_REAR:   (void)seq_control_trigger(2U); break;
        case SEQ_EVT_ONKEY_HOME:   (void)seq_control_trigger(3U); break;
        default:
            /* 步骤完成 / 超时 / 中止等由引擎内部处理 */
            break;
    }
}

/* ============================================================
 * 启车状态设置
 * ------------------------------------------------------------
 * 【D-11 / 2026-11-23】这份副本**不再参与门禁**——门禁直接问
 * vehicle_is_started()，避免"事件漏清 ⇒ 前置条件误放行"。
 * 保留本函数与 ctx.vehicle_started 供诊断/日志观察状态跳变。
 * ============================================================ */
void seq_control_set_vehicle_started(uint8_t started)
{
    g_seq_obj.ctx.vehicle_started = started;
}

/* ============================================================
 * 周期更新（由主循环或控制对象广播调用）
 * ============================================================ */
void seq_control_update(void)
{
    seq_ctrl_context_t *sctx = &g_seq_obj.ctx;

    if (!seq_engine_is_active(&sctx->engine)) {
        return;
    }

    /* 中断检查（手动键 / 故障 / 模式变更 → 立即停止整条序列） */
    if (seq_check_interrupt(sctx)) {
        rt_kprintf("[Seq] Interrupted\n");
        sctx->abort_reason = 1U;
        seq_engine_abort(&sctx->engine);
        control_object_process(&g_seq_obj.base, SEQ_OBJ_EVT_ABORT);
        sctx->current_onekey = 0U;
        return;
    }

    /* 引擎更新 */
    seq_engine_update(&sctx->engine);

    /* 检查是否完成/中止 */
    seq_state_t state = seq_engine_get_state(&sctx->engine);
    if (state == SEQ_STATE_DONE || state == SEQ_STATE_ABORTED) {
        if (state == SEQ_STATE_ABORTED && sctx->abort_reason == 0U) {
            /* 引擎内部中止（步骤超时等），与"用户/他键中断"区分开 */
            sctx->abort_reason = 3U;
        }
        sctx->current_onekey = 0U;
        control_object_process(&g_seq_obj.base, SEQ_OBJ_EVT_FINISH);
    }
}

/* ============================================================
 * 查询接口
 * ============================================================ */
seq_ctrl_context_t* seq_control_get_context(void)
{
    return &g_seq_obj.ctx;
}

/* ============================================================
 * FinSH 调试命令
 * ============================================================ */
#ifdef RT_USING_FINSH
#include <finsh.h>

/* ------------------------------------------------------------
 * seq_status —— 序列观测（P2 扩展）
 * 覆盖：状态 / 步骤 / 步骤名 / 步骤已耗时 / 当前一键 / 中止原因 / 计数
 * ------------------------------------------------------------ */
static void cmd_seq_status(int argc, char **argv)
{
    seq_ctrl_context_t *sctx = &g_seq_obj.ctx;
    seq_context_t      *eng  = &sctx->engine;
    uint32_t            step = seq_engine_get_step(eng);
    uint32_t            elapsed = 0U;
    const char         *step_name = "-";

    (void)argc;
    (void)argv;

    /* 【P3】非运行态不显示步骤名：abort/DONE 之后 config 与 step 仍是旧值，
     * 直接打印 "Raise to +25" 会让人误判为"还在跑这一步"。 */
    if ((seq_engine_is_active(eng) != 0U) &&
        (eng->config != RT_NULL) && (step < eng->config->step_count)) {
        step_name = eng->config->steps[step].name;
    }
    if ((seq_engine_is_active(eng) != 0U) && (eng->step_entered != 0U)) {
        elapsed = (rt_tick_get() - eng->step_start_tick) * 1000U / RT_TICK_PER_SECOND;
    }

    rt_kprintf("Sequence state: %d\n", (int)seq_engine_get_state(eng));
    /* 【P7 / B-4】显示当前生效的配置名与步数：回位有 home(5 步) 与
     * home_at_zero(2 步) 两份，光看 "当前步 0" 分不出走的是哪一份。
     * 非运行态保留上次值，便于事后确认刚跑完的是哪条序列。 */
    if (eng->config != RT_NULL) {
        rt_kprintf("Sequence:       %s (%lu steps)\n",
                   eng->config->name,
                   (unsigned long)eng->config->step_count);
    }
    rt_kprintf("Current step:   %lu\n", (unsigned long)step);
    rt_kprintf("Step name:      %s\n", step_name);
    rt_kprintf("Step elapsed:   %lu ms\n", (unsigned long)elapsed);
    rt_kprintf("One-key:        %s\n",
               seq_control_onekey_name(sctx->current_onekey));
    rt_kprintf("Abort reason:   %s\n", seq_abort_name(sctx->abort_reason));
    rt_kprintf("Force cond:     %u\n",
               (unsigned)seq_engine_get_force_cond());
    rt_kprintf("Requests: %lu, Rejects: %lu (dup=%lu precond=%lu limit=%lu multi=%lu)\n",
               (unsigned long)sctx->request_count,
               (unsigned long)sctx->reject_count,
               (unsigned long)sctx->rej_dup,
               (unsigned long)sctx->rej_precond,
               (unsigned long)sctx->rej_limit,
               (unsigned long)sctx->rej_multi);
    rt_kprintf("Complete: %lu, Abort: %lu\n",
               (unsigned long)eng->complete_count,
               (unsigned long)eng->abort_count);
}
MSH_CMD_EXPORT_ALIAS(cmd_seq_status, seq_status, show sequence status);

/* ------------------------------------------------------------
 * t_seq_run <mid|rear|home|mid+rear+home|none> —— 绕过按键直接启动序列
 * 用途：分离"事件源故障"与"引擎故障"。仍走 seq_control_trigger_multi()，
 *       因此前置条件（启车/模式/无故障/无手动）与 P3 仲裁（多键/超限）照常生效。
 * ------------------------------------------------------------ */
static const char *seq_ret_name(int ret)
{
    switch (ret) {
        case SEQ_RET_OK:      return "OK";
        case SEQ_RET_DUP:     return "ignored (same key running)";
        case SEQ_RET_PRECOND: return "rejected (precondition)";
        case SEQ_RET_LIMIT:   return "denied (limit)";
        case SEQ_RET_MULTI:   return "denied (multi-key)";
        default:              return "?";
    }
}

static void cmd_seq_run(int argc, char **argv)
{
    uint8_t mid = 0U, rear = 0U, home = 0U;
    int     i;
    int     ret;

    if (sys_cap_require(SYS_CAP_DIAG_WRITE, "t_seq_run") == 0U) { return; }

    if (argc < 2) {
        rt_kprintf("usage: t_seq_run <mid|rear|home|mid+rear+home|none>\n");
        return;
    }

    /* 支持 'mid'、'mid+rear+home'、'none'（= 三键同按），也支持分开写 'mid rear home' */
    for (i = 1; i < argc; i++) {
        if (strstr(argv[i], "none") != NULL) { mid = rear = home = 1U; }
        if (strstr(argv[i], "mid")  != NULL) { mid  = 1U; }
        if (strstr(argv[i], "rear") != NULL) { rear = 1U; }
        if (strstr(argv[i], "home") != NULL) { home = 1U; }
    }

    if ((uint8_t)(mid + rear + home) == 0U) {
        rt_kprintf("usage: t_seq_run <mid|rear|home|mid+rear+home|none>\n");
        return;
    }

    ret = seq_control_trigger_multi(mid, rear, home);
    rt_kprintf("[Seq] t_seq_run -> %d: %s\n", ret, seq_ret_name(ret));
}
MSH_CMD_EXPORT_ALIAS(cmd_seq_run, t_seq_run, start one-key sequence directly);

/* ------------------------------------------------------------
 * t_seq_case <1..5> —— P3 预置仲裁场景
 *   1 同键重复（期望 -1 ignored）
 *   2 异键中断（期望 interrupts 并重启）
 *   3 三键同按（期望 -4 multi-key）
 *   4 超右（期望 rear -3 limit / mid 0 OK）   ← 跑前先 t_sim swing 2180
 *   5 超左（期望三种一键都 0 OK）             ← 跑前先 t_sim swing -1030
 * 每个场景跑完请用 seq_abort 收尾，避免残留一条在跑。
 * ------------------------------------------------------------ */
static void cmd_seq_case(int argc, char **argv)
{
    int     c = (argc >= 2) ? atoi(argv[1]) : 0;
    int32_t angle = swing_angle_get_value(&g_swing_angle);
    int     ret;

    if (sys_cap_require(SYS_CAP_DIAG_WRITE, "t_seq_case") == 0U) { return; }

    rt_kprintf("[case] angle=%ld(x10) force=%u state=%d one-key=%s\n",
               (long)angle,
               (unsigned)seq_engine_get_force_cond(),
               (int)seq_engine_get_state(&g_seq_obj.ctx.engine),
               seq_control_onekey_name(g_seq_obj.ctx.current_onekey));

    switch (c) {
    case 1:
        rt_kprintf("[case] 1: mid twice -> 2nd should be IGNORED\n");
        ret = seq_control_trigger(1U); rt_kprintf("[case] 1st -> %d: %s\n", ret, seq_ret_name(ret));
        ret = seq_control_trigger(1U); rt_kprintf("[case] 2nd -> %d: %s\n", ret, seq_ret_name(ret));
        break;

    case 2:
        rt_kprintf("[case] 2: mid then rear -> rear should INTERRUPT mid\n");
        ret = seq_control_trigger(1U); rt_kprintf("[case] mid  -> %d: %s\n", ret, seq_ret_name(ret));
        ret = seq_control_trigger(2U); rt_kprintf("[case] rear -> %d: %s\n", ret, seq_ret_name(ret));
        break;

    case 3:
        rt_kprintf("[case] 3: mid+rear+home -> should be MULTI denied\n");
        ret = seq_control_trigger_multi(1U, 1U, 1U);
        rt_kprintf("[case] multi -> %d: %s\n", ret, seq_ret_name(ret));
        break;

    case 4:
        rt_kprintf("[case] 4: over right -> rear DENIED, mid ALLOWED\n");
        ret = seq_control_trigger(2U); rt_kprintf("[case] rear -> %d: %s\n", ret, seq_ret_name(ret));
        ret = seq_control_trigger(1U); rt_kprintf("[case] mid  -> %d: %s\n", ret, seq_ret_name(ret));
        break;

    case 5:
        rt_kprintf("[case] 5: over left -> all three ALLOWED\n");
        ret = seq_control_trigger(1U); rt_kprintf("[case] mid  -> %d: %s\n", ret, seq_ret_name(ret));
        ret = seq_control_trigger(2U); rt_kprintf("[case] rear -> %d: %s\n", ret, seq_ret_name(ret));
        ret = seq_control_trigger(3U); rt_kprintf("[case] home -> %d: %s\n", ret, seq_ret_name(ret));
        break;

    default:
        rt_kprintf("usage: t_seq_case <1..5>\n");
        rt_kprintf("  1=dup 2=interrupt 3=multi-key 4=over-right 5=over-left\n");
        break;
    }
}
MSH_CMD_EXPORT_ALIAS(cmd_seq_case, t_seq_case, one-key arbitration scenarios);

/* ------------------------------------------------------------
 * t_seq_step [n] —— 手动推进 n 次引擎更新（默认 1）
 * 用途：逐拍观察步骤条件判定，不依赖 50ms tick 的节奏。
 * ------------------------------------------------------------ */
static void cmd_seq_step(int argc, char **argv)
{
    seq_ctrl_context_t *sctx = &g_seq_obj.ctx;
    int n = (argc >= 2) ? atoi(argv[1]) : 1;
    int i;

    if (sys_cap_require(SYS_CAP_DIAG_WRITE, "t_seq_step") == 0U) { return; }

    if (n <= 0) { n = 1; }
    if (n > 10) { n = 10; }

    for (i = 0; i < n; i++) {
        rt_kprintf("[seq-step] %d/%d before: state=%d step=%lu\n",
                   i + 1, n,
                   (int)seq_engine_get_state(&sctx->engine),
                   (unsigned long)seq_engine_get_step(&sctx->engine));
        seq_engine_update(&sctx->engine);
        rt_kprintf("[seq-step] %d/%d after : state=%d step=%lu\n",
                   i + 1, n,
                   (int)seq_engine_get_state(&sctx->engine),
                   (unsigned long)seq_engine_get_step(&sctx->engine));
    }
}
MSH_CMD_EXPORT_ALIAS(cmd_seq_step, t_seq_step, advance sequence n times);

/* ------------------------------------------------------------
 * t_seq_auto <0|1> —— 台架：角度类完成条件直接判满足
 * 只影响 LIFT_GE/LE/NEAR 与 SWING_NEAR；TIME/NONE/CUSTOM 保持真实。
 * ------------------------------------------------------------ */
static void cmd_seq_auto(int argc, char **argv)
{
    int en = (argc >= 2) ? atoi(argv[1]) : -1;

    if (sys_cap_require(SYS_CAP_DIAG_WRITE, "t_seq_auto") == 0U) { return; }

    if ((en != 0) && (en != 1)) {
        rt_kprintf("usage: t_seq_auto <0|1>  (now %u)\n",
                   (unsigned)seq_engine_get_force_cond());
        return;
    }

    seq_engine_set_force_cond((uint8_t)en);
    rt_kprintf("[Seq] force cond -> %d\n", en);
}
MSH_CMD_EXPORT_ALIAS(cmd_seq_auto, t_seq_auto, bench: force angle conditions);

static void cmd_seq_abort(int argc, char **argv)
{
    seq_ctrl_context_t *sctx = &g_seq_obj.ctx;

    (void)argc;
    (void)argv;

    sctx->abort_reason = 2U;
    seq_engine_abort(&sctx->engine);
    sctx->current_onekey = 0U;
    rt_kprintf("Sequence aborted\n");
}
MSH_CMD_EXPORT_ALIAS(cmd_seq_abort, seq_abort, abort current sequence);
#endif
