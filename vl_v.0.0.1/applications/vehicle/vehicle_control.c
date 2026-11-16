/**
 * @file    vehicle_control.c
 * @brief   整车四态单机实现（2026-11-14 P2 / 设计基线 V3.0 §5）
 *
 * 【需求依据】需求基线 §4.5（V1.9，用户 2026-11-15 20:08 裁定）
 *   Q-A：rpm > 400 **连续 500ms** 或 (电压有效 && V > 13.0V) **连续 500ms** ⇒ STARTED
 *        ⇒ **两条件各自独立软件计时，任一达标即启车**；不再 OR 合并共用单一计时器。
 *        反例自证：rpm 满足 300ms 后消失、V 再满足 300ms ⇒ 两个计时器各 300ms，不启车。
 *        实现：按上一轮建议沿用 50ms tick 的两个独立累加器（零新增线程、与 E-4 一致）。
 *   Q-B：CAN 总线静默（无任何帧触发解析线程查表，Q-D1）/ 电压采集缺失，
 *        两条件 OR，连续 2s ⇒ FAULT（仅手动）；故障位可扩展（Q-D3 解读 A）
 *   Q-C：钥匙下电去抖 500ms 窗口内动作放行；确认后门禁由 app_main 处理
 *   B-1：STARTED 保持至钥匙下电，唯一清除途径 = acc_off → OFF
 *
 * 【单一真值源】状态只存在 base.sm 里，ctx 不保存 state 副本（D-8 / 缺陷基线 §6）。
 * 【运行副本 R6】判据参数只在 init 从配置表读一次，poll 内不直读配置表。
 * 【同步投递】VEH_EVT_* 一律经 control_object_process()，禁止 post_event。
 */

#include "vehicle_control.h"
#include "vehicle_trans_table.h"
#include "app_event.h"
#include "../service/sys_voltage.h"
#include "../service/key_state.h"
#include "../config/sys_config.h"
#include "../CAN/ut_can_handlers.h"
#include "../CAN/ut_can_scheduler.h"   /* Q-D1: can_is_bus_silent() */
#include "framework/state_engine/state_engine.h"
#include "framework/state_engine/sm_assert.h"
#include "framework/port/osal.h"

#include <string.h>

/* ============================================================
 * 全局对象实例（黑板唯一真值源）
 * ============================================================ */
vehicle_control_t g_vehicle_obj;

/* ============================================================
 * 配置属性 → 运行副本（R6 / Q-84）
 * ============================================================
 * sys_config 在 BOOT_LV_STORAGE(10)，本模块在 BOOT_LV_SERVICE(60)，
 * 实测 boot_table.c:161 / :220 ⇒ 装载时配置表已就绪。
 * 读取失败保持 ROM 默认值，不静默改行为。
 *
 * ⚠️ CFG_ID_VEH_EXIT_* 三项按 B-1 作废，本函数**不读取**。
 * ============================================================ */
int vehicle_cfg_load(void)
{
    uint32_t v;

    if (sys_config_get_u32(CFG_ID_VEH_ENTER_RPM, &v) == 0) {
        g_vehicle_cfg.enter_rpm = (uint16_t)v;
    }
    if (sys_config_get_u32(CFG_ID_VEH_ENTER_VOLT_DV, &v) == 0) {
        g_vehicle_cfg.enter_volt_dv = (uint16_t)v;
    }
    if (sys_config_get_u32(CFG_ID_VEH_ENTER_MS, &v) == 0) {
        g_vehicle_cfg.enter_ms = (uint16_t)v;
    }
    if (sys_config_get_u32(CFG_ID_VEH_FAULT_MS, &v) == 0) {
        g_vehicle_cfg.fault_ms = (uint16_t)v;
    }
    if (sys_config_get_u32(CFG_ID_VEH_CAN_SILENT_MS, &v) == 0) {
        g_vehicle_cfg.can_silent_ms = (uint16_t)v;
    }
    return 0;
}

/* ============================================================
 * 初始化并注册
 * ============================================================ */
int vehicle_control_register(ctrl_manager_t *mgr)
{
    obj_err_t ret;

    SM_ASSERT_PTR(mgr);

    /* 1. 初始化运行属性 */
    memset(&g_vehicle_obj.ctx, 0, sizeof(vehicle_context_t));

    /* 2. 从配置表装载判据到运行副本（只读一次） */
    (void)vehicle_cfg_load();

    /* 3. 绑定状态机指针（必须先绑定，再调 init） */
    g_vehicle_obj.base.sm = &g_vehicle_obj.sm_instance;

    /* 4. 单线程运行时配置 */
    ctrl_obj_runtime_cfg_t runtime = {
        .use_thread  = 0,
        .thread_prio = 0,
        .stack_size  = 0,
        .mq_size     = 0,
    };

    /* 5. 初始化控制对象（走基类） */
    ret = control_object_init(&g_vehicle_obj.base,
                              "VehicleControl",
                              CTRL_OBJ_TYPE_VEHICLE,
                              &g_vehicle_config,
                              &g_vehicle_obj.ctx,
                              &runtime);
    if (ret != OBJ_OK) {
        osal_log("[VEH] control_object_init failed (%ld)\n", (long)ret);
        return -1;
    }

    /* 6. 注册到管理器 */
    ret = ctrl_manager_register(mgr, &g_vehicle_obj.base);
    if (ret != 0) {
        osal_log("[VEH] ctrl_manager_register failed (%ld)\n", (long)ret);
        return -1;
    }

    osal_log("[VEH] init (4-state: OFF/ACC_ON/STARTED/FAULT)\n");
    osal_log("[VEH]   Q-A start : rpm>%u OR volt>%u.%uV, EACH held %ums (independent timers)\n",
             (unsigned)g_vehicle_cfg.enter_rpm,
             (unsigned)(g_vehicle_cfg.enter_volt_dv / 10U),
             (unsigned)(g_vehicle_cfg.enter_volt_dv % 10U),
             (unsigned)g_vehicle_cfg.enter_ms);
    osal_log("[VEH]   Q-B fault : CAN-timeout / volt-missing, held %ums\n",
             (unsigned)g_vehicle_cfg.fault_ms);

    return 0;
}

/* ============================================================
 * 周期轮询（50ms tick 调用）
 *
 * 判定优先级（自上而下，命中即 return）：
 *   ① 钥匙下电（B-1 唯一清除途径）→ 任意非 OFF 态 → OFF
 *   ② OFF + 钥匙上电 → ACC_ON
 *   ③ 故障信号连续 fault_ms → FAULT（Q-B）
 *   ④ ACC_ON + 转速/电压任一独立计时达 enter_ms → STARTED（Q-A V1.9）
 *   ⑤ STARTED 保持 / FAULT 保持（均不自动退出）
 *
 * 【Q-C】钥匙下电去抖由 acc_state 负责：去抖窗口内 acc_is_on() 仍为 1，
 *   本函数不会提前清状态，动作自然放行；确认后返回 0 才进入分支 ①。
 * ============================================================ */
void vehicle_poll(void)
{
    vehicle_context_t *ctx = &g_vehicle_obj.ctx;
    State_t state;
    uint16_t rpm;
    uint16_t dv;
    uint8_t  volt_ok;
    uint8_t  acc_on;

    /* ---- 采样：一律读源头，不读副本 ---- */
    rpm     = can_get_engine_rpm();      /* 已带 100ms 超时归零 */
    dv      = sys_voltage_get_dv();
    volt_ok = sys_voltage_is_ready();
    acc_on  = acc_is_on();               /* 去抖后的稳定值（Q-C） */

    ctx->last_rpm     = rpm;
    ctx->last_dv      = dv;
    ctx->last_volt_ok = volt_ok;

    /* [TEMP-E6] 只读观测快照：记录「发动机仍在转」这一物理事实。
     * 硬边界：不参与状态迁移 / 门禁 / fault_word（X-1 待后续裁定）。 */
    ctx->engine_running = (rpm > 0U) ? 1U : 0U;

    /* ---- 故障信号汇聚（只读镜像既有信号，不新建故障源，设计基线 §13A.2）----
     * 【Q-D3 解读 A / 2026-11-14 16:49】当前两条件，OR 关系：
     *   bit0 CAN 总线静默（Q-D1：解析线程查表心跳 + Q-D2 500ms 门限）
     *   bit1 电压采集缺失（sys_voltage_is_ready()==0）
     * ⚠️ 扩展点：后续新增条件，只需在 vehicle_control.h 加一个 VEH_FAULT_* 位，
     *    并在本段加一行 |= —— 判定链、去抖、FAULT 迁移、门禁全部无需改动。
     *    不补判 dv==0（解读 A 未采纳解读 B；G-9 仍按既有缺陷登记）。 */
    ctx->fault_word = 0UL;
    if (can_is_bus_silent(g_vehicle_cfg.can_silent_ms) != 0U) {
        ctx->fault_word |= VEH_FAULT_CAN_SILENT;
    }
    if (volt_ok == 0U) {
        ctx->fault_word |= VEH_FAULT_VOLT_MISSING;
    }

    /* SWR-N-002：2s 确认 / 2s 恢复 —— 信号中断即清零重计时 */
    if (ctx->fault_word != 0UL) {
        ctx->fault_cond_ms = (uint16_t)(ctx->fault_cond_ms + g_vehicle_cfg.poll_ms);
    } else {
        ctx->fault_cond_ms = 0U;
    }

    state = StateMachine_GetState(&g_vehicle_obj.sm_instance);

    /* ---- ① 钥匙下电：B-1 唯一清除途径，FAULT 也由此回 OFF（E-3）----
     * ⚠️ 2026-11-14 16:49 修正：fault_cond_ms 在上面「无条件累加」，
     *    若钥匙未上电期间总线一直静默（can_silent_ms=500ms 必然触发），
     *    计时器会被「预充值」；钥匙一拧进 ACC_ON 就会在 ③ 立刻进 FAULT。
     *    ⇒ 钥匙未上电时**无条件清零**，让故障 2s 计时从工作态重新开始。 */
    if (acc_on == 0U) {
        ctx->rpm_cond_ms   = 0U;    /* Q-A V1.9：两个独立计时器都清零 */
        ctx->volt_cond_ms  = 0U;
        ctx->fault_cond_ms = 0U;
        if (state != VEH_STATE_OFF) {
            control_object_process(&g_vehicle_obj.base, VEH_EVT_ACC_OFF);
        }
        return;
    }

    /* ---- ② OFF + 钥匙已上电 → ACC_ON ---- */
    if (state == VEH_STATE_OFF) {
        ctx->rpm_cond_ms   = 0U;
        ctx->volt_cond_ms  = 0U;
        ctx->fault_cond_ms = 0U;    /* 上电起点，故障计时从头开始 */
        control_object_process(&g_vehicle_obj.base, VEH_EVT_ACC_ON);
        return;
    }

    /* ---- ③ Q-B 故障确认（FAULT 态本身不重复投递）---- */
    if (state != VEH_STATE_FAULT) {
        if (ctx->fault_cond_ms >= g_vehicle_cfg.fault_ms) {
            ctx->rpm_cond_ms  = 0U;
            ctx->volt_cond_ms = 0U;
            control_object_process(&g_vehicle_obj.base, VEH_EVT_FAULT);
            return;
        }
    }

    /* ---- ④ Q-A 启车：ACC_ON 才计时；两条件独立软件计时（V1.9）----
     * 【用户 2026-11-15 20:08 裁定】rpm>400 与 V>13.0V **各用独立计时器**，
     * 任一连续满足 enter_ms 即启车；某条件中断只清它自己的计时器。
     * 【实现】按上一轮建议沿用 50ms tick 的两个 uint16_t 累加器（软件计时），
     * 不引入 rt_timer / 新线程；周期判定仍在上游，不进引擎（E-1 / E-4）。
     * ⚠️ enter_ms 两个计时器共用同一阈值（按建议取值，待复核）。
     * ⚠️ 电压条件保留原有 volt_ok 前置（采样有效性，非第三个启车条件）——
     *    与需求基线 §4.5「仅两项」的措辞差异已登记，本批不自行改（R-01）。 */
    if (state == VEH_STATE_ACC_ON) {
        if (rpm > g_vehicle_cfg.enter_rpm) {
            ctx->rpm_cond_ms = (uint16_t)(ctx->rpm_cond_ms + g_vehicle_cfg.poll_ms);
        } else {
            ctx->rpm_cond_ms = 0U;
        }

        if ((volt_ok != 0U) && (dv > g_vehicle_cfg.enter_volt_dv)) {
            ctx->volt_cond_ms = (uint16_t)(ctx->volt_cond_ms + g_vehicle_cfg.poll_ms);
        } else {
            ctx->volt_cond_ms = 0U;
        }

        if ((ctx->rpm_cond_ms  >= g_vehicle_cfg.enter_ms) ||
            (ctx->volt_cond_ms >= g_vehicle_cfg.enter_ms)) {
            ctx->rpm_cond_ms  = 0U;
            ctx->volt_cond_ms = 0U;
            control_object_process(&g_vehicle_obj.base, VEH_EVT_CONFIRM);
        }
        return;
    }

    /* ---- ⑤ STARTED（B-1 只进不退）/ FAULT（Q-B 仅手动）----
     * 两者都不因输入变化自动退出，只能由 ① 钥匙下电清除。 */
}

/* ============================================================
 * 查询接口 / 黑板只读
 * ============================================================ */
uint8_t vehicle_is_started(void)
{
    /* FAULT 返回 0 ⇒ 一键 / 离合被拒；手动四向由门 1 决定 ⇒ Q-B「仅手动」 */
    return (StateMachine_GetState(&g_vehicle_obj.sm_instance) == VEH_STATE_STARTED) ? 1U : 0U;
}

vehicle_state_t vehicle_state_get(void)
{
    return (vehicle_state_t)StateMachine_GetState(&g_vehicle_obj.sm_instance);
}

uint32_t vehicle_fault_word_get(void)
{
    return g_vehicle_obj.ctx.fault_word;
}

uint8_t vehicle_engine_running_get(void)
{
    return g_vehicle_obj.ctx.engine_running;   /* [TEMP-E6] */
}

vehicle_context_t *vehicle_control_get_context(void)
{
    return &g_vehicle_obj.ctx;
}

void vehicle_force_reset(void)
{
    g_vehicle_obj.ctx.rpm_cond_ms   = 0U;
    g_vehicle_obj.ctx.volt_cond_ms  = 0U;
    g_vehicle_obj.ctx.fault_cond_ms = 0U;
    control_object_process(&g_vehicle_obj.base, VEH_EVT_FORCE_RESET);
    osal_log("[VEH] force reset -> OFF\n");
}

/* ============================================================
 * 兼容入口（原 service/vehicle_state.c 的 vehicle_state_init）
 * ============================================================ */
int vehicle_state_init(void)
{
    extern ctrl_manager_t g_ctrl_mgr;
    return vehicle_control_register(&g_ctrl_mgr);
}

/* ============================================================
 * 状态名（调试用）
 * ============================================================ */
const char *vehicle_state_name(vehicle_state_t state)
{
    switch (state) {
    case VEH_STATE_OFF:      return "OFF";
    case VEH_STATE_ACC_ON:   return "ACC_ON";
    case VEH_STATE_STARTED:  return "STARTED";
    case VEH_STATE_FAULT:    return "FAULT";
    default:                 return "?";
    }
}

/* ============================================================
 * 调试命令（沿用 t_veh / t_veh_clr）
 * ============================================================ */
#ifdef RT_USING_FINSH
#include <finsh.h>
#include <rtthread.h>

static void t_vehicle(void)
{
    vehicle_context_t *ctx = &g_vehicle_obj.ctx;
    State_t st = StateMachine_GetState(&g_vehicle_obj.sm_instance);

    rt_kprintf("\n===== VEHICLE OBJECT (4-state, V3.0) =====\n");
    rt_kprintf("state     : %lu (%s)\n",
               (unsigned long)st, vehicle_state_name((vehicle_state_t)st));
    rt_kprintf("started   : %u   [FAULT => 0, manual-only]\n",
               (unsigned)vehicle_is_started());
    rt_kprintf("acc_on    : %u\n", (unsigned)acc_is_on());
    rt_kprintf("rpm_ms    : %u / %u   (Q-A V1.9 independent timer)\n",
               (unsigned)ctx->rpm_cond_ms, (unsigned)g_vehicle_cfg.enter_ms);
    rt_kprintf("volt_ms   : %u / %u   (Q-A V1.9 independent timer)\n",
               (unsigned)ctx->volt_cond_ms, (unsigned)g_vehicle_cfg.enter_ms);
    rt_kprintf("fault_ms  : %u / %u   (Q-B held)\n",
               (unsigned)ctx->fault_cond_ms, (unsigned)g_vehicle_cfg.fault_ms);
    rt_kprintf("fault_word: 0x%lX  (bit0=CAN-silent bit1=volt-missing, bit2-7 reserved)\n",
               (unsigned long)ctx->fault_word);
    rt_kprintf("can_silent: %u  (0=comm OK, 1=silent; limit %u ms, lookup heartbeat)\n",
               (unsigned)can_is_bus_silent(g_vehicle_cfg.can_silent_ms),
               (unsigned)g_vehicle_cfg.can_silent_ms);
    rt_kprintf("rpm       : %u   (enter>%u)\n",
               (unsigned)ctx->last_rpm, (unsigned)g_vehicle_cfg.enter_rpm);
    rt_kprintf("volt      : %u.%uV (enter>%u.%uV, ready=%u)\n",
               (unsigned)(ctx->last_dv / 10U), (unsigned)(ctx->last_dv % 10U),
               (unsigned)(g_vehicle_cfg.enter_volt_dv / 10U),
               (unsigned)(g_vehicle_cfg.enter_volt_dv % 10U),
               (unsigned)ctx->last_volt_ok);
    rt_kprintf("[TEMP-E6] engine_running : %u  (observation only, X-1 pending)\n",
               (unsigned)ctx->engine_running);
    rt_kprintf("counts    : started=%lu, off=%lu\n",
               (unsigned long)ctx->started_count,
               (unsigned long)ctx->off_count);
    rt_kprintf("==========================================\n\n");
}

static void t_vehicle_clr(void)
{
    vehicle_force_reset();
}

MSH_CMD_EXPORT_ALIAS(t_vehicle,     t_veh,     dump vehicle object state);
MSH_CMD_EXPORT_ALIAS(t_vehicle_clr, t_veh_clr, clear vehicle state);
#endif /* RT_USING_FINSH */
