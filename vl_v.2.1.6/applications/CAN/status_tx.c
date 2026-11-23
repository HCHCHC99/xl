/**
 * @file    status_tx.c
 * @brief   0x18FF11D1 控制器状态反馈帧组包实现
 * @note
 *   本机发送帧，方向：UTMotVCU → HMI，周期 100ms。
 *
 *   【P8 / 2026-11-23 重构】
 *   旧实现：9 个字段各由一个 CAN_EVT_STA_* 事件单独刷新。
 *   实际后果：只有 POWER / CALIB / VALVE / TEETH 四条有投递者，
 *   其余（comm / work_mode / encoder+motor / sensor / lift / angle）
 *   全工程零投递 —— 永远停在 reset 初值。屏上看到的是
 *   "角度恒 0°、电机恒停止、举升恒停止、模式恒待机、通信恒正常"。
 *
 *   现在：50ms tick 调 status_tx_refresh() 全量读 getter 填缓存。
 *   pack 仍只 memcpy，发送线程不访问别的模块（保持原有线程约束）。
 *   事件入口保留，内部等同一次 refresh，避免 app_main 里到处改。
 */

#include "status_tx.h"
#include "protocol_defs.h"
#include "hw_objects.h"
#include "can_tx_cfg.h"          /* CAN_TX_EN_STATUS 模块级开关 */
#include "ut_can_scheduler.h"    /* can_is_bus_silent */
#include "../sys_param/sys_param.h"
#include "../service/sys_voltage.h"   /* sys_voltage_is_uv()（§10.5-E）*/
#include "../config/sys_config.h"     /* CFG_ID_VEH_CAN_SILENT_MS */
#include "../swing/swing_control.h"   /* g_swing_obj / swing_control_get_state */
#include "../swing/swing_angle.h"     /* g_swing_angle / swing_angle_get_value */
#include "../lift/lift_control.h"     /* g_lift_obj / lift_control_get_state */
#include "../lift/lift_angle.h"       /* g_lift_angle / lift_angle_get_value */
#include "../sequence/seq_control.h"  /* seq_control_get_context / engine */
#include "../motor/motor_stall.h"     /* motor_stall_is_faulted */

#include <string.h>
#include <rtthread.h>

/* ============================================================
 * 本地缓存（refresh 写入，pack 读出）
 * ============================================================ */
static frame_0x18FF11D1_t s_tx;

/* 最近一次 refresh 读到的原始值（t_sta 打印"为什么是这个值"） */
static struct {
    int32_t swing_x10;
    int32_t lift_x10;
    uint8_t uv;
    uint8_t comm;
    uint8_t calibrated;
    uint8_t encoder_fault;
    uint8_t sensor_fault;
    uint8_t stall;
    uint8_t onekey;
    uint8_t hold_dir;
} s_diag;

/* ============================================================
 * 默认值：零位角度 + 全部正常
 * ============================================================ */
static void status_tx_reset_default(void)
{
    memset(s_tx.raw, 0, sizeof(s_tx.raw));
    memset(&s_diag, 0, sizeof(s_diag));

    /* 角度写零位（物理 0°），而非 0 原始值（那样是 -300° / -90°） */
    s_tx.bits.swing_angle_raw = (int16_t)STATUS_ANGLE_SWING_OFFSET;
    s_tx.bits.lift_angle_raw  = (uint8_t)STATUS_ANGLE_LIFT_OFFSET;

    s_tx.bits.power_voltage_sta = STA_POWER_NORMAL;
    s_tx.bits.calib_sta         = STA_CALIB_NONE;
    s_tx.bits.comm_sta          = 0U;
    s_tx.bits.work_mode         = STA_MODE_STANDBY;

    s_tx.bits.encoder_sta       = 0U;
    s_tx.bits.motor_sta         = STA_MOTOR_STOP;
    s_tx.bits.lift_sensor_sta   = 0U;
    s_tx.bits.lift_sta          = STA_LIFT_STOP;

    s_tx.bits.up_valve_sta      = STA_VALVE_NORMAL;
    s_tx.bits.down_valve_sta    = STA_VALVE_NORMAL;

    s_tx.bits.teeth_set_sta     = 0U;
    s_tx.bits.teeth_code        = 0U;
}

/* ============================================================
 * 字段推导
 * ============================================================ */

/* Byte0 bit5-7 工作模式（TC-ONE-017）
 *   这与 MAINT/WORK 那套系统模式不是一个维度：这里回答的是
 *   "控制器当前在做什么动作"，不是"当前权限到哪一级"。
 *   优先级：一键 > 手动 > 待机。 */
static uint8_t derive_work_mode(void)
{
    seq_ctrl_context_t *sctx = seq_control_get_context();
    uint8_t onekey = 0U;

    if ((sctx != RT_NULL) && (seq_engine_is_active(&sctx->engine) != 0U)) {
        onekey = sctx->current_onekey;
    }
    s_diag.onekey = onekey;

    if (onekey == 3U) {
        return STA_MODE_ONKEY_HOME;      /* 011 一键回位 */
    }
    if ((onekey == 1U) || (onekey == 2U)) {
        return STA_MODE_ONKEY_EXPAND;    /* 010 一键展开 */
    }

    /* 手动：任一轴处于运行态，或手动键正被按住 */
    {
        swing_state_t hs = swing_control_get_state();
        lift_state_t  ps = lift_control_get_state();

        if ((hs == H_STATE_LEFT_RUNNING) || (hs == H_STATE_RIGHT_RUNNING) ||
            (ps == P_STATE_UP_RUNNING)   || (ps == P_STATE_DOWN_RUNNING) ||
            (g_swing_obj.ctx.left_sw_pressed != 0U) ||
            (g_swing_obj.ctx.right_sw_pressed != 0U) ||
            (g_lift_obj.ctx.up_sw_pressed != 0U) ||
            (g_lift_obj.ctx.down_sw_pressed != 0U)) {
            return STA_MODE_MANUAL;      /* 001 手动 */
        }
    }

    return STA_MODE_STANDBY;             /* 000 待机 */
}

/* Byte1 bit1-3 电机状态
 *   堵转优先；转向取 swing 状态机。协议里的 011「开路」需要电机三相
 *   开路检测，本工程没有该数据源，留 TODO。
 *   注意 D-1：限位拦截时 swing 状态会短暂残留 RUNNING，已在
 *   swing_actions.c 修正（拦截即投停止事件）。 */
static uint8_t derive_motor_sta(void)
{
    swing_state_t st = swing_control_get_state();

    s_diag.stall = motor_stall_is_faulted();
    if (s_diag.stall != 0U) {
        return STA_MOTOR_STALL;          /* 100 堵转 */
    }
    if (st == H_STATE_LEFT_RUNNING) {
        return STA_MOTOR_LEFT;           /* 001 左转 */
    }
    if (st == H_STATE_RIGHT_RUNNING) {
        return STA_MOTOR_RIGHT;          /* 010 右转 */
    }
    return STA_MOTOR_STOP;               /* 000 停止 */
}

/* Byte1 bit5-7 举升状态
 *   LIMIT_HOLDING 时阀仍在通电（P6 的 5s 保压），按 hold_direction
 *   如实报 上升/下降，而不是报"停止"——见 §15.12 的口径说明。 */
static uint8_t derive_lift_sta(void)
{
    lift_state_t st = lift_control_get_state();

    if (st == P_STATE_UP_RUNNING) {
        return STA_LIFT_UP;
    }
    if (st == P_STATE_DOWN_RUNNING) {
        return STA_LIFT_DOWN;
    }
    if (st == P_STATE_LIMIT_HOLDING) {
        s_diag.hold_dir = (uint8_t)(g_lift_obj.ctx.hold_direction != 0U ? 1U : 0U);
        return (s_diag.hold_dir != 0U) ? STA_LIFT_UP : STA_LIFT_DOWN;
    }
    return STA_LIFT_STOP;
}

/* ============================================================
 * 全量刷新（P8）
 * ============================================================ */
void status_tx_refresh(void)
{
    int32_t silent_ms = 500;
    int32_t ha, la;

    /* ----- Byte0 ----- */
    s_diag.uv = sys_voltage_is_uv();
    s_tx.bits.power_voltage_sta = (s_diag.uv != 0U) ? STA_POWER_UNDER : STA_POWER_NORMAL;

    s_diag.calibrated = (uint8_t)(g_sys_params.is_calibrated != 0U ? 1U : 0U);
    s_tx.bits.calib_sta = (s_diag.calibrated != 0U) ? STA_CALIB_DONE : STA_CALIB_UNDONE;

    if (sys_config_get_i32(CFG_ID_VEH_CAN_SILENT_MS, &silent_ms) != 0) {
        silent_ms = 500;
    }
    s_diag.comm = can_is_bus_silent((uint32_t)silent_ms);
    s_tx.bits.comm_sta = (s_diag.comm != 0U) ? 1U : 0U;

    s_tx.bits.work_mode = derive_work_mode();

    /* ----- Byte1 ----- */
    s_diag.encoder_fault = (uint8_t)(g_swing_obj.ctx.encoder_fault != 0U ? 1U : 0U);
    s_tx.bits.encoder_sta = s_diag.encoder_fault;
    s_tx.bits.motor_sta   = derive_motor_sta();

    s_diag.sensor_fault = (uint8_t)(g_lift_obj.ctx.sensor_fault != 0U ? 1U : 0U);
    s_tx.bits.lift_sensor_sta = s_diag.sensor_fault;
    s_tx.bits.lift_sta        = derive_lift_sta();

    /* ----- Byte2：阀故障码（0 正常 / 1 开路 / 2 短路） -----
     * 用 P4 去抖**确认后**的码，不用 CAN 原始码：
     * 否则 HMI 会跟着每一帧抖动（红一下又绿），去抖就白做了。
     * 去抖窗口内按"正常"报，与举升状态机、面板灯口径一致。 */
    {
        lift_vfb_state_t vfb;
        uint8_t i;

        lift_control_valve_fb_get(&vfb);
        i = 0U;
        s_tx.bits.up_valve_sta = (vfb.ch[i].latched != 0U)
                                 ? (uint8_t)(vfb.ch[i].raw & 0x03U) : 0U;
        i = 1U;
        s_tx.bits.down_valve_sta = (vfb.ch[i].latched != 0U)
                                   ? (uint8_t)(vfb.ch[i].raw & 0x03U) : 0U;
    }

    /* ----- Byte3：机型 ----- */
    s_tx.bits.teeth_set_sta = (g_sys_params.teeth_type != TEETH_UNSET) ? 1U : 0U;
    switch (g_sys_params.teeth_type) {
    case TEETH_14: s_tx.bits.teeth_code = 1U; break;
    case TEETH_11: s_tx.bits.teeth_code = 2U; break;
    case TEETH_13:
    default:       s_tx.bits.teeth_code = 0U; break;
    }

    /* ----- Byte5-7：角度（0.1° → 原始值） -----
     * raw = 物理角度(整度) + 偏移；四舍五入用就近取整，避免 -0.4° 变 0° 后
     * 再减 300 变成 299（差 1 度）引起的边界抖动。 */
    ha = swing_angle_get_value(&g_swing_angle);
    la = lift_angle_get_value(&g_lift_angle);
    s_diag.swing_x10 = ha;
    s_diag.lift_x10  = la;

    {
        int32_t hs = (ha >= 0) ? ((ha + 5) / 10) : ((ha - 5) / 10);
        int32_t ls = (la >= 0) ? ((la + 5) / 10) : ((la - 5) / 10);

        s_tx.bits.swing_angle_raw = (int16_t)(hs + STATUS_ANGLE_SWING_OFFSET);
        s_tx.bits.lift_angle_raw  = (uint8_t)(ls + STATUS_ANGLE_LIFT_OFFSET);
    }
}

/* ============================================================
 * 事件入口：统一转成一次全量刷新
 * ============================================================ */
void status_tx_on_event(app_event_t event)
{
    switch (event) {
    case CAN_EVT_STA_POWER:
    case CAN_EVT_STA_CALIB:
    case CAN_EVT_STA_COMM:
    case CAN_EVT_STA_MODE:
    case CAN_EVT_STA_MOTOR:
    case CAN_EVT_STA_LIFT_SEN:
    case CAN_EVT_STA_LIFT:
    case CAN_EVT_STA_VALVE:
    case CAN_EVT_STA_TEETH:
    case CAN_EVT_STA_ANGLE:
        status_tx_refresh();
        break;

    default:
        break;
    }
}

/* ============================================================
 * 组包（can_tx 发送线程调用）
 * ============================================================ */
void status_tx_pack(can_tx_frame_t *frame)
{
    if (frame == RT_NULL) {
        return;
    }
    memcpy(frame->data, s_tx.raw, 8);
}

/* ============================================================
 * 名称表（t_sta 打印用）
 * ============================================================ */
static const char *power_name(uint8_t v)
{
    switch (v) {
        case STA_POWER_NORMAL: return "normal";
        case STA_POWER_UNDER:  return "undervoltage";
        case STA_POWER_OVER:   return "overvoltage";
        default:               return "rsvd";
    }
}

static const char *calib_name(uint8_t v)
{
    switch (v) {
        case STA_CALIB_NONE:   return "none";
        case STA_CALIB_DONE:   return "calibrated";
        case STA_CALIB_UNDONE: return "NOT-calibrated";
        default:               return "rsvd";
    }
}

static const char *mode_name(uint8_t v)
{
    switch (v) {
        case STA_MODE_STANDBY:      return "standby";
        case STA_MODE_MANUAL:       return "manual";
        case STA_MODE_ONKEY_EXPAND: return "onekey-expand";
        case STA_MODE_ONKEY_HOME:   return "onekey-home";
        default:                    return "rsvd";
    }
}

static const char *motor_name(uint8_t v)
{
    switch (v) {
        case STA_MOTOR_STOP:  return "stop";
        case STA_MOTOR_LEFT:  return "left";
        case STA_MOTOR_RIGHT: return "right";
        case STA_MOTOR_OPEN:  return "open";
        case STA_MOTOR_STALL: return "stall";
        default:              return "rsvd";
    }
}

static const char *lift_name(uint8_t v)
{
    switch (v) {
        case STA_LIFT_STOP: return "stop";
        case STA_LIFT_UP:   return "up";
        case STA_LIFT_DOWN: return "down";
        default:            return "rsvd";
    }
}

static const char *valve_name(uint8_t v)
{
    switch (v) {
        case STA_VALVE_NORMAL: return "normal";
        case STA_VALVE_OPEN:   return "open";
        case STA_VALVE_SHORT:  return "short";
        default:               return "rsvd";
    }
}

static const char *teeth_name(uint8_t v)
{
    switch (v) {
        case 0U:  return "13T";
        case 1U:  return "14T";
        case 2U:  return "11T";
        default:  return "?";
    }
}

/* 0.1° 拆成 "整数.小数"（本工程日志禁用 %f，见 任务清单 §11.5） */
static void print_x10(const char *label, int32_t x10)
{
    int32_t av = (x10 < 0) ? -x10 : x10;

    rt_kprintf("  %s = %s%ld.%ld deg\n", label,
               (x10 < 0) ? "-" : "",
               (long)(av / 10), (long)(av % 10));
}

/* ============================================================
 * 打印（t_sta 调用）
 * ============================================================ */
void status_tx_dump(void)
{
    /* 先刷一次：保证打印的是"现在"，而不是上次 tick 的残留 */
    status_tx_refresh();

    rt_kprintf("\n========== CAN TX: status ==========\n");
    rt_kprintf("id       : 0x%08X\n", (unsigned)STATUS_FRAME_ID);
    rt_kprintf("period   : %u ms (phase %u ms)\n",
               (unsigned)STATUS_FRAME_PERIOD_MS, (unsigned)STATUS_FRAME_PHASE_MS);
#if CAN_TX_EN_STATUS
    rt_kprintf("send     : ON  (CAN_TX_EN_STATUS=1)\n");
#else
    rt_kprintf("send     : OFF (CAN_TX_EN_STATUS=0, bus silent; dump still valid)\n");
#endif
    rt_kprintf("raw      : %02X %02X %02X %02X %02X %02X %02X %02X\n",
               s_tx.raw[0], s_tx.raw[1], s_tx.raw[2], s_tx.raw[3],
               s_tx.raw[4], s_tx.raw[5], s_tx.raw[6], s_tx.raw[7]);

    rt_kprintf("--- decode ---\n");
    rt_kprintf("B0 power=%u(%s) calib=%u(%s) comm=%u(%s) mode=%u(%s)\n",
               (unsigned)s_tx.bits.power_voltage_sta,
               power_name(s_tx.bits.power_voltage_sta),
               (unsigned)s_tx.bits.calib_sta,
               calib_name(s_tx.bits.calib_sta),
               (unsigned)s_tx.bits.comm_sta,
               (s_tx.bits.comm_sta != 0U) ? "FAULT" : "ok",
               (unsigned)s_tx.bits.work_mode,
               mode_name(s_tx.bits.work_mode));

    rt_kprintf("B1 enc=%u(%s) motor=%u(%s) sensor=%u(%s) lift=%u(%s)\n",
               (unsigned)s_tx.bits.encoder_sta,
               (s_tx.bits.encoder_sta != 0U) ? "FAULT" : "ok",
               (unsigned)s_tx.bits.motor_sta,
               motor_name(s_tx.bits.motor_sta),
               (unsigned)s_tx.bits.lift_sensor_sta,
               (s_tx.bits.lift_sensor_sta != 0U) ? "FAULT" : "ok",
               (unsigned)s_tx.bits.lift_sta,
               lift_name(s_tx.bits.lift_sta));

    rt_kprintf("B2 up_valve=%u(%s) dn_valve=%u(%s)\n",
               (unsigned)s_tx.bits.up_valve_sta,
               valve_name(s_tx.bits.up_valve_sta),
               (unsigned)s_tx.bits.down_valve_sta,
               valve_name(s_tx.bits.down_valve_sta));

    rt_kprintf("B3 set=%u(%s) teeth=%u(%s)\n",
               (unsigned)s_tx.bits.teeth_set_sta,
               (s_tx.bits.teeth_set_sta != 0U) ? "configured" : "UNSET",
               (unsigned)s_tx.bits.teeth_code,
               teeth_name(s_tx.bits.teeth_code));

    rt_kprintf("B5-6 swing_raw=%d\n", (int)s_tx.bits.swing_angle_raw);
    print_x10("     swing", s_diag.swing_x10);
    rt_kprintf("B7   lift_raw=%u\n", (unsigned)s_tx.bits.lift_angle_raw);
    print_x10("     lift ", s_diag.lift_x10);

    rt_kprintf("--- inputs ---\n");
    rt_kprintf("  uv=%u comm=%u calib=%u enc_fault=%u sensor_fault=%u stall=%u\n",
               (unsigned)s_diag.uv, (unsigned)s_diag.comm,
               (unsigned)s_diag.calibrated, (unsigned)s_diag.encoder_fault,
               (unsigned)s_diag.sensor_fault, (unsigned)s_diag.stall);
    rt_kprintf("  swing_st=%u lift_st=%u hold_dir=%u onekey=%u\n",
               (unsigned)swing_control_get_state(),
               (unsigned)lift_control_get_state(),
               (unsigned)s_diag.hold_dir, (unsigned)s_diag.onekey);
    rt_kprintf("====================================\n\n");
}

/* ============================================================
 * 注册
 * ============================================================ */
static const can_tx_entry_t s_status_entry = {
    .name      = "status",
    .id        = STATUS_FRAME_ID,
    .dlc       = 8U,
    .period_ms = STATUS_FRAME_PERIOD_MS,
    .phase_ms  = STATUS_FRAME_PHASE_MS,
    .trig      = CAN_TX_TRIG_PERIODIC,
    .pack      = status_tx_pack,
};

int status_tx_init(void)
{
    status_tx_reset_default();

    /* 【P8】先刷一遍真实值。此时 lift/swing/seq 已初始化（boot 14~16），
     * can 调度器也已起（boot 25），can_is_bus_silent 可用。 */
    status_tx_refresh();

    /* 【can_tx_cfg.h】模块级开关：0 = 完全不注册（不占 slot、不调 pack）。
     * 返回 0 而非错误：boot 表把本步标为 BOOT_FATAL，返回非 0 会中断启动。
     * 「关闭」是正常状态，不是故障。t_sta 在关闭时仍可打印。 */
#if !CAN_TX_EN_STATUS
    rt_kprintf("[CAN-TX] status frame DISABLED by can_tx_cfg.h (t_sta still works)\n");
    return 0;
#else
    return can_tx_register(&s_status_entry);
#endif
}
