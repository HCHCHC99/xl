/**
 * @file    led_effect.c
 * @brief   面板灯色决策实现（P8 / 2026-11-23）
 *
 * 只读各模块 getter，不做任何输出动作，也不碰 CAN。
 * 规则见 led_effect.h 顶部说明，逐条对应用例编号写在函数里。
 */

#include "led_effect.h"

#include "../swing/swing_control.h"     /* g_swing_obj / swing_control_get_state */
#include "../lift/lift_control.h"       /* g_lift_obj / lift_control_get_state */
#include "../sequence/seq_control.h"    /* seq_control_get_context */
#include "../motor/motor_stall.h"       /* motor_stall_is_faulted */
#include "../sys_param/sys_param.h"     /* g_sys_params.is_calibrated */
#include "../service/sys_voltage.h"     /* sys_voltage_is_uv */
#include "../calib/calib_ctrl.h"        /* calib_ctrl_state（P9 校准流程灯） */
#include "../config/sys_config.h"       /* CFG_ID_VEH_CAN_SILENT_MS */
#include "ut_can_scheduler.h"           /* can_is_bus_silent */
#include "hw_objects.h"                 /* g_solenoid 阀原始故障码 */

#include <rtthread.h>

/* ============================================================
 * 阀故障码：取"已确认"的那一份
 * ------------------------------------------------------------
 * 不能直接用 g_solenoid.up_fault —— 那是 CAN 原始码，每帧都刷新。
 * P4 特意做了 500 ms 去抖（滤掉总线偶发误码/接触抖动），如果灯和
 * 状态帧读原始码，就会"红闪一下又变绿"，比不去抖还让人困惑。
 *
 * 这里读去抖层的 latched + raw：
 *   latched=0（含去抖窗口内）→ 视为正常
 *   latched=1                → 用它记住的 raw 区分 开路(1)/短路(2)
 * 这样灯、状态帧、举升状态机三者口径一致。
 * ============================================================ */
static uint8_t valve_confirmed_code(uint8_t ch)
{
    lift_vfb_state_t st;

    lift_control_valve_fb_get(&st);
    if (st.ch[ch & 0x01U].latched == 0U) {
        return 0U;
    }
    return (uint8_t)(st.ch[ch & 0x01U].raw & 0x03U);
}

/* ============================================================
 * 采集输入快照
 * ============================================================ */
static void collect_inputs(led_effect_t *e)
{
    seq_ctrl_context_t *sctx = seq_control_get_context();
    int32_t silent_ms = 500;

    e->in.swing_state    = (uint8_t)swing_control_get_state();
    e->in.lift_state     = (uint8_t)lift_control_get_state();
    e->in.lift_hold_dir  = (uint8_t)(g_lift_obj.ctx.hold_direction != 0U ? 1U : 0U);
    e->in.encoder_fault  = (uint8_t)(g_swing_obj.ctx.encoder_fault != 0U ? 1U : 0U);
    e->in.stall_fault    = motor_stall_is_faulted();
    /* 用去抖确认后的码，不用 CAN 原始码（理由见 valve_confirmed_code） */
    e->in.valve_up_raw   = valve_confirmed_code(0U);
    e->in.valve_down_raw = valve_confirmed_code(1U);
    e->in.sensor_fault   = (uint8_t)(g_lift_obj.ctx.sensor_fault != 0U ? 1U : 0U);
    e->in.undervoltage   = sys_voltage_is_uv();
    e->in.calibrated     = (uint8_t)(g_sys_params.is_calibrated != 0U ? 1U : 0U);
    e->in.calib_state    = calib_ctrl_state();   /* P9 校准流程态 */

    /* 通信故障：与整车 FAULT 同源（任何帧静默），门限复用 veh_can_silent_ms */
    if (sys_config_get_i32(CFG_ID_VEH_CAN_SILENT_MS, &silent_ms) != 0) {
        silent_ms = 500;
    }
    e->in.comm_fault = can_is_bus_silent((uint32_t)silent_ms);

    /* 一键：只有引擎在跑时才算"该键执行中"（结束后 current_onekey 会清零，
     * 这里再判一次 active 是为了防"刚完成那一拍还在显示闪烁"） */
    if ((sctx != RT_NULL) && (seq_engine_is_active(&sctx->engine) != 0U)) {
        e->in.seq_onekey = sctx->current_onekey;
    } else {
        e->in.seq_onekey = 0U;
    }
}

/* ============================================================
 * 举升方向输出（含 LIMIT_HOLDING 的保压）
 * ------------------------------------------------------------
 * P6 起，到限位后阀会继续保持 5 s。那 5 s 里阀确实带电，
 * 所以按键灯按"有输出"处理（黄），与状态帧 lift_sta 的口径一致。
 * ============================================================ */
static uint8_t lift_up_on(const led_effect_t *e)
{
    if (e->in.lift_state == (uint8_t)P_STATE_UP_RUNNING) {
        return 1U;
    }
    if ((e->in.lift_state == (uint8_t)P_STATE_LIMIT_HOLDING) &&
        (e->in.lift_hold_dir != 0U)) {
        return 1U;
    }
    return 0U;
}

static uint8_t lift_down_on(const led_effect_t *e)
{
    if (e->in.lift_state == (uint8_t)P_STATE_DOWN_RUNNING) {
        return 1U;
    }
    if ((e->in.lift_state == (uint8_t)P_STATE_LIMIT_HOLDING) &&
        (e->in.lift_hold_dir == 0U)) {
        return 1U;
    }
    return 0U;
}

/* ============================================================
 * 单个按键：左右转（TC-LED-001 / 002）
 * ------------------------------------------------------------
 * 编码器异常 → 红 1Hz 闪
 * 电机堵转   → 红常亮（松开后 stall 自清，灯自动回绿）
 * 有输出     → 黄常亮
 * 其余       → 绿常亮
 * ============================================================ */
static uint8_t key_swing(const led_effect_t *e, uint8_t running_state,
                         const char **why)
{
    if (e->in.encoder_fault != 0U) {
        *why = "encoder fault";
        return LED_RED_BLINK;
    }
    if (e->in.stall_fault != 0U) {
        *why = "motor stall";
        return LED_RED;
    }
    if (e->in.swing_state == running_state) {
        *why = "output on";
        return LED_YELLOW;
    }
    *why = "idle";
    return LED_GREEN;
}

/* ============================================================
 * 单个按键：升降（TC-LED-003 / 004）
 * ------------------------------------------------------------
 * 阀开路 → 红闪；阀短路 → 红常亮；有输出 → 黄；其余绿。
 * 阀故障优先级高于"有输出"：P4 确认故障后 lift 会进 FAULT_VALVE 并关阀，
 * 但灯的响应不该等到状态迁移完才变色。
 * ============================================================ */
static uint8_t key_lift(const led_effect_t *e, uint8_t raw, uint8_t output_on,
                        const char **why)
{
    if (raw == 1U) {
        *why = "valve open";
        return LED_RED_BLINK;
    }
    if (raw == 2U) {
        *why = "valve short";
        return LED_RED;
    }
    if (output_on != 0U) {
        *why = "output on";
        return LED_YELLOW;
    }
    *why = "idle";
    return LED_GREEN;
}

/* ============================================================
 * 单个按键：一键（TC-LED-005）
 * ------------------------------------------------------------
 * 该键对应的序列在执行 → 绿 1Hz 闪；其余绿常亮。
 * 用例里"机构按键灯黄色"含义不明（原文档作者自己也标注了），暂不实现。
 * ============================================================ */
static uint8_t key_onekey(const led_effect_t *e, uint8_t onekey_id,
                          const char **why)
{
    if ((onekey_id != 0U) && (e->in.seq_onekey == onekey_id)) {
        *why = "sequence running";
        return LED_GREEN_BLINK;
    }
    *why = "idle";
    return LED_GREEN;
}

/* ============================================================
 * 计算全部灯色
 * ============================================================ */
void led_effect_compute(led_effect_t *out)
{
    uint8_t i;

    if (out == RT_NULL) {
        return;
    }

    collect_inputs(out);

    /* ----- 10 个按键 ----- */
    out->key[LED_KEY_UP] = key_lift(out, out->in.valve_up_raw,
                                    lift_up_on(out), &out->key_why[LED_KEY_UP]);
    out->key[LED_KEY_DOWN] = key_lift(out, out->in.valve_down_raw,
                                      lift_down_on(out), &out->key_why[LED_KEY_DOWN]);

    out->key[LED_KEY_LEFT] = key_swing(out, (uint8_t)H_STATE_LEFT_RUNNING,
                                       &out->key_why[LED_KEY_LEFT]);
    out->key[LED_KEY_RIGHT] = key_swing(out, (uint8_t)H_STATE_RIGHT_RUNNING,
                                        &out->key_why[LED_KEY_RIGHT]);

    out->key[LED_KEY_HOME] = key_onekey(out, 3U, &out->key_why[LED_KEY_HOME]);
    out->key[LED_KEY_MID]  = key_onekey(out, 1U, &out->key_why[LED_KEY_MID]);
    out->key[LED_KEY_REAR] = key_onekey(out, 2U, &out->key_why[LED_KEY_REAR]);

    /* 校准键（TC-LED-006 + P9 / B-10）：
     *   流程态优先：PREP 黄 / ARMED 绿闪 / EXEC 黄闪（"执行中"灯色用例占位，
     *   暂按黄闪）/ DONE 绿 / FAIL 红；空闲回落到 未校准黄 / 已校准绿。 */
    switch (out->in.calib_state) {
    case 1U:    /* CALIB_ST_PREP */
        out->key[LED_KEY_CALIB] = LED_YELLOW;
        out->key_why[LED_KEY_CALIB] = "calib prep (5s)";
        break;
    case 2U:    /* CALIB_ST_ARMED */
        out->key[LED_KEY_CALIB] = LED_GREEN_BLINK;
        out->key_why[LED_KEY_CALIB] = "calib armed (hold)";
        break;
    case 3U:    /* CALIB_ST_EXEC */
        out->key[LED_KEY_CALIB] = LED_YELLOW_BLINK;
        out->key_why[LED_KEY_CALIB] = "calib running";
        break;
    case 4U:    /* CALIB_ST_DONE */
        out->key[LED_KEY_CALIB] = LED_GREEN;
        out->key_why[LED_KEY_CALIB] = "calib ok";
        break;
    case 5U:    /* CALIB_ST_FAIL */
        out->key[LED_KEY_CALIB] = LED_RED;
        out->key_why[LED_KEY_CALIB] = "calib fail";
        break;
    default:    /* CALIB_ST_IDLE */
        if (out->in.calibrated == 0U) {
            out->key[LED_KEY_CALIB] = LED_YELLOW;
            out->key_why[LED_KEY_CALIB] = "not calibrated";
        } else {
            out->key[LED_KEY_CALIB] = LED_GREEN;
            out->key_why[LED_KEY_CALIB] = "calibrated";
        }
        break;
    }

    /* 离合两个键（TC-LED-007）：本机不驱动离合电机，只做按键仲裁。
     * 用例原文明显是左转键的复制件（"右转有输出：黄灯"），且标着"不李姐"，
     * 暂按绿常亮占位 —— TODO 待确认。 */
    for (i = LED_KEY_CLUTCH_COM; i <= LED_KEY_CLUTCH_SEP; i++) {
        out->key[i] = LED_GREEN;
        out->key_why[i] = "n/a (this unit)";
    }

    /* ----- 三个指示灯 ----- */
    /* 电机灯（TC-LED-008）：堵转红闪 / 编码器故障红闪 / 正常绿。
     * 用例区分 1Hz 与 5Hz，但协议只有 1 个闪烁位，无法表达频率 —— 见头文件说明。 */
    if (out->in.stall_fault != 0U) {
        out->ind[LED_IND_MOTOR] = LED_RED_BLINK;
        out->ind_why[LED_IND_MOTOR] = "motor stall";
    } else if (out->in.encoder_fault != 0U) {
        out->ind[LED_IND_MOTOR] = LED_RED_BLINK;
        out->ind_why[LED_IND_MOTOR] = "encoder fault";
    } else {
        out->ind[LED_IND_MOTOR] = LED_GREEN;
        out->ind_why[LED_IND_MOTOR] = "normal";
    }

    /* 控制器灯（TC-LED-009）：通信故障 > 欠压 > 未校准 > 正常。
     * 用例原文 3) 过欠压红快闪 与 5) 过压/低压保护红常亮 自相矛盾，
     * 这里取"红闪"，频率待定 —— TODO。 */
    if (out->in.comm_fault != 0U) {
        out->ind[LED_IND_CTRL] = LED_RED_BLINK;
        out->ind_why[LED_IND_CTRL] = "comm fault";
    } else if (out->in.undervoltage != 0U) {
        out->ind[LED_IND_CTRL] = LED_RED_BLINK;
        out->ind_why[LED_IND_CTRL] = "undervoltage";
    } else if (out->in.calibrated == 0U) {
        out->ind[LED_IND_CTRL] = LED_YELLOW;
        out->ind_why[LED_IND_CTRL] = "not calibrated";
    } else {
        out->ind[LED_IND_CTRL] = LED_GREEN;
        out->ind_why[LED_IND_CTRL] = "normal";
    }

    /* 传感器灯（TC-LED-010）：故障红，其余绿 */
    if (out->in.sensor_fault != 0U) {
        out->ind[LED_IND_SENSOR] = LED_RED;
        out->ind_why[LED_IND_SENSOR] = "sensor fault";
    } else {
        out->ind[LED_IND_SENSOR] = LED_GREEN;
        out->ind_why[LED_IND_SENSOR] = "normal";
    }

    /* 背景灯（TC-LED-010）：钥匙上电即亮。协议只有 1 bit，"绿色"由面板硬件定。 */
    out->backlight = 1U;
}

/* ============================================================
 * 名称表（t_led 打印用）
 * ============================================================ */
const char *led_effect_color_name(uint8_t code)
{
    switch (code) {
        case LED_OFF:           return "off";
        case LED_GREEN:         return "green";
        case LED_YELLOW:        return "yellow";
        case LED_RED:           return "red";
        case LED_GREEN_BLINK:   return "green-blink";
        case LED_YELLOW_BLINK:  return "yellow-blink";
        case LED_RED_BLINK:     return "red-blink";
        default:                return "?";
    }
}

const char *led_effect_key_name(led_key_t key)
{
    switch (key) {
        case LED_KEY_UP:         return "up";
        case LED_KEY_DOWN:       return "down";
        case LED_KEY_LEFT:       return "left";
        case LED_KEY_RIGHT:      return "right";
        case LED_KEY_HOME:       return "home";
        case LED_KEY_CALIB:      return "calib";
        case LED_KEY_MID:        return "mid";
        case LED_KEY_REAR:       return "rear";
        case LED_KEY_CLUTCH_COM: return "clu_com";
        case LED_KEY_CLUTCH_SEP: return "clu_sep";
        default:                 return "?";
    }
}
