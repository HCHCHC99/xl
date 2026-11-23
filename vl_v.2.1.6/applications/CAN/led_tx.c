/**
 * @file    led_tx.c
 * @brief   0x18FFA4F0 开关面板灯状态帧组包实现
 * @note
 *   本机发送帧，方向：UTMotVCU → 开关面板，周期 100ms（相位 50ms）。
 *
 *   【P8 / 2026-11-23 分层】
 *   灯色规则全部移入 led_effect.c（纯逻辑，只读 getter）。
 *   本文件只做两件事：① 把 led_effect 的结果写进 8 字节位域；
 *                    ② 交给 can_tx 周期发送（可用 can_tx_cfg.h 关掉）。
 *
 *   这样 t_led 打印的是"决策结果"，与"帧有没有真的发出去"彻底解耦：
 *   台架上把 CAN_TX_EN_LED 置 0，总线静默，t_led 仍然能验全部灯色规则。
 */

#include "led_tx.h"
#include "protocol_defs.h"
#include "can_tx_cfg.h"          /* CAN_TX_EN_LED 模块级开关 */

#include <string.h>
#include <rtthread.h>

/* ============================================================
 * 本地缓存
 * ============================================================ */
static frame_0x18FFA4F0_t s_tx;
static led_effect_t       s_eff;     /* 最近一次决策结果（t_led 读它） */
static uint8_t            s_eff_valid = 0U;

/* ============================================================
 * 默认值：全部熄灭（首个 refresh 之前的状态）
 * ============================================================ */
static void led_tx_reset_default(void)
{
    memset(s_tx.raw, 0, sizeof(s_tx.raw));
    memset(&s_eff, 0, sizeof(s_eff));
    s_eff_valid = 0U;
}

/* ============================================================
 * 重新计算灯色并写入帧缓存
 * ============================================================ */
void led_tx_refresh(void)
{
    /* 决策（只读各模块 getter，不碰 CAN） */
    led_effect_compute(&s_eff);
    s_eff_valid = 1U;

    /* 按键灯：10 个半字节，顺序与协议一致 */
    s_tx.bits.key_up_led         = s_eff.key[LED_KEY_UP];
    s_tx.bits.key_down_led       = s_eff.key[LED_KEY_DOWN];
    s_tx.bits.key_left_led       = s_eff.key[LED_KEY_LEFT];
    s_tx.bits.key_right_led      = s_eff.key[LED_KEY_RIGHT];
    s_tx.bits.key_home_led       = s_eff.key[LED_KEY_HOME];
    s_tx.bits.key_calib_led      = s_eff.key[LED_KEY_CALIB];
    s_tx.bits.key_mid_led        = s_eff.key[LED_KEY_MID];
    s_tx.bits.key_rear_led       = s_eff.key[LED_KEY_REAR];
    s_tx.bits.key_clutch_com_led = s_eff.key[LED_KEY_CLUTCH_COM];
    s_tx.bits.key_clutch_sep_led = s_eff.key[LED_KEY_CLUTCH_SEP];

    /* 指示灯 */
    s_tx.bits.ind_motor_led  = s_eff.ind[LED_IND_MOTOR];
    s_tx.bits.ind_ctrl_led   = s_eff.ind[LED_IND_CTRL];
    s_tx.bits.ind_sensor_led = s_eff.ind[LED_IND_SENSOR];

    /* 背景灯 */
    s_tx.bits.backlight = (s_eff.backlight != 0U) ? 1U : 0U;

    /* reserved5 / reserved7 保持 0（memset 后未改动） */
}

/* ============================================================
 * 事件入口：任何灯相关事件都触发一次全量重算
 * ============================================================ */
void led_tx_on_event(app_event_t event)
{
    switch (event) {
    case CAN_EVT_LED_KEY:
    case CAN_EVT_LED_IND:
    case CAN_EVT_LED_BACKLIGHT:
        led_tx_refresh();
        break;

    default:
        break;
    }
}

/* ============================================================
 * 组包（can_tx 发送线程调用）
 * ============================================================ */
void led_tx_pack(can_tx_frame_t *frame)
{
    if (frame == RT_NULL) {
        return;
    }
    memcpy(frame->data, s_tx.raw, 8);
}

/* ============================================================
 * 诊断接口
 * ============================================================ */
const led_effect_t *led_tx_get_effect(void)
{
    if (s_eff_valid == 0U) {
        led_tx_refresh();       /* 还没算过就先算一次，保证 dump 有内容 */
    }
    return &s_eff;
}

void led_tx_dump(void)
{
    const led_effect_t *e = led_tx_get_effect();
    uint8_t i;

    rt_kprintf("\n========== CAN TX: led ==========\n");
    rt_kprintf("id       : 0x%08X\n", (unsigned)LED_FRAME_ID);
    rt_kprintf("period   : %u ms (phase %u ms)\n",
               (unsigned)LED_FRAME_PERIOD_MS, (unsigned)LED_FRAME_PHASE_MS);
#if CAN_TX_EN_LED
    rt_kprintf("send     : ON  (CAN_TX_EN_LED=1)\n");
#else
    rt_kprintf("send     : OFF (CAN_TX_EN_LED=0, bus silent; dump still valid)\n");
#endif
    rt_kprintf("raw      : %02X %02X %02X %02X %02X %02X %02X %02X\n",
               s_tx.raw[0], s_tx.raw[1], s_tx.raw[2], s_tx.raw[3],
               s_tx.raw[4], s_tx.raw[5], s_tx.raw[6], s_tx.raw[7]);

    rt_kprintf("--- key LEDs (bit0=G bit1=Y bit2=R bit3=BLINK) ---\n");
    for (i = 0U; i < (uint8_t)LED_KEY_COUNT; i++) {
        rt_kprintf("  %-8s: 0x%X  %-12s <- %s\n",
                   led_effect_key_name((led_key_t)i),
                   (unsigned)e->key[i],
                   led_effect_color_name(e->key[i]),
                   (e->key_why[i] != RT_NULL) ? e->key_why[i] : "-");
    }

    rt_kprintf("--- indicators ---\n");
    rt_kprintf("  motor   : 0x%X  %-12s <- %s\n",
               (unsigned)e->ind[LED_IND_MOTOR],
               led_effect_color_name(e->ind[LED_IND_MOTOR]),
               (e->ind_why[LED_IND_MOTOR] != RT_NULL) ? e->ind_why[LED_IND_MOTOR] : "-");
    rt_kprintf("  ctrl    : 0x%X  %-12s <- %s\n",
               (unsigned)e->ind[LED_IND_CTRL],
               led_effect_color_name(e->ind[LED_IND_CTRL]),
               (e->ind_why[LED_IND_CTRL] != RT_NULL) ? e->ind_why[LED_IND_CTRL] : "-");
    rt_kprintf("  sensor  : 0x%X  %-12s <- %s\n",
               (unsigned)e->ind[LED_IND_SENSOR],
               led_effect_color_name(e->ind[LED_IND_SENSOR]),
               (e->ind_why[LED_IND_SENSOR] != RT_NULL) ? e->ind_why[LED_IND_SENSOR] : "-");
    rt_kprintf("  backlight: %u\n", (unsigned)e->backlight);

    rt_kprintf("--- inputs ---\n");
    rt_kprintf("  swing_st=%u lift_st=%u hold_dir=%u onekey=%u\n",
               (unsigned)e->in.swing_state, (unsigned)e->in.lift_state,
               (unsigned)e->in.lift_hold_dir, (unsigned)e->in.seq_onekey);
    rt_kprintf("  enc=%u stall=%u valve_up=%u valve_dn=%u sensor=%u\n",
               (unsigned)e->in.encoder_fault, (unsigned)e->in.stall_fault,
               (unsigned)e->in.valve_up_raw, (unsigned)e->in.valve_down_raw,
               (unsigned)e->in.sensor_fault);
    rt_kprintf("  comm=%u uv=%u calib=%u\n",
               (unsigned)e->in.comm_fault, (unsigned)e->in.undervoltage,
               (unsigned)e->in.calibrated);
    rt_kprintf("=================================\n\n");
}

/* ============================================================
 * 注册
 * ============================================================ */
static const can_tx_entry_t s_led_entry = {
    .name      = "led",
    .id        = LED_FRAME_ID,
    .dlc       = 8U,
    .period_ms = LED_FRAME_PERIOD_MS,
    .phase_ms  = LED_FRAME_PHASE_MS,
    .trig      = CAN_TX_TRIG_PERIODIC,
    .pack      = led_tx_pack,
};

int led_tx_init(void)
{
    led_tx_reset_default();

    /* 【P8】即使不注册也要先算一遍灯色：
     *   - 面板在首个事件前就有内容可发（如果开着）；
     *   - t_led 在"发送关闭"时也能立即看到决策结果。
     * 这是与旧实现的差别：旧版在 #if 里连缓存都不建立，
     * 关掉发送后 t_led 将无从打印。 */
    led_tx_refresh();

#if !CAN_TX_EN_LED
    rt_kprintf("[CAN-TX] led frame DISABLED by can_tx_cfg.h (t_led still works)\n");
    return 0;
#else
    return can_tx_register(&s_led_entry);
#endif
}
