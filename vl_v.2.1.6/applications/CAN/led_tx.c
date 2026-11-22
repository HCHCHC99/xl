/**
 * @file    led_tx.c
 * @brief   0x18FFA4F0 开关面板灯状态帧组包实现
 * @note
 *   本机发送帧，方向：UTMotVCU → 开关面板，周期 100ms。
 *   字段由 CAN_EVT_LED_* 事件刷新，pack 只做 memcpy。
 *
 *   【灯色规则待确认 N-2】
 *   下方 key_led_of() 中的映射按 SRS REQ003-FUN003 推导：
 *     无故障无输出 → 绿常亮
 *     有输出       → 黄常亮
 *     编码器异常   → 红 1Hz 闪烁
 *     电机堵转     → 红常亮
 *     优先级：红 > 黄 > 绿
 */

#include "led_tx.h"
#include "protocol_defs.h"
#include "hw_objects.h"
#include "can_tx_cfg.h"          /* CAN_TX_EN_LED 模块级开关 */
#include "../sys_param/sys_param.h"

#include <string.h>
#include <rtthread.h>

/* ============================================================
 * 本地缓存
 * ============================================================ */
static frame_0x18FFA4F0_t s_tx;

/* ============================================================
 * 按键编号（与协议灯帧顺序一致）
 * ============================================================ */
typedef enum {
    LED_KEY_UP = 0,         /* 按键1（上） */
    LED_KEY_DOWN,           /* 按键2（下） */
    LED_KEY_LEFT,           /* 按键3（左） */
    LED_KEY_RIGHT,          /* 按键4（右） */
    LED_KEY_HOME,           /* 按键5（一键回位） */
    LED_KEY_CALIB,          /* 按键6（校准） */
    LED_KEY_MID,            /* 按键7（一键右中） */
    LED_KEY_REAR,           /* 按键8（一键右后） */
    LED_KEY_CLUTCH_COM,     /* 按键9（离合结合） */
    LED_KEY_CLUTCH_SEP,     /* 按键10（离合分离） */
    LED_KEY_MAX
} led_key_id_t;

/* ============================================================
 * 默认值：全部熄灭
 * ============================================================ */
static void led_tx_reset_default(void)
{
    memset(s_tx.raw, 0, sizeof(s_tx.raw));
}

/* ============================================================
 * 单键灯色推导
 * ============================================================
 * TODO(N-1)(N-2)：待各模块状态 getter 就绪后，按确认的规则填充。
 * 当前统一返回「绿常亮」（无故障无输出），保证面板可见且不误报。
 * ============================================================ */
static uint8_t key_led_of(led_key_id_t key)
{
    (void)key;
    return LED_GREEN;
}

/* ============================================================
 * 事件入口
 * ============================================================ */
void led_tx_on_event(app_event_t event)
{
    switch (event) {

    case CAN_EVT_LED_KEY:
        s_tx.bits.key_up_led         = key_led_of(LED_KEY_UP);
        s_tx.bits.key_down_led       = key_led_of(LED_KEY_DOWN);
        s_tx.bits.key_left_led       = key_led_of(LED_KEY_LEFT);
        s_tx.bits.key_right_led      = key_led_of(LED_KEY_RIGHT);
        s_tx.bits.key_home_led       = key_led_of(LED_KEY_HOME);
        s_tx.bits.key_calib_led      = key_led_of(LED_KEY_CALIB);
        s_tx.bits.key_mid_led        = key_led_of(LED_KEY_MID);
        s_tx.bits.key_rear_led       = key_led_of(LED_KEY_REAR);
        s_tx.bits.key_clutch_com_led = key_led_of(LED_KEY_CLUTCH_COM);
        s_tx.bits.key_clutch_sep_led = key_led_of(LED_KEY_CLUTCH_SEP);
        break;

    case CAN_EVT_LED_IND:
        /* TODO(N-1)：电机/控制器/传感器故障标志就绪后替换
         *   正常 → LED_GREEN；故障 → LED_RED */
        s_tx.bits.ind_motor_led  = LED_GREEN;
        s_tx.bits.ind_ctrl_led   = LED_GREEN;
        s_tx.bits.ind_sensor_led = LED_GREEN;
        break;

    case CAN_EVT_LED_BACKLIGHT:
        /* 钥匙上电点亮背景灯（默认点亮） */
        s_tx.bits.backlight = 1U;
        break;

    default:
        break;
    }
}

/* ============================================================
 * 组包
 * ============================================================ */
void led_tx_pack(can_tx_frame_t *frame)
{
    if (frame == RT_NULL) {
        return;
    }
    memcpy(frame->data, s_tx.raw, 8);
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
    /* 【can_tx_cfg.h】模块级开关：0 = 完全不注册。
     * 注意连 led_tx_on_event 的三次默认灯色建立也一起跳过 ——
     * 关闭就该彻底关，留下"建立了灯色但从不发帧"是更难查的半关状态。 */
#if !CAN_TX_EN_LED
    rt_kprintf("[CAN-TX] led frame DISABLED by can_tx_cfg.h\n");
    return 0;
#else
    led_tx_reset_default();

    /* 上电立即建立默认灯色，避免面板在首个按键事件前无任何指示 */
    led_tx_on_event(CAN_EVT_LED_KEY);
    led_tx_on_event(CAN_EVT_LED_IND);
    led_tx_on_event(CAN_EVT_LED_BACKLIGHT);

    return can_tx_register(&s_led_entry);
#endif
}
