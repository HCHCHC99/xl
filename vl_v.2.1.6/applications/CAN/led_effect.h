/**
 * @file    led_effect.h
 * @brief   面板灯色决策（纯逻辑，不依赖 CAN 发送）
 *
 * ============================================================
 * 为什么单独一层（P8 / 2026-11-23）
 * ============================================================
 * 灯色的本质是「状态 → 4bit 编码」，与「这 8 字节怎么发出去」是两件事。
 * 混在 led_tx.c 里的后果：台架上没有实物手柄、CAN 对端也没接，
 * 想验「编码器故障该亮红闪」就必须先让整条 CAN 链路通 —— 排查成本极高。
 *
 * 拆开后：
 *   led_effect_compute()  只读各模块 getter，输出每键灯色 + 判定依据
 *   led_tx.c              把结果塞进 0x18FFA4F0 的位域，交给 can_tx
 *   t_led                 RTT 直接打印 led_effect 的结果，完全不碰总线
 *
 * 因此 CAN 发送开着还是关着（can_tx_cfg.h 的 CAN_TX_EN_LED），
 * t_led 的输出都一样，验的是决策而不是传输。
 *
 * ============================================================
 * 规则来源
 * ============================================================
 * xl.xlsx `14_指示灯与按键反馈` TC-LED-001 ~ 011。
 * 优先级：红 > 黄 > 绿（TC-LED-011）；闪烁位可叠加在任一颜色上。
 *
 * 协议限制：0x18FFA4F0 每键只有 4 bit（绿/黄/红/闪），**表达不了频率**。
 * 用例里「红 1Hz」「红 5Hz」在实际帧上都只能是同一个"红+闪"，
 * 频率由面板固件自行决定 —— 这条在 §待确认 里挂着。
 */

#ifndef __LED_EFFECT_H
#define __LED_EFFECT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 单键灯 4bit 编码（与 0x18FFA4F0 字段位义一致）
 *   bit0 = 绿 / bit1 = 黄 / bit2 = 红 / bit3 = 闪烁
 * ============================================================ */
#define LED_OFF                 (0x00U)
#define LED_GREEN               (0x01U)                 /* 绿常亮 */
#define LED_YELLOW              (0x02U)                 /* 黄常亮 */
#define LED_RED                 (0x04U)                 /* 红常亮 */
#define LED_RED_BLINK           (0x04U | 0x08U)         /* 红闪 */
#define LED_YELLOW_BLINK        (0x02U | 0x08U)         /* 黄闪 */
#define LED_GREEN_BLINK         (0x01U | 0x08U)         /* 绿闪 */

/* ============================================================
 * 按键编号（与灯帧字节顺序一致）
 * ============================================================ */
typedef enum {
    LED_KEY_UP = 0,         /* Byte0 低半字节 按键1 上 */
    LED_KEY_DOWN,           /* Byte0 高半字节 按键2 下 */
    LED_KEY_LEFT,           /* Byte1 低半字节 按键3 左 */
    LED_KEY_RIGHT,          /* Byte1 高半字节 按键4 右 */
    LED_KEY_HOME,           /* Byte2 低半字节 按键5 一键回位 */
    LED_KEY_CALIB,          /* Byte2 高半字节 按键6 校准 */
    LED_KEY_MID,            /* Byte3 低半字节 按键7 一键右中 */
    LED_KEY_REAR,           /* Byte3 高半字节 按键8 一键右后 */
    LED_KEY_CLUTCH_COM,     /* Byte4 低半字节 按键9 离合结合 */
    LED_KEY_CLUTCH_SEP,     /* Byte4 高半字节 按键10 离合分离 */
    LED_KEY_COUNT
} led_key_t;

/* ============================================================
 * 指示灯编号（Byte6 / Byte7）
 * ============================================================ */
typedef enum {
    LED_IND_MOTOR = 0,      /* Byte6 低半字节 指示灯1 电机 */
    LED_IND_CTRL,           /* Byte6 高半字节 指示灯2 控制器 */
    LED_IND_SENSOR,         /* Byte7 bit0-3   指示灯3 传感器 */
    LED_IND_COUNT
} led_ind_t;

/* ============================================================
 * 决策结果
 * ============================================================ */
typedef struct {
    uint8_t     key[LED_KEY_COUNT];         /* 每键 4bit 灯色 */
    const char *key_why[LED_KEY_COUNT];     /* 判定依据（t_led 打印用，静态串） */
    uint8_t     ind[LED_IND_COUNT];         /* 三个指示灯 4bit */
    const char *ind_why[LED_IND_COUNT];
    uint8_t     backlight;                  /* Byte7 bit4 */

    /* ----- 输入快照（算灯色那一刻读到的原始值，供 t_led 定位"为什么"）----- */
    struct {
        uint8_t swing_state;        /* swing_state_t */
        uint8_t lift_state;         /* lift_state_t */
        uint8_t lift_hold_dir;      /* LIMIT_HOLDING 的方向：1=上 0=下 */
        uint8_t seq_onekey;         /* 0=无 1=mid 2=rear 3=home */
        uint8_t encoder_fault;
        uint8_t stall_fault;
        uint8_t valve_up_raw;       /* 0/1/2 */
        uint8_t valve_down_raw;
        uint8_t sensor_fault;
        uint8_t comm_fault;
        uint8_t undervoltage;
        uint8_t calibrated;
        uint8_t calib_state;        /* calib_state_t（P9）：0 IDLE / 1 PREP / 2 ARMED / 3 EXEC / 4 DONE / 5 FAIL */
    } in;
} led_effect_t;

/**
 * @brief 计算全部灯色（只读 getter，无副作用，可在任意线程调用）
 * @param out 输出结构（内部各字符串均为静态常量，不需要释放）
 */
void led_effect_compute(led_effect_t *out);

/**
 * @brief 4bit 灯色的英文名（t_led 打印用）
 */
const char *led_effect_color_name(uint8_t code);

/**
 * @brief 按键索引的英文名（t_led 打印用）
 */
const char *led_effect_key_name(led_key_t key);

#ifdef __cplusplus
}
#endif

#endif /* __LED_EFFECT_H */
