/**
 * @file    keyboard_defs.h
 * @brief   键盘模块公共类型定义（无逻辑，纯类型）
 *
 * 【四层抽象】
 *   物理按键位(mask) → 虚拟键值(key_code) → 键盘指令(key_cmd) → 业务动作
 *
 * 【职责边界】
 *   本模块只负责「用户想干什么」，不关心「怎么执行」。
 *   执行由 lift / swing / seq / calibration 各自的业务模块完成。
 */

#ifndef __KEYBOARD_DEFS_H
#define __KEYBOARD_DEFS_H

#include <stdint.h>
#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 1. 物理按键位掩码
 * ============================================================ */
#define KB_UP               (1U << 0)
#define KB_DOWN             (1U << 1)
#define KB_LEFT             (1U << 2)
#define KB_RIGHT            (1U << 3)
#define KB_ONKEY_MID        (1U << 4)      /* 一键右中 */
#define KB_ONKEY_REAR       (1U << 5)      /* 一键右后 */
#define KB_ONKEY_HOME       (1U << 6)      /* 一键回位 */
#define KB_CALIB            (1U << 7)      /* 校准键 */
#define KB_CLUTCH_COM       (1U << 8)      /* 预留：离合结合 */
#define KB_CLUTCH_SEP       (1U << 9)      /* 预留：离合分离 */

#define KB_MANUAL_MASK      (KB_UP | KB_DOWN | KB_LEFT | KB_RIGHT)
#define KB_ONEKEY_MASK      (KB_ONKEY_MID | KB_ONKEY_REAR | KB_ONKEY_HOME)

/* ============================================================
 * 2. 虚拟键值
 * ============================================================
 * 独立键与组合键统一编码，键值由键盘映射表（keymap）解析得出。
 * ============================================================ */
typedef enum {
    KEY_CODE_NONE = 0,

    /* ---- 单键 ---- */
    KEY_CODE_UP,
    KEY_CODE_DOWN,
    KEY_CODE_LEFT,
    KEY_CODE_RIGHT,
    KEY_CODE_ONKEY_MID,
    KEY_CODE_ONKEY_REAR,
    KEY_CODE_ONKEY_HOME,
    KEY_CODE_CALIB,

    /* ---- 同轴对向（本层仅归类，消解由 _kb_axis_filter 完成） ---- */
    KEY_CODE_UP_DOWN,
    KEY_CODE_LEFT_RIGHT,

    /* ---- 异轴双键（合法，双轴同时动作） ---- */
    KEY_CODE_UP_LEFT,
    KEY_CODE_UP_RIGHT,
    KEY_CODE_DOWN_LEFT,
    KEY_CODE_DOWN_RIGHT,

    /* ---- 类别冲突（手动 + 一键，手动优先） ---- */
    KEY_CODE_MANUAL_ONEKEY,

    /* ---- 三键及以上（本层仅归类，逐轴消解后各轴独立判定） ---- */
    KEY_CODE_MULTI,

    KEY_CODE_MAX
} key_code_t;

/* ============================================================
 * 3. 键盘指令（位域，支持多轴同时动作）
 * ============================================================ */
typedef struct {
    uint8_t up          : 1;
    uint8_t down        : 1;
    uint8_t left        : 1;
    uint8_t right       : 1;
    uint8_t onekey_mid  : 1;
    uint8_t onekey_rear : 1;
    uint8_t onekey_home : 1;
    uint8_t calib       : 1;
    uint8_t blocked     : 1;    /* 整体锁死，所有动作无效 */
} key_cmd_t;

/* 指令是否有任一手动方向 */
static inline uint8_t key_cmd_has_manual(const key_cmd_t *c)
{
    return (c->up || c->down || c->left || c->right) ? 1U : 0U;
}

/* 指令是否有任一鍵 */
static inline uint8_t key_cmd_is_empty(const key_cmd_t *c)
{
    return ((c->up == 0U) && (c->down == 0U) && (c->left == 0U) &&
            (c->right == 0U) && (c->onekey_mid == 0U) &&
            (c->onekey_rear == 0U) && (c->onekey_home == 0U) &&
            (c->blocked == 0U)) ? 1U : 0U;
}

/* ============================================================
 * 4. 键盘状态机状态
 * ============================================================ */
typedef enum {
    KSM_IDLE = 0,       /* 无有效指令 */
    KSM_ACTIVE,         /* 有动作生效 */
    KSM_BLOCKED,        /* 锁死：必须回到全松开才解除 */
} ksm_state_t;

/* ============================================================
 * 5. 按键快照（CAN 线程 → keyboard 线程）
 * ============================================================
 * 单写者（can_parser）/ 单读者（keyboard），uint16/uint32 对齐访问原子，
 * 无需加锁。
 * ============================================================ */
typedef struct {
    volatile uint16_t mask;         /* 当前按键掩码 */
    volatile uint32_t rx_tick;      /* 最近收帧时刻 */
    volatile uint8_t  valid;        /* 1 = 数据有效（300ms 内收到过） */
} kb_snapshot_t;

/* ============================================================
 * 6. 超时参数
 * ============================================================ */
#define KB_RX_TIMEOUT_MS    300U    /* 报文 100ms 周期，容忍连丢 2 帧 */
#define KB_MUTEX_WINDOW_MS  300U    /* 同轴互斥判定窗口（两方向状态变化时差） */

/* ============================================================
 * 7. 单轴互斥消解记忆
 * ============================================================
 * 每个轴（举升 / 回转）各一份，两轴完全独立，故「上升 + 左转」
 * 这类异轴组合不受影响。
 *
 * 判定依据是「状态帧」语义（100ms 周期，按下=1 / 释放=0），
 * 因此不需要边沿检测，直接比对上一帧状态即可。
 * ============================================================ */
typedef struct {
    uint8_t   last_pos;      /* 上一帧该轴正向位（举升=上升，回转=左转） */
    uint8_t   last_neg;      /* 上一帧该轴反向位（举升=下降，回转=右转） */
    uint8_t   conflict;      /* 1 = 互斥判定中，该轴所有方向视为没按 */
    rt_tick_t change_tick;   /* 该轴状态最后一次变化的时刻（用于 300ms 窗口） */
} kb_axis_mem_t;

#ifdef __cplusplus
}
#endif

#endif /* __KEYBOARD_DEFS_H */
