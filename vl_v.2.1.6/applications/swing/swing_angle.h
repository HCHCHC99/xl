/**
 * @file    swing_angle.h
 * @brief   回转角度对象（编码器数据源）
 * @note    与 lift_angle_t 对称设计，通过函数指针获取编码器值
 */

#ifndef __SWING_ANGLE_H
#define __SWING_ANGLE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 台架模拟开关（任务清单 §11）
 * ------------------------------------------------------------
 * 1 = 允许 t_sim swing <x10> 覆盖回转编码器 raw，并让
 *     swing_angle_is_solve_valid() 直接返回有效
 *     （MARK-13 修复前角度链路恒 0，业务调试必须靠它）
 * 0 = 关闭，走真实编码器/解算
 * 总开关 INPUT_SIM_ENABLE 在 service/input_sim.h
 * ============================================================ */
#ifndef SWING_ANGLE_SIM_ENABLE
#define SWING_ANGLE_SIM_ENABLE 1
#endif

/* ============================================================
 * 位置枚举
 * ============================================================ */
typedef enum {
    SWING_POS_UNKNOWN = 0,
    SWING_POS_LEFT_LIMIT,        /* 左限位（≤ -100° ± 2°） */
    SWING_POS_RIGHT_LIMIT,       /* 右限位（≥ +215° ± 2°） */
    SWING_POS_ZERO,              /* 零位（0° ± 0.5°） */
    SWING_POS_MIDDLE             /* 中间位（其他） */
} swing_position_t;

/* ============================================================
 * 编码器读取函数类型（依赖注入）
 * ============================================================ */
typedef int32_t (*swing_encoder_getter_t)(void *user_data);

/* ============================================================
 * 位置变化回调类型
 * ============================================================ */
typedef void (*swing_position_change_cb_t)(swing_position_t old_pos,
                                          swing_position_t new_pos);

/* ============================================================
 * 编码器故障回调（2026-11-23 新增）
 * ------------------------------------------------------------
 * 解算有效性变化时通知上层写 ctx.encoder_fault：
 *   1 = 刚判为故障（有效 → 无效）
 *   0 = 刚恢复（无效 → 有效）
 *
 * 为什么用回调而不是直接写 g_swing_obj：
 *   swing_angle 是纯角度对象（可单独测试），反向依赖控制对象会形成环。
 *   这里与已有的 on_position_change 是同一套写法，接线在 swing_init.c。
 *
 * ⚠【启用 SPI 绝对式编码器前必读】业务.md §13.8.7
 *   届时故障来源会有两个（解算 + SPI 看门狗），本回调的 1/0 语义
 *   必须改成"来源位掩码"，否则两源互相覆盖。
 * ============================================================ */
typedef void (*swing_fault_change_cb_t)(uint8_t fault);

/* ============================================================
 * 回转角度对象结构体
 * ============================================================ */
typedef struct {
    /* ----- 物理量 ----- */
    int32_t  current_angle_x10;     /* 当前角度（-1000 ~ +2150） */
    int32_t  encoder_zero_offset;   /* 编码器零位偏移 */

    /* ----- 位置状态 ----- */
    swing_position_t position;
    swing_position_t last_position;
    uint8_t  is_at_left_limit;
    uint8_t  is_at_right_limit;
    uint8_t  is_at_zero;
    uint8_t  is_at_middle;

    /* ----- 限位常量 ----- */
    int32_t left_limit_x10;         /* -1000 (-100.0°) */
    int32_t right_limit_x10;        /* +2150 (+215.0°) */
    int32_t zero_center_x10;        /* 0 (0.0°) */
    int32_t limit_tolerance_x10;    /* ±20 (±2°) */
    int32_t zero_tolerance_x10;     /* ±5 (±0.5°) */

    /* ----- 依赖注入 ----- */
    swing_encoder_getter_t get_encoder;
    void *user_data;

    /* ----- 事件回调 ----- */
    swing_position_change_cb_t on_position_change;

    /* ----- 编码器故障上报（2026-11-23） -----
     * on_fault_change：解算有效性变化时通知上层（1=故障 0=恢复）
     * enc_fault_reported：已上报标志。既用于防重复投递，
     *   也用于判定"恢复"（只有报过才需要清）。 */
    swing_fault_change_cb_t on_fault_change;
    uint8_t enc_fault_reported;

    /* ----- 过零停车（2026-11-10 新增，REQ-017.1）-----
     *
     * need_zero_stop = 1 时，update() 内做符号翻转过零检测，
     * 检出后投 SWING_EVT_ZERO_REACHED（状态机据此停车+锁刹车）。
     *
     * 【默认 0 = 不做过零停车】
     *   用户 2026-11-10 裁定：
     *   「手动模式，跳过执行过零位的操作方式」
     *   ⇒ 手动模式下不启用，置 0。
     *
     * 【为什么不直接删掉这段逻辑】
     *   过零检测是序列"回位"场景的必需能力（跨零位到对侧），
     *   只是当前阶段不启用。保留开关便于后续按需打开。 */
    uint8_t need_zero_stop;

    uint8_t is_initialized;
} swing_angle_t;

/* ============================================================
 * API
 * ============================================================ */

void swing_angle_init(swing_angle_t *angle,
                     int32_t encoder_zero_offset,
                     swing_encoder_getter_t getter,
                     void *user_data);

void swing_angle_bind_sensor(swing_angle_t *angle,
                            swing_encoder_getter_t getter,
                            void *user_data);

void swing_angle_update(swing_angle_t *angle);

void swing_angle_calibrate(swing_angle_t *angle, int32_t encoder_raw);

/**
 * @brief 把"当前位置"标定为 0°（P9 / B-10，2026-11-23）
 * @note  等价于 offset_new = offset_old + current_angle_x10（角度 = raw − offset）。
 *        只改 RAM 的 encoder_zero_offset，落盘走快区 tube_angle_x10（批次 4 回灌消费）。
 */
void swing_angle_zero_here(swing_angle_t *angle);

swing_position_t swing_angle_get_position(const swing_angle_t *angle);
uint8_t swing_angle_is_left_limit(const swing_angle_t *angle);
uint8_t swing_angle_is_right_limit(const swing_angle_t *angle);
uint8_t swing_angle_is_zero(const swing_angle_t *angle);
uint8_t swing_angle_is_middle(const swing_angle_t *angle);
int32_t swing_angle_get_value(const swing_angle_t *angle);
uint8_t swing_angle_is_safe(const swing_angle_t *angle);

/**
 * @brief  三编码器解算是否可信（2026-11-10 新增）
 * @retval 1=可信（mode ∈ COARSE/INCR 且 last_err==OK）；0=不可信
 *
 * @note  过零/限位判定的前提。解算失败时角度可能是任意值，
 *        上层应报编码器故障而不是拿它判位置。
 */
uint8_t swing_angle_is_solve_valid(void);

/* ============================================================
 * 全局对象引用
 * ============================================================ */
extern swing_angle_t g_swing_angle;

swing_angle_t* swing_angle_get(void);

#ifdef __cplusplus
}
#endif

#endif /* __SWING_ANGLE_H */
