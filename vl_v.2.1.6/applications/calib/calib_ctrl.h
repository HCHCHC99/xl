/**
 * @file    calib_ctrl.h
 * @brief   零点校准状态机（P9 / B-10，2026-11-23）
 *
 * ============================================================
 * 口径（选项 A，业务.md §13.9）
 * ============================================================
 * 「业务零点」不单独落盘——ABZ 是增量编码器，上电后 count 没有绝对意义，
 * 单存一个零点跨不了上电。校准的动作是：
 *   按键流程 → Foc_Proc_Start(FOC_PROC_ENC_ALIGN)（两点式，即 focmode 3）
 *            → 成功后 is_calibrated=1 + 立即写快区（offset/dir/tube_x10=0，
 *              valid = bit0|bit1）+ 筒角置 0（swing_angle_zero_here）。
 * 跨上电靠快区快照（批次 4 的上电回灌消费），本批只保证"落盘且重启后
 * t_param 里还在"。
 *
 * 按键流程（TC-CAL-001~011）：
 *   双击(≤1s) → PREP 黄灯 5s → 5s 内再按(按住) → ARMED 绿 5Hz 闪
 *             → 长按 ≥2s → EXEC → 成功绿 / 失败红
 *             → 不足 2s 松开 → FAIL 红
 *   每上电周期最多 2 次（TC-CAL-008/010）；一键运行中拒绝（TC-CAL-011）。
 *
 * ⚠ uv_snapshot_save() 必须同批改为"保留 tube_angle_x10 / bit1"，
 *   否则校准后一次欠压就把筒角清零（app_main.c，同批已改）。
 */

#ifndef __CALIB_CTRL_H
#define __CALIB_CTRL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 状态（led_effect / t_calib / 状态帧共用）
 * ============================================================ */
typedef enum {
    CALIB_ST_IDLE = 0,      /* 空闲 */
    CALIB_ST_PREP,          /* 准备态：黄灯，5s 内等"再按" */
    CALIB_ST_ARMED,         /* 就绪态：按住中，绿 5Hz 闪，等长按 ≥2s */
    CALIB_ST_EXEC,          /* 执行中：跑 FOC_PROC_ENC_ALIGN */
    CALIB_ST_DONE,          /* 成功：绿常亮 2s 后回 IDLE */
    CALIB_ST_FAIL,          /* 失败：红常亮 2s 后回 IDLE */
} calib_state_t;

/**
 * @brief 初始化（app_main 线程启动时调用一次）
 */
void calib_ctrl_init(void);

/**
 * @brief 喂校准事件（CALIB_EVT_CLICK / DOUBLE_CLICK / LONG_PRESS / KEY_UP）
 * @note  app_main 事件循环把 0x3000 区间转发进来；t_calib sim 也走这里。
 */
void calib_ctrl_feed(uint16_t event);

/**
 * @brief 50ms 周期推进（PREP 超时 / EXEC 轮询 / DONE、FAIL 回 IDLE）
 */
void calib_ctrl_tick_50ms(void);

/**
 * @brief 直接启动（绕过按键时序，前置条件照常检查）
 * @return 0 = 进入执行；-1 = 被拒（reason 可用 calib_ctrl_reason() 取）
 */
int calib_ctrl_start(void);

/**
 * @brief 强制回 IDLE（EXEC 中会同时 Foc_Proc_Abort()）
 */
void calib_ctrl_abort(void);

/* ----- getter（led_effect / t_calib 用） ----- */
uint8_t     calib_ctrl_state(void);
uint8_t     calib_ctrl_count(void);         /* 本上电周期已用次数 */
const char *calib_ctrl_state_name(uint8_t st);
const char *calib_ctrl_reason(void);        /* 最近一次拒绝/失败原因（静态串） */

#ifdef __cplusplus
}
#endif

#endif /* __CALIB_CTRL_H */
