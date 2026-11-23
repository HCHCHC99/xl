/**
 * @file    swing_axis.h
 * @brief   回转轴适配层（H1-A）
 *
 * 职责（唯一 PWM/FOC 业务出口）：
 *   1. 手动左/右：位置环增量跟随（动态步长，见 业务.md §15.19）
 *   2. 每次停→动：解锁抱闸 → α 一点式对齐 200ms → 等到 500ms → 位置环
 *   3. 松开/停止：位置环减速到 in_pos → FOC IDLE → 物理抱闸锁止
 *   4. 限位/零点 guard、t_axis/t_axis zero 诊断
 *
 * 不负责：一键绝对目标（H1-B）、离合（P11 暂缓）、SPI 绝对编码器。
 */

#ifndef __SWING_AXIS_H
#define __SWING_AXIS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 方向：与 swing_control 的 pending_dir 一致 */
#define SWING_AXIS_DIR_LEFT     1U
#define SWING_AXIS_DIR_RIGHT    2U

typedef enum {
    SWING_AXIS_IDLE = 0,
    SWING_AXIS_WAIT_BRAKE,      /* 等物理抱闸推定解锁 */
    SWING_AXIS_ALIGN,           /* α 一点式对齐 200ms */
    SWING_AXIS_GATE,            /* 等到“按键时刻 + 500ms” */
    SWING_AXIS_POSITION,        /* 位置环增量跟随 */
    SWING_AXIS_STOPPING,        /* 松开后等 in_pos / 低速 */
    SWING_AXIS_LOCK_WAIT,       /* 等物理抱闸推定锁止 */
    SWING_AXIS_FAULT,
} swing_axis_state_t;

void    swing_axis_init(void);

/* 手动按键按下（enter_unlocking 时调用），记录 500ms 起点 */
void    swing_axis_prepare(uint8_t dir);

/* 进入 LEFT/RIGHT_RUNNING 时调用；返回 0 = 拒绝（未置零/方向被限） */
uint8_t swing_axis_start(void);

/* 松开/停止；进入 LOCKING 时调用。位置环会先走完最后一个增量。 */
void    swing_axis_stop(void);

/* 故障/急停：立即 FOC IDLE + 锁抱闸 */
void    swing_axis_abort(void);

/* 50ms tick，由 swing_act_tick_check() 调用 */
void    swing_axis_tick(void);

/* t_axis zero：当前筒角置 0，并让位置环零点一致 */
void    swing_axis_zero_here(void);

/* 观测 */
swing_axis_state_t swing_axis_get_state(void);
uint8_t swing_axis_is_idle(void);
const char *swing_axis_state_name(swing_axis_state_t st);

/* H1-B 预留：绝对位置目标；H1-A 只给 t_axis move 做换算验证 */
int32_t swing_axis_move_to(int32_t angle_x10);

#ifdef __cplusplus
}
#endif

#endif /* __SWING_AXIS_H */
