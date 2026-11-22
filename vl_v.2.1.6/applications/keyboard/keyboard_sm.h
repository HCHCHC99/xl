/**
 * @file    keyboard_sm.h
 * @brief   L3 键盘指令状态机（独立线程，事件驱动 + 300ms 软件定时器）
 *
 * 【架构】
 *   can_parser 线程 ──kb_on_can_frame()──→ 快照 + 重启 300ms 定时器 + 唤醒
 *   定时器线程     ──超时回调────────────→ 快照清零 + 唤醒
 *   keyboard 线程  ──kb_sm_poll──────────→ 状态机 + 事件投递 + 上下文同步
 *
 * 【为什么纯事件驱动】
 *   按键状态机没有"时间累积"类状态（无去抖、无延时），
 *   只在「收到新帧」与「超时」两个时刻需要推进，故不需要周期轮询。
 */

#ifndef __KEYBOARD_SM_H
#define __KEYBOARD_SM_H

#include <stdint.h>
#include "keyboard_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 对象
 * ============================================================ */
typedef struct {
    ksm_state_t state;
    key_code_t  cur_key;
    key_cmd_t   cmd;            /* 本拍指令 */
    key_cmd_t   prev_cmd;       /* 上拍指令（跳变检测） */

    /* 两轴互斥消解记忆（独立，异轴组合不受影响） */
    kb_axis_mem_t lift_mem;    /* 举升：pos=上升, neg=下降 */
    kb_axis_mem_t swing_mem;     /* 回转：pos=左转, neg=右转 */

    /* 统计 */
    uint32_t    cnt_frames;     /* 收到帧数 */
    uint32_t    cnt_blocked;    /* 进入锁死次数（仅启车重按） */
    uint32_t    cnt_conflict;   /* 互斥判定次数 */
    uint32_t    cnt_reject;     /* 一键被手动拒绝次数 */
    uint32_t    cnt_timeout;    /* 超时次数 */
} kb_sm_t;

/* ============================================================
 * API
 * ============================================================ */

/**
 * @brief 初始化：创建信号量、300ms 一次性定时器、keyboard 线程 [单线程]
 * @return 0 成功；<0 失败
 */
int kb_init(void);

/**
 * @brief 收到面板报文时调用（can_parser 线程） [线程安全]
 * @param data 8 字节报文
 * @param len  长度
 */
void kb_on_can_frame(const uint8_t *data, uint8_t len);

/**
 * @brief 请求「重新按下」：当前按住的键作废，须松开再按 [线程安全]
 * @note  用于启车瞬间（SRS：启车后需重新按下）。
 *        若此时有键按着，内部进入锁死态，直到全部松开才恢复。
 */
void kb_request_rebaseline(void);

/**
 * @brief 查询当前指令（只读） [线程安全]
 */
const key_cmd_t *kb_get_cmd(void);

/**
 * @brief 查询当前状态机状态 [线程安全]
 */
ksm_state_t kb_get_state(void);

/**
 * @brief 状态名（调试）
 */
const char *kb_sm_state_name(ksm_state_t s);

#ifdef __cplusplus
}
#endif

#endif /* __KEYBOARD_SM_H */
