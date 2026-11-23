/**
 * @file    seq_control.h
 * @brief   序列控制对象
 */

#ifndef __SEQ_CONTROL_H
#define __SEQ_CONTROL_H

#include <stdint.h>
#include "../framework/core/control_object.h"
#include "../framework/manager/ctrl_manager.h"
#include "seq_engine.h"
#include "../app_event.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 序列控制上下文
 * ============================================================ */
typedef struct {
    seq_context_t    engine;            /* 序列引擎上下文 */
    uint32_t         request_count;     /* 请求计数 */
    uint32_t         reject_count;      /* 拒绝计数（= 下面四项之和） */

    /* 【P3】拒绝分项，便于现场判断被拒原因 */
    uint32_t         rej_dup;           /* 同键重复被忽略 */
    uint32_t         rej_precond;       /* 前置条件不满足 */
    uint32_t         rej_limit;         /* 超限禁止 */
    uint32_t         rej_multi;         /* 同拍多键 */

    /* 一键按键上升沿检测 */
    uint8_t          prev_middle;
    uint8_t          prev_rear;
    uint8_t          prev_home;

    /* 【P2】当前在跑的一键：0 无 / 1 右中 / 2 右后 / 3 回位 */
    uint8_t          current_onekey;

    /* 【P2】最近一次中止原因：0 无 / 1 被中断 / 2 用户中止 / 3 步骤超时或引擎自停 */
    uint8_t          abort_reason;

    /* 启车状态【D-11】事件侧副本，仅用于诊断/日志；
     * 门禁一律读 vehicle_is_started()，不要用本字段判断能否启动 */
    uint8_t          vehicle_started;
} seq_ctrl_context_t;

/* ============================================================
 * 序列控制对象
 * ============================================================ */
typedef struct {
    control_object_t base;
    StateMachine_t   sm_instance;
    seq_ctrl_context_t ctx;
} seq_control_t;

/* ============================================================
 * 外部引用
 * ============================================================ */
extern seq_control_t g_seq_obj;

/* ============================================================
 * 触发返回码（P3）
 * ============================================================ */
#define SEQ_RET_OK          0       /* 已启动（或已中断旧序列并启动新序列） */
#define SEQ_RET_DUP        (-1)     /* 同键重复：忽略，序列继续 */
#define SEQ_RET_PRECOND    (-2)     /* 前置条件不满足：未启车 / 模式 / 故障 / 有手动键 */
#define SEQ_RET_LIMIT      (-3)     /* 超限禁止（当前仅"超右时的右后"） */
#define SEQ_RET_MULTI      (-4)     /* 同拍多键：全部不启动 */

/* ============================================================
 * API
 * ============================================================ */

int  seq_control_register(ctrl_manager_t *mgr);

/**
 * @brief 一键按键更新（上升沿触发，兼容旧接口）
 * @note  P2 起键盘直接投 SEQ_EVT_ONKEY_*，本接口仅为兼容保留；
 *        实际逻辑已收敛到 seq_control_trigger()。
 */
void seq_control_update_onekey(uint8_t middle, uint8_t rear, uint8_t home);

/**
 * @brief 触发一个一键动作（单键入口）
 * @param key 1 = 右中 / 2 = 右后 / 3 = 回位
 * @return SEQ_RET_*（0 = 成功；-1/-2/-3/-4 = 各类拒绝）
 */
int  seq_control_trigger(uint8_t key);

/**
 * @brief 触发一键动作（组合入口，P3）
 * @param mid  1 = 一键右中被按下
 * @param rear 1 = 一键右后被按下
 * @param home 1 = 一键回位被按下
 * @return SEQ_RET_*；两个及以上同时为 1 时返回 SEQ_RET_MULTI
 * @note  仅用于 msh（`t_seq_run mid+rear+home`）与将来的 CAN 组合注入；
 *        键盘路径由 keyboard_sm 在"同帧多键"时直接把一键位清零，不会走到这里。
 */
int  seq_control_trigger_multi(uint8_t mid, uint8_t rear, uint8_t home);

/**
 * @brief 序列对象的事件分发入口（P2）
 * @note  由 app_main 在描述表命中后调用。序列对象的状态机只有 IDLE/ACTIVE
 *        两态（SEQ_OBJ_EVT_START/FINISH/ABORT），一键按键本身不引起状态迁移，
 *        因此需要在投递后补一次"命令分发"。
 */
void seq_control_on_event(app_event_t event);

/**
 * @brief 查询当前一键类型名称（1/2/3 → "RIGHT_MIDDLE"/"RIGHT_BEHIND"/"HOME"）
 */
const char *seq_control_onekey_name(uint8_t key);

void seq_control_set_vehicle_started(uint8_t started);

void seq_control_update(void);

seq_ctrl_context_t* seq_control_get_context(void);

#ifdef __cplusplus
}
#endif

#endif /* __SEQ_CONTROL_H */
