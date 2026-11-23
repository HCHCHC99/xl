/**
 * @file    arbiter.h
 * @brief   业务事件仲裁器（P1：影子骨架，只记录不拦截）
 * @note    零 RTOS 依赖：本头文件与 arbiter.c 只用 stdint。
 *          tick / 快照采集放在 arb_snap.c，msh 打印放在 arb_cmd.c。
 *
 *          设计见 业务.md §14.7 / §14.9.2：
 *          - 集中仲裁器 + 正交模块
 *          - 优先级 STOP_ALL > CLUTCH > MANUAL > ONEKEY > CALIB
 *          - P1 只搭骨架：所有仲裁类事件都判 GRANT，通过与"实际是否投递"
 *            对账来暴露链路断点，不改变任何业务行为。
 */

#ifndef __ARBITER_H
#define __ARBITER_H

#include <stdint.h>
#include "app_event.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 请求类别（按业务语义分，不按事件区间）
 * ============================================================ */
typedef enum {
    ARB_CLS_NONE = 0,       /* 未分类 */
    ARB_CLS_STOP_ALL,       /* 急停 / 关阀 / 回 IDLE */
    ARB_CLS_CLUTCH,         /* 离合结合/分离 */
    ARB_CLS_MANUAL,         /* 手动四向按下 */
    ARB_CLS_ONEKEY,         /* 一键序列 */
    ARB_CLS_CALIB,          /* 校准 */
    ARB_CLS_PASS,           /* 不需要仲裁：松开/限位/故障/状态刷新等 */
    ARB_CLS_MAX
} arb_class_t;

/* ============================================================
 * 被仲裁的模块
 * ============================================================ */
typedef enum {
    ARB_MOD_NONE = 0,
    ARB_MOD_LIFT,
    ARB_MOD_SWING,
    ARB_MOD_SEQ,
    ARB_MOD_CALIB,
    ARB_MOD_CLUTCH,
    ARB_MOD_MAX
} arb_module_t;

/* ============================================================
 * 决策与模式
 * ============================================================ */
typedef enum {
    ARB_DEC_GRANT = 0,      /* 放行 */
    ARB_DEC_REJECT,         /* 拒绝 */
    ARB_DEC_PREEMPT,        /* 抢占（先停 owner，再执行）*/
} arb_decision_t;

typedef enum {
    ARB_MODE_SHADOW = 0,    /* 只记录，不拦截（P1 固定）*/
    ARB_MODE_ENFORCE,       /* 真拦截（P2 起才允许）*/
} arb_mode_t;

/* ============================================================
 * 一次仲裁请求
 * ============================================================ */
typedef struct {
    app_event_t  event;
    arb_class_t  cls;
    arb_module_t module;        /* 请求要驱动哪个模块 */
    uint32_t     tick;          /* 调用方填（arbiter.c 不依赖 rt_tick_get）*/
} arb_req_t;

/* ============================================================
 * 状态快照（由 arb_snap.c 采集，唯一接触各模块 getter 的地方）
 * ============================================================ */
typedef struct {
    uint8_t      acc_on;
    uint8_t      vehicle_started;
    uint8_t      uv;
    uint32_t     fault_word;    /* 整车故障汇聚字（只读）*/
    uint8_t      seq_active;
    uint8_t      angle_valid;   /* 【P2】回转角解算是否有效（solve_valid）*/
    uint8_t      ref_valid;     /* 筒角参考是否有效（批次 4 前恒 0）*/
    arb_module_t owner;         /* 仲裁器观察到的当前 owner */
    uint32_t     tick;
} arb_snap_t;

/* ============================================================
 * 一次仲裁结果
 * ============================================================ */
typedef struct {
    arb_decision_t dec;
    const char    *reason;
} arb_result_t;

/* ============================================================
 * 影子日志（环形，供 t_arb log 查看最近事件）
 * ============================================================ */
#define ARB_LOG_MAX     8U

typedef struct {
    app_event_t    event;
    arb_class_t    cls;
    arb_module_t   module;
    arb_decision_t dec;
    arb_module_t   owner_before;
    uint8_t        delivered;   /* 1 = 已投给对象；0 = 未投递 */
    uint32_t       tick;
} arb_log_t;

/* ============================================================
 * 统计
 * ============================================================ */
typedef struct {
    uint32_t submitted;         /* 已决策条数（仲裁类）*/
    uint32_t grant;
    uint32_t reject;
    uint32_t preempt;
    uint32_t mismatch_link;     /* GRANT 但未投递：链路断 */
    uint32_t mismatch_bypass;   /* 非 GRANT 却投递：旁路 */
    uint32_t checked;           /* 已对账条数（仲裁类）*/
} arb_stat_t;

/* ============================================================
 * 核心 API（arbiter.c，零 RTOS）
 * ============================================================ */

/** 事件分类：PASS 表示不需要仲裁，module 可为 NONE */
arb_class_t arb_classify(app_event_t evt, arb_module_t *module);

/** 提交一次仲裁：填 result + 记录 last（等待 actual 对账）*/
arb_result_t arb_submit(const arb_req_t *req, const arb_snap_t *snap);

/** 影子对账：delivered=1 表示事件确实交给了对象 */
void arb_shadow_actual(uint8_t delivered);

arb_mode_t   arb_get_mode(void);
int          arb_set_mode(arb_mode_t mode);   /* P1 只接受 SHADOW */
arb_module_t arb_get_owner(void);

/** 【P2】释放 owner：仅当当前 owner 就是 m 时置 NONE（由 arb_owner_sync 调用）*/
void arb_release_owner(arb_module_t m);

void arb_get_stat(arb_stat_t *out);
uint8_t arb_get_last(arb_log_t *out);              /* 1 = 有记录 */
uint8_t arb_get_log(arb_log_t *out, uint8_t max);  /* 返回条数，旧→新 */
void arb_reset_stats(void);

/* ============================================================
 * 快照 / 探针（arb_snap.c，依赖 RTOS getter）
 * ============================================================ */

/** 采集当前状态快照 */
void arb_snap_fill(arb_snap_t *snap);

/** app_main 路由前调用：分类 + 采集快照 + 提交仲裁（影子只记录）*/
void arb_shadow_probe(app_event_t evt);

/**
 * @brief 【P2】owner 释放：模块回到 IDLE 后让出 owner
 * @note  由 app_main 的 50 ms tick 调用。只释放、不抢占，影子模式与
 *        enforce 均需要；否则 owner 会一直停在最后一个"被投递过的模块"。
 */
void arb_owner_sync(void);

#ifdef __cplusplus
}
#endif

#endif /* __ARBITER_H */
