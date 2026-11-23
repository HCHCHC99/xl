/**
 * @file    lift_control.h
 * @brief   举升控制状态机（上升/下降/限位保护）
 * @note    状态机引擎 + 举升角对象协同工作
 */

#ifndef __LIFT_CONTROL_H
#define __LIFT_CONTROL_H

#include <stdint.h>
#include "../framework/core/control_object.h"
#include "../framework/manager/ctrl_manager.h"
#include "lift_angle.h"
#include "hw_objects.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 状态枚举
 * ============================================================ */
typedef enum {
    P_STATE_IDLE = 0,           /* 空闲，无输出 */
    P_STATE_UP_RUNNING,         /* 上升中，上升阀得电 */
    P_STATE_DOWN_RUNNING,       /* 下降中，下降阀得电 */
    P_STATE_LIMIT_HOLDING,      /* 限位保持中（5秒延时） */
    P_STATE_FAULT_VALVE,        /* 电磁阀故障：完全禁止输出 */
    P_STATE_FAULT_SENSOR,       /* 传感器故障：限位失效，仅允许手动 */
    P_STATE_MAX
} lift_state_t;

/* ============================================================
 * 举升控制上下文
 * ============================================================ */
typedef struct {
    /* ----- 硬件依赖 ----- */
    solenoid_box_t  *solenoid_ptr;      /* 电磁阀对象 */
    lift_angle_t   *lift_ptr;         /* 举升角对象 */

    /* ----- 开关状态（来自 CAN 解析） ----- */
    uint8_t         up_sw_pressed;      /* 上升开关状态 (0/1) */
    uint8_t         down_sw_pressed;    /* 下降开关状态 (0/1) */

    /* ----- 限位保持状态 ----- */
    uint32_t        hold_start_tick;    /* 进入保持状态的时间戳 */
    uint8_t         hold_direction;     /* 1=上升方向, 0=下降方向 */
    uint8_t         valve_was_on;       /* 离开 UP/DOWN_RUNNING 时阀是否真的通过电（P6/B-7） */

    /* ----- 故障状态 ----- */
    uint8_t         valve_fault_up;     /* 上升阀故障 (1=故障) */
    uint8_t         valve_fault_down;   /* 下降阀故障 (1=故障) */
    uint8_t         sensor_fault;       /* 传感器故障 (1=故障) */

    /* ----- CAN 反馈（用于诊断） ----- */
    uint8_t         valve_fb_up;        /* 上升阀反馈 (1=打开) */
    uint8_t         valve_fb_down;      /* 下降阀反馈 (1=打开) */
} lift_context_t;

/* ============================================================
 * 控制对象
 * ============================================================ */
typedef struct {
    control_object_t base;
    StateMachine_t   sm_instance;
    lift_context_t  ctx;
} lift_control_t;

/* ============================================================
 * 外部引用：
 * ============================================================ */
extern lift_control_t g_lift_obj;

/* ============================================================
 * API
 * ============================================================ */

/**
 * @brief 注册举升控制对象到管理器
 */
int lift_control_register(ctrl_manager_t *mgr);

/**
 * @brief 更新开关状态
 */
void lift_control_update_switch(uint8_t up, uint8_t down);

/**
 * @brief 更新电磁阀故障状态
 * @param up_fault   0x18FF0E63 上升阀原始故障码（0 正常 / 1 开路 / 2 短路）
 * @param down_fault 下降阀原始故障码（同上）
 * @note  【P4 / B-5】入参由"布尔"改为"原始码"：去抖需要保留开路/短路的区分，
 *        且 11D1 Byte2 也按 01/10 上报。是否确认由 lift_control_valve_fb_poll()
 *        按 confirm_ms 判定，本函数只登记原始码与计时起点。
 */
void lift_control_update_valve_fault(uint8_t up_fault, uint8_t down_fault);

/**
 * @brief 阀故障去抖轮询（50 ms tick 调用）
 * @note  连续满足 confirm_ms 才确认并投 LIFT_EVT_VALVE_FAULT；
 *        故障码消失立即清除（不延时），投 LIFT_EVT_VALVE_FAULT_CLEAR。
 */
void lift_control_valve_fb_poll(void);

/* ---- 阀故障去抖诊断（t_valve_fb 用） ---- */
typedef struct {
    uint8_t  raw;          /* 最近一次原始码 0/1/2 */
    uint8_t  latched;      /* 已确认的故障 */
    uint8_t  timing;       /* 正在计时 */
    uint32_t elapsed_ms;   /* 已持续时长（计时中有效） */
} lift_vfb_ch_t;

typedef struct {
    uint8_t        debounce_en;   /* 1=去抖（默认） 0=立即报（旧行为） */
    uint16_t       confirm_ms;    /* 确认门限 */
    lift_vfb_ch_t  ch[2];         /* 0=上升阀 1=下降阀 */
} lift_vfb_state_t;

/**
 * @brief 读去抖状态
 */
void lift_control_valve_fb_get(lift_vfb_state_t *out);

/**
 * @brief 设去抖开关与门限（纯 RAM，重启回默认 500 ms / 开启）
 */
void lift_control_valve_fb_set(uint8_t debounce_en, uint16_t confirm_ms);

/**
 * @brief 更新传感器故障状态
 */
void lift_control_update_sensor_fault(uint8_t fault);

/**
 * @brief 更新电磁阀反馈
 */
void lift_control_update_valve_fb(uint8_t up_fb, uint8_t down_fb);

/**
 * @brief 启动 5s 超时定时器
 */
void lift_control_start_timer(void);

/**
 * @brief 停止 5s 超时定时器
 */
void lift_control_stop_timer(void);

/**
 * @brief 限位保持剩余时间（P6 / B-7）
 * @return >0 剩余 ms；-1 当前不在保持态
 */
int32_t lift_control_hold_remaining_ms(void);

/**
 * @brief 获取当前状态
 */
lift_state_t lift_control_get_state(void);

/**
 * @brief 获取上下文
 */
lift_context_t* lift_control_get_context(void);

#ifdef __cplusplus
}
#endif

#endif /* __LIFT_CONTROL_H */
