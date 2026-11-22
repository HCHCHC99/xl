/**
 * @file    motor_config.h
 * @brief   电机配置表（六大维度 + 安全层）
 * @note    编译期配置 + 运行期配置
 */

#ifndef __MOTOR_CONFIG_H
#define __MOTOR_CONFIG_H

#include <stdint.h>
#include <rtthread.h>
#include "motor_types.h"
#include "soft_ramp.h"
#include "motor_oc.h"
//#include "motor_drv.h"
//#include "motor_fb.h"
//#include "motor_ctrl.h"


#ifdef __cplusplus
extern "C" {
#endif


/* ============================================================
 * 前置声明 ops 类型
 * ============================================================ */
typedef struct motor_drv_ops  motor_drv_ops_t;
typedef struct motor_fb_ops   motor_fb_ops_t;
typedef struct motor_ctrl_ops motor_ctrl_ops_t;


/* ============================================================
 * 硬件配置结构（与 BSP 层对接）
 * ============================================================ */

/* ----- PWM 驱动配置 ----- */
typedef struct {
    uint32_t pwm_freq_hz;           /* PWM 频率（如 20000） */
    uint32_t pwm_deadtime_ns;       /* 死区时间（如 1000ns） */
    uint32_t max_duty;              /* 最大占空比（如 1000） */
} motor_pwm_cfg_t;

/* ----- Hall 换向配置 ----- */
typedef struct {
    uint8_t  hall_state_table[8];   /* 霍尔状态 → 换向序号（1~6），0 为无效 */
    uint8_t  hall_active_level;     /* 有效电平（0 或 1） */
} motor_hall_cfg_t;

///* ============================================================
// * 过流保护配置（安全层）
// * ============================================================ */
//typedef struct {
//    /* ----- 额定值 ----- */
//    uint16_t rated_current_a10;     /* 额定电流（0.1A），370 = 37A */
//
//    /* ----- 一级过流（警告） ----- */
//    uint16_t oc1_current_a10;       /* 阈值，450 = 45A */
//    uint16_t oc1_debounce_ms;       /* 去抖，500ms */
//
//    /* ----- 二级过流（降功率） ----- */
//    uint16_t oc2_current_a10;       /* 阈值，500 = 50A */
//    uint16_t oc2_debounce_ms;       /* 去抖，100ms */
//    uint8_t  oc2_deboost_pct;       /* 降功率百分比，如 50 = 50% */
//
//    /* ----- 三级过流（紧急停机） ----- */
//    uint16_t oc3_current_a10;       /* 阈值，550 = 55A */
//    uint16_t oc3_debounce_ms;       /* 去抖，10ms */
//
//    /* ----- 恢复策略 ----- */
//    uint8_t  auto_recover;          /* 0=需手动复位，1=自动恢复 */
//} motor_oc_cfg_t;

/* ============================================================
 * 电机主配置（六维度 + 接口绑定）
 * ============================================================ */
typedef struct motor_config {
    /* ----- 身份标识 ----- */
    const char        *name;

    /* ----- D1：电机类型 ----- */
    motor_type_t       type;
    uint8_t            pole_pairs;      /* 极对数（如 4） */

    /* ----- D2：驱动方式 ----- */
    motor_drv_hw_t     drv_hw;
    const void        *drv_hw_cfg;      /* motor_pwm_cfg_t 等 */

    /* ----- D3：换向信号 ----- */
    motor_comm_t       comm_type;
    const void        *comm_cfg;        /* motor_hall_cfg_t 等 */

    /* ----- D4：驱动算法（默认） ----- */
    motor_algo_t       default_algo;

    /* ----- D5：工作模式（默认） ----- */
    motor_work_mode_t  default_mode;

    /* ----- D6：软启动/软停 ----- */
    const soft_ramp_cfg_t *ramp_cfg;

    /* ----- 安全层：过流保护 ----- */
    const motor_oc_cfg_t  *oc_cfg;

    /* ----- 三层接口绑定 ----- */
    const motor_drv_ops_t  *drv_ops;
    const motor_fb_ops_t   *fb_ops;
    const motor_ctrl_ops_t *ctrl_ops;

    const void            *ctrl_cfg;

    /* ----- 实时性 ----- */
    uint32_t update_period_ms;          /* 控制环周期（如 5ms） */
    uint32_t drv_update_period_us;      /* 驱动层周期（如 100us） */
} motor_config_t;

/* ============================================================
 * 运行期配置（可通过 API / CAN / FinSH 修改）
 * ============================================================ */
typedef struct {
    motor_work_mode_t work_mode;        /* 当前工作模式 */
    motor_algo_t      active_algo;      /* 当前驱动算法 */
    soft_ramp_cfg_t   ramp_cfg;         /* 软启动参数（可改） */
    motor_oc_cfg_t    oc_cfg;           /* 过流保护（可改） */

    /* ----- 过流运行状态 ----- */
    motor_oc_state_t  oc_state;         /* 当前过流级别 */
    uint32_t          oc_start_tick;    /* 过流触发时间 */
    uint32_t          oc_trigger_count; /* 累计触发次数 */
} motor_runtime_t;

/* ============================================================
 * 全局配置表
 * ============================================================ */
extern const motor_config_t g_motor_configs[];
extern const uint32_t       g_motor_config_count;

/**
 * @brief 按名称查找配置
 */
const motor_config_t* motor_config_find(const char *name);

/**
 * @brief 获取默认配置（第一个）
 */
const motor_config_t* motor_config_default(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_CONFIG_H */
