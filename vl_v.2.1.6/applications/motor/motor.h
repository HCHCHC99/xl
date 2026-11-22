/**
 * @file    motor.h
 * @brief   电机对象统一接口（三层桥接架构）
 * @note    支持自由组合：驱动层 + 反馈层 + 控制层
 */

#ifndef __MOTOR_H
#define __MOTOR_H

#include <stdint.h>
#include <rtthread.h>
#include "motor_types.h"
#include "soft_ramp.h"
#include "motor_oc.h"

/*  前向声明：让 motor_drv.h / motor_fb.h / motor_ctrl.h 能引用 motor_t */
typedef struct motor motor_t;
/* ----- ops 类型完整定义 ----- */
#include "motor_drv.h"
#include "motor_fb.h"
#include "motor_ctrl.h"

/* ----- 配置类型（仅使用 ops 指针） ----- */
#include "motor_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 前置声明
 * ============================================================ */
typedef struct motor motor_t;

///* ============================================================
// * 控制模式枚举
// * ============================================================ */
//typedef enum {
//    MOTOR_CTRL_MODE_OPEN_LOOP = 0,
//    MOTOR_CTRL_MODE_SPEED,
//    MOTOR_CTRL_MODE_POSITION,
//    MOTOR_CTRL_MODE_SPEED_CURRENT,
//    MOTOR_CTRL_MODE_POS_SPD_CUR,
//} motor_ctrl_mode_t;

/* ============================================================
 * 运行状态枚举
 * ============================================================ */
typedef enum {
    MOTOR_STATE_IDLE = 0,       /* 空闲 */
    MOTOR_STATE_RUNNING,        /* 运行中 */
    MOTOR_STATE_RAMPING,        /* 软启动/停止中 */
    MOTOR_STATE_BRAKING,        /* 刹车中 */
    MOTOR_STATE_FAULT,          /* 故障 */
} motor_state_t;

/* ============================================================
 * 方向定义
 * ============================================================ */
#define MOTOR_DIR_CW        1       /* 正转 */
#define MOTOR_DIR_CCW       2       /* 反转 */

/* ============================================================
 * 电机对象结构体
 * ============================================================ */
struct motor {
    /* ----- 身份标识 ----- */
    const char        *name;
    motor_type_t       type;
    uint8_t            pole_pairs;

    /* ----- 三层接口 ----- */
    const motor_drv_ops_t  *drv_ops;
    void                   *drv_data;
    const motor_fb_ops_t   *fb_ops;
    void                   *fb_data;
    const motor_ctrl_ops_t *ctrl_ops;
    void                   *ctrl_data;

    /* ----- 运行时状态 ----- */
    motor_state_t  state;
    uint8_t        direction;
    int32_t        target_position;
    int32_t        target_speed;
    int32_t        target_output;
    int32_t        current_position;
    int32_t        current_speed;
    int32_t        current_current;
    uint8_t        fault;

    /* ----- 软启动/停止 ----- */
    soft_ramp_t    ramp;

    /* ----- 过流保护（安全层） ----- */
    motor_oc_ctx_t oc_ctx;

    /* ----- 周期控制 ----- */
    uint32_t       last_update_tick;
    uint32_t       update_period_ms;

    /* ----- 配置引用 ----- */
    const motor_config_t *config;
};

/* ============================================================
 * 公开 API
 * ============================================================ */

/**
 * @brief 初始化电机对象
 */
rt_err_t motor_init(motor_t *motor, const motor_config_t *cfg);

/**
 * @brief 反初始化
 */
rt_err_t motor_deinit(motor_t *motor);

/**
 * @brief 启动电机（指定方向）
 */
rt_err_t motor_start(motor_t *motor, uint8_t direction);

/**
 * @brief 停止电机（触发软停止）
 */
rt_err_t motor_stop(motor_t *motor);

/**
 * @brief 刹车
 */
rt_err_t motor_brake(motor_t *motor);

/**
 * @brief 设置电机控制目标（通用接口）
 * @param motor  电机对象
 * @param target 目标值（含义由当前工作模式决定）
 * @return RT_EOK 成功
 * @note  开环模式：target = 占空比（0~max_duty）
 *        速度模式：target = 速度（0.1 RPM）
 *        位置模式：target = 角度（0.1°）
 */
rt_err_t motor_set_target(motor_t *motor, int32_t target);

/**
 * @brief 设置目标位置（位置环模式）
 */
rt_err_t motor_set_position(motor_t *motor, int32_t position);

/**
 * @brief 设置目标速度（速度环模式）
 */
rt_err_t motor_set_speed(motor_t *motor, int32_t speed);

/**
 * @brief 周期更新（由后台调度器调用）
 */
void motor_update(motor_t *motor);

/**
 * @brief 驱动层更新（由霍尔中断或高频定时器调用）
 */
void motor_drv_update(motor_t *motor);

/* ============================================================
 * 状态查询
 * ============================================================ */
int32_t       motor_get_position(const motor_t *motor);
int32_t       motor_get_speed(const motor_t *motor);
uint8_t       motor_get_fault(const motor_t *motor);
motor_state_t motor_get_state(const motor_t *motor);

/**
 * @brief  采样母线电流（0.1A，单向无符号）
 *
 * @note   这是**弱符号接口**：
 *           motor.c   提供 __attribute__((weak)) 桩，恒返回 0
 *           bsp_adc_foc.c 提供强定义（真实 ADC+DMA 数据）
 *         弱桩存在的意义：未接入电流采样时链接不失败，
 *         但会静默返回 0 —— 所以接入电流采样是**必须**的，
 *         否则过流保护形同虚设。
 *
 * 【2026-11-10 补充声明】
 *   此前只在 bsp_adc_foc.c 里定义，没有任何头文件声明，
 *   其他模块调用时会触发 -Wimplicit-function-declaration 警告
 *   （ctrl_loop_init.c 就踩到了）。在此补充声明消除隐患。
 */
uint16_t motor_sample_current_a10(motor_t *motor);

/* 全局回转电机实例 */
extern motor_t g_swing_motor;

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */
