/**
 * @file    motor.c
 * @brief   电机对象核心实现
 */

#include "../hw/hw_brake.h"
#include "motor.h"

/* 【2026-11-10 彩标 P0】电机初始化：失败=ERR，成功=OK */
#include "../log/log_color.h"
#include "motor_drv.h"
#include "motor_fb.h"
#include "motor_ctrl.h"
#include "motor_config.h"
#include "framework/state_engine/sm_assert.h"
#include <string.h>
#include <rtthread.h>

/* ============================================================
 * 全局回转电机实例
 * ============================================================ */
motor_t g_swing_motor;

/* ============================================================
 * 初始化
 * ============================================================ */
rt_err_t motor_init(motor_t *motor, const motor_config_t *cfg)
{
    SM_ASSERT_PTR(motor);
    SM_ASSERT_PTR(cfg);

    memset(motor, 0, sizeof(motor_t));

    /* 1. 绑定配置 */
    motor->name = cfg->name;
    motor->type = cfg->type;
    motor->pole_pairs = cfg->pole_pairs;
    motor->config = cfg;
    motor->update_period_ms = cfg->update_period_ms;

    /* 2. 绑定三层接口 */
    motor->drv_ops = cfg->drv_ops;
    motor->fb_ops = cfg->fb_ops;
    motor->ctrl_ops = cfg->ctrl_ops;

    /* 3. 初始化驱动层 */
    if (motor->drv_ops && motor->drv_ops->init) {
        rt_err_t ret = motor->drv_ops->init(motor);
        if (ret != RT_EOK) {
            LOG_ERR("[Motor] %s: drv init failed (%d)\n", motor->name, ret);
            return ret;
        }
    }

    /* 4. 初始化反馈层 */
    if (motor->fb_ops && motor->fb_ops->init) {
        rt_err_t ret = motor->fb_ops->init(motor);
        if (ret != RT_EOK) {
            LOG_ERR("[Motor] %s: fb init failed (%d)\n", motor->name, ret);
            return ret;
        }
    }

    /* 5. 初始化控制层 */
    if (motor->ctrl_ops && motor->ctrl_ops->init) {
        rt_err_t ret = motor->ctrl_ops->init(motor);
        if (ret != RT_EOK) {
            LOG_ERR("[Motor] %s: ctrl init failed (%d)\n", motor->name, ret);
            return ret;
        }
    }

    /* 6. 初始化软启动/停止 */
    soft_ramp_init(&motor->ramp, cfg->ramp_cfg);

    /* 7. 初始化过流保护 */
    if (cfg->oc_cfg) {
        motor_oc_init(&motor->oc_ctx, cfg->oc_cfg);
    }

    motor->state = MOTOR_STATE_IDLE;
    motor->last_update_tick = rt_tick_get();

    LOG_OK("[Motor] %s initialized.\n", motor->name);
    return RT_EOK;
}

/* ============================================================
 * 反初始化
 * ============================================================ */
rt_err_t motor_deinit(motor_t *motor)
{
    SM_ASSERT_PTR(motor);

    if (motor->ctrl_ops && motor->ctrl_ops->deinit) {
        motor->ctrl_ops->deinit(motor);
    }
    if (motor->fb_ops && motor->fb_ops->deinit) {
        motor->fb_ops->deinit(motor);
    }
    if (motor->drv_ops && motor->drv_ops->deinit) {
        motor->drv_ops->deinit(motor);
    }
    return RT_EOK;
}

/* ============================================================
 * 运行时控制
 * ============================================================ */
rt_err_t motor_start(motor_t *motor, uint8_t direction)
{
    SM_ASSERT_PTR(motor);

    motor->direction = direction;

    /* 开环模式下默认 50% 占空比 */
    int32_t target = 500;

    if (motor->ctrl_ops && motor->ctrl_ops->set_target) {
        motor->ctrl_ops->set_target(motor, target);
    }
    return RT_EOK;
}

rt_err_t motor_stop(motor_t *motor)
{
    SM_ASSERT_PTR(motor);

    if (motor->ctrl_ops && motor->ctrl_ops->stop) {
        motor->ctrl_ops->stop(motor);
    }
    return RT_EOK;
}

rt_err_t motor_brake(motor_t *motor)
{
    SM_ASSERT_PTR(motor);

    if (motor->drv_ops && motor->drv_ops->brake) {
        motor->drv_ops->brake(motor);
    }
    motor->state = MOTOR_STATE_BRAKING;
    return RT_EOK;
}

rt_err_t motor_set_target(motor_t *motor, int32_t target)
{
    SM_ASSERT_PTR(motor);

    if (motor->ctrl_ops && motor->ctrl_ops->set_target) {
        motor->ctrl_ops->set_target(motor, target);
        return RT_EOK;
    }
    return -RT_ERROR;
}

rt_err_t motor_set_position(motor_t *motor, int32_t position)
{
    SM_ASSERT_PTR(motor);
    motor->target_position = position;
    return motor_set_target(motor, position);
}

rt_err_t motor_set_speed(motor_t *motor, int32_t speed)
{
    SM_ASSERT_PTR(motor);
    motor->target_speed = speed;
    return motor_set_target(motor, speed);
}

/* ============================================================
 * 电流采样桩（弱符号，允许被更强定义覆盖）
 * ------------------------------------------------------------
 * 【2026-11-04 / R8】本处保留 weak 版本作为"无 ADC 环境"的兜底。
 * 真实实现放在 bsp_adc_foc.c：foc_adc_get_bus_current_a10()。
 *
 * 为什么不在本文件直接调 foc_adc：
 *   motor 是平台无关层，不应硬依赖具体 ADC 驱动；
 *   保持 weak 约定，让 BSP 侧提供实现，符合分层。
 *
 * 如果链接后发现电流恒为 0，先查：
 *   ① bsp_adc_foc.c 里是否有强定义的 motor_sample_current_a10
 *   ② tbl 里 foc_adc 步骤是否成功（看 [BOOT] 打印）
 *   ③ t_iadc_stat 的 conv_cnt 是否在增长
 * ============================================================ */
__attribute__((weak)) uint16_t motor_sample_current_a10(motor_t *motor)
{
    (void)motor;
    return 0;
}


/* ============================================================
 * 周期更新（由后台调度器调用）
 * ============================================================ */
void motor_update(motor_t *motor)
{
    SM_ASSERT_PTR(motor);

    uint32_t now = rt_tick_get();
    uint32_t elapsed_ms = (now - motor->last_update_tick) * 1000 / RT_TICK_PER_SECOND;

    if (elapsed_ms < motor->update_period_ms) return;
    motor->last_update_tick = now;

    /* 1. 采样电流（弱函数，用户实现）*/
    motor->current_current = motor_sample_current_a10(motor);

    /* 2. 更新反馈 */
    if (motor->fb_ops && motor->fb_ops->update) {
        motor->fb_ops->update(motor);
    }

    /* 3. 过流保护检查 */
    if (motor->config && motor->config->oc_cfg) {
        motor_oc_state_t oc = motor_oc_update(&motor->oc_ctx,
                                               (uint16_t)motor->current_current);
        if (oc != MOTOR_OC_NONE) {
            /* 根据客户要求，触发任意过流保护：紧急停机 */
            if (motor->drv_ops && motor->drv_ops->brake) {
                motor->drv_ops->brake(motor);
            }
            extern hw_brake_t g_swing_brake;
            hw_brake_emergency_lock(&g_swing_brake);
            motor->state = MOTOR_STATE_FAULT;
            return;
        }
    }

    /* 4. 控制环更新 */
    if (motor->ctrl_ops && motor->ctrl_ops->update) {
        motor->ctrl_ops->update(motor, elapsed_ms);
    }

    /* 5. 驱动 tick（强拖 / 状态机）*/
    if (motor->drv_ops && motor->drv_ops->tick) {
        motor->drv_ops->tick(motor, elapsed_ms);
    }
}

/* ============================================================
 * 驱动层更新（由霍尔中断调用）
 * ============================================================ */
void motor_drv_update(motor_t *motor)
{
    SM_ASSERT_PTR(motor);
    if (motor->drv_ops && motor->drv_ops->update) {
        motor->drv_ops->update(motor);
    }
}

/* ============================================================
 * 状态查询
 * ============================================================ */
int32_t motor_get_position(const motor_t *motor)
{
    SM_ASSERT_PTR(motor);
    return motor->current_position;
}

int32_t motor_get_speed(const motor_t *motor)
{
    SM_ASSERT_PTR(motor);
    return motor->current_speed;
}

uint8_t motor_get_fault(const motor_t *motor)
{
    SM_ASSERT_PTR(motor);
    return motor->fault;
}

motor_state_t motor_get_state(const motor_t *motor)
{
    SM_ASSERT_PTR(motor);
    return motor->state;
}
