/**
 * @file    app_hw.c
 * @brief   硬件对象实例化 + 初始化入口
 * @note    统一管理所有 hw 层对象，风格对齐 motor_init
 */

#include "hw/hw_pwm.h"
#include "hw/hw_brake.h"
#include "hw/hw_hall.h"
#include "bsp/bsp_pwm.h"
#include "bsp/bsp_brake.h"
#include "bsp/bsp_hall.h"
#include "framework/state_engine/sm_assert.h"
#include <rtthread.h>

/* ============================================================
 * 全局对象实例（应用层持有）
 * ============================================================
 * 声明归口：
 *   g_swing_pwm   → bsp_pwm.h
 *   g_swing_brake → bsp_brake.h
 *   g_swing_hall  → bsp_hall.h
 *   g_brake_provider_timer → hw_brake.h
 * 本文件只负责「定义」，不再手写 extern，避免与头文件声明不一致。
 * ============================================================ */
hw_pwm_t   g_swing_pwm;
hw_brake_t g_swing_brake;
hw_hall_t  g_swing_hall;

/* ============================================================
 * 初始化入口
 * ============================================================ */
int app_hw_init(void)
{
    rt_err_t ret;

    /* 1. 回转轴 PWM */
    ret = hw_pwm_init(&g_swing_pwm,
                      &g_swing_pwm_cfg,
                      &g_bsp_pwm_ops);
    if (ret != RT_EOK) {
        rt_kprintf("[App-HW] swing pwm init failed (%d)\n", ret);
        return -1;
    }

    /* 2. 回转轴刹车盘 */
    ret = hw_brake_init(&g_swing_brake,
                        &g_swing_brake_cfg,
                        &g_bsp_brake_ops,
                        &g_brake_provider_timer);
    if (ret != RT_EOK) {
        rt_kprintf("[App-HW] swing brake init failed (%d)\n", ret);
        return -1;
    }

    /* 3. 回转轴 Hall */
    ret = hw_hall_init(&g_swing_hall,
                       &g_swing_hall_cfg,
                       &g_bsp_hall_ops);
    if (ret != RT_EOK) {
        rt_kprintf("[App-HW] swing hall init failed (%d)\n", ret);
        return -1;
    }

    rt_kprintf("[App-HW] All hardware objects initialized.\n\n");
    return 0;
}
//INIT_PREV_EXPORT(app_hw_init);


