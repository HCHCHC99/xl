/**
 * @file    motor_init.c
 * @brief   电机模块初始化
 */

#include "motor.h"
#include "motor_config.h"
#include "framework/state_engine/sm_assert.h"
#include <rtthread.h>

/* ============================================================
 * 初始化所有电机
 * ============================================================ */
int motor_module_init(void)
{
    rt_kprintf("[motor] motor_module_init()\n");
    rt_err_t ret;

    /* 1. 初始化回转电机 */
    const motor_config_t *cfg = motor_config_find("swing_motor");
    SM_ASSERT_PTR(cfg);

    ret = motor_init(&g_swing_motor, cfg);
    if (ret != RT_EOK) {
        rt_kprintf("[Motor] swing_motor init failed (%d)\n", ret);
        return -1;
    }

    rt_kprintf("[Motor] Module initialized.\n");
    rt_kprintf("[Motor] Try: motor_start 500 / motor_stop / motor_status\n\n");
    return 0;
}
//INIT_PREV_EXPORT(motor_module_init);
