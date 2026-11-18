/**
 * @file    lift_init.c
 * @brief   举升控制初始化
 */

#include "lift_control.h"
#include "lift_angle.h"
#include "lift_sensor.h"
#include "lift_sensor_config.h"
#include "lift_adapter.h"
#include "lift_events.h"

/* 【2026-11-10 彩标 P2】举升初始化：失败=ERR，完成=OK */
#include "../log/log_color.h"
#include "lift_adc.h"
#include "valve_tx.h"
#include "../sys_param/sys_param.h"
#include "framework/state_engine/sm_assert.h"
#include "../framework/manager/ctrl_manager.h"
#include <rtthread.h>


/* ============================================================
 * 配置选择
 * ============================================================
 * 说明：
 *   - NULL = 使用默认配置（数组第一个）
 *   - 可填 "lift_voltage" / "lift_current" / "lift_angle"
 * ============================================================ */
#define LIFT_SENSOR_CONFIG_NAME    ("lift_voltage")


/* ============================================================
 * 全局传感器实例
 * ============================================================ */
static lift_sensor_t g_lift_sensor;

/* ============================================================
 * 初始化
 * ============================================================ */
int lift_init(void)
{
    rt_kprintf("[lift] lift_init()\n");
    rt_err_t ret;
    sys_param_t *param = sys_param_get();
    const lift_sensor_config_t *config;
    lift_adapter_cfg_t adapter;

    /* 0. ADC 采集（必须先于传感器使用） */
    if (lift_adc_init() != 0) {
        rt_kprintf("[Lift] adc init failed\n");
        return -1;
    }

    /* 1. 查找配置 */
    if (LIFT_SENSOR_CONFIG_NAME == NULL) {
        config = lift_sensor_config_default();
    } else {
        config = lift_sensor_config_find(LIFT_SENSOR_CONFIG_NAME);
    }
    SM_ASSERT_PTR(config);

    rt_kprintf("[Lift] Using config: %s\n", config->name);

    /* 2. 初始化传感器 */
    ret = lift_sensor_init_from_config(&g_lift_sensor, config);
    if (ret != RT_EOK) {
        rt_kprintf("[Lift] Sensor init failed (%d)\n", ret);
        return -1;
    }

    /* 3. 初始化举升角对象 */
    lift_angle_init(&g_lift_angle, &g_lift_sensor);
    lift_angle_init_events(&g_lift_angle);

    /* 4. 配置适配器（从配置表 + sys_param 动态覆盖） */
    adapter = config->adapter;

    /* 电压型零位从 sys_param 加载 */
    if (config->type == LIFT_SENSOR_TYPE_VOLTAGE) {
        if (param->is_calibrated) {
            adapter.zero_raw = param->lift_zero_voltage_mv;   /* 用已保存的校准值 */
            rt_kprintf("[Lift] use calibrated zero=%umV\n", adapter.zero_raw);
        } else {
            adapter.zero_raw = LIFT_ADC_DEFAULT_ZERO_MV;      /* 2203，默认校准点 */
            rt_kprintf("[Lift] NOT calibrated, use default zero=%umV\n",
                       (unsigned)LIFT_ADC_DEFAULT_ZERO_MV);
        }
    }
    lift_angle_set_adapter(&g_lift_angle, &adapter);
    g_lift_angle.is_calibrated = param->is_calibrated;

    rt_kprintf("[Lift] Adapter: zero=%d, scale=%d/%d\n",
               adapter.zero_raw, adapter.scale_num, adapter.scale_den);

    /* 5. 注册举升控制对象 */
    extern ctrl_manager_t g_ctrl_mgr;
    ret = lift_control_register(&g_ctrl_mgr);
    if (ret != 0) {
        LOG_ERR("[Lift] Register failed\n");
        return -1;
    }
    if (valve_tx_init() != 0) {
        rt_kprintf("[Lift] valve_tx register failed\n");
        return -1;
    }

    LOG_OK("[Lift] Initialized (sensor=%s)\n", g_lift_sensor.name);
    return 0;
}
//INIT_DEVICE_EXPORT(lift_init);
