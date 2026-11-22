/**
 * @file    lift_sensor_config.c
 * @brief   举升传感器配置表实例
 * @note    用户在此文件中配置传感器类型和参数
 */

#include <string.h>
#include <rtthread.h>

#include "lift_sensor_config.h"
#include "framework/state_engine/sm_assert.h"
#include "../sys_param/sys_param.h"
//#include "../hw/hw_adc.h"
#include "lift_adc.h"

/* ============================================================
 * 适配器：把 hw_adc 接口适配为传感器需要的签名
 * ============================================================
 * 说明：
 *   - 通过 user_data 传递 hw_adc_t 指针
 *   - 这样 BSP 层无需知道"lift"、"swing"等业务概念
 * ============================================================ */
//static uint16_t lift_adc_read_mv(void *user_data)
//{
//    hw_adc_t *adc = (hw_adc_t*)user_data;
//    if (adc == NULL) return 0;
//    return hw_adc_read_mv(adc, HW_ADC_CH_VOLTAGE);
//}

//static uint16_t lift_adc_read_mv_adapter(void *user_data)
//{
//    return lift_adc_read_mv(user_data);      /* 直接转发 */
//}


/* ============================================================
 * 传感器配置表
 * ============================================================
 * 说明：
 *   - 通过 name 选择使用的配置
 *   - 默认使用第一个配置
 *   - 适配器零位从 sys_param 动态加载（见 lift_init）
 * ============================================================ */
const lift_sensor_config_t g_lift_sensor_configs[] = {
    /* ----- 配置1：电压型（默认） ----- */
    {
        .name = "lift_voltage",
        .type = LIFT_SENSOR_TYPE_VOLTAGE,
        .params.voltage = {
            .read_adc_mv   = lift_adc_read_mv,
//            .user_data     = &g_hw_adc,     /* 全局 ADC 实例 */
            .user_data     = NULL,
            .fault_high_mv = 4700,          /* 过压阈值 */
            .fault_low_mv  = 300,           /* 欠压阈值 */
        },
        .adapter = {
            .zero_raw      = 2203,          /* 默认零位电压 */
            .scale_num     = LIFT_ADAPTER_SCALE_NUM,
            .scale_den     = LIFT_ADAPTER_SCALE_DEN,
            .min_angle_x10 = -300,          /* -30° */
            .max_angle_x10 = 430,           /* +43° */
        },
        .update_period_ms = 50,
    },

//    /* ----- 配置2：电流型（4~20mA） ----- */
//    {
//        .name = "lift_current",
//        .type = LIFT_SENSOR_TYPE_CURRENT,
//        .params.current = {
//            .read_adc_mv         = default_adc_read_mv,
//            .user_data           = NULL,
//            .sample_resistor_ohm = 100,     /* 100Ω 采样电阻 */
//            .fault_high_ma10     = 220,     /* 22.0mA 过流 */
//            .fault_low_ma10      = 35,      /* 3.5mA 欠流 */
//        },
//        .adapter = {
//            /* 4~20mA 对应 -30°~+43°（跨度 73°）
//             * 零位 12.0mA = 120 (0.1mA)
//             * 跨度 8.0mA = 80 (0.1mA)
//             */
//            .zero_raw      = 120,
//            .scale_num     = 730,           /* 73° × 10 */
//            .scale_den     = 80,            /* 80 (0.1mA) */
//            .min_angle_x10 = -300,
//            .max_angle_x10 = 430,
//        },
//        .update_period_ms = 50,
//    },
//
//    /* ----- 配置3：角度型（预留） ----- */
//    {
//        .name = "lift_angle",
//        .type = LIFT_SENSOR_TYPE_ANGLE,
//        .params.angle = {
//            .read_angle_x10 = NULL,
//            .user_data      = NULL,
//            .fault_code     = 0xFFFF,
//        },
//        .adapter = {
//            .zero_raw      = 0,
//            .scale_num     = 1,
//            .scale_den     = 1,
//            .min_angle_x10 = -300,
//            .max_angle_x10 = 430,
//        },
//        .update_period_ms = 50,
//    },
};

const uint32_t g_lift_sensor_config_count =
    sizeof(g_lift_sensor_configs) / sizeof(g_lift_sensor_configs[0]);

/* ============================================================
 * 查找 API
 * ============================================================ */

/**
 * @brief 按名称查找配置
 */
const lift_sensor_config_t* lift_sensor_config_find(const char *name)
{
    if (name == NULL) return NULL;

    for (uint32_t i = 0; i < g_lift_sensor_config_count; i++) {
        if (strcmp(g_lift_sensor_configs[i].name, name) == 0) {
            return &g_lift_sensor_configs[i];
        }
    }
    return NULL;
}

/**
 * @brief 获取默认配置
 */
const lift_sensor_config_t* lift_sensor_config_default(void)
{
    if (g_lift_sensor_config_count == 0) return NULL;
    return &g_lift_sensor_configs[0];
}
