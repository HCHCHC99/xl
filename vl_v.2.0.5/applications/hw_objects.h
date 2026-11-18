/**
 * @file    hw_objects.h
 * @brief   硬件对象声明
 * @note    电机对象已迁移至 motor 模块，本文件仅定义传感器和电磁阀
 */

#ifndef __HW_OBJECTS_H
#define __HW_OBJECTS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

///* ============================================================
// * 举升传感器对象
// * ============================================================ */
//typedef struct {
//    uint16_t zero_voltage_mv;
//    uint16_t current_voltage_mv;
//    int32_t  angle_current_x10;
//    uint8_t  fault_high_voltage;
//    uint8_t  fault_low_voltage;
//    uint32_t fault_start_tick;
//} lift_sensor_t;

/* ============================================================
 * 电磁阀对象
 * ============================================================ */
typedef struct {
    uint8_t valve_output;       /* Bit0: 上升阀, Bit1: 下降阀 */
    uint8_t valve_feedback;     /* 反馈开关状态：Bit0 上升打开, Bit1 下降打开 */

    /* 故障码（来自 0x18FF0E63 Byte1，供 0x18FF11D1 Byte2 上报）
     * 0 = 正常；1 = 开路；2 = 短路；3 = 预留 */
    uint8_t up_fault;
    uint8_t down_fault;
} solenoid_box_t;

/* ============================================================
 * 全局硬件对象
 * ============================================================ */
//extern lift_sensor_t g_sensor;
extern solenoid_box_t g_solenoid;

#ifdef __cplusplus
}
#endif

#endif /* __HW_OBJECTS_H */
