#ifndef __APP_REALTIME_H__
#define __APP_REALTIME_H__

#include <stdint.h>

/*=============================================================================
 * 实时数据模块初始化
 *============================================================================*/

/**
 * @brief  初始化实时数据模块
 * @note   在系统初始化时调用（Modbus_Init 之前或之后均可）
 */
void Realtime_Init(void);

/*=============================================================================
 * 实时数据读取接口（直接从设备获取最新值）
 *============================================================================*/

/**
 * @brief  获取实时转速
 * @return 转速值 (r/min)
 */
int16_t Realtime_GetSpeed(void);

/**
 * @brief  获取实时角度
 * @return 角度值 (0.1度)
 */
int16_t Realtime_GetAngle(void);

/**
 * @brief  获取实时电压
 * @return 电压值 (0.1V)
 */
uint16_t Realtime_GetVoltage(void);

/**
 * @brief  获取实时电流
 * @return 电流值 (1mA)
 */
uint16_t Realtime_GetCurrent(void);

/**
 * @brief  获取实时方向
 * @return 0=停止, 1=正转, 2=反转
 */
int16_t Realtime_GetDirection(void);

#endif /* __APP_REALTIME_H__ */
