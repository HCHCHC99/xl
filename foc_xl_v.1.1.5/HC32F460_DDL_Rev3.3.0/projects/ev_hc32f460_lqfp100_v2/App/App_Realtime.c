#include "App_Realtime.h"
#include "App_Motor_Project.h"  // 包含 ID_RTURN, ID_VOLTAGE_BUS, ID_SENSOR_CURRENT
#include "dev_rturn.h"
#include "dev_voltage.h"
#include "dev_sensor.h"
#include "device_manager.h"
#include "rtt_log.h"

/*=============================================================================
 * 调试宏定义
 *============================================================================*/
#ifdef APP_REALTIME_DBG
    #define REALTIME_DBG(fmt, ...)    MAIN_D("[REALTIME] " fmt, ##__VA_ARGS__)
#else
    #define REALTIME_DBG(fmt, ...)    ((void)0)
#endif

/*=============================================================================
 * @brief  初始化实时数据模块
 *============================================================================*/
void Realtime_Init(void)
{
    REALTIME_DBG("Realtime module initialized");
}

/*=============================================================================
 * @brief  获取实时转速 (r/min)
 *============================================================================*/
int16_t Realtime_GetSpeed(void)
{
    float fSpeedDegS = 0.0f;
    DeviceCommandData_t stcCmd;
    memset(&stcCmd, 0, sizeof(DeviceCommandData_t));
    stcCmd.cmd = CMD_RTURN_GET_SPEED;
    stcCmd.params = NULL;
    stcCmd.param_size = 0;
    stcCmd.response = &fSpeedDegS;
    stcCmd.response_size = sizeof(float);

    DeviceResult_t res = Device_Control(ID_RTURN, &stcCmd);
    if (res == RESULT_OK)
    {
        /* deg/s -> r/min: r/min = deg/s / 360 * 60 = deg/s / 6 */
        return (int16_t)(fSpeedDegS / 6.0f);
    }

    REALTIME_DBG("GetSpeed failed, res=%d", (int)res);
    return 0;
}

/*=============================================================================
 * @brief  获取实时角度 (0.1度)
 *============================================================================*/
int16_t Realtime_GetAngle(void)
{
    float fAngleDeg = 0.0f;
    DeviceCommandData_t stcCmd;
    memset(&stcCmd, 0, sizeof(DeviceCommandData_t));
    stcCmd.cmd = CMD_RTURN_GET_ANGLE_DEG;
    stcCmd.params = NULL;
    stcCmd.param_size = 0;
    stcCmd.response = &fAngleDeg;
    stcCmd.response_size = sizeof(float);

    DeviceResult_t res = Device_Control(ID_RTURN, &stcCmd);
    if (res == RESULT_OK)
    {
        /* deg -> 0.1deg */
        return (int16_t)(fAngleDeg * 10.0f);
    }

    REALTIME_DBG("GetAngle failed, res=%d", (int)res);
    return 0;
}

/*=============================================================================
 * @brief  获取实时方向 (0=停止, 1=正转, 2=反转)
 *============================================================================*/
int16_t Realtime_GetDirection(void)
{
    uint8_t u8Dir = 0;
    DeviceCommandData_t stcCmd;
    memset(&stcCmd, 0, sizeof(DeviceCommandData_t));
    stcCmd.cmd = CMD_RTURN_GET_DIRECTION;
    stcCmd.params = NULL;
    stcCmd.param_size = 0;
    stcCmd.response = &u8Dir;
    stcCmd.response_size = sizeof(uint8_t);

    DeviceResult_t res = Device_Control(ID_RTURN, &stcCmd);
    if (res == RESULT_OK)
    {
        return (int16_t)u8Dir;
    }

    REALTIME_DBG("GetDirection failed, res=%d", (int)res);
    return 0;
}

/*=============================================================================
 * @brief  获取实时电压 (0.1V)
 *============================================================================*/
uint16_t Realtime_GetVoltage(void)
{
    uint32_t u32BusVoltageMv = 0;
    DeviceCommandData_t stcCmd;
    memset(&stcCmd, 0, sizeof(DeviceCommandData_t));
    stcCmd.cmd = CMD_VOLTAGE_GET_BUS_MV;
    stcCmd.params = NULL;
    stcCmd.param_size = 0;
    stcCmd.response = &u32BusVoltageMv;
    stcCmd.response_size = sizeof(uint32_t);

    DeviceResult_t res = Device_Control(ID_VOLTAGE_BUS, &stcCmd);
    if (res == RESULT_OK)
    {
        /* mV -> 0.1V: 0.1V = mV / 100 */
        return (uint16_t)(u32BusVoltageMv / 100UL);
    }

    REALTIME_DBG("GetVoltage failed, res=%d", (int)res);
    return 0;
}

/*=============================================================================
 * @brief  获取实时电流 (1mA)
 *============================================================================*/
uint16_t Realtime_GetCurrent(void)
{
    int32_t s32CurrentMa = 0;
    DeviceCommandData_t stcCmd;
    memset(&stcCmd, 0, sizeof(DeviceCommandData_t));
    stcCmd.cmd = CMD_SENSOR_GET_CURRENT_MA;
    stcCmd.params = NULL;
    stcCmd.param_size = 0;
    stcCmd.response = &s32CurrentMa;
    stcCmd.response_size = sizeof(int32_t);

    DeviceResult_t res = Device_Control(ID_SENSOR_CURRENT, &stcCmd);
    if (res == RESULT_OK)
    {
        /* 取绝对值（Modbus 为 uint16_t） */
        if (s32CurrentMa < 0)
        {
            return (uint16_t)(-s32CurrentMa);
        }
        return (uint16_t)s32CurrentMa;
    }

    REALTIME_DBG("GetCurrent failed, res=%d", (int)res);
    return 0;
}
