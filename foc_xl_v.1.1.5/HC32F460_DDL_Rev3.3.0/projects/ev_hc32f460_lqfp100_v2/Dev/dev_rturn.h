#ifndef DEV_RTURN_H_
#define DEV_RTURN_H_

#include "device_manager.h"
#include "EventBus.h"
#include "dev_motor_hall.h"
#include <stdint.h>
#include <stdbool.h>
#include "rtt_manager.h"

// ========== 调试宏定义 ==========
// 开关在 rtt_manager.h 中统一管理：DEV_RTURN

#ifdef DEV_RTURN
    #define RTURN_DEBUG(fmt, ...)    MAIN_D("[RTURN_DEBUG] " fmt, ##__VA_ARGS__)
    #define RTURN_OUT(fmt, ...)      MAIN_D("[RTURN_OUT] " fmt, ##__VA_ARGS__)
#else
    #define RTURN_DEBUG(fmt, ...)    ((void)0)
    #define RTURN_OUT(fmt, ...)      ((void)0)
#endif

// 前向声明（避免循环依赖）
struct Current_AlarmEvent_t;

// ========== 圆弧转动机构设备命令码 ==========
#define CMD_RTURN_GET_ANGLE         (CMD_BASE_RTURN + 0x01)     // 获取当前角度(度x100)
#define CMD_RTURN_GET_ANGLE_DEG     (CMD_BASE_RTURN + 0x02)     // 获取当前角度(度)
#define CMD_RTURN_GET_SPEED         (CMD_BASE_RTURN + 0x03)     // 获取当前转速(度/秒)
#define CMD_RTURN_GET_DIRECTION     (CMD_BASE_RTURN + 0x04)     // 获取当前运动方向
#define CMD_RTURN_GET_LOCKED_DIR    (CMD_BASE_RTURN + 0x05)     // 获取锁死方向
#define CMD_RTURN_RESET_POSITION    (CMD_BASE_RTURN + 0x06)     // 重置当前位置为零位
#define CMD_RTURN_CLEAR_LOCK        (CMD_BASE_RTURN + 0x07)     // 清除锁死方向
#define CMD_RTURN_GET_CALIBRATED    (CMD_BASE_RTURN + 0x08)     // 获取找零有效位

// ========== 圆弧转动机构限位事件结构体 ==========
typedef struct {
    uint8_t  u8Direction;           // 触发的方向：1=正转限位, 2=反转限位
    float    fAngle;                // 触发时的角度(度)
    uint8_t  u8IsActive;            // 1=限位触发, 0=限位解除
} RTurn_LimitEvent_t;

// 限位方向常量
#define RTURN_LIMIT_FORWARD    1    // 正转限位（上限位）
#define RTURN_LIMIT_REVERSE    2    // 反转限位（下限位）

// 圆弧机构方向常量
#define RTURN_DIR_STOP      0
#define RTURN_DIR_FORWARD   1
#define RTURN_DIR_REVERSE   2

// ========== 圆弧转动机构配置 ==========
typedef struct {
    // 依赖的设备ID
    uint8_t u8MotorHallDevId;       // 电机霍尔设备ID（获取实际转速和方向）
    uint8_t u8MotorArbiterDevId;    // 电机仲裁设备ID（获取期望方向）
    uint8_t u8SensorDevId;          // 电流传感器设备ID

    // 机械参数
    float   fReductionRatio;        // 减速比（电机转N圈，输出轴转1圈）
    float   fMaxAngle;              // 最大角度(度)
    float   fMinAngle;              // 最小角度(度)
    uint8_t u8ReverseOutput;        // 输出方向是否反向（1=反向, 0=同向）

    // 设备配置
    uint8_t u8DeviceId;             // 设备ID
    uint16_t u16UpdateIntervalMs;   // 更新间隔(ms)
} RTurn_Config_t;

// ========== 锁死方向状态 ==========
typedef struct {
    uint8_t u8LockedDir;            // 锁死的方向：0=无锁死, 1=锁死正转, 2=锁死反转
    uint8_t u8LockActive;           // 锁死是否生效
} RTurn_LockState_t;

// ========== 圆弧转动机构设备结构体 ==========
typedef struct {
    RTurn_Config_t      stcConfig;      // 设备配置
    uint8_t             u8Initialized;  // 初始化标志

    // 状态缓存
    float               fCurrentAngle;      // 当前角度(度)
    float               fCurrentSpeed;      // 当前转速(度/秒)
    uint8_t             u8CurrentDir;       // 当前运动方向
    uint8_t             u8Calibrated;       // 找零有效位(0=未校准,1=已校准)

    // 锁死方向状态
    RTurn_LockState_t   stcLockState;

    // 限位触发标志
    uint8_t             u8LimitTriggered;       // 限位是否已触发

    // 方向缓存（方案4：用于主动检测过流时，如果期望方向已变为STOP，使用缓存方向）
    uint8_t             u8LastDesiredDir;       // 上一次有效的期望方向（RTURN_DIR_xxx）

    // 角度积分
    float               fLastSpeed;
    uint32_t            u32LastAngleTime;

    // 时间管理
    uint32_t            u32LastUpdateTime;
} RTurn_Device_t;

// ========== 读取响应结构体 ==========
typedef struct {
    float   fAngle;
    float   fSpeed;
    uint8_t u8Direction;
    uint8_t u8LockedDir;
    uint8_t u8Calibrated;
} RTurn_ReadResponse_t;

// ========== 标准设备操作 ==========
DeviceResult_t RTurn_Device_Init(void* handle);
DeviceResult_t RTurn_Device_Deinit(void* handle);
DeviceResult_t RTurn_Device_Read(void* handle, void* data, uint32_t size);
DeviceResult_t RTurn_Device_Write(void* handle, const void* data, uint32_t size);
DeviceResult_t RTurn_Device_Control(void* handle, DeviceCommandData_t* cmd);
DeviceResult_t RTurn_Device_Update(void* handle);

// ========== 圆弧转动机构特定接口 ==========
float RTurn_Device_GetAngle(RTurn_Device_t* pstcDev);
float RTurn_Device_GetAngleDeg(RTurn_Device_t* pstcDev);
float RTurn_Device_GetSpeed(RTurn_Device_t* pstcDev);
uint8_t RTurn_Device_GetDirection(RTurn_Device_t* pstcDev);
uint8_t RTurn_Device_GetLockedDir(RTurn_Device_t* pstcDev);
void RTurn_Device_ResetPosition(RTurn_Device_t* pstcDev);
void RTurn_Device_ClearLock(RTurn_Device_t* pstcDev);
uint8_t RTurn_Device_IsCalibrated(RTurn_Device_t* pstcDev);
RTurn_Device_t* RTurn_Device_Create(const RTurn_Config_t* pstcConfig);

// ========== EventBus回调函数声明 ==========
void RTurn_OnCurrentAlarm(void* payload);

// ========== 全局操作函数表 ==========
extern const DeviceOps_t g_rturn_ops;

// ========== Keil Watch调试全局变量 ==========
// 这些变量在dev_rturn.c中定义，每次RTurn_Device_Update时刷新
// 方便在Keil Watch窗口中直接观察
// volatile防止编译器优化，确保Watch窗口能实时读取到变化
extern volatile float    g_fDbgRTurnAngle;       // 当前角度(度)
extern volatile float    g_fDbgRTurnSpeed;       // 当前转速(度/秒)
extern volatile uint8_t  g_u8DbgRTurnDir;        // 当前实际转向(0=停止,1=正转,2=反转)
extern volatile uint8_t  g_u8DbgRTurnDesiredDir; // 仲裁器期望方向(0=停止,1=正转,2=反转)
extern volatile uint8_t  g_u8DbgRTurnLockedDir;  // 锁死方向(0=无锁死,1=锁死正转,2=锁死反转)
extern volatile uint8_t  g_u8DbgRTurnLockActive; // 锁死是否生效(0=未锁死,1=已锁死)
extern volatile uint8_t  g_u8DbgRTurnCalibrated; // 找零有效位(0=未校准,1=已校准)
extern volatile uint8_t  g_u8DbgRTurnLimitTrig;  // 限位触发标志(0=未触发,1=已触发)

#endif /* DEV_RTURN_H_ */
