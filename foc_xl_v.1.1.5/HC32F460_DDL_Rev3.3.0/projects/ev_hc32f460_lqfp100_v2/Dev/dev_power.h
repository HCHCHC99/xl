#ifndef DEV_POWER_H_
#define DEV_POWER_H_

#include "device_manager.h"
#include "Gpio_io.h"
#include "EventBus.h"
#include "dev_motor.h"
#include "rtt_manager.h"

// ========== 调试宏定义 ==========
// 开关在 rtt_manager.h 中统一管理：DEV_POWER

#ifdef DEV_POWER
    #define POWER_DEBUG(fmt, ...)    MAIN_D("[POWER_DEBUG] " fmt, ##__VA_ARGS__)
#else
    #define POWER_DEBUG(fmt, ...)    ((void)0)
#endif

// ========== 电源模拟模式宏定义 ==========
// 开启后，电源设备将不读取真实GPIO，而是使用模拟值
#define POWER_SIMULATION_MODE     // 电源模拟模式（单极性模拟开关）

// ========== 电源设备命令码 ==========
#define CMD_POWER_GET_STATE     (CMD_BASE_POWER + 0x01)
#define CMD_POWER_SET_DEBOUNCE  (CMD_BASE_POWER + 0x02)

// ========== 电源设备配置 ==========
typedef struct {
    uint8_t port;               // GPIO端口
    uint16_t pin;               // GPIO引脚
    uint8_t active_level;       // 有效电平
    uint8_t power_id;           // 电源ID
    uint16_t debounce_ms;       // 防抖时间(ms)
    uint8_t window_size;        // 采样窗口大小
    uint16_t sample_interval;   // 采样间隔(ms)
} Power_Config_t;

// ========== 电源设备结构体 ==========
typedef struct {
    uint8_t port;               // GPIO端口
    uint16_t pin;               // GPIO引脚
    uint8_t active_level;       // 有效电平
    uint8_t power_id;           // 电源ID
    uint16_t debounce_ms;       // 防抖时间
    uint8_t window_size;        // 采样窗口
    uint16_t sample_interval;   // 采样间隔

    uint8_t initialized;        // 初始化标志
    uint8_t current_state;      // 当前电源状态（0=无电，1=有电）
    uint8_t last_stable_state;  // 上次稳定状态
    uint32_t last_change_time;  // 上次状态变化时间
    uint32_t last_sample_time;  // 上次采样时间
    uint8_t sample_buffer[8];   // 采样缓冲区
    uint8_t sample_index;       // 采样索引
    uint8_t sample_count;       // 有效采样数
} Power_Device_t;

// ========== 标准设备操作 ==========
DeviceResult_t Power_Device_Init(void* handle);
DeviceResult_t Power_Device_Deinit(void* handle);
DeviceResult_t Power_Device_Read(void* handle, void* data, uint32_t size);
DeviceResult_t Power_Device_Write(void* handle, const void* data, uint32_t size);
DeviceResult_t Power_Device_Control(void* handle, DeviceCommandData_t* cmd);
DeviceResult_t Power_Device_Update(void* handle);

// ========== 电源特定接口 ==========
uint8_t Power_Device_GetState(Power_Device_t* dev);
Power_Device_t* Power_Device_Create(const Power_Config_t* config);

// ========== 模拟模式接口 ==========
#ifdef POWER_SIMULATION_MODE
void Power_SetSimulationState(uint8_t u8State);  // 设置模拟电源状态
#endif

// ========== 全局操作函数表 ==========
extern const DeviceOps_t g_power_ops;

#endif /* DEV_POWER_H_ */
