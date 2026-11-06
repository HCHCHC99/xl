#include "dev_power.h"
#include "TickTimer.h"
#include "rtt_log.h"
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

#ifdef POWER_SIMULATION_MODE
    static uint8_t s_u8SimPowerState = 1;  // 默认有电
    static uint8_t s_u8LastSimPowerState = 1;
#endif

// 统一 GPIO 读取函数
static uint8_t Power_ReadPinState(Power_Device_t* dev) {
    en_pin_state_t pin_state = GPIO_READ(dev->port, dev->pin);
    uint8_t state = (pin_state == PIN_SET) ? 1 : 0;
    return state;
}

// 软件滤波函数
static uint8_t Power_SoftwareFilter(Power_Device_t* dev, uint8_t new_sample) {
    uint8_t window_size = dev->window_size;
    if (window_size > sizeof(dev->sample_buffer)) {
        window_size = sizeof(dev->sample_buffer);
    }
    if (window_size < 1) {
        return new_sample;
    }

    dev->sample_buffer[dev->sample_index] = new_sample;
    dev->sample_index = (dev->sample_index + 1) % window_size;
    if (dev->sample_count < window_size) {
        dev->sample_count++;
    }

    uint8_t high_count = 0;
    for (uint8_t i = 0; i < dev->sample_count; i++) {
        if (dev->sample_buffer[i]) {
            high_count++;
        }
    }

    return (high_count > (dev->sample_count / 2)) ? 1 : 0;
}

// 发布事件 - 添加返回值确认
static void Power_PublishEvent(Power_Device_t* dev, uint8_t new_state) {
    MotorPowerEvent_t powerEvent = {
        .power_id = dev->power_id,
        .is_on = (new_state == 1)
    };

    MAIN_D("[POWER] Publishing: ID=%d, is_on=%d\r\n", dev->power_id, new_state);  // 添加这行

    EventBus_Publish(TOPIC_POWER, &powerEvent);
}

// ========== 标准设备操作实现 ==========

DeviceResult_t Power_Device_Init(void* handle) {
    Power_Device_t* dev = (Power_Device_t*)handle;
    if (!dev) return RESULT_PARAM_ERR;

    MAIN_D("[POWER] === Init called: ID=%d ===\r\n", dev->power_id);  // 添加这行

#ifdef POWER_SIMULATION_MODE
    // ========== 模拟模式：不读取GPIO，直接使用模拟值 ==========
    MAIN_D("[POWER] SIM mode: ID=%d, SimState=%d\r\n", dev->power_id, s_u8SimPowerState);

    dev->initialized = 1;
    dev->current_state = s_u8SimPowerState;
    dev->last_stable_state = s_u8SimPowerState;
    dev->last_change_time = tickTimer_GetCount();
    dev->last_sample_time = tickTimer_GetCount();
    dev->sample_index = 0;
    dev->sample_count = 0;
    memset(dev->sample_buffer, 0, sizeof(dev->sample_buffer));

    MAIN_D("[POWER] Init success (SIM): ID=%d, state=%d\r\n", dev->power_id, dev->current_state);

    // 发布初始状态
    Power_PublishEvent(dev, dev->current_state);

    return RESULT_OK;
#else
    // ========== 真实模式：配置GPIO并读取 ==========
    Input_GPIO_Init(dev->port, dev->pin, ENABLE);

    dev->initialized = 1;
    dev->current_state = 0;
    dev->last_stable_state = 0;
    dev->last_change_time = tickTimer_GetCount();
    dev->last_sample_time = tickTimer_GetCount();
    dev->sample_index = 0;
    dev->sample_count = 0;
    memset(dev->sample_buffer, 0, sizeof(dev->sample_buffer));

    uint8_t raw_state = Power_ReadPinState(dev);
    uint8_t init_state = (raw_state == dev->active_level) ? 1 : 0;
    dev->current_state = init_state;
    dev->last_stable_state = init_state;

    MAIN_D("[POWER] Real mode init: ID=%d, state=%d\r\n", dev->power_id, init_state);

    Power_PublishEvent(dev, init_state);

    return RESULT_OK;
#endif
}

DeviceResult_t Power_Device_Deinit(void* handle) {
    Power_Device_t* dev = (Power_Device_t*)handle;
    if (!dev) return RESULT_PARAM_ERR;
    if (!dev->initialized) return RESULT_OK;

    MAIN_D("[POWER] Deinit: ID=%d\r\n", dev->power_id);
    dev->initialized = 0;
    return RESULT_OK;
}

DeviceResult_t Power_Device_Read(void* handle, void* data, uint32_t size) {
    Power_Device_t* dev = (Power_Device_t*)handle;
    if (!dev || !data) return RESULT_PARAM_ERR;
    if (!dev->initialized) return RESULT_ERROR;

    if (size == sizeof(uint8_t)) {
        *(uint8_t*)data = dev->current_state;
        return RESULT_OK;
    }
    return RESULT_PARAM_ERR;
}

DeviceResult_t Power_Device_Write(void* handle, const void* data, uint32_t size) {
    (void)handle;
    (void)data;
    (void)size;
    return RESULT_ERROR;
}

DeviceResult_t Power_Device_Control(void* handle, DeviceCommandData_t* cmd) {
    Power_Device_t* dev = (Power_Device_t*)handle;
    if (!dev || !cmd) return RESULT_PARAM_ERR;
    if (!dev->initialized) return RESULT_ERROR;

    switch(cmd->cmd) {
        case CMD_POWER_GET_STATE:
            if (cmd->response && cmd->response_size >= sizeof(uint8_t)) {
                *(uint8_t*)cmd->response = dev->current_state;
                return RESULT_OK;
            }
            return RESULT_PARAM_ERR;

        case CMD_POWER_SET_DEBOUNCE:
            if (cmd->params && cmd->param_size == sizeof(uint16_t)) {
                dev->debounce_ms = *(uint16_t*)cmd->params;
                return RESULT_OK;
            }
            return RESULT_PARAM_ERR;

        default:
            return RESULT_ERROR;
    }
}

DeviceResult_t Power_Device_Update(void* handle) {
    Power_Device_t* dev = (Power_Device_t*)handle;
    if (!dev || !dev->initialized) return RESULT_ERROR;

#ifdef POWER_SIMULATION_MODE
    // ========== 模拟模式：使用模拟值 ==========
    if (s_u8SimPowerState != s_u8LastSimPowerState) {
        s_u8LastSimPowerState = s_u8SimPowerState;
        dev->current_state = s_u8SimPowerState;
        dev->last_stable_state = s_u8SimPowerState;
        MAIN_D("[POWER] SIM state changed: ID=%d, NewState=%d\r\n", dev->power_id, dev->current_state);
        Power_PublishEvent(dev, dev->current_state);
    }
    return RESULT_OK;
#else
    // ========== 真实模式：读取GPIO并处理 ==========
    uint32_t now = tickTimer_GetCount();

    if (now - dev->last_sample_time < dev->sample_interval) {
        return RESULT_OK;
    }
    dev->last_sample_time = now;

    uint8_t raw_state = Power_ReadPinState(dev);
    uint8_t active_state = (raw_state == dev->active_level) ? 1 : 0;
    uint8_t filtered_state = Power_SoftwareFilter(dev, active_state);

    if (filtered_state != dev->last_stable_state) {
        if (now - dev->last_change_time >= dev->debounce_ms) {
            dev->last_stable_state = filtered_state;
            dev->current_state = filtered_state;
            dev->last_change_time = now;
            MAIN_D("[POWER] State changed: ID=%d, New=%d\r\n", dev->power_id, filtered_state);
            Power_PublishEvent(dev, filtered_state);
        }
    }

    return RESULT_OK;
#endif
}

// ========== 电源特定接口 ==========

uint8_t Power_Device_GetState(Power_Device_t* dev) {
    if (!dev || !dev->initialized) return 0;
    return dev->current_state;
}

Power_Device_t* Power_Device_Create(const Power_Config_t* config) {
    if (!config) return NULL;

    Power_Device_t* dev = (Power_Device_t*)malloc(sizeof(Power_Device_t));
    if (!dev) return NULL;

    memset(dev, 0, sizeof(Power_Device_t));
    dev->port = config->port;
    dev->pin = config->pin;
    dev->active_level = config->active_level;
    dev->power_id = config->power_id;
    dev->debounce_ms = config->debounce_ms;
    dev->window_size = config->window_size;
    dev->sample_interval = config->sample_interval;

    MAIN_D("[POWER] Created: ID=%d, Port=%d, Pin=%d\r\n",
           dev->power_id, dev->port, dev->pin);

    return dev;
}

#ifdef POWER_SIMULATION_MODE
void Power_SetSimulationState(uint8_t u8State) {
    s_u8SimPowerState = (u8State != 0) ? 1 : 0;
    MAIN_D("[POWER] SIM state set to %d\r\n", s_u8SimPowerState);
}
#endif

const DeviceOps_t g_power_ops = {
    .init = Power_Device_Init,
    .deinit = Power_Device_Deinit,
    .read = Power_Device_Read,
    .write = Power_Device_Write,
    .control = Power_Device_Control,
    .update = Power_Device_Update
};
