#include "dev_pwm.h"
#include "TickTimer.h"
#include <string.h>

// ========== 内部辅助函数 ==========
static pwm_error_t PWM_Device_ConvertError(DeviceResult_t* result, pwm_error_t pwm_err);

static pwm_error_t PWM_Device_ConvertError(DeviceResult_t* result, pwm_error_t pwm_err) {
    if (result == NULL) return pwm_err;

    switch(pwm_err) {
        case PWM_OK:
            *result = RESULT_OK;
            break;
        case PWM_ERR_NULL_PTR:
        case PWM_ERR_INVALID_PARAM:
            *result = RESULT_PARAM_ERR;
            break;
        case PWM_ERR_CLK_FAIL:
            *result = RESULT_ERROR;
            break;
        case PWM_ERR_NOT_INIT:
            *result = RESULT_ERROR;
            break;
        case PWM_ERR_RAMP_BUSY:
            *result = RESULT_BUSY;
            break;
        default:
            *result = RESULT_ERROR;
            break;
    }

    return pwm_err;
}

// ========== 标准设备操作接口实现 ==========

DeviceResult_t PWM_Device_Init(void* handle) {
    PWM_Device_t* dev = (PWM_Device_t*)handle;
    if (dev == NULL) return RESULT_PARAM_ERR;

    // 初始化PWM抽象层实例
    dev->pwm_instance = PWM_Init(
        dev->config.tmr,
        dev->config.periphClk,
        dev->config.channel,
        dev->config.port,
        dev->config.pin,
        dev->config.pinFunc,
        dev->config.countMode,
        dev->config.countDir,
        dev->config.defaultFreqHz,
        dev->config.defaultDutyPercent,
        dev->config.activePolarity
    );

    // 检查初始化是否成功
    if (dev->pwm_instance.tmr == NULL) {
        return RESULT_ERROR;
    }

    dev->initialized = 1;
    dev->output_enabled = 1;
    dev->last_update_time = tickTimer_GetCount();

    // 启动PWM定时器
    pwm_error_t err = PWM_Start(&dev->pwm_instance);
    if (err != PWM_OK) {
        dev->initialized = 0;
        return RESULT_ERROR;
    }

    // 使能输出
    err = PWM_OutputCmd(&dev->pwm_instance, PWM_OUTPUT_ENABLE);
    if (err != PWM_OK) {
        dev->initialized = 0;
        return RESULT_ERROR;
    }

    return RESULT_OK;
}

DeviceResult_t PWM_Device_Deinit(void* handle) {
    PWM_Device_t* dev = (PWM_Device_t*)handle;
    if (dev == NULL) return RESULT_PARAM_ERR;

    if (!dev->initialized) return RESULT_OK;

    // 停止PWM
    PWM_Stop(&dev->pwm_instance);

    // 禁用输出
    PWM_OutputCmd(&dev->pwm_instance, PWM_OUTPUT_DISABLE);

    dev->initialized = 0;
    dev->output_enabled = 0;

    return RESULT_OK;
}

DeviceResult_t PWM_Device_Read(void* handle, void* data, uint32_t size) {
    PWM_Device_t* dev = (PWM_Device_t*)handle;
    if (dev == NULL || data == NULL) return RESULT_PARAM_ERR;

    if (!dev->initialized) return RESULT_ERROR;

    // 返回PWM状态信息
    if (size == sizeof(uint16_t)) {
        // 读取当前占空比
        *(uint16_t*)data = PWM_GetDuty(&dev->pwm_instance);
    } else if (size == sizeof(uint32_t)) {
        // 读取当前频率
        *(uint32_t*)data = PWM_GetFreq(&dev->pwm_instance);
    } else {
        return RESULT_PARAM_ERR;
    }

    return RESULT_OK;
}

DeviceResult_t PWM_Device_Write(void* handle, const void* data, uint32_t size) {
    PWM_Device_t* dev = (PWM_Device_t*)handle;
    if (dev == NULL || data == NULL) return RESULT_PARAM_ERR;

    if (!dev->initialized) return RESULT_ERROR;

    if (size == sizeof(uint16_t)) {
        // 写入占空比
        uint16_t duty = *(uint16_t*)data;
        return PWM_Device_SetDuty(dev, duty);
    } else if (size == sizeof(uint32_t)) {
        // 写入频率
        uint32_t freq = *(uint32_t*)data;
        return PWM_Device_SetFrequency(dev, freq);
    }

    return RESULT_PARAM_ERR;
}

DeviceResult_t PWM_Device_Control(void* handle, DeviceCommandData_t* cmd) {
    PWM_Device_t* dev = (PWM_Device_t*)handle;
    if (dev == NULL || cmd == NULL) return RESULT_PARAM_ERR;

    if (!dev->initialized) return RESULT_ERROR;

    switch(cmd->cmd) {
        case CMD_PWM_SET_DUTY:
            if (cmd->params == NULL || cmd->param_size != sizeof(uint16_t)) {
                return RESULT_PARAM_ERR;
            }
            return PWM_Device_SetDuty(dev, *(uint16_t*)cmd->params);

        case CMD_PWM_SET_DUTY_SLOW:
            if (cmd->params == NULL || cmd->param_size != sizeof(PWM_RampCmd_t)) {
                return RESULT_PARAM_ERR;
            }
            {
                PWM_RampCmd_t* ramp = (PWM_RampCmd_t*)cmd->params;
                return PWM_Device_SetDutyWithRamp(dev, ramp->targetDuty, ramp->rampTimeMs);
            }

        case CMD_PWM_SET_FREQ:
            if (cmd->params == NULL || cmd->param_size != sizeof(uint32_t)) {
                return RESULT_PARAM_ERR;
            }
            return PWM_Device_SetFrequency(dev, *(uint32_t*)cmd->params);

        case CMD_PWM_START:
            return PWM_Device_Start(dev);

        case CMD_PWM_STOP:
            return PWM_Device_Stop(dev);

        case CMD_PWM_OUTPUT_ENABLE:
            return PWM_Device_OutputEnable(dev);

        case CMD_PWM_OUTPUT_DISABLE:
            return PWM_Device_OutputDisable(dev);

        case CMD_PWM_EMERGENCY_STOP:
            return PWM_Device_EmergencyStop(dev);

        default:
            return RESULT_ERROR;
    }
}

DeviceResult_t PWM_Device_Update(void* handle) {
    PWM_Device_t* dev = (PWM_Device_t*)handle;
    if (dev == NULL) return RESULT_PARAM_ERR;

    if (!dev->initialized) return RESULT_ERROR;

    // 更新PWM状态（处理缓启动）
    pwm_error_t err = PWM_Update(&dev->pwm_instance);

    DeviceResult_t result;
    PWM_Device_ConvertError(&result, err);
    return result;
}

// ========== PWM设备特定接口实现 ==========

DeviceResult_t PWM_Device_SetDuty(PWM_Device_t* dev, uint16_t dutyPercent) {
    if (dev == NULL) return RESULT_PARAM_ERR;
    if (!dev->initialized) return RESULT_ERROR;

    if (dutyPercent > 100) return RESULT_PARAM_ERR;

    pwm_error_t err = PWM_SetDuty(&dev->pwm_instance, dutyPercent);

    DeviceResult_t result;
    PWM_Device_ConvertError(&result, err);
    return result;
}

DeviceResult_t PWM_Device_SetDutyWithRamp(PWM_Device_t* dev, uint16_t targetDuty, uint16_t rampTimeMs) {
    if (dev == NULL) return RESULT_PARAM_ERR;
    if (!dev->initialized) return RESULT_ERROR;

    if (targetDuty > 100) return RESULT_PARAM_ERR;

    pwm_error_t err = PWM_StartRampFromCurrent(&dev->pwm_instance, targetDuty, rampTimeMs);

    DeviceResult_t result;
    PWM_Device_ConvertError(&result, err);
    return result;
}

DeviceResult_t PWM_Device_SetFrequency(PWM_Device_t* dev, uint32_t freqHz) {
    if (dev == NULL) return RESULT_PARAM_ERR;
    if (!dev->initialized) return RESULT_ERROR;

    if (freqHz < 1) return RESULT_PARAM_ERR;

    pwm_error_t err = PWM_SetFrequency(&dev->pwm_instance, freqHz);

    DeviceResult_t result;
    PWM_Device_ConvertError(&result, err);
    return result;
}

DeviceResult_t PWM_Device_Start(PWM_Device_t* dev) {
    if (dev == NULL) return RESULT_PARAM_ERR;
    if (!dev->initialized) return RESULT_ERROR;

    pwm_error_t err = PWM_Start(&dev->pwm_instance);

    DeviceResult_t result;
    PWM_Device_ConvertError(&result, err);
    return result;
}

DeviceResult_t PWM_Device_Stop(PWM_Device_t* dev) {
    if (dev == NULL) return RESULT_PARAM_ERR;
    if (!dev->initialized) return RESULT_ERROR;

    pwm_error_t err = PWM_Stop(&dev->pwm_instance);

    DeviceResult_t result;
    PWM_Device_ConvertError(&result, err);
    return result;
}

DeviceResult_t PWM_Device_EmergencyStop(PWM_Device_t* dev) {
    if (dev == NULL) return RESULT_PARAM_ERR;
    if (!dev->initialized) return RESULT_ERROR;

    // 紧急停止：立即停止PWM并禁用输出
    PWM_StopRamp(&dev->pwm_instance);           // 停止缓启动
    PWM_SetDuty(&dev->pwm_instance, 0);         // 占空比设为0
    PWM_OutputCmd(&dev->pwm_instance, PWM_OUTPUT_DISABLE);  // 禁用输出

    dev->output_enabled = 0;

    return RESULT_OK;
}

DeviceResult_t PWM_Device_OutputEnable(PWM_Device_t* dev) {
    if (dev == NULL) return RESULT_PARAM_ERR;
    if (!dev->initialized) return RESULT_ERROR;

    pwm_error_t err = PWM_OutputCmd(&dev->pwm_instance, PWM_OUTPUT_ENABLE);

    DeviceResult_t result;
    PWM_Device_ConvertError(&result, err);

    if (result == RESULT_OK) {
        dev->output_enabled = 1;
    }

    return result;
}

DeviceResult_t PWM_Device_OutputDisable(PWM_Device_t* dev) {
    if (dev == NULL) return RESULT_PARAM_ERR;
    if (!dev->initialized) return RESULT_ERROR;

    pwm_error_t err = PWM_OutputCmd(&dev->pwm_instance, PWM_OUTPUT_DISABLE);

    DeviceResult_t result;
    PWM_Device_ConvertError(&result, err);

    if (result == RESULT_OK) {
        dev->output_enabled = 0;
    }

    return result;
}

// ========== 状态查询接口实现 ==========

uint16_t PWM_Device_GetDuty(PWM_Device_t* dev) {
    if (dev == NULL || !dev->initialized) return 0;
    return PWM_GetDuty(&dev->pwm_instance);
}

uint32_t PWM_Device_GetFrequency(PWM_Device_t* dev) {
    if (dev == NULL || !dev->initialized) return 0;
    return PWM_GetFreq(&dev->pwm_instance);
}

pwm_state_t PWM_Device_GetState(PWM_Device_t* dev) {
    if (dev == NULL || !dev->initialized) return PWM_STATE_IDLE;
    return PWM_GetState(&dev->pwm_instance);
}

bool PWM_Device_IsOutputEnabled(PWM_Device_t* dev) {
    if (dev == NULL || !dev->initialized) return false;
    return dev->output_enabled;
}
