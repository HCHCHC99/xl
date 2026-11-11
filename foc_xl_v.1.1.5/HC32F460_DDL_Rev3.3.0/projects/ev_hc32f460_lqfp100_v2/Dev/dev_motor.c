#include "dev_motor.h"
#include "dev_pwm.h"          // PWM设备层
#include "dev_voltage.h"      // 电压告警事件类型
#include "dev_sensor.h"       // 电流告警事件类型
#include "device_manager.h"
#include "TickTimer.h"
#include <string.h>
#include "rtt_log.h"
#include "dev_rturn.h"          // 添加这行，用于 RTurn_LimitEvent_t
#include "Template_Pwm.h"       // PWM配置（TMRA_4, PB6/PB7）
#include "hc32_ll_tmra.h"      // TMRA寄存器操作
#include "Pwm.h"                // PWM驱动

/* ============================================================================
 * 注意：本文件已移除对 g_motor_pwm_ch1 / g_motor_pwm_ch2 的依赖
 *       PWM 操作直接通过 TMRA 寄存器进行
 * ============================================================================*/

// 在文件开头，其他静态变量定义附近添加
static MotorDir_t s_eLastArbitrationDir = DIR_NONE;  // 上次仲裁的方向
static uint16_t s_u16LastTargetDuty = 0;              // 上次的目标占空比
static bool s_bStopPolarityPending = false;           // 是否需要切换停止极性

// 在文件开头，其他 include 之后添加
// 打印间隔控制
volatile int motor_state = 0;
static uint32_t s_u32LastMotorPrintTime = 0;
#define MOTOR_PRINT_INTERVAL_MS   500

// 缓启动时间配置（毫秒）
#define MOTOR_RAMP_TIME_MS      800

// 占空比限制（2%-98%）
#define MOTOR_DUTY_MIN          2
#define MOTOR_DUTY_MAX          98

// ========== Keil Watch 调试全局变量 ==========
// 电机仲裁层调试变量 - 在 Watch 窗口添加这些变量即可实时查看电机仲裁状态
MotorDevice_t* volatile g_pMotor_Dev_Watch = NULL;      // 电机设备指针（可展开查看所有成员）
MotorDebugInfo_t volatile g_stcMotorDebug = {0};     // 调试信息副本（结构体形式）
// 为了方便查看，再导出一些单独的变量
volatile uint8_t  g_u8MotorDebug_BlockFwdCount = 0;   // 正转阻塞队列数量
volatile uint8_t  g_u8MotorDebug_BlockRevCount = 0;   // 反转阻塞队列数量
volatile uint8_t  g_u8MotorDebug_AllowFwdCount = 0;   // 正转允许队列数量
volatile uint8_t  g_u8MotorDebug_AllowRevCount = 0;   // 反转允许队列数量
volatile uint8_t  g_u8MotorDebug_ActiveDeviceId = 0;  // 当前活动设备ID
volatile uint8_t  g_u8MotorDebug_ActiveDir = 0;       // 当前活动方向(0=停止,1=正转,2=反转)
volatile uint8_t  g_u8MotorDebug_State = 0;           // 电机状态(0=IDLE,1=RAMPING,2=RUNNING)
volatile float    g_fMotorDebug_CurrentDuty = 0.0f;   // 当前占空比
volatile uint8_t  g_u8MotorDebug_ConflictFault = 0;   // 冲突故障标志

// ========== 设备ID定义 ==========
#define DEV_PWM_MOTOR_ID    1   // 电机PWM设备ID
#define DEV_MOTOR_ID        0   // 电机设备ID（与 App_Motor_Project.h 中的 ID_MOTOR 保持一致）

// ========== 设备基因定义 ==========
typedef struct {
    MotorDeviceId_t id;
    MotorPriority_t priority;
    uint32_t capability;
} MotorDeviceGene_t;

// 根据优先级模式动态定义优先级值
#if MOTOR_PRIORITY_IO_HIGH
    #define MANUAL_PRIORITY  3
    #define CAN_PRIORITY     4
#else
    #define MANUAL_PRIORITY  4
    #define CAN_PRIORITY     3
#endif

// 设备基因表
static const MotorDeviceGene_t c_device_genes[] = {
#if MOTOR_MODE_BIPOLAR
    { DEV_ID_POWER_POS,  PRIO_POWER,     CAP_ALLOW },
    { DEV_ID_POWER_NEG,  PRIO_POWER,     CAP_ALLOW },
#endif
    // { DEV_ID_LIMIT_FWD,  PRIO_LIMIT,     CAP_BLOCK },
    // { DEV_ID_LIMIT_REV,  PRIO_LIMIT,     CAP_BLOCK },
    { DEV_ID_RTURN_FWD,         PRIO_LIMIT,     CAP_BLOCK },
    { DEV_ID_RTURN_REV,         PRIO_LIMIT,     CAP_BLOCK },
    { DEV_ID_OVERVOLTAGE_FWD,   PRIO_LIMIT,     CAP_BLOCK },
    { DEV_ID_OVERVOLTAGE_REV,   PRIO_LIMIT,     CAP_BLOCK },
    { DEV_ID_UNDERVOLTAGE_FWD,  PRIO_LIMIT,     CAP_BLOCK },
    { DEV_ID_UNDERVOLTAGE_REV,  PRIO_LIMIT,     CAP_BLOCK },
	{ DEV_ID_OVERCUR_FWD,       PRIO_LIMIT,     CAP_BLOCK },
    // { DEV_ID_CAN,        (MotorPriority_t)CAN_PRIORITY,   MOTOR_CAN_CAPABILITY },
    { DEV_ID_IO_FWD,     (MotorPriority_t)MANUAL_PRIORITY, MOTOR_MANUAL_CAPABILITY },
    { DEV_ID_IO_REV,     (MotorPriority_t)MANUAL_PRIORITY, MOTOR_MANUAL_CAPABILITY },
    // { DEV_ID_EMERGENCY,  PRIO_EMERGENCY, CAP_BLOCK }
};

// ========== 辅助函数：限制占空比在2%-98% ==========
static uint16_t Motor_LimitDuty(uint16_t duty) {
    if (duty < MOTOR_DUTY_MIN) return MOTOR_DUTY_MIN;
    if (duty > MOTOR_DUTY_MAX) return MOTOR_DUTY_MAX;
    return duty;
}

// ========== 使用TMRA寄存器设置停止极性（50%占空比） ==========
static void Motor_SetStopDuty(void) {
    // 停止：PB6=50%(高有效), PB7=50%(低有效)
    // 直接操作寄存器实现停止时的特殊极性
    
    stc_tmra_pwm_init_t stcPwmInit;
    (void)TMRA_PWM_StructInit(&stcPwmInit);
    uint32_t u32Period = TMRA_GetPeriodValue(CM_TMRA_4);
    uint32_t u32Compare = (u32Period * 50U) / 100U;

    /* CH1(PB6) - 高有效，50% */
    stcPwmInit.u32CompareValue        = u32Compare;
    stcPwmInit.u16StartPolarity       = TMRA_PWM_LOW;
    stcPwmInit.u16StopPolarity        = TMRA_PWM_LOW;
    stcPwmInit.u16CompareMatchPolarity = TMRA_PWM_HIGH;
    stcPwmInit.u16PeriodMatchPolarity = TMRA_PWM_LOW;
    (void)TMRA_PWM_Init(CM_TMRA_4, TMRA_CH1, &stcPwmInit);
    TMRA_PWM_OutputCmd(CM_TMRA_4, TMRA_CH1, ENABLE);

    /* CH2(PB7) - 低有效，50% */
    stcPwmInit.u32CompareValue        = u32Compare;
    stcPwmInit.u16StartPolarity       = TMRA_PWM_HIGH;
    stcPwmInit.u16StopPolarity        = TMRA_PWM_HIGH;
    stcPwmInit.u16CompareMatchPolarity = TMRA_PWM_LOW;
    stcPwmInit.u16PeriodMatchPolarity = TMRA_PWM_HIGH;
    (void)TMRA_PWM_Init(CM_TMRA_4, TMRA_CH2, &stcPwmInit);
    TMRA_PWM_OutputCmd(CM_TMRA_4, TMRA_CH2, ENABLE);

//    /* CH3(PB8) - 高有效，50% */
//    stcPwmInit.u32CompareValue        = u32Compare;
//    stcPwmInit.u16StartPolarity       = TMRA_PWM_LOW;
//    stcPwmInit.u16StopPolarity        = TMRA_PWM_LOW;
//    stcPwmInit.u16CompareMatchPolarity = TMRA_PWM_HIGH;
//    stcPwmInit.u16PeriodMatchPolarity = TMRA_PWM_LOW;
//    (void)TMRA_PWM_Init(CM_TMRA_4, TMRA_CH3, &stcPwmInit);
//    TMRA_PWM_OutputCmd(CM_TMRA_4, TMRA_CH3, ENABLE);
//
//    /* CH4(PB9) - 低有效，50% */
//    stcPwmInit.u32CompareValue        = u32Compare;
//    stcPwmInit.u16StartPolarity       = TMRA_PWM_HIGH;
//    stcPwmInit.u16StopPolarity        = TMRA_PWM_HIGH;
//    stcPwmInit.u16CompareMatchPolarity = TMRA_PWM_LOW;
//    stcPwmInit.u16PeriodMatchPolarity = TMRA_PWM_HIGH;
//    (void)TMRA_PWM_Init(CM_TMRA_4, TMRA_CH4, &stcPwmInit);
//    TMRA_PWM_OutputCmd(CM_TMRA_4, TMRA_CH4, ENABLE);
}

// ========== 设置运行时的低有效极性（全部低有效） ==========
static void Motor_SetRunPolarity(void) {
    stc_tmra_pwm_init_t stcPwmInit;
    (void)TMRA_PWM_StructInit(&stcPwmInit);

    /* CH1(PB6) - 低有效 */
    stcPwmInit.u16StartPolarity       = TMRA_PWM_HIGH;
    stcPwmInit.u16StopPolarity        = TMRA_PWM_HIGH;
    stcPwmInit.u16CompareMatchPolarity = TMRA_PWM_LOW;
    stcPwmInit.u16PeriodMatchPolarity = TMRA_PWM_HIGH;
    (void)TMRA_PWM_Init(CM_TMRA_4, TMRA_CH1, &stcPwmInit);
    TMRA_PWM_OutputCmd(CM_TMRA_4, TMRA_CH1, ENABLE);

    /* CH2(PB7) - 低有效 */
    (void)TMRA_PWM_Init(CM_TMRA_4, TMRA_CH2, &stcPwmInit);
    TMRA_PWM_OutputCmd(CM_TMRA_4, TMRA_CH2, ENABLE);

//    /* CH3(PB8) - 低有效 */
//    (void)TMRA_PWM_Init(CM_TMRA_4, TMRA_CH3, &stcPwmInit);
//    TMRA_PWM_OutputCmd(CM_TMRA_4, TMRA_CH3, ENABLE);
//
//    /* CH4(PB9) - 低有效 */
//    (void)TMRA_PWM_Init(CM_TMRA_4, TMRA_CH4, &stcPwmInit);
//    TMRA_PWM_OutputCmd(CM_TMRA_4, TMRA_CH4, ENABLE);
}

// ========== 使用TMRA寄存器设置运行时的占空比 ==========
static void Motor_SetRunDutyDirect(uint16_t ch1, uint16_t ch2, uint16_t ch3, uint16_t ch4) {
    // 直接操作寄存器设置占空比
    uint32_t u32Period = TMRA_GetPeriodValue(CM_TMRA_4);
    TMRA_SetCompareValue(CM_TMRA_4, TMRA_CH1, (u32Period * ch1) / 100U);
    TMRA_SetCompareValue(CM_TMRA_4, TMRA_CH2, (u32Period * ch2) / 100U);
//    TMRA_SetCompareValue(CM_TMRA_4, TMRA_CH3, (u32Period * ch3) / 100U);
//    TMRA_SetCompareValue(CM_TMRA_4, TMRA_CH4, (u32Period * ch4) / 100U);
}

// ========== 修改：切换极性前先将占空比归零 ==========
static void Motor_RampForward(uint16_t target_duty) {
    uint16_t limited_duty = Motor_LimitDuty(target_duty);
    uint16_t duty_low = limited_duty;
    uint16_t duty_high = 100 - limited_duty;
    
    // 先快速归零（直接设置50%停止占空比）
    Motor_SetStopDuty();
    
    // 等待归零完成（简单延时）
    tickTimer_DelayMs(10);
    
    MAIN_D("[MOTOR] Ramp FWD: target CH1/2=%d%%, CH3/4=%d%%, time=%dms\r\n",
           duty_low, duty_high, MOTOR_RAMP_TIME_MS);

    // 切换极性（此时占空比为50%停止状态，不会冲击）
    Motor_SetRunPolarity();

    // 从 50% 缓启动到目标值（直接寄存器操作实现缓启动）
    uint32_t u32Period = TMRA_GetPeriodValue(CM_TMRA_4);
    uint16_t start_val = 50;
    uint16_t end_val_low = duty_low;
    uint16_t end_val_high = duty_high;
    
    // 简单缓启动：分10步，每步 MOTOR_RAMP_TIME_MS/10 ms
    uint16_t steps = 20;
    uint16_t delay_per_step = MOTOR_RAMP_TIME_MS / steps;
    
    for (uint16_t step = 0; step <= steps; step++) {
        float ratio = (float)step / (float)steps;
        uint16_t current_low = start_val + (uint16_t)((end_val_low - start_val) * ratio);
        uint16_t current_high = 100 - current_low;
        
        TMRA_SetCompareValue(CM_TMRA_4, TMRA_CH1, (u32Period * current_low) / 100U);
        TMRA_SetCompareValue(CM_TMRA_4, TMRA_CH2, (u32Period * current_low) / 100U);
//        TMRA_SetCompareValue(CM_TMRA_4, TMRA_CH3, (u32Period * current_high) / 100U);
//        TMRA_SetCompareValue(CM_TMRA_4, TMRA_CH4, (u32Period * current_high) / 100U);
        
        tickTimer_DelayMs(delay_per_step);
    }
    
    MAIN_D("[MOTOR] Ramp FWD: complete, duty=%d%%\r\n", limited_duty);
}

// ========== 立即设置反转（无缓启动） ==========
static void Motor_RampReverse(uint16_t target_duty) {
    uint16_t limited_duty = Motor_LimitDuty(target_duty);
    uint16_t duty_low = 100 - limited_duty;
    uint16_t duty_high = limited_duty;
    
    // 先快速归零（直接设置50%停止占空比）
    Motor_SetStopDuty();
    
    // 等待归零完成（简单延时）
    tickTimer_DelayMs(10);
    
    MAIN_D("[MOTOR] Ramp REV: target CH1/2=%d%%, time=%dms\r\n",
           duty_low, MOTOR_RAMP_TIME_MS);

    // 切换极性（此时占空比为50%停止状态，不会冲击）
    Motor_SetRunPolarity();

    // 从 50% 缓启动到目标值（直接寄存器操作实现缓启动）
    uint32_t u32Period = TMRA_GetPeriodValue(CM_TMRA_4);
    uint16_t start_val = 50;
    uint16_t end_val_low = duty_low;
    uint16_t end_val_high = duty_high;
    
    // 简单缓启动：分20步
    uint16_t steps = 20;
    uint16_t delay_per_step = MOTOR_RAMP_TIME_MS / steps;
    
    for (uint16_t step = 0; step <= steps; step++) {
        float ratio = (float)step / (float)steps;
        uint16_t current_low = start_val + (uint16_t)((end_val_low - start_val) * ratio);
        uint16_t current_high = 100 - current_low;
        
        TMRA_SetCompareValue(CM_TMRA_4, TMRA_CH1, (u32Period * current_low) / 100U);
        TMRA_SetCompareValue(CM_TMRA_4, TMRA_CH2, (u32Period * current_low) / 100U);
//        TMRA_SetCompareValue(CM_TMRA_4, TMRA_CH3, (u32Period * current_high) / 100U);
//        TMRA_SetCompareValue(CM_TMRA_4, TMRA_CH4, (u32Period * current_high) / 100U);
        
        tickTimer_DelayMs(delay_per_step);
    }
    
    MAIN_D("[MOTOR] Ramp REV: complete, duty=%d%%\r\n", limited_duty);
}

// ========== 立即设置正转（无缓启动） ==========
static void Motor_RunForwardImmediate(uint16_t duty) {
    uint16_t limited_duty = Motor_LimitDuty(duty);
    uint16_t duty_low = limited_duty;
    uint16_t duty_high = 100 - limited_duty;
    
    // 先设置运行极性
    Motor_SetRunPolarity();
    
    // 立即设置占空比
    Motor_SetRunDutyDirect(duty_low, duty_low, duty_high, duty_high);

    MAIN_D("[MOTOR] Immediate FWD: duty=%d%%, CH1/2=%d%%, CH3/4=%d%%\r\n",
           limited_duty, duty_low, duty_high);
}

// ========== 立即设置反转（无缓启动） ==========
static void Motor_RunReverseImmediate(uint16_t duty) {
    uint16_t limited_duty = Motor_LimitDuty(duty);
    uint16_t duty_low = 100 - limited_duty;
    uint16_t duty_high = limited_duty;
    
    // 先设置运行极性
    Motor_SetRunPolarity();
    
    // 立即设置占空比
    Motor_SetRunDutyDirect(duty_low, duty_low, duty_high, duty_high);

    MAIN_D("[MOTOR] Immediate REV: duty=%d%%, CH1/2=%d%%, CH3/4=%d%%\r\n",
           limited_duty, duty_low, duty_high);
}

// ========== 电机仲裁结果回调函数（弱定义，用户可重写） ==========
__weak void Motor_OnArbitrationStop(MotorDevice_t* motor) {
    (void)motor;
    
    // 检查是否需要执行停止
    static bool s_bFirstStop = true;
    
    if (s_eLastArbitrationDir != DIR_NONE || s_bFirstStop) {
        MAIN_D("[MOTOR_ARB] Stop: from dir=%d, first_stop=%d\r\n", 
               s_eLastArbitrationDir, s_bFirstStop);
        
        if (s_bFirstStop) {
            // 第一次停止：直接设置停止极性和50%占空比
            Motor_SetStopDuty();
            s_bFirstStop = false;
            MAIN_D("[MOTOR_ARB] First stop: set stop polarity and 50%% duty directly\r\n");
        } else {
            // 后续停止：使用缓启动
            Motor_SetStopDuty();
            MAIN_D("[MOTOR_ARB] Stop: set stop polarity\r\n");
        }
        
        // 更新方向记录
        s_eLastArbitrationDir = DIR_NONE;
        s_u16LastTargetDuty = 0;
    }
    
    motor_state = 0;
}

__weak void Motor_OnArbitrationFwd(MotorDevice_t* motor, float duty) {
    (void)motor;
    uint16_t duty_percent = (uint16_t)(duty);
    
    // 检查是否需要执行缓启动
    if (s_eLastArbitrationDir != DIR_FWD || s_u16LastTargetDuty != duty_percent) {
        MAIN_D("[MOTOR_ARB] FWD: start ramp...\r\n");
        Motor_RampForward(duty_percent);
        
        s_eLastArbitrationDir = DIR_FWD;
        s_u16LastTargetDuty = duty_percent;
    }
    motor_state = 1;
}

__weak void Motor_OnArbitrationRev(MotorDevice_t* motor, float duty) {
    (void)motor;
    uint16_t duty_percent = (uint16_t)(duty);
    
    // 检查是否需要执行缓启动
    if (s_eLastArbitrationDir != DIR_REV || s_u16LastTargetDuty != duty_percent) {
        MAIN_D("[MOTOR_ARB] REV: start ramp...\r\n");
        Motor_RampReverse(duty_percent);
        
        s_eLastArbitrationDir = DIR_REV;
        s_u16LastTargetDuty = duty_percent;
    }
    motor_state = 2;
}

// ========== 内部函数声明 ==========
static const MotorDeviceGene_t* Motor_GetGene(MotorDeviceId_t id);
static void Motor_CmdList_Remove(MotorCommandList_t* q, MotorDeviceId_t id);
static void Motor_CmdList_SetAllow(MotorCommandList_t* q, MotorControlCommand_t cmd);
static void Motor_CmdList_SetBlock(MotorCommandList_t* q, MotorControlCommand_t cmd);
static void Motor_ArbitrationDecision(MotorDevice_t* motor);
static void Motor_UpdateDebugInfo(MotorDevice_t* motor);

// ========== 内部函数实现 ==========

static const MotorDeviceGene_t* Motor_GetGene(MotorDeviceId_t id) {
    for(int i = 0; i < sizeof(c_device_genes)/sizeof(MotorDeviceGene_t); i++) {
        if(c_device_genes[i].id == id) return &c_device_genes[i];
    }
    return NULL;
}

static void Motor_CmdList_Remove(MotorCommandList_t* q, MotorDeviceId_t id) {
    if (id == DEV_ID_RTURN_FWD || id == DEV_ID_RTURN_REV) {
        MAIN_D("[REMOVE_DEBUG] Trying to remove id=%d from queue at 0x%p, current count=%d\r\n", 
               id, q, q->count);
    }    
    for (int i = 0; i < q->count; i++) {
        if (q->commands[i].device_id == id) {
            MAIN_D("[REMOVE_DEBUG] FOUND and REMOVED id=%d from queue at 0x%p\r\n", id, q);            
            for (int j = i; j < q->count - 1; j++) {
                q->commands[j] = q->commands[j + 1];
            }
            q->count--;
            // 清空被移除的位置
            if (q->count < MAX_COMMANDS_PER_DIRECTION) {
                q->commands[q->count].device_id = DEV_ID_NONE;
                q->commands[q->count].priority = PRIO_NONE;
                q->commands[q->count].type = CMD_TYPE_NONE_USE;
                q->commands[q->count].param = 0.0f;
                q->commands[q->count].timestamp = 0;
            }
            // 实时更新 Keil Watch 独立变量
            if (q == &g_pMotor_Dev_Watch->block_fwd) {
                g_u8MotorDebug_BlockFwdCount = q->count;
            } else if (q == &g_pMotor_Dev_Watch->block_rev) {
                g_u8MotorDebug_BlockRevCount = q->count;
            } else if (q == &g_pMotor_Dev_Watch->allow_fwd) {
                g_u8MotorDebug_AllowFwdCount = q->count;
            } else if (q == &g_pMotor_Dev_Watch->allow_rev) {
                g_u8MotorDebug_AllowRevCount = q->count;
            }
            return;
        }
    }
}

static void Motor_CmdList_SetAllow(MotorCommandList_t* q, MotorControlCommand_t cmd) {
    Motor_CmdList_Remove(q, cmd.device_id);
    if (q->count >= MAX_COMMANDS_PER_DIRECTION) return;

    int idx = 0;
    for (; idx < q->count; idx++) {
        if (cmd.priority < q->commands[idx].priority) break;
    }

    for (int i = q->count; i > idx; i--) {
        q->commands[i] = q->commands[i - 1];
    }

    q->commands[idx] = cmd;
    q->count++;
    
    // 实时更新 Keil Watch 独立变量
    if (g_pMotor_Dev_Watch != NULL) {
        if (q == &g_pMotor_Dev_Watch->allow_fwd) {
            g_u8MotorDebug_AllowFwdCount = q->count;
        } else if (q == &g_pMotor_Dev_Watch->allow_rev) {
            g_u8MotorDebug_AllowRevCount = q->count;
        }
    }
}

static void Motor_CmdList_SetBlock(MotorCommandList_t* q, MotorControlCommand_t cmd) {
    for (int i = 0; i < q->count; i++) {
        if (q->commands[i].device_id == cmd.device_id) return;
    }
    if (q->count >= MAX_COMMANDS_PER_DIRECTION) return;
    q->commands[q->count++] = cmd;
    
    // 实时更新 Keil Watch 独立变量
    if (g_pMotor_Dev_Watch != NULL) {
        if (q == &g_pMotor_Dev_Watch->block_fwd) {
            g_u8MotorDebug_BlockFwdCount = q->count;
        } else if (q == &g_pMotor_Dev_Watch->block_rev) {
            g_u8MotorDebug_BlockRevCount = q->count;
        }
    }
}

static void Motor_UpdateDebugInfo(MotorDevice_t* motor) {
    MotorDebugInfo_t* d = &motor->debug_info;

    // 同步block_fwd数组
    d->block_fwd.count = motor->block_fwd.count;
    for(int i = 0; i < MAX_COMMANDS_PER_DIRECTION; i++) {
        if (i < d->block_fwd.count) {
            d->block_fwd.device_ids[i] = motor->block_fwd.commands[i].device_id;
        } else {
            d->block_fwd.device_ids[i] = DEV_ID_NONE;
        }
    }

    // 同步block_rev数组
    d->block_rev.count = motor->block_rev.count;
    for(int i = 0; i < MAX_COMMANDS_PER_DIRECTION; i++) {
        if (i < d->block_rev.count) {
            d->block_rev.device_ids[i] = motor->block_rev.commands[i].device_id;
        } else {
            d->block_rev.device_ids[i] = DEV_ID_NONE;
        }
    }

    // 同步allow_fwd数组
    d->allow_fwd.count = motor->allow_fwd.count;
    for(int i = 0; i < MAX_COMMANDS_PER_DIRECTION; i++) {
        if (i < d->allow_fwd.count) {
            d->allow_fwd.device_ids[i] = motor->allow_fwd.commands[i].device_id;
            d->allow_fwd.priorities[i] = motor->allow_fwd.commands[i].priority;
        } else {
            d->allow_fwd.device_ids[i] = DEV_ID_NONE;
            d->allow_fwd.priorities[i] = PRIO_NONE;
        }
    }

    // 同步allow_rev数组
    d->allow_rev.count = motor->allow_rev.count;
    for(int i = 0; i < MAX_COMMANDS_PER_DIRECTION; i++) {
        if (i < d->allow_rev.count) {
            d->allow_rev.device_ids[i] = motor->allow_rev.commands[i].device_id;
            d->allow_rev.priorities[i] = motor->allow_rev.commands[i].priority;
        } else {
            d->allow_rev.device_ids[i] = DEV_ID_NONE;
            d->allow_rev.priorities[i] = PRIO_NONE;
        }
    }

    d->state = motor->state;
    d->active_dir = motor->active_dir;
    d->active_device_id = motor->active_device_id;
    d->current_duty = motor->last_sent_duty;
    d->conflict_fault = motor->debug_info.conflict_fault;
    
    // ========== 同步到全局调试变量（供 Keil Watch 查看） ==========
    memcpy((void*)&g_stcMotorDebug, &motor->debug_info, sizeof(MotorDebugInfo_t));
    
    // 同步到单独变量
    g_u8MotorDebug_BlockFwdCount = motor->block_fwd.count;
    g_u8MotorDebug_BlockRevCount = motor->block_rev.count;
    g_u8MotorDebug_AllowFwdCount = motor->allow_fwd.count;
    g_u8MotorDebug_AllowRevCount = motor->allow_rev.count;
    g_u8MotorDebug_ActiveDeviceId = motor->active_device_id;
    g_u8MotorDebug_ActiveDir = motor->active_dir;
    g_u8MotorDebug_State = motor->state;
    g_fMotorDebug_CurrentDuty = motor->last_sent_duty;
    g_u8MotorDebug_ConflictFault = motor->debug_info.conflict_fault ? 1 : 0;
}

static void Motor_ArbitrationDecision(MotorDevice_t* motor) {
    if (!motor->enable) return;

    // 1. 指令候选选择
    MotorControlCommand_t* fwd_cmd = NULL;
    MotorControlCommand_t* rev_cmd = NULL;

    if(motor->block_fwd.count == 0 && motor->allow_fwd.count > 0) {
        fwd_cmd = &motor->allow_fwd.commands[0];
    }

    if(motor->block_rev.count == 0 && motor->allow_rev.count > 0) {
        rev_cmd = &motor->allow_rev.commands[0];
    }

    // 2. 冲突判定
    MotorControlCommand_t* final = NULL;
    motor->debug_info.conflict_fault = false;

    if (fwd_cmd && rev_cmd) {
        if (fwd_cmd->device_id == rev_cmd->device_id) {
            final = NULL;
            motor->debug_info.conflict_fault = true;
        } else if (fwd_cmd->priority < rev_cmd->priority) {
            final = fwd_cmd;
        } else if (rev_cmd->priority < fwd_cmd->priority) {
            final = rev_cmd;
        } else {
            final = NULL;
        }
    } else {
        final = fwd_cmd ? fwd_cmd : rev_cmd;
    }

    // 3. 执行决策 - 调试打印
    if (final) {
        MOTOR_DEBUG("Arbitration: Selected %s (prio=%d), dir=%s, duty=%.1f%%\r\n",
                    (final->device_id == DEV_ID_IO_FWD) ? "IO_FWD" :
                    (final->device_id == DEV_ID_IO_REV) ? "IO_REV" :
                    (final->device_id == DEV_ID_RTURN_FWD) ? "RTURN_FWD" :
                    (final->device_id == DEV_ID_RTURN_REV) ? "RTURN_REV" :
					(final->device_id == DEV_ID_OVERCUR_FWD) ? "OVERCUR_FWD" :
                    (final->device_id == DEV_ID_CAN) ? "CAN" :
                    (final->device_id == DEV_ID_POWER_POS) ? "POWER_POS" :
                    (final->device_id == DEV_ID_POWER_NEG) ? "POWER_NEG" : "UNKNOWN",
                    final->priority,
                    (final->type == CMD_TYPE_RUN_FWD || final->type == CMD_TYPE_RAMP_FWD) ? "FWD" : "REV",
                    final->param);
        
        motor->active_device_id = final->device_id;
        motor->active_dir = (final->type == CMD_TYPE_RUN_FWD || final->type == CMD_TYPE_RAMP_FWD) ? DIR_FWD : DIR_REV;
        motor->last_sent_duty = final->param;
        motor->state = (final->type == CMD_TYPE_RAMP_FWD || final->type == CMD_TYPE_RAMP_REV) ? MS_RAMPING : MS_RUNNING;

        // 调用仲裁结果回调（直接操作 TMRA 寄存器）
        if (motor->active_dir == DIR_FWD) {
            Motor_OnArbitrationFwd(motor, motor->last_sent_duty);
        } else {
            Motor_OnArbitrationRev(motor, motor->last_sent_duty);
        }
    } else {
        MOTOR_DEBUG("Arbitration: No command selected - STOP\r\n");
        
        motor->last_sent_duty = 0.0f;
        motor->state = MS_IDLE;
        motor->active_dir = DIR_NONE;
        motor->active_device_id = DEV_ID_NONE;

        // 调用仲裁停止回调（直接操作 TMRA 寄存器）
        Motor_OnArbitrationStop(motor);
    }

    Motor_UpdateDebugInfo(motor);
}

// ========== EventBus回调函数 ==========

void Motor_OnOvercurrent(void* payload) {
    MotorOvercurrentEvent_t* ev = (MotorOvercurrentEvent_t*)payload;
    MAIN_D("Motor: Overcurrent event! ADC=%d, Current=%d mA, Threshold=%d mA, Duration=%lu ms\r\n",
           ev->adc_id, ev->current_ma, ev->threshold_ma, ev->duration_ms);

    // 执行紧急停止
    DeviceNode_t* node = DeviceManager_Get(DEV_MOTOR_ID);
    if (node && node->private_data) {
        MotorDevice_t* motor = (MotorDevice_t*)node->private_data;
        Motor_EmergencyStop(motor);
        MAIN_D("Motor: Emergency stop due to overcurrent!\r\n");
    }
}

// ========== 电压告警回调 ==========
void Motor_OnVoltageAlarm(void* payload) {
    Voltage_AlarmEvent_t* ev = (Voltage_AlarmEvent_t*)payload;

    DeviceNode_t* node = DeviceManager_Get(DEV_MOTOR_ID);
    if (!node || !node->private_data) return;

    MotorDevice_t* motor = (MotorDevice_t*)node->private_data;

    if (ev->u8IsActive) {
        MotorDeviceId_t dev_id_fwd, dev_id_rev;
        const char* alarm_name;

        if (ev->u8AlarmType == VOLTAGE_ALARM_OVERVOLTAGE) {
            dev_id_fwd = DEV_ID_OVERVOLTAGE_FWD;
            dev_id_rev = DEV_ID_OVERVOLTAGE_REV;
            alarm_name = "OVERVOLTAGE";
        } else {
            dev_id_fwd = DEV_ID_UNDERVOLTAGE_FWD;
            dev_id_rev = DEV_ID_UNDERVOLTAGE_REV;
            alarm_name = "UNDERVOLTAGE";
        }

        MotorControlCommand_t block_fwd_cmd = {
            .device_id = dev_id_fwd,
            .priority = PRIO_LIMIT,
            .type = CMD_TYPE_BLOCK_FWD,
            .timestamp = tickTimer_GetCount()
        };
        Motor_CmdList_SetBlock(&motor->block_fwd, block_fwd_cmd);

        MotorControlCommand_t block_rev_cmd = {
            .device_id = dev_id_rev,
            .priority = PRIO_LIMIT,
            .type = CMD_TYPE_BLOCK_REV,
            .timestamp = tickTimer_GetCount()
        };
        Motor_CmdList_SetBlock(&motor->block_rev, block_rev_cmd);

        MAIN_D("Motor: %s alarm! Bus=%lu mV, Threshold=%lu mV - Block both directions!\r\n",
               alarm_name, ev->u32BusVoltageMv, ev->u32ThresholdMv);
    } else {
        MotorDeviceId_t dev_id_fwd, dev_id_rev;
        const char* alarm_name;

        if (ev->u8AlarmType == VOLTAGE_ALARM_OVERVOLTAGE) {
            dev_id_fwd = DEV_ID_OVERVOLTAGE_FWD;
            dev_id_rev = DEV_ID_OVERVOLTAGE_REV;
            alarm_name = "OVERVOLTAGE";
        } else {
            dev_id_fwd = DEV_ID_UNDERVOLTAGE_FWD;
            dev_id_rev = DEV_ID_UNDERVOLTAGE_REV;
            alarm_name = "UNDERVOLTAGE";
        }

        Motor_CmdList_Remove(&motor->block_fwd, dev_id_fwd);
        Motor_CmdList_Remove(&motor->block_rev, dev_id_rev);

        MAIN_D("Motor: %s released! Bus=%lu mV - Unblock both directions\r\n",
               alarm_name, ev->u32BusVoltageMv);
    }

    Motor_ArbitrationDecision(motor);
}

void Motor_OnRTurnLimit(void* payload) {
    RTurn_LimitEvent_t* ev = (RTurn_LimitEvent_t*)payload;

    DeviceNode_t* node = DeviceManager_Get(DEV_MOTOR_ID);
    if (!node || !node->private_data) return;

    MotorDevice_t* motor = (MotorDevice_t*)node->private_data;

    if (ev->u8IsActive) {
        MotorControlCommand_t cmd = {
            .device_id = (ev->u8Direction == RTURN_LIMIT_FORWARD) ? DEV_ID_RTURN_FWD : DEV_ID_RTURN_REV,
            .priority = PRIO_LIMIT,
            .timestamp = tickTimer_GetCount()
        };

        if (ev->u8Direction == RTURN_LIMIT_FORWARD) {
            cmd.type = CMD_TYPE_BLOCK_FWD;
            Motor_CmdList_SetBlock(&motor->block_fwd, cmd);
            Motor_ClearAllowFwd(motor);
            MAIN_D("Motor: RTurn forward limit triggered, block forward + clear allow_fwd\r\n");
        } else {
            cmd.type = CMD_TYPE_BLOCK_REV;
            Motor_CmdList_SetBlock(&motor->block_rev, cmd);
            Motor_ClearAllowRev(motor);
            MAIN_D("Motor: RTurn reverse limit triggered, block reverse + clear allow_rev\r\n");
        }
    } else {
        MotorDeviceId_t id = (ev->u8Direction == RTURN_LIMIT_FORWARD) ? DEV_ID_RTURN_FWD : DEV_ID_RTURN_REV;
        if (ev->u8Direction == RTURN_LIMIT_FORWARD) {
            Motor_CmdList_Remove(&motor->block_fwd, id);
            MAIN_D("Motor: RTurn forward limit released\r\n");
        } else if (ev->u8Direction == RTURN_LIMIT_REVERSE) {
            Motor_CmdList_Remove(&motor->block_rev, id);
            MAIN_D("Motor: RTurn reverse limit released\r\n");
        } else {
            Motor_CmdList_Remove(&motor->block_fwd, DEV_ID_RTURN_FWD);
            Motor_CmdList_Remove(&motor->block_rev, DEV_ID_RTURN_REV);
        }
    }

    Motor_ArbitrationDecision(motor);
}

// ========== 电流告警回调 ==========
void Motor_OnCurrentAlarm(void* payload) {
    Current_AlarmEvent_t* ev = (Current_AlarmEvent_t*)payload;

    DeviceNode_t* node = DeviceManager_Get(DEV_MOTOR_ID);
    if (!node || !node->private_data) return;

    MotorDevice_t* motor = (MotorDevice_t*)node->private_data;

    if (ev->u8IsActive) {
        MotorDir_t eDesiredDir = Motor_GetDesiredDirection(motor);
        
        if (eDesiredDir == DIR_FWD)
        {
            MotorControlCommand_t block_fwd_cmd = {
                .device_id = DEV_ID_OVERCUR_FWD,
                .priority = PRIO_LIMIT,
                .type = CMD_TYPE_BLOCK_FWD,
                .timestamp = tickTimer_GetCount()
            };
            Motor_CmdList_SetBlock(&motor->block_fwd, block_fwd_cmd);

            MAIN_D("Motor: OVERCURRENT alarm (FWD)! Current=%ld mA, Threshold=%ld mA - Block forward!\r\n",
                   (long)ev->s32CurrentMa, (long)ev->s32ThresholdMa);
        }
        else
        {
            MAIN_D("Motor: OVERCURRENT alarm (dir=%d, REV/STOP is normal stall)! Current=%ld mA, Threshold=%ld mA - Skip block forward\r\n",
                   (int)eDesiredDir, (long)ev->s32CurrentMa, (long)ev->s32ThresholdMa);
        }
    } else {
        Motor_CmdList_Remove(&motor->block_fwd, DEV_ID_OVERCUR_FWD);

        MAIN_D("Motor: OVERCURRENT released! Current=%ld mA - Unblock forward\r\n",
               (long)ev->s32CurrentMa);
    }

    Motor_ArbitrationDecision(motor);
}


void Motor_OnHardLimit(void* payload) {
    MotorLimitEvent_t* ev = (MotorLimitEvent_t*)payload;
    DeviceNode_t* node = DeviceManager_Get(DEV_MOTOR_ID);
    if (!node || !node->private_data) return;

    MotorDevice_t* motor = (MotorDevice_t*)node->private_data;

    MotorDeviceId_t id = (ev->dir == DIR_FWD) ? DEV_ID_LIMIT_FWD : DEV_ID_LIMIT_REV;

    if (ev->is_active) {
        MotorControlCommand_t cmd = {
            .device_id = id,
            .timestamp = tickTimer_GetCount()
        };

        if (ev->dir == DIR_FWD) {
            cmd.type = CMD_TYPE_BLOCK_FWD;
            Motor_CmdList_SetBlock(&motor->block_fwd, cmd);
        } else {
            cmd.type = CMD_TYPE_BLOCK_REV;
            Motor_CmdList_SetBlock(&motor->block_rev, cmd);
        }
    } else {
        if (ev->dir == DIR_FWD) {
            Motor_CmdList_Remove(&motor->block_fwd, id);
        } else {
            Motor_CmdList_Remove(&motor->block_rev, id);
        }
    }

    Motor_ArbitrationDecision(motor);
}

void Motor_OnPowerEvent(void* payload) {
    MotorPowerEvent_t* ev = (MotorPowerEvent_t*)payload;
    MAIN_D("[MOTOR] OnPowerEvent: power_id=%d, is_on=%d\r\n", ev->power_id, ev->is_on);

    DeviceNode_t* node = DeviceManager_Get(DEV_MOTOR_ID);
    if (!node || !node->private_data) return;

    MotorDevice_t* motor = (MotorDevice_t*)node->private_data;

#if MOTOR_MODE_BIPOLAR
    MotorDeviceId_t id = (ev->power_id == 0) ? DEV_ID_POWER_POS : DEV_ID_POWER_NEG;
    const MotorDeviceGene_t* gene = Motor_GetGene(id);
    if (!gene) return;

    MotorControlCommand_t cmd = {
        .device_id = id,
        .priority = gene->priority,
        .param = 85.0f,
        .timestamp = tickTimer_GetCount()
    };

    if (ev->is_on) {
        if (id == DEV_ID_POWER_POS) {
            cmd.type = CMD_TYPE_RUN_FWD;
            Motor_CmdList_SetAllow(&motor->allow_fwd, cmd);
            MAIN_D("[MOTOR] Bipolar: POWER_POS ON, added allow_fwd\r\n");
        } else {
            cmd.type = CMD_TYPE_RUN_REV;
            Motor_CmdList_SetAllow(&motor->allow_rev, cmd);
            MAIN_D("[MOTOR] Bipolar: POWER_NEG ON, added allow_rev\r\n");
        }
    } else {
        if (id == DEV_ID_POWER_POS) {
            Motor_CmdList_Remove(&motor->allow_fwd, id);
            MAIN_D("[MOTOR] Bipolar: POWER_POS OFF, removed allow_fwd\r\n");
        } else {
            Motor_CmdList_Remove(&motor->allow_rev, id);
            MAIN_D("[MOTOR] Bipolar: POWER_NEG OFF, removed allow_rev\r\n");
        }
    }
#else
    MAIN_D("[MOTOR] Unipolar mode: power_id=%d, is_on=%d - power device not involved in arbitration\r\n",
           ev->power_id, ev->is_on);
#endif

    Motor_ArbitrationDecision(motor);
}

void Motor_OnManualIO(void* payload) {
    MotorManualIOEvent_t* ev = (MotorManualIOEvent_t*)payload;
    
    if (ev == NULL) {
        MAIN_D("[MOTOR] OnManualIO: NULL payload!\r\n");
        return;
    }
    
    uint32_t speed_raw = *(uint32_t*)&ev->speed;
    MAIN_D("[MOTOR] OnManualIO RAW: dir=%d, type=%d, speed_raw=0x%08X\r\n", 
           ev->dir, ev->type, speed_raw);
    
    DeviceNode_t* node = DeviceManager_Get(DEV_MOTOR_ID);
    if (!node || !node->private_data) {
        MAIN_D("[MOTOR] OnManualIO: Device not found!\r\n");
        return;
    }

    MotorDevice_t* motor = (MotorDevice_t*)node->private_data;

    MotorDeviceId_t io_dev_id = DEV_ID_NONE;
    
    if (ev->dir == DIR_FWD) {
        io_dev_id = DEV_ID_IO_FWD;
    } else if (ev->dir == DIR_REV) {
        io_dev_id = DEV_ID_IO_REV;
    } else if (ev->dir == DIR_NONE) {
        io_dev_id = DEV_ID_NONE;
    } else {
        MAIN_D("[MOTOR] OnManualIO: Invalid dir=%d, expected 0,1,2\r\n", ev->dir);
        return;
    }

    const MotorDeviceGene_t* gene = NULL;
    if (io_dev_id != DEV_ID_NONE) {
        gene = Motor_GetGene(io_dev_id);
        if (!gene) {
            MAIN_D("[MOTOR] OnManualIO: gene not found for dev_id=%d!\r\n", io_dev_id);
            return;
        }
    }
    
    int32_t speed_int = (int32_t)(ev->speed * 10);
    MAIN_D("[MOTOR] OnManualIO: dir=%d, type=%d, speed=%ld.%ld%%, io_dev_id=%d\r\n",
           ev->dir, ev->type, (long)(speed_int / 10), (long)(speed_int % 10), io_dev_id);
    MAIN_D("[MOTOR] OnManualIO: Before - allow_fwd.count=%d, allow_rev.count=%d, block_fwd.count=%d, block_rev.count=%d\r\n",
           motor->allow_fwd.count, motor->allow_rev.count, motor->block_fwd.count, motor->block_rev.count);

    if (ev->type == CMD_TYPE_RUN_FWD || ev->type == CMD_TYPE_RAMP_FWD) {
        if (io_dev_id != DEV_ID_IO_FWD) {
            MAIN_D("[MOTOR] ERROR: RUN_FWD but io_dev_id=%d (expected %d)\r\n", io_dev_id, DEV_ID_IO_FWD);
            return;
        }
        
        MotorControlCommand_t cmd = {
            .device_id = io_dev_id,
            .priority = gene->priority,
            .type = ev->type,
            .param = ev->speed,
            .timestamp = tickTimer_GetCount()
        };

        Motor_CmdList_Remove(&motor->block_fwd, io_dev_id);
        Motor_CmdList_SetAllow(&motor->allow_fwd, cmd);
        
        MAIN_D("[MOTOR] IO_FWD: RUN, removed block_fwd, added allow_fwd\r\n");
    }
    else if (ev->type == CMD_TYPE_RUN_REV || ev->type == CMD_TYPE_RAMP_REV) {
        if (io_dev_id != DEV_ID_IO_REV) {
            MAIN_D("[MOTOR] ERROR: RUN_REV but io_dev_id=%d (expected %d)\r\n", io_dev_id, DEV_ID_IO_REV);
            return;
        }
        
        MotorControlCommand_t cmd = {
            .device_id = io_dev_id,
            .priority = gene->priority,
            .type = ev->type,
            .param = ev->speed,
            .timestamp = tickTimer_GetCount()
        };

        Motor_CmdList_Remove(&motor->block_rev, io_dev_id);
        Motor_CmdList_SetAllow(&motor->allow_rev, cmd);
        
        MAIN_D("[MOTOR] IO_REV: RUN, removed block_rev, added allow_rev\r\n");
    }
    else if (ev->type == CMD_TYPE_STOP) {
        if (ev->dir == DIR_FWD) {
            if (gene == NULL) {
                gene = Motor_GetGene(DEV_ID_IO_FWD);
                if (!gene) {
                    MAIN_D("[MOTOR] OnManualIO: gene not found for IO_FWD!\r\n");
                    return;
                }
            }
            
            Motor_CmdList_Remove(&motor->allow_fwd, DEV_ID_IO_FWD);
            
            MotorControlCommand_t block_cmd = {
                .device_id = DEV_ID_IO_FWD,
                .priority = gene->priority,
                .type = CMD_TYPE_BLOCK_FWD,
                .timestamp = tickTimer_GetCount()
            };
            Motor_CmdList_SetBlock(&motor->block_fwd, block_cmd);
            
            MAIN_D("[MOTOR] IO_FWD: STOP, removed allow_fwd, added block_fwd\r\n");
        }
        else if (ev->dir == DIR_REV) {
            if (gene == NULL) {
                gene = Motor_GetGene(DEV_ID_IO_REV);
                if (!gene) {
                    MAIN_D("[MOTOR] OnManualIO: gene not found for IO_REV!\r\n");
                    return;
                }
            }
            
            Motor_CmdList_Remove(&motor->allow_rev, DEV_ID_IO_REV);
            
            MotorControlCommand_t block_cmd = {
                .device_id = DEV_ID_IO_REV,
                .priority = gene->priority,
                .type = CMD_TYPE_BLOCK_REV,
                .timestamp = tickTimer_GetCount()
            };
            Motor_CmdList_SetBlock(&motor->block_rev, block_cmd);
            
            MAIN_D("[MOTOR] IO_REV: STOP, removed allow_rev, added block_rev\r\n");
        }
        else {
            const MotorDeviceGene_t* gene_fwd = Motor_GetGene(DEV_ID_IO_FWD);
            const MotorDeviceGene_t* gene_rev = Motor_GetGene(DEV_ID_IO_REV);
            
            Motor_CmdList_Remove(&motor->allow_fwd, DEV_ID_IO_FWD);
            Motor_CmdList_Remove(&motor->allow_rev, DEV_ID_IO_REV);
            
            if (gene_fwd) {
                MotorControlCommand_t block_fwd_cmd = {
                    .device_id = DEV_ID_IO_FWD,
                    .priority = gene_fwd->priority,
                    .type = CMD_TYPE_BLOCK_FWD,
                    .timestamp = tickTimer_GetCount()
                };
                Motor_CmdList_SetBlock(&motor->block_fwd, block_fwd_cmd);
            }
            
            if (gene_rev) {
                MotorControlCommand_t block_rev_cmd = {
                    .device_id = DEV_ID_IO_REV,
                    .priority = gene_rev->priority,
                    .type = CMD_TYPE_BLOCK_REV,
                    .timestamp = tickTimer_GetCount()
                };
                Motor_CmdList_SetBlock(&motor->block_rev, block_rev_cmd);
            }
            
            MAIN_D("[MOTOR] All IO: STOP, removed all allow, added all block\r\n");
        }
    }
    else {
        MAIN_D("[MOTOR] OnManualIO: Unknown type=%d\r\n", ev->type);
        return;
    }

    Motor_ArbitrationDecision(motor);
}

void Motor_OnCANEvent(void* payload) {
    (void)payload;
}

// ========== 电机霍尔转速反馈回调 ==========
void Motor_OnSpeedFeedback(void* payload) {
    int32_t* speed_rpm = (int32_t*)payload;

    DeviceNode_t* node = DeviceManager_Get(DEV_MOTOR_ID);
    if (!node || !node->private_data) return;

    MotorDevice_t* motor = (MotorDevice_t*)node->private_data;

    if (motor->state == MS_RUNNING || motor->state == MS_RAMPING) {
        (void)motor;
        (void)speed_rpm;
    }
}

// ========== 标准设备操作接口实现 ==========
DeviceResult_t Motor_Init(void* handle) {
    MotorDevice_t* motor = (MotorDevice_t*)handle;
    if (!motor) return RESULT_PARAM_ERR;

    memset(motor, 0, sizeof(MotorDevice_t));
    
    // 初始化方向跟踪变量
    s_eLastArbitrationDir = DIR_NONE;
    s_u16LastTargetDuty = 0;
    s_bStopPolarityPending = false;

    // 初始化命令列表
    for(int i = 0; i < MAX_COMMANDS_PER_DIRECTION; i++) {
        motor->allow_fwd.commands[i].device_id = DEV_ID_NONE;
        motor->allow_fwd.commands[i].priority = PRIO_NONE;
        motor->allow_rev.commands[i].device_id = DEV_ID_NONE;
        motor->allow_rev.commands[i].priority = PRIO_NONE;
        motor->block_fwd.commands[i].device_id = DEV_ID_NONE;
        motor->block_rev.commands[i].device_id = DEV_ID_NONE;
    }

    // 初始化IO设备默认状态
    MotorControlCommand_t io_fwd_block = {
        .device_id = DEV_ID_IO_FWD,
        .priority = MANUAL_PRIORITY,
        .type = CMD_TYPE_BLOCK_FWD,
        .timestamp = tickTimer_GetCount()
    };
    Motor_CmdList_SetBlock(&motor->block_fwd, io_fwd_block);
    
    MotorControlCommand_t io_rev_block = {
        .device_id = DEV_ID_IO_REV,
        .priority = MANUAL_PRIORITY,
        .type = CMD_TYPE_BLOCK_REV,
        .timestamp = tickTimer_GetCount()
    };
    Motor_CmdList_SetBlock(&motor->block_rev, io_rev_block);

    motor->enable = 1;
    motor->last_arbitration_time = tickTimer_GetCount();

    // 设置调试全局指针
    g_pMotor_Dev_Watch = motor;
    
    // 初始化调试变量
    Motor_UpdateDebugInfo(motor);

    MOTOR_DEBUG("Motor_Init completed, g_pMotor_Dev_Watch set\r\n");
    return RESULT_OK;
}

DeviceResult_t Motor_Deinit(void* handle) {
    MotorDevice_t* motor = (MotorDevice_t*)handle;
    if (!motor) return RESULT_PARAM_ERR;

    motor->enable = 0;
    Motor_EmergencyStop(motor);
    
    g_pMotor_Dev_Watch = NULL;
    memset((void*)&g_stcMotorDebug, 0, sizeof(MotorDebugInfo_t));

    return RESULT_OK;
}

DeviceResult_t Motor_Read(void* handle, void* data, uint32_t size) {
    MotorDevice_t* motor = (MotorDevice_t*)handle;
    if (!motor || !data) return RESULT_PARAM_ERR;
    if (!motor->enable) return RESULT_ERROR;

    if (size == sizeof(MotorDebugInfo_t)) {
        memcpy(data, &motor->debug_info, sizeof(MotorDebugInfo_t));
        return RESULT_OK;
    }
    else if (size == sizeof(Motor_StateInfo_t)) {
        Motor_StateInfo_t* pstcState = (Motor_StateInfo_t*)data;
        pstcState->desired_dir = DIR_NONE;
        if (motor->state == MS_RUNNING || motor->state == MS_RAMPING) {
            pstcState->desired_dir = motor->active_dir;
        }
        pstcState->state = motor->state;
        pstcState->active_dir = motor->active_dir;
        pstcState->current_duty = motor->current_duty;
        pstcState->enable = motor->enable;
        return RESULT_OK;
    }

    return RESULT_PARAM_ERR;
}

DeviceResult_t Motor_Write(void* handle, const void* data, uint32_t size) {
    (void)handle;
    (void)data;
    (void)size;
    return RESULT_ERROR;
}

DeviceResult_t Motor_Control(void* handle, DeviceCommandData_t* cmd) {
    MotorDevice_t* motor = (MotorDevice_t*)handle;
    if (!motor || !cmd) return RESULT_PARAM_ERR;

    switch(cmd->cmd) {
        case CMD_MOTOR_STOP:
            Motor_Stop(motor);
            break;
        case CMD_MOTOR_RUN_FWD:
            Motor_Start(motor, DIR_FWD);
            break;
        case CMD_MOTOR_RUN_REV:
            Motor_Start(motor, DIR_REV);
            break;
        case CMD_MOTOR_SET_SPEED:
            if (cmd->params && cmd->param_size == sizeof(float)) {
                Motor_SetSpeed(motor, *(float*)cmd->params);
            }
            break;
        case CMD_MOTOR_EMERGENCY_STOP:
            Motor_EmergencyStop(motor);
            break;
        case CMD_MOTOR_GET_DESIRED_DIR:
            if (cmd->response && cmd->response_size >= sizeof(uint8_t)) {
                MotorDir_t desired_dir = DIR_NONE;
                if (motor->state == MS_RUNNING || motor->state == MS_RAMPING) {
                    desired_dir = motor->active_dir;
                }
                *(uint8_t*)cmd->response = (uint8_t)desired_dir;
                return RESULT_OK;
            }
            return RESULT_PARAM_ERR;
        default:
            return RESULT_ERROR;
    }

    return RESULT_OK;
}

DeviceResult_t Motor_Update(void* handle) {
    MotorDevice_t* motor = (MotorDevice_t*)handle;
    if (!motor || !motor->enable) return RESULT_ERROR;

    uint32_t now = tickTimer_GetCount();

    // 每50ms仲裁一次
    if (now - motor->last_arbitration_time >= 50) {
        Motor_ArbitrationDecision(motor);
        motor->last_arbitration_time = now;
    }

    // ========== 每500ms打印一次电机仲裁状态 ==========
    if (now - s_u32LastMotorPrintTime >= MOTOR_PRINT_INTERVAL_MS) {
        s_u32LastMotorPrintTime = now;

        MOTOR_OUT("=== Motor Arbitration State ===\r\n");

        const char* state_str = (motor->state == MS_IDLE) ? "IDLE" :
                                (motor->state == MS_RAMPING) ? "RAMPING" :
                                (motor->state == MS_RUNNING) ? "RUNNING" : "LOCKED";
        const char* dir_str = (motor->active_dir == DIR_FWD) ? "FWD" :
                              (motor->active_dir == DIR_REV) ? "REV" : "NONE";
        MOTOR_OUT("  State=%s, ActiveDir=%s, Duty=%.1f%%\r\n",
                  state_str, dir_str, motor->current_duty);

        MOTOR_OUT("  Active Allow FWD: ");
        if (motor->allow_fwd.count > 0) {
            for (int i = 0; i < motor->allow_fwd.count; i++) {
                const char* dev_name = "";
                switch(motor->allow_fwd.commands[i].device_id) {
                    case DEV_ID_IO_FWD: dev_name = "IO_FWD"; break;
                    case DEV_ID_IO_REV: dev_name = "IO_REV"; break;
                    case DEV_ID_CAN: dev_name = "CAN"; break;
                    case DEV_ID_POWER_POS: dev_name = "POWER_POS"; break;
                    case DEV_ID_POWER_NEG: dev_name = "POWER_NEG"; break;
                    default: dev_name = "UNKNOWN"; break;
                }
                MOTOR_OUT("%s(Prio%d) ", dev_name, motor->allow_fwd.commands[i].priority);
            }
        } else {
            MOTOR_OUT("(none)");
        }
        MOTOR_OUT("\r\n");

        MOTOR_OUT("  Active Allow REV: ");
        if (motor->allow_rev.count > 0) {
            for (int i = 0; i < motor->allow_rev.count; i++) {
                const char* dev_name = "";
                switch(motor->allow_rev.commands[i].device_id) {
                    case DEV_ID_IO_FWD: dev_name = "IO_FWD"; break;
                    case DEV_ID_IO_REV: dev_name = "IO_REV"; break;
                    case DEV_ID_CAN: dev_name = "CAN"; break;
                    case DEV_ID_POWER_POS: dev_name = "POWER_POS"; break;
                    case DEV_ID_POWER_NEG: dev_name = "POWER_NEG"; break;
                    default: dev_name = "UNKNOWN"; break;
                }
                MOTOR_OUT("%s(Prio%d) ", dev_name, motor->allow_rev.commands[i].priority);
            }
        } else {
            MOTOR_OUT("(none)");
        }
        MOTOR_OUT("\r\n");

        MOTOR_OUT("  Active Block FWD: ");
        if (motor->block_fwd.count > 0) {
            for (int i = 0; i < motor->block_fwd.count; i++) {
                const char* dev_name = "";
                switch(motor->block_fwd.commands[i].device_id) {
                    case DEV_ID_IO_FWD: dev_name = "IO_FWD"; break;
                    case DEV_ID_IO_REV: dev_name = "IO_REV"; break;
                    case DEV_ID_CAN: dev_name = "CAN"; break;
                    case DEV_ID_LIMIT_FWD: dev_name = "LIMIT_FWD"; break;
                    case DEV_ID_LIMIT_REV: dev_name = "LIMIT_REV"; break;
                    case DEV_ID_POWER_POS: dev_name = "POWER_POS"; break;
                    case DEV_ID_POWER_NEG: dev_name = "POWER_NEG"; break;
                    case DEV_ID_RTURN_FWD: dev_name = "RTURN_FWD"; break;
                    case DEV_ID_RTURN_REV: dev_name = "RTURN_REV"; break;
                    case DEV_ID_OVERVOLTAGE_FWD: dev_name = "OVERVOLTAGE_FWD"; break;
                    case DEV_ID_OVERVOLTAGE_REV: dev_name = "OVERVOLTAGE_REV"; break;
                    case DEV_ID_UNDERVOLTAGE_FWD: dev_name = "UNDERVOLTAGE_FWD"; break;
                    case DEV_ID_UNDERVOLTAGE_REV: dev_name = "UNDERVOLTAGE_REV"; break;
                    case DEV_ID_OVERCUR_FWD: dev_name = "OVERCUR_FWD"; break;
                    default: dev_name = "UNKNOWN"; break;
                }
                MOTOR_OUT("%s ", dev_name);
            }
        } else {
            MOTOR_OUT("(none)");
        }
        MOTOR_OUT("\r\n");

        MOTOR_OUT("  Active Block REV: ");
        if (motor->block_rev.count > 0) {
            for (int i = 0; i < motor->block_rev.count; i++) {
                const char* dev_name = "";
                switch(motor->block_rev.commands[i].device_id) {
                    case DEV_ID_IO_FWD: dev_name = "IO_FWD"; break;
                    case DEV_ID_IO_REV: dev_name = "IO_REV"; break;
                    case DEV_ID_CAN: dev_name = "CAN"; break;
                    case DEV_ID_LIMIT_FWD: dev_name = "LIMIT_FWD"; break;
                    case DEV_ID_LIMIT_REV: dev_name = "LIMIT_REV"; break;
                    case DEV_ID_POWER_POS: dev_name = "POWER_POS"; break;
                    case DEV_ID_POWER_NEG: dev_name = "POWER_NEG"; break;
                    case DEV_ID_RTURN_FWD: dev_name = "RTURN_FWD"; break;
                    case DEV_ID_RTURN_REV: dev_name = "RTURN_REV"; break;
                    case DEV_ID_OVERVOLTAGE_FWD: dev_name = "OVERVOLTAGE_FWD"; break;
                    case DEV_ID_OVERVOLTAGE_REV: dev_name = "OVERVOLTAGE_REV"; break;
                    case DEV_ID_UNDERVOLTAGE_FWD: dev_name = "UNDERVOLTAGE_FWD"; break;
                    case DEV_ID_UNDERVOLTAGE_REV: dev_name = "UNDERVOLTAGE_REV"; break;
                    case DEV_ID_OVERCUR_FWD: dev_name = "OVERCUR_FWD"; break;
                    default: dev_name = "UNKNOWN"; break;
                }
                MOTOR_OUT("%s ", dev_name);
            }
        } else {
            MOTOR_OUT("(none)");
        }
        MOTOR_OUT("\r\n");

        // 检查基因表中的所有设备
        MOTOR_OUT("  Registered but Inactive Devices:\r\n");
        bool has_inactive = false;
        
        struct { uint8_t id; const char* name; } all_devices[] = {
            {DEV_ID_POWER_POS, "POWER_POS"},
            {DEV_ID_POWER_NEG, "POWER_NEG"},
            {DEV_ID_LIMIT_FWD, "LIMIT_FWD"},
            {DEV_ID_LIMIT_REV, "LIMIT_REV"},
            {DEV_ID_RTURN_FWD, "RTURN_FWD"},
            {DEV_ID_RTURN_REV, "RTURN_REV"},
            {DEV_ID_CAN, "CAN"},
            {DEV_ID_IO_FWD, "IO_FWD"},
            {DEV_ID_IO_REV, "IO_REV"},
            {DEV_ID_EMERGENCY, "EMERGENCY"}
        };
        
        uint8_t active_devices[16] = {0};
        uint8_t active_count = 0;
        
        for (int i = 0; i < motor->allow_fwd.count; i++) {
            uint8_t dev_id = motor->allow_fwd.commands[i].device_id;
            bool found = false;
            for (int j = 0; j < active_count; j++) {
                if (active_devices[j] == dev_id) { found = true; break; }
            }
            if (!found) active_devices[active_count++] = dev_id;
        }
        
        for (int i = 0; i < motor->allow_rev.count; i++) {
            uint8_t dev_id = motor->allow_rev.commands[i].device_id;
            bool found = false;
            for (int j = 0; j < active_count; j++) {
                if (active_devices[j] == dev_id) { found = true; break; }
            }
            if (!found) active_devices[active_count++] = dev_id;
        }
        
        for (int i = 0; i < motor->block_fwd.count; i++) {
            uint8_t dev_id = motor->block_fwd.commands[i].device_id;
            bool found = false;
            for (int j = 0; j < active_count; j++) {
                if (active_devices[j] == dev_id) { found = true; break; }
            }
            if (!found) active_devices[active_count++] = dev_id;
        }
        
        for (int i = 0; i < motor->block_rev.count; i++) {
            uint8_t dev_id = motor->block_rev.commands[i].device_id;
            bool found = false;
            for (int j = 0; j < active_count; j++) {
                if (active_devices[j] == dev_id) { found = true; break; }
            }
            if (!found) active_devices[active_count++] = dev_id;
        }
        
        for (int i = 0; i < sizeof(all_devices)/sizeof(all_devices[0]); i++) {
            bool is_active = false;
            for (int j = 0; j < active_count; j++) {
                if (active_devices[j] == all_devices[i].id) {
                    is_active = true;
                    break;
                }
            }
            if (!is_active) {
                MOTOR_OUT("    %s (ID=%d) - No command\r\n", 
                          all_devices[i].name, all_devices[i].id);
                has_inactive = true;
            }
        }
        
        if (!has_inactive) {
            MOTOR_OUT("    (All registered devices are active)\r\n");
        }
        
        MOTOR_OUT("  Conflict=%d\r\n",
                  motor->debug_info.conflict_fault);
        
        const char* active_dev_name = "NONE";
        switch(motor->active_device_id) {
            case DEV_ID_IO_FWD: active_dev_name = "IO_FWD"; break;
            case DEV_ID_IO_REV: active_dev_name = "IO_REV"; break;
            case DEV_ID_CAN: active_dev_name = "CAN"; break;
            case DEV_ID_POWER_POS: active_dev_name = "POWER_POS"; break;
            case DEV_ID_POWER_NEG: active_dev_name = "POWER_NEG"; break;
            case DEV_ID_LIMIT_FWD: active_dev_name = "LIMIT_FWD"; break;
            case DEV_ID_LIMIT_REV: active_dev_name = "LIMIT_REV"; break;
            case DEV_ID_OVERCUR_FWD: active_dev_name = "OVERCUR_FWD"; break;
            default: active_dev_name = "NONE"; break;
        }
        MOTOR_OUT("  Active Device: %s (ID=%d)\r\n", active_dev_name, motor->active_device_id);
    }
    
    return RESULT_OK;
}

// ========== 电机特定接口实现 ==========

void Motor_SetSpeed(MotorDevice_t* motor, float duty) {
    if (!motor || !motor->enable) return;
    (void)duty;
    // PWM 操作已由仲裁回调处理
}

void Motor_Start(MotorDevice_t* motor, MotorDir_t dir) {
    if (!motor || !motor->enable) return;
    
    MotorManualIOEvent_t ev = {
        .dir = dir,
        .type = CMD_TYPE_RUN_FWD,
        .speed = 50.0f
    };
    
    if (dir == DIR_FWD) {
        ev.type = CMD_TYPE_RUN_FWD;
    } else if (dir == DIR_REV) {
        ev.type = CMD_TYPE_RUN_REV;
    }
    
    EventBus_Publish(TOPIC_MANUAL_IO, &ev);
}

void Motor_Stop(MotorDevice_t* motor) {
    if (!motor) return;
    
    MotorManualIOEvent_t ev = {
        .dir = DIR_NONE,
        .type = CMD_TYPE_STOP,
        .speed = 0
    };
    
    EventBus_Publish(TOPIC_MANUAL_IO, &ev);
}

void Motor_EmergencyStop(MotorDevice_t* motor) {
    if (!motor) return;
    
    MOTOR_DEBUG("EmergencyStop: clearing all allow commands\r\n");
    
    for (int i = 0; i < MAX_COMMANDS_PER_DIRECTION; i++) {
        motor->allow_fwd.commands[i].device_id = DEV_ID_NONE;
        motor->allow_fwd.commands[i].priority = PRIO_NONE;
        motor->allow_fwd.commands[i].type = CMD_TYPE_NONE_USE;
        motor->allow_fwd.commands[i].param = 0.0f;
        motor->allow_fwd.commands[i].timestamp = 0;
    }
    motor->allow_fwd.count = 0;
    
    for (int i = 0; i < MAX_COMMANDS_PER_DIRECTION; i++) {
        motor->allow_rev.commands[i].device_id = DEV_ID_NONE;
        motor->allow_rev.commands[i].priority = PRIO_NONE;
        motor->allow_rev.commands[i].type = CMD_TYPE_NONE_USE;
        motor->allow_rev.commands[i].param = 0.0f;
        motor->allow_rev.commands[i].timestamp = 0;
    }
    motor->allow_rev.count = 0;
    
    g_u8MotorDebug_AllowFwdCount = 0;
    g_u8MotorDebug_AllowRevCount = 0;
    
    Motor_ArbitrationDecision(motor);
}

void Motor_ClearAllowFwd(MotorDevice_t* motor) {
    if (!motor) return;
    
    MOTOR_DEBUG("ClearAllowFwd: clearing all allow_fwd commands\r\n");
    
    for (int i = 0; i < MAX_COMMANDS_PER_DIRECTION; i++) {
        motor->allow_fwd.commands[i].device_id = DEV_ID_NONE;
        motor->allow_fwd.commands[i].priority = PRIO_NONE;
        motor->allow_fwd.commands[i].type = CMD_TYPE_NONE_USE;
        motor->allow_fwd.commands[i].param = 0.0f;
        motor->allow_fwd.commands[i].timestamp = 0;
    }
    motor->allow_fwd.count = 0;
    
    g_u8MotorDebug_AllowFwdCount = 0;
    
    Motor_ArbitrationDecision(motor);
}

void Motor_ClearAllowRev(MotorDevice_t* motor) {
    if (!motor) return;
    
    MOTOR_DEBUG("ClearAllowRev: clearing all allow_rev commands\r\n");
    
    for (int i = 0; i < MAX_COMMANDS_PER_DIRECTION; i++) {
        motor->allow_rev.commands[i].device_id = DEV_ID_NONE;
        motor->allow_rev.commands[i].priority = PRIO_NONE;
        motor->allow_rev.commands[i].type = CMD_TYPE_NONE_USE;
        motor->allow_rev.commands[i].param = 0.0f;
        motor->allow_rev.commands[i].timestamp = 0;
    }
    motor->allow_rev.count = 0;
    
    g_u8MotorDebug_AllowRevCount = 0;
    
    Motor_ArbitrationDecision(motor);
}

const MotorDebugInfo_t* Motor_GetDebugInfo(MotorDevice_t* motor) {
    if (!motor) return NULL;
    return &motor->debug_info;
}

MotorDir_t Motor_GetDesiredDirection(MotorDevice_t* motor) {
    if (!motor) return DIR_NONE;
    if (motor->state == MS_RUNNING || motor->state == MS_RAMPING) {
        return motor->active_dir;
    }
    return DIR_NONE;
}

// ========== 过流告警手动清除接口 ==========
void Motor_ClearOvercurrentBlock(MotorDevice_t* motor) {
    if (!motor) return;
    
    MOTOR_DEBUG("ClearOvercurrentBlock: removing block_fwd for DEV_ID_OVERCUR_FWD\r\n");
    
    Motor_CmdList_Remove(&motor->block_fwd, DEV_ID_OVERCUR_FWD);
    
    Motor_ArbitrationDecision(motor);
}

// ========== 电压告警手动清除接口 ==========
void Motor_ClearVoltageBlock(MotorDevice_t* motor, uint8_t u8AlarmType) {
    if (!motor) return;
    
    MotorDeviceId_t dev_id_fwd, dev_id_rev;
    const char* alarm_name;
    
    if (u8AlarmType == VOLTAGE_ALARM_OVERVOLTAGE) {
        dev_id_fwd = DEV_ID_OVERVOLTAGE_FWD;
        dev_id_rev = DEV_ID_OVERVOLTAGE_REV;
        alarm_name = "OVERVOLTAGE";
    } else if (u8AlarmType == VOLTAGE_ALARM_UNDERVOLTAGE) {
        dev_id_fwd = DEV_ID_UNDERVOLTAGE_FWD;
        dev_id_rev = DEV_ID_UNDERVOLTAGE_REV;
        alarm_name = "UNDERVOLTAGE";
    } else {
        return;
    }
    
    MOTOR_DEBUG("ClearVoltageBlock: %s - removing block_fwd/rev\r\n", alarm_name);
    
    Motor_CmdList_Remove(&motor->block_fwd, dev_id_fwd);
    Motor_CmdList_Remove(&motor->block_rev, dev_id_rev);
    
    Motor_ArbitrationDecision(motor);
}