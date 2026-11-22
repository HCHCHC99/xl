/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-01-28     gylocal       the first version
 */
#ifndef APPLICATIONS_APP_H_
#define APPLICATIONS_APP_H_


#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <rtdef.h>
#include <stdbool.h>

#define POWER_ERROR_VOLTAGE_EVENT        ( 0x01 << 1 )     /* 电源错误 事件 */
#define POWER_ERROR_OVERVOLTAGE_EVENT    ( 0x01 << 2 )     /* 电源错误 事件  过压*/
#define POWER_ERROR_UNDERVOLTAGE_EVENT   ( 0x01 << 3 )     /* 电源错误 事件  欠压*/
#define POWER_ERROR_CURRENT_EVENT        ( 0x01 << 4 )     /* 电源错误 事件  包含：过流*/
#define POWER_ERROR_DOWNVOLTAGE_EVENT    ( 0x01 << 5 )     /* 电源错误 事件  掉电*/

#define SYS_OVER_HEATING_EVENT           ( 0x01 << 0 )     /* 系统 过热 事件 */
#define SYS_CALIBRATED_EVENT             ( 0x01 << 1 )     /* 系统 已校准 事件 */
#define SYS_UNCALIBRATED_EVENT           ( 0x01 << 2 )     /* 系统 未校准 事件 */

#define SYS_CALIBRATED_FIRST_RUN_EVENT   ( 0x01 << 3 )     /* 系统 第一次运行已校准，校准磁编码器，但是不校准系统 事件 */
#define SYS_CAN_ERR_EVENT                ( 0x01 << 4 )     /* can 通信错误 事件*/
#define SYS_MOTOR_ENCODER_ERR_EVENT      ( 0x01 << 5 )     /* MOTOR ENCODER 错误 事件*/

#define SYS_WORK_PORCESS_STOP_EVENT      ( 0x01 << 6 )     /* 工作过程 停止 事件*/
#define SYS_WORK_PORCESS_LEFT_EVENT      ( 0x01 << 7 )     /* 工作过程 左转 事件*/
#define SYS_WORK_PORCESS_RIGHT_EVENT     ( 0x01 << 8 )     /* 工作过程 右转 事件*/
#define SYS_WORK_PORCESS_OPEN_EVENT      ( 0x01 << 9 )     /* 工作过程 开路 事件*/
#define SYS_WORK_PORCESS_LOCKED_EVENT    ( 0x01 << 10 )    /* 工作过程 堵转 事件*/

extern struct rt_messagequeue     mq_cmd_CAN ;            /* 系统消息队列 CAN消息命令 */;
extern struct rt_messagequeue     mq_abs_pos ;            /* 系统消息队列  绝对位置信息 */;
extern struct rt_messagequeue     mq_power_voltage_state;        /* 系统消息队列  电源状态     错误状态包含：欠压、过压 */
extern struct rt_messagequeue     mq_power_current_state;        /* 系统消息队列  电源状态     错误状态包含：过流 */
extern struct rt_messagequeue     mq_arm_abs_angle;              /* arm 角度 */
extern struct rt_messagequeue mq_CAN_to_Axis;

extern struct rt_event  system_event ;
extern struct rt_event  power_voltage_event;
extern struct rt_event  power_current_event;

extern struct rt_mutex power_mutex_error ;

extern struct rt_mutex power_mutex_voltage_error ;
extern struct rt_mutex power_mutex_current_error ;


extern void clear_event_flag(rt_event_t event, rt_uint32_t clear_flags);

typedef enum
{
    SYSTEM_MQ_CAN = 0,
    SYSTEM_MQ_KEY,
    SYSTEM_MQ_LED,
    SYSTEM_MQ_DOOR,
    SYSTEM_MQ_BLINDS,
    SYSTEM_EVENT,
    SYSTEM_MAX
}SYSTEM_IPC_E;

extern void* system_ipc_arr[SYSTEM_MAX];



/* 声明外部 线程句柄 */
extern struct rt_thread can_master_thread;
extern struct rt_thread can_received_thread;
extern struct rt_thread can2_received_thread;
extern struct rt_thread key_thread;
extern struct rt_thread led_thread;
extern struct rt_thread pwm_thread;
extern struct rt_thread door_thread;
extern struct rt_thread blinds_thread;
extern struct rt_thread lock_thread;
extern struct rt_thread data_thread;
extern struct rt_thread monitor_thread;
extern struct rt_thread monitor_thread;
extern struct rt_thread daemon_thread;



//extern rt_uint8_t can_send_thread_stack[1024];
//extern struct rt_thread can_analysis_thread;
//extern struct rt_thread key_thread;
//extern struct rt_thread led_thread;
//extern struct rt_thread door_thread;
//extern struct rt_thread data_thread;
//extern struct rt_thread monitor_thread;
//extern struct rt_thread daemon_thread;


// 线程监测信息数组，按照不同线程ID初始化各线程及其心跳定时器相关信息
// 这里使用静态内存分配方式，提前定义好栈空间等资源
typedef enum
{
    CAN_MASTER_THREAD_ID = 0x00,
    CAN_RECEIVED_THREAD_ID,
    CAN_SEND_THREAD_ID,
    ABS_ANGLE_THREAD_ID,
    BLDC_CTRL_THREAD_ID,
    POWER_THREAD_ID,
    THREAD_ID_MAX
}THREAD_ID_T;

typedef struct
{
    struct rt_thread target_thread;
    const char       *name;
    void (*entry)(void *parameter);
    void             *parameter;
    void             *stack_start;
    rt_uint32_t       stack_size;
    rt_uint8_t        priority;
    rt_uint32_t       tick;
    rt_timer_t        heartbeat_timer;
} THREAD_MONITOR_INFO;

extern THREAD_MONITOR_INFO thread_monitor_info[THREAD_ID_MAX];

extern void clear_event_flag(rt_event_t event, rt_uint32_t clear_flags);


/************************************************* 系统定义*****************************/

//   系统配置  定义 start  *******************************************************************
typedef enum {
    CONFIG_ID_CAN_ID = 0,
    CONFIG_ID_CAN_BAUDRATE,
    CONFIG_ID_SOFT_START_RAMP,
    CONFIG_ID_SOFT_START_TIME,
    CONFIG_ID_SOFT_STOP_RAMP,
    CONFIG_ID_SOFT_STOP_TIME,
    CONFIG_ID_FIXED_SPEED,
    CONFIG_ID_OVERCURRENT_THR,
    CONFIG_ID_OVERCURRENT_TIME,
    CONFIG_ID_UNDERVOLTAGE_THR,
    CONFIG_ID_UNDERVOLTAGE_TIME,
    CONFIG_ID_OVERVOLTAGE_THR,
    CONFIG_ID_OVERVOLTAGE_TIME,
    CONFIG_ID_ZERO_OFFSET,
    CONFIG_ID_MAX
} CONFIG_ID_TYPE_DEF;

typedef struct {
    // CAN配置
    uint8_t  can_protocol_version; // CAN协议版本
    uint16_t can_baudrate;        // CAN波特率
    uint32_t can_id;              // CAN通信ID

    // 电机控制配置
    uint16_t soft_start_ramp;     // 缓启动斜坡(ms/°)
    uint16_t soft_start_time;     // 缓启动总时间(ms)
    uint16_t soft_stop_ramp;      // 缓停止斜坡(ms/°)
    uint16_t soft_stop_time;      // 缓停止总时间(ms)
    uint16_t fixed_speed;         // 定速运行速度(°/s)

    // 保护配置
    uint16_t overcurrent_thr;     // 过流阈值(mA)
    uint16_t overcurrent_time;    // 过流触发时间(ms)
    uint16_t undervoltage_thr;    // 欠压阈值(mV)
    uint16_t undervoltage_time;   // 欠压触发时间(ms)
    uint16_t overvoltage_thr;     // 过压阈值(mV)
    uint16_t overvoltage_time;    // 过压触发时间(ms)

    // 采样配置
    uint16_t adc_sample_interval; // ADC采样间隔(ms)
    uint16_t trigger_thr_algorithm; // 触发阈值算法编码

    // 校准配置
    float zero_offset_angle;      // 零点偏移角度
    uint32_t calibrate_timestamp; // 校准时间戳
} SYS_CONFIG_DEF;


//   系统配置 定义 end    *******************************************************************


//   电源 定义 start  *******************************************************************
typedef enum {
//    POWER_VOLTAGE_STATE_NULL = 0 ,
    POWER_VOLTAGE_STATE_NORMAL = 0,
    POWER_VOLTAGE_STATE_UNDERVOLTAGE,
    POWER_VOLTAGE_STATE_OVERVOLTAGE,
    POWER_VOLTAGE_STATE_DOWNVOLTAGE,
//    POWER_VOLTAGE_STATE_MAX
} POWER_VOLTAGE_STATUS_TYPE_DEF;

typedef enum {
    POWER_CURRENT_STATE_NULL = 0 ,
    POWER_CURRENT_STATE_NORMAL,
    POWER_CURRENT_STATE_OVERCURRENT,
    POWER_CURRENT_STATE_MAX
} POWER_CURRENT_STATUS_TYPE_DEF;

typedef struct {
    POWER_VOLTAGE_STATUS_TYPE_DEF voltage_state;  // 当前 电压状态
    POWER_CURRENT_STATUS_TYPE_DEF current_state;  // 当前 电流状态

    float32_t rated_voltage_mv;    // 额定电压(12000mV)
    float32_t downvoltage_thr;     // 掉电阈值( 7000mV)
    float32_t undervoltage_thr;    // 欠压阈值( 9000mV)
    float32_t overvoltage_thr;     // 过压阈值(18000mV)
    float32_t overcurrent_thr;     // 40A == 40000mA

    float32_t  voltage_gain;        // 电压放大倍数
    float32_t  current_gain;        // 电流放大倍数

    float32_t voltage_offset;     // 电压  偏移量
    float32_t current_offset;     // 电流 偏移量

    float32_t current_voltage;     // 当前电压(mV)
    rt_tick_t voltage_error_tick;
    float32_t current_current;     // 当前电流(A)
    rt_tick_t current_error_tick;

} POWER_DATA_DEF;

extern POWER_DATA_DEF power_data;
//   电源 定义 end    *******************************************************************


//   保护  定义 start  *******************************************************************
typedef enum {
    PROTECT_STATE_NORMAL = 0,
    PROTECT_STATE_OVERCURRENT,
    PROTECT_STATE_UNDERVOLTAGE,
    PROTECT_STATE_OVERVOLTAGE,
    PROTECT_STATE_LOCK_ROTOR,
    PROTECT_STATE_HALL_ERROR
} PROTECT_STATE_TYPE_DEF;

typedef struct {
    PROTECT_STATE_TYPE_DEF state;    // 保护状态
    uint8_t fault_level;             // 故障级别：0-警告 1-保护 2-紧急停机
    uint32_t bus_current;            // 母线电流(mA)
    uint32_t bus_voltage;            // 母线电压(mV)
    uint32_t fault_timestamp;        // 故障时间戳
} PROTECT_DATA_DEF;
//   保护  定义 end    *******************************************************************






//   PWM  定义 start  *******************************************************************

/* 【2026-10-27 修复 · D-60】原为裸定义 `struct rt_device_pwm *dev_pwm;`
 * 放在头文件里 —— 这是 C 语言的经典错误：任何包含 app.h 的 .c 都会
 * 生成一份 dev_pwm 的定义，全量编译时链接器报：
 *   multiple definition of `dev_pwm'
 *   （bsp_dev_pwm.o / bsp_dev_hall_uvw.o / bsp_drv_spi.o / bsp_mod_power.o ...）
 *
 * 之前没暴露是因为 Debug/ 下一直存在旧的 .o 缓存，从未触发全量重编。
 * 本次因修改 rtconfig.h（被所有文件包含）导致全量重编，缺陷才浮现。
 *
 * 正确做法：头文件只放 extern 声明，定义放在唯一的 .c 中。
 * 定义点：applications/bsp_dev_pwm.c（它是唯一给 dev_pwm 赋值的模块）。
 */
extern struct rt_device_pwm *dev_pwm;

#define DEV_PWM_NAME "pwm4"

#define DEV_PWM_CHAN_U_H  (6)
#define DEV_PWM_CHAN_U_L  (2)

#define DEV_PWM_CHAN_V_H  (3)
#define DEV_PWM_CHAN_V_L  (4)

#define DEV_PWM_CHAN_W_H  (5)
#define DEV_PWM_CHAN_W_L  (1)


//#define MOTOR_PWM_PERIOD   (1000000)              //周期值 100000ns 频率 10kHz  即100 us     启用rtt设备框架时，应用
//#define MOTOR_ON_PULSE     (980000)


#define MOTOR_PWM_PERIOD   (311)              //周期值 100000ns 频率 10kHz  即100 us   使用 hc32_ll 库时，应用
//#define MOTOR_ON_PULSE     (980000)


#define MOTOR_OFF_PULSE    (0)


typedef enum {
    MOTOR_U = 0,                   // 电机 U 相
    MOTOR_V,                       // 电机 V 相
    MOTOR_W,                       // 电机 W 相
    MOTOR_MAX
} MotorUVWTypeDef;

typedef enum {
    ALIGNMENGT_EDGE = 0,           // 边沿对齐
    ALIGNMENGT_CENTER,             // 中心对齐
    ALIGNMENGT_MAX
} PWMAlignmentTypeDef;

typedef enum {
    PWM_POLARITY_HIGH = 0,          // 输出极性 高
    PWM_POLARITY_LOW,               // 输出极性 低
    PWM_POLARITY_MAX
} PWMPolarityTypeDef;

typedef struct {
    uint8_t  channel;               // PWM 通道
    uint32_t duty;                  // PWM 占空比
    PWMPolarityTypeDef polarity;    // PWM 输出极性
} PWMCHnConfigDef;


typedef struct {
    bool     is_pwm_enabled;        // PWM 使能
    uint32_t period;                // PWM 周期    （ns）
    uint32_t freq;                  // PWM 频率
    uint16_t prescaler;             // PWM 预分频系数
    uint16_t dead_time;             // PWM 死区时间
    PWMAlignmentTypeDef alignment;  // PWM 对齐方式
    PWMCHnConfigDef PWM_CHn[MOTOR_MAX];
} PWMConfigDef;

/* ------------------- 共用体：PWM通道切换表 ------------------- */
// 六步方波的桥臂状态：1=开通，0=关闭（上桥臂与下桥臂互斥）

// 因硬件导线长度问题，电机绕线 U相 与 W相互换
typedef struct
{
    rt_uint8_t pwm_u_l : 1;  // U相下桥臂
    rt_uint8_t pwm_u_h : 1;  // U相上桥臂

    rt_uint8_t pwm_v_l : 1;  // V相下桥臂
    rt_uint8_t pwm_v_h : 1;  // V相上桥臂

    rt_uint8_t pwm_w_l : 1;  // W相下桥臂
    rt_uint8_t pwm_w_h : 1;  // W相上桥臂

//    rt_uint8_t dir:1;        // 电机运转方向
    rt_uint8_t reserve : 2;
} pwm_bridge_state_t;

// 共用体：PWM切换表（索引对应霍尔位置0-5）
typedef union
{
    pwm_bridge_state_t hall_pos[8]; // 按霍尔位置索引访问
    rt_uint8_t raw[8];              // 按原始字节访问（便于初始化）
} pwm_switch_table_t;


//extern pwm_switch_table_t pwm_table_forward;
//extern pwm_switch_table_t pwm_table_reverse;

/* 全局变量：PWM通道切换表（六步方波标准相序，可根据电机转向调整） */
// 正转切换表（标准六步相序）
extern const pwm_switch_table_t pwm_table_forward;

// 反转切换表（正转表的逆序，核心：霍尔位置对应的桥臂状态反向）
extern const pwm_switch_table_t pwm_table_reverse;

//   PWM  定义 end    *******************************************************************


//   MOTOR  定义 start    *******************************************************************


//   MOTOR  定义 end    *******************************************************************
typedef enum{
    MOTOR_DIRECTION_NULL,
    MOTOR_DIRECTION_LEFT,
    MOTOR_DIRECTION_RIGHT,
    MOTOR_DIRECTION_MAX
}MOTOR_DIRECTION_DEF;


typedef enum {
    MOTOR_STATE_NULL = 0,
    MOTOR_STATE_STOP,
    MOTOR_STATE_SOFT_START_LEFT,
    MOTOR_STATE_RUNNING_LEFT,
    MOTOR_STATE_SOFT_STOP_LEFT,

    MOTOR_STATE_SOFT_START_RIGHT,
    MOTOR_STATE_RUNNING_RIGHT,
    MOTOR_STATE_SOFT_STOP_RIGHT,

    MOTOR_STATE_HOLD,
    MOTOR_STATE_FAULT,
    MOTOR_STATE_MAX
} MOTOR_STATUS_TYPE_DEF;

typedef enum {
    MOTOR_CMD_NULL = 0,
    MOTOR_CMD_STOP,
    MOTOR_CMD_TURN_LEFT,
    MOTOR_CMD_TURN_RIGHT,
//    MOTOR_CMD_SOFT_STOP_LEFT,
//
//    MOTOR_CMD_SOFT_START_RIGHT,
//    MOTOR_CMD_RUNNING_RIGHT,
//    MOTOR_CMD_SOFT_STOP_RIGHT,
//
//    MOTOR_CMD_HOLD,
//    MOTOR_CMD_FAULT,
    MOTOR_CMD_MAX
} MOTOR_CMD_TYPE_DEF;

typedef struct{
    bool                  is_motor_enable;  // 电机使能
    MOTOR_DIRECTION_DEF   motor_direction;
    MOTOR_STATUS_TYPE_DEF motor_state;      // 电机状态
    uint32_t              motor_pwm_duty;   // 电机 占空比
}MOTOR_STATE_DEF;


//   绝对位置  定义 start  *******************************************************************

// 使用磁编码器
//#define USING_MAGNETIC_ENCODER

// 使用 uvw hall信号  + mcu 内部 flash 存储
#define USING_UVW_FLASH_ENCODER


typedef enum{
    ANGEL_STATE_IS_NULL =0,
    ANGLE_STATE_IS_CALIBRATED,
    ANGEL_STATE_IS_UNCALIBRATED,
    ANGEL_STATE_IS_CALIBRATED_SYS_NOT
//    ANGEL_STATE_MAX
}ANGLE_CALI_STATE_DEF;

extern ANGLE_CALI_STATE_DEF angle_calibrate_state ;

typedef struct {
    bool angle_valid;               // 角度有效性标志
    uint32_t raw_angle_41_offset;   // 原始角度值 偏移量
    uint32_t raw_angle_42_offset;   // 原始角度值 偏移量
    uint32_t raw_angle_43_offset;   // 原始角度值 偏移量
} ABS_ANGLE_CALIBRATE_DEF;

typedef struct {
    bool angle_valid;               // 角度有效性标志
    float32_t motor_angle;          // 电机角度(°)
    float32_t arm_angle;            // 摆臂角度(°)
    uint32_t raw_angle_41;          // 原始角度值
    uint32_t raw_angle_42;          // 原始角度值
    uint32_t raw_angle_43;          // 原始角度值
    uint32_t update_timestamp;      // 更新时间戳
} ABS_POS_DATA_DEF;

typedef enum{
    KTH7812_DEV_41 = 0,
    KTH7812_DEV_42,
    KTH7812_DEV_43,
    KTH7812_DEV_MAX
}KTH7812_DEV_DEF;


//   摆臂  定义 start  *******************************************************************

#define  ARM_ANGLE_TOLERANCE  (0.1)
typedef enum
{
    SWING_ARM_STATUS_NULL = 0,
    SWING_ARM_STATUS_STOPPED,
    SWING_ARM_STATUS_TURN_RIGHT,
    SWING_ARM_STATUS_TURN_LEFT,
    SWING_ARM_STATUS_MAX
}ARM_STATUS;

typedef struct
{
    ARM_STATUS       arm_run_status;           // 摆臂 状态
    float32_t        arm_angle_tolerance;  // 摆臂 允许误差
    float32_t        arm_angle_realtime;   // 摆臂 实时角度
    float32_t        arm_angle_target;     // 摆臂 目标 角度

}ARM_POS_DEF;

//   摆臂  定义 end    *******************************************************************

//   绝对位置  定义 end    *******************************************************************


//   CAN通信 定义 start *******************************************************************



typedef enum{
    CAN_CMD_NULL = 0,
    CAN_CMD_TURN_STOP,
    CAN_CMD_ARM_TURN_ZERO_AUTO,
    CAN_CMD_ARM_TURN_LEFT,
    CAN_CMD_ARM_TURN_RIGHT,
    CAN_CMD_ARM_TURN_RIGHT_MIDDLE_AUTO,
    CAN_CMD_ARM_TURN_RIGHT_REAL_AUTO,
    CAN_CMD_CALIBRATE,
    CAN_CMD_UPDATE_STATUS,
    CAN_CMD_CALIBRATE_FOR_FIRSET_RUN,
    CAN_CMD_MAX
}CAN_CMD_TYPE_E;


typedef struct {
    CAN_CMD_TYPE_E CAN_current_cmd;  // 当前 can cmd
    CAN_CMD_TYPE_E can_last_cmd;     // 上一次 can cmd
//    float target_angle;           // 目标角度
//    uint32_t param;               // 配置参数
} CAN_CMD_DEF;

extern CAN_CMD_DEF CAN_cmd_data;

typedef struct {
    bool is_can_enabled;          //
    uint32_t can_id;              //
    uint32_t can_baudrate;        //
//    CAN_CMD_DEF can_cmd;          //
} CAN_CONFIG_DEF;

typedef enum
{
    IS_CONNECTED    = 0xE0,
    IS_DISCONNECTED = 0xE1
}CAN_STATUS_E;

typedef enum
{
    CAN_DEV_ID_1 = 0x00,
    CAN_DEV_ID_2,
    CAN_DEV_ID_MAX
}CAN_DEV_ID_E;


#define CAN_timeout          10                       // 定义 CAN通信超时  10s
// CAN通信结构体
typedef struct
{
    CAN_STATUS_E       CAN_status;                    // CAN 通信状态
    uint16_t           CAN_ID;                        // CAN 通信ID
    uint32_t           CAN_baud;                      // CAN 通信波特率  单位  bps
    uint32_t           CAN_msg_lost_count;            // CAN 消息丢失计数  can生命信号 以1s周期性下发，超过10s未收到消息，判定can通信异常
}CAN_CLASS_T;

//   CAN通信 定义 end*******************************************************************



//   换向 hall  定义 start  *******************************************************************
typedef struct {
    uint8_t u_state;                //  U 相 状态  0/1
    uint8_t v_state;                //  V 相 状态  0/1
    uint8_t w_state;                //  W 相 状态  0/1
    uint8_t bldc_step;              //  换向步数
} HALL_UVW_DEF;

typedef union{

    struct{
        uint8_t hall_u   :1;    //  该位 位于 该字节的 最右侧
        uint8_t hall_v   :1;
        uint8_t hall_w   :1;
        uint8_t reserved :5;
    }hall_bits;

    uint8_t code;

}HALL_STATE_DEF;
//   换向 hall  定义 end    *******************************************************************




//   BLDC  定义 start    *******************************************************************

typedef struct {
    float target_arm_angle;           // 目标角度
    float current_arm_angle;          // 当前角度
    float arm_speed;                  // 当前速度(°/s)
    uint8_t commutation_step;     // 当前换向步数
    uint32_t start_timestamp;     // 启动时间戳
    uint32_t stop_timestamp;      // 停止时间戳
} BLDC_CTRL_DATA_DEF;


// 三相半桥导通状态定义
typedef struct {
    bool u_top;  // U相上桥臂：true=PWM输出，false=截止
    bool v_top;  // V相上桥臂：true=PWM输出，false=截止
    bool w_top;  // W相上桥臂：true=PWM输出，false=截止
    bool u_bot;  // U相下桥臂：true=导通，false=截止
    bool v_bot;  // V相下桥臂：true=导通，false=截止
    bool w_bot;  // W相下桥臂：true=导通，false=截止
} Half_Bridge_State_TypeDef;

// 电机换相控制句柄（整合霍尔、半桥、PWM、转向）
typedef struct {
//    Hall_State_TypeDef current_hall_state;  // 当前霍尔状态编码
//    Motor_Dir_TypeDef current_dir;          // 当前电机转向
    uint16_t pwm_duty_cycle;                // PWM占空比（0~1000，对应0~100%）
    uint32_t pwm_freq;                      // PWM频率（如20kHz，避免电机啸叫）
    Half_Bridge_State_TypeDef bridge_state; // 当前半桥导通状态
} Motor_Commutate_HandleTypeDef;

typedef enum{
    SYS_STATE_NULL = 0,
    SYS_STATE_INIT,
    SYS_STATE_NORMAL,
    SYS_STATE_FAULT,
    SYS_STATE_MAX
}SYS_STATE_DEF;

typedef struct{
    SYS_STATE_DEF sys_state;


}SYS_STATE_CONFIG_DEF;

//SYS_STATE_CONFIG_DEF sys_state_config={0};


// 2. 定义全局数据结构体（存储所有状态和参数，解耦数据与逻辑）
typedef struct {
    // 电源 电压状态数据
//    POWER_VOLTAGE_STATUS_TYPE_DEF power_voltage_state;
//    // 电源 电流状态数据
//    POWER_CURRENT_STATUS_TYPE_DEF power_current_state;
    SYS_STATE_DEF   current_sys_state;

    // 状态机当前状态
    MOTOR_STATE_DEF current_motor_state;



    // 摆臂角度数据
    ARM_POS_DEF arm_pos;

    // CAN命令数据
    CAN_CMD_TYPE_E can_cmd;           // CAN控制命令：0-停止，1-左转，2-右转
    MOTOR_CMD_TYPE_DEF motor_cmd;     // 电机控制命令
//    uint8_t can_feedback;      // CAN反馈数据

    // UVW换向信号
    HALL_STATE_DEF hall_state_data;

    uint32_t period;                // PWM 周期    （ns）

    uint8_t  duty;                  // 占空比 2~98
    uint8_t  soft_start_duty;

} MOTOR_CTRL_HANDLE_DEF;


extern MOTOR_CTRL_HANDLE_DEF  motor_ctrl_data;



//   BLDC  定义 end    *******************************************************************



//   错误 定义 start*******************************************************************
typedef enum {
    ERR_NONE = 0,
    ERR_POWER_UNDERVOLTAGE,
    ERR_POWER_OVERVOLTAGE,
    ERR_OVERCURRENT,
    ERR_HALL_COMM,
    ERR_HALL_ANGLE,
    ERR_CALIB_FAILED,
    ERR_FLASH,
    ERR_CAN_COMM,
    ERR_LOCK_ROTOR,
    ERR_MAX
} ErrorCodeTypeDef;

typedef enum {
    ERROR_LEVEL_WARNING = 0,
    ERROR_LEVEL_ERROR,
    ERROR_LEVEL_FATAL
} ErrorLevelTypeDef;
//   错误  定义 end*******************************************************************


//   data  定义 start*******************************************************************
// 系统信息
extern const char myself_software_VN[10];


ALIGN(RT_ALIGN_SIZE)
typedef struct
{
    uint16_t  system_is_setup;                        // 系统配置状态  已配置，未配置

    char      product_SN[24];                         // 产品序列号    QW_H2413_20250125001
    char      software_VN[12];                        // 软件版本       V00.00.00
    char      hardware_VN[12];                        // 硬件版本       V00.00.00
}INFO_CLASS_T;

#define  GET_DATA_IS_OK         (0xA0A0A0A0)
// 数据存储结构体
typedef struct
{
    uint32_t  data_storage_times;                     // 数据存储   次数
    uint32_t  data_sector_number;                     // 数据存储   扇区编号

    uint32_t  get_data_is_ok;                         // get_data_is_ok == 0xA0A0A0A0  表示数据读取有效

    uint16_t  device_set_CAN_BAUD1;                   //  device_set_CAN_BAUD == 100 /  125 / 250  kpbs 默认 100
    uint16_t  device_set_CAN_BAUD2;                   //  device_set_CAN_BAUD == 100 /  125 / 250  kpbs 默认 100

}DATA_STORAGE_CLASS;

extern   DATA_STORAGE_CLASS  data_storage;

//   data  定义 end*******************************************************************

// 定义系统指令枚举类型
typedef enum{
    SYSTEM_CMD_IDLE     = 0x00,      /* 系统控制指令  空闲*/

    SYSTEM_CMD_MAX
}SYSTEM_CMD_E;

// 定义系统指令枚举类型
typedef enum{
    MODEL_TYPE_NONE   = 0x00,      /* 未设置机型    00 */
    MODEL_TYPE_13     = 0x01,      /* 机型：01--13齿机型 */
    MODEL_TYPE_14     = 0x02,      /* 机型：10--14齿机型 */
    MODEL_TYPE_11     = 0x03       /* 机型：11--11齿机型 */
}MODEL_TYPE_E;

typedef struct{
    MODEL_TYPE_E   model_name;          // 机型
    uint8_t        model_set_state;     // 机型设置状态       0-未设置；1-已设置
}MODEL_DEF_T;

#define SYSTEM_IS_SET_OK             (0xAAAA)

#define SYSTEM_IS_CALI_MODE          (0xA0)
/* 系统结构体 */
typedef struct
{
    INFO_CLASS_T                       system_info;              /* 系统信息 版本，序列号*/
    MODEL_DEF_T                        model;                    /* 机型信息 */
    POWER_VOLTAGE_STATUS_TYPE_DEF      power_voltage_status;      /* 系统供电状态={系统上电、系统掉电、系统欠压、系统过压} */
    POWER_CURRENT_STATUS_TYPE_DEF      power_current_status;
    SYSTEM_CMD_E                       system_cmd;               /* 系统控制指令  空闲、打开、关闭，可存储10个控制指令队列 */
    uint8_t                            cali_mode;                // 当  cali_mode == 0xA0 时，系统进入标定模式，获取门的实际开度，电推杆关门最大电流



}SYSTEM_CLASS_T;

extern SYSTEM_CLASS_T system_class;

extern uint8_t system_class_cali_mode;

void get_system_config(void);


typedef struct{
    uint32_t  tick_power_thread;
    uint32_t  tick_can_received_thread;
    uint32_t  tick_can_send_thread;
}CHECK_TICK_DEF;

extern CHECK_TICK_DEF check_tick_data;

#endif /* APPLICATIONS_APP_H_ */
