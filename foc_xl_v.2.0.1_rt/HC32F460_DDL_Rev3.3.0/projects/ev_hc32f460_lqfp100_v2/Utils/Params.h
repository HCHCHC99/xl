#ifndef __PARAMS_H__
#define __PARAMS_H__

#include <stdint.h>
#include <stdbool.h>
#include "rtt_manager.h"
#include "param_manager.h"      // 添加这行，提供 Param_Config_t 定义
/*=============================================================================
 * 调试宏定义
 * 开关在 rtt_manager.h 中统一管理：APP_PARAMS_DBG
 *============================================================================*/
#ifdef APP_PARAMS_DBG
    #define PARAMS_DBG(fmt, ...)    MAIN_D("[PARAMS] " fmt, ##__VA_ARGS__)
#else
    #define PARAMS_DBG(fmt, ...)    ((void)0)
#endif
#ifdef PARAM_PRINT_DBG
    #define PARAM_PRINT(fmt, ...)    MAIN_D("[PARAM_PRINT] " fmt, ##__VA_ARGS__)
#else
    #define PARAM_PRINT(fmt, ...)    ((void)0)
#endif

/*=============================================================================
 * 客户协议寄存器地址定义
 * 保持寄存器 (Holding Register, 功能码 03/06/10)
 *============================================================================*/

/* --- 配置参数（断电保存，Flash 存储） --- */
#define REG_NODE_ID                 (0x2710U)   /* 设备地址 1~247 (uint16_t) */
#define REG_TARGET_SPEED            (0x2711U)   /* 转速 r/min (int16_t) */
#define REG_TARGET_ANGLE            (0x2712U)   /* 角度 0.1度 (int16_t) */
#define REG_VOLTAGE_UPPER_LIMIT     (0x2714U)   /* 电压上限 0.1V (uint16_t) */
#define REG_VOLTAGE_LOWER_LIMIT     (0x2715U)   /* 电压下限 0.1V (uint16_t) */
#define REG_CURRENT_UPPER_LIMIT     (0x2716U)   /* 电流上限 1mA (uint16_t) */
#define REG_CLOSE_LIMIT_ANGLE       (0x271CU)   /* 关闭极限角度 0.1度 (int16_t) */
#define REG_OPEN_LIMIT_ANGLE        (0x271DU)   /* 打开极限角度 0.1度 (int16_t) */
#define REG_CURRENT_DETECT_MS       (0x271EU)   /* 电流检测时间 1ms (uint16_t) */

/* 注意：0x2713, 0x2717-0x271B, 0x271F 为保留地址，不暴露给 Modbus */

/* --- 控制命令（写触发，读回实际状态） --- */
#define REG_CTRL_CMD                (0x2720U)   /* 控制命令寄存器 (uint16_t) */

/* REG_CTRL_CMD (0x2720) 位定义：
 * 写操作：
 *   bit0 = 1: 启动（使能 RS485 控制），必须先发此指令
 *   bit1 = 1: 停止（关闭 RS485 控制）
 *   bit2 = 1: 急停（取消正转/反转，不关闭 RS485 控制）
 *   bit3 = 1: 复位（系统软件复位）
 *   bit4 = 1: 正转（仅在已启动状态下有效，与 bit5 互斥）
 *   bit5 = 1: 反转（仅在已启动状态下有效，与 bit4 互斥）
 * 读操作：
 *   bit4 = 1: 当前正在正转
 *   bit5 = 1: 当前正在反转
 *   bit4=0 且 bit5=0: 当前停止
 */
#define CTRL_CMD_START              (0x0001U)   /* bit0: 启动指令 */
#define CTRL_CMD_STOP               (0x0002U)   /* bit1: 停止指令 */
#define CTRL_CMD_ESTOP              (0x0004U)   /* bit2: 急停指令 */
#define CTRL_CMD_RESET              (0x0008U)   /* bit3: 复位指令 */
#define CTRL_CMD_FWD                (0x0010U)   /* bit4: 正转指令/状态 */
#define CTRL_CMD_REV                (0x0020U)   /* bit5: 反转指令/状态 */

/* --- 实时数据（只读，不存 Flash） --- */
#define REG_REAL_SPEED              (0x2730U)   /* 实时转速 r/min (int16_t) */
#define REG_REAL_ANGLE              (0x2731U)   /* 实时角度 0.1度 (int16_t) */
#define REG_REAL_VOLTAGE            (0x2732U)   /* 电压 0.1V (uint16_t) */
#define REG_REAL_CURRENT            (0x2733U)   /* 电流 1mA (uint16_t) */
#define REG_REAL_DIRECTION          (0x2737U)   /* 实时转向 (int16_t) */

/* --- 实时故障（只读，不存 Flash） --- */
#define REG_FAULT_STATUS            (0x2740U)   /* 故障状态 (uint16_t) */

/*=============================================================================
 * Flash 存储参数默认值宏定义
 * 对应 AppParamRecord_t 结构体字段，上电从 Flash 加载
 * 注意：电压滞回值单位 0.1V，电流滞回值单位 mA
 *============================================================================*/

/* --- Modbus 协议参数（保持寄存器，断电保存，暴露给 Modbus） --- */
#define PARAM_DEFAULT_NODE_ID                   (1U)        /* 设备地址 1~247 */
#define PARAM_DEFAULT_TARGET_SPEED              (0)         /* 转速 r/min */
#define PARAM_DEFAULT_TARGET_ANGLE              (0)         /* 角度 0.1度 */
#define PARAM_DEFAULT_VOLTAGE_UPPER_LIMIT       (270U)      /* 电压上限 0.1V (27.0V) */
#define PARAM_DEFAULT_VOLTAGE_LOWER_LIMIT       (210U)      /* 电压下限 0.1V (21.0V) */
#define PARAM_DEFAULT_CURRENT_UPPER_LIMIT       (5000)      /* 电流上限 1mA (3A) */
#define PARAM_DEFAULT_CURRENT_DETECT_MS         (20)        /* 电流检测时间 1ms */
#define PARAM_DEFAULT_CLOSE_LIMIT_ANGLE         (-20)       /* 关闭极限角度 0.1度 */
#define PARAM_DEFAULT_OPEN_LIMIT_ANGLE          (880)       /* 打开极限角度 0.1度 */
#define PARAM_DEFAULT_BAUD_RATE                 (9600UL)    /* 默认波特率 */

/* --- 内部参数（不暴露给 Modbus，仅 Flash 存储，上电加载） --- */
#define PARAM_DEFAULT_VOLTAGE_UPPER_HYSTERESIS  (20U)       /* 过压滞回 0.1V (2.0V) */
#define PARAM_DEFAULT_VOLTAGE_LOWER_HYSTERESIS  (20U)       /* 欠压滞回 0.1V (2.0V) */
#define PARAM_DEFAULT_OVERVOLTAGE_TRIGGER_CNT   (1U)        /* 过压触发次数 */
#define PARAM_DEFAULT_UNDERVOLTAGE_TRIGGER_CNT  (1U)        /* 欠压触发次数 */
#define PARAM_DEFAULT_CURRENT_HYSTERESIS_MA     (0U)        /* 电流滞回 1mA (0A) */
#define PARAM_DEFAULT_CURRENT_RELEASE_MS        (200U)      /* 电流解除时间 1ms (0.2s) */
#define PARAM_DEFAULT_OVERCURRENT_TRIGGER_CNT   (1U)        /* 过流触发次数 */

/*=============================================================================
 * 核心参数记录结构体 (客户 Modbus 协议定义)
 * 注意：结构体布局必须与 Flash 中存储的布局一致
 *       head_magic 和 tail_magic 用于 Flash 块识别
 *       checksum 用于数据完整性校验
 *============================================================================*/
#pragma pack(4)
typedef struct {
    /* 头部信息 */
    uint32_t head_magic;        /* 头部魔数 (0x55AA55AA) */
    uint32_t sequence_id;       /* 序列号，每次修改递增 */
    uint32_t erase_count;       /* Flash擦除次数记录 */

    /* 客户 Modbus 协议参数 (保持寄存器，断电保存，暴露给 Modbus) */
    uint16_t node_id;               /* 0x2710: 设备地址 1~247 */
    int16_t  target_speed;          /* 0x2711: 转速 (r/min) */
    int16_t  target_angle;          /* 0x2712: 角度 (0.1度) */
    uint16_t voltage_upper_limit;   /* 0x2714: 电压上限 (0.1V) */
    uint16_t voltage_lower_limit;   /* 0x2715: 电压下限 (0.1V) */
    uint16_t current_upper_limit;   /* 0x2716: 电流上限 (1mA) */
    uint16_t current_detect_ms;     /* 0x271E: 电流检测时间 (1ms) */
    int16_t  close_limit_angle;     /* 0x271C: 关闭极限角度 (0.1度) */
    int16_t  open_limit_angle;      /* 0x271D: 打开极限角度 (0.1度) */
    uint32_t baud_rate;             /* 波特率 (内部参数，不上Modbus) */

    /* 内部参数（不暴露给 Modbus，仅 Flash 存储，上电加载） */
    uint16_t voltage_upper_hysteresis;   /* 过压滞回 (0.1V) */
    uint16_t voltage_lower_hysteresis;   /* 欠压滞回 (0.1V) */
    uint8_t  overvoltage_trigger_count;  /* 过压触发次数 */
    uint8_t  undervoltage_trigger_count; /* 欠压触发次数 */
    uint16_t current_hysteresis_ma;      /* 电流滞回 (mA) */
    uint16_t current_release_ms;         /* 电流解除时间窗口 (ms) */
    uint8_t  overcurrent_trigger_count;  /* 过流触发次数 */
    uint8_t  reserved[3];                /* 保留字节，对齐到4字节边界 */

    /* 尾部信息 */
    uint32_t checksum;              /* CRC32校验和 */
    uint32_t tail_magic;            /* 尾部魔数 (0xAA44AA44) */
} AppParamRecord_t;
#pragma pack()

/*=============================================================================
 * 全局变量声明
 *============================================================================*/
extern AppParamRecord_t g_AppParam;         /* 应用程序参数（Flash 存储） */

/*=============================================================================
 * 参数读写接口函数声明
 *============================================================================*/

/**
 * @brief  参数管理器初始化
 * @param  pConfig  初始化配置（参数缓冲区指针、大小、魔数等）
 * @param  pSetDefaults 设置默认值的回调函数
 * @return PARAM_OK 成功，PARAM_ERR 失败
 */
int32_t Param_Init(const Param_Config_t *pConfig, void (*pSetDefaults)(void));

/**
 * @brief  保存参数到 Flash
 * @param  pConfig  初始化配置
 * @return PARAM_OK 成功，PARAM_ERR 失败
 */
int32_t Param_Save(const Param_Config_t *pConfig);

/**
 * @brief  根据寄存器地址读取参数值
 * @param  regAddr  寄存器地址 (REG_xxx)
 * @param  pValue   输出参数值指针
 * @return 0 成功，-3 无效地址
 */
int32_t Param_ReadByReg(uint16_t regAddr, uint16_t *pValue);

/**
 * @brief  根据寄存器地址写入参数值（只写入内存，不执行业务逻辑）
 * @param  regAddr  寄存器地址 (REG_xxx)
 * @param  value    要写入的值
 * @return 0 成功，-3 无效地址
 */
int32_t Param_WriteByReg(uint16_t regAddr, uint16_t value);

/**
 * @brief  检查寄存器地址是否在有效范围内
 * @param  regAddr  寄存器地址
 * @retval true     地址有效
 * @retval false    地址无效
 */
bool Param_IsValidRegister(uint16_t regAddr);

#endif /* __PARAMS_H__ */
