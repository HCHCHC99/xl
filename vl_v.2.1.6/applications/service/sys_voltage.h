/**
 * @file    sys_voltage.h
 * @brief   系统（母线）电压采集
 *
 * 【硬件与数据源】
 *   AD_VM → PA6 → ADC12_IN6
 *   FOC 的 ADC1 SEQ_A 每 50us 与三相电流同步采样 CH6，
 *   DMA 中断每帧调用 sys_voltage_vbus_sample_isr(raw)。
 *   本模块维护 20 点（20kHz → 1ms）环形缓冲区，求平均后按板型分压换算。
 *
 * 【分压系数】（定义在 sys_voltage.c）
 *   板型 0：100k/3.3k ≈ 31.03 倍
 *   板型 1/2：25 倍
 *
 * 【不再使用 ADC2】
 *   PA6 只有 FOC ADC1 CH6 一个采样源；ADC2 设备仍保留给 lift_adc(CH3)。
 */

#ifndef __SYS_VOLTAGE_H
#define __SYS_VOLTAGE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 台架模拟开关（任务清单 §11）
 * ------------------------------------------------------------
 * 1 = 允许 t_sim bus <mV> 覆盖母线电压（会驱动欠压状态机、启车判定、
 *     状态帧 0x18FF11D1 的电源位；台架无需可调电源）
 * 0 = 关闭，只读真实 ADC
 * 总开关 INPUT_SIM_ENABLE 在 service/input_sim.h
 * ============================================================ */
#ifndef SYS_VOLT_SIM_ENABLE
#define SYS_VOLT_SIM_ENABLE 1
#endif

/* ============================================================
 * 标定常量
 * ============================================================ */
#define SYS_VOLT_ADC_FULL_SCALE     4096U   /* 12bit 分频基数 */
#define SYS_VOLT_ADC_VREF_MV        3300U   /* VDDA = 3.3V */

/* ============================================================
 * API
 * ============================================================ */

/**
 * @brief 初始化（清空环形缓冲区） [单线程]
 * @return 0
 */
int sys_voltage_init(void);

/**
 * @brief 送入一个母线电压原始码（必须在 20kHz ADC 中断内调用） [ISR]
 * @param raw  PA6/CH6 的 12bit 原始值
 * @note  每满 20 点（1ms）求一次平均，并按板型分压系数换算成 mV。
 *        写入与平均都在同一中断上下文，无需临界区。
 */
void sys_voltage_vbus_sample_isr(uint16_t raw);

/**
 * @brief 取 1ms 平均后的母线电压（mV） [ISR/线程安全]
 * @return 0 = 尚未采满 1ms（FOC 未启动）
 */
uint32_t sys_voltage_get_bus_mv_1ms(void);

/**
 * @brief 读取母线电压（mV，1ms 平均值） [线程安全]
 * @return 未就绪时返回 0
 */
uint32_t sys_voltage_read_bus_mv(void);

/**
 * @brief 取最近一次母线电压缓存（mV） [ISR 安全]
 */
uint32_t sys_voltage_get_bus_mv_cached(void);

/**
 * @brief 读取母线电压（0.1V 单位，例如 13.0V → 130） [线程安全]
 * @note  供启车判定与状态帧上报使用
 */
uint16_t sys_voltage_get_dv(void);

/**
 * @brief 采集是否就绪（已采满 1ms 并得到有效值）
 * @return 1 = 可用；0 = 不可用
 */
uint8_t sys_voltage_is_ready(void);

/* ============================================================
 * 欠压状态机（业务.md §13.6-A / 任务清单 §10.5-A）
 * ------------------------------------------------------------
 * sys_voltage_init() 末尾启动 10 ms 软定时器轮询 1 ms 平均母线电压：
 *   SYS_POWER_NORMAL --(bus < enter_mv 连续 confirm_ms，且已过 blank_ms 消隐)-->
 *   SYS_POWER_UV
 *   SYS_POWER_UV     --(bus > recover_mv 连续 recover_ms)--> SYS_POWER_NORMAL
 * 状态变化各投递一次 SYS_EVT_UV_ENTER / SYS_EVT_UV_RECOVER。
 *
 * 5 个门限参数全部来自 sys_config 的 CFG_ID_UV_*（首次定时器回调读取，
 * 失败用 CFG_DEF_UV_* 兜底）；本模块只做判定，**不执行急停、不写存储**。
 * ============================================================ */
typedef enum {
    SYS_POWER_NORMAL = 0,       /* 正常 */
    SYS_POWER_UV     = 1,       /* 欠压 */
} sys_power_state_t;

/**
 * @brief 取当前电源状态 [线程安全]
 */
sys_power_state_t sys_voltage_get_power_state(void);

/**
 * @brief 是否处于欠压态 [线程安全，供 FOC 门禁/状态帧使用]
 * @return 1 = 欠压；0 = 正常
 */
uint8_t sys_voltage_is_uv(void);

/**
 * @brief 打印生效的欠压参数（10 ms 轮询已在 sys_voltage_init() 内启动）
 * @note  仅诊断用途；可在启动完成后调用一次，不重复启动定时器。
 */
void sys_voltage_monitor_start(void);

#ifdef __cplusplus
}
#endif

#endif /* __SYS_VOLTAGE_H */
