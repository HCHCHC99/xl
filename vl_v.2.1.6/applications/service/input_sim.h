/**
 * @file    input_sim.h
 * @brief   业务输入台架模拟层（任务清单 §11）
 *
 * 【解决什么问题】
 *   业务功能（启车/门禁/一键/限位/欠压/阀故障/灯帧…）的调试依赖 6 类外部输入：
 *   ACC 钥匙、母线电压、面板按键、发动机转速、阀反馈、举升传感器。
 *   台架上往往凑不齐（没有钥匙、没有可调电源、没有 CPB 面板、没有 ECU）。
 *   本层给每个输入一个"覆盖值"，打开后各模块的 getter 直接返回该值，
 *   业务代码一行都不用改，就能把整条链路跑起来。
 *
 * 【怎么用】
 *   t_sim                    → 列出所有输入：名字 / 是否覆盖 / 当前值
 *   t_sim acc on             → 模拟钥匙上电（等价 PB0 拉低）
 *   t_sim bus 10800          → 模拟母线 10.8V（驱动欠压状态机）
 *   t_sim swing 1200         → 模拟回转角 120.0°
 *   t_sim lift 2203          → 模拟举升传感器 2203mV（= 0°）
 *   t_sim key mid hold       → 按住"一键右中"（50ms 周期重发，模拟真实面板）
 *   t_sim key up pulse       → 短按"上升"300ms 后自动松开
 *   t_sim rpm 600            → 模拟发动机 600rpm（50ms 重发，避免 100ms 超时归零）
 *   t_sim valve 1 0          → 模拟上升阀开路故障
 *   t_sim teeth 1            → 模拟 HMI 下发 13 齿
 *   t_sim cansilent 1        → 强制 CAN 静默（测 comm_sta / 整车 FAULT）
 *   t_sim clear [name]       → 恢复真实输入（不带 name = 全部恢复）
 *
 * 【安全性】
 *   - 默认 `INPUT_SIM_ENABLE=1`：台架可用，量产可整体关掉（命令一起消失）。
 *   - 每个输入另有独立编译开关，放在**该输入自己的头文件**里：
 *       ACC              → key_state.h        ACC_SIM_ENABLE
 *       母线             → sys_voltage.h      SYS_VOLT_SIM_ENABLE
 *       回转角           → swing_angle.h      SWING_ANGLE_SIM_ENABLE
 *       举升传感器       → lift_adc.h         LIFT_ADC_SIM_ENABLE
 *       按键/转速/阀/机型 → ut_can_handlers.h  CAN_SIM_ENABLE
 *       CAN 静默         → ut_can_scheduler.h CAN_SILENT_SIM_ENABLE
 *   - 模拟值只影响"读输入"的位置，不绕过门禁、状态机、限位与故障判定，
 *     因此用它测出来的行为与真实输入一致（这正是它的意义）。
 *   - 打印全部为 ASCII/整数（RTT 控制台不支持中文与浮点）。
 */

#ifndef __INPUT_SIM_H
#define __INPUT_SIM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 主开关
 * ------------------------------------------------------------
 * 0 = 整层不参与编译：t_sim 命令消失，各钩子回到真实输入。
 * 量产固件请置 0（与 SYS_CAP_TEMP_ALLOW_PARAM_WRITE 同批处理）。
 * ============================================================ */
#ifndef INPUT_SIM_ENABLE
#define INPUT_SIM_ENABLE            1
#endif

/* ============================================================
 * 可模拟的输入源
 * ============================================================ */
typedef enum {
    INPUT_SIM_ACC = 0,      /* 值 = PB0 原始电平：0=钥匙ON(低) / 1=OFF(高) */
    INPUT_SIM_BUS,          /* 值 = 母线 mV（1ms 平均值） */
    INPUT_SIM_SWING,        /* 值 = 回转编码器 raw，单位 0.1°（角度 = 值 − 零点） */
    INPUT_SIM_LIFT,         /* 值 = 举升传感器 mV（2203 = 0°，36.4mV/°） */
    INPUT_SIM_KEY,          /* 值 = 按键掩码（KB_* 位组合） */
    INPUT_SIM_RPM,          /* 值 = 发动机转速 rpm */
    INPUT_SIM_VALVE,        /* 值 = 打包位：bit0-1 up_fault / bit2-3 down_fault /
                                       bit4 up_open / bit5 down_open */
    INPUT_SIM_TEETH,        /* 值 = 0x18FF7F18 协议值 0清除/1 13齿/2 14齿/3 11齿 */
    INPUT_SIM_CANSILENT,    /* 值 = 1 强制总线静默 / 0 正常 */
    INPUT_SIM_SRC_COUNT
} input_sim_src_t;

/**
 * @brief 该输入当前是否被模拟覆盖 [线程安全，单字读写]
 * @return 1 = 用模拟值；0 = 用真实输入
 */
uint8_t input_sim_active(input_sim_src_t src);

/**
 * @brief 取模拟值（未覆盖时返回 0，调用方必须先判 input_sim_active）
 */
int32_t input_sim_value(input_sim_src_t src);

/**
 * @brief 打开覆盖并置值（首次调用会懒创建 50ms 重发定时器）
 * @return 0 成功；-1 名字/参数非法
 */
int input_sim_set(input_sim_src_t src, int32_t value);

/**
 * @brief 关闭单个输入的覆盖
 */
void input_sim_clear(input_sim_src_t src);

/**
 * @brief 关闭全部覆盖
 */
void input_sim_clear_all(void);

/**
 * @brief 创建重发定时器（幂等；t_sim 与 set 内部自动调用）
 */
void input_sim_init(void);

/**
 * @brief 名字 → 源（t_sim 用）
 * @return 0 成功；-1 未知名字
 */
int input_sim_name_to_src(const char *name, input_sim_src_t *out);

#ifdef __cplusplus
}
#endif

#endif /* __INPUT_SIM_H */