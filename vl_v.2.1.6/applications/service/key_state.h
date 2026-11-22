/**
 * @file    key_state.h
 * @brief   钥匙上电状态检测（PB0，独立于整车启动判定）
 *
 * 【需求来源】用户 2026-11-12：
 *   钥匙上电状态：PB0 外部输出，信号类型 IO，低电平有效。
 *   当 PB0 满足有效电平，持续 500ms，判定钥匙上电，否则判定钥匙下电。
 *
 * 【Q33 裁定：独立】2026-11-12 用户明确「Q33 独立」。
 *   本模块与 vehicle_state（整车启动判定）是**两个独立状态**：
 *     - key_on        : 钥匙上电（PB0 电平）
 *     - vehicle_started: 整车启动（转速 >400rpm 或 电压 >13V）
 *   二者互不参与对方的判定条件，也不互为前置使能。
 *   → 不再把钥匙并入 vehicle_state 的「或」条件，也不改其判据。
 *
 * 【Q34 裁定：外部 10k 上拉】2026-11-12 用户确认：
 *   PB0 外部硬件已加 10k 上拉到 3.3V。
 *   ⇒ MCU 内部上拉【必须保持关闭】(PIN_PU_OFF)，避免并联降低等效阻值。
 *   ⇒ 未接/断线时 PB0 被外部上拉为高 = 下电（安全侧），符合设计意图。
 *
 * 【电气约定】
 *   低电平有效：PB0 读 0 → 钥匙上电（ON）；读 1 → 钥匙下电（OFF）
 *   去抖 500ms：与 vehicle_state 同构（轮询累加，中断即清零重计时）
 */

#ifndef __KEY_STATE_H
#define __KEY_STATE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 台架模拟开关（任务清单 §11）
 * ------------------------------------------------------------
 * 1 = 允许 t_sim acc on|off 覆盖 PB0 电平（去抖仍生效，500ms 后确认）
 * 0 = 关闭该输入的模拟，acc_read_raw_level() 只读真实引脚
 * 总开关 INPUT_SIM_ENABLE 在 service/input_sim.h
 * ============================================================ */
#ifndef ACC_SIM_ENABLE
#define ACC_SIM_ENABLE      1
#endif

/* ============================================================
 * 判定参数（现场标定只改这里）
 * ============================================================ */
typedef struct {
    uint16_t enter_ms;      /* 上电确认去抖（默认 500ms） */
    uint16_t exit_ms;       /* 下电确认去抖（默认 500ms） */
    uint16_t poll_ms;       /* 轮询周期（默认 50ms，与 50ms tick 一致） */
} acc_state_cfg_t;

/* E-3：改为 RAM 运行副本，由 acc_state_init() 从配置表装载（R6 / Q-84）。
 * ROM 默认值仍在 key_state.c 中，读取失败时保持不变。 */
extern acc_state_cfg_t g_acc_state_cfg;

/* ============================================================
 * 接口
 * ============================================================ */

/**
 * @brief 初始化 PB0 为输入（内部上拉关闭，外部已有 10k 上拉）
 * @return 0 成功；<0 失败
 */
int acc_state_init(void);

/**
 * @brief 周期轮询（在 50ms tick 内调用）
 * @note  内部做 500ms 去抖；状态跳变时投递 SYS_EVT_ACC_ON / SYS_EVT_ACC_OFF
 */
void acc_poll(void);

/**
 * @brief 查询钥匙是否已上电（去抖后的稳定状态）
 * @return 1 = 上电；0 = 下电
 */
uint8_t acc_is_on(void);

/**
 * @brief 查询 PB0 当前原始电平（调试用，未去抖）
 * @return 1 = 高电平（下电）；0 = 低电平（上电）
 */
uint8_t acc_read_raw_level(void);

/**
 * @brief 强制复位为下电状态（调试/测试用）
 */
void acc_force_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* __KEY_STATE_H */