/**
 * @file    sys_caps.h
 * @brief   系统模式能力位（静态可配置权限表）
 *
 * ============================================================
 * 【设计说明】
 * ------------------------------------------------------------
 * 1) 为什么用能力位而不是散落的 if 判断
 *    「某模式下能不能做某事」是横切关注点。若在每个执行入口各写一遍
 *    模式判断，新增功能要改 N 处，且无法回答"刚才那次是谁拒的"。
 *    这里把权限收敛为「能力位」+「单点判定函数」，调用方只声明
 *    "我需要 XX 能力"，不再关心当前是什么模式。
 *
 * 2) 三张 ROM 默认表
 *    每个模式一张 uint16 掩码，定义在 sys_caps.c。
 *    权唯一权威来源是 docs/02-设计/模式权限静态配置表-V1.0.md，
 *    改权限只改表，不改调用点。
 *
 * 3) 现场裁剪（cap_override）
 *    mode_config_t.reserved1 复用为 cap_override：
 *       0x00 = 不覆盖，用默认表
 *       其它 = 按位收紧
 *    有效能力 = 默认表[mode] & (cap_override ? cap_override : 0xFFFF)
 *    【安全约束】只能收紧，不能越权——即使 cap_override 置了某位，
 *    默认表该位为 0 时仍为 0（按位与天然保证）。
 *
 * 4) 与旧接口的关系
 *    sys_mode.h 里的 sys_mode_allows_sequence() 等 4 个 inline 函数
 *    全部改为转调本模块，存量调用点零改动。
 *
 * @version V1.0
 * @date    2026-10-27
 * ============================================================
 */

#ifndef __SYS_CAPS_H
#define __SYS_CAPS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 能力位定义（16 bit，已用 12，余 4 预留）
 * ============================================================ */
#define SYS_CAP_MANUAL          (1U << 0)   /* 手动四向（回转左右/举升上下） */
#define SYS_CAP_SEQUENCE        (1U << 1)   /* 一键序列（右中/右后/回位） */
#define SYS_CAP_CALIB           (1U << 2)   /* 校准（按键流程 + MSH 命令） */
#define SYS_CAP_PARAM_WRITE     (1U << 3)   /* 参数写入（机型表/系统参数） */
#define SYS_CAP_DIAG_READ       (1U << 4)   /* 诊断只读（各类 dump/show） */
#define SYS_CAP_DIAG_WRITE      (1U << 5)   /* 危险调试写（PWM/电机/刹车/看门狗） */
#define SYS_CAP_DEBUG_OUT       (1U << 6)   /* 调试输出（日志详细度） */
#define SYS_CAP_MODE_SWITCH     (1U << 7)   /* 模式切换 */
#define SYS_CAP_LOG_READ        (1U << 8)   /* 日志读取（预留，日志功能未实现） */
#define SYS_CAP_LOG_ERASE       (1U << 9)   /* 日志清除（预留） */
#define SYS_CAP_CONFIG_SAVE     (1U << 10)  /* 配置落盘（sys_param_save） */
#define SYS_CAP_CAN_TX_TEST     (1U << 11)  /* CAN 手动发帧测试 */

#define SYS_CAP_ALL             (0x0FFFU)

/* ============================================================
 * 初始化与查询
 * ============================================================ */

/**
 * @brief 按当前模式重算能力掩码
 * @note  由 mode_actions.c 的 mode_switch_to() 在每次模式切换后调用；
 *        也必须在 mode_init 完成后调用一次以建立初值。
 *        内部读取 mode_config 的 cap_override 做收紧。
 */
void sys_caps_apply(void);

/**
 * @brief 查询当前是否具备某能力（不打印）
 * @param cap 单个能力位（可用 | 组合，表示"全部具备"）
 * @return 1=具备，0=不具备
 */
uint8_t sys_cap_has(uint16_t cap);

/**
 * @brief 判定并记录拒绝日志（执行入口统一用这个）
 * @param cap    需求能力（单个位，或 | 组合表示"全部需要"）
 * @param caller 调用者标识，如 "seq_start"、"t_pwm"，进日志
 * @return 1=允许，0=拒绝（已打印原因）
 *
 * 拒绝日志格式： [CAP] deny SEQUENCE by seq_start (mode=MAINT, caps=0x003F)
 */
uint8_t sys_cap_require(uint16_t cap, const char *caller);

/**
 * @brief 获取当前能力掩码（调试/诊断用）
 */
uint16_t sys_caps_get(void);

/**
 * @brief 获取指定模式的默认能力掩码（不叠加 override）
 */
uint16_t sys_caps_default_of(uint8_t mode);

/**
 * @brief 能力位名称（调试输出用）
 */
const char *sys_cap_name(uint16_t cap);

#ifdef __cplusplus
}
#endif

#endif /* __SYS_CAPS_H */