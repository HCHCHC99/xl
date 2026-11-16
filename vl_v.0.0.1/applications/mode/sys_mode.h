/**
 * @file    sys_mode.h
 * @brief   系统运行模式（全局快速查询接口）
 * @note    模式是系统级属性，独立于模式控制对象
 *          所有控制对象通过 sys_get_mode() 快速查询
 */

#ifndef __SYS_MODE_H
#define __SYS_MODE_H

#include <stdint.h>

/* 能力位模块（sys_caps.h 不反向包含 sys_mode.h，无循环依赖） */
#include "sys_caps.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 系统模式枚举
 * ============================================================
 * 三级模式（原 TEST 语义并入 MAINT，枚举值保持不变以兼容存量代码）：
 *   FACTORY 工厂模式：全量调试输出，用于产线测试
 *   MAINT   维护模式：日志导出、参数读写，用于现场维护
 *   WORK    工作模式：正常业务，不输出调试信息
 * ============================================================ */
typedef enum {
    SYS_MODE_FACTORY = 0,        /* 工厂模式 */
    SYS_MODE_MAINT,              /* 维护模式（原 SYS_MODE_TEST） */
    SYS_MODE_WORK,               /* 工作模式 */
    SYS_MODE_MAX
} sys_mode_t;

/* 兼容旧命名：TEST 已并入 MAINT */
#define SYS_MODE_TEST   SYS_MODE_MAINT

/* ============================================================
 * 全局模式变量（外部引用）
 * ============================================================ */
extern volatile sys_mode_t g_current_mode;

/* ============================================================
 * 快速查询接口
 * ============================================================ */

/**
 * @brief 获取当前系统模式
 */
static inline sys_mode_t sys_get_mode(void)
{
    return g_current_mode;
}

/**
 * @brief 设置当前系统模式（由 mode_control 调用）
 */
static inline void sys_set_mode(sys_mode_t mode)
{
    if (mode < SYS_MODE_MAX) {
        g_current_mode = mode;
    }
}

/**
 * @brief 是否允许一键序列
 * @note  【2026-10-27】改为转调能力位模块，权限表在 sys_caps.c。
 *        FACTORY 默认禁一键（产线不做整机行程联动），见静态配置表。
 */
static inline uint8_t sys_mode_allows_sequence(void)
{
    return sys_cap_has(SYS_CAP_SEQUENCE);
}

/**
 * @brief 是否允许校准
 * @note  【2026-10-27】改为转调能力位。WORK 默认禁校准（防现场误校准）。
 *        校准另有前置条件（REQ010 §①），本函数只管模式权限。
 */
static inline uint8_t sys_mode_allows_calibration(void)
{
    return sys_cap_has(SYS_CAP_CALIB);
}

/**
 * @brief 是否允许回转运动
 * @note  【2026-10-27】改为转调 SYS_CAP_MANUAL。三模式默认均为 Y，
 *        但保留能力位后现场可用 cap_override 收紧。
 */
static inline uint8_t sys_mode_allows_swing_motion(void)
{
    return sys_cap_has(SYS_CAP_MANUAL);
}

/**
 * @brief 是否允许详细调试输出（日志分级预留接口）
 * @note  【2026-10-27】改为转调 SYS_CAP_DEBUG_OUT。
 *        FACTORY / MAINT 为 Y，WORK 为 N。
 */
static inline uint8_t sys_mode_allows_debug(void)
{
    return sys_cap_has(SYS_CAP_DEBUG_OUT);
}

/**
 * @brief 模式名称（调试输出用）
 */
static inline const char *sys_mode_name(sys_mode_t mode)
{
    switch (mode) {
    case SYS_MODE_FACTORY: return "FACTORY";
    case SYS_MODE_MAINT:   return "MAINT";
    case SYS_MODE_WORK:    return "WORK";
    default:               return "?";
    }
}

#ifdef __cplusplus
}
#endif

#endif /* __SYS_MODE_H */
