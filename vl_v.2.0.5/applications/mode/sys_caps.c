/**
 * @file    sys_caps.c
 * @brief   系统模式能力位实现（静态权限表 + 单点判定）
 *
 * 权唯一权威来源：docs/02-设计/模式权限静态配置表-V1.0.md
 * 改权限只改本文件的 g_caps_table[]，不改任何调用点。
 *
 * @version V1.0
 * @date    2026-10-27
 */

#include "sys_caps.h"
#include "sys_mode.h"
#include "mode_config.h"
#include "mode_control.h"

#include <rtthread.h>
#include <stdlib.h>
#include <finsh.h>

/* ============================================================
 * 三张默认权限表
 * ============================================================
 * 依据 2026-10-27 用户裁定：
 *   MAINT  功能最全（唯一全开），作为调试验证期默认模式
 *   FACTORY 禁一键（产线用单项动作验证，不做整机行程联动）
 *   WORK    禁校准 / 禁参数写 / 禁危险调试
 * ============================================================ */

/* FACTORY：除了"一键序列"全开。
 * 理由：产线逐项验证各执行器，但不跑整机行程联动（关两轴 + 升降，介入风险大）。 */
#define CAPS_FACTORY    (SYS_CAP_ALL & ~(SYS_CAP_SEQUENCE))

/* MAINT：唯一全开。调试、产线、售后维护都在这。
 * 注意 SYS_CAP_MODE_SWITCH 受 factory_lock 单独约束，在全开表里但 apply 时再判。 */
#define CAPS_MAINT      (SYS_CAP_ALL)

/* WORK：正常业务。
 * 禁 CALIB（防现场误校准，校准须进 MAINT）
 * 禁 PARAM_WRITE（机型/系统参数不可现场改）
 * 禁 DIAG_WRITE（t_pwm / t_motor_start_* / t_brake_unlock / t_wdt_off 等 12 条危险命令）
 * 禁 CONFIG_SAVE（参数不允许落盘）
 * 禁 CAN_TX_TEST（t_can_tx 手动发帧会扰乱总线）
 * 禁 LOG_ERASE（日志不可现场清）
 * 保留 MANUAL / SEQUENCE / DIAG_READ / LOG_READ
 *      + MODE_SWITCH（见下）
 *
 * 【2026-10-27 修正】MODE_SWITCH 必须在 WORK 里置位。
 * 原实现 WORK 表不含该位 ⇒ 一旦进入 WORK，mode_set / t_wmode 全部被拒，
 * 控制台无法再回 MAINT，只能断电重启。而《模式权限静态配置表-V1.0》
 * M-01 / M-03 对 WORK 标注的是「C（factory_lock=0）」，即有条件下允许，
 * 不是禁止。故此处置位，条件由 sys_caps_apply() 里的 factory_lock 单独收紧。 */
#define CAPS_WORK       (SYS_CAP_MANUAL     | \
                         SYS_CAP_SEQUENCE   | \
                         SYS_CAP_DIAG_READ  | \
                         SYS_CAP_LOG_READ   | \
                         SYS_CAP_MODE_SWITCH)

static const uint16_t g_caps_table[SYS_MODE_MAX] = {
    [SYS_MODE_FACTORY] = CAPS_FACTORY,
    [SYS_MODE_MAINT]   = CAPS_MAINT,
    [SYS_MODE_WORK]    = CAPS_WORK,
};

/* ============================================================
 * 运行时能力掩码
 * ============================================================ */
static uint16_t s_sys_caps = 0U;

/* ============================================================
 * 内部：取当前模式的 cap_override
 * ============================================================ */
static uint8_t caps_get_override(void)
{
    const mode_context_t *mctx = mode_control_get_context();

    if (mctx == RT_NULL) {
        return 0U;
    }

    /* reserved1 已复名为 cap_override（mode_config.h）。
     * 0 = 不覆盖，用默认表。 */
    return mctx->config.cap_override;
}

/* ============================================================
 * 内部：把 8 位的 cap_override 展开为 16 位掩码
 * ============================================================
 * cap_override 只有 1 字节，只覆盖低 8 位能力（MANUAL..MODE_SWITCH）。
 * 高 4 位（LOG_READ / LOG_ERASE / CONFIG_SAVE / CAN_TX_TEST）无法被
 * 现场裁剪，恒按默认表执行 —— 这是有意的设计：现场只暴露最常用的
 * 8 个开关，避免一个字节被误填后把日志/落盘能力整个关掉。
 *
 * 【关键】必须把高 4 位补成 1，否则 base & 0x00FF 会把高 4 位全部误清。
 * ============================================================ */
static uint16_t caps_expand_override(uint8_t ovr)
{
    return (uint16_t)(0xFF00U | (uint16_t)ovr);
}

/* ============================================================
 * 公开 API
 * ============================================================ */
void sys_caps_apply(void)
{
    sys_mode_t mode = sys_get_mode();
    uint16_t   base;
    uint8_t    ovr;

    if ((uint32_t)mode >= (uint32_t)SYS_MODE_MAX) {
        mode = SYS_MODE_WORK;       /* 非法值回退最保守的业务模式 */
    }

    base = g_caps_table[mode];
    ovr  = caps_get_override();

    /* 覆盖只能收紧：默认表该位为 0 时，override 置 1 也不生效（按位与保证）。
     * 高 4 位不受 override 影响，见 caps_expand_override() 说明。 */
    s_sys_caps = (ovr != 0U) ? (uint16_t)(base & caps_expand_override(ovr))
                             : base;

    /* MODE_SWITCH 单独受 factory_lock 约束：
     * 锁上后 WORK/MAINT 不能切到 FACTORY。
     * 此处只做"是否有切换权限"的粗判，具体能否切到某模式由
     * mode_control_set_mode() 内部再判一次目标模式。 */
    if (mode == SYS_MODE_WORK) {
        const mode_context_t *mctx = mode_control_get_context();
        if ((mctx == RT_NULL) || (mctx->config.factory_lock != 0U)) {
            s_sys_caps &= (uint16_t)(~SYS_CAP_MODE_SWITCH);
        }
    }
}

uint8_t sys_cap_has(uint16_t cap)
{
    if (cap == 0U) {
        return 1U;      /* 不要求任何能力 */
    }
    return ((s_sys_caps & cap) == cap) ? 1U : 0U;
}

const char *sys_cap_name(uint16_t cap)
{
    switch (cap) {
    case SYS_CAP_MANUAL:      return "MANUAL";
    case SYS_CAP_SEQUENCE:    return "SEQUENCE";
    case SYS_CAP_CALIB:       return "CALIB";
    case SYS_CAP_PARAM_WRITE: return "PARAM_WRITE";
    case SYS_CAP_DIAG_READ:   return "DIAG_READ";
    case SYS_CAP_DIAG_WRITE:  return "DIAG_WRITE";
    case SYS_CAP_DEBUG_OUT:   return "DEBUG_OUT";
    case SYS_CAP_MODE_SWITCH: return "MODE_SWITCH";
    case SYS_CAP_LOG_READ:    return "LOG_READ";
    case SYS_CAP_LOG_ERASE:   return "LOG_ERASE";
    case SYS_CAP_CONFIG_SAVE: return "CONFIG_SAVE";
    case SYS_CAP_CAN_TX_TEST: return "CAN_TX_TEST";
    default:                  return "MULTI";
    }
}

uint8_t sys_cap_require(uint16_t cap, const char *caller)
{
    if (sys_cap_has(cap) != 0U) {
        return 1U;
    }

    /* 统一拒绝日志。格式固定，便于后续运行日志功能直接抓这一行。 */
    rt_kprintf("[CAP] deny %s by %s (mode=%s, caps=0x%04X)\n",
               sys_cap_name(cap),
               (caller != RT_NULL) ? caller : "?",
               sys_mode_name(sys_get_mode()),
               (unsigned)s_sys_caps);
    return 0U;
}

uint16_t sys_caps_get(void)
{
    return s_sys_caps;
}

uint16_t sys_caps_default_of(uint8_t mode)
{
    if ((uint32_t)mode >= (uint32_t)SYS_MODE_MAX) {
        return 0U;
    }
    return g_caps_table[mode];
}

/* ============================================================
 * FinSH 调试命令：t_caps
 * ============================================================
 * 用法：
 *   t_caps            显示当前模式、能力掩码、三模式默认表
 *   t_caps <mode>     显示指定模式的默认表（0=FACTORY 1=MAINT 2=WORK）
 * ============================================================ */
static void caps_print_mask(uint16_t mask)
{
    rt_kprintf("0x%04X [", (unsigned)mask);

    /* 按位序打印，只打置位的 */
    {
        static const uint16_t bits[] = {
            SYS_CAP_MANUAL, SYS_CAP_SEQUENCE, SYS_CAP_CALIB, SYS_CAP_PARAM_WRITE,
            SYS_CAP_DIAG_READ, SYS_CAP_DIAG_WRITE, SYS_CAP_DEBUG_OUT, SYS_CAP_MODE_SWITCH,
            SYS_CAP_LOG_READ, SYS_CAP_LOG_ERASE, SYS_CAP_CONFIG_SAVE, SYS_CAP_CAN_TX_TEST
        };
        uint32_t i;
        uint8_t  first = 1U;

        for (i = 0U; i < (sizeof(bits) / sizeof(bits[0])); i++) {
            if ((mask & bits[i]) != 0U) {
                rt_kprintf("%s%s", first ? "" : " ", sys_cap_name(bits[i]));
                first = 0U;
            }
        }
        if (first != 0U) {
            rt_kprintf("-");
        }
    }

    rt_kprintf("]\n");
}

static void t_caps(int argc, char **argv)
{
    uint8_t mode;

    if (argc >= 2) {
        mode = (uint8_t)atoi(argv[1]);
        if (mode >= SYS_MODE_MAX) {
            rt_kprintf("invalid mode %u (0=FACTORY 1=MAINT 2=WORK)\n",
                       (unsigned)mode);
            return;
        }
        rt_kprintf("%-8s default : ", sys_mode_name((sys_mode_t)mode));
        caps_print_mask(sys_caps_default_of(mode));
        return;
    }

    rt_kprintf("\n===== SYSTEM CAPABILITIES =====\n");
    rt_kprintf("mode      : %s (%d)\n",
               sys_mode_name(sys_get_mode()), (int)sys_get_mode());
    rt_kprintf("active    : ");
    caps_print_mask(s_sys_caps);

    rt_kprintf("override  : 0x%02X (%s)\n",
               (unsigned)caps_get_override(),
               (caps_get_override() == 0U) ? "none, using default table" : "narrowing");

    rt_kprintf("---------- defaults ----------\n");
    for (mode = 0U; mode < (uint8_t)SYS_MODE_MAX; mode++) {
        rt_kprintf("%-8s : ", sys_mode_name((sys_mode_t)mode));
        caps_print_mask(sys_caps_default_of((uint8_t)mode));
    }
    rt_kprintf("===============================\n\n");
}

MSH_CMD_EXPORT_ALIAS(t_caps, t_caps, show system capability masks);