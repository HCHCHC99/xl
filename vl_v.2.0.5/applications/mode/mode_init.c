/**
 * @file    mode_init.c
 * @brief   模式控制对象统一初始化入口
 */

#include "mode_control.h"
#include "sys_mode.h"
#include "sys_caps.h"
#include "app_event.h"

/* 【2026-11-10 彩标 P2】模式初始化：注册失败=ERR */
#include "../log/log_color.h"
#include "framework/state_engine/sm_assert.h"
#include "../framework/manager/ctrl_manager.h"
#include <rtthread.h>

/* 外部引用：控制对象管理器 */
extern ctrl_manager_t g_ctrl_mgr;

/* ============================================================
 * 验证期强制上电模式（2026-10-27 用户裁定）
 * ============================================================
 * 1 = 忽略 flash 配置，上电无条件进入 MAINT（维护模式，功能最全）
 * 0 = 按 mode_config 的 default_mode 决定
 *
 * 【重要】量产必须改回 0，否则所有机器上电都停在维护模式。
 * 已登记待办：量产前复核此项。
 * ============================================================ */
#ifndef MODE_BOOT_FORCE_MAINT
#define MODE_BOOT_FORCE_MAINT   1
#endif

/* ============================================================
 * 模式控制初始化
 * ============================================================ */
int mode_init(void)
{
    rt_kprintf("[SWING] mode_init()\n");
    rt_err_t ret;

    /* 1. 注册模式控制对象到管理器 */
    ret = mode_control_register(&g_ctrl_mgr);
    if (ret != 0) {
        LOG_ERR("[Mode] Register failed\n");
        return -1;
    }

    /* 2. 进入默认模式（根据配置） */
    mode_context_t *mctx = mode_control_get_context();
    sys_mode_t default_mode = (sys_mode_t)mctx->config.default_mode;

    if (default_mode >= SYS_MODE_MAX) {
        default_mode = SYS_MODE_WORK;
    }

#if MODE_BOOT_FORCE_MAINT
    /* 验证期：无视配置，直接进 MAINT。
     * 注意 SYS_MODE_MAINT 与 SYS_MODE_TEST 同值（sys_mode.h:33 兼容宏），
     * mode_control_set_mode() 内部映射到 MODE_EVT_ENTER_TEST。 */
    rt_kprintf("[Mode] FORCE MAINT (config says %d, overridden)\n",
               (int)default_mode);
    default_mode = SYS_MODE_MAINT;
#endif

    rt_kprintf("[Mode] Entering default mode: %d\n\n", default_mode);
    mode_control_set_mode(default_mode);

    /* 【2026-10-27】建立能力掩码初值。
     * 正常情况下 mode_switch_to() 已经调过 sys_caps_apply()，
     * 但若 default_mode 与当前状态相同（如 INIT 未迁移成功），
     * 迁移会被跳过，这里兜底再算一次，保证初值确定。 */
    sys_caps_apply();

    rt_kprintf("[Mode] Capability mask = 0x%04X\n\n",
               (unsigned)sys_caps_get());

    return 0;
}
//INIT_APP_EXPORT(mode_init);
