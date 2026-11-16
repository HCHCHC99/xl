/**
 * @file    sys_mode.c
 * @brief   系统模式全局变量定义 + 模式切换调试命令
 */

#include "sys_mode.h"
#include "sys_caps.h"               /* 能力位门禁（2026-10-27） */

#include <string.h>
#include <rtthread.h>
#include <finsh.h>

/* 全局模式变量（默认工作模式） */
volatile sys_mode_t g_current_mode = SYS_MODE_WORK;

/* ============================================================
 * 模式切换
 * ============================================================
 * 当前为内部 MSH 命令（W-4 裁定：CAN 报文接口待客户补充协议）。
 * 注意：本命令只改 g_current_mode 快查变量，不解动 mode_control
 *       状态机；现场调试可用，产品逻辑仍应走 mode_control。
 * ============================================================ */
static int sys_mode_apply(const char *name)
{
    if (strcmp(name, "work") == 0) {
        sys_set_mode(SYS_MODE_WORK);
    } else if (strcmp(name, "factory") == 0) {
        sys_set_mode(SYS_MODE_FACTORY);
    } else if ((strcmp(name, "maint") == 0) || (strcmp(name, "test") == 0)) {
        sys_set_mode(SYS_MODE_MAINT);
    } else {
        return -1;
    }
    return 0;
}

static void t_wmode(int argc, char **argv)
{
    if (argc < 2) {
        rt_kprintf("current mode : %s (%d)\n",
                   sys_mode_name(g_current_mode), (int)g_current_mode);
        rt_kprintf("debug output : %s\n",
                   sys_mode_allows_debug() ? "ON" : "OFF");
        rt_kprintf("caps         : 0x%04X\n", (unsigned)sys_caps_get());
        rt_kprintf("usage        : t_wmode work|factory|maint\n");
        return;
    }

    /* 【2026-10-27】模式切换门禁，与 mode_set 同源 */
    if (sys_cap_require(SYS_CAP_MODE_SWITCH, "t_wmode") == 0U) { return; }

    if (sys_mode_apply(argv[1]) != 0) {
        rt_kprintf("unknown mode '%s' (work|factory|maint)\n", argv[1]);
        return;
    }

    /* 注意：t_wmode 走 sys_set_mode() 直改快查变量，不经 mode_control 状态机，
     * 因此能力掩码不会被自动重算。这里显式补一次，否则 t_caps 显示与实际
     * 模式不一致（看似切换成功、权限却还是旧的）。 */
    sys_caps_apply();

    rt_kprintf("[MODE] -> %s, debug output %s\n",
               sys_mode_name(g_current_mode),
               sys_mode_allows_debug() ? "ON" : "OFF");
}

MSH_CMD_EXPORT_ALIAS(t_wmode, t_wmode, switch mode: work|factory|maint);
