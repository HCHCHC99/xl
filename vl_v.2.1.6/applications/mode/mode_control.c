/**
 * @file    mode_control.c
 * @brief   模式控制对象实现
 */

#include "mode_control.h"
#include "mode_actions.h"
#include "sys_mode.h"
#include "sys_caps.h"               /* 能力位门禁（2026-10-27） */
#include "framework/state_engine/sm_assert.h"
#include "app_event.h"
#include "../framework/manager/ctrl_manager.h"
#include <string.h>
#include <stdlib.h>
#include <rtthread.h>

/* ============================================================
 * 全局对象实例
 * ============================================================ */
mode_control_t g_mode_obj;

/* ============================================================
 * 外部引用：转移表配置
 * ============================================================ */
extern const StateMachineConfig_t g_mode_config;

/* ============================================================
 * 初始化并注册
 * ============================================================ */
int mode_control_register(ctrl_manager_t *mgr)
{
    rt_err_t ret;

    SM_ASSERT_PTR(mgr);

    /* 1. 初始化上下文 */
    memset(&g_mode_obj.ctx, 0, sizeof(mode_context_t));

    /* 2. 加载配置 */
    if (mode_config_load(&g_mode_obj.ctx.config) != 0) {
        rt_kprintf("[Mode] Config load failed, using default\n");
    }

    /* 3. 设置初始模式（INIT 状态） */
    g_mode_obj.ctx.current_mode = MODE_STATE_INIT;

    /* 4. 绑定状态机指针（关键：先绑定，再调用 control_object_init） */
    g_mode_obj.base.sm = &g_mode_obj.sm_instance;

    /* 5. 单线程运行时配置 */
    ctrl_obj_runtime_cfg_t runtime = {
        .use_thread  = 0,      /* 共享主线程 */
        .thread_prio = 0,
        .stack_size  = 0,
        .mq_size     = 0,
    };

    /* 6. 初始化控制对象 */
    ret = control_object_init(&g_mode_obj.base,
                              "ModeControl",
                              CTRL_OBJ_TYPE_MODE,
                              &g_mode_config,
                              &g_mode_obj.ctx,
                              &runtime);
    if (ret != RT_EOK) {
        rt_kprintf("[Mode] control_object_init failed (%d)\n", ret);
        return -1;
    }

    /* 7. 注册到管理器 */
    ret = ctrl_manager_register(mgr, &g_mode_obj.base);
    if (ret != 0) {
        rt_kprintf("[Mode] ctrl_manager_register failed (%d)\n", ret);
        return -1;
    }

    rt_kprintf("[Mode] Mode control registered.\n");
    return 0;
}

/* ============================================================
 * 查询接口
 * ============================================================ */
mode_context_t* mode_control_get_context(void)
{
    return &g_mode_obj.ctx;
}

/* ============================================================
 * 调试接口：设置模式
 * ============================================================ */
void mode_control_set_mode(sys_mode_t mode)
{
    app_event_t evt;

    switch (mode) {
        case SYS_MODE_FACTORY: evt = MODE_EVT_ENTER_FACTORY; break;
        case SYS_MODE_TEST:    evt = MODE_EVT_ENTER_TEST;    break;
        case SYS_MODE_WORK:    evt = MODE_EVT_ENTER_WORK;    break;
        default: return;
    }

    control_object_emit(&g_mode_obj.base, evt);
}

/* ============================================================
 * FinSH 调试命令
 * ============================================================ */
#ifdef RT_USING_FINSH
#include <finsh.h>

static void cmd_mode_set(int argc, char **argv)
{
    if (argc < 2) {
        rt_kprintf("Usage: mode_set <0=factory|1=test|2=work>\n");
        rt_kprintf("Current mode: %d\n", sys_get_mode());
        return;
    }

    int mode = atoi(argv[1]);
    if (mode < 0 || mode >= SYS_MODE_MAX) {
        rt_kprintf("Invalid mode: %d\n", mode);
        return;
    }

    /* 【2026-10-27】模式切换门禁。WORK 下 MODE_SWITCH 置位（可切回），
     * 但 factory_lock=1 时被 sys_caps_apply() 收紧 ⇒ 此处统一判定。 */
    if (sys_cap_require(SYS_CAP_MODE_SWITCH, "mode_set") == 0U) { return; }

    mode_control_set_mode((sys_mode_t)mode);
    rt_kprintf("Mode set to %d\n", mode);
}
MSH_CMD_EXPORT_ALIAS(cmd_mode_set, mode_set, set system mode);

static void cmd_mode_show(int argc, char **argv)
{
    mode_context_t *ctx = mode_control_get_context();
    rt_kprintf("Current mode: %d\n", sys_get_mode());
    rt_kprintf("Enter tick:   %u\n", ctx->mode_enter_tick);
    rt_kprintf("Counts: factory=%u, test=%u, work=%u\n",
               ctx->factory_enter_count,
               ctx->test_enter_count,
               ctx->work_enter_count);
}
MSH_CMD_EXPORT_ALIAS(cmd_mode_show, mode_show, show current mode);
#endif
