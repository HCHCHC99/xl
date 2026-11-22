/**
 * @file    seq_init.c
 * @brief   序列控制初始化
 */

#include "seq_control.h"
#include "framework/state_engine/sm_assert.h"
#include "../framework/manager/ctrl_manager.h"

/* 【2026-11-10 彩标 P2】序列初始化：注册失败=ERR，完成=OK */
#include "../log/log_color.h"
#include <rtthread.h>

extern ctrl_manager_t g_ctrl_mgr;

int seq_init(void)
{
    rt_kprintf("[seq] seq_init()\n");
    rt_err_t ret;

    ret = seq_control_register(&g_ctrl_mgr);
    if (ret != 0) {
        LOG_ERR("[Seq] Register failed\n");
        return -1;
    }

    LOG_OK("[Seq] Initialized.\n");
    return 0;
}
//INIT_APP_EXPORT(seq_init);
