/**
 * @file    ut_can_init.c
 * @brief   CAN 模块初始化（INIT_APP_EXPORT 入口）
 */

#include "ut_can_scheduler.h"

/* 【2026-11-10 彩标 P3】CAN 初始化：完成=OK */
#include "../log/log_color.h"
#include "ut_can_send.h"
#include "framework/state_engine/sm_assert.h"
#include <rtthread.h>

/* ============================================================
 * CAN 模块初始化
 * ============================================================ */
int can_init(void)
{
    rt_kprintf("[CAN] can_init()\n");
    /* 1. 初始化 CAN 接收调度器 */
    can_scheduler_init();

    /* 2. 初始化 CAN 发送模块 */
    can_send_init();

    LOG_OK("[CAN] Module initialized.\n");

    can_scheduler_start();
    return 0;
}
//INIT_APP_EXPORT(can_init);
