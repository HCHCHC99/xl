/**
 * @file    main.c
 * @brief   系统入口
 * @note    仅启动 IPC → 自动初始化 → 创建双线程
 */

#include <rtthread.h>
#include "ipc_center.h"
#include "framework/state_engine/sm_assert.h"

#include "boot/boot.h"
#include "ota_shared/ota_shared.h"

/* ============================================================
 * 外部函数声明（由各模块通过 INIT_APP_EXPORT 实现）
 * ============================================================ */
/* storage.c */
extern int storage_all_init(void);
/* device_info.c */
extern int device_info_init(void);
/* ota_interface.c */
extern int ota_interface_init(void);
/* lift/vert_control.c */
extern int lift_init(void);
/* can/ut_can_scheduler.c + ut_can_send.c */
extern int can_init(void);
/* bg_scheduler.c */
extern int bg_scheduler_init(void);

/* ============================================================
 * 外部函数声明（需要显式启动的组件）
 * ============================================================ */
/* can/ut_can_scheduler.c */
extern void can_scheduler_start(void);

/* ============================================================
 * 外部线程入口声明
 * ============================================================ */
/* app_main.c */
extern void app_main_thread_entry(void *param);
/* bg_scheduler.c */
extern void bg_scheduler_thread_entry(void *param);

/* ============================================================
 * main() - 系统入口
 * ============================================================ */

/* 板级/驱动层仍走 RT-Thread 自动初始化（INIT_BOARD/DEVICE_EXPORT）
 * 应用层全部收敛到 boot 表 */
int main(void)
{
    if (boot_run_all() != 0) {
        rt_kprintf("[Main] BOOT FAILED - halted for debug\n");
        rt_kprintf("[Main] run 't_boot_dump' to inspect\n");
        while (1) rt_thread_mdelay(1000);
    }
    rt_kprintf("[Main] System started successfully.\n");
    return 0;
}
