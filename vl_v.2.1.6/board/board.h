/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 * Copyright (c) 2022, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-04-28     CDT          first version
 */

#ifndef __BOARD_H__
#define __BOARD_H__

#include <rtthread.h>
#include "hc32_ll.h"
#include "drv_gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

#define HC32_SRAM_SIZE                  (188)
#define HC32_SRAM_END                   (0x1FFF8000 + HC32_SRAM_SIZE * 1024)

#ifdef __ARMCC_VERSION
extern int Image$$RW_IRAM1$$ZI$$Limit;
#define HEAP_BEGIN                      (&Image$$RW_IRAM1$$ZI$$Limit)
#elif __ICCARM__
#pragma section="HEAP"
#define HEAP_BEGIN                      (__segment_end("HEAP"))
#else
/* 【2026-10-27 修复 · D-57】原为 (&__bss_end)，会让 RT-Thread 堆
 * 覆盖启动文件的 .stack 段（MSP 栈区），线程栈与 MSP 互相踩踏。
 *
 * 内存布局（修复前）：
 *   __bss_end       = 0x1FFFB894  ← 原 HEAP_BEGIN
 *   .stack (MSP)    = 0x1FFFD898 ~ 0x1FFFF898   (8 KB, 由 __StackTop 向下)
 *   __StackTop      = 0x1FFFF898
 *   HEAP_END        = 0x20027000  (HC32_SRAM_END)
 *   → 堆范围 0x1FFFB894~0x20027000 把 MSP 栈区整段吞掉
 *
 * 修复后：堆从 __StackTop 开始，MSP 栈区独立且完整。
 *   MSP    : 0x1FFFD898 ~ 0x1FFFF898  (8 KB，向下增长)
 *   RT 堆  : 0x1FFFF898 ~ 0x20027000  (161,640 B ≈ 158 KB，向上增长)
 *   两者零重叠，线程栈(PSP)在堆内，中断(MSP)在栈区，永不交叉。
 *
 * 注意：__StackTop 由 startup_hc32f460.S 的 .stack 段定义，
 *       必须用 extern 声明后取地址，不能直接当值使用。 */
extern char __StackTop;
#define HEAP_BEGIN                      (&__StackTop)
#endif

#define HEAP_END                        HC32_SRAM_END

#ifdef __cplusplus
}
#endif

#endif
