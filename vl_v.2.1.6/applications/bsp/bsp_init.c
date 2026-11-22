/**
 * @file    bsp_init.c
 * @brief   BSP 层统一初始化入口（已废弃，保留空壳）
 *
 * @note    【已废弃】
 *   硬件资源初始化已迁移到 bsp_hw_init.c 的表驱动调度：
 *       INIT_DEVICE_EXPORT(bsp_hw_init)
 *
 *   迁移原因：
 *     1. 顺序可控 —— 表下标即执行顺序，不受链接顺序影响
 *     2. 错误可传播 —— 每步返回 int，失败写入 g_hw_init_error_map
 *     3. 可观测 —— 每步打印名称与结果
 *
 *   【禁止】恢复本文件的 INIT_DEVICE_EXPORT(bsp_all_init)，
 *   否则将与 bsp_hw_init() 重复初始化同一批硬件资源。
 */

#include <rtthread.h>

/**
 * @brief 兼容旧调用点，不执行任何初始化
 * @return 恒返回 0
 */
int bsp_all_init(void)
{
    rt_kprintf("[BSP] bsp_all_init() deprecated, use bsp_hw_init()\n");
    return 0;
}

/* 已废弃：见文件头说明 */
//INIT_DEVICE_EXPORT(bsp_all_init);
