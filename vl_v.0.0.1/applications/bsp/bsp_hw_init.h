/**
 * @file    bsp_hw_init.h
 * @brief   MCU 硬件资源初始化调度表
 * @note
 *   分层职责：
 *     - MCU 资源初始化 → 本模块（由 INIT_DEVICE_EXPORT 触发）
 *     - 抽象对象初始化 → applications/boot/boot_table.c
 *   本模块只配置寄存器与引脚，禁止出现 rt_malloc / 对象注册 / 线程创建。
 *
 *   顺序保证：
 *     表下标 = 执行顺序，不受链接顺序影响（区别于多个 INIT_*_EXPORT 并存）。
 */

#ifndef __BSP_HW_INIT_H
#define __BSP_HW_INIT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 类型定义
 * ============================================================ */

/**
 * @brief 单步硬件初始化函数
 * @param arg 由表项 arg 字段传入，可为 NULL
 * @return 0 成功；非 0 失败
 */
typedef int (*hw_init_fn_t)(void *arg);

/**
 * @brief 硬件初始化表项
 */
typedef struct {
    const char   *name;      /* 步骤名（日志与诊断用） */
    hw_init_fn_t  fn;        /* 执行体 */
    void         *arg;       /* 传给 fn 的参数，可为 NULL */
    uint8_t       critical;  /* 1 = 失败立即中止后续步骤；0 = 记录后继续 */
} hw_init_step_t;

/* ============================================================
 * 调度表（定义在 bsp_hw_init.c）
 * ============================================================ */
extern const hw_init_step_t g_hw_init_steps[];
extern const uint16_t       g_hw_init_step_count;

/**
 * @brief 失败位图
 * @note  bitN = 第 N 步失败；0 表示全部成功。
 *        由 boot_table 首步 hw_check 校验。
 */
extern uint32_t             g_hw_init_error_map;

/* ============================================================
 * 调度入口
 * ============================================================ */

/**
 * @brief 执行全部硬件初始化
 * @return 0 全部成功；-1 存在失败
 * @note  [单线程] 由 INIT_DEVICE_EXPORT 在 main 线程启动前调用。
 *        可重复调用：各步内部自带幂等保护。
 */
int bsp_hw_init(void);

/* ============================================================
 * 步骤实现（供表引用）
 * ============================================================ */

/**
 * @brief TIM4_3 + EMB 寄存器初始化
 * @param arg const hw_pwm_cfg_t *（不可为 NULL）
 */
int bsp_pwm_hw_init(void *arg);

/**
 * @brief Hall 三路 EXTI + NVIC 初始化
 * @param arg 忽略
 */
int bsp_hall_hw_init(void *arg);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_HW_INIT_H */
