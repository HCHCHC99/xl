/**
 *******************************************************************************
 * @file    wdt_monitor.h
 * @brief   SWDT 硬件看门狗监控模块 —— idle 线程喂狗 + 可选的多客户端签到
 * @note    喂狗位置：rt_thread_idle_sethook() 注册的 idle 钩子
 *
 *          设计语义：
 *            基础模式（签到关闭）：只要 CPU 有余量调度到 idle 就喂狗
 *                                  → 【CPU 活性检测器】
 *            签到模式（签到开启）：所有已注册关键线程按期签到才喂狗
 *                                  → 【关键线程活性检测器】
 *
 *          能覆盖：高优先级线程死循环不让出 CPU / 中断风暴 / 全局关中断 /
 *                  调度器被破坏 / 系统时钟跑飞（SWDT 用内部独立时钟，
 *                  不依赖 PCLK，主时钟跑飞时仍能复位）
 *          签到模式下额外覆盖：关键线程永久阻塞在等待原语上
 *          不覆盖：未注册签到的线程发生阻塞（需自行注册）
 *
 *          三条硬约束（维护时务必遵守）：
 *            1) idle 钩子内【禁止】rt_kprintf / 浮点 / 大局部变量
 *               —— 本工程 IDLE_THREAD_STACK_SIZE = 1024，且钩子执行频率极高
 *            2) SWDT 刷新窗口必须为「全窗口可刷新」
 *               —— ICG 默认 ICG_SWDT_RANGE_0TO100PCT，勿改为窗口模式
 *            3) SWDT 精确超时值不可运行时设定，仅作「最后保险」
 *               —— 精确超时判定由本模块签到机制（rt_tick_get）负责
 *
 * @version V1.0
 * @date    2026-10-20
 *******************************************************************************
 */

#ifndef __WDT_MONITOR_H
#define __WDT_MONITOR_H

#include <rtthread.h>       /* 引入 rtconfig.h，供下方编译期检查使用 */

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 编译期依赖检查
 * ============================================================
 * idle 钩子需在 rtconfig.h 中打开 RT_USING_IDLE_HOOK
 * （RT-Thread Studio：RT-Thread Settings → Kernel → Idle hook）
 * ============================================================ */
#if !defined(RT_USING_IDLE_HOOK)
#error "wdt_monitor: 需在 rtconfig.h 打开 RT_USING_IDLE_HOOK（Kernel -> Idle hook）"
#endif

/* ============================================================
 * 常量定义
 * ============================================================ */

/**
 * @brief 签到客户端数量上限
 * @note  可按需覆盖，但须保证足够容纳所有需要监控的关键线程
 */
#ifndef WDT_CLIENT_MAX
#define WDT_CLIENT_MAX      (8)
#endif

/** @brief 无效的客户端 ID */
#define WDT_CLIENT_INVALID  (-1)

/* ============================================================
 * 类型定义
 * ============================================================ */

/**
 * @brief 复位原因枚举
 */
typedef enum {
    WDT_RESET_POR  = 0,     /*!< 上电复位（或非看门狗引起的复位） */
    WDT_RESET_SWDT = 1,     /*!< SWDT 溢出复位                     */
} wdt_reset_cause_t;

/* ============================================================
 * 生命周期接口
 * ============================================================ */

/**
 * @brief  初始化并启动看门狗
 * @note   调用时机：所有业务线程创建完毕之后
 *         本工程由 boot 表最后一步（BOOT_LV_WDT）调用
 *         内部顺序：读复位原因 → 注册 idle 钩子 → 首次喂狗（启动计数）
 *                   → 清除复位标志
 * @retval 0    成功
 * @retval <0   RT-Thread 负错误码（-RT_EFULL：idle 钩子列表已满）
 */
int wdt_monitor_init(void);

/**
 * @brief  查询本次上电的复位原因
 * @retval WDT_RESET_SWDT : 上次复位由看门狗溢出引起
 * @retval WDT_RESET_POR  : 上次复位为其他原因（上电/引脚/欠压等）
 */
wdt_reset_cause_t wdt_monitor_get_reset_cause(void);

/* ============================================================
 * 签到机制接口（默认关闭，按需启用）
 * ============================================================
 * 启用后：所有已注册且已使能的客户端须在 timeout_ms 内签到，
 *         否则停止喂狗，等待 SWDT 溢出复位。
 * ============================================================ */

/**
 * @brief  注册一个签到客户端
 * @param  [in] name        客户端名称，须为静态字符串（仅用于调试显示）
 * @param  [in] timeout_ms  允许的最大签到间隔，超时判为过期
 * @retval >=0                    客户端 ID
 * @retval WDT_CLIENT_INVALID(-1) 失败（名称为空 / 超时为 0 / 表已满）
 * @note   timeout_ms 建议取实际周期的 3 倍以上，避免调度抖动误判
 */
int8_t wdt_client_register(const char *name, uint32_t timeout_ms);

/**
 * @brief  签到（各关键线程周期调用）
 * @param  [in] id  wdt_client_register() 返回的客户端 ID
 * @note   任意线程上下文均可调用；内部仅一次 tick 读取 + 一次赋值，
 *         开销极小，在 ISR 中调用亦安全
 */
void wdt_client_checkin(int8_t id);

/**
 * @brief  注销客户端（不再参与签到检查）
 * @param  [in] id  客户端 ID
 */
void wdt_client_unregister(int8_t id);

/**
 * @brief  启用 / 禁用签到检查
 * @param  [in] enable  1 = 启用（过期即停止喂狗）；0 = 禁用（退回基础模式）
 * @note   默认为禁用。建议先用 MSH 命令 t_wdt_on 试跑验证，
 *         确认不会误复位后再在代码中固化启用
 */
void wdt_checkin_enable(uint8_t enable);

/**
 * @brief  查询签到检查是否已启用
 * @retval 1 = 已启用；0 = 已禁用
 */
uint8_t wdt_checkin_is_enabled(void);

/* ============================================================
 * 诊断接口
 * ============================================================ */

/**
 * @brief  打印看门狗状态（含各客户端签到情况）
 * @note   仅在 MSH 命令上下文中调用
 *         禁止在 idle 钩子或中断上下文中调用
 */
void wdt_monitor_dump(void);

#ifdef __cplusplus
}
#endif

#endif /* __WDT_MONITOR_H */
