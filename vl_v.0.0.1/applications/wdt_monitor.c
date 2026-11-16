/**
 *******************************************************************************
 * @file    wdt_monitor.c
 * @brief   HC32F460 SWDT 看门狗监控实现 —— idle 线程喂狗 + 多客户端签到
 * @note    维护前请先阅读下方「SWDT 超时说明」与「硬约束」
 *
 * ============================================================
 * 【SWDT 超时说明】务必先读
 * ------------------------------------------------------------
 * HC32F460 的 SWDT 不在运行时配置。其时钟分频、计数周期、
 * 刷新窗口、异常类型全部由 ICG（Initial Configuration）决定，
 * 即 Flash 0x400 处的常量表，上电复位时由硬件加载。
 *
 * 本工程默认值（见 libraries/.../inc/hc32_ll_icg.h）：
 *   ICG_RB_SWDT_AUTS = ICG_SWDT_RST_STOP         复位后停止，首次刷新启动
 *   ICG_RB_SWDT_ITS  = ICG_SWDT_EXP_TYPE_RST     溢出触发复位（非中断）
 *   ICG_RB_SWDT_PERI = ICG_SWDT_CNT_PERIOD65536  65536 个计数周期
 *   ICG_RB_SWDT_CKS  = ICG_SWDT_CLK_DIV2048      计数时钟 = 源时钟 / 2048
 *   ICG_RB_SWDT_WDPT = ICG_SWDT_RANGE_0TO100PCT  全窗口可刷新
 *
 * 因此实际超时 ≈ 65536 / (源时钟 / 2048)，取决于芯片内部低速时钟，
 * 偏差较大且【无法在运行时修改】。
 *
 * 两条重要结论：
 *   1) 不要依赖 SWDT 的精确超时值，把它当「最后保险」；
 *      精确的超时判定由本模块签到机制（rt_tick_get）负责。
 *   2) 实测超时的方法：上电后停止一切喂狗，用 RTT 日志计时，
 *      观察实际复位时刻，据此反推内部时钟频率。
 * ============================================================
 *
 * 【硬约束】
 *   1) idle 钩子内禁止 rt_kprintf / 浮点 / 大局部变量
 *   2) SWDT 刷新窗口必须为全窗口可刷新，勿改为窗口模式
 *   3) 首次喂狗即启动计数，因此「注册钩子」必须早于「首次喂狗」
 *
 * @version V1.0
 * @date    2026-10-20
 *******************************************************************************
 */

/* ============================================================
 * 头文件
 * ============================================================ */
#include <rtthread.h>
#include <finsh.h>
#include <stdlib.h>                 /* atoi() */

#include "wdt_monitor.h"
#include "mode/sys_caps.h"          /* 能力位门禁（2026-10-27） */

/* HC32F460 底层驱动 */
#include "hc32_ll.h"
#include "hc32_ll_swdt.h"           /* SWDT_FeedDog / SWDT_GetCountValue */
#include "hc32_ll_rmu.h"            /* RMU_GetStatus / RMU_ClearStatus    */

/* ============================================================
 * 版本适配区
 * ============================================================
 * 不同 hc32_ll 版本的喂狗接口命名可能不同：
 *   本工程（hc32_ll_swdt.h）为 SWDT_FeedDog()
 *   若使用其他 DDL 版本，仅需修改本宏
 * ============================================================ */
#define WDT_FEED()                  SWDT_FeedDog()

/* ============================================================
 * 参数定义
 * ============================================================ */

/** @brief 签到检查节流间隔（ms）
 *  @note  idle 钩子执行频率极高，全表扫描需节流，避免无谓开销 */
#define WDT_CHECK_INTERVAL_MS       (50U)

/* ============================================================
 * 模块状态（内部）
 * ============================================================ */

/** @brief 1 = 计数器已启动，钩子可执行喂狗 */
static volatile uint8_t  s_wdt_ready   = 0;

/** @brief 本次上电的复位原因 */
static volatile uint8_t  s_reset_cause = WDT_RESET_POR;

/** @brief 本次上电的 RMU 原始复位标志（供诊断，RMU_ClearStatus 前采样） */
static volatile uint32_t s_rmu_flags = 0U;

/* 采样全部 RMU 复位标志 */
static uint32_t wdt_sample_rmu_flags(void)
{
    uint32_t f = 0U;

    if (RMU_GetStatus(RMU_FLAG_PWR_ON)         == SET) f |= RMU_FLAG_PWR_ON;
    if (RMU_GetStatus(RMU_FLAG_PIN)            == SET) f |= RMU_FLAG_PIN;
    if (RMU_GetStatus(RMU_FLAG_BROWN_OUT)      == SET) f |= RMU_FLAG_BROWN_OUT;
    if (RMU_GetStatus(RMU_FLAG_PVD1)           == SET) f |= RMU_FLAG_PVD1;
    if (RMU_GetStatus(RMU_FLAG_PVD2)           == SET) f |= RMU_FLAG_PVD2;
    if (RMU_GetStatus(RMU_FLAG_WDT)            == SET) f |= RMU_FLAG_WDT;
    if (RMU_GetStatus(RMU_FLAG_SWDT)           == SET) f |= RMU_FLAG_SWDT;
    if (RMU_GetStatus(RMU_FLAG_PWR_DOWN)       == SET) f |= RMU_FLAG_PWR_DOWN;
    if (RMU_GetStatus(RMU_FLAG_SW)             == SET) f |= RMU_FLAG_SW;
    if (RMU_GetStatus(RMU_FLAG_MPU_ERR)        == SET) f |= RMU_FLAG_MPU_ERR;
    if (RMU_GetStatus(RMU_FLAG_RAM_PARITY_ERR) == SET) f |= RMU_FLAG_RAM_PARITY_ERR;
    if (RMU_GetStatus(RMU_FLAG_RAM_ECC)        == SET) f |= RMU_FLAG_RAM_ECC;
    if (RMU_GetStatus(RMU_FLAG_CLK_ERR)        == SET) f |= RMU_FLAG_CLK_ERR;
    if (RMU_GetStatus(RMU_FLAG_XTAL_ERR)       == SET) f |= RMU_FLAG_XTAL_ERR;
    if (RMU_GetStatus(RMU_FLAG_MX)             == SET) f |= RMU_FLAG_MX;

    return f;
}

/* 打印 RMU 复位标志（仅线程上下文） */
static void wdt_print_rmu_flags(uint32_t f)
{
    rt_kprintf("RMU flags=0x%08X:", (unsigned)f);
    if (f & RMU_FLAG_PWR_ON)         rt_kprintf(" POR");
    if (f & RMU_FLAG_PIN)            rt_kprintf(" PIN");
    if (f & RMU_FLAG_BROWN_OUT)      rt_kprintf(" BOR");
    if (f & RMU_FLAG_PVD1)           rt_kprintf(" PVD1");
    if (f & RMU_FLAG_PVD2)           rt_kprintf(" PVD2");
    if (f & RMU_FLAG_WDT)            rt_kprintf(" WDT");
    if (f & RMU_FLAG_SWDT)           rt_kprintf(" SWDT");
    if (f & RMU_FLAG_PWR_DOWN)       rt_kprintf(" PWRDOWN");
    if (f & RMU_FLAG_SW)             rt_kprintf(" SW");
    if (f & RMU_FLAG_MPU_ERR)        rt_kprintf(" MPU");
    if (f & RMU_FLAG_RAM_PARITY_ERR) rt_kprintf(" RAMPAR");
    if (f & RMU_FLAG_RAM_ECC)        rt_kprintf(" RAMECC");
    if (f & RMU_FLAG_CLK_ERR)        rt_kprintf(" CLKERR");
    if (f & RMU_FLAG_XTAL_ERR)       rt_kprintf(" XTALERR");
    if (f & RMU_FLAG_MX)             rt_kprintf(" MULTI");
    rt_kprintf("\n");
}

/* ============================================================
 * 签到客户端表（内部）
 * ============================================================ */

/**
 * @brief 签到客户端描述
 */
typedef struct {
    const char       *name;                 /*!< 名称（静态字符串）        */
    volatile uint32_t last_checkin_tick;    /*!< 最近一次签到的 tick        */
    uint32_t          timeout_ticks;        /*!< 允许的最大签到间隔（tick） */
    uint8_t           used;                 /*!< 1 = 表项已占用             */
    uint8_t           enabled;              /*!< 1 = 参与签到检查           */
} wdt_client_t;

static wdt_client_t      s_clients[WDT_CLIENT_MAX];

/** @brief 1 = 启用签到检查 */
static volatile uint8_t  s_checkin_enable  = 0;

/** @brief 本周期的签到新鲜度缓存（由节流后的全表扫描更新） */
static volatile uint8_t  s_fresh_cache     = 1;

/** @brief 上一次执行全表扫描的 tick */
static volatile uint32_t s_last_check_tick = 0;

/** @brief 最近一次判定为过期的客户端下标，0xFF = 无 */
static volatile uint8_t  s_last_expired    = 0xFF;

/** @brief 因签到过期而停止喂狗的累计次数（诊断用，事后可查） */
static volatile uint32_t s_hold_count      = 0;

/* ============================================================
 * 内部函数：签到新鲜度判定
 * ============================================================ */

/**
 * @brief  判定所有已使能客户端是否均按期签到
 * @param  [in] now  当前 tick（由调用方一次性读取，避免重复调用 rt_tick_get）
 * @retval 1 = 全部新鲜（可喂狗）
 * @retval 0 = 存在过期客户端（不可喂狗）
 * @note   使用 tick 差值比较，天然处理 tick 回绕，
 *         同时避免 (diff * 1000) 在长期未签到时溢出
 */
static uint8_t _wdt_checkins_fresh(uint32_t now)
{
    uint8_t i;

    for (i = 0; i < WDT_CLIENT_MAX; i++) {
        const wdt_client_t *c = &s_clients[i];

        if (!c->used || !c->enabled) {
            continue;
        }

        if ((now - c->last_checkin_tick) > c->timeout_ticks) {
            s_last_expired = i;
            return 0U;                          /* 存在过期客户端 */
        }
    }

    return 1U;
}

/* ============================================================
 * idle 线程钩子
 * ============================================================ */

/**
 * @brief  idle 线程钩子：条件喂狗
 * @note   运行于 idle 线程上下文，执行频率 = idle 循环频率（极高）
 *         只允许「读 tick + 若干比较 + 寄存器写」
 *         绝对禁止 rt_kprintf（会占用大量栈并拖垮系统）
 */
static void _wdt_idle_hook(void)
{
    if (!s_wdt_ready) {
        return;
    }

    /* ----- 签到模式：所有客户端新鲜才喂狗 ----- */
    if (s_checkin_enable) {
        uint32_t now = rt_tick_get();

        /* 节流：每 WDT_CHECK_INTERVAL_MS 重算一次新鲜度，结果缓存复用 */
        if ((now - s_last_check_tick) * 1000U / RT_TICK_PER_SECOND
                >= WDT_CHECK_INTERVAL_MS) {
            s_last_check_tick = now;
            s_fresh_cache     = _wdt_checkins_fresh(now);
        }

        if (!s_fresh_cache) {
            /* 存在过期客户端：停止喂狗，等待 SWDT 溢出复位
             * 此路径下【不可】打印，仅累加计数器供事后诊断 */
            s_hold_count++;
            return;
        }
    }

    /* ----- 基础模式 / 签到均新鲜：喂狗 ----- */
    WDT_FEED();
}

/* ============================================================
 * 生命周期接口
 * ============================================================ */

/**
 * @brief  初始化并启动看门狗
 */
int wdt_monitor_init(void)
{
    /* ------------------------------------------------------------
     * 1. 读取复位原因
     *    必须在首次喂狗（启动计数器）之前完成，否则会被本次运行覆盖
     * ------------------------------------------------------------ */
    if (RMU_GetStatus(RMU_FLAG_SWDT) == SET) {
        s_reset_cause = WDT_RESET_SWDT;
    } else {
        s_reset_cause = WDT_RESET_POR;
    }

    /* ------------------------------------------------------------
     * 2. 注册 idle 钩子
     *    必须先注册再启动计数器，否则存在「狗已计时但无人喂」的空窗
     *    RT-Thread 4.1 使用钩子列表，RT_IDLE_HOOK_LIST_SIZE = 4
     *    注意：SEGGER RTT 适配层 drv_rtt.c 已占用 1 个钩子位
     * ------------------------------------------------------------ */
    if (rt_thread_idle_sethook(_wdt_idle_hook) != RT_EOK) {
        rt_kprintf("[WDT] idle hook register failed (hook list full?)\n");
        return -RT_EFULL;
    }

    /* ------------------------------------------------------------
     * 3. 启动计数器
     *    ICG 配置为「软件启动模式」（SWDTAUTS = STOP），
     *    首次 SWDT_FeedDog() 即启动计数
     * ------------------------------------------------------------ */
    s_wdt_ready = 1;
    WDT_FEED();

    /* ------------------------------------------------------------
     * 4. 清除复位标志，供下一次上电判定
     * ------------------------------------------------------------ */
    RMU_ClearStatus();

    rt_kprintf("[WDT] SWDT started, last_reset=%s, checkin = %s\n\n",
               (s_reset_cause == WDT_RESET_SWDT) ? "SWDT" : "POR",
               s_checkin_enable ? "ON" : "OFF");
    rt_kprintf("[WDT] ");
    wdt_print_rmu_flags((uint32_t)s_rmu_flags);

    return RT_EOK;
}

/**
 * @brief  查询本次上电的复位原因
 */
wdt_reset_cause_t wdt_monitor_get_reset_cause(void)
{
    return (wdt_reset_cause_t)s_reset_cause;
}

/* ============================================================
 * 签到机制接口
 * ============================================================ */

/**
 * @brief  注册一个签到客户端
 */
int8_t wdt_client_register(const char *name, uint32_t timeout_ms)
{
    int8_t i;

    if (name == RT_NULL || timeout_ms == 0U) {
        return WDT_CLIENT_INVALID;
    }

    for (i = 0; i < WDT_CLIENT_MAX; i++) {
        if (s_clients[i].used) {
            continue;
        }

        s_clients[i].name              = name;
        s_clients[i].timeout_ticks     = timeout_ms * RT_TICK_PER_SECOND / 1000U;
        s_clients[i].last_checkin_tick = rt_tick_get();     /* 注册即算一次签到 */
        s_clients[i].used              = 1U;
        s_clients[i].enabled           = 1U;

        /* 保证超时门限至少为 1 个 tick */
        if (s_clients[i].timeout_ticks == 0U) {
            s_clients[i].timeout_ticks = 1U;
        }

        rt_kprintf("[WDT] client[%d] '%s' timeout=%ums\n",
                   (int)i, name, (unsigned)timeout_ms);

        return i;
    }

    rt_kprintf("[WDT] client table full (max=%d)\n", WDT_CLIENT_MAX);
    return WDT_CLIENT_INVALID;
}

/**
 * @brief  签到（各关键线程周期调用）
 */
void wdt_client_checkin(int8_t id)
{
    if (id >= 0 && id < WDT_CLIENT_MAX && s_clients[id].used) {
        s_clients[id].last_checkin_tick = rt_tick_get();
    }
}

/**
 * @brief  注销客户端
 */
void wdt_client_unregister(int8_t id)
{
    if (id >= 0 && id < WDT_CLIENT_MAX) {
        s_clients[id].used    = 0U;
        s_clients[id].enabled = 0U;
        s_clients[id].name    = RT_NULL;

        rt_kprintf("[WDT] client[%d] unregistered\n", (int)id);
    }
}

/**
 * @brief  启用 / 禁用签到检查
 */
void wdt_checkin_enable(uint8_t enable)
{
    s_checkin_enable = enable ? 1U : 0U;

    /* 使能后先假设新鲜，避免因历史 tick 造成立即扣住喂狗 */
    s_fresh_cache = 1U;

    rt_kprintf("[WDT] checkin %s\n", s_checkin_enable ? "ENABLED" : "DISABLED");
}

/**
 * @brief  查询签到检查是否已启用
 */
uint8_t wdt_checkin_is_enabled(void)
{
    return s_checkin_enable;
}

/* ============================================================
 * 诊断接口
 * ============================================================ */

/**
 * @brief  打印看门狗状态
 * @note   仅在 MSH 命令上下文中调用
 */
void wdt_monitor_dump(void)
{
    uint32_t now = rt_tick_get();
    uint8_t  i;
    uint8_t  any = 0;

    rt_kprintf("\n================== WDT MONITOR ==================\n");
    rt_kprintf("ready=%u  checkin=%u  hold_count=%u  last_expired=%s\n",
               (unsigned)s_wdt_ready,
               (unsigned)s_checkin_enable,
               (unsigned)s_hold_count,
               (s_last_expired == 0xFFU) ? "-" : s_clients[s_last_expired].name);
    rt_kprintf("last_reset=%s  swdt_count=%u\n",
               (s_reset_cause == WDT_RESET_SWDT) ? "SWDT" : "POR",
               (unsigned)SWDT_GetCountValue());
    rt_kprintf("[WDT] ");
    wdt_print_rmu_flags((uint32_t)s_rmu_flags);
    rt_kprintf("-------------------------------------------------\n");
    rt_kprintf(" id | name            | timeout | age(ms) | state\n");
    rt_kprintf("-------------------------------------------------\n");

    for (i = 0; i < WDT_CLIENT_MAX; i++) {
        const wdt_client_t *c = &s_clients[i];
        uint32_t age_tick;
        uint32_t age_ms;
        uint8_t  stale;

        if (!c->used) {
            continue;
        }
        any = 1U;

        age_tick = now - c->last_checkin_tick;
        age_ms   = age_tick * 1000U / RT_TICK_PER_SECOND;
        stale    = (age_tick > c->timeout_ticks) ? 1U : 0U;

        rt_kprintf("%3u | %-15s | %5ums | %7u | %s%s\n",
                   (unsigned)i,
                   c->name,
                   (unsigned)(c->timeout_ticks * 1000U / RT_TICK_PER_SECOND),
                   (unsigned)age_ms,
                   stale ? "STALE" : "ok",
                   c->enabled ? "" : " (disabled)");
    }

    if (!any) {
        rt_kprintf("(no client registered)\n");
    }

    rt_kprintf("=================================================\n\n");
}

/* ============================================================
 * MSH 调试命令
 * ============================================================ */

/**
 * @brief  t_wdt —— 打印看门狗状态与各客户端签到情况
 */
static void cmd_wdt_dump(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    wdt_monitor_dump();
}

/**
 * @brief  t_wdt_on —— 启用签到检查
 * @note   建议先试跑验证，确认不会误复位后再在代码中固化启用
 */
static void cmd_wdt_on(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    /* 启用签到会改变整机复位行为，属于诊断写 */
    if (sys_cap_require(SYS_CAP_DIAG_WRITE, "t_wdt_on") == 0U) { return; }

    wdt_checkin_enable(1);
}

/**
 * @brief  t_wdt_off —— 禁用签到检查（退回基础喂狗模式）
 */
static void cmd_wdt_off(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    /* 【最高危】关掉签到检查 = 去掉整机死机保护。工作模式绝对禁止，
     * 只有工厂/维护模式（DIAG_WRITE 置位）才允许。 */
    if (sys_cap_require(SYS_CAP_DIAG_WRITE, "t_wdt_off") == 0U) { return; }

    wdt_checkin_enable(0);
}

/**
 * @brief  t_wdt_stale <id> —— 人为将某客户端判为过期
 * @note   把该客户端的最近签到时间推回 100 秒前，等价于「该线程卡死」
 *         用于验证整条链路：签到 → 判定 → 停止喂狗 → SWDT 复位 → 原因识别
 *         用法：t_wdt_stale 0
 */
static void cmd_wdt_stale(int argc, char **argv)
{
    int8_t id;

    if (argc < 2) {
        rt_kprintf("usage: t_wdt_stale <client_id>\n");
        wdt_monitor_dump();
        return;
    }

    id = (int8_t)atoi(argv[1]);

    if (id < 0 || id >= WDT_CLIENT_MAX || !s_clients[id].used) {
        rt_kprintf("invalid client id %d\n", (int)id);
        return;
    }

    /* 这条命令的目的是「人为制造卡死 → 期待整机复位」，等同主动触发
     * 一次硬件复位。工作模式下必须禁止，否则正常运行中被误敲就掉机。 */
    if (sys_cap_require(SYS_CAP_DIAG_WRITE, "t_wdt_stale") == 0U) { return; }

    s_clients[id].last_checkin_tick =
        rt_tick_get() - 100U * RT_TICK_PER_SECOND;

    rt_kprintf("[WDT] client[%d] '%s' forced stale (expect reset soon)\n",
               (int)id, s_clients[id].name);
}

MSH_CMD_EXPORT_ALIAS(cmd_wdt_dump,  t_wdt,       dump watchdog state);
MSH_CMD_EXPORT_ALIAS(cmd_wdt_on,    t_wdt_on,    enable watchdog checkin);
MSH_CMD_EXPORT_ALIAS(cmd_wdt_off,   t_wdt_off,   disable watchdog checkin);
MSH_CMD_EXPORT_ALIAS(cmd_wdt_stale, t_wdt_stale, force a wdt client stale);
