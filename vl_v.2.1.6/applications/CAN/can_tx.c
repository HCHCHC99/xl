/**
 * @file    can_tx.c
 * @brief   CAN 周期发送调度器实现
 * @note    ① timer 回调只释放信号量（优先级 0，禁止阻塞）
 *          ② 独立线程执行 can_send（can_send 为阻塞语义，PTB 忙时自旋重试）
 *          ③ 累计毫秒推进，支持任意周期且抖动不累积
 *          ④ phase_ms 仅决定首帧相位，不参与每周期计算
 */
#include "can_tx.h"
#include "ut_can_send.h"
#include "framework/state_engine/sm_assert.h"
#include <rtthread.h>
#include <string.h>
#include <finsh.h>

/* ================= 配置 ================= */
#define CAN_TX_TICK_MS       10U    /* 时基周期 */
#define CAN_TX_THREAD_PRIO   9      /* 低于 app_main(6)，高于 bg_sched(15) */

/* 【2026-10-27 重新核算 · D-57/D-58/D-59】
 *
 * 演进过程（记录以免重复踩坑）：
 *   1024 → 报 stack overflow
 *   2048 → 仍然报 stack overflow   ← 说明"加大数字"方向本身错了
 *
 * 真正原因有两个，必须同时修：
 *   ① 布局 bug（D-57）：RT-Thread 堆 HEAP_BEGIN=__bss_end 覆盖了
 *      启动文件的 .stack(MSP) 段，线程栈落点压进 MSP 区。扩栈会让
 *      栈顶更高、更深地刺入 MSP 区，所以越扩越糟。
 *   ② 断言开销（D-58）：can_send_core / valve_tx_pack 在热路径上
 *      调用 SM_ASSERT_*，其失败分支要走 rt_kprintf 链 ≈ 600 B，
 *      迫使正常路径为永不发生的分支预留栈。
 *
 * 实测栈需求（展开 ELF 调用图，1,992 个函数）：
 *   修复前：正常 240 B / 含打印 872 B
 *   修复后：正常 240 B / 含打印 ≈ 270 B（断言已降级为 return）
 *   叠加硬件压栈 104 + 调度现场 40 + FPU 64 ≈ 240 B
 *   → 峰值约 510 B
 *
 * 取 1024：留 2.0 倍余量。can_tx 通过 pack 函数指针调用各模块组包
 * （valve_tx_pack 32 B / status_tx_pack 16 B / led_tx_pack 16 B，
 *  静态图看不到但都很小），1 KB 余量足够。
 *
 * ⚠️ 若后续新增的 pack 回调内部使用大局部数组或调用 rt_kprintf，
 *    必须用 MSH `list_thread` 看 can_tx 的 max used 复核。 */
#define CAN_TX_THREAD_STACK  1024U

/* ================= 槽位 ================= */
static const can_tx_entry_t *s_slot[CAN_TX_SLOT_MAX];
static uint8_t  s_count = 0U;
static uint32_t s_acc[CAN_TX_SLOT_MAX];   /* 各帧累计 ms */
static uint32_t s_ok [CAN_TX_SLOT_MAX];
static uint32_t s_err[CAN_TX_SLOT_MAX];

static rt_timer_t s_timer = RT_NULL;
static rt_sem_t   s_wake  = RT_NULL;

/* ================= 时基回调 ================= */
static void _tick_cb(void *param)
{
    (void)param;
    if (s_wake != RT_NULL) {
        rt_sem_release(s_wake);      /* 只做这一件事 */
    }
}

/* ================= 注册 ================= */
int can_tx_register(const can_tx_entry_t *entry)
{
    SM_ASSERT_PTR(entry);

    if (entry->pack == RT_NULL) {
        rt_kprintf("[CAN-TX] register '%s' failed: pack is NULL\n", entry->name);
        return -1;
    }
    if (s_count >= CAN_TX_SLOT_MAX) {
        rt_kprintf("[CAN-TX] register '%s' failed: slot full\n", entry->name);
        return -1;
    }
    /* 同 ID 去重 */
    for (uint8_t i = 0U; i < s_count; i++) {
        if (s_slot[i]->id == entry->id) {
            return 0;
        }
    }

    s_slot[s_count]  = entry;
    s_ok[s_count]    = 0U;
    s_err[s_count]   = 0U;

    /* 首次发送相位偏移：
     *   phase_ms 表示「首帧在启动后第 phase_ms 毫秒发送」，
     *   之后按 period_ms 稳定周期发送。
     *   预置 acc = period - phase，使首拍累加到达时间恰为 phase_ms。
     *   注意：phase 只影响首帧，不得叠加到每周期上，
     *        否则实际周期会变成 period + phase（曾导致 A4F0 实测 150ms）。 */
    if (entry->period_ms == 0U) {
        s_acc[s_count] = 0U;
    } else {
        uint16_t ph = (uint16_t)(entry->phase_ms % entry->period_ms);
        s_acc[s_count] = (uint32_t)entry->period_ms - (uint32_t)ph;
    }
    s_count++;

    rt_kprintf("[CAN-TX] registered: %-12s id=0x%08X period=%ums phase=%ums\n",
               entry->name, entry->id, entry->period_ms, entry->phase_ms);
    return 0;
}

/* ================= 变更即发 ================= */
void can_tx_force(uint32_t id)
{
    for (uint8_t i = 0U; i < s_count; i++) {
        if (s_slot[i]->id == id) {
            /* 直接置为到期，下一拍（≤10ms）立即发出 */
            s_acc[i] = (uint32_t)s_slot[i]->period_ms;
            return;
        }
    }
}

/* ================= 调度核心 ================= */
void can_tx_poll(uint32_t elapsed_ms)
{
    for (uint8_t i = 0U; i < s_count; i++) {
        const can_tx_entry_t *e = s_slot[i];
        can_tx_frame_t f;

        if (e->period_ms == 0U) continue;

        s_acc[i] += elapsed_ms;
        if (s_acc[i] < (uint32_t)e->period_ms) continue;

        /* 减去周期而非清零：保留 tick 余数，避免周期随抖动累积变大 */
        s_acc[i] -= (uint32_t)e->period_ms;

        f.id  = e->id;
        f.dlc = e->dlc;
        memset(f.data, 0, CAN_TX_DLC_MAX);

        e->pack(&f);                                  /* ★ 组包 */

        if (can_send(f.id, f.data, f.dlc) == 0) {
            s_ok[i]++;
        } else {
            s_err[i]++;                               /* 仅计数，不重试 */
        }
    }
}

/* ================= 发送线程 ================= */
static void _tx_thread_entry(void *param)
{
    (void)param;
    while (1) {
        if (rt_sem_take(s_wake, RT_WAITING_FOREVER) == RT_EOK) {
            can_tx_poll(CAN_TX_TICK_MS);
        }
    }
}

/* ================= 初始化 ================= */
int can_tx_init(void)
{
    rt_thread_t tid;

    if (s_timer != RT_NULL) {
        rt_kprintf("[CAN-TX] already initialized\n");
        return 0;
    }

    s_wake = rt_sem_create("can_tx", 0, RT_IPC_FLAG_PRIO);
    if (s_wake == RT_NULL) return -1;

    s_timer = rt_timer_create("can_tx", _tick_cb, RT_NULL,
                              rt_tick_from_millisecond(CAN_TX_TICK_MS),
                              RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    if (s_timer == RT_NULL) return -1;

    tid = rt_thread_create("can_tx", _tx_thread_entry, RT_NULL,
                           CAN_TX_THREAD_STACK, CAN_TX_THREAD_PRIO, 10);
    if (tid == RT_NULL) return -1;
    rt_thread_startup(tid);

    if (rt_timer_start(s_timer) != RT_EOK) return -1;

    rt_kprintf("[CAN-TX] started: %u slots, tick=%ums\n",
               (unsigned)s_count, CAN_TX_TICK_MS);
    return 0;
}

/* ================= 调试 ================= */
void can_tx_dump(void)
{
    rt_kprintf("\n========== CAN TX PERIODIC ==========\n");
    rt_kprintf("tick=%ums  slots=%u\n", CAN_TX_TICK_MS, (unsigned)s_count);
    rt_kprintf(" idx | name         | id         | period | trig | ok    | err\n");
    rt_kprintf("-----------------------------------------------------------------\n");
    for (uint8_t i = 0U; i < s_count; i++) {
        const can_tx_entry_t *e = s_slot[i];
        rt_kprintf("  %u  | %-12s | 0x%08X | %4ums | %s | %5u | %u\n",
                   (unsigned)i, e->name, e->id, e->period_ms,
                   (e->trig == CAN_TX_TRIG_ONCHANGE) ? "CHG " : "PER ",
                   (unsigned)s_ok[i], (unsigned)s_err[i]);
    }
    rt_kprintf("=====================================\n\n");
}

static void t_can_tx(int argc, char **argv) { (void)argc; (void)argv; can_tx_dump(); }
MSH_CMD_EXPORT_ALIAS(t_can_tx, t_cantx, dump can tx periodic table);
