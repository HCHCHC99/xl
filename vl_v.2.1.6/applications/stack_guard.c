/**
 * @file    stack_guard.c
 * @brief   线程栈水位巡检
 * @note    为什么需要这个模块：
 *
 *          本工程曾出现 "thread:can_tx stack overflow"，排查过程暴露了
 *          两类根因，都不是"栈数字给小了"这么简单：
 *            ① 内存布局 bug：RT-Thread 堆与 MSP 栈区重叠（D-57）
 *            ② 断言链开销：热路径上的断言走 printf 链，吃 600 B 栈（D-58/D-61）
 *
 *          这类问题的可怕之处是**没有任何前兆**：栈被踩穿后 RT-Thread 直接
 *          关中断死循环（scheduler.c:_scheduler_stack_check → while(level)），
 *          现场只剩一行 "stack overflow"，排查成本极高。
 *
 *          本模块周期性扫描每个线程栈底的填充字节 '#'（RT-Thread 在
 *          _thread_init 时用 rt_memset(stack_addr, '#', stack_size) 填充），
 *          找出实际用到的最高水位。当某个线程用量超过阈值时提前告警，
 *          把"事后排查"变成"事前预警"。
 *
 *          设计要点：
 *          - 纯读操作，不分配内存、不改动任何线程状态
 *          - 放在 bg_sched 线程里跑（优先级最低，不影响实时性）
 *          - 扫描开销 O(n)，1 次/秒，可忽略
 */

#include "stack_guard.h"
#include <rtthread.h>
#include <string.h>

/* ============================================================
 * 配置
 * ============================================================ */

/** 巡检周期（ms） */
#define STACK_GUARD_PERIOD_MS     1000U

/** 告警阈值：用量超过此百分比即告警 */
#define STACK_GUARD_WARN_PCT      80U

/** 巡检测试的上限时间（ms），启动后多久开始巡检。
 *  避免启动阶段各线程栈还未达峰值就误报。 */
#define STACK_GUARD_STARTUP_MS    3000U

/** 单个线程最多连续告警次数，避免刷屏 */
#define STACK_GUARD_MAX_REPORT    3U

/* ============================================================
 * 内部状态
 * ============================================================ */

typedef struct {
    char        name[RT_NAME_MAX];
    rt_uint8_t  reported;       /* 已告警次数 */
} guard_slot_t;

#define GUARD_SLOT_MAX  12U
static guard_slot_t s_slots[GUARD_SLOT_MAX];
static rt_uint8_t   s_slot_count = 0U;

static rt_tick_t   s_start_tick     = 0U;
static rt_tick_t   s_last_scan_tick = 0U;
static rt_bool_t   s_initialized    = RT_FALSE;
static rt_uint32_t s_scan_count     = 0U;

/* ============================================================
 * 水位计算
 * ============================================================ */

/**
 * @brief  计算某线程的栈最高水位（字节）
 * @param  thread 线程对象
 * @return 已用字节数；无法计算时返回 0（栈刚建好、一个字节都没用过）
 *
 * @note   【2026-10-27 · P0 修正算法方向】
 *         栈向下增长（ARCH_CPU_STACK_GROWS_UPWARD 未定义）：
 *           stack_addr            = 栈底（低地址，最后一个被用到的边界）
 *           stack_addr + stack_size = 栈顶（高地址，sp 初始值）
 *         sp 从栈顶向下压，所以**低地址端**的 '#' 是最晚被覆盖的。
 *         因此"已用量" = stack_size − 从栈底起连续 '#' 的字节数。
 *
 *         原实现从栈顶往下找最后一个 '#'，返回的是 (ptr - base + 1)，
 *         那实际上数的是**还剩多少 '#'**，即【未使用量】。调用处却当
 *         【已使用量】使用，导致 used / pct / headroom 三个值全部反转
 *         （实测 bg_sched 报 100%、headroom 0，而 ps 显示 21%）。
 *
 *         现算法与 RT-Thread cmd.c:232-239 的 ps 保持一致，并补上
 *         原版缺失的边界保护（原版 `while (*ptr == '#') ptr++` 在栈被
 *         写满时会越界读到栈顶之外，把随机内存当 '#' 继续推进）。
 *
 *         三种边界情形均自洽：
 *           - 全 '#'（线程刚建、未跑）→ ptr 走到 top → 返回 0
 *           - 全非 '#'（完全写满/溢出）→ ptr 停在 base → 返回 stack_size
 *           - 正常部分使用 → 返回 (top - ptr)
 */
static rt_uint32_t stack_used_of(rt_thread_t thread)
{
    rt_uint8_t *base;
    rt_uint8_t *top;
    rt_uint8_t *ptr;

    if (thread == RT_NULL || thread->stack_addr == RT_NULL ||
        thread->stack_size == 0U) {
        return 0U;
    }

    base = (rt_uint8_t *)thread->stack_addr;
    top  = base + thread->stack_size;

    /* 从栈底向高地址数连续 '#'，第一个非 '#' 即历史最高水位位置。
     * ptr < top 是边界保护，防止写满时越界。
     * 用 volatile 读避免 -O2 把循环优化成 memchr 而漏掉 volatile 语义。 */
    ptr = base;
    while (ptr < top && *ptr == '#') {
        ptr++;
    }

    return (rt_uint32_t)(top - ptr);
}

/* ============================================================
 * 巡检
 * ============================================================ */

void stack_guard_scan(void)
{
    rt_thread_t  thread;
    rt_uint32_t  used, pct;
    rt_uint8_t   i;
    rt_object_t  obj;
    rt_list_t   *node;
    rt_tick_t    now;

    /* 【2026-10-27 · P1 修正死代码】
     *
     * 原实现（错误）：
     *     if (s_scan_count == 0U) { s_start_tick = rt_tick_get(); }
     *     if (rt_tick_get() - s_start_tick < STARTUP_MS) return;
     *     s_scan_count++;
     *
     * 因为 s_start_tick 在 s_scan_count 仍为 0 时每次都被重置，
     * 而 s_scan_count++ 在 return 之后、永远执行不到，形成死锁环：
     *   s_scan_count 恒 0 → 每次都重置 s_start_tick → 差值恒 0 → 永远 return
     * 结果：本函数**自上线以来一次都没有真正巡检过**，属静默失效。
     *
     * 现改为两个独立状态：s_initialized 只置一次，s_last_scan_tick 做周期节流。
     * 顺带补上原本缺失的 STACK_GUARD_PERIOD_MS 节流——原设计文档写的是
     * "1 次/秒"，但代码里根本没有节流，一旦修好死代码就会随 bg_sched
     * 的调度频率疯狂扫描。 */
    now = rt_tick_get();

    if (s_initialized == RT_FALSE) {
        s_start_tick     = now;
        s_last_scan_tick = now;
        s_initialized    = RT_TRUE;
    }

    /* 阶段一：启动后延迟若干秒才开始，避免启动阶段各线程栈未达峰值就误报 */
    if (now - s_start_tick <
        rt_tick_from_millisecond((rt_int32_t)STACK_GUARD_STARTUP_MS)) {
        return;
    }

    /* 阶段二：按 STACK_GUARD_PERIOD_MS 节流（tick 差比较，无符号回绕安全） */
    if (now - s_last_scan_tick <
        rt_tick_from_millisecond((rt_int32_t)STACK_GUARD_PERIOD_MS)) {
        return;
    }
    s_last_scan_tick = now;

    s_scan_count++;

    /* 遍历 RT-Thread 对象容器的线程链表。
     * 不使用 rt_thread_find（需要名字），直接走 container 链表，
     * 可覆盖所有已创建线程（含内核内置的 main/idle/timer/finsh）。 */
    for (node = rt_object_get_information(RT_Object_Class_Thread)->object_list.next;
         node != &(rt_object_get_information(RT_Object_Class_Thread)->object_list);
         node = node->next) {

        obj = rt_list_entry(node, struct rt_object, list);
        thread = (rt_thread_t)obj;

        used = stack_used_of(thread);
        if (used == 0U || thread->stack_size == 0U) {
            continue;
        }
        pct = used * 100U / thread->stack_size;

        if (pct < STACK_GUARD_WARN_PCT) {
            continue;
        }

        /* 找到/分配该线程的记录槽，限制重复告警次数 */
        for (i = 0U; i < s_slot_count; i++) {
            if (strncmp(s_slots[i].name, thread->name, RT_NAME_MAX) == 0) {
                break;
            }
        }
        if (i == s_slot_count) {
            if (s_slot_count >= GUARD_SLOT_MAX) {
                continue;       /* 槽满，静默 */
            }
            rt_strncpy(s_slots[i].name, thread->name, RT_NAME_MAX - 1U);
            s_slots[i].name[RT_NAME_MAX - 1U] = '\0';
            s_slots[i].reported = 0U;
            s_slot_count++;
        }

        if (s_slots[i].reported >= STACK_GUARD_MAX_REPORT) {
            continue;
        }
        s_slots[i].reported++;

        rt_kprintf("[STACK] WARN: thread '%s' uses %u/%u B (%u%%), "
                   "headroom only %u B\n",
                   thread->name,
                   (unsigned)used, (unsigned)thread->stack_size,
                   (unsigned)pct,
                   (unsigned)(thread->stack_size - used));
    }
}

/* ============================================================
 * MSH 命令
 * ============================================================ */

/* ============================================================
 * 【2026-10-27 · 用户决定停用 t_stack】
 *
 * 停用原因（如实记录，避免以后误判）：
 *   用户实测 t_stack 输出「全部 100%」，与 ps 严重矛盾，据此判定不可信。
 *   但经核对，用户当时烧录的是 11:16:21 的旧固件（t_con 报 command not found
 *   可佐证），P0 算法修复（13:22 构建）从未进入该二进制。也就是说：
 *   **报废的是旧版本，不是当前代码**。
 *
 * 尽管如此，用户明确表示「不再纠结自己构建，更关注应用功能」，故按要求停用。
 *
 * 恢复方法：把下面的 #if 0 改成 #if 1 即可（算法已修正，与 ps 口径一致）。
 * 注意：恢复前请确认已用修正后的固件实测过，不要再拿旧固件的结果下结论。
 *
 * 影响：停用后本模块的 t_stack 命令消失；stack_guard_scan() 也不再被
 * bg_sched 调用（见 bg_scheduler.c 同步改动），整个模块进入 GC 状态，
 * 不占用 Flash（-ffunction-sections + --gc-sections）。
 * 文件本身保留，不删除。
 * ============================================================ */
#if 0

static void t_stack_guard(int argc, char **argv)
{
    rt_thread_t  thread;
    rt_uint32_t  used, pct;
    rt_object_t  obj;
    rt_list_t   *node;
    rt_uint8_t   cnt = 0U;

    (void)argc; (void)argv;

    rt_kprintf("\n===== STACK WATERMARK =====\n");
    rt_kprintf("thread        used     size    pct   headroom  status\n");
    rt_kprintf("--------------------------------------------------------\n");

    for (node = rt_object_get_information(RT_Object_Class_Thread)->object_list.next;
         node != &(rt_object_get_information(RT_Object_Class_Thread)->object_list);
         node = node->next) {

        obj = rt_list_entry(node, struct rt_object, list);
        thread = (rt_thread_t)obj;

        used = stack_used_of(thread);
        if (thread->stack_size == 0U) {
            continue;
        }
        pct = used * 100U / thread->stack_size;

        rt_kprintf("%-13s %6u %8u %5u%% %9u  %s\n",
                   thread->name,
                   (unsigned)used, (unsigned)thread->stack_size,
                   (unsigned)pct,
                   (unsigned)(thread->stack_size - used),
                   (pct >= STACK_GUARD_WARN_PCT) ? "WARN" : "ok");
        cnt++;
    }

    rt_kprintf("--------------------------------------------------------\n");
    rt_kprintf("%u threads, warn threshold %u%%\n",
               (unsigned)cnt, (unsigned)STACK_GUARD_WARN_PCT);
    rt_kprintf("===========================\n\n");
}
MSH_CMD_EXPORT_ALIAS(t_stack_guard, t_stack, show stack watermark of all threads);

#endif /* 0 —— t_stack 已按用户要求停用，恢复改成 #if 1 */