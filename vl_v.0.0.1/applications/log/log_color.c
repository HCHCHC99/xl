/**
 * @file    log_color.c
 * @brief   彩色日志实现 —— 语义色映射 + 独立缓冲 + 原子输出
 * @date    2026-11-10
 *
 * 设计要点见 log_color.h 头部注释。这里只补充两处实现细节：
 *
 * 【1. 为什么缓冲用 static 而不是栈】
 *   256 B 栈开销对 finsh(3072B) / app_main(2048B) 不算小，
 *   而日志调用可能出现在任意线程。静态缓冲 + 互斥锁更省。
 *   风险是持锁期间被高优先级线程抢占会阻塞对方，
 *   但一次 vsnprintf + 一次 device_write 只有微秒级，
 *   远小于任何线程的调度片，实际无影响。
 *
 * 【2. 为什么 FAULT 用背景色而其它的不用】
 *   背景色视觉权重极高，一旦滥用，"异常"就失去警示意义。
 *   本工程真正需要立即停机的只有：急停、过流、堵转、解算失败。
 *   这些用红底白字；警告用黄字；错误用亮红字。
 *   分级之后，扫屏时"有没有红底块"就是一眼可判的最高优先级信号。
 */

#include "log_color.h"

#include <stdarg.h>
#include <string.h>

/* ============================================================
 * 配置常量
 * ============================================================ */

/* 独立格式化缓冲。
 * 256 = 现有最长展开(~100B) + 颜色码 11B + 结尾 1B + 一倍余量。
 * 为什么不复用 rt_kprintf 的 rt_log_buf：那是 static 且 128 B，
 * 见 log_color.h 的详细说明。 */
#define LOG_BUF_SIZE        (256U)

/* 正文可用长度 = 缓冲 - 颜色码(最多 11) - 结尾 \0 - 截断标记 */
#define LOG_TEXT_MAX        (LOG_BUF_SIZE - 16U)

/* 截断标记。为什么需要：原 rt_kprintf 超长时静默砍尾，
 * 排查时会误以为"输出就到这里"，实际是被截了。显式标记避免误判。 */
#define LOG_CUT_MARK        "<cut>"

/* ============================================================
 * 语义色映射表
 * ------------------------------------------------------------
 * ANSI SGR 码含义（ECMA-48）：
 *   0       重置
 *   1       加粗/高亮
 *   2       暗淡
 *   30~37   前景色
 *   40~47   背景色
 *   24      关闭下划线（用于抵消某些终端的默认样式）
 *
 * 配色原则：
 *   INFO  默认色   —— 大量存在，给颜色会视觉疲劳
 *   OK    亮绿     —— "通过"的正反馈
 *   WARN  亮黄     —— 标准警告色
 *   ERR   亮红     —— 标准错误色
 *   FAULT 红底白字 —— 唯一使用背景色，扫屏第一优先级
 *   STATE 亮青     —— 状态跃迁，与"数据"区分
 *   ADC   亮蓝     —— 测量数据，与"状态"区分
 *   TRACE 暗灰     —— 高频输出，刻意降低视觉权重
 *   CMD   亮品红   —— 命令回显，与设备日志区分
 * ============================================================ */
static const char *const s_palette[LOG_LVL_MAX] = {
    /* LOG_LVL_INFO  */ "",                      /* 默认色，不加码 */
    /* LOG_LVL_OK    */ "\x1B[1;32m",            /* 亮绿 */
    /* LOG_LVL_WARN  */ "\x1B[1;33m",            /* 亮黄 */
    /* LOG_LVL_ERR   */ "\x1B[1;31m",            /* 亮红 */
    /* LOG_LVL_FAULT */ "\x1B[1;37;41m",         /* 亮白 + 红底 */
    /* LOG_LVL_STATE */ "\x1B[1;36m",            /* 亮青 */
    /* LOG_LVL_ADC   */ "\x1B[1;34m",            /* 亮蓝 */
    /* LOG_LVL_TRACE */ "\x1B[2;37m",            /* 暗白/灰 */
    /* LOG_LVL_CMD   */ "\x1B[1;35m",            /* 亮品红 */
};

#define LOG_RESET_CODE      "\x1B[0m"

/* ============================================================
 * 模块状态
 * ============================================================ */
static uint8_t      s_enabled = 1U;         /* 默认开启 */
static char         s_buf[LOG_BUF_SIZE];
static rt_mutex_t   s_lock = RT_NULL;
static uint32_t     s_drop_cnt;             /* 取锁超时次数（诊断用）*/
static uint32_t     s_isr_drop_cnt;         /* ISR 里被闸门丢弃次数 */

/* ============================================================
 * 对外接口
 * ============================================================ */

const char *log_color_code(log_lvl_t lvl)
{
    if (s_enabled == 0U) {
        return "";
    }
    if (((uint32_t)lvl >= (uint32_t)LOG_LVL_MAX) || (lvl < LOG_LVL_INFO)) {
        return "";
    }
    return s_palette[lvl];
}

const char *log_color_reset(void)
{
    if (s_enabled == 0U) {
        return "";
    }
    return LOG_RESET_CODE;
}

void log_color_set(uint8_t on)
{
    s_enabled = (on != 0U) ? 1U : 0U;
}

uint8_t log_color_enabled(void)
{
    return s_enabled;
}

/* 中断内允许打印的最高级别。
 *
 * 【为什么需要这个闸门】
 *   本模块自身的锁逻辑已经处理了中断（跳过取锁），但
 *   "能在中断里跑"不等于"应该在中断里跑"：
 *   一次 vsnprintf + 3 次 device_write 在 20kHz(50μs) 中断里
 *   会直接导致超时。本工程确实存在这样的调用链
 *   （stall_trigger -> hw_brake_emergency_lock），
 *   已在 hw_brake.c 改为"中断置标志 + 5ms 任务补打"。
 *
 *   但代码会演化，将来难免有人在 ISR 里加 LOG_ERR。
 *   所以在这里加一道兜底：中断上下文只放行 FAULT 级
 *   （真正的安全事故，宁可拖长一次中断也必须留痕），
 *   其余级别在中断里直接丢弃并计数。
 *
 *   丢弃而不是延迟：本模块没有队列，做队列要动态内存，
 *   对调试模块不划算。丢掉的量可以 t_log stat 看到。
 */
#define LOG_ISR_MAX_LVL     (LOG_LVL_FAULT)

void log_printf(log_lvl_t lvl, const char *fmt, ...)
{
    va_list     args;
    int         n;
    uint32_t    text_len;
    const char *cc;
    const char *cr;
    rt_device_t console;
    uint8_t     in_isr;

    if (fmt == RT_NULL) {
        return;
    }

    /* ---- 中断上下文闸门 ----
     * 见 LOG_ISR_MAX_LVL 的说明。 */
    in_isr = (rt_interrupt_get_nest() > 0U) ? 1U : 0U;

    if ((in_isr != 0U) && ((uint32_t)lvl > (uint32_t)LOG_ISR_MAX_LVL))
    {
        s_isr_drop_cnt++;
        return;
    }

    /* ---- 取锁 ----
     * 中断里不能取互斥锁（rt_mutex_take 会断言），故跳过。
     * 线程上下文取锁以保证"颜色码+正文+复位码"不可分割。 */
    if ((in_isr == 0U) && (s_lock != RT_NULL))
    {
        if (rt_mutex_take(s_lock, rt_tick_from_millisecond(20)) != RT_EOK)
        {
            /* 取锁超时：仍有输出义务（宁可交错也不丢日志），
             * 但记一笔，t_log stat 可看到是否有并发压力。 */
            s_drop_cnt++;
        }
    }

    /* ---- 格式化到独立缓冲 ----
     * 只给 LOG_TEXT_MAX 空间，预留颜色码与截断标记的位置。 */
    va_start(args, fmt);
    n = rt_vsnprintf(s_buf, LOG_TEXT_MAX, fmt, args);
    va_end(args);

    if (n < 0) {
        n = 0;
        s_buf[0] = '\0';
    }

    /* rt_vsnprintf 的返回值是"本该写入的长度"（不含结尾 0）。
     * 超过 LOG_TEXT_MAX-1 说明被截断，补标记提示，避免误判。 */
    if ((uint32_t)n >= LOG_TEXT_MAX - 1U) {
        text_len = LOG_TEXT_MAX - 1U;

        /* 在可写范围内尽量把截断标记贴在末尾。
         * 步骤：
         *   ① 回退到 LOG_CUT_MARK 长度能放下的位置
         *   ② 覆盖为标记（不再补 '\0'，因为下面按长度写出） */
        {
            uint32_t mark_len = (uint32_t)(sizeof(LOG_CUT_MARK) - 1U);
            uint32_t pos = text_len;

            if (pos > mark_len) {
                pos -= mark_len;
            } else {
                pos = 0U;
            }

            if ((pos + mark_len) < LOG_BUF_SIZE) {
                (void)memcpy(&s_buf[pos], LOG_CUT_MARK, mark_len);
                text_len = pos + mark_len;
            }
        }
    } else {
        text_len = (uint32_t)n;
    }

    /* 长度为 0 直接返回（避免输出空颜色对）。
     * 但若 fmt 本身是空串则仍需保持原语义——rt_kprintf 也不输出。 */
    if (text_len == 0U) {
        if ((in_isr == 0U) && (s_lock != RT_NULL)) {
            (void)rt_mutex_release(s_lock);
        }
        return;
    }

    cc = log_color_code(lvl);
    cr = log_color_reset();

    console = rt_console_get_device();
    if (console == RT_NULL) {
        if ((in_isr == 0U) && (s_lock != RT_NULL)) {
            (void)rt_mutex_release(s_lock);
        }
        return;
    }

    /* ---- 输出 ----
     * 分三次 write：颜色码 / 正文 / 复位码。
     *
     * 为什么不合并成一次 write：
     *   合并需要把 11 B 颜色码 挪到正文之前，意味着 memmove 整段
     *   或者再开一块缓冲。而 rt_device_write 对 RTT 只是一次
     *   SEGGER_RTT_Write，三次调用的开销是三次函数调用（纳秒级），
     *   远低于 memmove 256 B 的代价。
     *
     * 为什么这样仍然安全（不会与别的输出交错）：
     *   三次 write 都在 s_lock 保护内（线程上下文），
     *   其他线程的 log_printf / rt_kprintf 无法插入。
     *
     * ⚠️ 唯一的例外：其他线程直接用 rt_kprintf 时**不取本模块的锁**，
     *    仍可能插入到三次 write 之间。这是"存量代码绕过模块"的
     *    固有限制，方案 B 的接受代价。实际表现是极偶发的染色，
     *    不影响日志内容。彻底解决需要存量全部迁移到 LOG_*，
     *    那是后续渐进工作，不在本次范围。 */
    if (cc[0] != '\0') {
        (void)rt_device_write(console, 0, cc, strlen(cc));
    }
    (void)rt_device_write(console, 0, s_buf, text_len);
    if (cr[0] != '\0') {
        (void)rt_device_write(console, 0, cr, strlen(cr));
    }

    if ((in_isr == 0U) && (s_lock != RT_NULL)) {
        (void)rt_mutex_release(s_lock);
    }
}

/* ============================================================
 * 初始化
 * ============================================================ */

/**
 * @brief  日志模块初始化
 * @note   必须在 rt_console_set_device / 控制台就绪后调用。
 *         放在启动表靠后位置（见 boot_table.c），
 *         因为它只影响输出表现，不阻塞任何功能。
 *
 *         若初始化失败（互斥锁创建失败），s_lock 保持 NULL，
 *         log_printf 退化为无锁输出 —— 功能不丢，只是可能交错。
 *         不返回错误码：日志模块不该阻止系统启动。
 */
int log_color_init(void)
{
    if (s_lock == RT_NULL) {
        s_lock = rt_mutex_create("log_c", RT_IPC_FLAG_PRIO);
        if (s_lock == RT_NULL) {
            rt_kprintf("[LOG] WARN: mutex create failed, run unlocked\n");
        }
    }

    s_enabled = 1U;
    s_drop_cnt = 0U;
    s_isr_drop_cnt = 0U;

    rt_kprintf("[LOG] color enabled (ANSI via RTT)\n");
    rt_kprintf("[LOG] try: t_log test / t_log off / t_log stat\n");
    return 0;
}

/* ============================================================
 * 诊断接口（供 t_log stat 使用）
 * ============================================================ */
uint32_t log_color_get_drop_cnt(void)
{
    return s_drop_cnt;
}

uint32_t log_color_get_isr_drop_cnt(void)
{
    return s_isr_drop_cnt;
}