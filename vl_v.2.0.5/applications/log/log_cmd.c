/**
 * @file    log_cmd.c
 * @brief   彩色日志的 MSH 命令（t_log）
 * @date    2026-11-10
 *
 * 命令设计原则：这个模块是"调试辅助"，命令本身要能自证。
 * 所以 t_log test 会把所有级别各打一行，现场一眼就能判断
 * "RTT Viewer 是否真的支持 ANSI"——如果显示乱码方块，
 * 说明宿主端不支持，此时应 t_log off 退回纯文本。
 */

#include "log_color.h"

#include <stdlib.h>
#include <string.h>

#include "../mode/sys_caps.h"

/* ============================================================
 * 级别名表（与 log_lvl_t 顺序严格对应）
 * ============================================================ */
static const char *const s_lvl_name[LOG_LVL_MAX] = {
    "INFO", "OK", "WARN", "ERR", "FAULT", "STATE", "ADC", "TRACE", "CMD"
};

static const char *s_lvl_desc[LOG_LVL_MAX] = {
    "module init / config load",
    "success / pass",
    "warning, can continue",
    "error, function failed",
    "fault/estop, use BG color",
    "state transition",
    "measurement data",
    "high-freq trace",
    "msh command output",
};

/* ============================================================
 * t_log test —— 色板自检
 * ------------------------------------------------------------
 * 这是最重要的子命令：它既是"效果预览"，也是"兼容性探针"。
 * 若某行显示为 "←[1;31m..." 这样的字面量，说明当前终端
 * 不解析 ANSI，应立即 t_log off。
 * ============================================================ */
static void cmd_test(void)
{
    int i;

    rt_kprintf("\n[LOG] color palette test\n");
    rt_kprintf("  If you see literal \"<-[[1;31m\" text instead of colors,\n");
    rt_kprintf("  your terminal does NOT support ANSI -> run: t_log off\n\n");

    for (i = 0; i < (int)LOG_LVL_MAX; i++)
    {
        /* 每行形如：
         *   [OK   ] success / pass                  <- 这行是绿字
         *
         * 注意这里必须用 LOGF 而不是直接拼字符串，
         * 才能验证"宏 -> log_printf -> ANSI"整条链路。 */
        LOGF((log_lvl_t)i, "[%-5s] %-30s  sample: value=%d\n",
             s_lvl_name[i], s_lvl_desc[i], i * 100);
    }

    rt_kprintf("\n[LOG] end of palette\n");
}

/* ============================================================
 * t_log map —— 打印"级别 -> ANSI 码"映射
 * ------------------------------------------------------------
 * 用途：把本模块的配色抄到 RTT Viewer 或其他工具里做对照，
 * 也便于确认"关闭彩色后码确实为空"。
 * ============================================================ */
static void cmd_map(void)
{
    int i;
    const char *cc;

    rt_kprintf("\n[LOG] level -> ANSI map (enabled=%u)\n",
               (unsigned)log_color_enabled());
    rt_kprintf("  lvl     name    ansi-code        visible\n");
    rt_kprintf("  ------  ------  ---------------  -------\n");

    for (i = 0; i < (int)LOG_LVL_MAX; i++)
    {
        cc = log_color_code((log_lvl_t)i);

        if (cc[0] == '\0') {
            rt_kprintf("  %-6d  %-6s  %-15s  %s\n",
                       i, s_lvl_name[i], "(none)", "(plain)");
        } else {
            /* 码本身含 ESC，直接打印会染色本行。
             * 这里刻意**不**终止颜色，让 visible 列展示真实效果，
             * 行尾再统一 RESET。 */
            rt_kprintf("  %-6d  %-6s  ESC[%-11s  %sVISIBLE%s\n",
                       i, s_lvl_name[i], cc + 2,
                       cc, log_color_reset());
        }
    }
    rt_kprintf("\n");
}

/* ============================================================
 * t_log stat —— 运行统计
 * ============================================================ */
static void cmd_stat(void)
{
    rt_kprintf("\n[LOG] stat\n");
    rt_kprintf("  enabled   = %u\n", (unsigned)log_color_enabled());
    rt_kprintf("  buf_size  = %u B (rt_kprintf uses %u B)\n",
               256U, 128U);
    rt_kprintf("  lock_to   = %u  (mutex take timeout, >0 means contention)\n",
               (unsigned)log_color_get_drop_cnt());
    rt_kprintf("  isr_drop  = %u  (dropped in ISR, only FAULT allowed there)\n",
               (unsigned)log_color_get_isr_drop_cnt());

    if (log_color_get_isr_drop_cnt() > 0U) {
        rt_kprintf("\n  ALARM: 有日志在中断里被丢弃。\n");
        rt_kprintf("         说明某处 ISR 用了 LOG_INFO/WARN/ERR。\n");
        rt_kprintf("         改法：ISR 里只置标志，由 5ms 任务补打\n");
        rt_kprintf("         （参考 hw_brake.c 的 s_log_pending 做法）。\n");
    }

    if (log_color_get_drop_cnt() > 0U) {
        rt_kprintf("  NOTE: timeout>0 说明日志并发较高，锁等待 20ms 仍失败。\n");
        rt_kprintf("        通常是某个线程高频日志 + 低优先级导致。\n");
        rt_kprintf("        可把该类输出改用 LOG_TRACE 并降频。\n");
    }
    rt_kprintf("\n");
}

/* ============================================================
 * 主命令
 * ============================================================ */
static void cmd_log(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("\n[LOG] color log commands\n");
        rt_kprintf("  t_log on              enable color (default)\n");
        rt_kprintf("  t_log off             disable -> plain text (for file export)\n");
        rt_kprintf("  t_log test            print palette (also an ANSI probe)\n");
        rt_kprintf("  t_log map             show level -> ANSI code map\n");
        rt_kprintf("  t_log stat            runtime statistics\n");
        rt_kprintf("\n  macros: LOG_INFO/OK/WARN/ERR/FAULT/STATE/ADC/TRACE/CMD\n");
        rt_kprintf("  note  : rt_kprintf stays uncolored until migrated\n\n");
        return;
    }

    if (strcmp(argv[1], "on") == 0)
    {
        log_color_set(1U);
        rt_kprintf("[LOG] color ON\n");
        return;
    }

    if (strcmp(argv[1], "off") == 0)
    {
        /* 关闭前提示：这是导出日志到文件前该做的动作，
         * 否则 ANSI 码会污染文件内容。 */
        log_color_set(0U);
        rt_kprintf("[LOG] color OFF (plain text, safe for file export)\n");
        return;
    }

    if (strcmp(argv[1], "test") == 0)
    {
        /* 这个子命令属"诊断写"（会输出大量行），用 READ 权限即可，
         * 因为它不改变系统状态。 */
        if (sys_cap_require(SYS_CAP_DIAG_READ, "t_log test") == 0U) { return; }
        cmd_test();
        return;
    }

    if (strcmp(argv[1], "map") == 0)
    {
        cmd_map();
        return;
    }

    if (strcmp(argv[1], "stat") == 0)
    {
        cmd_stat();
        return;
    }

    rt_kprintf("[LOG] unknown subcmd '%s', try: t_log\n", argv[1]);
}

MSH_CMD_EXPORT_ALIAS(cmd_log, t_log, color log: on|off|test|map|stat);