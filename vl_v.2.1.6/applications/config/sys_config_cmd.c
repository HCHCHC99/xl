/**
 * @file    sys_config_cmd.c
 * @brief   t_cfg 命令集 —— 配置表的唯一 MSH 入口
 * @note    设计文档：docs/02-设计/方案-系统配置表sys_config-V1.0.md §6
 *
 * ============================================================
 * 【命令一览】
 *   t_cfg                        等价 list
 *   t_cfg list [all]             列参数（默认隐藏 ADVANCED）
 *   t_cfg get <id|name>          查单项详情
 *   t_cfg set <id|name> <val> [save]   改 RAM；带 save 则立即落盘
 *   t_cfg save                   把全部 dirty 项落盘
 *   t_cfg load                   从 Flash 重载（丢弃未存修改）
 *   t_cfg diff                   列出未保存的修改
 *   t_cfg default [id|all] [save]      恢复默认
 *   t_cfg erase confirm          擦除整块
 *   t_cfg info                   版本/项数/写入次数
 *   t_cfg export                 打印可回填的 C 初始化片段
 * ============================================================
 */

#include "sys_config.h"
#include "../mode/sys_caps.h"

#include <stdlib.h>
#include <string.h>
#include <rtthread.h>

/* ============================================================
 * 工具
 * ============================================================ */

/** 解析 ID：支持 0x0101 / 257 / 名称 */
static int32_t parse_id(const char *s)
{
    const cfg_def_t *d;

    if (s == RT_NULL || *s == '\0') { return -1; }

    /* 先按名字查 */
    d = sys_config_find_def_by_name(s);
    if (d != RT_NULL) { return (int32_t)d->id; }

    /* 再按数字：0x 前缀自动识别 */
    if ((s[0] == '0') && ((s[1] == 'x') || (s[1] == 'X'))) {
        return (int32_t)strtoul(s, RT_NULL, 16);
    }
    return (int32_t)strtoul(s, RT_NULL, 10);
}

/** 取当前值的 int32 表示（U32 超 2^31 时仅用于显示，精度不重要） */
static int32_t cur_i32(uint16_t idx)
{
    const cfg_item_t *it = sys_config_item_at(idx);

    if (it == RT_NULL) { return 0; }

    switch (it->type) {
        case CFG_T_U8:  return (int32_t)it->v.u8;
        case CFG_T_U16: return (int32_t)it->v.u16;
        case CFG_T_I16: return it->v.i16;
        case CFG_T_I32: return it->v.i32;
        case CFG_T_U32: return (int32_t)it->v.u32;
        default:        return 0;
    }
}

/** 从 item 指针反查下标 */
static uint16_t i_index_of(const cfg_item_t *it)
{
    uint16_t i, n = sys_config_item_count();

    for (i = 0U; i < n; i++) {
        if (sys_config_item_at(i) == it) { return i; }
    }
    return 0U;
}

/* ============================================================
 * list
 * ============================================================ */
static void cmd_list(int show_all)
{
    uint16_t i;
    uint16_t n = sys_config_item_count();

    rt_kprintf("  ID     name                 type  value        def     src    flags\n");
    rt_kprintf("  -----  -------------------  ----  -----------  ------  -----  -----\n");

    for (i = 0U; i < n; i++) {
        const cfg_item_t *it = sys_config_item_at(i);
        const cfg_def_t  *d;
        char valbuf[12];
        char flagbuf[6];

        if (it == RT_NULL) { continue; }

        d = sys_config_find_def(it->id);
        if (d == RT_NULL) { continue; }

        if ((show_all == 0) && ((d->flags & CFG_F_ADVANCED) != 0U)) { continue; }

        if (it->type == CFG_T_U32) {
            rt_snprintf(valbuf, sizeof(valbuf), "%lu", (unsigned long)it->v.u32);
        } else {
            rt_snprintf(valbuf, sizeof(valbuf), "%d", (int)cur_i32(i));
        }

        flagbuf[0] = ((d->flags & CFG_F_READONLY)    ? 'R' : '.');
        flagbuf[1] = ((d->flags & CFG_F_REQ_RESTART) ? 'T' : '.');
        flagbuf[2] = ((d->flags & CFG_F_REQ_CALIB)   ? 'C' : '.');
        flagbuf[3] = ((d->flags & CFG_F_REQ_SAFE)    ? 'S' : '.');
        flagbuf[4] = ((d->flags & CFG_F_ADVANCED)    ? 'A' : '.');
        flagbuf[5] = '\0';

        rt_kprintf("  0x%04X %-21s %-4s  %-11s  %-6d  %-5s  %s%s\n",
                   (unsigned)it->id, d->name,
                   sys_config_type_name(it->type),
                   valbuf, (int)d->def_val,
                   sys_config_src_name(sys_config_get_src(it->id)),
                   flagbuf,
                   (sys_config_item_is_dirty(i) != 0U) ? " *" : "");
    }

    rt_kprintf("\n  flags: R=readonly T=req-restart C=req-calib S=req-safe A=advanced\n");
    rt_kprintf("  '*' = not saved yet, run 't_cfg save'\n");
    rt_kprintf("  dirty=%u  (use 't_cfg list all' to include advanced)\n",
               (unsigned)sys_config_dirty_count());
}

/* ============================================================
 * get
 * ============================================================ */

/**
 * @brief 判断字符串是否"看起来像数字"（可被 strtoul 解析）
 * @note  【2026-11-04 新增】用于区分"名称找不到"与"ID 找不到"。
 *
 *        背景：parse_id() 先按名字查，查不到就回退 strtoul()。
 *        而 strtoul("stall_oc3_a10", 10) 会返回 0（因为首字符非数字），
 *        于是 cmd_get 报 "id=0x0000 not found" —— 这个提示极具误导性，
 *        让人以为参数 ID 写错了，实际是**名字不存在**（多为固件版本旧）。
 *
 *        典型踩坑：新老固件交叉验证时看到 0x0000，误判成代码 bug。
 */
static uint8_t looks_like_number(const char *s)
{
    if (s == RT_NULL || *s == '\0') { return 0U; }

    if ((s[0] == '0') && ((s[1] == 'x') || (s[1] == 'X')))
    {
        /* 0x 前缀：后面必须至少有一位十六进制数字 */
        return (s[2] != '\0') ? 1U : 0U;
    }

    /* 十进制：首位必须 0~9 */
    if ((s[0] >= '0') && (s[0] <= '9')) { return 1U; }

    return 0U;
}

static void cmd_get(const char *s)
{
    int32_t id;
    const cfg_def_t *d;
    uint16_t i, n, idx = 0U;
    uint8_t found = 0U;

    if (s == RT_NULL || *s == '\0') {
        rt_kprintf("[CFG] usage: t_cfg get <id|name>\n");
        return;
    }

    id = parse_id(s);

    if (id < 0) {
        rt_kprintf("[CFG] usage: t_cfg get <id|name>\n");
        return;
    }

    d = sys_config_find_def((uint16_t)id);
    if (d == RT_NULL) {
        /* 【2026-11-04 修正】区分两种失败原因，避免误导 */
        if (looks_like_number(s) == 0U) {
            rt_kprintf("[CFG] name '%s' NOT FOUND in this firmware\n", s);
            rt_kprintf("      (if it is a new parameter, firmware may be old)\n");
            rt_kprintf("      try 't_cfg list all' to see what exists\n");
        } else {
            rt_kprintf("[CFG] id=0x%04X NOT FOUND\n", (unsigned)id);
        }
        return;
    }

    sys_config_refresh_mirrors();

    n = sys_config_item_count();
    for (i = 0U; i < n; i++) {
        const cfg_item_t *p = sys_config_item_at(i);
        if ((p != RT_NULL) && (p->id == (uint16_t)id)) { idx = i; found = 1U; break; }
    }

    rt_kprintf("  id    = 0x%04X\n", (unsigned)d->id);
    rt_kprintf("  name  = %s\n", d->name);
    rt_kprintf("  type  = %s\n", sys_config_type_name(d->type));

    if (found != 0U) {
        const cfg_item_t *it = sys_config_item_at(idx);

        if ((it != RT_NULL) && (it->type == CFG_T_U32)) {
            rt_kprintf("  value = %lu\n", (unsigned long)it->v.u32);
        } else {
            rt_kprintf("  value = %d\n", (int)cur_i32(idx));
        }
    } else {
        rt_kprintf("  value = (not in RAM table)\n");
    }

    rt_kprintf("  def   = %d\n", (int)d->def_val);
    rt_kprintf("  range = [%d, %d] %s\n",
               (int)d->min_val, (int)d->max_val, d->unit);
    rt_kprintf("  src   = %s\n",
               sys_config_src_name(sys_config_get_src((uint16_t)id)));

    rt_kprintf("  flags =");
    if ((d->flags & CFG_F_READONLY)    != 0U) { rt_kprintf(" READONLY"); }
    if ((d->flags & CFG_F_REQ_RESTART) != 0U) { rt_kprintf(" REQ_RESTART"); }
    if ((d->flags & CFG_F_REQ_CALIB)   != 0U) { rt_kprintf(" REQ_CALIB"); }
    if ((d->flags & CFG_F_REQ_SAFE)    != 0U) { rt_kprintf(" REQ_SAFE"); }
    if ((d->flags & CFG_F_ADVANCED)    != 0U) { rt_kprintf(" ADVANCED"); }
    if (d->flags == 0U)                        { rt_kprintf(" (none)"); }
    rt_kprintf("\n");
}

/* ============================================================
 * diff
 * ============================================================ */
static void cmd_diff(void)
{
    uint16_t i;
    uint16_t n = sys_config_item_count();
    uint32_t cnt = 0U;

    rt_kprintf("[CFG] unsaved changes:\n");

    for (i = 0U; i < n; i++) {
        const cfg_item_t *it;
        const cfg_def_t  *d;

        if (sys_config_item_is_dirty(i) == 0U) { continue; }

        it = sys_config_item_at(i);
        if (it == RT_NULL) { continue; }
        d = sys_config_find_def(it->id);
        if (d == RT_NULL) { continue; }

        rt_kprintf("  0x%04X %-21s def=%-6d ram=",
                   (unsigned)it->id, d->name, (int)d->def_val);
        if (it->type == CFG_T_U32) {
            rt_kprintf("%lu\n", (unsigned long)it->v.u32);
        } else {
            rt_kprintf("%d\n", (int)cur_i32(i));
        }
        cnt++;
    }

    if (cnt == 0U) {
        rt_kprintf("  (none, all values persisted)\n");
    } else {
        rt_kprintf("  run 't_cfg save' to persist %u item(s)\n", (unsigned)cnt);
    }
}

/* ============================================================
 * export
 * ============================================================ */
static void cmd_export(void)
{
    uint16_t i;
    uint16_t n = sys_config_item_count();

    rt_kprintf("/* current values, paste into s_cfg_def[] in sys_config.c */\n");

    for (i = 0U; i < n; i++) {
        const cfg_item_t *it = sys_config_item_at(i);
        const cfg_def_t  *d;

        if (it == RT_NULL) { continue; }
        d = sys_config_find_def(it->id);
        if (d == RT_NULL) { continue; }

        if (it->type == CFG_T_U32) {
            rt_kprintf("    { 0x%04X, \"%s\", %s, 0x%02X, %lu, %d, %d, \"%s\" },\n",
                       (unsigned)d->id, d->name, sys_config_type_name(d->type),
                       (unsigned)d->flags, (unsigned long)it->v.u32,
                       (int)d->min_val, (int)d->max_val, d->unit);
        } else {
            rt_kprintf("    { 0x%04X, \"%s\", %s, 0x%02X, %d, %d, %d, \"%s\" },\n",
                       (unsigned)d->id, d->name, sys_config_type_name(d->type),
                       (unsigned)d->flags, (int)cur_i32(i),
                       (int)d->min_val, (int)d->max_val, d->unit);
        }
    }
}

/* ============================================================
 * 命令入口
 * ============================================================ */
static void t_cfg(int argc, char **argv)
{
    const char *sub = (argc >= 2) ? argv[1] : "list";

    if (sys_cap_require(SYS_CAP_DIAG_READ, "t_cfg") == 0U) { return; }

    if (strcmp(sub, "list") == 0) {
        int all = ((argc >= 3) && (strcmp(argv[2], "all") == 0)) ? 1 : 0;
        sys_config_refresh_mirrors();
        cmd_list(all);
        return;
    }

    if (strcmp(sub, "get") == 0) {
        if (argc < 3) { rt_kprintf("[CFG] usage: t_cfg get <id|name>\n"); return; }
        cmd_get(argv[2]);
        return;
    }

    if (strcmp(sub, "set") == 0) {
        int32_t id;
        int32_t val;
        int do_save;

        if (argc < 4) {
            rt_kprintf("[CFG] usage: t_cfg set <id|name> <value> [save]\n");
            return;
        }
        if (sys_cap_require(SYS_CAP_PARAM_WRITE, "t_cfg set") == 0U) { return; }

        id  = parse_id(argv[2]);
        if (id < 0) { rt_kprintf("[CFG] bad id\n"); return; }

        val = (int32_t)strtol(argv[3], RT_NULL, 0);
        do_save = ((argc >= 5) && (strcmp(argv[4], "save") == 0)) ? 1 : 0;

        if (sys_config_set_i32((uint16_t)id, val) != 0) { return; }

        if (do_save != 0) { (void)sys_config_save(); }
        return;
    }

    if (strcmp(sub, "save") == 0) {
        if (sys_cap_require(SYS_CAP_PARAM_WRITE, "t_cfg save") == 0U) { return; }
        (void)sys_config_save();
        return;
    }

    if (strcmp(sub, "load") == 0) {
        rt_kprintf("[CFG] reloading from flash (unsaved RAM changes lost)\n");
        (void)sys_config_load();
        return;
    }

    if (strcmp(sub, "diff") == 0) {
        cmd_diff();
        return;
    }

    if (strcmp(sub, "default") == 0) {
        uint16_t id = 0U;
        int do_save = 0;
        int argi = 2;

        if (sys_cap_require(SYS_CAP_PARAM_WRITE, "t_cfg default") == 0U) { return; }

        /* 末尾的 "save" 是修饰词，不是 ID */
        if ((argc >= 3) && (strcmp(argv[argc - 1], "save") == 0)) {
            do_save = 1;
        }
        if (argi < argc && strcmp(argv[argi], "save") != 0) {
            if (strcmp(argv[argi], "all") != 0) {
                int32_t v = parse_id(argv[argi]);
                if (v < 0) { rt_kprintf("[CFG] bad id\n"); return; }
                id = (uint16_t)v;
            }
        }

        if (sys_config_restore_default(id) != 0) { return; }
        if (do_save != 0) { (void)sys_config_save(); }
        return;
    }

    if (strcmp(sub, "erase") == 0) {
        if ((argc < 3) || (strcmp(argv[2], "confirm") != 0)) {
            rt_kprintf("[CFG] erase will reset ALL params to default.\n");
            rt_kprintf("      run 't_cfg erase confirm' to proceed\n");
            return;
        }
        if (sys_cap_require(SYS_CAP_PARAM_WRITE, "t_cfg erase") == 0U) { return; }
        (void)sys_config_erase();
        return;
    }

    if (strcmp(sub, "info") == 0) {
        sys_config_dump_stats();
        return;
    }

    if (strcmp(sub, "export") == 0) {
        sys_config_refresh_mirrors();
        cmd_export();
        return;
    }

    rt_kprintf("[CFG] unknown sub-command: %s\n", sub);
    rt_kprintf("  list [all] | get <id> | set <id> <val> [save] | save | load\n");
    rt_kprintf("  diff | default [id|all] [save] | erase confirm | info | export\n");
}

MSH_CMD_EXPORT_ALIAS(t_cfg, t_cfg, system config table read/write);