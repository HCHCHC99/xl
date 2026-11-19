/**
 * @file    param_cmd.c
 * @brief   t_param 命令集 —— 参数存储的 MSH 诊断/写入入口
 *
 * ============================================================
 * 【命令一览】
 *   t_param                      等价 dump（存储状态 + 各组当前值）
 *   t_param info                 只看扇区/剩余槽位/擦除次数/seq
 *   t_param values               只看各组当前配置值与 ID
 *   t_param set <id|name> <val> [save]   改 RAM；带 save 立即落盘
 *   t_param save [scope]         立即落盘，scope=all|cfg|am|dev|fast
 *   t_param erase [scope] confirm        擦除后把当前 RAM 值重新写入
 *   t_param write <group> [count]        对某区精确写入 count 次（默认 1）
 *   t_param erasecnt <group> <n>         设置当前扇区擦除次数（触发寿命轮转）
 *   t_param fill <group> [leave] 重复写入，直到当前扇区只剩 leave 个槽位
 *
 * 所有打印均为整型/ASCII，不使用中文与浮点（RTT 控制台限制）。
 * ============================================================
 */

#include "param_store.h"
#include "../config/sys_config.h"
#include "../mode/sys_caps.h"

#include <stdlib.h>
#include <string.h>
#include <rtthread.h>

/** 解析 ID：支持 0x0101 / 257 / 名称 */
static int32_t parse_id(const char *s)
{
    const cfg_def_t *d;

    if (s == RT_NULL || *s == '\0') { return -1; }

    d = sys_config_find_def_by_name(s);
    if (d != RT_NULL) { return (int32_t)d->id; }

    if ((s[0] == '0') && ((s[1] == 'x') || (s[1] == 'X'))) {
        return (int32_t)strtoul(s, RT_NULL, 16);
    }
    return (int32_t)strtoul(s, RT_NULL, 10);
}

/**
 * @brief 解析保存/擦除范围
 * @return 1 成功；0 未知范围
 */
static int parse_scope(const char *s, uint32_t *mask)
{
    if ((s == RT_NULL) || (strcmp(s, "all") == 0)) {
        *mask = PARAM_REQ_CFG | PARAM_REQ_AM | PARAM_REQ_DEV | PARAM_REQ_FAST;
        return 1;
    }
    if ((strcmp(s, "cfg") == 0) ||
        (strcmp(s, "motion") == 0) ||
        (strcmp(s, "system") == 0)) {
        *mask = PARAM_REQ_CFG;
        return 1;
    }
    if ((strcmp(s, "am") == 0) || (strcmp(s, "angle_mech") == 0)) {
        *mask = PARAM_REQ_AM;
        return 1;
    }
    if ((strcmp(s, "dev") == 0) || (strcmp(s, "device") == 0)) {
        *mask = PARAM_REQ_DEV;
        return 1;
    }
    if (strcmp(s, "fast") == 0) {
        *mask = PARAM_REQ_FAST;
        return 1;
    }
    if (strcmp(s, "slow") == 0) {
        *mask = PARAM_REQ_CFG | PARAM_REQ_AM | PARAM_REQ_DEV;
        return 1;
    }
    return 0;
}

static void t_param(int argc, char **argv)
{
    const char *sub = (argc >= 2) ? argv[1] : "dump";

    if (strcmp(sub, "set") == 0) {
        int32_t id;
        int32_t val;
        int     do_save;

        if (argc < 4) {
            rt_kprintf("[PARAM] usage: t_param set <id|name> <value> [save]\n");
            return;
        }
        if (sys_cap_require(SYS_CAP_PARAM_WRITE, "t_param set") == 0U) { return; }

        id = parse_id(argv[2]);
        if (id < 0) {
            rt_kprintf("[PARAM] bad id\n");
            return;
        }

        val     = (int32_t)strtol(argv[3], RT_NULL, 0);
        do_save = ((argc >= 5) && (strcmp(argv[4], "save") == 0)) ? 1 : 0;

        if (sys_config_set_i32((uint16_t)id, val) != 0) { return; }

        if (do_save != 0) {
            (void)param_store_save_cmd(PARAM_REQ_CFG | PARAM_REQ_AM | PARAM_REQ_DEV);
        }
        return;
    }

    if (strcmp(sub, "save") == 0) {
        uint32_t mask;

        if (sys_cap_require(SYS_CAP_PARAM_WRITE, "t_param save") == 0U) { return; }

        if (argc >= 3) {
            /* 不带 scope 时默认 all，但 all 不含 fast（避免无意义擦写） */
            if (strcmp(argv[2], "all") == 0) {
                mask = PARAM_REQ_CFG | PARAM_REQ_AM | PARAM_REQ_DEV;
            } else if (parse_scope(argv[2], &mask) == 0) {
                rt_kprintf("[PARAM] usage: t_param save [all|cfg|am|dev|fast]\n");
                return;
            }
        } else {
            mask = PARAM_REQ_CFG | PARAM_REQ_AM | PARAM_REQ_DEV;
        }

        (void)param_store_save_cmd(mask);
        return;
    }

    if (strcmp(sub, "erase") == 0) {
        uint32_t mask = PARAM_REQ_CFG | PARAM_REQ_AM | PARAM_REQ_DEV | PARAM_REQ_FAST;
        int      confirm = 0;
        int      i;

        if (sys_cap_require(SYS_CAP_PARAM_WRITE, "t_param erase") == 0U) { return; }

        for (i = 2; i < argc; i++) {
            if (strcmp(argv[i], "confirm") == 0) {
                confirm = 1;
            } else if (parse_scope(argv[i], &mask) == 0) {
                rt_kprintf("[PARAM] erase: unknown scope '%s'\n", argv[i]);
                rt_kprintf("  usage: t_param erase [all|slow|fast] confirm\n");
                return;
            }
        }

        if (confirm == 0) {
            rt_kprintf("[PARAM] erase will WIPE records and rewrite current RAM values.\n");
            rt_kprintf("[PARAM]   scope all : sectors 56~61 + 63\n");
            rt_kprintf("[PARAM]   scope slow: sectors 56,57,58,63\n");
            rt_kprintf("[PARAM]   scope fast: sectors 59,60,61\n");
            rt_kprintf("      run 't_param erase confirm' to proceed\n");
            return;
        }

        (void)param_store_erase_cmd(mask);
        return;
    }

    if (strcmp(sub, "fill") == 0) {
        uint16_t leave = 2U;

        if (argc < 3) {
            rt_kprintf("[PARAM] usage: t_param fill <motion|system|angle_mech|device|fast> [leave]\n");
            return;
        }
        if (sys_cap_require(SYS_CAP_PARAM_WRITE, "t_param fill") == 0U) { return; }

        if (argc >= 4) {
            leave = (uint16_t)strtoul(argv[3], RT_NULL, 0);
        }
        (void)param_store_fill_cmd(argv[2], leave);
        return;
    }

    if (strcmp(sub, "write") == 0) {
        uint16_t count = 1U;

        if (argc < 3) {
            rt_kprintf("[PARAM] usage: t_param write <motion|system|angle_mech|device|fast> [count]\n");
            return;
        }
        if (sys_cap_require(SYS_CAP_PARAM_WRITE, "t_param write") == 0U) { return; }

        if (argc >= 4) {
            count = (uint16_t)strtoul(argv[3], RT_NULL, 0);
        }
        (void)param_store_write_cmd(argv[2], count);
        return;
    }

    if (strcmp(sub, "erasecnt") == 0) {
        uint32_t count;

        if (argc < 4) {
            rt_kprintf("[PARAM] usage: t_param erasecnt <motion|system|angle_mech|device|fast> <count>\n");
            return;
        }
        if (sys_cap_require(SYS_CAP_PARAM_WRITE, "t_param erasecnt") == 0U) { return; }

        count = (uint32_t)strtoul(argv[3], RT_NULL, 0);
        (void)param_store_set_erase_cmd(argv[2], count);
        return;
    }

    if (sys_cap_require(SYS_CAP_DIAG_READ, "t_param") == 0U) { return; }

    if ((strcmp(sub, "dump") == 0) || (strcmp(sub, "all") == 0)) {
        param_store_dump(PARAM_DUMP_INFO | PARAM_DUMP_VALUES);
        return;
    }
    if (strcmp(sub, "info") == 0) {
        param_store_dump(PARAM_DUMP_INFO);
        return;
    }
    if (strcmp(sub, "values") == 0) {
        param_store_dump(PARAM_DUMP_VALUES);
        return;
    }

    rt_kprintf("[PARAM] unknown sub-command: %s\n", sub);
    rt_kprintf("  dump | info | values\n");
    rt_kprintf("  set <id|name> <val> [save]\n");
    rt_kprintf("  save [all|cfg|am|dev|fast]\n");
    rt_kprintf("  erase [all|slow|fast] confirm\n");
    rt_kprintf("  write <motion|system|angle_mech|device|fast> [count]\n");
    rt_kprintf("  erasecnt <motion|system|angle_mech|device|fast> <n>\n");
    rt_kprintf("  fill <motion|system|angle_mech|device|fast> [leave]\n");
}

MSH_CMD_EXPORT_ALIAS(t_param, t_param, param storage dump/set/save/erase/write/erasecnt/fill);
