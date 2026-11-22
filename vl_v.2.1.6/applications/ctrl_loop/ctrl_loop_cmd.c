/**
 * @file    ctrl_loop_cmd.c
 * @brief   三环模块 MSH 命令
 *
 * @note    命令一览：
 *            cl list                      列出所有已注册实例
 *            cl show <inst>               打印三环参数与统计
 *            cl set <inst> <ring> <param> <val>
 *                                         改单个参数（RAM，不落盘）
 *            cl period <inst> <ring> <ms> 改周期
 *            cl ring <inst> <ring> <0|1>  单环开关
 *            cl enable <inst> <0|1>       实例总开关
 *            cl target <inst> <ring> <v>  手工设目标（调试用）
 *            cl reset <inst> [ring]       清积分与统计
 *            cl save                     所有实例写入 flash
 *            cl load <inst>               从 flash 重载
 *            cl trace <inst> <n> <ms>     连续采样打印三环误差/输出
 */

#include "ctrl_loop.h"

/* 【2026-11-10 彩标 P4】命令结论行分级。
 * 原 save/load 用三元表达式 (ret==0)?"OK":"FAIL"，
 * 同一行既有成功又有失败，无法整体染色。
 * 改为 if/else 分支后可分别着 OK（绿）/ FAIL（红）。*/
#include "../log/log_color.h"
#include "ctrl_loop_cfg.h"
#include "../mode/sys_caps.h"

#include <rtthread.h>
#include <finsh.h>
#include <stdlib.h>
#include <string.h>

/* ============================================================
 * 内部：字符串 → 环类型
 * ============================================================ */
static int8_t _ring_from_str(const char *s)
{
    if (s == RT_NULL) { return -1; }
    if (strcmp(s, "pos") == 0) { return (int8_t)CTRL_LOOP_POSITION; }
    if (strcmp(s, "spd") == 0) { return (int8_t)CTRL_LOOP_SPEED; }
    if (strcmp(s, "cur") == 0) { return (int8_t)CTRL_LOOP_CURRENT; }
    return -1;
}

/* ============================================================
 * 内部：参数名 → PID 字段
 * ============================================================ */
static int8_t _param_from_str(const char *s)
{
    if (s == RT_NULL) { return -1; }
    if (strcmp(s, "kp") == 0) { return 0; }
    if (strcmp(s, "ki") == 0) { return 1; }
    if (strcmp(s, "kd") == 0) { return 2; }
    return -1;
}

static void _set_pid_field(ctrl_pid_cfg_t *c, int8_t f, int32_t v)
{
    switch (f) {
        case 0: c->kp_x100 = v; break;
        case 1: c->ki_x100 = v; break;
        case 2: c->kd_x100 = v; break;
        default: break;
    }
}

static int32_t _get_pid_field(const ctrl_pid_cfg_t *c, int8_t f)
{
    switch (f) {
        case 0: return c->kp_x100;
        case 1: return c->ki_x100;
        case 2: return c->kd_x100;
        default: return 0;
    }
}

/* ============================================================
 * 内部：打印定点值（x100 → 可读）
 * ------------------------------------------------------------
 * 用整数打印，避免浮点（Q-63 未豁免本模块）与 %f 的库开销。
 * 负数处理：先取绝对值再补负号，
 * 否则 -1205/100 = -12、-1205%100 = -5，会拼成 "-12.-5"。
 * ============================================================ */
static void _print_x100(const char *tag, int32_t v)
{
    int32_t a = (v < 0) ? -v : v;

    rt_kprintf("%s=%s%d.%02d", tag, (v < 0) ? "-" : "",
               (int)(a / 100), (int)(a % 100));
}

/* ============================================================
 * cl list
 * ============================================================ */
static void _cmd_list(void)
{
    uint8_t i;
    uint8_t n = ctrl_loop_inst_count();

    rt_kprintf("[ctrl_loop] %u instance(s), max %u\n",
               (unsigned)n, (unsigned)CTRL_LOOP_MAX_INST);

    if (n == 0U) {
        rt_kprintf("  (none registered)\n");
        return;
    }

    rt_kprintf("  idx  name              en   pos  spd  cur\n");
    for (i = 0U; i < n; i++) {
        ctrl_loop_t *l = ctrl_loop_get(i);

        if (l == RT_NULL) { continue; }

        rt_kprintf("  %-4u %-16s %-4u %-4u %-4u %-4u\n",
                   (unsigned)i, l->name,
                   (unsigned)l->enabled,
                   (unsigned)l->ring[CTRL_LOOP_POSITION].cfg.enable,
                   (unsigned)l->ring[CTRL_LOOP_SPEED].cfg.enable,
                   (unsigned)l->ring[CTRL_LOOP_CURRENT].cfg.enable);
    }
}

/* ============================================================
 * cl show <inst>
 * ============================================================ */
static void _cmd_show(uint8_t inst)
{
    ctrl_loop_t *l = ctrl_loop_get(inst);
    uint8_t k;

    if (l == RT_NULL) {
        rt_kprintf("  inst %u not registered\n", (unsigned)inst);
        return;
    }

    rt_kprintf("[inst %u] %s  enabled=%u  inited=%u\n",
               (unsigned)inst, l->name,
               (unsigned)l->enabled, (unsigned)l->inited);

    rt_kprintf("  ring      en  period      kp      ki      kd"
               "   target     err     out   sat  runs\n");

    for (k = 0U; k < (uint8_t)CTRL_LOOP_RING_MAX; k++) {
        ctrl_loop_ring_t *r = &l->ring[k];

        rt_kprintf("  %-8s  %-2u", ctrl_loop_kind_name((ctrl_loop_kind_t)k),
                   (unsigned)r->cfg.enable);

        if (r->period_ms > 0U) {
            rt_kprintf("  %5ums", (unsigned)r->period_ms);
        } else {
            rt_kprintf("   isr  ");
        }

        _print_x100("", r->cfg.kp_x100); rt_kprintf(" ");
        _print_x100("", r->cfg.ki_x100); rt_kprintf(" ");
        _print_x100("", r->cfg.kd_x100);

        rt_kprintf("  %7d %7d %7d %5u %6u\n",
                   (int)l->target[k], (int)r->err, (int)r->last_out,
                   (unsigned)r->sat_cnt, (unsigned)r->run_cnt);
    }

    rt_kprintf("  out range: [%d, %d]  integ_max=%d\n",
               (int)l->ring[CTRL_LOOP_POSITION].cfg.out_min,
               (int)l->ring[CTRL_LOOP_POSITION].cfg.out_max,
               (int)l->ring[CTRL_LOOP_POSITION].cfg.integ_max);
}

/* ============================================================
 * cl set <inst> <ring> <kp|ki|kd> <val>
 * ============================================================ */
static void _cmd_set(uint8_t inst, int8_t ring, int8_t param, int32_t val)
{
    ctrl_loop_t *l = ctrl_loop_get(inst);
    ctrl_pid_cfg_t c;

    if (l == RT_NULL) {
        rt_kprintf("  inst %u not registered\n", (unsigned)inst);
        return;
    }
    if (ring < 0) {
        rt_kprintf("  ring must be pos|spd|cur\n");
        return;
    }
    if (param < 0) {
        rt_kprintf("  param must be kp|ki|kd\n");
        return;
    }

    ctrl_loop_get_pid(l, (ctrl_loop_kind_t)ring, &c);
    _set_pid_field(&c, param, val);
    ctrl_loop_set_pid(l, (ctrl_loop_kind_t)ring, &c);

    rt_kprintf("  inst%u %s kp/ki/kd = %d/%d/%d (RAM only, use 'cl save')\n",
               (unsigned)inst, ctrl_loop_kind_name((ctrl_loop_kind_t)ring),
               (int)c.kp_x100, (int)c.ki_x100, (int)c.kd_x100);
}

/* ============================================================
 * cl trace <inst> <n> <ms>
 * ------------------------------------------------------------
 * 连续采样，用于整定时观察响应曲线（数值形式）。
 * 为什么用数值而不是图形：串口日志最实用，可复制出来做表。
 * ============================================================ */
static void _cmd_trace(uint8_t inst, int n, int ms)
{
    ctrl_loop_t *l = ctrl_loop_get(inst);
    int i;

    if (l == RT_NULL) {
        rt_kprintf("  inst %u not registered\n", (unsigned)inst);
        return;
    }
    if (n <= 0) { n = 20; }
    if (n > 500) { n = 500; }
    if (ms <= 0) { ms = 100; }
    if (ms > 2000) { ms = 2000; }

    rt_kprintf("  #    pos_t  pos_e  pos_o   spd_t  spd_e  spd_o\n");

    for (i = 0; i < n; i++) {
        rt_kprintf("  %-4d %6d %6d %6d   %6d %6d %6d\n",
                   i,
                   (int)l->target[CTRL_LOOP_POSITION],
                   (int)l->ring[CTRL_LOOP_POSITION].err,
                   (int)l->ring[CTRL_LOOP_POSITION].last_out,
                   (int)l->target[CTRL_LOOP_SPEED],
                   (int)l->ring[CTRL_LOOP_SPEED].err,
                   (int)l->ring[CTRL_LOOP_SPEED].last_out);

        if (i < n - 1) { rt_thread_mdelay(ms); }
    }
}

/* ============================================================
 * 主命令
 * ============================================================ */
static void cmd_cl(int argc, char **argv)
{
    if (argc < 2) {
        rt_kprintf("usage: cl list\n");
        rt_kprintf("       cl show <inst>\n");
        rt_kprintf("       cl set <inst> <pos|spd|cur> <kp|ki|kd> <val_x100>\n");
        rt_kprintf("       cl period <inst> <pos|spd|cur> <ms>\n");
        rt_kprintf("       cl ring <inst> <pos|spd|cur> <0|1>\n");
        rt_kprintf("       cl enable <inst> <0|1>\n");
        rt_kprintf("       cl target <inst> <pos|spd|cur> <val>\n");
        rt_kprintf("       cl reset <inst> [pos|spd|cur]\n");
        rt_kprintf("       cl save\n");
        rt_kprintf("       cl load <inst>\n");
        rt_kprintf("       cl trace <inst> <n> <ms>\n");
        return;
    }

    /* ---------------- list ---------------- */
    if (strcmp(argv[1], "list") == 0) {
        if (sys_cap_require(SYS_CAP_DIAG_READ, "cl list") == 0U) { return; }
        _cmd_list();
        return;
    }

    /* ---------------- save ---------------- */
    if (strcmp(argv[1], "save") == 0) {
        uint8_t i;
        uint8_t n = ctrl_loop_inst_count();
        int ret = 0;

        if (sys_cap_require(SYS_CAP_DIAG_WRITE, "cl save") == 0U) { return; }

        for (i = 0U; i < n; i++) {
            ctrl_loop_t *l = ctrl_loop_get(i);
            if (l != RT_NULL) {
                if (ctrl_loop_cfg_store(i, l) != 0) { ret = -1; }
            }
        }
        if (ret == 0) {
            if (ctrl_loop_cfg_save() != 0) { ret = -1; }
        }

        /* 【2026-11-10】拆三元表达式：原写法同一行内既有 OK 又有 FAIL，
         * 无论整体染绿还是染红都会误导。拆开后各分支独立着色。 */
        if (ret == 0) {
            LOG_OK("  save OK\n");
        } else {
            LOG_ERR("  save FAIL\n");
        }
        return;
    }

    /* ---------------- 以下命令都需要 <inst> ---------------- */
    if (argc < 3) {
        rt_kprintf("  need <inst>\n");
        return;
    }

    {
        int inst_i = atoi(argv[2]);

        if ((inst_i < 0) || (inst_i >= (int)CTRL_LOOP_MAX_INST)) {
            rt_kprintf("  inst must be 0..%u\n",
                       (unsigned)(CTRL_LOOP_MAX_INST - 1U));
            return;
        }

        /* -------- show -------- */
        if (strcmp(argv[1], "show") == 0) {
            if (sys_cap_require(SYS_CAP_DIAG_READ, "cl show") == 0U) { return; }
            _cmd_show((uint8_t)inst_i);
            return;
        }

        /* -------- load -------- */
        if (strcmp(argv[1], "load") == 0) {
            ctrl_loop_t *l = ctrl_loop_get((uint8_t)inst_i);
            int ret;

            if (sys_cap_require(SYS_CAP_DIAG_WRITE, "cl load") == 0U) { return; }
            if (l == RT_NULL) { rt_kprintf("  not registered\n"); return; }

            ret = ctrl_loop_cfg_load((uint8_t)inst_i, l);
            /* 【2026-11-10】同 save：拆三元表达式以支持分级着色 */
            if (ret == 0) {
                LOG_OK("  load OK\n");
            } else {
                LOG_ERR("  load FAIL\n");
            }
            _cmd_show((uint8_t)inst_i);
            return;
        }

        /* -------- trace -------- */
        if (strcmp(argv[1], "trace") == 0) {
            int n  = (argc >= 4) ? atoi(argv[3]) : 20;
            int ms = (argc >= 5) ? atoi(argv[4]) : 100;

            if (sys_cap_require(SYS_CAP_DIAG_READ, "cl trace") == 0U) { return; }
            _cmd_trace((uint8_t)inst_i, n, ms);
            return;
        }

        /* -------- enable -------- */
        if (strcmp(argv[1], "enable") == 0) {
            ctrl_loop_t *l = ctrl_loop_get((uint8_t)inst_i);

            if (argc < 4) { rt_kprintf("  need 0|1\n"); return; }
            if (sys_cap_require(SYS_CAP_DIAG_WRITE, "cl enable") == 0U) { return; }
            if (l == RT_NULL) { rt_kprintf("  not registered\n"); return; }

            l->enabled = (atoi(argv[3]) != 0) ? 1U : 0U;
            rt_kprintf("  inst%u enabled=%u\n", (unsigned)inst_i,
                       (unsigned)l->enabled);
            return;
        }

        /* -------- reset -------- */
        if (strcmp(argv[1], "reset") == 0) {
            ctrl_loop_t *l = ctrl_loop_get((uint8_t)inst_i);

            if (sys_cap_require(SYS_CAP_DIAG_WRITE, "cl reset") == 0U) { return; }
            if (l == RT_NULL) { rt_kprintf("  not registered\n"); return; }

            if (argc >= 4) {
                int8_t r = _ring_from_str(argv[3]);
                if (r < 0) { rt_kprintf("  ring must be pos|spd|cur\n"); return; }
                ctrl_loop_reset(l, (ctrl_loop_kind_t)r);
            } else {
                ctrl_loop_reset(l, CTRL_LOOP_POSITION);
                ctrl_loop_reset(l, CTRL_LOOP_SPEED);
                ctrl_loop_reset(l, CTRL_LOOP_CURRENT);
            }
            rt_kprintf("  reset done\n");
            return;
        }

        /* -------- 以下需要 <ring> -------- */
        if (argc < 4) {
            rt_kprintf("  need <pos|spd|cur>\n");
            return;
        }

        {
            int8_t ring = _ring_from_str(argv[3]);
            ctrl_loop_t *l = ctrl_loop_get((uint8_t)inst_i);

            if (ring < 0) {
                rt_kprintf("  ring must be pos|spd|cur\n");
                return;
            }
            if (l == RT_NULL) {
                rt_kprintf("  not registered\n");
                return;
            }

            /* -------- set -------- */
            if (strcmp(argv[1], "set") == 0) {
                int8_t p;
                int32_t v;

                if (argc < 6) {
                    rt_kprintf("  need <kp|ki|kd> <val>\n");
                    return;
                }
                if (sys_cap_require(SYS_CAP_DIAG_WRITE, "cl set") == 0U) { return; }

                p = _param_from_str(argv[4]);
                v = (int32_t)atoi(argv[5]);
                _cmd_set((uint8_t)inst_i, ring, p, v);
                return;
            }

            /* -------- period -------- */
            if (strcmp(argv[1], "period") == 0) {
                int ms = (argc >= 5) ? atoi(argv[4]) : 0;

                if (sys_cap_require(SYS_CAP_DIAG_WRITE, "cl period") == 0U) { return; }

                if (ctrl_loop_set_period(l, (ctrl_loop_kind_t)ring,
                                         (uint32_t)ms) != 0) {
                    rt_kprintf("  invalid period %d\n", ms);
                    return;
                }
                rt_kprintf("  inst%u %s period = %dms\n",
                           (unsigned)inst_i,
                           ctrl_loop_kind_name((ctrl_loop_kind_t)ring), ms);
                return;
            }

            /* -------- ring -------- */
            if (strcmp(argv[1], "ring") == 0) {
                int en;

                if (argc < 5) { rt_kprintf("  need 0|1\n"); return; }
                if (sys_cap_require(SYS_CAP_DIAG_WRITE, "cl ring") == 0U) { return; }

                en = atoi(argv[4]);
                l->ring[ring].cfg.enable = (en != 0) ? 1U : 0U;
                ctrl_loop_reset(l, (ctrl_loop_kind_t)ring);

                rt_kprintf("  inst%u %s enable = %u\n",
                           (unsigned)inst_i,
                           ctrl_loop_kind_name((ctrl_loop_kind_t)ring),
                           (unsigned)l->ring[ring].cfg.enable);
                return;
            }

            /* -------- target -------- */
            if (strcmp(argv[1], "target") == 0) {
                int32_t v;

                if (argc < 5) { rt_kprintf("  need <val>\n"); return; }
                if (sys_cap_require(SYS_CAP_DIAG_WRITE, "cl target") == 0U) { return; }

                v = (int32_t)atoi(argv[4]);
                l->target[ring] = v;
                rt_kprintf("  inst%u %s target = %d\n",
                           (unsigned)inst_i,
                           ctrl_loop_kind_name((ctrl_loop_kind_t)ring),
                           (int)v);
                return;
            }
        }
    }

    rt_kprintf("  unknown subcmd: %s\n", argv[1]);
}

MSH_CMD_EXPORT_ALIAS(cmd_cl, cl,
    "ctrl loop: list|show|set|period|ring|enable|target|reset|save|load|trace");