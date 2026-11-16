/**
 * @file    mech_cmd.c
 * @brief   机械配置表 MSH 调试命令（Q-41：本轮仅 MSH，不做 CAN 下发）
 * @note    PLAN-ABS-ANGLE-V1.3 §5.1.9 ④、§7.6
 *
 * ============================================================
 * 为什么只做 MSH
 * ============================================================
 * 用户 2026-10-26 裁定 Q-41 = "先支持 MSH"。理由：
 *   - 0x18FF7F18 已被"机型设置"占用，另开 CAN ID 要连带改协议表、
 *     上位机、权限校验，工作量约 1 个批次；
 *   - 更关键：切 profile 会清掉现场校准。若做成 CAN 自动联动，
 *     等于远端一条帧就能清掉客户的标定结果。机械参数必须走本地 MSH。
 *
 * 命令清单：
 *   t_mech                  打印生效行 + 全部派生量
 *   t_mech_list             列出全部行与合法性
 *   t_mech_set <字段> <值>   改 active 行单字段（先校验，非法拒绝）
 *   t_mech_use <idx> [force] 切换行（会清校准，需 force）
 *   t_mech_copy <src> <dst>  复制一行
 *   t_mech_def              恢复出厂默认
 *   t_mech_check            只跑校验并打印
 * ============================================================
 */

#include "mech_cfg.h"

/* 【2026-11-10 彩标 P4】check PASS=OK，check FAIL=ERR */
#include "../log/log_color.h"
#include "mech_cfg_store.h"
#include "../mode/sys_caps.h"       /* 能力位门禁（2026-10-27） */
#include "../sys_param/sys_param.h"
#include "../config/sys_config.h"   /* 0x0Axx 机械传动选择项（2026-11-04） */
#include "framework/state_engine/sm_assert.h"

#include <string.h>
#include <stdlib.h>
#include <rtthread.h>

/* 【2026-11-04】判断 tbl.row[0] 与候选是否等价。
 * 定义在 mech_cfg_store.c（static），此处需要同样语义做"无变化则不褪零"判断，
 * 故改为非 static 并在头文件声明，避免两处各写一份比较逻辑（C-05 双定义漂移）。 */
extern uint8_t mech_row_equals_cand(const mech_cfg_t *row, uint8_t cand_id);

/* ============================================================
 * 字段名 → 偏移 的映射表
 * ============================================================ */
typedef enum {
    FLD_U8 = 0,
    FLD_U16,
    FLD_I32,
} fld_type_t;

typedef struct {
    const char *name;
    uint16_t    offset;
    uint8_t     type;
} fld_desc_t;

#define FLD_ENTRY(nm, member, tp) \
    { nm, (uint16_t)offsetof(mech_cfg_t, member), tp }

/* 【2026-11-04 修复：uint8 字段被错标成 FLD_U16 导致写越界】
 *
 * 症状：t_mech_set teeth_tag 2 会把相邻的 out_min_x10 低字节一起改掉，
 *       -100.0°(-1000=0xFC18) 的低字节 0x18 被写成 0x00 ⇒ 变成 -1024 = -102.4°。
 *       同理 t_mech_set pole_pairs 会连带改掉 teeth_tag。
 *
 * 根因：fld_write() 按 type 决定写多少字节。pole_pairs / teeth_tag 在
 *       mech_cfg_t 里是 uint8_t，标成 FLD_U16 就会写 2 字节，
 *       覆盖紧随其后的字段。结构布局（mech_cfg.h:192-197）：
 *         uint8_t pole_pairs;  uint8_t teeth_tag;
 *         int16_t out_min_x10; int16_t out_max_x10;
 *       三者相邻，越界必然命中。
 *
 * 为什么以前没暴露：这两个字段恰好是本次才第一次要改的（选机型）。
 *                 其它字段都是 U16/I32，标注正确。 */
static const fld_desc_t s_flds[] = {
    FLD_ENTRY("enc_z01_num",        enc_z01_num,        FLD_U16),
    FLD_ENTRY("enc_z01_den",        enc_z01_den,        FLD_U16),
    FLD_ENTRY("enc_z02_num",        enc_z02_num,        FLD_U16),
    FLD_ENTRY("enc_z02_den",        enc_z02_den,        FLD_U16),
    FLD_ENTRY("i_in_num",           i_in_num,           FLD_U16),
    FLD_ENTRY("i_in_den",           i_in_den,           FLD_U16),
    FLD_ENTRY("i_out_num",          i_out_num,          FLD_U16),
    FLD_ENTRY("i_out_den",          i_out_den,          FLD_U16),
    FLD_ENTRY("pole_pairs",         pole_pairs,         FLD_U8),   /* uint8_t */
    FLD_ENTRY("teeth_tag",          teeth_tag,          FLD_U8),   /* uint8_t */
    FLD_ENTRY("out_min_x10",        out_min_x10,        FLD_U16),
    FLD_ENTRY("out_max_x10",        out_max_x10,        FLD_U16),
    FLD_ENTRY("tol_override_x1000", tol_override_x1000, FLD_I32),
};

#define FLD_COUNT   (sizeof(s_flds) / sizeof(s_flds[0]))

/* 便捷读写（按偏移 + 类型） */
static void fld_write(mech_cfg_t *cfg, const fld_desc_t *f, int32_t v)
{
    uint8_t *p = (uint8_t *)cfg + f->offset;

    switch (f->type) {
        case FLD_U8:  *(uint8_t  *)p = (uint8_t)v;            break;
        case FLD_U16: *(uint16_t *)p = (uint16_t)v;           break;
        case FLD_I32: *(int32_t  *)p = v;                     break;
        default: break;
    }
}

static int32_t fld_read(const mech_cfg_t *cfg, const fld_desc_t *f)
{
    const uint8_t *p = (const uint8_t *)cfg + f->offset;

    switch (f->type) {
        case FLD_U8:  return (int32_t)(*(const uint8_t  *)p);
        case FLD_U16: return (int32_t)(*(const uint16_t *)p);
        case FLD_I32: return *(const int32_t *)p;
        default:      return 0;
    }
}

static const fld_desc_t* find_fld(const char *name)
{
    uint32_t i;

    for (i = 0U; i < FLD_COUNT; i++) {
        if (strcmp(s_flds[i].name, name) == 0) {
            return &s_flds[i];
        }
    }
    return RT_NULL;
}

/* ============================================================
 * t_mech
 * ============================================================ */
static void cmd_mech_show(void)
{
    if (g_mech.is_loaded == 0U) {
        rt_kprintf("[MECH] not initialized\n");
        return;
    }

    rt_kprintf("[MECH] table: count=%u active=%u row_size=%u size=%uB blk=%uB\n",
               (unsigned)g_mech.tbl.count, (unsigned)g_mech.tbl.active,
               (unsigned)g_mech.tbl.row_size,
               (unsigned)sizeof(mech_table_t),
               (unsigned)(sizeof(mech_table_t) + 8U));

    mech_cfg_dump(&g_mech.cfg, &g_mech.rt, RT_NULL);

    rt_kprintf("[MECH] status: cfg_fault=%u active=%d teeth_tag=%u src=%s\n",
               (unsigned)g_mech.cfg_fault, (int)g_mech.active_idx,
               (unsigned)g_mech.cfg.teeth_tag,
               (g_mech.src_is_flash != 0U) ? "flash" : "default");
}

/* ============================================================
 * t_mech_list
 * ============================================================ */
static void cmd_mech_list(void)
{
    if (g_mech.is_loaded == 0U) {
        rt_kprintf("[MECH] not initialized\n");
        return;
    }
    mech_table_dump(&g_mech.tbl);
}

/* ============================================================
 * t_mech_set <字段> <值>
 * ============================================================ */
static void cmd_mech_set(int argc, char **argv)
{
    const fld_desc_t *f;
    mech_cfg_t        bak;
    mech_rt_t         tmp_rt;
    char              err[48];
    int32_t           v;

    if (argc < 3) {
        rt_kprintf("usage: t_mech_set <field> <value>\n");
        rt_kprintf("fields:");
        {
            uint32_t i;
            for (i = 0U; i < FLD_COUNT; i++) {
                rt_kprintf(" %s", s_flds[i].name);
            }
        }
        rt_kprintf("\n");
        return;
    }

    f = find_fld(argv[1]);
    if (f == RT_NULL) {
        rt_kprintf("[MECH] unknown field '%s'\n", argv[1]);
        return;
    }

    v = (int32_t)atoi(argv[2]);

    /* 先备份，供失败回滚 */
    bak = g_mech.tbl.row[g_mech.tbl.active];

    fld_write(&g_mech.tbl.row[g_mech.tbl.active], f, v);

    /* 校验该行（含 V1~V8）；非法立即回滚，绝不落盘 */
    if (mech_cfg_derive(&g_mech.tbl.row[g_mech.tbl.active], &tmp_rt,
                        err, sizeof(err)) != MECH_OK) {
        g_mech.tbl.row[g_mech.tbl.active] = bak;
        rt_kprintf("[MECH] reject %s=%d: %s\n", f->name, (int)v, err);
        return;
    }

    /* 整表校验：防止改坏 T4/T6 这类表级约束 */
    if (mech_table_validate(&g_mech.tbl, err, sizeof(err)) != MECH_OK) {
        g_mech.tbl.row[g_mech.tbl.active] = bak;
        rt_kprintf("[MECH] reject %s=%d: %s\n", f->name, (int)v, err);
        return;
    }

    rt_kprintf("[MECH] %s: %d -> %d (validated)\n",
               f->name, (int)fld_read(&bak, f), (int)v);

    /* 生效并落盘 */
    if (mech_apply_active() != MECH_OK) {
        g_mech.tbl.row[g_mech.tbl.active] = bak;
        (void)mech_apply_active();
        rt_kprintf("[MECH] derive failed after set, rolled back\n");
        return;
    }
    (void)mech_table_save(&g_mech.tbl);
}

/* ============================================================
 * t_mech_use <idx> [force]
 * ============================================================ */
static void cmd_mech_use(int argc, char **argv)
{
    uint8_t idx;
    uint8_t force = 0U;

    if (argc < 2) {
        rt_kprintf("usage: t_mech_use <idx> [force]\n");
        rt_kprintf("  WARNING: switching profile will ERASE calibration.\n");
        return;
    }

    idx = (uint8_t)atoi(argv[1]);
    if (argc >= 3 && strcmp(argv[2], "force") == 0) {
        force = 1U;
    }

    (void)mech_set_active_profile(idx, force);
}

/* ============================================================
 * t_mech_copy <src> <dst>
 * ============================================================ */
static void cmd_mech_copy(int argc, char **argv)
{
    uint8_t src, dst;
    char    err[48];

    if (argc < 3) {
        rt_kprintf("usage: t_mech_copy <src_idx> <dst_idx>\n");
        return;
    }

    src = (uint8_t)atoi(argv[1]);
    dst = (uint8_t)atoi(argv[2]);

    if (src >= g_mech.tbl.count) {
        rt_kprintf("[MECH] src %u out of range\n", (unsigned)src);
        return;
    }
    if (dst >= MECH_PROFILE_MAX) {
        rt_kprintf("[MECH] dst %u >= MAX(%u)\n", (unsigned)dst,
                   (unsigned)MECH_PROFILE_MAX);
        return;
    }

    g_mech.tbl.row[dst] = g_mech.tbl.row[src];

    /* 新行默认标 0（兜底），避免与 src 的 teeth_tag 重复触发 T4。
     * 需要指定机型时用 t_mech_set teeth_tag <n>。 */
    g_mech.tbl.row[dst].teeth_tag = 0U;

    /* 扩 count 以包含 dst */
    if (dst >= g_mech.tbl.count) {
        g_mech.tbl.count = (uint8_t)(dst + 1U);
    }

    if (mech_table_validate(&g_mech.tbl, err, sizeof(err)) != MECH_OK) {
        rt_kprintf("[MECH] copy result invalid: %s\n", err);
        return;
    }

    (void)mech_table_save(&g_mech.tbl);
    rt_kprintf("[MECH] copied row %u -> %u (teeth_tag reset to 0)\n",
               (unsigned)src, (unsigned)dst);
}

/* ============================================================
 * t_mech_def
 * ============================================================ */
static void cmd_mech_def(void)
{
    mech_table_default(&g_mech.tbl);

    if (mech_apply_active() != MECH_OK) {
        rt_kprintf("[MECH] default row derive failed!\n");
        return;
    }

    (void)mech_table_save(&g_mech.tbl);
    g_mech.cfg_fault = 0U;

    rt_kprintf("[MECH] restored factory default (count=1, row0)\n");
    mech_cfg_dump(&g_mech.cfg, &g_mech.rt, &g_mech.tbl);
}

/* ============================================================
 * t_mech_check
 * ============================================================ */
static void cmd_mech_check(void)
{
    char err[48];
    int  rc;

    rc = mech_table_validate(&g_mech.tbl, err, sizeof(err));
    if (rc == MECH_OK) {
        LOG_OK("[MECH] check PASS: table valid (T1~T6, V1~V8 all ok)\n");
    } else {
        LOG_ERR("[MECH] check FAIL: %s\n", err);
    }

    /* 顺带把每行的结论打出来，便于定位是第几行坏 */
    mech_table_dump(&g_mech.tbl);
}

/* ============================================================
 * t_mech_cand —— 【Q-49「按需配置，初始化时选择」】
 *
 *   t_mech_cand            列出全部出厂候选 + 逐条自检结论
 *   t_mech_cand <id>       选中该候选为兜底行（会褪零，见 C-08）
 *
 * 与 t_mech_use 的区别：
 *   t_mech_use  切的是 tbl.row[] 里**已存在**的行（现场录入的）
 *   t_mech_cand 把**出厂常量候选**填进 row[0]，用于"初始化时选择"
 * 两者都必须褪零：换了齿数/减速比，旧 cnt0_zero 在新模型下无意义。
 * ============================================================ */
static void cmd_mech_cand(int argc, char **argv)
{
    if (argc < 2) {
        mech_cand_selftest();
        rt_kprintf("[MECH] usage: t_mech_cand <id 0..%u>  (erases calibration)\n",
                   (unsigned)(MECH_CAND_COUNT - 1U));
        return;
    }

    {
        int      id = atoi(argv[1]);
        uint8_t  n  = (uint8_t)id;

        if (id < 0 || n >= MECH_CAND_COUNT) {
            rt_kprintf("[MECH] id %d out of range 0..%u\n",
                       id, (unsigned)(MECH_CAND_COUNT - 1U));
            return;
        }

        /* 1. 填候选（内部先校验，非法不写坏 tbl） */
        if (mech_table_apply_cand(&g_mech.tbl, n) != MECH_OK) {
            rt_kprintf("[MECH] apply cand %u failed, table untouched\n",
                       (unsigned)n);
            return;
        }

        /* 2. 派生 */
        if (mech_apply_active() != MECH_OK) {
            rt_kprintf("[MECH] derive failed, restoring default table\n");
            mech_table_default(&g_mech.tbl);
            (void)mech_apply_active();
            return;
        }

        /* 3. 【C-08】褪零 + 清校准标志，顺序与 mech_set_active_profile 一致 */
        sys_param_clear_calibration();
        abs_angle_invalidate();

        /* 4. 落盘 */
        if (mech_table_save(&g_mech.tbl) != MECH_OK) {
            rt_kprintf("[MECH] save failed, calibration already cleared!\n");
            return;
        }

        g_mech.cfg_fault = 0U;
        rt_kprintf("[MECH] selected cand %u: %s, calibration CLEARED\n",
                   (unsigned)n, mech_cand_name(n));
        mech_cfg_dump(&g_mech.cfg, &g_mech.rt, &g_mech.tbl);
    }
}

/* ============================================================
 * t_mech_model —— 【2026-11-04 新增】一条命令切换整套传动参数
 * ------------------------------------------------------------
 *   t_mech_model                    显示当前三项与全部可选值
 *   t_mech_model <enc> <iin> <iout> 切换
 *   t_mech_model <cand_id>          直接按候选 id 切换
 *   t_mech_model <enc> <iin> <iout> save
 *
 * 为什么需要它：
 *   t_mech_set 一次只能改一个字段，换机型要改 i_in_num/i_in_den/
 *   i_out_num/i_out_den 四条，中间任一时刻表都是**不自洽**的
 *   （已落盘、但半改状态），掉电就会留在错误配置上。
 *   本命令把三项一次性合成候选 id，整行替换 + 校验 + 褪零 + 落盘，
 *   原子完成，不存在中间态。
 *
 * 为什么同时同步 sys_config 的 0x0A01/02/03：
 *   上电选行（mech_apply_cfg_selection）读的就是这三项。
 *   若只改 mech_param 而不同步 sys_config，下次上电会按旧配置
 *   把刚设好的行又切回去 —— 典型的"改完不生效"。
 * ============================================================ */
static void cmd_mech_model(int argc, char **argv)
{
    uint8_t  enc_sel, iin_sel, iout_sel;
    uint8_t  cand_id;
    uint8_t  do_save = 1U;
    int32_t  v;

    if (g_mech.is_loaded == 0U) {
        rt_kprintf("[MECH] not initialized\n");
        return;
    }

    /* ---- 无参：显示当前与全部可选 ---- */
    if (argc < 2) {
        int32_t e = 0, i2 = 0, o = 0;
        uint8_t cur;

        (void)sys_config_get_i32(CFG_ID_MECH_ENC_SEL,  &e);
        (void)sys_config_get_i32(CFG_ID_MECH_IIN_SEL,  &i2);
        (void)sys_config_get_i32(CFG_ID_MECH_IOUT_SEL, &o);
        cur = mech_cand_make_id((uint8_t)e, (uint8_t)i2, (uint8_t)o);

        rt_kprintf("[MECH] cfg selection (sys_config 0x0A01/02/03):\n");
        rt_kprintf("  enc_sel  = %d  (%s)\n", (int)e, mech_enc_sel_name((uint8_t)e));
        rt_kprintf("  iin_sel  = %d  (%s)\n", (int)i2, mech_iin_sel_name((uint8_t)i2));
        rt_kprintf("  iout_sel = %d  (%s)\n", (int)o, mech_iout_sel_name((uint8_t)o));
        rt_kprintf("  -> cand id = %u (%s)\n",
                   (unsigned)cur, mech_cand_name(cur));

        rt_kprintf("[MECH] active row: z01=%u:%u z02=%u:%u i=%u/%u*%u/%u\n",
                   (unsigned)g_mech.cfg.enc_z01_num, (unsigned)g_mech.cfg.enc_z01_den,
                   (unsigned)g_mech.cfg.enc_z02_num, (unsigned)g_mech.cfg.enc_z02_den,
                   (unsigned)g_mech.cfg.i_in_num, (unsigned)g_mech.cfg.i_in_den,
                   (unsigned)g_mech.cfg.i_out_num, (unsigned)g_mech.cfg.i_out_den);
        rt_kprintf("[MECH] i_total=%d.%02d res=%u ndeg/LSB\n",
                   (int)(g_mech.rt.i_total_x100 / 100),
                   (int)(g_mech.rt.i_total_x100 % 100),
                   (unsigned)g_mech.rt.res_ndeg_per_lsb);
        rt_kprintf("\n  enc_sel : 0=41:42:43   1=55:56:57\n");
        rt_kprintf("  iin_sel : 0=136.6      1=216\n");
        rt_kprintf("  iout_sel: 0=65:13(13T) 1=75:14(14T) 2=53:11(11T)\n");
        rt_kprintf("usage: t_mech_model <enc> <iin> <iout>\n");
        rt_kprintf("       t_mech_model <cand_id>       (0..%u)\n",
                   (unsigned)(MECH_CAND_COUNT - 1U));
        rt_kprintf("  WARNING: switching always ERASES calibration (C-08)\n");
        return;
    }

    /* ---- 解析参数 ---- */
    if (argc == 2) {
        /* 单参 = 候选 id 直选 */
        v = (int32_t)atoi(argv[1]);
        if ((v < 0) || (v >= (int32_t)MECH_CAND_COUNT)) {
            rt_kprintf("[MECH] cand id %d out of range 0..%u\n",
                       (int)v, (unsigned)(MECH_CAND_COUNT - 1U));
            return;
        }
        cand_id = (uint8_t)v;
        if (mech_cand_split_id(cand_id, &enc_sel, &iin_sel, &iout_sel) != MECH_OK) {
            rt_kprintf("[MECH] split id %u failed\n", (unsigned)cand_id);
            return;
        }
    } else {
        /* 三参 = (enc, iin, iout) */
        int32_t e = (int32_t)atoi(argv[1]);
        int32_t i2 = (int32_t)atoi(argv[2]);
        int32_t o = (int32_t)atoi(argv[3]);

        if ((e < 0) || (e >= (int32_t)MECH_ENC_SEL_COUNT) ||
            (i2 < 0) || (i2 >= (int32_t)MECH_IIN_SEL_COUNT) ||
            (o < 0) || (o >= (int32_t)MECH_IOUT_SEL_COUNT)) {
            rt_kprintf("[MECH] sel out of range: enc=%d(0..%u) iin=%d(0..%u) "
                       "iout=%d(0..%u)\n",
                       (int)e, (unsigned)(MECH_ENC_SEL_COUNT - 1U),
                       (int)i2, (unsigned)(MECH_IIN_SEL_COUNT - 1U),
                       (int)o, (unsigned)(MECH_IOUT_SEL_COUNT - 1U));
            return;
        }
        enc_sel  = (uint8_t)e;
        iin_sel  = (uint8_t)i2;
        iout_sel = (uint8_t)o;
        cand_id  = mech_cand_make_id(enc_sel, iin_sel, iout_sel);

        if (argc >= 5 && strcmp(argv[4], "save") != 0) {
            do_save = 1U;   /* 保持默认；多余的参数只警告不报错 */
        }
    }
    (void)do_save;

    if (cand_id >= MECH_CAND_COUNT) {
        rt_kprintf("[MECH] computed cand id %u invalid\n", (unsigned)cand_id);
        return;
    }

    /* ---- 已是目标组合：不动，避免无谓褪零 ---- */
    if (mech_row_equals_cand(&g_mech.tbl.row[0], cand_id) != 0U) {
        rt_kprintf("[MECH] already = cand %u (%s), nothing changed\n",
                   (unsigned)cand_id, mech_cand_name(cand_id));
        return;
    }

    rt_kprintf("[MECH] switching to cand %u: %s\n",
               (unsigned)cand_id, mech_cand_name(cand_id));

    /* ---- 1. 填候选（内部先校验，非法不写坏 tbl）---- */
    if (mech_table_apply_cand(&g_mech.tbl, cand_id) != MECH_OK) {
        rt_kprintf("[MECH] apply cand %u failed, table untouched\n",
                   (unsigned)cand_id);
        return;
    }

    /* ---- 2. 派生 ---- */
    if (mech_apply_active() != MECH_OK) {
        rt_kprintf("[MECH] derive failed, restoring factory default\n");
        mech_table_default(&g_mech.tbl);
        (void)mech_apply_active();
        return;
    }

    /* ---- 3. 同步 sys_config 三项（否则下次上电会被切回去）---- */
    (void)sys_config_set_i32(CFG_ID_MECH_ENC_SEL,  (int32_t)enc_sel);
    (void)sys_config_set_i32(CFG_ID_MECH_IIN_SEL,  (int32_t)iin_sel);
    (void)sys_config_set_i32(CFG_ID_MECH_IOUT_SEL, (int32_t)iout_sel);
    if (sys_config_save() == 0) {
        rt_kprintf("[MECH] sys_config synced (enc=%u iin=%u iout=%u)\n",
                   (unsigned)enc_sel, (unsigned)iin_sel, (unsigned)iout_sel);
    } else {
        LOG_ERR("[MECH] WARN: sys_config save FAILED, "
                   "next boot may revert!\n");
    }

    /* ---- 4. 【C-08】褪零 + 让角度层重跑解算 ---- */
    sys_param_clear_calibration();
    abs_angle_invalidate();

    /* ---- 5. 落盘 ---- */
    if (mech_table_save(&g_mech.tbl) != MECH_OK) {
        rt_kprintf("[MECH] save failed, calibration already cleared!\n");
        return;
    }

    g_mech.cfg_fault = 0U;
    rt_kprintf("[MECH] switched OK, calibration CLEARED\n");
    mech_cfg_dump(&g_mech.cfg, &g_mech.rt, &g_mech.tbl);
}

/* ============================================================
 * MSH 注册
 * ============================================================ */
static void t_mech(int argc, char **argv)
{
    cmd_mech_show();
    (void)argc; (void)argv;
}

static void t_mech_list(int argc, char **argv)
{
    cmd_mech_list();
    (void)argc; (void)argv;
}

static void t_mech_set(int argc, char **argv)
{
    /* 【2026-10-27】机型表字段写入 = 改传动参数，工作模式必须禁掉。
     * 表配置见《模式权限静态配置表-V1.0》M-30（F/Y M/Y W/N）。 */
    if (sys_cap_require(SYS_CAP_PARAM_WRITE, "t_mech_set") == 0U) { return; }
    cmd_mech_set(argc, argv);
}

static void t_mech_use(int argc, char **argv)
{
    /* 切换机型会 ERASE 校准 ⇒ 除写参数外还消耗 CALIB 能力 */
    if (sys_cap_require(SYS_CAP_PARAM_WRITE, "t_mech_use") == 0U) { return; }
    if (sys_cap_require(SYS_CAP_CALIB,       "t_mech_use") == 0U) { return; }
    cmd_mech_use(argc, argv);
}

static void t_mech_copy(int argc, char **argv)
{
    if (sys_cap_require(SYS_CAP_PARAM_WRITE, "t_mech_copy") == 0U) { return; }
    cmd_mech_copy(argc, argv);
}

static void t_mech_def(int argc, char **argv)
{
    /* 恢复出厂默认会重建整表，隐含清除校准基准 */
    if (sys_cap_require(SYS_CAP_PARAM_WRITE, "t_mech_def") == 0U) { return; }
    cmd_mech_def();
    (void)argc; (void)argv;
}

static void t_mech_check(int argc, char **argv)
{
    cmd_mech_check();
    (void)argc; (void)argv;
}

static void t_mech_cand(int argc, char **argv)
{
    /* 选择候选机型同样 ERASE 校准（与 t_mech_use 同源） */
    if (sys_cap_require(SYS_CAP_PARAM_WRITE, "t_mech_cand") == 0U) { return; }
    if (sys_cap_require(SYS_CAP_CALIB,       "t_mech_cand") == 0U) { return; }
    cmd_mech_cand(argc, argv);
}

static void t_mech_model(int argc, char **argv)
{
    /* 换机型齿比 = 改传动参数 + 褪零，双重门禁：
     *   PARAM_WRITE 防工作模式下误改
     *   CALIB       换行必清标定，属消耗标定能力的操作 */
    if (sys_cap_require(SYS_CAP_PARAM_WRITE, "t_mech_model") == 0U) { return; }
    if (sys_cap_require(SYS_CAP_CALIB,       "t_mech_model") == 0U) { return; }
    cmd_mech_model(argc, argv);
}

MSH_CMD_EXPORT_ALIAS(t_mech,       t_mech,       show active mech profile);
MSH_CMD_EXPORT_ALIAS(t_mech_list,  t_mech_list,  list all mech profiles);
MSH_CMD_EXPORT_ALIAS(t_mech_set,   t_mech_set,   set one field of active profile);
MSH_CMD_EXPORT_ALIAS(t_mech_use,   t_mech_use,   switch profile (erases calibration));
MSH_CMD_EXPORT_ALIAS(t_mech_copy,  t_mech_copy,  copy one profile row);
MSH_CMD_EXPORT_ALIAS(t_mech_def,   t_mech_def,   restore factory default table);
MSH_CMD_EXPORT_ALIAS(t_mech_check, t_mech_check, validate table only);
MSH_CMD_EXPORT_ALIAS(t_mech_cand,  t_mech_cand,  list/select factory cand (erases cal));
MSH_CMD_EXPORT_ALIAS(t_mech_model, t_mech_model, select enc/iin/iout combo (erases cal));