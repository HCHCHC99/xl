/**
 * @file    mech_cfg_store.c
 * @brief   机械配置表持久化实现
 * @note    照 mode_config.c 的 storage 用法，保持工程风格一致
 */

#include "mech_cfg_store.h"
#include "mech_cfg.h"

/* 【2026-11-10】彩色日志：ERROR 亮红、WARN 亮黄、init/final 常规色 */
#include "../log/log_color.h"
#include "../param/param_store.h"
#include "../sys_param/sys_param.h"
#include "../config/sys_config.h"     /* 0x0Axx 机械传动选择项 */
#include "framework/state_engine/sm_assert.h"

#include <string.h>
#include <rtthread.h>

/* ============================================================
 * 全局对象
 * ============================================================ */
mech_ctx_t g_mech;

/* ============================================================
 * 弱符号：角度层失效通知
 * ------------------------------------------------------------
 * abs_angle 模块尚未接入时，此处为空实现，链接不会失败。
 * abs_angle.c 接入后提供同名强符号即可自动覆盖（无需改本文件）。
 * 注意：本工程用 RT-Thread 的 RT_WEAK（GCC 下展开为 __attribute__((weak))），
 *       没有 __WEAK 这个宏 —— 曾误用导致 "expected ';' before 'void'"。
 * ============================================================ */
RT_WEAK void abs_angle_invalidate(void)
{
    /* 无 abs_angle 模块时无操作 */
}

/* ============================================================
 * 加载
 *
 * 新存储方案只持久化 active 行（32B）+ active/count + 三选择子。
 * 本工程"按需配置"始终 count=1，加载时把该行铺满整张表即可通过
 * T1~T6 校验；多行 profile 不在本次范围内。
 * ============================================================ */
int mech_table_load(mech_table_t *tbl)
{
    mech_cfg_t row;
    uint8_t    active = 0U;
    uint8_t    count  = 1U;
    uint8_t    i;
    char       err[48];

    SM_ASSERT_PTR(tbl);

    (void)param_store_init();

    if (param_store_am_get_row(&row, &active, &count) != 0) {
        rt_kprintf("[MECH] no valid angle_mech record\n");
        return MECH_ERR_IO;
    }

    if ((count == 0U) || (count > MECH_PROFILE_MAX)) {
        count = 1U;
    }
    if (active >= count) {
        active = 0U;
    }

    memset(tbl, 0, sizeof(*tbl));
    tbl->version  = MECH_TABLE_VERSION;
    tbl->row_size = (uint16_t)sizeof(mech_cfg_t);

    if (row.teeth_tag == 0U) {
        tbl->count  = count;
        tbl->active = active;
        for (i = 0U; i < MECH_PROFILE_MAX; i++) {
            tbl->row[i] = row;
        }
    } else {
        /* t_mech_set 可能把 teeth_tag 改成非 0；T6 要求存在兜底行，
         * 因此复制一份 teeth_tag=0 的行放在 row[0]，active 行放 row[1]。 */
        tbl->row[0] = row;
        tbl->row[0].teeth_tag = 0U;
        tbl->row[1] = row;
        for (i = 2U; i < MECH_PROFILE_MAX; i++) {
            tbl->row[i] = tbl->row[1];
        }
        tbl->count  = 2U;
        tbl->active = 1U;
    }

    if (mech_table_validate(tbl, err, sizeof(err)) != MECH_OK) {
        rt_kprintf("[MECH] table invalid: %s\n", err);
        return MECH_ERR_VALIDATE;
    }

    return MECH_OK;
}

int mech_table_save(const mech_table_t *tbl)
{
    char err[48];

    SM_ASSERT_PTR(tbl);

    /* 写前必校验：非法配置绝不落盘（否则"改完不生效也不报错"） */
    if (mech_table_validate(tbl, err, sizeof(err)) != MECH_OK) {
        rt_kprintf("[MECH] refuse to save: %s\n", err);
        return MECH_ERR_VALIDATE;
    }

    (void)param_store_init();

    if (param_store_am_set_row(&tbl->row[tbl->active],
                               tbl->active, tbl->count) != 0) {
        return MECH_ERR_IO;
    }
    if (param_store_am_save() != 0) {
        rt_kprintf("[MECH] save request failed\n");
        return MECH_ERR_IO;
    }

    rt_kprintf("[MECH] saved (count=%u active=%u)\n",
               (unsigned)tbl->count, (unsigned)tbl->active);
    return MECH_OK;
}

/* ============================================================
 * 派生量刷新
 * ============================================================ */
int mech_apply_active(void)
{
    char err[48];
    int  rc;

    if (g_mech.tbl.active >= g_mech.tbl.count) {
        return MECH_ERR_PARAM;
    }

    /* 先拷配置，再算派生量 */
    g_mech.cfg = g_mech.tbl.row[g_mech.tbl.active];
    g_mech.active_idx = (int32_t)g_mech.tbl.active;

    rc = mech_cfg_derive(&g_mech.cfg, &g_mech.rt, err, sizeof(err));
    if (rc != MECH_OK) {
        rt_kprintf("[MECH] derive failed: %s\n", err);
        return rc;
    }
    return MECH_OK;
}

/* ============================================================
 * C-08：切换 profile 必须原子褪零
 * ============================================================ */
int mech_set_active_profile(uint8_t idx, uint8_t force)
{
    char err[48];

    if (idx >= g_mech.tbl.count) {
        rt_kprintf("[MECH] idx %u out of range (count=%u)\n",
                   (unsigned)idx, (unsigned)g_mech.tbl.count);
        return MECH_ERR_PARAM;
    }

    /* 1. 先校验目标行，非法直接拒绝，什么都不动 */
    if (mech_cfg_validate(&g_mech.tbl.row[idx], err, sizeof(err)) != MECH_OK) {
        rt_kprintf("[MECH] target row %u illegal: %s\n", (unsigned)idx, err);
        return MECH_ERR_VALIDATE;
    }

    /* 2. 未确认则只警告，不执行 */
    if (force == 0U) {
        rt_kprintf("[MECH] WARNING: switch to profile %u (teeth_tag=%u) "
                   "will ERASE calibration!\n",
                   (unsigned)idx, (unsigned)g_mech.tbl.row[idx].teeth_tag);
        rt_kprintf("[MECH]          current zero=%d is from profile %u.\n",
                   (int)sys_param_get()->encoder_zero_raw,
                   (unsigned)g_mech.tbl.active);
        rt_kprintf("[MECH]          reply 't_mech_use %u force' to confirm.\n",
                   (unsigned)idx);
        return MECH_ERR_PARAM;
    }

    /* 3. 先落内存派生（失败则保持旧 profile，状态不动） */
    g_mech.tbl.active = idx;
    if (mech_apply_active() != MECH_OK) {
        /* 回滚 active，避免半切状态 */
        rt_kprintf("[MECH] derive failed, rollback active\n");
        return MECH_ERR_VALIDATE;
    }

    /* 4. 【C-08】褪零 + 清校准标志（一次 Flash 写完成）
     *    顺序不可换：先清状态、后落盘 active，掉电后是"未校准"可自愈；
     *    反序会得到"新行 + 旧零点 + 已校准标志"，无法自愈。 */
    sys_param_clear_calibration();

    /* 5. 通知角度层重跑游标解算。
     *    用 __WEAK 弱符号，避免本模块强依赖尚未接入的 abs_angle。 */
    abs_angle_invalidate();

    /* 6. 落盘 */
    if (mech_table_save(&g_mech.tbl) != MECH_OK) {
        rt_kprintf("[MECH] save failed, calibration already cleared!\n");
        return MECH_ERR_IO;
    }

    rt_kprintf("[MECH] switched to profile %u, calibration CLEARED\n",
               (unsigned)idx);
    return MECH_OK;
}

int mech_select_by_teeth(uint8_t teeth_type)
{
    int idx = mech_find_profile(&g_mech.tbl, teeth_type);

    if (idx < 0) {
        rt_kprintf("[MECH] no profile for teeth_type=%u, keep current\n",
                   (unsigned)teeth_type);
        return MECH_ERR_PARAM;
    }
    if ((uint8_t)idx == g_mech.tbl.active) {
        return MECH_OK;   /* 已经是这一行，不动校准 */
    }
    /* 换行必然褪零（C-08） */
    return mech_set_active_profile((uint8_t)idx, 1U);
}

const mech_cfg_t* mech_get_cfg(void) { return &g_mech.cfg; }
const mech_rt_t*  mech_get_rt(void)  { return &g_mech.rt;  }

/* ============================================================
 * 【2026-11-04】按 sys_config 的三个选择子上电选行
 * ------------------------------------------------------------
 * 用户裁定 Q5="一并纳入"，即 2 套编码器 × 2 套内部减速比 × 3 套机型齿比
 * 的 12 组合全部内置于候选表，上电按配置表选其一。
 *
 * 【为什么读 sys_config 而不是 sys_param.teeth_type】
 *   sys_param.teeth_type 只表达"几齿机型"（11/13/14），
 *   不足以区分编码器齿数比与电机内部减速比。三项必须独立。
 *
 * 【为什么与当前行"相同就不动"】
 *   褪零是破坏性操作（sys_param_clear_calibration 会清掉现场标定）。
 *   若每次上电都无条件 apply + 褪零，设备每重启一次就丢一次标定。
 *   因此只在"目标行与 Flash 里已落盘的行确实不同"时才褪零重写。
 *
 * 【选择子非法时的处理】
 *   sys_config 层已按 [min,max] 逐项校验，越界项会回退默认值。
 *   此处再兜一层：任一选择子超出 SEL_COUNT 就用 MECH_CAND_DEFAULT，
 *   并打印告警 —— 宁可回到已验证的默认组合，也不能选到不存在的候选。
 * ============================================================ */
static uint8_t mech_sel_from_cfg(void)
{
    int32_t enc = (int32_t)MECH_ENC_SEL_555657;
    int32_t iin = (int32_t)MECH_IIN_SEL_216;
    int32_t iout = (int32_t)MECH_IOUT_SEL_65_13;
    uint8_t id;

    (void)sys_config_get_i32(CFG_ID_MECH_ENC_SEL,  &enc);
    (void)sys_config_get_i32(CFG_ID_MECH_IIN_SEL,  &iin);
    (void)sys_config_get_i32(CFG_ID_MECH_IOUT_SEL, &iout);

    /* 显式范围检查：三个选择子任一越界就整体回默认组合 */
    if ((enc  < 0) || (enc  >= (int32_t)MECH_ENC_SEL_COUNT) ||
        (iin  < 0) || (iin  >= (int32_t)MECH_IIN_SEL_COUNT) ||
        (iout < 0) || (iout >= (int32_t)MECH_IOUT_SEL_COUNT)) {
        LOG_WARN("[MECH] WARN: cfg sel out of range (enc=%d iin=%d iout=%d)"
                   " -> use factory default cand %u\n",
                   (int)enc, (int)iin, (int)iout, (unsigned)MECH_CAND_DEFAULT);
        return MECH_CAND_DEFAULT;
    }

    id = mech_cand_make_id((uint8_t)enc, (uint8_t)iin, (uint8_t)iout);
    if (id >= MECH_CAND_COUNT) {
        LOG_WARN("[MECH] WARN: cand id %u invalid -> default %u\n",
                   (unsigned)id, (unsigned)MECH_CAND_DEFAULT);
        return MECH_CAND_DEFAULT;
    }
    return id;
}

/**
 * @brief 判断 tbl.row[0] 与目标候选是否等价
 * @note  只比"影响 i_total 与游标解算"的字段，不比 version/crc32/tol_override
 *        —— 后者变化不需要褪零（tol_override 是调试参数，改它不该清标定）。
 */
uint8_t mech_row_equals_cand(const mech_cfg_t *row, uint8_t cand_id)
{
    mech_cfg_t c;

    if (row == RT_NULL) {
        return 0U;
    }
    if (mech_cand_get(cand_id, &c) != MECH_OK) {
        return 0U;
    }
    if ((row->enc_z01_num != c.enc_z01_num) ||
        (row->enc_z01_den != c.enc_z01_den) ||
        (row->enc_z02_num != c.enc_z02_num) ||
        (row->enc_z02_den != c.enc_z02_den) ||
        (row->i_in_num    != c.i_in_num)    ||
        (row->i_in_den    != c.i_in_den)    ||
        (row->i_out_num   != c.i_out_num)   ||
        (row->i_out_den   != c.i_out_den)   ||
        (row->pole_pairs  != c.pole_pairs)  ||
        (row->out_min_x10 != c.out_min_x10) ||
        (row->out_max_x10 != c.out_max_x10)) {
        return 0U;
    }
    return 1U;
}

/**
 * @brief 上电按配置表选行（仅在确实变化时褪零 + 落盘）
 * @return MECH_OK 无需变更或已成功切换；<0 失败（保持原行，不半切）
 */
int mech_apply_cfg_selection(void)
{
    uint8_t want = mech_sel_from_cfg();
    uint8_t enc_sel = 0U, iin_sel = 0U, iout_sel = 0U;

    (void)mech_cand_split_id(want, &enc_sel, &iin_sel, &iout_sel);

    /* ---------- 已经是目标组合：什么都不做（保护现场标定）---------- */
    if ((g_mech.tbl.count == 1U) &&
        (mech_row_equals_cand(&g_mech.tbl.row[0], want) != 0U)) {
        LOG_OK("[MECH] cfg sel unchanged: cand %u (%s / %s / %s)\n",
                   (unsigned)want,
                   mech_enc_sel_name(enc_sel),
                   mech_iin_sel_name(iin_sel),
                   mech_iout_sel_name(iout_sel));
        return MECH_OK;
    }

    /* ---------- 需要切换 ---------- */
    rt_kprintf("[MECH] cfg sel CHANGED -> cand %u (%s / %s / %s)\n",
               (unsigned)want,
               mech_enc_sel_name(enc_sel),
               mech_iin_sel_name(iin_sel),
               mech_iout_sel_name(iout_sel));

    /* 1. 填候选（mech_table_apply_cand 内部先校验到临时变量，
     *    非法候选绝不写坏现有表，故下面这条分支实际不可达）*/
    if (mech_table_apply_cand(&g_mech.tbl, want) != MECH_OK) {
        /* 不可达的防御分支：候选表是编译期常量，若走到这里说明
         * 候选定义本身非法（常量写错）。此时回出厂默认行最能保命。 */
        LOG_ERR("[MECH] ERROR: apply cand %u failed (cand table broken?)\n",
                   (unsigned)want);
        mech_table_default(&g_mech.tbl);
        (void)mech_apply_active();
        g_mech.cfg_fault = 1U;
        return MECH_ERR_VALIDATE;
    }

    /* 2. 派生（失败则整表回默认，不带病运行）*/
    if (mech_apply_active() != MECH_OK) {
        LOG_ERR("[MECH] ERROR: derive failed, fallback to factory default\n");
        mech_table_default(&g_mech.tbl);
        (void)mech_apply_active();
        g_mech.cfg_fault = 1U;
        return MECH_ERR_VALIDATE;
    }

    /* 3. 【C-08】褪零：换了齿数/减速比，旧零点在新模型下无意义。
     *    顺序不可换 —— 先清状态、后落盘，掉电后是"未校准"可自愈。 */
    sys_param_clear_calibration();
    abs_angle_invalidate();

    /* 4. 落盘 */
    if (mech_table_save(&g_mech.tbl) != MECH_OK) {
        LOG_WARN("[MECH] WARN: save failed, calibration already cleared!\n");
        return MECH_ERR_IO;
    }

    rt_kprintf("[MECH] switched: z01=%u:%u z02=%u:%u i=%u/%u*%u/%u calib CLEARED\n",
               (unsigned)g_mech.cfg.enc_z01_num, (unsigned)g_mech.cfg.enc_z01_den,
               (unsigned)g_mech.cfg.enc_z02_num, (unsigned)g_mech.cfg.enc_z02_den,
               (unsigned)g_mech.cfg.i_in_num, (unsigned)g_mech.cfg.i_in_den,
               (unsigned)g_mech.cfg.i_out_num, (unsigned)g_mech.cfg.i_out_den);
    return MECH_OK;
}

/* ============================================================
 * 初始化
 * ============================================================ */
int mech_cfg_store_init(void)
{
    int rc;

    memset(&g_mech, 0, sizeof(g_mech));

    /* 先建默认表：无论加载成功与否，内存里都有一份可用配置 */
    mech_table_default(&g_mech.tbl);

    rc = mech_table_load(&g_mech.tbl);
    if (rc == MECH_OK) {
        g_mech.src_is_flash = 1U;
        g_mech.cfg_fault    = 0U;
        rt_kprintf("[MECH] loaded from flash, count=%u active=%u\n",
                   (unsigned)g_mech.tbl.count, (unsigned)g_mech.tbl.active);
    } else {
        /* 加载或校验失败：回默认 + 置故障标志，不猜、不带病运行 */
        mech_table_default(&g_mech.tbl);
        g_mech.src_is_flash = 0U;
        g_mech.cfg_fault    = 1U;
        rt_kprintf("[MECH] load failed (%d), fallback to DEFAULT, cfg_fault=1\n", rc);
    }

    /* 派生 active 行 */
    rc = mech_apply_active();
    if (rc != MECH_OK) {
        rt_kprintf("[MECH] active row derive failed, use row0 default\n");
        mech_cfg_default(&g_mech.cfg);
        g_mech.tbl.active = 0U;
        g_mech.active_idx = 0;
        (void)mech_cfg_derive(&g_mech.cfg, &g_mech.rt, RT_NULL, 0);
        g_mech.cfg_fault = 1U;
    }

    g_mech.is_loaded = 1U;

    LOG_OK("[MECH] init done: z01=%u:%u z02=%u:%u i=%u/%u*%u/%u pp=%u\n",
               (unsigned)g_mech.cfg.enc_z01_num, (unsigned)g_mech.cfg.enc_z01_den,
               (unsigned)g_mech.cfg.enc_z02_num, (unsigned)g_mech.cfg.enc_z02_den,
               (unsigned)g_mech.cfg.i_in_num, (unsigned)g_mech.cfg.i_in_den,
               (unsigned)g_mech.cfg.i_out_num, (unsigned)g_mech.cfg.i_out_den,
               (unsigned)g_mech.cfg.pole_pairs);

    /* ============================================================
     * 【2026-11-04】按 sys_config 的三个选择子上电选行
     * ------------------------------------------------------------
     * 必须在 sys_config_init 之后（启动表第 4 步 vs 本步第 5 步，天然满足），
     * 也必须在 mech_apply_active 之后 —— 这样"当前生效行"已就绪，
     * mech_apply_cfg_selection() 才能正确判断"是否需要切换"。
     *
     * 失败不阻断启动：选行失败时已回退到出厂默认组合，
     * 系统仍能以一套已知可用的机械参数运行。
     * ============================================================ */
    (void)mech_apply_cfg_selection();

    /* 重新打印一次：选行可能改动了 cfg，此处刷出最终生效值 */
    LOG_OK("[MECH] final: z01=%u:%u z02=%u:%u i=%u/%u*%u/%u i_total=%d.%02d\n",
               (unsigned)g_mech.cfg.enc_z01_num, (unsigned)g_mech.cfg.enc_z01_den,
               (unsigned)g_mech.cfg.enc_z02_num, (unsigned)g_mech.cfg.enc_z02_den,
               (unsigned)g_mech.cfg.i_in_num, (unsigned)g_mech.cfg.i_in_den,
               (unsigned)g_mech.cfg.i_out_num, (unsigned)g_mech.cfg.i_out_den,
               (int)(g_mech.rt.i_total_x100 / 100),
               (int)(g_mech.rt.i_total_x100 % 100));

    return MECH_OK;
}

/* ============================================================
 * 表打印
 * ============================================================ */
void mech_table_dump(const mech_table_t *tbl)
{
    uint8_t i;
    char    err[48];

    SM_ASSERT_PTR(tbl);

    rt_kprintf("[MECH] table ver=0x%04X count=%u active=%u row_size=%u "
               "sizeof=%uB blk=%uB\n",
               (unsigned)tbl->version, (unsigned)tbl->count,
               (unsigned)tbl->active, (unsigned)tbl->row_size,
               (unsigned)sizeof(mech_table_t),
               (unsigned)(sizeof(mech_table_t) + 8U));
    rt_kprintf("[MECH] idx tag  z01    z02    i_in     i_out    chk  note\n");

    for (i = 0U; i < tbl->count && i < MECH_PROFILE_MAX; i++) {
        const mech_cfg_t *c = &tbl->row[i];
        mech_rt_t tmp_rt;
        int ok;

        ok = mech_cfg_derive(c, &tmp_rt, err, sizeof(err));

        rt_kprintf("[MECH]  %u   %u   %u:%-2u  %u:%-2u  %u/%-3u  %u/%-3u  %s  %s\n",
                   (unsigned)i, (unsigned)c->teeth_tag,
                   (unsigned)c->enc_z01_num, (unsigned)c->enc_z01_den,
                   (unsigned)c->enc_z02_num, (unsigned)c->enc_z02_den,
                   (unsigned)c->i_in_num, (unsigned)c->i_in_den,
                   (unsigned)c->i_out_num, (unsigned)c->i_out_den,
                   (ok == MECH_OK) ? "OK  " : "FAIL",
                   (i == tbl->active) ? "*active" : "");
        if (ok != MECH_OK) {
            rt_kprintf("[MECH]       -> %s\n", err);
        }
    }
}