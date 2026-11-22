/**
 * @file    mech_cfg.h
 * @brief   机械参数配置表（编码器齿数比 / 两级减速比 / 极对数 / 行程）
 * @note    PLAN-ABS-ANGLE-V1.3 §5.1
 *
 * ============================================================
 * 设计约束（改动前必读）
 * ============================================================
 * 1. 本模块**禁止出现 float / double**。理由见 §3.6：
 *    float 只能精确表示 ≤ 2^24 = 16,777,216 的整数，当前 cnt 跨度
 *    3,870,720 尚有 4.33x 余量，但临界 i_total = 4681.1，
 *    配置表开放后减速比一调大就静默丢整数位且不报错。
 *    全部换算用 int32 约分系数，误差 ≤ 0.5 LSB。
 *
 * 2. 落盘结构（mech_cfg_t）与运行时派生结构（mech_rt_t）**必须分离**。
 *    派生量落盘后，一旦公式修正就会出现"Flash 里是旧公式算的、
 *    代码是新公式"的静默错误（C-05）。
 *
 * 3. "齿"有三套含义，禁止混淆（C-06）：
 *    - teeth_type  : 机型齿型（11/13/14齿），仅上报，不参与运算
 *    - enc_z0x_*   : 编码器齿轮组齿数比，决定游标解算
 *    - teeth_tag   : 本 profile 行适用的 teeth_type，仅用于查表
 *
 * 4. sizeof(mech_cfg_t) = 32 B（含尾部 crc32）。单块上限
 *    MECH_PROFILE_MAX = 7，因 7*32+8 = 232 <= 256 而 8*32+8 = 264 > 256
 *    会触发 flash_drv.c:102 的 blk_size <= 256 断言（C-07）。
 * ============================================================
 */

#ifndef __MECH_CFG_H
#define __MECH_CFG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 1. 常量
 * ============================================================ */

/* 单块最多行数：7*32 + 8(表头) + 4(表CRC) = 236 B，blk = 244 B <= 256
 * 8 行会得到 264 B 块，撞 flash_drv.c:102 断言，上电即死机。见 C-07 */
#define MECH_PROFILE_MAX        7

/* 表结构版本（结构体布局变更时递增，老表判 T3 后回默认） */
#define MECH_TABLE_VERSION      0x0100U

/* mech_cfg_t 结构版本 */
#define MECH_CFG_VERSION        0x0100U

/* 校验阈值（见 §5.3.1） */
#define MECH_TRAVEL_MARGIN_X100 115      /* V3：联合周期需 > 行程圈数 * 1.15 */
#define MECH_CNT_SPAN_MAX       2000000000L  /* V4：cnt 跨度 <= 2.0e9 */
/* V5：约分系数位宽 —— 【Q-47 方案 1「放宽判据到 uint32 能力」，2026-10-27 用户批准】
 *
 * 原判据 `i_total/gcd(225,i_total) <= 255` 是照抄 uint8 时代的魔数，
 * 与现在的字段类型不匹配，会拒绝合法配置（见实测）。
 *
 * ---- 判据的真实数学含义（此前的注释没有写清，容易误改）----
 *   3600 = 16·225，4096 = 16·256，且 gcd(256,225)=1
 *   => gcd(3600, 4096·i) = 16·gcd(225, i)
 *   => out_den = 4096·i / gcd(3600,4096·i) = 256 · [i / gcd(225,i)]
 *                                        = 256 · i_coef_den
 *   所以 V5 判 i_coef_den，本质上就是判 out_den 会不会溢出 uint32。
 *   这正是 V5 的设计意图（文档 §5.3.1："out_den 溢出 uint32 /
 *   Δcnt×num 爆 int32"），只是当年用 uint8 眼光的 255 来表达，错了 5 个数量级。
 *
 * ---- 实测（全量扫描 i = 1..200000，见本轮分析）----
 *   out_num    = 3600/g   <= 225     (g>=16)   恒 <= UINT16_MAX  ✓
 *   out_md_num = 360000/g <= 11250   (g>=32)   恒 <= UINT16_MAX  ✓
 *   out_den    = out_md_den 最大 12,799,936 (i=199999)，UINT32_MAX 余量 335×  ✓
 *   旧判据 255 会拒绝 i_total=1081/5003 等大量合法值（out_den 仅 27 万/128 万，
 *   远未溢出），属"算了个不用的量再拒绝合法配置"。
 *
 * ---- 取值 ----
 *   UINT32_MAX / 256 = 16,777,215  ⟸ out_den 恰好溢出 uint32 的临界点。
 *   （提出方案时曾写 1048575 = UINT32_MAX/4096，那是漏算了 3600 与 4096
 *     之间还能约掉的 16 倍，偏严格 16×；两者都能让 136.6×5 的 683 通过，
 *     此处用精确值，才符合「按 uint32 实际能力算」的原意。）
 *   在 V8 的 i_total <= 200000 约束下 i_coef_den <= 200000，远低于此值，
 *   故 V5 退化为纯防御性检查（防止将来放宽 V8 时静默溢出）。
 *   保留它而不删除，是因为它的失效后果（静默位宽截断 → 角度错）太难排查。 */
#define MECH_ICOEF_MAX          16777215UL   /* V5：= UINT32_MAX/256，即 out_den 不溢出 uint32 */
#define MECH_RES_MAX_NDEG       10000000L  /* V6：分辨率 <= 0.01 deg/LSB = 1e7 ndeg/LSB */
#define MECH_ITOTAL_MAX         200000L  /* V8：i_total 上界，防填离谱值 */
#define MECH_TRAVEL_SPAN_MAX_X10 4000L   /* V7：行程跨度 <= 400.0 deg */

/* 默认值（Q-28 / 用户 2026-10-26 裁定；齿数 2026-11-03 实测更正）
 * 注：这些宏只服务 mech_cfg_default() 的**兜底路径**（候选表异常时）。
 *     正常路径全部走 g_mech_cand[]，不要再往这里加参数，否则又变两处定义。
 *
 * 【2026-11-03 齿数更正 41:42:43 → 55:56:57】
 * 依据：t_enc_tri 以 1ms 间隔采样 4 次（2000 帧），实测 travel ratio
 *       Z1/Z0 = 0.9820~0.9823、Z2/Z0 = 0.9648~0.9653，重复性 ±0.03%。
 *       穷举 3~300 齿全部组合反查，仅 55k:56k:57k 一族吻合（误差 0.03%），
 *       41:42:43 偏差 +0.61% / +1.21%，被排除。
 *       必须在 1ms 下采样：手转 80~226 LSB/ms，间隔 ≥10ms 即超半圈混叠，
 *       比值失真（10ms 组曾出现 Z1/Z0=1.018 这种不可能值）。
 * ⚠️ 兜底值必须与 MECH_CAND_DEFAULT 保持同一套齿数，
 *    否则候选表校验失败回退时会静默退回错误齿比。 */
#define MECH_DEF_Z0             55
#define MECH_DEF_Z1             56
#define MECH_DEF_Z2             57
#define MECH_DEF_I_IN           216
#define MECH_DEF_I_IN_DEN       1
/* 【2026-11-04 外部减速比语义更正 5/1 → 65/13】
 *
 * 原值 5/1 是"13 齿机型齿比 13:65 的约分值"，13:65 = 5 恰好整除，
 * 所以对 13 齿机型一直精确；但换成 14 齿(75:14) 或 11 齿(53:11)
 * 时 5/1 就不再成立，会带入 0.03%~0.25% 的系统角度偏差。
 *
 * 现改为记录**真实齿比分子/分母**，由 i_total = i_in × (i_out_num/i_out_den)
 * 自然覆盖三种机型：
 *   13 齿  65/13 = 5         → 216×5      = 1080     （与原 5/1 完全等价）
 *   14 齿  75/14 ≈ 5.3571    → 216×75/14  = 1157.14
 *   11 齿  53/11 ≈ 4.8182    → 216×53/11  = 1040.73
 * 13 齿数值一位不差，14/11 齿从此正确。 */
#define MECH_DEF_I_OUT          65
#define MECH_DEF_I_OUT_DEN      13
#define MECH_DEF_POLE_PAIRS     4        /* board_motor_cfg.h:104：12槽4对极 */
#define MECH_DEF_OUT_MIN_X10    (-1000)  /* -100.0 deg */
#define MECH_DEF_OUT_MAX_X10    (2150)   /* +215.0 deg */
#define MECH_DEV_MARGIN_X10     20       /* ±2.0 deg 物理容差（Q-34） */

/* ============================================================
 * 出厂候选表 —— 【Q-49「按需配置，初始化时选择」，2026-10-27 裁定】
 *            【2026-11-04 扩为 3 维 12 组合，用户裁定"一并纳入"】
 *
 * 机械传动链有三段**互相独立**的减速环节，必须分别配置：
 *
 *   ① 编码器齿轮组齿数比 enc_z0x   —— 决定游标解算，实测两套
 *        41:42:43    (2026-11-03 实测排除，保留备选)
 *        55:56:57    (2026-11-03 实测确认，出厂默认)
 *
 *   ② 电机内部减速比 i_in          —— 电机本体减速箱
 *        136.6  (= 683/5，禁止浮点)
 *        216
 *
 *   ③ 电机外部减速比 i_out         —— 机型齿比（卸粮筒/搅龙与电机之间）
 *        65/13  (13 齿机型，= 5)
 *        75/14  (14 齿机型，≈5.3571)   ← 用户 2026-11-04 更正，原记 74/14 为笔误
 *        53/11  (11 齿机型，≈4.8182)
 *
 * 三者正交 ⇒ 2 × 2 × 3 = 12 组合。全部已核算通过 V1~V8：
 *
 *   组合                     i_total    joint  V3 need  V4跨度
 *   enc       i_in  i_out
 *   41:42:43  136.6 65/13    683.0000   1806    688   2,478,956
 *   41:42:43  136.6 75/14    731.7857   1806    737   2,656,004
 *   41:42:43  136.6 53/11    658.1636   1806    663   2,388,799
 *   41:42:43  216   65/13   1080.0000   1806   1087   3,919,872
 *   41:42:43  216   75/14   1157.1429   1806   1165   4,199,852
 *   41:42:43  216   53/11   1040.7273   1806   1048   3,777,305
 *   55:56:57  136.6 65/13    683.0000   3192    688   2,478,956
 *   55:56:57  136.6 75/14    731.7857   3192    737   2,656,004
 *   55:56:57  136.6 53/11    658.1636   3192    663   2,388,799
 *   55:56:57  216   65/13   1080.0000   3192   1087   3,919,872
 *   55:56:57  216   75/14   1157.1429   3192   1165   4,199,852
 *   55:56:57  216   53/11   1040.7273   3192   1048   3,777,305
 *
 *   ⚠️ 41:42:43 在 joint=1806 下 V3 余量仅 1.66x（216 组合 1087 需求），
 *      是本表最紧的组合，但仍在 1.15x 硬门限之上，合法。
 *
 * ------------------------------------------------------------
 * 【id 编号规则 —— 一旦分配不可变（会随 tbl 落盘）】
 * ------------------------------------------------------------
 *   id = i_out_sel * 4 + i_in_sel * 2 + enc_sel
 *
 *   enc_sel : 0 = 41:42:43,  1 = 55:56:57
 *   i_in_sel: 0 = 136.6,     1 = 216
 *   i_out_sel: 0 = 65/13,    1 = 75/14,    2 = 53/11
 *
 *   ⇒ id 0~3 与 2026-10-27 那版 4 条的语义**完全一致**
 *     （当时只有 65/13 一种外部齿比），故老现场 id 不需迁移。
 *
 * ------------------------------------------------------------
 * 【为什么 teeth_tag 一律写 0】
 * ------------------------------------------------------------
 *   mech_cfg_t.teeth_tag 的语义是"本行适用的机型齿型"（TEETH_11/13/14），
 *   用于 mech_find_profile() 查行。但本工程"按需配置"只落**一行**
 *   (count=1)，若给该行填了非 0 的 teeth_tag，表级校验 T6
 *   （必须存在兜底行 teeth_tag==0）会失败，整表被判非法。
 *   机型信息归属 sys_config 的 mech_i_out 项，本表只记真实齿比，
 *   不重复表达"这台是几齿机型"—— 避免 C-06 那类"齿有三套含义"的错位。
 * ============================================================ */
#define MECH_CAND_COUNT         12U

/* --- 选择子（对应 sys_config 的 0x0A01/0x0A02/0x0A03 三个配置项）--- */
#define MECH_ENC_SEL_414243     0U
#define MECH_ENC_SEL_555657     1U
#define MECH_ENC_SEL_COUNT      2U

#define MECH_IIN_SEL_1366       0U
#define MECH_IIN_SEL_216        1U
#define MECH_IIN_SEL_COUNT      2U

#define MECH_IOUT_SEL_65_13     0U   /* 13 齿机型 */
#define MECH_IOUT_SEL_75_14     1U   /* 14 齿机型（2026-11-04 更正，原 74/14 笔误）*/
#define MECH_IOUT_SEL_53_11     2U   /* 11 齿机型 */
#define MECH_IOUT_SEL_COUNT     3U

/* id = i_out_sel*4 + i_in_sel*2 + enc_sel（公式见上方注释，顺序不可变）*/
#define MECH_CAND_ID(enc, iin, iout) \
    ((uint8_t)((uint8_t)(iout) * 4U + (uint8_t)(iin) * 2U + (uint8_t)(enc)))

/* 兼容旧名：2026-10-27 版 4 条的别名，等价于 i_out_sel = 65/13 */
#define MECH_CAND_A2_216        MECH_CAND_ID(MECH_ENC_SEL_414243, MECH_IIN_SEL_216,  MECH_IOUT_SEL_65_13) /* 1 */
#define MECH_CAND_B2_216        MECH_CAND_ID(MECH_ENC_SEL_555657, MECH_IIN_SEL_216,  MECH_IOUT_SEL_65_13) /* 3 */
#define MECH_CAND_A1_1366       MECH_CAND_ID(MECH_ENC_SEL_414243, MECH_IIN_SEL_1366, MECH_IOUT_SEL_65_13) /* 0 */
#define MECH_CAND_B1_1366       MECH_CAND_ID(MECH_ENC_SEL_555657, MECH_IIN_SEL_1366, MECH_IOUT_SEL_65_13) /* 2 */

/* 【2026-11-03 默认项 A2 → B2 依据见上方 MECH_DEF_Z0 注释块】
 * 【2026-11-04 确认】默认组合 = 55:56:57 + 216 + 65/13 = id 3
 *   = 现状现场实测通过的组合（原 i_out 写作 5/1，数值完全等价）。 */
#define MECH_CAND_DEFAULT       MECH_CAND_B2_216

/* 错误码 */
#define MECH_OK                 0
#define MECH_ERR_NOT_INIT      (-1)
#define MECH_ERR_PARAM         (-2)
#define MECH_ERR_IO            (-3)
#define MECH_ERR_VALIDATE      (-4)

/* ============================================================
 * 2. 落盘结构：只放真·可配参数，严禁放派生量
 * ============================================================ */
typedef struct {
    uint16_t version;           /* MECH_CFG_VERSION，不符则整行回默认 */

    /* 编码器齿数比（Z0 主动 / Zi 从动），C-06 强制 enc_ 前缀 */
    uint16_t enc_z01_num;       /* 默认 41 */
    uint16_t enc_z01_den;       /* 默认 42 */
    uint16_t enc_z02_num;       /* 默认 41 */
    uint16_t enc_z02_den;       /* 默认 43 */

    /* 两级减速比（分子/分母） */
    uint16_t i_in_num;          /* 默认 216 */
    uint16_t i_in_den;          /* 默认 1   */
    uint16_t i_out_num;         /* 默认 5   */
    uint16_t i_out_den;         /* 默认 1   */

    uint8_t  pole_pairs;        /* 电机极对数，默认 4 */
    uint8_t  teeth_tag;         /* 本行适用的 teeth_type；0 = 通用/兜底行 */

    /* 卸粮筒行程（0.1 deg），不含 ±2 deg 容差，容差在派生时加 */
    int16_t  out_min_x10;       /* 默认 -1000 */
    int16_t  out_max_x10;       /* 默认 +2150 */

    /* 调试用容差覆盖（0.001 deg）。0 = 用按齿数自动推导值（C-04） */
    int32_t  tol_override_x1000;

    uint32_t crc32;             /* 行内 CRC，仅单行导入/导出时校验 */
} mech_cfg_t;                   /* sizeof = 32 B */

/* ============================================================
 * 3. 落盘表：表头 + 定长行数组
 * ============================================================ */
typedef struct {
    uint16_t   version;         /* MECH_TABLE_VERSION */
    uint8_t    count;           /* 有效行数 1~MECH_PROFILE_MAX */
    uint8_t    active;          /* 当前生效行索引，须 < count */
    uint16_t   row_size;        /* = sizeof(mech_cfg_t)，防结构体变更后错读（T3） */
    uint16_t   reserved;
    mech_cfg_t row[MECH_PROFILE_MAX];
    uint32_t   crc32;           /* 覆盖表头 + 全部 row */
} mech_table_t;                 /* sizeof = 236 B，blk = 244 B */

/* ============================================================
 * 4. 运行时派生结构（RAM only，不落盘）
 * ============================================================ */
typedef struct {
    /* --- 由齿数推导 --- */
    uint16_t z01_p_num;         /* 约分后 Z0:Z1 */
    uint16_t z01_p_den;
    uint16_t z02_p_num;         /* 约分后 Z0:Z2 */
    uint16_t z02_p_den;

    int32_t  d1_x1000;          /* 游标间距 Δ1，0.001 deg */
    int32_t  d2_x1000;          /* 游标间距 Δ2，0.001 deg */
    int32_t  tol_x1000;         /* 游标容差（自适应），0.001 deg */
    int32_t  verify_tol_x1000;  /* 交叉校验容差（更松） */
    int32_t  tol_limit_x1000;   /* 容差上限 min(|Δ1|,|Δ2|)/2，仅显示用 */

    /* --- 由减速比推导 --- */
    int32_t  i_total_x100;      /* 总减速比 *100（1080 -> 108000） */
    uint32_t joint_period;      /* lcm(约分分母)，V3 判据 */

    /* --- 行程折算 --- */
    int32_t  cnt_min;           /* Z0 累计计数下限（含 ±2 deg 容差） */
    int32_t  cnt_max;
    int32_t  k_min;             /* Z0 绝对圈数范围 */
    int32_t  k_max;
    int32_t  k_count;           /* k_max - k_min + 1 */

    int32_t  out_min_x10;       /* 行程（含容差），0.1 deg */
    int32_t  out_max_x10;
    int32_t  travel_turns_x100; /* 行程折算圈数 *100 */

    /* --- 换算系数（约分后，int32 内完成）---
     *
     * 【2026-11-04 Q2 修复：分子由 uint16 拓宽到 uint32】
     *
     * 原设计 base 在 i_total 的**整数值**上算系数：
     *   den = 4096 * i_total_int          （mech_cfg.c 旧 :236）
     * 而 i_total_int 是 i_total_x100 又被 /100 截断一次的结果。
     * 14 齿 75/14、11 齿 53/11 的 i_total 不是整数（1157.1429 / 1040.7273），
     * 截断后系数带固定比例偏差：
     *   75/14 + 216 → +0.0290°，75/14 + 136.6 → +0.2526°（@235°，含容差）
     * 这不是随机误差，是**固定偏置**，标定零点时会被吸收，反而更难发现。
     *
     * 改为直接用 i_total_x100（保留两位小数）推导，误差降到 ≤0.0018°：
     *   out_mdeg = dcnt * 360000*100 / (4096 * i_total_x100)
     * 代价是分子变大：最大 140,625 > 65535，必须 uint32。
     *   ⇒ out_num / out_md_num 由 uint16 升为 uint32。
     *
     * 为什么这不会溢出 int64 中间量：
     *   cnt 最大约 4.4e6（含容差 235° × i_total 1157）
     *   4.4e6 × 140,625 ≈ 6.2e11 < 9.2e18 ✓
     *   abs_angle.c:405 已是 int64 乘法，无需改动。
     *
     * 注：mech_rt_t 是 RAM only（mech_cfg.h:15-17 约束2），不落盘，
     *     改字段宽度不影响 Flash 兼容性。 */
    uint32_t out_num;           /* out_x10 = div_round(dcnt * out_num, out_den) */
    uint32_t out_den;
    uint32_t out_md_num;        /* out_mdeg = div_round(dcnt * out_md_num, out_md_den) */
    uint32_t out_md_den;
    uint16_t e_num;             /* theta_e_mdeg = div_round(raw * e_num, e_den) */
    uint16_t e_den;
    uint16_t e_shift;           /* 极对数 > 8 时先右移再乘，防溢出 */

    uint32_t res_ndeg_per_lsb;  /* 分辨率，1e-9 deg/LSB，仅显示与 V6 用 */
} mech_rt_t;

/* ============================================================
 * 5. 定点工具（头文件内联，禁止浮点）
 * ============================================================ */

/**
 * @brief 对称四舍五入除法（全整数）
 * @note  C 的 '/' 向零截断，直接加 0.5 对负数会偏向零。
 *        本实现保证 div_round(-n,d) == -div_round(n,d)。
 * @param  num 被除数
 * @param  den 除数，必须 > 0
 * @return 四舍五入结果
 */
static inline int32_t mech_div_round(int32_t num, int32_t den)
{
    if (den <= 0) return 0;
    if (num >= 0) {
        return (num + den / 2) / den;
    }
    return -((-num + den / 2) / den);
}

/**
 * @brief int64 版本的对称四舍五入（用于中间量可能超 int32 的场合）
 */
static inline int32_t mech_div_round64(int64_t num, int64_t den)
{
    if (den <= 0) return 0;
    if (num >= 0) {
        return (int32_t)((num + den / 2) / den);
    }
    return (int32_t)(-((-num + den / 2) / den));
}

/**
 * @brief 最大公约数（欧几里得，全整数）
 */
static inline uint32_t mech_gcd(uint32_t a, uint32_t b)
{
    while (b != 0U) {
        uint32_t t = a % b;
        a = b;
        b = t;
    }
    return a;
}

/**
 * @brief 最小公倍数（已做溢出保护：先除后乘）
 */
static inline uint32_t mech_lcm(uint32_t a, uint32_t b)
{
    uint32_t g;
    if (a == 0U || b == 0U) return 0U;
    g = mech_gcd(a, b);
    /* a/g * b 可能超 uint32，此处按业务量级（<10000）安全 */
    return (a / g) * b;
}

/**
 * @brief 归一化到 [-180000, +180000] 的 0.001 deg 角（用于增量跟踪）
 * @param mdeg 输入角度，0.001 deg，任意范围
 * @return 归一化结果
 */
static inline int32_t mech_norm_pm180_x1000(int32_t mdeg)
{
    while (mdeg >  180000) mdeg -= 360000;
    while (mdeg < -180000) mdeg += 360000;
    return mdeg;
}

/* ============================================================
 * 6. API
 * ============================================================ */

/**
 * @brief 填默认值到一整行（41/42、41/43、216/1、5/1、4极对、-100~+215）
 */
void mech_cfg_default(mech_cfg_t *cfg);

/**
 * @brief 构建默认整表（count=1，仅兜底行 teeth_tag=0，内容取 MECH_CAND_DEFAULT）
 */
void mech_table_default(mech_table_t *tbl);

/* ============================================================
 * 出厂候选表 API —— 【Q-49「按需配置，初始化时选择」】
 * ============================================================ */

/**
 * @brief 取一条出厂候选的参数（不改任何全局状态，可直接用于校验/打印）
 * @param id  候选 id，MECH_CAND_ID(enc, iin, iout)；越界返回 -1
 * @param out 输出缓冲区
 * @return MECH_OK / MECH_ERR_PARAM
 */
int mech_cand_get(uint8_t id, mech_cfg_t *out);

/**
 * @brief 候选的短名（MSH 列表用），越界返回 "?"
 */
const char *mech_cand_name(uint8_t id);

/**
 * @brief 由三个选择子合成候选 id
 * @param enc_sel    MECH_ENC_SEL_xxx
 * @param iin_sel    MECH_IIN_SEL_xxx
 * @param iout_sel   MECH_IOUT_SEL_xxx
 * @return 候选 id（0~MECH_CAND_COUNT-1）；任一选择子越界返回 0xFF
 * @note   等价于宏 MECH_CAND_ID()，此函数版多做越界检查，
 *         供 MSH / 上电初始化这类"值来自外部"的场合使用。
 */
uint8_t mech_cand_make_id(uint8_t enc_sel, uint8_t iin_sel, uint8_t iout_sel);

/**
 * @brief 把候选 id 拆回三个选择子
 * @param id        候选 id
 * @param enc_sel   输出，可为 NULL
 * @param iin_sel   输出，可为 NULL
 * @param iout_sel  输出，可为 NULL
 * @return MECH_OK / MECH_ERR_PARAM
 */
int mech_cand_split_id(uint8_t id, uint8_t *enc_sel,
                       uint8_t *iin_sel, uint8_t *iout_sel);

/**
 * @brief 取某个选择子的可读名（MSH 打印用）
 */
const char *mech_enc_sel_name(uint8_t sel);
const char *mech_iin_sel_name(uint8_t sel);
const char *mech_iout_sel_name(uint8_t sel);

/**
 * @brief 把一条候选填成 tbl.row[0]（兜底行），并把 active 归 0
 * @note  只改内存，不落盘、不动校准。调用方负责 save 与褪零。
 *        这样拆开是为了让初始化流程与 MSH 命令复用同一段逻辑，
 *        且"C-08 褪零"只在真正切行时发生一次。
 * @return MECH_OK / MECH_ERR_PARAM（候选非法不会写坏 tbl）
 */
int mech_table_apply_cand(mech_table_t *tbl, uint8_t id);

/**
 * @brief 逐个候选跑 V1~V8，打印结论（t_mech_cand 用）
 * @note  这是"出厂自检"：即使现场没用到某套参数，也要确认它没被改坏。
 */
void mech_cand_selftest(void);

/**
 * @brief 单行校验 V1~V8（只校验，不改状态）
 * @param cfg  待校验行
 * @param err  错误信息输出缓冲，可为 NULL
 * @param err_len 缓冲长度
 * @return MECH_OK 或 MECH_ERR_VALIDATE
 */
int mech_cfg_validate(const mech_cfg_t *cfg, char *err, uint16_t err_len);

/**
 * @brief 表级校验 T1~T6（含逐行 V1~V8）
 * @return MECH_OK 或 MECH_ERR_VALIDATE
 */
int mech_table_validate(const mech_table_t *tbl, char *err, uint16_t err_len);

/**
 * @brief 校验 + 计算全部派生量
 * @note  调用前会先跑 mech_cfg_validate，失败则 bool 输出不修改。
 * @return MECH_OK / MECH_ERR_PARAM / MECH_ERR_VALIDATE
 */
int mech_cfg_derive(const mech_cfg_t *cfg, mech_rt_t *rt,
                    char *err, uint16_t err_len);

/**
 * @brief 按 teeth_type 查行索引（先精确匹配 teeth_tag，再回落 teeth_tag==0）
 * @return 行索引，无可用行返回 -1
 */
int mech_find_profile(const mech_table_t *tbl, uint8_t teeth_type);

/**
 * @brief 打印一行配置与派生量（t_mech 用）
 */
void mech_cfg_dump(const mech_cfg_t *cfg, const mech_rt_t *rt,
                   const mech_table_t *tbl);

#ifdef __cplusplus
}
#endif

#endif /* __MECH_CFG_H */