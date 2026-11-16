/**
 * @file    mech_cfg.c
 * @brief   机械参数配置表实现（校验 V1~V8 / T1~T6 + 派生量计算）
 * @note    PLAN-ABS-ANGLE-V1.3 §5.1、§5.3.1
 *
 * ============================================================
 * 实现要点
 * ============================================================
 * 1. 校验(V)与派生(derive)共用同一个内部计算函数 mech_calc()，
 *    保证"校验通过的判据"与"实际用的系数"绝对一致。
 *    若分两处实现，必然出现 V3 用裸分母、derive 用约分分母这类错位。
 *
 * 2. 全程 int32/int64 定点，无 float/double（§3.6）。
 *    唯一用 int64 的地方是 cnt*num 与 res_ndeg 的中间量。
 *
 * 3. 所有除法的除数在使用前都校验 > 0（V1 的实质目的）。
 * ============================================================
 */

#include "mech_cfg.h"
#include "framework/state_engine/sm_assert.h"

#include <string.h>
#include <rtthread.h>

/* 无符号除法辅助：避免混用有/无符号比较告警 */
#define MECH_DIV(a, b)      ((uint32_t)((a) / (b)))

/* ============================================================
 * int64 版最大公约数
 * ------------------------------------------------------------
 * 【为什么不能复用 mech_cfg.h 的 mech_gcd()】
 *   那个是 uint32 版。Q2 修复后 4096*i_total_x100 可达 8.2e10，
 *   早已超出 uint32，必须用 int64 才能正确约分。
 *   写成 int64 版本而不是"先除后乘"的技巧，是因为这里两个数
 *   都带 4096 因子之外的公共因子（如 75/14 组合的 gcd 很大），
 *   分开处理反而容易算错。
 * ============================================================ */
static int64_t mech_gcd64(int64_t a, int64_t b)
{
    if (a < 0) { a = -a; }
    if (b < 0) { b = -b; }
    while (b != 0) {
        int64_t t = a % b;
        a = b;
        b = t;
    }
    return (a == 0) ? 1 : a;   /* 返回 1 而非 0，避免调用方除零 */
}

/* ============================================================
 * 内部：错误信息写入
 * ============================================================ */
static void set_err(char *err, uint16_t err_len, const char *fmt, int32_t a, int32_t b, int32_t c)
{
    if (err == RT_NULL || err_len == 0U) {
        return;
    }
    /* 简易 %d 格式化，避免引入 printf 浮点支持 */
    {
        const char *p = fmt;
        uint16_t   i = 0U;
        int32_t    vals[3];
        uint8_t    vi = 0U;

        vals[0] = a;
        vals[1] = b;
        vals[2] = c;

        while (*p != '\0' && i < (uint16_t)(err_len - 1U)) {
            if (*p == '%' && *(p + 1) == 'd') {
                /* 手写整数转字符串 */
                char     buf[12];
                int8_t   n = 0;
                int32_t  v;
                uint8_t  neg = 0U;

                if (vi >= 3U) {
                    break;
                }
                v = vals[vi];
                vi++;

                if (v < 0) {
                    neg = 1U;
                    v = -v;
                }
                if (v == 0) {
                    buf[n++] = '0';
                } else {
                    while (v > 0 && n < (int8_t)sizeof(buf)) {
                        buf[n++] = (char)('0' + (v % 10));
                        v /= 10;
                    }
                }
                if (neg != 0U && i < (uint16_t)(err_len - 1U)) {
                    err[i++] = '-';
                }
                while (n > 0 && i < (uint16_t)(err_len - 1U)) {
                    err[i++] = buf[--n];
                }
                p += 2;
                continue;
            }
            err[i++] = *p++;
        }
        err[i] = '\0';
    }
}

/* ============================================================
 * 内部：由一行配置算出全部派生量
 * ------------------------------------------------------------
 * 输出 out 与 r 都是可选（NULL 表示只要校验结论）。
 * 返回 0 表示全部通过；否则返回失败的校验编号（1~8）负数，
 * 例如 -3 表示 V3 失败。调用方据此组合错误信息。
 * ============================================================ */
static int mech_calc(const mech_cfg_t *cfg, mech_rt_t *r)
{
    mech_rt_t tmp;

    /* 派生中间量（全整数） */
    uint32_t g1, g2;
    uint32_t p1_den, p2_den;          /* 约分后分母 */
    int64_t  i_total_x100;            /* 总减速比 *100 */
    uint32_t i_total_int;
    uint32_t i_coef_den;              /* i_total / gcd(225, i_total) */
    uint32_t joint;
    int64_t  cnt_min64, cnt_max64;
    /* 【2026-11-04 Q2】系数改用 int64 中间量：4096*i_total_x100 可超 uint32 */
    int64_t  n64, d64, g64;

    if (cfg == RT_NULL) {
        return -1;
    }

    /* ---------- V1：零值 ---------- */
    if (cfg->enc_z01_num == 0U || cfg->enc_z01_den == 0U ||
        cfg->enc_z02_num == 0U || cfg->enc_z02_den == 0U) {
        return -1;
    }
    if (cfg->i_in_num == 0U || cfg->i_in_den == 0U ||
        cfg->i_out_num == 0U || cfg->i_out_den == 0U) {
        return -1;
    }
    if (cfg->pole_pairs == 0U) {
        return -1;
    }

    /* ---------- V2：游标有效（齿数不能相等，否则 Δ=0 无游标效应）---------- */
    if (cfg->enc_z01_num == cfg->enc_z01_den) {
        return -2;
    }
    if (cfg->enc_z02_num == cfg->enc_z02_den) {
        return -2;
    }

    /* ---------- 约分 ---------- */
    g1 = mech_gcd(cfg->enc_z01_num, cfg->enc_z01_den);
    g2 = mech_gcd(cfg->enc_z02_num, cfg->enc_z02_den);
    tmp.z01_p_num = (uint16_t)(cfg->enc_z01_num / g1);
    tmp.z01_p_den = (uint16_t)(cfg->enc_z01_den / g1);
    tmp.z02_p_num = (uint16_t)(cfg->enc_z02_num / g2);
    tmp.z02_p_den = (uint16_t)(cfg->enc_z02_den / g2);

    p1_den = tmp.z01_p_den;
    p2_den = tmp.z02_p_den;

    /* ---------- C-01：联合周期必须用约分后分母 ---------- */
    joint = mech_lcm(p1_den, p2_den);
    tmp.joint_period = joint;

    /* ---------- 总减速比（*100 定点，避免浮点）----------
     * i_total_x100 = (i_in_num * i_out_num * 100) / (i_in_den * i_out_den)
     * 先乘后除，用 int64 防中间溢出。 */
    {
        int64_t num64 = (int64_t)cfg->i_in_num * (int64_t)cfg->i_out_num * 100;

        i_total_x100 = num64 / ((int64_t)cfg->i_in_den * (int64_t)cfg->i_out_den);
    }

    /* ---------- V8：i_total 上界（先挡离谱值，否则下面会溢出）---------- */
    if (i_total_x100 <= 0 || i_total_x100 > (int64_t)MECH_ITOTAL_MAX * 100) {
        return -8;
    }
    tmp.i_total_x100 = (int32_t)i_total_x100;
    i_total_int = (uint32_t)(i_total_x100 / 100);

    /* ---------- V3：联合周期 > 行程圈数 * 1.15 ----------
     * 行程圈数 = travel_x10 / 3600 * i_total_x100  （结果 *100 定点）
     * 计数跨度 = travel_x10 / 360000 * i_total_x100 * 4096
     * 注意除数：i_total_x100 已是 *100，故分母是 3600 / 360000 而非 36 / 3600。
     * （此处曾误写 36/3600，导致数字差 10 倍，已修正） */
    {
        int32_t travel_x10;

        /* V7：行程自洽 */
        if (cfg->out_min_x10 >= cfg->out_max_x10) {
            return -7;
        }
        travel_x10 = (int32_t)cfg->out_max_x10 - (int32_t)cfg->out_min_x10;
        if (travel_x10 > (int32_t)MECH_TRAVEL_SPAN_MAX_X10) {
            return -7;
        }

        /* 含 ±2 deg 容差后的实际可达计数边界 */
        tmp.out_min_x10 = (int32_t)cfg->out_min_x10 - MECH_DEV_MARGIN_X10;
        tmp.out_max_x10 = (int32_t)cfg->out_max_x10 + MECH_DEV_MARGIN_X10;

        /* 只需 cnt_min/cnt_max 两个边界：它们已含 ±2 deg 容差，
         * 是全行程的外包络，比单独的 travel_cnt 更严格，故不再单算跨度。 */
        cnt_min64 = ((int64_t)tmp.out_min_x10 * 4096 * (int64_t)tmp.i_total_x100) / 360000;
        cnt_max64 = ((int64_t)tmp.out_max_x10 * 4096 * (int64_t)tmp.i_total_x100) / 360000;

        /* V4：cnt 跨度不爆 int32 */
        if ((cnt_max64 - cnt_min64) > (int64_t)MECH_CNT_SPAN_MAX ||
            cnt_min64 < (int64_t)(-2147483647L) ||
            cnt_max64 > (int64_t)(2147483647L)) {
            return -4;
        }
        tmp.cnt_min = (int32_t)cnt_min64;
        tmp.cnt_max = (int32_t)cnt_max64;

        /* 行程圈数（*100） = travel_x10 * i_total_x100 / 3600 */
        tmp.travel_turns_x100 = (int32_t)(((int64_t)travel_x10 * tmp.i_total_x100) / 3600);
    }

    /* V3 判据：joint > travel_turns * 1.15
     * 两侧同乘 10000 化为整数比较，避免 1.15 的浮点：
     *   joint * 10000 > travel_turns_x100 * 115 */
    if ((int64_t)joint * 10000 <= (int64_t)tmp.travel_turns_x100 * MECH_TRAVEL_MARGIN_X100) {
        return -3;
    }

    /* ---------- k 范围（按 cnt 边界反推，不是按名义行程）----------
     * k = cnt / 4096，向下取整（负数也向下） */
    tmp.k_min = (tmp.cnt_min >= 0) ? (tmp.cnt_min / 4096)
                                   : -(((-tmp.cnt_min) + 4095) / 4096);
    tmp.k_max = (tmp.cnt_max >= 0) ? (tmp.cnt_max / 4096)
                                   : -(((-tmp.cnt_max) + 4095) / 4096);
    tmp.k_count = tmp.k_max - tmp.k_min + 1;

    /* ---------- V5：约分系数位宽（= out_den 是否溢出 uint32）----------
     * 【Q-47 方案 1，2026-10-27】阈值由 255 放宽到 UINT32_MAX/256，
     * 详细推导见 mech_cfg.h 的 MECH_ICOEF_MAX 注释。要点：
     *   out_den = 256 * i_coef_den  ⟹ 判 i_coef_den 就是判 out_den 的位宽。
     * 在 V8（i_total <= 200000）约束下 i_coef_den <= 200000，本判据不会触发；
     * 保留纯为防御——若将来放宽 V8，此处会先挡住静默截断而不是让角度错。
     * 注意 i_coef_den 只作校验中间量，不写入 mech_rt_t。 */
    i_coef_den = i_total_int / mech_gcd(225U, i_total_int);
    if (i_coef_den > (uint32_t)MECH_ICOEF_MAX) {
        return -5;
    }

    /* ---------- 换算系数：out_x10 = div_round(dcnt*num, den) ----------
     *
     * 【2026-11-04 Q2 修复：系数改用 i_total_x100，不再用 i_total_int】
     *
     * 目标：out_mdeg = dcnt / 4096 / i_total * 360000
     *                = dcnt * 360000 / (4096 * i_total)
     *
     * 旧实现用 i_total_int（= i_total_x100/100，向下截断）当分母，
     * 对 i_total 不是整数的组合会带入固定比例偏差：
     *   75/14 + 216   → i_total=1157.1429，用 1157 → +0.0290° @235°
     *   75/14 + 136.6 → i_total= 731.7857，用  731 → +0.2526° @235°
     * 13 齿 65/13（=5）与 136.6×65/13（=683）恰好整除，故一直没暴露。
     *
     * 现直接用 i_total_x100（保留 2 位小数）：
     *   i_total = i_total_x100 / 100
     *   num/den = 360000*100 / (4096 * i_total_x100)
     * 实测 12 组合误差全部 ≤0.0018°，13 齿组合结果一位不差。
     *
     * 分子最大 140,625 > 65535 ⇒ out_num/out_md_num 已拓宽为 uint32
     * （见 mech_cfg.h 的 mech_rt_t 注释）。
     *
     * 溢出核查：45 和 4096*i_total_x100 都远小于 uint32 上限
     * （i_total_x100 <= 200000*100=2e7，×4096 = 8.2e10 已超 uint32！）
     * ⇒ 必须先约分再乘，不能先乘。下面按 num 与 den 各自带 4096 因子
     *    的形式算 gcd 后再除，避免中间量溢出。 */
    /* out_x10 = dcnt * 3600*100 / (4096 * i_total_x100) */
    n64 = 3600LL * 100LL;
    d64 = 4096LL * (int64_t)i_total_x100;
    g64 = mech_gcd64(n64, d64);
    tmp.out_num = (uint32_t)(n64 / g64);
    tmp.out_den = (uint32_t)(d64 / g64);

    /* out_mdeg = dcnt * 360000*100 / (4096 * i_total_x100) */
    n64 = 360000LL * 100LL;
    d64 = 4096LL * (int64_t)i_total_x100;
    g64 = mech_gcd64(n64, d64);
    tmp.out_md_num = (uint32_t)(n64 / g64);
    tmp.out_md_den = (uint32_t)(d64 / g64);

    /* ---------- 电角度：只与极对数有关（C-02）----------
     * theta_e_mdeg = raw * 360 * pp * 1000 / 4096
     * 关键：**必须先约分，再判断是否需要移位**。
     * 若先移位再约分，会把 1440000/4096 变成 360000/4096 = 5625/64，
     * 再乘回 2^shift 才等价，既多一次乘法又引入舍入误差。
     * pp=4 时约分后正是 raw*5625/16（与文档 §3.5 一致），且
     * 4095 * 5625 = 2.3e7，远离 int32 上限，根本不需要移位。 */
    {
        uint32_t pp = cfg->pole_pairs;
        uint32_t en = 360000U * pp;   /* pp<=8 -> 2.88e6，uint32 安全 */
        uint32_t ed = 4096U;
        uint32_t eg = mech_gcd(en, ed);

        en /= eg;
        ed /= eg;

        /* 仅在约分后仍可能溢出时才移位（pp > 100 量级才会发生）。
         * 语义：theta_e_mdeg = div_round(raw * e_num, e_den) << e_shift */
        tmp.e_shift = 0U;
        while (en > 524287U) {        /* 2^31 / 4095 ≈ 524287 */
            en >>= 1;
            tmp.e_shift++;
        }
        if (ed == 0U) {
            return -1;
        }
        tmp.e_num = (uint16_t)en;
        tmp.e_den = (uint16_t)ed;
    }

    /* ---------- 游标间距 Δ 与容差（C-04 自动推导）----------
     * Δ1 = 360 * (z01_num - z01_den) / z01_den，用 *1000 定点
     *    = 360000 * (num-den) / den（单位 0.001 deg）
     * 注意用 mech_div_round 而非 C 的 '/'：C 向零截断，-360000/42
     * 会得到 -8571（应为 -8572 四舍五入），与 Python 的向下取整不同。
     * 这里统一用对称四舍五入，保证正负齿差对称。 */
    tmp.d1_x1000 = mech_div_round64((int64_t)360000 * ((int32_t)tmp.z01_p_num - (int32_t)tmp.z01_p_den),
                                     (int64_t)tmp.z01_p_den);
    tmp.d2_x1000 = mech_div_round64((int64_t)360000 * ((int32_t)tmp.z02_p_num - (int32_t)tmp.z02_p_den),
                                     (int64_t)tmp.z02_p_den);

    {
        int32_t a1 = (tmp.d1_x1000 < 0) ? -tmp.d1_x1000 : tmp.d1_x1000;
        int32_t a2 = (tmp.d2_x1000 < 0) ? -tmp.d2_x1000 : tmp.d2_x1000;
        int32_t lim = (a1 < a2) ? a1 : a2;

        tmp.tol_limit_x1000 = lim / 2;                 /* 上限 = min(|Δ|)/2 */
        if (cfg->tol_override_x1000 != 0) {
            tmp.tol_x1000 = cfg->tol_override_x1000;   /* 调试覆盖 */
        } else {
            /* 0.7x：必须先乘后除。写成 (lim/2)/10*7 会丢精度：
             * lim/2=4286 时 4286/10=428, 428*7=2996，而正解是 3000 差一点。
             * 正解：4286*7/10 = 3000（文档 2999 是 d1 精确值 4285.7/2*0.7 的结果）。*/
            tmp.tol_x1000 = (int32_t)(((int64_t)(lim / 2) * 7) / 10);
        }
        tmp.verify_tol_x1000 = tmp.tol_x1000 * 2;      /* 交叉校验放宽 */
    }

    /* ---------- V6：分辨率 <= 0.01 deg/LSB ----------
     * res = 360 / i_total / 4096 deg = 360e9 / (i_total*4096) ndeg
     * 用 int64 计算，避免溢出
     *
     * 【2026-11-04 Q2】同步改用 i_total_x100，与换算系数口径一致。
     *   i_total_x100 = i_total*100
     *   ⇒ res = 360e9 * 100 / (i_total_x100 * 4096) ndeg
     * 旧式用 i_total_int（截断），会把 731.7857 当 731，
     * 分辨率报大 0.1%，虽只用于显示与 V6 判据，但与系数口径不一
     * 会让"t_mech 显示的 res"和"实际换算精度"对不上，排障时误导。
     * 溢出核查：360e9*100 = 3.6e13 < 9.2e18 ✓ */
    {
        int64_t res_ndeg = ((int64_t)360000000000LL * 100LL) /
                           (i_total_x100 * 4096LL);

        if (res_ndeg > (int64_t)MECH_RES_MAX_NDEG) {
            return -6;
        }
        tmp.res_ndeg_per_lsb = (uint32_t)res_ndeg;
    }

    /* ---------- 全部通过，写出 ---------- */
    if (r != RT_NULL) {
        *r = tmp;
    }
    return 0;
}

/* ============================================================
 * 默认值
 * ============================================================ */
void mech_cfg_default(mech_cfg_t *cfg)
{
    SM_ASSERT_PTR(cfg);

    /* 【Q-49】默认值不再内联硬写，改为取候选表的默认项。
     * 这样"出厂默认是哪一套"只有 MECH_CAND_DEFAULT 一处定义，
     * 不会出现 mech_cfg_default() 写 216 而候选表写 683 的两处漂移。 */
    if (mech_cand_get(MECH_CAND_DEFAULT, cfg) != MECH_OK) {
        /* 候选表异常时给出最小可用值，保证不返回全 0（全 0 会触发 V1） */
        memset(cfg, 0, sizeof(mech_cfg_t));
        cfg->version            = MECH_CFG_VERSION;
        cfg->enc_z01_num        = MECH_DEF_Z0;
        cfg->enc_z01_den        = MECH_DEF_Z1;
        cfg->enc_z02_num        = MECH_DEF_Z0;
        cfg->enc_z02_den        = MECH_DEF_Z2;
        cfg->i_in_num           = MECH_DEF_I_IN;
        cfg->i_in_den           = MECH_DEF_I_IN_DEN;
        cfg->i_out_num          = MECH_DEF_I_OUT;
        cfg->i_out_den          = MECH_DEF_I_OUT_DEN;
        cfg->pole_pairs         = MECH_DEF_POLE_PAIRS;
        cfg->out_min_x10        = MECH_DEF_OUT_MIN_X10;
        cfg->out_max_x10        = MECH_DEF_OUT_MAX_X10;
        cfg->tol_override_x1000 = 0;
    }
}

/* ============================================================
 * 出厂候选表 —— 【Q-49「按需配置，初始化时选择」】
 *            【2026-11-04 扩为 3 维 12 组合，用户裁定"一并纳入"】
 *
 * 12 组合的完整核算表、id 编号规则、teeth_tag 为何写 0，
 * 全部见 mech_cfg.h 的 MECH_CAND_COUNT 注释块（唯一真值说明处）。
 * 这里只放"真·参数"，派生量（joint / tol / out_den）一律不存 ——
 * 存了就会出现"代码改了公式、表里还是旧值"的静默错误（C-05）。
 *
 * 表顺序必须与 id 公式一致：
 *   id = i_out_sel*4 + i_in_sel*2 + enc_sel
 *   即外层 i_out、中层 i_in、内层 enc。
 * ============================================================ */
typedef struct {
    const char *name;
    uint16_t z0;        /* Z0 主动齿数 */
    uint16_t z1;        /* Z1 从动齿数（Z01 = z0:z1） */
    uint16_t z2;        /* Z2 从动齿数（Z02 = z0:z2） */
    uint16_t i_in_num;  /* 减速比第一级分子 */
    uint16_t i_in_den;  /* 减速比第一级分母（136.6 = 683/5，禁浮点） */
    uint16_t i_out_num; /* 减速比第二级分子（机型齿比主动齿） */
    uint16_t i_out_den; /* 减速比第二级分母（机型齿比从动齿） */
} mech_cand_desc_t;

static const mech_cand_desc_t g_mech_cand[MECH_CAND_COUNT] = {
    /* ---------- i_out_sel = 0 · 65:13（13 齿机型，= 5）---------- */
    /* 0 · 41:42:43 + 136.6 + 65:13 —— 2026-11-03 实测排除，保留备选 */
    { "41:42:43 136.6x65:13 ", 41U, 42U, 43U, 683U, 5U, 65U, 13U },
    /* 1 · 41:42:43 + 216   + 65:13 */
    { "41:42:43 216x65:13   ", 41U, 42U, 43U, 216U, 1U, 65U, 13U },
    /* 2 · 55:56:57 + 136.6 + 65:13 */
    { "55:56:57 136.6x65:13 ", 55U, 56U, 57U, 683U, 5U, 65U, 13U },
    /* 3 · 55:56:57 + 216   + 65:13 —— 出厂默认（现场实测通过）*/
    { "55:56:57 216x65:13   ", 55U, 56U, 57U, 216U, 1U, 65U, 13U },

    /* ---------- i_out_sel = 1 · 75:14（14 齿机型，2026-11-04 新增）---------- */
    /* 4 · 41:42:43 + 136.6 + 75:14 */
    { "41:42:43 136.6x75:14 ", 41U, 42U, 43U, 683U, 5U, 75U, 14U },
    /* 5 · 41:42:43 + 216   + 75:14 */
    { "41:42:43 216x75:14   ", 41U, 42U, 43U, 216U, 1U, 75U, 14U },
    /* 6 · 55:56:57 + 136.6 + 75:14 */
    { "55:56:57 136.6x75:14 ", 55U, 56U, 57U, 683U, 5U, 75U, 14U },
    /* 7 · 55:56:57 + 216   + 75:14 */
    { "55:56:57 216x75:14   ", 55U, 56U, 57U, 216U, 1U, 75U, 14U },

    /* ---------- i_out_sel = 2 · 53:11（11 齿机型，2026-11-04 新增）---------- */
    /* 8 · 41:42:43 + 136.6 + 53:11 */
    { "41:42:43 136.6x53:11 ", 41U, 42U, 43U, 683U, 5U, 53U, 11U },
    /* 9 · 41:42:43 + 216   + 53:11 */
    { "41:42:43 216x53:11   ", 41U, 42U, 43U, 216U, 1U, 53U, 11U },
    /*10 · 55:56:57 + 136.6 + 53:11 */
    { "55:56:57 136.6x53:11 ", 55U, 56U, 57U, 683U, 5U, 53U, 11U },
    /*11 · 55:56:57 + 216   + 53:11 */
    { "55:56:57 216x53:11   ", 55U, 56U, 57U, 216U, 1U, 53U, 11U },
};

const char *mech_cand_name(uint8_t id)
{
    if (id >= MECH_CAND_COUNT) {
        return "?";
    }
    return g_mech_cand[id].name;
}

uint8_t mech_cand_make_id(uint8_t enc_sel, uint8_t iin_sel, uint8_t iout_sel)
{
    if ((enc_sel >= MECH_ENC_SEL_COUNT) ||
        (iin_sel >= MECH_IIN_SEL_COUNT) ||
        (iout_sel >= MECH_IOUT_SEL_COUNT)) {
        return 0xFFU;
    }
    return (uint8_t)((uint8_t)(iout_sel) * 4U +
                     (uint8_t)(iin_sel) * 2U +
                     (uint8_t)(enc_sel));
}

/* ============================================================
 * 【2026-11-05 修复：enc_sel 位宽取错，导致 12 个候选里 6 个显示 "?"】
 * ------------------------------------------------------------
 * 原实现：*enc_sel = id & 0x03U;        ← 取 2 位
 *
 * 但 id 的编码是  id = iout*4 + iin*2 + enc，
 * 而 enc_sel 只有 2 个取值（0=41:42:43, 1=55:56:57），
 * 即**只占 1 位**。用 0x03 掩码会把 iin 那一位也当成 enc，
 * 于是 id=2/3/6/7/10/11 得到 enc_sel=2 或 3 ⇒ 越界 ⇒ 打印 "?"。
 *
 * 实测症状（2026-11-04 上机日志）：
 *   [MECH] cfg sel unchanged: cand 3 (? / 216 / 65:13 (13T))
 *                                   ↑ 这里本该是 55:56:57
 *
 * 纯显示缺陷，不影响 i_total / 系数派生（那些走 mech_cand_get 取表），
 * 但排障时看到 "?" 会怀疑配置错了，必须修。
 *
 * 与反向函数 mech_cand_make_id() 严格对称：
 *   make:  id = iout*4 + iin*2 + enc
 *   split: enc  = id & 0x01        (1 位)
 *          iin  = (id>>1) & 0x01   (1 位)
 *          iout = (id>>2) & 0x03   (2 位，3 个取值)
 * ============================================================ */
int mech_cand_split_id(uint8_t id, uint8_t *enc_sel,
                       uint8_t *iin_sel, uint8_t *iout_sel)
{
    if (id >= MECH_CAND_COUNT) {
        return MECH_ERR_PARAM;
    }
    if (enc_sel  != RT_NULL) { *enc_sel  = (uint8_t)(id & 0x01U); }
    if (iin_sel  != RT_NULL) { *iin_sel  = (uint8_t)((id >> 1) & 0x01U); }
    if (iout_sel != RT_NULL) { *iout_sel = (uint8_t)((id >> 2) & 0x03U); }
    return MECH_OK;
}

const char *mech_enc_sel_name(uint8_t sel)
{
    switch (sel) {
        case MECH_ENC_SEL_414243: return "41:42:43";
        case MECH_ENC_SEL_555657: return "55:56:57";
        default:                  return "?";
    }
}

const char *mech_iin_sel_name(uint8_t sel)
{
    switch (sel) {
        case MECH_IIN_SEL_1366: return "136.6";
        case MECH_IIN_SEL_216:  return "216";
        default:                return "?";
    }
}

const char *mech_iout_sel_name(uint8_t sel)
{
    switch (sel) {
        case MECH_IOUT_SEL_65_13: return "65:13 (13T)";
        case MECH_IOUT_SEL_75_14: return "75:14 (14T)";
        case MECH_IOUT_SEL_53_11: return "53:11 (11T)";
        default:                  return "?";
    }
}

int mech_cand_get(uint8_t id, mech_cfg_t *out)
{
    const mech_cand_desc_t *d;

    if (out == RT_NULL || id >= MECH_CAND_COUNT) {
        return MECH_ERR_PARAM;
    }
    d = &g_mech_cand[id];

    memset(out, 0, sizeof(mech_cfg_t));
    out->version            = MECH_CFG_VERSION;
    out->enc_z01_num        = d->z0;
    out->enc_z01_den        = d->z1;
    out->enc_z02_num        = d->z0;
    out->enc_z02_den        = d->z2;
    out->i_in_num           = d->i_in_num;
    out->i_in_den           = d->i_in_den;
    out->i_out_num          = d->i_out_num;
    out->i_out_den          = d->i_out_den;
    out->pole_pairs         = MECH_DEF_POLE_PAIRS;
    out->teeth_tag          = 0U;                  /* 兜底行，与机型齿型无关 */
    out->out_min_x10        = MECH_DEF_OUT_MIN_X10;
    out->out_max_x10        = MECH_DEF_OUT_MAX_X10;
    out->tol_override_x1000 = 0;                   /* 用按齿数自动推导 */
    out->crc32              = 0U;                  /* 由 table 层统一计算 */
    return MECH_OK;
}

int mech_table_apply_cand(mech_table_t *tbl, uint8_t id)
{
    mech_cfg_t cand;
    char       err[48];

    if (tbl == RT_NULL) {
        return MECH_ERR_PARAM;
    }
    /* 先算到临时变量并校验：非法候选绝不写坏现有表 */
    if (mech_cand_get(id, &cand) != MECH_OK) {
        return MECH_ERR_PARAM;
    }
    if (mech_cfg_validate(&cand, err, sizeof(err)) != MECH_OK) {
        rt_kprintf("[MECH] cand %u illegal: %s\n", (unsigned)id, err);
        return MECH_ERR_VALIDATE;
    }

    /* 只替换兜底行、count 归 1 —— "按需配置，初始化时选择"的语义就是
     * 一次只生效一套。其余机型行由 t_mech_copy / t_mech_set 现场追加。 */
    memset(tbl, 0, sizeof(mech_table_t));
    tbl->version  = MECH_TABLE_VERSION;
    tbl->count    = 1U;
    tbl->active   = 0U;
    tbl->row_size = (uint16_t)sizeof(mech_cfg_t);
    tbl->row[0]   = cand;
    return MECH_OK;
}

void mech_cand_selftest(void)
{
    uint8_t i;

    rt_kprintf("[MECH] cand selftest (%u candidates, id=iout*4+iin*2+enc):\n",
               (unsigned)MECH_CAND_COUNT);
    for (i = 0U; i < MECH_CAND_COUNT; i++) {
        mech_cfg_t  c;
        mech_rt_t   r;
        char        err[48];

        if (mech_cand_get(i, &c) != MECH_OK) {
            rt_kprintf("  [%2u] %s  GET FAIL\n", (unsigned)i, mech_cand_name(i));
            continue;
        }
        if (mech_cfg_derive(&c, &r, err, sizeof(err)) != MECH_OK) {
            rt_kprintf("  [%2u] %s  FAIL: %s\n",
                       (unsigned)i, mech_cand_name(i), err);
            continue;
        }
        rt_kprintf("  [%2u] %s OK joint=%d i=%d.%02d res=%u out=%u/%u\n",
                   (unsigned)i, mech_cand_name(i),
                   (int)r.joint_period,
                   (int)(r.i_total_x100 / 100), (int)(r.i_total_x100 % 100),
                   (unsigned)r.res_ndeg_per_lsb,
                   (unsigned)r.out_md_num, (unsigned)r.out_md_den);
    }
}

void mech_table_default(mech_table_t *tbl)
{
    SM_ASSERT_PTR(tbl);

    /* 【Q-49】默认表 = 默认候选的那一套。若候选表异常则退回最小可用表。 */
    if (mech_table_apply_cand(tbl, MECH_CAND_DEFAULT) != MECH_OK) {
        memset(tbl, 0, sizeof(mech_table_t));
        tbl->version  = MECH_TABLE_VERSION;
        tbl->count    = 1U;
        tbl->active   = 0U;
        tbl->row_size = (uint16_t)sizeof(mech_cfg_t);
        mech_cfg_default(&tbl->row[0]);
    }
    /* crc32 由 mech_table_save 计算 */
}

/* ============================================================
 * 校验（对外，只给结论）
 * ============================================================ */
int mech_cfg_validate(const mech_cfg_t *cfg, char *err, uint16_t err_len)
{
    int rc;

    if (cfg == RT_NULL) {
        return MECH_ERR_PARAM;
    }

    rc = mech_calc(cfg, RT_NULL);
    if (rc != 0) {
        /* rc 为 -1..-8，对应 V1..V8 */
        int32_t vn = -rc;
        set_err(err, err_len, "V%d fail", vn, 0, 0);
        return MECH_ERR_VALIDATE;
    }
    return MECH_OK;
}

int mech_table_validate(const mech_table_t *tbl, char *err, uint16_t err_len)
{
    uint8_t i;
    uint8_t seen[MECH_PROFILE_MAX];
    uint8_t has_fallback = 0U;

    if (tbl == RT_NULL) {
        return MECH_ERR_PARAM;
    }

    /* ---------- T1：count 范围 ---------- */
    if (tbl->count < 1U || tbl->count > MECH_PROFILE_MAX) {
        set_err(err, err_len, "T1 fail: count=%d > %d", tbl->count,
                MECH_PROFILE_MAX, 0);
        return MECH_ERR_VALIDATE;
    }

    /* ---------- T2：active 范围 ---------- */
    if (tbl->active >= tbl->count) {
        set_err(err, err_len, "T2 fail: active=%d >= count=%d", tbl->active,
                tbl->count, 0);
        return MECH_ERR_VALIDATE;
    }

    /* ---------- T3：row_size 匹配 ---------- */
    if (tbl->row_size != (uint16_t)sizeof(mech_cfg_t)) {
        set_err(err, err_len, "T3 fail: row_size=%d != %d", tbl->row_size,
                (int32_t)sizeof(mech_cfg_t), 0);
        return MECH_ERR_VALIDATE;
    }

    /* ---------- T4：teeth_tag 唯一（0 除外）---------- */
    for (i = 0U; i < MECH_PROFILE_MAX; i++) {
        seen[i] = 0U;
    }
    for (i = 0U; i < tbl->count; i++) {
        uint8_t tag = tbl->row[i].teeth_tag;

        if (tag == 0U) {
            has_fallback = 1U;
            continue;
        }
        if (seen[tag] != 0U) {
            set_err(err, err_len, "T4 fail: teeth_tag=%d duplicated", tag, 0, 0);
            return MECH_ERR_VALIDATE;
        }
        if (tag >= MECH_PROFILE_MAX) {
            set_err(err, err_len, "T4 fail: teeth_tag=%d out of range", tag, 0, 0);
            return MECH_ERR_VALIDATE;
        }
        seen[tag] = 1U;
    }

    /* ---------- T6：兜底行必须存在 ---------- */
    if (has_fallback == 0U) {
        set_err(err, err_len, "T6 fail: no teeth_tag=0 fallback row", 0, 0, 0);
        return MECH_ERR_VALIDATE;
    }

    /* ---------- T5：每行单独合法（一行坏则整表拒）---------- */
    for (i = 0U; i < tbl->count; i++) {
        int rc = mech_calc(&tbl->row[i], RT_NULL);

        if (rc != 0) {
            set_err(err, err_len, "T5 fail: row%d V%d illegal", (int32_t)i,
                    -rc, 0);
            return MECH_ERR_VALIDATE;
        }
    }

    return MECH_OK;
}

int mech_cfg_derive(const mech_cfg_t *cfg, mech_rt_t *rt,
                    char *err, uint16_t err_len)
{
    int rc;

    if (cfg == RT_NULL || rt == RT_NULL) {
        return MECH_ERR_PARAM;
    }

    rc = mech_calc(cfg, rt);
    if (rc != 0) {
        set_err(err, err_len, "V%d fail", -rc, 0, 0);
        return MECH_ERR_VALIDATE;
    }
    return MECH_OK;
}

int mech_find_profile(const mech_table_t *tbl, uint8_t teeth_type)
{
    uint8_t i;
    int     fallback = -1;

    if (tbl == RT_NULL || tbl->count == 0U || tbl->count > MECH_PROFILE_MAX) {
        return -1;
    }

    for (i = 0U; i < tbl->count; i++) {
        if (tbl->row[i].teeth_tag == teeth_type && teeth_type != 0U) {
            return (int)i;
        }
        if (tbl->row[i].teeth_tag == 0U) {
            fallback = (int)i;
        }
    }
    return fallback;
}

/* ============================================================
 * 打印（t_mech 用）
 * ============================================================ */

/* 定点小数打印辅助：取绝对值供拆分整数用。
 * INT32_MIN 取反会溢出，先夹到 INT32_MAX 兜底（正常量级远达不到）。 */
static int32_t fix_abs(int32_t v)
{
    if (v == INT32_MIN) {
        return INT32_MAX;
    }
    return (v < 0) ? -v : v;
}

void mech_cfg_dump(const mech_cfg_t *cfg, const mech_rt_t *rt,
                   const mech_table_t *tbl)
{
    SM_ASSERT_PTR(cfg);
    SM_ASSERT_PTR(rt);

    if (tbl != RT_NULL) {
        rt_kprintf("[MECH] table: count=%d active=%d row_size=%d size=%dB\n",
                   (int)tbl->count, (int)tbl->active, (int)tbl->row_size,
                   (int)sizeof(mech_table_t));
    }

    rt_kprintf("[MECH] cfg: z01=%d:%d z02=%d:%d i_in=%d/%d i_out=%d/%d pp=%d\n",
               (int)cfg->enc_z01_num, (int)cfg->enc_z01_den,
               (int)cfg->enc_z02_num, (int)cfg->enc_z02_den,
               (int)cfg->i_in_num, (int)cfg->i_in_den,
               (int)cfg->i_out_num, (int)cfg->i_out_den,
               (int)cfg->pole_pairs);
    rt_kprintf("[MECH] cfg: teeth_tag=%d travel=%d..%d (x10) tol_ovr=%d\n",
               (int)cfg->teeth_tag, (int)cfg->out_min_x10,
               (int)cfg->out_max_x10, (int)cfg->tol_override_x1000);

    /* 派生量：d1/d2/tol 用 0.001 deg，手工拆整数避免浮点 printf。
     * 注意负数：C 的 / 向零截断、% 取余符号跟被除数走，
     * 直接 %03d 会把 -8571 打成 "-8.-571"。故先取绝对值再拆，
     * 符号单独打印。 */
    rt_kprintf("[MECH] derive: d1=%s%d.%03d d2=%s%d.%03d tol=%s%d.%03d (limit %s%d.%03d)\n",
               (rt->d1_x1000        < 0) ? "-" : "", (int)(fix_abs(rt->d1_x1000) / 1000),        (int)(fix_abs(rt->d1_x1000) % 1000),
               (rt->d2_x1000        < 0) ? "-" : "", (int)(fix_abs(rt->d2_x1000) / 1000),        (int)(fix_abs(rt->d2_x1000) % 1000),
               (rt->tol_x1000       < 0) ? "-" : "", (int)(fix_abs(rt->tol_x1000) / 1000),       (int)(fix_abs(rt->tol_x1000) % 1000),
               (rt->tol_limit_x1000 < 0) ? "-" : "", (int)(fix_abs(rt->tol_limit_x1000) / 1000), (int)(fix_abs(rt->tol_limit_x1000) % 1000));

    rt_kprintf("[MECH] derive: i_total=%s%d.%02d joint_period=%d travel=%s%d.%02d turns\n",
               (rt->i_total_x100      < 0) ? "-" : "", (int)(fix_abs(rt->i_total_x100) / 100),      (int)(fix_abs(rt->i_total_x100) % 100),
               (int)rt->joint_period,
               (rt->travel_turns_x100 < 0) ? "-" : "", (int)(fix_abs(rt->travel_turns_x100) / 100), (int)(fix_abs(rt->travel_turns_x100) % 100));

    /* V3 的 need 阈值 = ceil(travel_turns * 1.15)，打印出来才能判断好还是坏。
     * 用向上取整，与文档"need>=1087"一致（精确值 1086.0 的 ceil 是 1087 的说法见下）*/
    {
        int32_t need_x1 = (int32_t)(((int32_t)rt->travel_turns_x100 * MECH_TRAVEL_MARGIN_X100
                                     + 9999) / 10000);
        rt_kprintf("[MECH] derive: joint need>=%d (travel*1.15)\n", (int)need_x1);
    }

    rt_kprintf("[MECH] derive: cnt=[%d,%d] k=[%d,%d] n=%d\n",
               (int)rt->cnt_min, (int)rt->cnt_max,
               (int)rt->k_min, (int)rt->k_max, (int)rt->k_count);

    rt_kprintf("[MECH] derive: res=%u ndeg/LSB\n", (unsigned)rt->res_ndeg_per_lsb);

    /* 【2026-11-04 Q2】分子已拓宽到 uint32，最大值 140625，
     * 用 (int) 强转虽不会溢出（int 是 32 位），但改用 %u 更贴合无符号语义 */
    rt_kprintf("[MECH] coef: out_x10=dcnt*%u/%u  out_mdeg=dcnt*%u/%u\n",
               (unsigned)rt->out_num, (unsigned)rt->out_den,
               (unsigned)rt->out_md_num, (unsigned)rt->out_md_den);

    rt_kprintf("[MECH] coef: theta_e_mdeg=raw*%d/%d (shift %d)\n",
               (int)rt->e_num, (int)rt->e_den, (int)rt->e_shift);
}