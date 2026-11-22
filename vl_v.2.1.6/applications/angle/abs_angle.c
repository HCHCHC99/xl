/**
 * @file    abs_angle.c
 * @brief   回转角度解算 —— 两段式（上电游标 / 运行增量）
 * @note    2026-10-28 · 仅整数运算，无浮点（遵循工程 STD-FIXEDPOINT 约定）
 *
 * ============================================================
 * 【游标解算的数学基础（一句话版）】
 * ------------------------------------------------------------
 * Z0 转 1 圈，Z1 转 z0/z1 圈 ⇒ Z1 相对 Z0 的相位漂移
 *     Δ1 = 360 × (z0 - z1) / z1 度
 * 所以每隔 360/|Δ1| 圈，相位差才走满一圈。
 * 反过来说：给定 Z0 的 12bit 单圈值，只有唯一一个"绝对圈数 k"
 * 能让 Z1 的预测值对上实测值 —— 这就是绝对位置的来源。
 *
 * A2 配置（41:42:43）：Δ1 = -8.571°，唯一区间 = 42 圈；
 * 相邻 k 造成的 Z1 残差差 = 4096×(41-42)/42 ≈ -97.5 LSB。
 * ⇒ 只要残差 < 48 LSB 且次小值明显更大，就是唯一解。
 *
 * ============================================================
 * 【与本工程既有 mech_cfg 的衔接】
 * ------------------------------------------------------------
 * 圈数范围 k ∈ [rt->k_min, rt->k_max] 已由 mech_cfg 按行程算好
 * （含 ±2° 容差），本模块直接复用，不重新推导，避免两处漂移。
 * 角度换算也用 mech_rt_t 的 out_md_num/out_md_den 约分系数。
 * ============================================================
 */

#include <rtthread.h>
#include <string.h>

#include "abs_angle.h"
#include "mech_cfg_store.h"
#include "bsp_dev_kth7812.h"
#include "../sys_param/sys_param.h"     /* 三颗编码器零点（2026-11-02） */

/* 单圈计数 */
#define ABS_ENC_FULL        4096
#define ABS_ENC_HALF        2048

/* 连续失败多少次判为丢步 */
#define ABS_JUMP_LIMIT      3

/* 增量单次跳变上限：超过半圈（2048）认为丢步。
 * 依据：1 kHz 采样、Z0 侧 3000 rpm 时每 ms 仅 205 LSB，余量 10 倍。
 * 若将来降低采样率或提高转速，必须重新核算此值。 */
#define ABS_INCR_MAX_JUMP   2048

/* ============================================================
 * 私有状态
 * ============================================================ */
static abs_angle_state_t s_st;

/* 上一次读到的 Z0 原始值；-1 = 尚未建立基线 */
static int32_t s_last_raw0 = -1;
static uint8_t s_jump_streak = 0;

/* ============================================================
 * 工具
 * ============================================================ */

/** 环绕差：把 a-b 归一到 [-2048, 2047]，处理 4095→0 跳变 */
static int32_t wrap_diff(int32_t a, int32_t b)
{
    int32_t d = (a - b) % ABS_ENC_FULL;

    if (d > ABS_ENC_HALF)  { d -= ABS_ENC_FULL; }
    if (d < -ABS_ENC_HALF) { d += ABS_ENC_FULL; }
    return d;
}

static int32_t iabs32(int32_t v)
{
    return (v < 0) ? -v : v;
}

/**
 * @brief 预测：给定 Z0 累计计数，推算 Z1（或 Z2）的 12bit 值
 * @param cnt       Z0 累计计数（可为负）
 * @param p_num/p_den 约分后的传动比 Z0:Zn
 * @return 0~4095 的预测原始值
 * @note  用 int64 防中间溢出；除法用对称四舍五入，保证正负计数一致
 */
static int32_t predict_raw(int32_t cnt, uint16_t p_num, uint16_t p_den)
{
    int64_t num = (int64_t)cnt * (int64_t)p_num;
    int64_t den = (int64_t)p_den;
    int64_t q;
    int32_t r;

    if (den <= 0) { return 0; }

    if (num >= 0) { q = (num + den / 2) / den; }
    else          { q = -((-num + den / 2) / den); }

    r = (int32_t)(q % ABS_ENC_FULL);
    if (r < 0) { r += ABS_ENC_FULL; }
    return r;
}

/**
 * @brief 唯一性半间距：相邻 k 之间 Zn 残差的步长 / 2
 * @note  Δstep = 4096 × (p_num - p_den) / p_den，取绝对值一半
 *        A2 的 Z01: |4096×(41-42)/42| = 97.5 ⇒ 48
 */
static int32_t uniq_half_step(uint16_t p_num, uint16_t p_den)
{
    int32_t step;

    if (p_den == 0U) { return 1; }
    step = (int32_t)(((int64_t)ABS_ENC_FULL *
                      ((int32_t)p_num - (int32_t)p_den)) / (int32_t)p_den);
    step = iabs32(step);
    if (step < 2) { step = 2; }
    return step / 2;
}

const char *abs_angle_mode_name(abs_angle_mode_t m)
{
    switch (m) {
        case ABS_ANGLE_MODE_UNINIT: return "UNINIT";
        case ABS_ANGLE_MODE_COARSE: return "COARSE";
        case ABS_ANGLE_MODE_INCR:   return "INCR";
        case ABS_ANGLE_MODE_LOST:   return "LOST";
        default:                    return "?";
    }
}

const char *abs_angle_err_name(abs_angle_err_t e)
{
    switch (e) {
        case ABS_ANGLE_OK:            return "OK";
        case ABS_ANGLE_ERR_NOT_INIT:  return "NOT_INIT";
        case ABS_ANGLE_ERR_ENC_Z0:    return "ENC_Z0_FAIL";
        case ABS_ANGLE_ERR_ENC_Z1:    return "ENC_Z1_FAIL";
        case ABS_ANGLE_ERR_ENC_Z2:    return "ENC_Z2_FAIL";
        case ABS_ANGLE_ERR_NO_MATCH:  return "NO_MATCH";
        case ABS_ANGLE_ERR_AMBIGUOUS: return "AMBIGUOUS";
        case ABS_ANGLE_ERR_OUT_RANGE: return "OUT_OF_RANGE";
        case ABS_ANGLE_ERR_JUMP:      return "JUMP";
        case ABS_ANGLE_ERR_NO_CFG:    return "NO_CFG";
        default:                      return "?";
    }
}

/* ============================================================
 * 上电解算（游标法）
 * ============================================================ */
abs_angle_err_t abs_angle_solve(abs_angle_err_t *err)
{
    const mech_rt_t *rt;
    uint16_t raw0, raw1, raw2;
    int32_t  k;
    int32_t  best_k    = 0;
    int32_t  best_score = 0x7FFFFFFF;
    int32_t  second_score = 0x7FFFFFFF;
    int32_t  guard1, guard2, guard;
    uint8_t  found = 0;

    rt = mech_get_rt();
    if (rt == RT_NULL || g_mech.is_loaded == 0U) {
        if (err) { *err = ABS_ANGLE_ERR_NO_CFG; }
        s_st.last_err = ABS_ANGLE_ERR_NO_CFG;
        return ABS_ANGLE_ERR_NO_CFG;
    }

    /* ---- 读三颗（各读 2 次取一致，降低单次噪声影响）---- */
    {
        uint16_t a0, b0, a1, b1, a2, b2;

        a0 = KTH78_ReadAngle((KTH7812_DEV_DEF)0);
        b0 = KTH78_ReadAngle((KTH7812_DEV_DEF)0);
        if (a0 == KTH7812_ANGLE_INVALID || b0 == KTH7812_ANGLE_INVALID) {
            s_st.fail_cnt++;
            s_st.last_err = ABS_ANGLE_ERR_ENC_Z0;
            if (err) { *err = ABS_ANGLE_ERR_ENC_Z0; }
            return ABS_ANGLE_ERR_ENC_Z0;
        }
        /* 两次差超过 2 LSB 说明在动或噪声大，取后一次并继续
         * （解算本身有容差，不要求严格相等，否则手一碰就失败） */
        raw0 = b0;
        (void)a0;

        a1 = KTH78_ReadAngle((KTH7812_DEV_DEF)1);
        b1 = KTH78_ReadAngle((KTH7812_DEV_DEF)1);
        if (a1 == KTH7812_ANGLE_INVALID || b1 == KTH7812_ANGLE_INVALID) {
            s_st.fail_cnt++;
            s_st.last_err = ABS_ANGLE_ERR_ENC_Z1;
            if (err) { *err = ABS_ANGLE_ERR_ENC_Z1; }
            return ABS_ANGLE_ERR_ENC_Z1;
        }
        raw1 = b1;

        a2 = KTH78_ReadAngle((KTH7812_DEV_DEF)2);
        b2 = KTH78_ReadAngle((KTH7812_DEV_DEF)2);
        if (a2 == KTH7812_ANGLE_INVALID || b2 == KTH7812_ANGLE_INVALID) {
            s_st.fail_cnt++;
            s_st.last_err = ABS_ANGLE_ERR_ENC_Z2;
            if (err) { *err = ABS_ANGLE_ERR_ENC_Z2; }
            return ABS_ANGLE_ERR_ENC_Z2;
        }
        raw2 = b2;
    }

    /* ============================================================
     * 【2026-11-02 新增】应用三颗编码器零点偏移（方案 A）
     * ------------------------------------------------------------
     * 为什么放在这里（读三颗之后、游标循环之前）：
     *   游标算法用的是三颗之间的"相对角度关系"，零点偏移属于
     *   各颗的独立安装误差。先减掉零点，三颗就都归到同一物理基准，
     *   游标公式完全不用改。
     *
     * 未校准时（enc_zero_valid=0）直接跳过，等价于零点全 0，
     * 保持原有行为不变——这样在没做过校准的机器上不会引入回归。
     * ============================================================ */
    {
        uint16_t zero[3];

        if (sys_param_get_enc_zero(zero) != 0U) {
            raw0 = (uint16_t)(((int32_t)raw0 - (int32_t)zero[0] + ABS_ENC_FULL)
                              % ABS_ENC_FULL);
            raw1 = (uint16_t)(((int32_t)raw1 - (int32_t)zero[1] + ABS_ENC_FULL)
                              % ABS_ENC_FULL);
            raw2 = (uint16_t)(((int32_t)raw2 - (int32_t)zero[2] + ABS_ENC_FULL)
                              % ABS_ENC_FULL);
        }
    }

    /* ---- 唯一性守卫：残差必须小于相邻 k 的半间距 ---- */
    guard1 = uniq_half_step(rt->z01_p_num, rt->z01_p_den);
    guard2 = uniq_half_step(rt->z02_p_num, rt->z02_p_den);
    guard  = (guard1 < guard2) ? guard1 : guard2;
    if (guard < 3) { guard = 3; }     /* 不低于量化噪声 */

    /* 【2026-11-02 新增】先把输入与判据存进状态，失败时也能查 */
    s_st.dbg_raw0 = (int32_t)raw0;
    s_st.dbg_raw1 = (int32_t)raw1;
    s_st.dbg_raw2 = (int32_t)raw2;
    s_st.dbg_guard = guard;

    /* ---- 遍历 k，取残差最小者 ---- */
    for (k = rt->k_min; k <= rt->k_max; k++) {
        int32_t cnt = k * ABS_ENC_FULL + (int32_t)raw0;
        int32_t r1  = wrap_diff(predict_raw(cnt, rt->z01_p_num, rt->z01_p_den), (int32_t)raw1);
        int32_t r2  = wrap_diff(predict_raw(cnt, rt->z02_p_num, rt->z02_p_den), (int32_t)raw2);
        int32_t score = iabs32(r1) + iabs32(r2);

        if (score < best_score) {
            second_score = best_score;
            best_score   = score;
            best_k       = k;
            found        = 1U;
        } else if (score < second_score) {
            second_score = score;
        }
    }

    if (found == 0U) {
        s_st.fail_cnt++;
        s_st.last_err = ABS_ANGLE_ERR_NO_MATCH;
        if (err) { *err = ABS_ANGLE_ERR_NO_MATCH; }
        return ABS_ANGLE_ERR_NO_MATCH;
    }

    /* 两路残差各自都要落在守卫内，否则说明"用错了齿型"或机械打滑 */
    {
        int32_t cnt = best_k * ABS_ENC_FULL + (int32_t)raw0;
        int32_t r1  = wrap_diff(predict_raw(cnt, rt->z01_p_num, rt->z01_p_den), (int32_t)raw1);
        int32_t r2  = wrap_diff(predict_raw(cnt, rt->z02_p_num, rt->z02_p_den), (int32_t)raw2);

        /* 【2026-11-02 新增】失败路径也要留下中间量，否则只能靠猜 */
        s_st.dbg_best_k = best_k;
        s_st.dbg_r1     = r1;
        s_st.dbg_r2     = r2;
        s_st.dbg_score  = iabs32(r1) + iabs32(r2);
        s_st.dbg_second = second_score;

        if (iabs32(r1) > guard || iabs32(r2) > guard) {
            s_st.fail_cnt++;
            s_st.last_err = ABS_ANGLE_ERR_NO_MATCH;
            if (err) { *err = ABS_ANGLE_ERR_NO_MATCH; }
            return ABS_ANGLE_ERR_NO_MATCH;
        }

        /* 唯一性：最优必须明显优于次优（差 2 倍以上），
         * 否则说明落在两个 k 的分界上，位置不可信 */
        if (second_score < 0x7FFFFFFF &&
            best_score * 2 > second_score &&
            second_score - best_score < guard) {
            s_st.fail_cnt++;
            s_st.last_err = ABS_ANGLE_ERR_AMBIGUOUS;
            if (err) { *err = ABS_ANGLE_ERR_AMBIGUOUS; }
            return ABS_ANGLE_ERR_AMBIGUOUS;
        }
    }

    /* ---- 行程范围校验 ---- */
    {
        int32_t cnt = best_k * ABS_ENC_FULL + (int32_t)raw0;
        if (cnt < rt->cnt_min || cnt > rt->cnt_max) {
            s_st.fail_cnt++;
            s_st.last_err = ABS_ANGLE_ERR_OUT_RANGE;
            if (err) { *err = ABS_ANGLE_ERR_OUT_RANGE; }
            return ABS_ANGLE_ERR_OUT_RANGE;
        }

        /* ---- 成功：建立增量基线 ---- */
        s_st.k          = best_k;
        s_st.cnt_accum  = cnt;
        s_st.raw_z0     = (int32_t)raw0;
        s_last_raw0     = (int32_t)raw0;
        s_jump_streak   = 0U;
        s_st.solve_cnt++;
        s_st.mode       = ABS_ANGLE_MODE_INCR;
        s_st.last_err   = ABS_ANGLE_OK;
    }

    if (err) { *err = ABS_ANGLE_OK; }
    return ABS_ANGLE_OK;
}

/* ============================================================
 * 运行期增量更新
 * ============================================================ */
abs_angle_err_t abs_angle_update(uint32_t now_ms)
{
    uint16_t raw;
    int32_t  raw_i;
    int32_t  delta;

    (void)now_ms;   /* 预留：将来做速度估计/超时检测 */

    if (s_st.mode != ABS_ANGLE_MODE_INCR &&
        s_st.mode != ABS_ANGLE_MODE_COARSE) {
        s_st.last_err = ABS_ANGLE_ERR_NOT_INIT;
        return ABS_ANGLE_ERR_NOT_INIT;
    }

    raw = KTH78_ReadAngle((KTH7812_DEV_DEF)0);
    if (raw == KTH7812_ANGLE_INVALID) {
        s_st.fail_cnt++;
        s_st.last_err = ABS_ANGLE_ERR_ENC_Z0;
        return ABS_ANGLE_ERR_ENC_Z0;
    }

    raw_i = (int32_t)raw;

    /* 首次调用：只建基线，不累加 */
    if (s_last_raw0 < 0) {
        s_last_raw0   = raw_i;
        s_st.raw_z0   = raw_i;
        s_st.mode     = ABS_ANGLE_MODE_INCR;
        s_st.last_err = ABS_ANGLE_OK;
        return ABS_ANGLE_OK;
    }

    delta = wrap_diff(raw_i, s_last_raw0);

    /* 跳变保护：单次超半圈认为丢步/干扰 */
    if (iabs32(delta) > ABS_INCR_MAX_JUMP) {
        s_jump_streak++;
        s_st.jump_cnt++;
        if (s_jump_streak >= ABS_JUMP_LIMIT) {
            s_st.mode     = ABS_ANGLE_MODE_LOST;
            s_st.last_err = ABS_ANGLE_ERR_JUMP;
            return ABS_ANGLE_ERR_JUMP;
        }
        /* 未达阈值：丢弃本次，不累加，避免污染累计值 */
        s_st.last_err = ABS_ANGLE_ERR_JUMP;
        return ABS_ANGLE_ERR_JUMP;
    }
    s_jump_streak = 0U;

    s_st.cnt_accum += delta;
    s_st.raw_z0     = raw_i;
    s_last_raw0     = raw_i;
    s_st.incr_cnt++;
    s_st.last_err   = ABS_ANGLE_OK;

    return ABS_ANGLE_OK;
}

/* ============================================================
 * 取角度
 * ============================================================ */
abs_angle_err_t abs_angle_get_mdeg(int32_t *mdeg)
{
    const mech_rt_t *rt;
    int32_t out;

    if (mdeg == RT_NULL) { return ABS_ANGLE_ERR_NOT_INIT; }

    if (s_st.mode != ABS_ANGLE_MODE_INCR &&
        s_st.mode != ABS_ANGLE_MODE_COARSE) {
        return ABS_ANGLE_ERR_NOT_INIT;
    }

    rt = mech_get_rt();
    if (rt == RT_NULL || rt->out_md_den == 0U) {
        return ABS_ANGLE_ERR_NO_CFG;
    }

    /* out_mdeg = div_round(cnt * out_md_num, out_md_den)
     * 用 int64：cnt 最大约 4.4e6，num 约 5625 ⇒ 2.5e10，超 int32 */
    {
        int64_t num = (int64_t)s_st.cnt_accum * (int64_t)rt->out_md_num;
        int64_t den = (int64_t)rt->out_md_den;

        if (num >= 0) { out = (int32_t)((num + den / 2) / den); }
        else          { out = (int32_t)(-((-num + den / 2) / den)); }
    }

    *mdeg = out;
    s_st.angle_mdeg = (uint32_t)((out < 0) ? -out : out);
    return ABS_ANGLE_OK;
}

abs_angle_err_t abs_angle_get_x10(int32_t *x10)
{
    int32_t mdeg = 0;
    abs_angle_err_t e;

    if (x10 == RT_NULL) { return ABS_ANGLE_ERR_NOT_INIT; }

    e = abs_angle_get_mdeg(&mdeg);
    if (e != ABS_ANGLE_OK) { return e; }

    /* mdeg -> x10：除以 100，对称四舍五入 */
    if (mdeg >= 0) { *x10 = (mdeg + 50) / 100; }
    else           { *x10 = -((-mdeg + 50) / 100); }

    return ABS_ANGLE_OK;
}

/* ============================================================
 * 失效
 * ============================================================ */
void abs_angle_invalidate(void)
{
    /* 注意：不要 memset 整个 s_st，否则统计量被清，
     * 排障时看不到"失效前发生过什么"。只清运行态。 */
    s_st.mode       = ABS_ANGLE_MODE_UNINIT;
    s_st.k          = 0;
    s_st.cnt_accum  = 0;
    s_st.raw_z0     = 0;
    s_st.angle_mdeg = 0U;
    s_st.last_err   = ABS_ANGLE_ERR_NOT_INIT;
    s_last_raw0     = -1;
    s_jump_streak   = 0U;
}

void abs_angle_get_state(abs_angle_state_t *out)
{
    if (out == RT_NULL) { return; }
    *out = s_st;
}