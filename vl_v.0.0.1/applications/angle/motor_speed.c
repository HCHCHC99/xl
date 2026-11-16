/**
 * @file    motor_speed.c
 * @brief   转速/角速度计算实现（Z0 差分 + 20kHz 硬件时基）
 *
 * 设计说明见 motor_speed.h 顶部注释。
 */

#include "motor_speed.h"
#include "mech_cfg_store.h"

#include <string.h>

/* ============================================================
 * 私有状态
 * ============================================================ */
typedef struct {
    int32_t  last_cnt;        /* 上次窗口起点的 Z0 计数 */
    uint32_t last_tick;       /* 上次窗口起点的硬件 tick */
    uint8_t  has_base;        /* 是否已建立窗口基线 */

    motor_speed_state_t pub;  /* 对外发布的状态 */
} motor_speed_priv_t;

static motor_speed_priv_t s_priv;

/* ============================================================
 * 初始化
 * ============================================================ */
void motor_speed_init(void)
{
    memset(&s_priv, 0, sizeof(s_priv));
}

/* ============================================================
 * 周期更新
 * ------------------------------------------------------------
 * 调用方保持 1ms 节拍；本函数按 tick 判断窗口是否已满。
 * 用 tick 而非 ms 判断，是为了让"窗口边界"的抖动也是 50us 级。
 * ============================================================ */
void motor_speed_update(int32_t cnt_now, uint32_t now_tick)
{
    uint32_t dt_ticks;
    int32_t  delta;
    const mech_rt_t *rt;

    /* ---- 首次：只建基线 ---- */
    if (s_priv.has_base == 0U)
    {
        s_priv.last_cnt  = cnt_now;
        s_priv.last_tick = now_tick;
        s_priv.has_base  = 1U;
        return;
    }

    /* ---- 未满窗口：直接返回 ----
     * 注意用无符号减法，天然处理 tick 回绕（2^32/20kHz ≈ 59.6h） */
    dt_ticks = now_tick - s_priv.last_tick;
    if (dt_ticks < MOTOR_SPEED_WINDOW_TICKS)
    {
        return;
    }

    /* ---- 窗口过长：暂停过或任务被长期阻塞 ----
     * 只重建基线、不计算。理由：
     *   1) dt 过大时 delta/dt 会算出接近 0 的假转速
     *   2) 这段时间内转角可能已超半圈，delta 本身不可信
     *      （motor_angle 的 wrap 保护已挡掉越界增量，
     *        但 delta 累计语义仍不可靠） */
    if (dt_ticks > MOTOR_SPEED_WINDOW_MAX_TICKS)
    {
        s_priv.last_cnt  = cnt_now;
        s_priv.last_tick = now_tick;
        s_priv.pub.reject_cnt++;
        s_priv.pub.valid = 0U;
        return;
    }

    /* 记录最大 dt，用于判断调度抖动（正常应 ≈ WINDOW_TICKS） */
    if (dt_ticks > s_priv.pub.dt_max_ticks)
    {
        s_priv.pub.dt_max_ticks = dt_ticks;
    }

    if (dt_ticks == 0U)
    {
        /* 理论不可达（上面已判 >= WINDOW），防御式返回 */
        s_priv.pub.reject_cnt++;
        return;
    }

    /* ---- 算增量 ---- */
    delta = cnt_now - s_priv.last_cnt;

    /* ============================================================
     * 核心换算：一步算到输出轴，避免二次截断
     * ------------------------------------------------------------
     * 物理关系：
     *   1) Z0 转 1 圈 = 4096 计数
     *   2) Z0 转 i_total 圈 = 输出轴转 1 圈
     *      （i_total_x100 = i_total * 100，1080 → 108000）
     *   3) 转速 RPM = 转数 / 时间(s) * 60
     *
     * 推导（全部整数，避免浮点，Q-63 未批准）：
     *   输出轴转数 = delta / 4096 / i_total
     *   时间(s)   = dt_ticks / 20000
     *   rpm_out   = (delta / 4096 / i_total) / (dt_ticks / 20000) * 60
     *             = delta * 20000 * 60 / (4096 * i_total * dt_ticks)
     *             = delta * 1200000 / (4096 * i_total * dt_ticks)
     *
     *   换成 x100（0.01 RPM）：
     *   rpm_out_x100 = delta * 1200000 * 100 / (4096 * i_total * dt_ticks)
     *
     *   把 i_total 换成 i_total_x100：
     *   i_total = i_total_x100 / 100
     *   => rpm_out_x100 = delta * 1200000 * 100 * 100
     *                     / (4096 * i_total_x100 * dt_ticks)
     *                   = delta * 1.2e10 / (4096 * i_total_x100 * dt_ticks)
     *
     *   约分：12000000000 / 4096 = 2929687.5，不是整数。
     *   改为先除公因子：1.2e10 = 12e9，4096 = 2^12
     *     12e9 / 2^9 = 23437500，4096/2^9 = 8
     *   => rpm_out_x100 = delta * 23437500 / (8 * i_total_x100 * dt_ticks)
     *
     *   i_total=1080 → i_total_x100=108000：
     *   den = 8 * 108000 * dt = 864000 * dt
     *   delta=-800, dt=100 → num=-800*23437500=-1.875e10
     *                      → rpm_out_x100 = -1.875e10/8.64e7 = -217.0
     *   => -2.17 RPM ✓ 与实测 -2.1 RPM 吻合
     *
     * 溢出核查（全用 int64）：
     *   delta 最大 ±2048（半圈），23437500 → 4.8e13，int64 安全
     *   den 最大 8*108000*10000 = 8.64e9，int64 安全
     * ============================================================ */
    rt = mech_get_rt();

    if ((rt == RT_NULL) || (rt->i_total_x100 <= 0))
    {
        /* 无机械配置：无法换算到输出轴。
         * 不静默返回 0——那样会被误认为"转速是 0"。
         * 标记无效并计数，让上层知道是配置缺失而非真静止。 */
        s_priv.pub.reject_cnt++;
        s_priv.pub.valid = 0U;
        s_priv.last_cnt  = cnt_now;
        s_priv.last_tick = now_tick;
        return;
    }

    {
        int64_t num64 = (int64_t)delta * 23437500LL;
        int64_t den64 = (int64_t)8 * (int64_t)rt->i_total_x100 * (int64_t)dt_ticks;
        int32_t rpm_out_x100;
        int32_t rpm_z0_x10;

        rpm_out_x100 = (int32_t)(num64 / den64);

        /* Z0 侧转速（诊断用，x10）：
         * rpm_z0 = delta / 4096 / (dt/20000) * 60
         * rpm_z0_x10 = delta * 20000 * 60 * 10 / (4096 * dt)
         *            = delta * 12000000 / (4096 * dt)
         * 约分：12000000/4096 = 2929.6875 → 提高精度先算
         *       delta * 12000000 / 4096 / dt
         * 全 int64，无需约分。 */
        {
            int64_t nz = (int64_t)delta * 12000000LL;
            int64_t dz = (int64_t)4096 * (int64_t)dt_ticks;

            rpm_z0_x10 = (int32_t)(nz / dz);
        }

        /* 角速度（0.001 deg/s）：
         * omega_out = rpm_out * 360 / 60 = rpm_out * 6  (deg/s)
         * omega_x1000 = rpm_out * 6 * 1000
         *             = (rpm_out_x100 / 100) * 6000
         *             = rpm_out_x100 * 60
         * 用 rpm_out_x100 直接乘 60，避免再次除法。 */
        {
            int64_t om = (int64_t)rpm_out_x100 * 60LL;

            /* 防溢出：rpm_out_x100 上限约 ±1000，*60 = 6e4，安全。
             * 仍夹一次，防止异常输入导致回绕。 */
            if (om > 2147483647LL)  { om = 2147483647LL; }
            if (om < -2147483648LL) { om = -2147483648LL; }

            s_priv.pub.omega_out_mdeg_s = (int32_t)om;
        }

        s_priv.pub.rpm_out_x100 = rpm_out_x100;
        s_priv.pub.rpm_z0_x10   = rpm_z0_x10;
        s_priv.pub.delta_cnt    = delta;
        s_priv.pub.dt_ticks     = dt_ticks;
        s_priv.pub.dt_ms        = dt_ticks / (MOTOR_SPEED_TICK_HZ / 1000U);

        s_priv.pub.update_cnt++;
        s_priv.pub.valid = 1U;
    }

    /* ---- 推进窗口 ---- */
    s_priv.last_cnt  = cnt_now;
    s_priv.last_tick = now_tick;
}

/* ============================================================
 * 重设基线 / 复位
 * ============================================================ */
void motor_speed_rebase(int32_t cnt_now, uint32_t now_tick)
{
    s_priv.last_cnt  = cnt_now;
    s_priv.last_tick = now_tick;
    s_priv.has_base  = 1U;

    /* 基线变了，上一次的速度值不再可信 —— 置无效，
     * 避免上层在角度失效期间继续用旧速度做闭环。
     *
     * 【2026-11-03 修正】这里不再累加 reject_cnt。
     * rebase 是"角度不可用"的被动结果，不是 dt 异常。
     * 混在一起会让 rejects 每帧暴涨（实测 33317），
     * 掩盖真正的时间基准问题。现在单独计入 rebase_cnt。 */
    s_priv.pub.valid = 0U;
    s_priv.pub.rebase_cnt++;
}

void motor_speed_reset(void)
{
    motor_speed_init();
}

/* ============================================================
 * 查询
 * ============================================================ */
uint8_t motor_speed_get(int32_t *rpm_out_x100)
{
    if (s_priv.pub.valid == 0U)
    {
        if (rpm_out_x100 != RT_NULL) { *rpm_out_x100 = 0; }
        return 0U;
    }
    if (rpm_out_x100 != RT_NULL) { *rpm_out_x100 = s_priv.pub.rpm_out_x100; }
    return 1U;
}

uint8_t motor_speed_get_omega(int32_t *omega_mdeg_s)
{
    if (s_priv.pub.valid == 0U)
    {
        if (omega_mdeg_s != RT_NULL) { *omega_mdeg_s = 0; }
        return 0U;
    }
    if (omega_mdeg_s != RT_NULL) { *omega_mdeg_s = s_priv.pub.omega_out_mdeg_s; }
    return 1U;
}

uint8_t motor_speed_get_z0(int32_t *rpm_z0_x10)
{
    if (s_priv.pub.valid == 0U)
    {
        if (rpm_z0_x10 != RT_NULL) { *rpm_z0_x10 = 0; }
        return 0U;
    }
    if (rpm_z0_x10 != RT_NULL) { *rpm_z0_x10 = s_priv.pub.rpm_z0_x10; }
    return 1U;
}

void motor_speed_get_state(motor_speed_state_t *out)
{
    if (out == RT_NULL) { return; }
    *out = s_priv.pub;
}