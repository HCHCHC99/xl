/**
 * @file    motor_angle.c
 * @brief   电机轴虚拟角度实现（见 motor_angle.h 的设计说明）
 */

#include "motor_angle.h"

/* 【2026-11-10 彩标 P1】解算失败=FAULT（安全事件）；降级运行=WARN；解算成功=OK */
#include "../log/log_color.h"
#include "abs_angle.h"
#include "bsp_dev_kth7812.h"
#include "../sys_param/sys_param.h"

#include <string.h>

/* ============================================================
 * 私有状态
 * ============================================================ */
static motor_angle_state_t s_st;

/* 过圈判据：|delta| >= 此值视为风险
 * 物理含义：单周期转过半圈。1ms 下对应 30000 RPM，
 * 远超实际 3500 RPM，一旦命中必然是采样周期被拉长或数据异常。 */
#define WRAP_RISK_THRESHOLD     (2047)

/* 环绕归一 */
static inline int32_t wrap_delta(int32_t d)
{
    if (d >  2048) { d -= MOTOR_ANGLE_ENC_FULL; }
    if (d < -2048) { d += MOTOR_ANGLE_ENC_FULL; }
    return d;
}

static int32_t iabs32(int32_t v)
{
    return (v < 0) ? -v : v;
}

/* ============================================================
 * 初始化
 * ============================================================ */
void motor_angle_init(void)
{
    memset(&s_st, 0, sizeof(s_st));
    s_st.last_raw = -1;         /* -1 = 未采过，首次 update 只建基线 */
    s_st.inited   = 0U;
}

/* ============================================================
 * 上电解算：三颗游标 → 绝对角度基线
 * ============================================================ */
motor_angle_err_t motor_angle_solve(void)
{
    abs_angle_err_t aerr = ABS_ANGLE_OK;
    abs_angle_state_t ast;
    uint16_t zero[3];
    uint16_t raw0;
    int32_t  cnt;

    /* 1. 走游标解算（三颗编码器）*/
    aerr = abs_angle_solve(&aerr);
    if (aerr != ABS_ANGLE_OK)
    {
        LOG_FAULT("[MAngle] solve FAIL: %s\n", abs_angle_err_name(aerr));

        /* ============================================================
         * 【2026-11-03 调试期降级策略】解算失败也建立基线，角度按 0°
         * ------------------------------------------------------------
         * 用户明确要求：调试期不能因为上电解算不通就卡死后续所有工作。
         *
         * 做法：只读 Z0 一颗，把它当作 0° 基准。
         *   - angle_mdeg = 0，cnt_accum = 0
         *   - last_raw  = 当前 Z0 原始值（增量基线）
         *   - inited    = 1（让 update/speed 能跑起来）
         *
         * 这样得到的是「相对角度」：从当前位置起算的增量，
         * 绝对值不正确，但增量、转速、方向、限位趋势全部可用。
         *
         * 【必须明确标记的状态】
         *   degenerate = 1，并且绝对位置不可信。
         *   t_mangle 会显示 ZERO-BASE 警告，避免被误当成真实绝对角。
         *
         * 【落地期方案 A】
         *   改为：置故障码 + 禁止运动指令，不允许在未知位置下动作。
         *   当前是调试期妥协，不是最终设计。
         * ============================================================ */
        raw0 = KTH78_ReadAngle((KTH7812_DEV_DEF)0);
        if (raw0 == KTH7812_ANGLE_INVALID)
        {
            LOG_ERR("[MAngle] Z0 also unreadable, cannot build any baseline\n");
            return MOTOR_ANGLE_ERR_NOT_INIT;
        }

        s_st.cnt_accum  = 0;
        s_st.angle_mdeg = 0;
        s_st.last_raw   = (int32_t)raw0;
        s_st.inited     = 1U;
        s_st.degenerate = 1U;       /* 零基点，非真实绝对角 */
        s_st.z0_raw0    = (int32_t)raw0;
        s_st.update_cnt++;

        LOG_WARN("[MAngle] DEGRADED: base=0 deg, Z0 raw=%u\n",
                   (unsigned)raw0);
        rt_kprintf("         absolute position UNKNOWN, increments still work\n");
        rt_kprintf("         (debug mode only, fix solve before release)\n");
        return MOTOR_ANGLE_ERR_NOT_INIT;   /* 仍返回错误，让上层知道不完美 */
    }

    /* 2. 取解算出的 Z0 累计计数 */
    abs_angle_get_state(&ast);
    cnt = ast.cnt_accum;

    /* 3. 若已做过零点校准，把零点减掉
     *    零点语义：回转臂在机械零位时，三颗各自的读数。
     *    减掉后 cnt 的零点对应机械零位，符合"卸粮筒角度"的直觉。 */
    if (sys_param_get_enc_zero(zero) != 0U)
    {
        /* cnt 减掉 Z0 零点后，角度零点就对齐机械零位。
         * 注意 cnt 是多圈量（k*4096 + raw），零点只应作用于 raw 部分，
         * 但 abs_angle 的 cnt 已经是连续量，直接整体平移即等价于
         * "把机械零位定义为 0 度"，这正是需求要的语义。 */
        cnt -= (int32_t)zero[0];
    }

    s_st.cnt_accum  = cnt;
    s_st.angle_mdeg = motor_angle_cnt_to_mdeg(cnt);
    s_st.last_raw   = ast.raw_z0;      /* 用解算时的 raw 作增量基线 */
    s_st.inited     = 1U;
    s_st.degenerate = 0U;              /* 真实绝对角 */
    s_st.z0_raw0    = ast.raw_z0;
    s_st.update_cnt++;

    LOG_OK("[MAngle] solved: cnt=%d angle=%d.%03d deg\n",
               (int)cnt,
               (int)(s_st.angle_mdeg / 1000),
               (int)((s_st.angle_mdeg % 1000 + 1000) % 1000));

    return MOTOR_ANGLE_OK;
}

/* ============================================================
 * 周期增量更新（只读 Z0）
 * ============================================================ */
motor_angle_err_t motor_angle_update(void)
{
    uint16_t raw;
    int32_t  raw_i;
    int32_t  delta;

    /* 【2026-11-02】每次调用都计数，用于在 t_mangle 里暴露"回调在跑但全部失败"
     * 这种状态以前完全看不出来：t_mangle 只显示 updates=0，
     * 分不清是定时器没启动、还是每次都被 NOT_INIT 早退。 */
    s_st.call_cnt++;

    if (s_st.inited == 0U)
    {
        s_st.not_init_cnt++;
        s_st.last_err = (uint32_t)MOTOR_ANGLE_ERR_NOT_INIT;
        return MOTOR_ANGLE_ERR_NOT_INIT;
    }

    /* ---- 只读 Z0 一颗 ---- */
    raw = KTH78_ReadAngle((KTH7812_DEV_DEF)0);
    if (raw == KTH7812_ANGLE_INVALID)
    {
        s_st.err_cnt++;
        s_st.last_err = (uint32_t)MOTOR_ANGLE_ERR_ENC_READ;
        return MOTOR_ANGLE_ERR_ENC_READ;
    }

    raw_i = (int32_t)raw;

    /* ---- 首次：只建基线 ---- */
    if (s_st.last_raw < 0)
    {
        s_st.last_raw = raw_i;
        s_st.update_cnt++;
        return MOTOR_ANGLE_OK;
    }

    /* ---- 算增量 ---- */
    delta = wrap_delta(raw_i - s_st.last_raw);

    /* ---- 过圈保护 ----
     * 不累加、不更新 last_raw（保持旧基线，下次再试），
     * 只计风险次数。这样单次异常不会污染角度。 */
    if ((delta >= WRAP_RISK_THRESHOLD) || (delta <= -WRAP_RISK_THRESHOLD))
    {
        s_st.wrap_risk_cnt++;
        s_st.last_err = (uint32_t)MOTOR_ANGLE_ERR_WRAP_RISK;
        return MOTOR_ANGLE_ERR_WRAP_RISK;
    }

    /* ---- 累加 ---- */
    if (delta != 0)
    {
        int32_t absd = iabs32(delta);
        if ((uint32_t)absd > s_st.max_delta)
        {
            s_st.max_delta = (uint32_t)absd;
        }

        s_st.cnt_accum  += delta;
        s_st.angle_mdeg  = motor_angle_cnt_to_mdeg(s_st.cnt_accum);
        s_st.last_raw    = raw_i;
    }
    else
    {
        /* delta==0：电机静止或采样间隔小于编码器分辨率。
         * 计数，用于区分"真的没动"和"读取路径被跳过"。 */
        s_st.still_cnt++;
    }

    s_st.update_cnt++;
    s_st.last_err = (uint32_t)MOTOR_ANGLE_OK;
    return MOTOR_ANGLE_OK;
}

/* ============================================================
 * 查询
 * ============================================================ */
motor_angle_err_t motor_angle_get(int32_t *angle_mdeg)
{
    if (s_st.inited == 0U)
    {
        return MOTOR_ANGLE_ERR_NOT_INIT;
    }
    if (angle_mdeg != RT_NULL)
    {
        *angle_mdeg = s_st.angle_mdeg;
    }
    return MOTOR_ANGLE_OK;
}

int32_t motor_angle_get_cnt(void)
{
    return s_st.cnt_accum;
}

void motor_angle_get_state(motor_angle_state_t *out)
{
    if (out == RT_NULL) { return; }
    *out = s_st;
}