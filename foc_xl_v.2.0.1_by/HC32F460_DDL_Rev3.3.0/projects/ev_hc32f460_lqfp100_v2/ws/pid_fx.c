/*=============================================================================
 * pid_fx.c  —  轻量定点 PID 实现（位置式，Q15）
 *=============================================================================*/

#include "pid_fx.h"

static int16_t PidFx_Sat16(int32_t v)
{
    if (v > 32767)  return (int16_t)32767;
    if (v < -32768) return (int16_t)(-32768);
    return (int16_t)v;
}

void PidFx_Init(PidFx_t *p)
{
    if (p == 0) {
        return;
    }

    p->kp_q15        = 0;
    p->ki_tick_q15   = 0;
    p->kd_q15        = 0;
    p->diff_alpha_q15 = 32767;
    p->out_min_q15   = -32768;
    p->out_max_q15   = 32767;
    p->iterm_max_q15 = 0;

    p->integral_q15 = 0;
    p->fb_prev_q15  = 0;
    p->diff_filt_q15 = 0;

    p->inited = 1u;
}

void PidFx_Reset(PidFx_t *p)
{
    if ((p == 0) || (!p->inited)) {
        return;
    }

    p->integral_q15  = 0;
    p->fb_prev_q15   = 0;
    p->diff_filt_q15 = 0;
}

void PidFx_SetGains(PidFx_t *p, int32_t kp_q15, int32_t ki_tick_q15, int32_t kd_q15)
{
    if ((p == 0) || (!p->inited)) {
        return;
    }

    p->kp_q15      = kp_q15;
    p->ki_tick_q15 = ki_tick_q15;
    p->kd_q15      = kd_q15;
}

void PidFx_SetLimits(PidFx_t *p, int16_t out_min_q15, int16_t out_max_q15)
{
    if ((p == 0) || (!p->inited)) {
        return;
    }

    if (out_min_q15 > out_max_q15) {
        int16_t t = out_min_q15;
        out_min_q15 = out_max_q15;
        out_max_q15 = t;
    }

    p->out_min_q15 = out_min_q15;
    p->out_max_q15 = out_max_q15;
}

void PidFx_SetITermMax(PidFx_t *p, int16_t iterm_max_q15)
{
    if ((p == 0) || (!p->inited)) {
        return;
    }

    p->iterm_max_q15 = (iterm_max_q15 > 0) ? (int32_t)iterm_max_q15 : 0;
}

void PidFx_SetDiffAlpha(PidFx_t *p, int16_t alpha_q15)
{
    if ((p == 0) || (!p->inited)) {
        return;
    }

    if (alpha_q15 < 0) {
        alpha_q15 = 0;
    }

    p->diff_alpha_q15 = alpha_q15;
}

int32_t PidFx_GetIntegral(const PidFx_t *p)
{
    if (p == 0) {
        return 0;
    }
    return p->integral_q15;
}

int16_t PidFx_Calc(PidFx_t *p, int16_t ref_q15, int16_t fb_q15)
{
    int32_t err;
    int32_t p_term;
    int32_t d_term;
    int32_t d_raw;
    int32_t i_new;
    int32_t unsat;
    int32_t out;

    if ((p == 0) || (!p->inited)) {
        return 0;
    }

    /* 误差（Q15），int16 范围内 */
    err = (int32_t)ref_q15 - (int32_t)fb_q15;

    /* P 项 */
    p_term = ((int32_t)p->kp_q15 * err);
    p_term = (p_term + 0x4000) >> 15;

    /* D 项：对反馈微分 + 一阶低通 */
    d_raw = (int32_t)p->fb_prev_q15 - (int32_t)fb_q15;
    p->diff_filt_q15 = PidFx_Sat16((int32_t)p->diff_filt_q15 +
                                   (((d_raw - (int32_t)p->diff_filt_q15) *
                                     (int32_t)p->diff_alpha_q15) >> 15));
    d_term = ((int32_t)p->kd_q15 * (int32_t)p->diff_filt_q15);
    d_term = (d_term + 0x4000) >> 15;

    /* I 项：加上本拍增量后先做钳位（积分项限幅优先于输出限幅） */
    i_new = p->integral_q15 +
            (((int32_t)p->ki_tick_q15 * err + 0x4000) >> 15);

    {
        int32_t i_lo = (int32_t)p->out_min_q15;
        int32_t i_hi = (int32_t)p->out_max_q15;

        if (p->iterm_max_q15 > 0) {
            i_lo = -p->iterm_max_q15;
            i_hi =  p->iterm_max_q15;
        }

        if (i_new > i_hi) {
            i_new = i_hi;
        } else if (i_new < i_lo) {
            i_new = i_lo;
        }
    }

    /* 条件积分：输出已饱和且误差仍朝饱和方向，冻结积分 */
    unsat = p_term + i_new + d_term;
    if (((unsat > (int32_t)p->out_max_q15) && (err > 0)) ||
        ((unsat < (int32_t)p->out_min_q15) && (err < 0))) {
        i_new = p->integral_q15;
    }

    p->integral_q15 = i_new;

    out = p_term + i_new + d_term;
    if (out > (int32_t)p->out_max_q15) {
        out = (int32_t)p->out_max_q15;
    } else if (out < (int32_t)p->out_min_q15) {
        out = (int32_t)p->out_min_q15;
    }

    p->fb_prev_q15 = fb_q15;

    return PidFx_Sat16(out);
}
