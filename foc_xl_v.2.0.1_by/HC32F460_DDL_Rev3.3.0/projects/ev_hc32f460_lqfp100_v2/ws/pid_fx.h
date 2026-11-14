/*=============================================================================
 * pid_fx.h  —  轻量定点 PID（位置式，Q15 输入输出）
 *
 * 与浮点 pid_ctrl 的区别（有意简化，便于定点实现）：
 *   - 只做位置式，不实现增量式
 *   - 微分对反馈（微分先行），带一阶低通
 *   - 抗饱和：积分钳位 + 条件积分（输出饱和且误差同向时冻结积分）
 *   - dt 不参与运算：ki 参数由调用者折算成“每拍”增量（见 cfg_fixed.h
 *     的 FX_KI_CUR_TICK_Q15）
 *
 * 单位约定：
 *   ref/fb/输出都是 Q15 标幺值；kp/ki_tick/kd 也是 Q15。
 *=============================================================================*/

#ifndef PID_FX_H
#define PID_FX_H

#include <stdint.h>

typedef struct {
    /* ---- 参数 ---- */
    int32_t kp_q15;
    int32_t ki_tick_q15;     /* 每拍积分增量系数（已含 dt） */
    int32_t kd_q15;
    int16_t diff_alpha_q15;  /* 不完全微分低通，32767 = 不滤波 */
    int16_t out_min_q15;
    int16_t out_max_q15;
    int32_t iterm_max_q15;   /* 积分项单独限幅（正数），0 = 跟随输出限幅 */

    /* ---- 状态 ---- */
    int32_t integral_q15;    /* 积分项输出，Q15，已钳位 */
    int16_t fb_prev_q15;
    int16_t diff_filt_q15;
    uint8_t inited;
} PidFx_t;

void PidFx_Init(PidFx_t *p);
void PidFx_Reset(PidFx_t *p);

void PidFx_SetGains(PidFx_t *p, int32_t kp_q15, int32_t ki_tick_q15, int32_t kd_q15);
void PidFx_SetLimits(PidFx_t *p, int16_t out_min_q15, int16_t out_max_q15);
void PidFx_SetITermMax(PidFx_t *p, int16_t iterm_max_q15);
void PidFx_SetDiffAlpha(PidFx_t *p, int16_t alpha_q15);

/* 计算：返回限幅后的 Q15 输出 */
int16_t PidFx_Calc(PidFx_t *p, int16_t ref_q15, int16_t fb_q15);

/* 观测：当前积分项输出（Q15） */
int32_t PidFx_GetIntegral(const PidFx_t *p);

#endif /* PID_FX_H */
