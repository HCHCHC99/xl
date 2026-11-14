/*=============================================================================
 * cfg_fixed.h  —  定点标幺基准与换算宏
 *
 * 基准（与 foc_fx / pid_fx 配套）：
 *   I_BASE   = 8 A
 *   V_BASE   = 12 / sqrt(3) = 6.928 V（SVPWM 线性区上限）
 *   RPM_BASE = 8000 rpm
 *   T_S      = 50 us
 *
 * FOC_USE_FIXED:
 *   0 = 走原浮点路径（阶段 1 默认，行为不变）
 *   1 = 走定点快路径（阶段 2 起逐段接入）
 *=============================================================================*/

#ifndef CFG_FIXED_H
#define CFG_FIXED_H

#include <stdint.h>

#include "cfg_motor.h"
#include "cfg_sampling.h"

/*---------------------------------------------------------------------------
 * 总开关
 *   FOC_USE_FIXED : 1 = 用 Q15 快路径，0 = 用原浮点路径
 *   FOC_FX_SHADOW : 1 = 浮点路径下额外算一遍 Q15，只统计误差（会加长 ISR）
 *---------------------------------------------------------------------------*/
/* 阶段 5 A/B 结论（mode 4，iq 0->1A，ISR 总耗时，单位 HCLK 周期）：
 *   浮点 @ -O0 : avg 6600 / max 9300
 *   定点 @ -O0 : avg 6600（含 SEG 探针） / max 9700
 *   定点 @ -O3 : avg 4800 / max 6600
 *   关键修复：hw_encoder_get_elec_phase 原来每拍一次 64 位除法
 *   （-O0 下 2000+ 周期），改成缓存的 32 位乘法后，定点 @ -O0
 *   与浮点持平（且还含探针开销），-O3 下比浮点 @ -O0 快约 27%。
 *   默认走定点；浮点路径保留，可直接切回对比 */
#ifndef FOC_USE_FIXED
#define FOC_USE_FIXED            1
#endif
#ifndef FOC_FX_SHADOW
#define FOC_FX_SHADOW            0
#endif

/* FOC_DWT_SEG : 1 = 统计 ISR 分段耗时（采样/变换/控制/输出），供 A/B 定位
 *               会有约 30~50 周期的探针开销，正式对比时置 0 */
#ifndef FOC_DWT_SEG
#define FOC_DWT_SEG              0
#endif

/*---------------------------------------------------------------------------
 * 时间与 CPU
 *---------------------------------------------------------------------------*/
#define FX_CPU_MHZ               200.0f
#define FX_CTRL_HZ               20000u
#define FX_TS_S                  (1.0f / (float)FX_CTRL_HZ)

/*---------------------------------------------------------------------------
 * 标幺基准
 *---------------------------------------------------------------------------*/
#define FX_I_BASE_A              8.0f
#define FX_V_BASE_V              (CFG_SAMPLING_SIM_VBUS_V * 0.57735027f)   /* 12/sqrt(3) */
#define FX_RPM_BASE              8000.0f

/* rpm -> rad/s: 2*pi/60 = 0.10471976 */
#define FX_WE_BASE_RAD_S         (FX_RPM_BASE * 0.10471976f * (float)CFG_MOTOR_POLE_PAIRS)

#define FX_Z_BASE_OHM            (FX_V_BASE_V / FX_I_BASE_A)
#define FX_L_BASE_H              (FX_V_BASE_V / (FX_WE_BASE_RAD_S * FX_I_BASE_A))
#define FX_FLUX_BASE_VS          (FX_V_BASE_V / FX_WE_BASE_RAD_S)

/*---------------------------------------------------------------------------
 * Q15 基本换算
 *---------------------------------------------------------------------------*/
#define FX_Q15_ONE               32767

#define FX_FLOAT_TO_Q15(x)       ((int32_t)((x) * 32768.0f))
#define FX_Q15_TO_FLOAT(q)       ((float)(q) * (1.0f / 32768.0f))

#define FX_CUR_TO_Q15(a)         ((int32_t)((a) / FX_I_BASE_A * 32768.0f))
#define FX_Q15_TO_CUR(q)         ((float)(q) * FX_I_BASE_A / 32768.0f)

#define FX_VOL_TO_Q15(v)         ((int32_t)((v) / FX_V_BASE_V * 32768.0f))
#define FX_Q15_TO_VOL(q)         ((float)(q) * FX_V_BASE_V / 32768.0f)

/* 电流环增益换算（ki 折算为“每拍”增量，dt 已包含在宏里） */
#define FX_KP_CUR_Q15(kp)        ((int32_t)((kp) * FX_I_BASE_A / FX_V_BASE_V * 32768.0f))
#define FX_KI_CUR_TICK_Q15(ki)   ((int32_t)((ki) * FX_I_BASE_A / FX_V_BASE_V * FX_TS_S * 32768.0f))

/* ADC 原始值 -> Q15 电流：i_q15 = (raw - offset) * FX_ADC_I_Q8 >> 8
 * 注意 CFG_SAMPLING_CURRENT_SCALE 为负（低侧采样取反），系数也为负 */
#define FX_ADC_I_Q8              ((int32_t)(CFG_SAMPLING_CURRENT_SCALE / FX_I_BASE_A * 32768.0f * 256.0f))

/*---------------------------------------------------------------------------
 * 角度相位（2^32 = 一个电周期）
 *---------------------------------------------------------------------------*/
#define FX_PHASE_PER_COUNT       ((int32_t)(((int64_t)CFG_MOTOR_POLE_PAIRS * 4294967296LL) / (int64_t)CFG_ENCODER_CPR))
#define FX_PHASE_PER_RAD         (4294967296.0f / 6.28318531f)

#endif /* CFG_FIXED_H */
