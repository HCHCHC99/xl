/**
 *******************************************************************************
 * @file  foc_current_loop.h
 * @brief L6.5 电流闭环层（力矩模式，有感）
 *
 *        数据流：ref 斜坡 -> d/q PID -> dq 解耦前馈 -> 限幅 -> vd/vq/theta_e
 *        契约：只做"给定→斜坡→PID→电压"，不碰 PWM/模式/保护
 *******************************************************************************
 */

#ifndef FOC_CURRENT_LOOP_H
#define FOC_CURRENT_LOOP_H

#include <stdbool.h>
#include <stdint.h>

#include "pid_ctrl.h"   /* PID_AW_* 抗饱和措施位掩码 */

#ifdef __cplusplus
extern "C" {
#endif

/*=============================================================================
 * 一、可调参数初值
 *=============================================================================*/

/* ref 斜坡：每拍 ref 最大变化量（A/拍）：
 *   20kHz 下 0.00005A/拍 = 1A/s；0 关闭 */
#define CL_REF_SLEW_INIT      0.00005f

/* 输出斜率限制：每拍 vq 最大变化量（V/拍）；0 关闭 */
#define CL_VQ_SLEW_INIT       0.0f

/* 进入预热拍数：4200 拍 = 210ms */
#define CL_WARMUP_INIT        4200u

/* 目标电流 clamp 上限（A） */
#define CL_IREF_MAX_INIT      50.0f

/* 物理电压上限（V） */
#define CL_VMAX_INIT          3.5f

/* PID 初值（3505-KV650，Rs=0.1Ω，Ls=42.3µH，20kHz） */
#define CL_KP_INIT            0.27f
#define CL_KI_INIT            630.0f
#define CL_KD_INIT            0.0f
#define CL_DIFF_ALPHA_INIT    0.2f
#define CL_BACK_GAIN_INIT     0.0f

#define CL_AW_MASK_INIT       (PID_AW_CLAMP | PID_AW_BACK_CALC)
#define CL_AW_COND_THR_INIT   (0.0f)

/* 积分项（ki*integral）电压限幅，V */
#define CL_ITERM_MAX_INIT     3.5f

/* id/iq 反馈 EMA 低通系数（1 = 关闭） */
#define CL_FB_ALPHA_INIT      0.3f

/* dq 解耦 + 反电动势前馈参数（mode 4 默认关闭） */
#define CL_FF_RS_OHM_INIT     0.1f
#define CL_FF_LS_UH_INIT      42.3f
#define CL_FF_FLUX_VS_INIT    0.00084f

/*=============================================================================
 * 二、可调参数变量（Watch 可改）
 *=============================================================================*/
extern volatile float    g_cl_ref_slew;
extern volatile float    g_cl_vq_slew;
extern volatile float    g_cl_fb_alpha;
extern volatile float    g_cl_iterm_max_watch;
extern volatile float    g_cl_back_gain;
extern volatile uint32_t g_cl_aw_mask;
extern volatile float    g_cl_aw_cond_thr;
extern volatile uint16_t g_cl_warmup_ticks;
extern volatile float    g_cl_iref_max;
extern volatile float    g_cl_vmax_watch;

extern volatile uint8_t  g_cl_ff_enable;
extern volatile float    g_cl_ff_rs_ohm;
extern volatile float    g_cl_ff_ls_uh;
extern volatile float    g_cl_ff_flux_vs;
extern volatile float    g_cl_we_rad_s;
extern volatile float    g_cl_vd_ff;
extern volatile float    g_cl_vq_ff;

/*=============================================================================
 * 三、受限原因枚举
 *=============================================================================*/
typedef enum {
    CL_LIMIT_NONE     = 0,
    CL_LIMIT_SELF     = 1,
    CL_LIMIT_VOLTAGE  = 2,
    CL_LIMIT_PROTECT  = 3,
} cl_limit_reason_t;

/*=============================================================================
 * 四、API
 *=============================================================================*/
void Foc_CurrentLoop_Init(void);
void Foc_CurrentLoop_Reset(void);
void Foc_CurrentLoop_SetRef(float id_ref_a, float iq_ref_a);
void Foc_CurrentLoop_SetRefDirect(float id_ref_a, float iq_ref_a);

void Foc_CurrentLoop_SetGains(float kp, float ki, float kd);
void Foc_CurrentLoop_SetITermMax(float v);
void Foc_CurrentLoop_SetAntiWindup(uint32_t mask, float cond_thr);
void Foc_CurrentLoop_SetBackGain(float v);
void Foc_CurrentLoop_SetFF(uint8_t en);
void Foc_CurrentLoop_SetVmax(float vmax_v);

void Foc_CurrentLoop_ApplyParams(float kp, float ki, float kd,
                                 float vmax_v, float iterm_max_v, float fb_alpha);

void Foc_CurrentLoop_Step(float *vd, float *vq, float *theta_e);
void Foc_CurrentLoop_OutputZeroVector(float *vd, float *vq, float *theta_e);

bool Foc_CurrentLoop_IsReady(void);
bool Foc_CurrentLoop_IsActive(void);
bool Foc_CurrentLoop_IsLimited(void);
cl_limit_reason_t Foc_CurrentLoop_GetLimitReason(void);
uint16_t Foc_CurrentLoop_GetSatCnt(void);
float Foc_CurrentLoop_GetVmagRaw(void);
float Foc_CurrentLoop_GetVmax(void);
float Foc_CurrentLoop_GetIdRefRamped(void);
float Foc_CurrentLoop_GetIqRefRamped(void);

#ifdef __cplusplus
}
#endif

#endif /* FOC_CURRENT_LOOP_H */
