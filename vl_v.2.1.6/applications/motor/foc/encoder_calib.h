/**
 *******************************************************************************
 * @file  encoder_calib.h
 * @brief 编码器电角度标定层（两点法强拖对齐）
 *
 *        mode 3：ZERO(零矢量等待+零偏采样) -> BETA(90°) -> ALPHA(0°)
 *                -> 判据 -> 写 offset/dir/g_motor_is_aligned
 *        mode 5：扫角测量，0/90/180/270 四个起始角各跑一遍两点对齐
 *        mode 6：对齐 -> 强拖 pp×360° -> 再对齐，测编码器计数
 *        mode 7：0°->90° 慢转，再 90°->0° 慢转
 *
 *        对齐期间 g_align_active=1，20kHz ISR 不碰 PWM：
 *          ZERO 阶段计时在 20kHz 里（EncoderCalib_Task20k）
 *          BETA/ALPHA 保持计时在 1ms 里（EncoderCalib_Task1ms）
 *******************************************************************************
 */

#ifndef ENCODER_CALIB_H
#define ENCODER_CALIB_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void    EncoderCalib_StartAlign(void);
uint8_t EncoderCalib_IsAlignOk(void);

/* H1-A：单点 α 对齐（只施加 0° 矢量 200ms，重建 offset；不改 dir）。
 * 前置：本上电周期已做过 mode 3，g_enc_elec_dir 已知。 */
void    EncoderCalib_StartOnePoint(void);

void    EncoderCalib_StartSweep(void);
uint8_t EncoderCalib_IsSweepDone(void);

void    EncoderCalib_StartRevMeasure(void);
uint8_t EncoderCalib_IsRevMeasureDone(void);

void    EncoderCalib_StartAlign7(void);
uint8_t EncoderCalib_IsAlign7Done(void);

void    EncoderCalib_Task20k(void);
void    EncoderCalib_Task1ms(void);
uint8_t EncoderCalib_IsDone(void);
uint8_t EncoderCalib_IsBusy(void);
void    EncoderCalib_Abort(void);
uint8_t EncoderCalib_ConsumeEvent(void);

/* 电角度对齐状态：1 = 已对齐 */
extern volatile uint8_t g_motor_is_aligned;

/* 事件码 */
#define EC_EVT_NONE       0u
#define EC_EVT_ZERO_DONE  1u
#define EC_EVT_BETA_DONE  2u
#define EC_EVT_DONE_OK    3u
#define EC_EVT_DONE_FAIL  4u
#define EC_EVT_OC         5u

/*---------------------------------------------------------------------------
 * 调试可写/可观测量
 *---------------------------------------------------------------------------*/
extern volatile float    g_align_volt;
extern volatile float    g_align_theta;
extern volatile uint32_t g_align_hold_ms;
extern volatile uint32_t g_align_ramp_ms;
extern volatile uint8_t  g_a7_done;

extern volatile int32_t  g_align_beta_hw;
extern volatile int32_t  g_align_alpha_hw;
extern volatile int32_t  g_align_moved;
extern volatile int32_t  g_align_offset;

extern volatile uint8_t  g_align_evt;

/* 扫角结果：4 组，对应起始角 0/90/180/270 */
extern volatile int32_t  g_sweep_beta[4];
extern volatile int32_t  g_sweep_alpha[4];
extern volatile int32_t  g_sweep_moved[4];
extern volatile int32_t  g_sweep_theta[4];
extern volatile uint8_t  g_sweep_idx;
extern volatile uint8_t  g_sweep_done;

/* 每段三相电流平均值（mA）：[idx][0..2]=BETA，[idx][3..5]=ALPHA */
extern volatile int32_t  g_sweep_i_ma[4][6];

/* mode6 一圈计数测量 */
extern volatile float    g_rev_volt;
extern volatile float    g_rev_freq_hz;
extern volatile uint32_t g_rev_align_ms;
extern volatile uint32_t g_rev_ramp_ms;
extern volatile int32_t  g_rev_start_cnt;
extern volatile int32_t  g_rev_end_cnt;
extern volatile int32_t  g_rev_delta;
extern volatile float    g_rev_pp_meas;
extern volatile int32_t  g_rev_cyc_min;
extern volatile int32_t  g_rev_cyc_max;
extern volatile uint8_t  g_rev_done;
extern volatile uint8_t  g_rev_slip;

#ifdef __cplusplus
}
#endif

#endif /* ENCODER_CALIB_H */
