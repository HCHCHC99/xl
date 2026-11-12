/*=============================================================================
 * encoder_calib.h  —  编码器电角度标定层（两点法强拖对齐，1ms 中断非阻塞）
 *
 * 职责：
 *   - mode 3 单次正式校准：ZERO(零矢量采零偏) -> BETA(90°) -> ALPHA(0°)
 *     -> 判据 -> 写 offset/dir/g_motor_is_aligned
 *   - mode 5 扫角测量：0/90/180/270 四个起始角各跑一遍两点对齐，只记录
 *
 * 对齐期间 g_align_active=1，20kHz ISR 不碰 PWM；
 *   - ZERO 阶段零偏采样在 20kHz ISR 里调 EncoderCalib_Task20k()
 *   - BETA/ALPHA 保持计时在 1ms 里调 EncoderCalib_Task1ms()
 * ISR 内不打印，只写观测量与事件；主循环消费 g_align_evt 打印
 *=============================================================================*/

#ifndef ENCODER_CALIB_H
#define ENCODER_CALIB_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 单次正式校准（mode 3）---- */
void    EncoderCalib_StartAlign(void);
uint8_t EncoderCalib_IsAlignOk(void);

/* ---- 扫角测量（mode 5）---- */
void    EncoderCalib_StartSweep(void);
uint8_t EncoderCalib_IsSweepDone(void);

/* ---- 一圈计数测量（mode 6）---- */
void    EncoderCalib_StartRevMeasure(void);
uint8_t EncoderCalib_IsRevMeasureDone(void);

/* ---- 斜坡对齐（mode 7）：0°->90° 慢转，再 90°->0° 慢转 ---- */
void    EncoderCalib_StartAlign7(void);
uint8_t EncoderCalib_IsAlign7Done(void);

/* ---- 共用 ---- */
void    EncoderCalib_Task20k(void);   /* 20kHz ISR 调用：ZERO 阶段零偏采样 */
void    EncoderCalib_Task1ms(void);   /* 1ms 中断调用：BETA/ALPHA 保持计时 */
uint8_t EncoderCalib_IsDone(void);
uint8_t EncoderCalib_ConsumeEvent(void); /* 主循环消费事件，返回后清 0 */

/* 电角度对齐状态：1 = 已对齐, 0 = 未对齐（仅 mode 3 成功时写 1） */
extern volatile uint8_t g_motor_is_aligned;

/* 事件码：0 NONE / 1 ZERO_DONE / 2 BETA_DONE / 3 DONE_OK / 4 DONE_FAIL / 5 OC */
#define EC_EVT_NONE       0u
#define EC_EVT_ZERO_DONE  1u
#define EC_EVT_BETA_DONE  2u
#define EC_EVT_DONE_OK    3u
#define EC_EVT_DONE_FAIL  4u
#define EC_EVT_OC         5u

/*---------------------------------------------------------------------------
 * 调试可写/可观测量（Keil Watch）
 *---------------------------------------------------------------------------*/
extern volatile float    g_align_volt;       /* 强拖电压幅值 V，默认 0.6 */
extern volatile float    g_align_theta;      /* mode 5 扫角用；mode 3 固定 90° */
extern volatile uint32_t g_align_hold_ms;    /* BETA/ALPHA 保持时间 ms，默认 200 */
extern volatile uint32_t g_align_ramp_ms;    /* mode7 斜坡时间 ms，默认 500 */
extern volatile uint8_t  g_a7_done;          /* mode7 完成标志 */

extern volatile int32_t  g_align_zero_u_ma;  /* ZERO 阶段 U 相残余零位（mA） */
extern volatile int32_t  g_align_zero_v_ma;
extern volatile int32_t  g_align_zero_w_ma;

extern volatile int32_t  g_align_beta_hw;    /* 累计 beta_hw */
extern volatile int32_t  g_align_alpha_hw;   /* 累计 alpha_hw */
extern volatile int32_t  g_align_moved;      /* alpha - beta */
extern volatile int32_t  g_align_offset;     /* 圈内 offset（0~CPR-1） */

extern volatile uint8_t  g_align_evt;        /* 事件码，主循环消费 */

/* 扫角结果：4 组，对应起始角 0/90/180/270 */
extern volatile int32_t  g_sweep_beta[4];
extern volatile int32_t  g_sweep_alpha[4];
extern volatile int32_t  g_sweep_moved[4];
extern volatile int32_t  g_sweep_theta[4];
extern volatile uint8_t  g_sweep_idx;
extern volatile uint8_t  g_sweep_done;

/* 每段三相电流平均值（mA）
 *   [idx][0..2] = BETA 段 iu, iv, iw
 *   [idx][3..5] = ALPHA 段 iu, iv, iw */
extern volatile int32_t  g_sweep_i_ma[4][6];

/* mode6 一圈计数测量：对齐1s -> 强拖 pp_cfg*360° -> 对齐1s
 *   delta    = 结束计数 - 开始计数（累加计数，可为负）
 *   pp_meas  = CPR * pp_cfg / |delta|（实际极对数估计）
 *   slip=1   = 每个电周期计数增量差异过大，疑似失步，结果不可信 */
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
