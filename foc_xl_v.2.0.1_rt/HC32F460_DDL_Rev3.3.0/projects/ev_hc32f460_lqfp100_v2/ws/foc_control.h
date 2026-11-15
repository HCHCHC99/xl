/*=============================================================================
 * foc_control.h  —  L5 控制层（20 kHz EOCA 中断宿主）
 *=============================================================================*/

#ifndef FOC_CONTROL_H
#define FOC_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "cfg_motor.h"
#define FOC_CONTROL_POLE_PAIRS   CFG_MOTOR_POLE_PAIRS

#define FOC_CONTROL_IRQ_PRIO     0u

void Foc_Control_Init(void);
void Foc_Control_Enable(void);
void Foc_Control_Disable(void);
uint8_t Foc_Control_IsEnabled(void);
void Foc_Control_ResetStats(void);
void Foc_Control_Step(void);

/*=============================================================================
 * Keil Watch 观测量
 *=============================================================================*/
extern volatile uint32_t g_dbg_cnt_at_isr;
extern volatile uint32_t g_fc_isr_cnt;
extern volatile uint32_t g_fc_isr_dt_max_us;
extern volatile uint32_t g_fc_isr_dt_min_us;
extern volatile uint32_t g_fc_isr_dt_last_us;

/* DWT 周期计数：ISR 软件计算耗时（不含异常出入栈）
 *   HCLK=200MHz 时 1 count = 5ns
 *   avg 每 1024 拍更新一次，min/max 为上电以来极值 */
extern volatile uint32_t g_dwt_isr_last_cyc;
extern volatile uint32_t g_dwt_isr_min_cyc;
extern volatile uint32_t g_dwt_isr_max_cyc;
extern volatile uint32_t g_dwt_isr_avg_cyc;
extern volatile uint32_t g_dwt_isr_win_cnt;

extern volatile int32_t  g_fc_enc_count;
extern volatile int32_t  g_fc_enc_delta;
extern volatile float    g_fc_enc_mech_deg;
extern volatile float    g_fc_enc_elec_deg;
extern volatile float    g_fc_enc_speed_rpm;

extern volatile float    g_fc_id;
extern volatile float    g_fc_iq;

extern volatile float    g_fc_i_total;
extern volatile float    g_fc_iu;
extern volatile float    g_fc_iv;
extern volatile float    g_fc_iw;
extern volatile float    g_fc_vbus;

extern volatile uint32_t g_fc_sample_seq;

/*---------------------------------------------------------------------------
 * Mode 3 对齐期间标志
 *   1 = ISR 不碰 PWM，由主循环 DoAlignBlocking 完全控制输出
 *   对齐期间 ISR 仍做编码器/电流采样，只是跳过 PWM 输出
 *---------------------------------------------------------------------------*/
extern volatile uint8_t  g_align_active;

#ifdef __cplusplus
}
#endif

#endif /* FOC_CONTROL_H */
