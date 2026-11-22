/**
 *******************************************************************************
 * @file  hw_encoder.h
 * @brief ABZ quadrature encoder driver (TMRA1 hardware quadrature count).
 *
 *        引脚：A = PA8/TIMA1_CLKA, B = PA9/TIMA1_CLKB, Z = PA10/EXTINT_CH10
 *        Encoder: 1024 lines x 4 = 4096 counts / rev（CPR 从 cfg_motor.h 取）
 *******************************************************************************
 */

#ifndef HW_ENCODER_H
#define HW_ENCODER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void    hw_encoder_init(void);
void    hw_encoder_update_count_from_isr(void);
void    hw_encoder_update(void);

/* 机械角 [0, 2π) */
float   hw_encoder_get_mech_angle_rad(void);

/* 电角度 [0, 2π)，已扣 g_enc_elec_offset 并按 g_enc_elec_dir 翻转 */
float   hw_encoder_get_elec_angle_rad(uint8_t pole_pairs);

int32_t hw_encoder_get_count(void);
float   hw_encoder_get_angle_deg(void);
float   hw_encoder_get_speed_rpm(void);
int8_t  hw_encoder_get_direction(void);
uint32_t hw_encoder_get_rev_count(void);

/*---------------------------------------------------------------------------
 * 可观测变量（供 Watch / msh t_enc）
 *---------------------------------------------------------------------------*/
extern volatile int32_t  g_enc_count;
extern volatile uint8_t  g_enc_dbg_print;
extern volatile float    g_enc_count_f;
extern volatile float    g_enc_angle_deg;
extern volatile float    g_enc_speed_rpm;
extern volatile int8_t   g_enc_dir;
extern volatile uint32_t g_enc_rev;

/* 电角度零点偏移（counts，由 mode 3 对齐写入） */
extern volatile int32_t  g_enc_elec_offset;

/* 电角度方向：+1 = 计数增即电角度增；-1 = 计数增即电角度减 */
extern volatile int8_t   g_enc_elec_dir;

#ifdef __cplusplus
}
#endif

#endif /* HW_ENCODER_H */
