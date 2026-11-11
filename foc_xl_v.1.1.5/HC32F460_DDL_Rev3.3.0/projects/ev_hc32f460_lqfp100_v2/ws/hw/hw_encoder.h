/**
 *******************************************************************************
 * @file  hw_encoder.h
 * @brief ABZ quadrature encoder driver (TIMERA_1 hardware quadrature count).
 *
 *        引脚：A = TIMA1_CLKA, B = TIMA1_CLKB, Z = EXTINT 上升沿
 *        Encoder: 1024 lines x 4 = 4096 counts / rev（CPR 从 cfg_motor.h 取）
 *
 * 职责划分：
 *   hw_gpio_init_all()            配置 A/B/Z 引脚（GPIO 模式/功能号）
 *   hw_encoder_init()             配置 TMRA_1 正交计数 + Z 中断 + EXTINT 使能
 *
 * 调用时机：
 *   hw_encoder_update_count_from_isr()   20 kHz 控制 ISR 里每拍调用
 *   hw_encoder_update()                  主循环里低频调用（10 ms 窗口）
 *
 * 电角度方向：
 *   g_enc_elec_dir 由 mode 3 对齐时按 moved 符号设置：
 *     +1 = 编码器计数增对应电角度增大
 *     -1 = 编码器计数增对应电角度减小
 *   hw_encoder_get_elec_angle_rad 内部按此方向翻转，保证 theta_e 与
 *   电机实际电角度一致（d/q 解耦正确）。
 *   机械角与转速也按同一 dir 翻转，使三者正方向与 FOC 正向（SVPWM 矢量
 *   角增）一致；g_enc_count / ch15 原始计数保持编码器接线方向不变。
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
 * 可观测变量（供 Watch / J-Scope）
 *---------------------------------------------------------------------------*/
extern volatile int32_t  g_enc_count;
extern volatile uint8_t  g_enc_dbg_print;
extern volatile float    g_enc_count_f;
extern volatile float    g_enc_angle_deg;
extern volatile float    g_enc_speed_rpm;
extern volatile int8_t   g_enc_dir;
extern volatile uint32_t g_enc_rev;

/* 电角度零点偏移（counts，由 DoAlignBlocking 写入） */
extern volatile int32_t  g_enc_elec_offset;

/* 电角度方向：+1 = 计数增即电角度增；-1 = 计数增即电角度减
 *   由 DoAlignBlocking 按 moved 符号设置 */
extern volatile int8_t   g_enc_elec_dir;

#ifdef __cplusplus
}
#endif

#endif /* HW_ENCODER_H */
