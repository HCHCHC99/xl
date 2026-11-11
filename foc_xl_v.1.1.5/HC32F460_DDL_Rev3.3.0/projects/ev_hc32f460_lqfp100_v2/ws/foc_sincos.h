/*=============================================================================
 * foc_sincos.h  —  Sin/Cos 查表模块（1024 点 + 线性插值，硬件无关，纯浮点）
 *
 *   - 表在 Foc_SinCos_Init() 里建一次（只在此时用 sinf）
 *   - 运行期（含 20kHz ISR）不再调用 sinf / cosf
 *   - 误差 < 0.1% 满量程；表占 4KB RAM（1024 float）
 *   - 角度单位 rad，任意值：内部单次除法折叠到 [0,2π)，恒定时间
 *=============================================================================*/

#ifndef FOC_SINCOS_H
#define FOC_SINCOS_H

#include <stdint.h>

#define FOC_SINCOS_LUT_BITS   10u
#define FOC_SINCOS_LUT_SIZE   (1u << FOC_SINCOS_LUT_BITS)   /* 1024 */
#define FOC_SINCOS_LUT_MASK   (FOC_SINCOS_LUT_SIZE - 1u)

/* 上电调用一次，建 sin 表（重复调用安全） */
void Foc_SinCos_Init(void);

/* 查表 sin/cos，theta 单位 rad，任意值（内部折叠到 [0,2π)） */
float Foc_Sin(float theta);
float Foc_Cos(float theta);

#endif /* FOC_SINCOS_H */
