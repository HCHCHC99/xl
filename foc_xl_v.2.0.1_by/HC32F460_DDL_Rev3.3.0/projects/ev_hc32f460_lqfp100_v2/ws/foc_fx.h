/*=============================================================================
 * foc_fx.h  —  定点运算基础库（Q15 / 相位 / 查表 sin-cos）
 *
 * 本阶段（阶段 1）只提供库，不接入控制路径。
 *
 * 约定：
 *   - 电流、电压、占空比偏移都用 Q15（int16，±1.0 pu）
 *   - 相位用 uint32，满量程 2^32 = 一个电周期
 *   - Q15 乘法统一四舍五入后饱和到 int16
 *=============================================================================*/

#ifndef FOC_FX_H
#define FOC_FX_H

#include <stdint.h>

#include "cfg_fixed.h"

#define FX_SINCOS_LUT_BITS       10u
#define FX_SINCOS_LUT_SIZE       (1u << FX_SINCOS_LUT_BITS)   /* 1024 */
#define FX_SINCOS_LUT_MASK       (FX_SINCOS_LUT_SIZE - 1u)

/*---------------------------------------------------------------------------
 * 基础运算
 *---------------------------------------------------------------------------*/

/* 饱和到 int16 */
static inline int16_t Fx_Sat16(int32_t v)
{
    if (v > 32767)  return (int16_t)32767;
    if (v < -32768) return (int16_t)(-32768);
    return (int16_t)v;
}

/* Q15 * Q15 -> Q15，四舍五入 + 饱和 */
static inline int16_t Fx_Q15Mul(int16_t a, int16_t b)
{
    int32_t p = (int32_t)a * (int32_t)b;

    p += 0x4000;
    return Fx_Sat16(p >> 15);
}

/* Q15 绝对值，返回 int32 避免 -32768 溢出 */
static inline int32_t Fx_AbsQ15(int16_t a)
{
    return (a < 0) ? -(int32_t)a : (int32_t)a;
}

/* 32 位无符号整数平方根（向下取整），用于定点矢量幅值 */
uint32_t Fx_ISqrt32(uint32_t x);

/*---------------------------------------------------------------------------
 * 初始化（建 Q15 sin 表；只在上电时调用一次，重复调用安全）
 *---------------------------------------------------------------------------*/
void Foc_Fx_Init(void);

/*---------------------------------------------------------------------------
 * 相位查表 sin/cos
 *   phase: uint32，2^32 = 2*pi
 *   输出:  Q15，±32767
 *---------------------------------------------------------------------------*/
void    Foc_Fx_SinCos(uint32_t phase, int16_t *sin_out, int16_t *cos_out);
int16_t Foc_Fx_Sin(uint32_t phase);
int16_t Foc_Fx_Cos(uint32_t phase);

/*---------------------------------------------------------------------------
 * 自检（阶段 1 用，不接控制路径）
 *   对 4096 个相位点比较 Q15 sin/cos 与浮点 sinf/cosf，
 *   返回 1 = 通过，0 = 超差；误差以 Q15 LSB 记录在 res。
 *---------------------------------------------------------------------------*/
typedef struct {
    uint32_t sin_max_lsb;   /* sin 最大绝对误差，Q15 LSB */
    uint32_t cos_max_lsb;   /* cos 最大绝对误差，Q15 LSB */
    uint32_t mul_max_lsb;   /* Q15 乘法抽查最大误差，Q15 LSB */
} FocFxSelfTest_t;

uint8_t Foc_Fx_SelfTest(FocFxSelfTest_t *res);

#endif /* FOC_FX_H */
