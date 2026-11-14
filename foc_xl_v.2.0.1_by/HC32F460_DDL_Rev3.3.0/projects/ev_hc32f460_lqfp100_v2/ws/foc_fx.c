/*=============================================================================
 * foc_fx.c  —  定点运算基础库实现
 *=============================================================================*/

#include "foc_fx.h"

#include <math.h>

#define FX_SC_PI      3.141592653589793f
#define FX_SC_2PI     6.283185307179586f
#define FX_SC_HALF_PI 1.570796326794897f

static int16_t s_sin_lut[FX_SINCOS_LUT_SIZE];
static uint8_t s_lut_inited = 0u;

void Foc_Fx_Init(void)
{
    uint32_t i;

    if (s_lut_inited) {
        return;
    }

    for (i = 0u; i < FX_SINCOS_LUT_SIZE; i++) {
        float s = sinf(FX_SC_2PI * (float)i / (float)FX_SINCOS_LUT_SIZE);

        /* 四舍五入到 Q15 */
        s *= 32767.0f;
        s += (s >= 0.0f) ? 0.5f : -0.5f;
        s_sin_lut[i] = (int16_t)s;
    }

    s_lut_inited = 1u;
}

static int16_t Fx_LutSin(uint32_t phase)
{
    uint32_t i0   = phase >> (32u - FX_SINCOS_LUT_BITS);
    uint32_t frac = (phase >> (32u - 2u * FX_SINCOS_LUT_BITS)) & (FX_SINCOS_LUT_SIZE - 1u);
    int32_t  s0   = (int32_t)s_sin_lut[i0];
    int32_t  s1   = (int32_t)s_sin_lut[(i0 + 1u) & FX_SINCOS_LUT_MASK];
    int32_t  d    = s1 - s0;

    /* d * frac 为 Q10，右移 10 得到 Q15 增量 */
    return Fx_Sat16(s0 + ((d * (int32_t)frac) >> FX_SINCOS_LUT_BITS));
}

void Foc_Fx_SinCos(uint32_t phase, int16_t *sin_out, int16_t *cos_out)
{
    if (sin_out != 0) {
        *sin_out = Fx_LutSin(phase);
    }
    if (cos_out != 0) {
        *cos_out = Fx_LutSin(phase + 0x40000000u);   /* +90 度 */
    }
}

int16_t Foc_Fx_Sin(uint32_t phase)
{
    return Fx_LutSin(phase);
}

int16_t Foc_Fx_Cos(uint32_t phase)
{
    return Fx_LutSin(phase + 0x40000000u);
}

/*---------------------------------------------------------------------------
 * 32 位无符号整数平方根，逐位试探，16 次循环
 *---------------------------------------------------------------------------*/
uint32_t Fx_ISqrt32(uint32_t x)
{
    uint32_t res = 0u;
    uint32_t bit = 1u << 30;

    while (bit > x) {
        bit >>= 2;
    }

    while (bit != 0u) {
        if (x >= (res + bit)) {
            x   -= (res + bit);
            res  = (res >> 1) + bit;
        } else {
            res >>= 1;
        }
        bit >>= 2;
    }

    return res;
}

/*---------------------------------------------------------------------------
 * 自检：Q15 sin/cos 与浮点基准对比，Q15 乘法抽查
 *---------------------------------------------------------------------------*/
uint8_t Foc_Fx_SelfTest(FocFxSelfTest_t *res)
{
    uint32_t i;
    uint32_t sin_max = 0u;
    uint32_t cos_max = 0u;
    uint32_t mul_max = 0u;

    Foc_Fx_Init();

    for (i = 0u; i < 4096u; i++) {
        uint32_t phase = i << 20;   /* 4096 点覆盖 2^32 */
        float    th    = FX_SC_2PI * (float)i / 4096.0f;
        int16_t  s;
        int16_t  c;
        int32_t  ref_s;
        int32_t  ref_c;
        uint32_t e;

        Foc_Fx_SinCos(phase, &s, &c);

        ref_s = (int32_t)(sinf(th) * 32767.0f);
        ref_c = (int32_t)(cosf(th) * 32767.0f);

        e = (uint32_t)Fx_AbsQ15((int16_t)((int32_t)s - ref_s));
        if (e > sin_max) sin_max = e;

        e = (uint32_t)Fx_AbsQ15((int16_t)((int32_t)c - ref_c));
        if (e > cos_max) cos_max = e;
    }

    /* 乘法抽查 */
    {
        static const int16_t a[] = { 32767, 16384, 8192, 1000, -32768, -16384, -1000, 12345 };
        static const int16_t b[] = { 32767, 16384, 8192, 1000, -32768,  16384,  1000, -6789 };
        uint32_t k;

        for (k = 0u; k < (sizeof(a) / sizeof(a[0])); k++) {
            int32_t  p    = (int32_t)a[k] * (int32_t)b[k];
            int32_t  ref  = (int32_t)((float)a[k] * (float)b[k] / 32768.0f);
            int16_t  got  = Fx_Q15Mul(a[k], b[k]);
            uint32_t e    = (uint32_t)Fx_AbsQ15((int16_t)((int32_t)got - ref));

            (void)p;
            if (e > mul_max) mul_max = e;
        }
    }

    if (res != 0) {
        res->sin_max_lsb = sin_max;
        res->cos_max_lsb = cos_max;
        res->mul_max_lsb = mul_max;
    }

    /* 查表 + 线性插值理论误差 < 2 LSB，留 8 LSB 余量 */
    return (uint8_t)((sin_max <= 8u) && (cos_max <= 8u) && (mul_max <= 2u)) ? 1u : 0u;
}
