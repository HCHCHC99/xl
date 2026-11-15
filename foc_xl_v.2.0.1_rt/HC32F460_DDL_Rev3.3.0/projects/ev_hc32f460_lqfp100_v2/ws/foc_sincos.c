/*=============================================================================
 * foc_sincos.c  —  Sin/Cos 查表实现
 *=============================================================================*/

#include "foc_sincos.h"
#include <math.h>

#define SC_PI       3.141592653589793f
#define SC_HALF_PI  1.570796326794897f
#define SC_2PI      6.283185307179586f

#define SC_LUT_SCALE  ((float)FOC_SINCOS_LUT_SIZE / SC_2PI)

static float   s_fSinLut[FOC_SINCOS_LUT_SIZE];
static uint8_t s_bLutInit = 0u;

void Foc_SinCos_Init(void)
{
    uint32_t i;

    if (s_bLutInit) {
        return;
    }
    for (i = 0u; i < FOC_SINCOS_LUT_SIZE; i++) {
        s_fSinLut[i] = sinf(SC_2PI * (float)i / (float)FOC_SINCOS_LUT_SIZE);
    }
    s_bLutInit = 1u;
}

/* 恒定时间折叠到 [0, 2π) */
static float Foc_SinCos_Fold(float theta)
{
    float k = (float)(int32_t)(theta * (1.0f / SC_2PI));

    theta -= k * SC_2PI;
    if (theta < 0.0f) {
        theta += SC_2PI;
    }
    if (theta >= SC_2PI) {
        theta -= SC_2PI;
    }
    return theta;
}

static float Foc_SinCos_LutSin(float theta)
{
    float    fPos = theta * SC_LUT_SCALE;
    uint32_t i0   = (uint32_t)fPos & FOC_SINCOS_LUT_MASK;
    uint32_t i1   = (i0 + 1u) & FOC_SINCOS_LUT_MASK;
    float    frac = fPos - (float)(uint32_t)fPos;

    return s_fSinLut[i0] + (s_fSinLut[i1] - s_fSinLut[i0]) * frac;
}

float Foc_Sin(float theta)
{
    return Foc_SinCos_LutSin(Foc_SinCos_Fold(theta));
}

float Foc_Cos(float theta)
{
    return Foc_SinCos_LutSin(Foc_SinCos_Fold(theta + SC_HALF_PI));
}
