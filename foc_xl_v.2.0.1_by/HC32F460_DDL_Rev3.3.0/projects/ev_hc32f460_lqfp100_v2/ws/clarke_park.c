/*=============================================================================
 * clarke_park.c  —  L3 变换层实现
 *
 * 相对旧版改动：
 *   - InvPark/Park 的 sin/cos 改为 Foc_Sin/Foc_Cos（LUT，ISR 安全）
 *   - Angle_Normalize 改为恒定时间（不再 while 循环）
 *   - 公式、函数名、签名、参数语义均未改
 *=============================================================================*/

#include "clarke_park.h"
#include "foc_sincos.h"
#include "foc_fx.h"
#include <stddef.h>

#define ONE_DIV_SQRT3_Q15   18919   /* 0.57735027 * 32768 */

/* int64 Q30 -> Q15，四舍五入并饱和 */
static int16_t Q15_FromQ30(int64_t v)
{
    v += 0x4000;
    v >>= 15;
    return Fx_Sat16((int32_t)v);
}

/* 角度归一化到 [0, 2π)，恒定时间 */
float Angle_Normalize(float theta)
{
    float k = (float)(int32_t)(theta * (1.0f / TWO_PI_F));

    theta -= k * TWO_PI_F;
    if (theta < 0.0f) {
        theta += TWO_PI_F;
    }
    if (theta >= TWO_PI_F) {
        theta -= TWO_PI_F;
    }
    return theta;
}

void InvPark(float vd, float vq, float theta_e, float *valpha, float *vbeta)
{
    float c, s;

    if ((valpha == NULL) || (vbeta == NULL)) {
        return;
    }

    c = Foc_Cos(theta_e);
    s = Foc_Sin(theta_e);

    *valpha = vd * c - vq * s;
    *vbeta  = vd * s + vq * c;
}

void Clarke(float ia, float ib, float ic, float *ialpha, float *ibeta)
{
    (void)ic;

    if ((ialpha == NULL) || (ibeta == NULL)) {
        return;
    }

    *ialpha = ia;
    *ibeta  = (ia + 2.0f * ib) * ONE_DIV_SQRT3_F;
}

void Park(float ialpha, float ibeta, float theta_e, float *id, float *iq)
{
    float c, s;

    if ((id == NULL) || (iq == NULL)) {
        return;
    }

    c = Foc_Cos(theta_e);
    s = Foc_Sin(theta_e);

    *id =  ialpha * c + ibeta * s;
    *iq = -ialpha * s + ibeta * c;
}

/*=============================================================================
 * Q15 定点版本
 *=============================================================================*/
void ClarkeQ15(int16_t ia, int16_t ib, int16_t ic,
               int16_t *ialpha, int16_t *ibeta)
{
    (void)ic;

    if ((ialpha == NULL) || (ibeta == NULL)) {
        return;
    }

    *ialpha = ia;

    /* ibeta = (ia + 2*ib) / sqrt(3)，用 Q15 系数并四舍五入 */
    {
        int32_t t = (int32_t)ia + 2 * (int32_t)ib;

        t = (t * ONE_DIV_SQRT3_Q15 + 0x4000) >> 15;
        *ibeta = Fx_Sat16(t);
    }
}

void ParkQ15(int16_t ialpha, int16_t ibeta, uint32_t phase,
             int16_t *id, int16_t *iq)
{
    int16_t s;
    int16_t c;

    if ((id == NULL) || (iq == NULL)) {
        return;
    }

    Foc_Fx_SinCos(phase, &s, &c);

    *id = Q15_FromQ30((int64_t)ialpha * (int64_t)c +
                      (int64_t)ibeta  * (int64_t)s);
    *iq = Q15_FromQ30(-(int64_t)ialpha * (int64_t)s +
                      (int64_t)ibeta  * (int64_t)c);
}

void InvParkQ15(int16_t vd, int16_t vq, uint32_t phase,
                int16_t *valpha, int16_t *vbeta)
{
    int16_t s;
    int16_t c;

    if ((valpha == NULL) || (vbeta == NULL)) {
        return;
    }

    Foc_Fx_SinCos(phase, &s, &c);

    *valpha = Q15_FromQ30((int64_t)vd * (int64_t)c -
                          (int64_t)vq * (int64_t)s);
    *vbeta  = Q15_FromQ30((int64_t)vd * (int64_t)s +
                          (int64_t)vq * (int64_t)c);
}
