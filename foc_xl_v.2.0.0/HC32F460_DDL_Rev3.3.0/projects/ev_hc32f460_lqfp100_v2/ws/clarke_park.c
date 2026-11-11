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
#include <stddef.h>

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
