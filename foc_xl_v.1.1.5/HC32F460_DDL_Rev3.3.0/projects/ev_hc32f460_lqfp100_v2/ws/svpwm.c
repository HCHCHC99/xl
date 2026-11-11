/*=============================================================================
 * svpwm.c  —  七段式 SVPWM 实现（无死区补偿版，最终修正版）
 *=============================================================================*/

#include "svpwm.h"
#include <math.h>

#define SQRT3          1.7320508f
#define SQRT3_DIV2     0.8660254f
#define ONE_DIV_SQRT3  0.57735027f

static float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

/* 扇区判断：符号用 >=0，保证 θ=0° 归扇区 1、边界 dv=dw */
static uint8_t Svpwm_Sector(float va, float vb)
{
    float v1 = vb;
    float v2 = SQRT3_DIV2 * va - 0.5f * vb;
    float v3 = -SQRT3_DIV2 * va - 0.5f * vb;

    uint8_t a = (v1 >= 0.0f) ? 1U : 0U;
    uint8_t b = (v2 >= 0.0f) ? 1U : 0U;
    uint8_t c = (v3 >= 0.0f) ? 1U : 0U;

    uint8_t s = (uint8_t)(a + 2U * b + 4U * c);

    switch (s) {
    case 3: return 1;
    case 1: return 2;
    case 5: return 3;
    case 4: return 4;
    case 6: return 5;
    case 2: return 6;
    default: return 1;
    }
}

static void Svpwm_Times(uint8_t sector, float va, float vb, float vbus,
                        float *t1, float *t2)
{
    float x = SQRT3 * vb / vbus;
    float y = (1.5f * va + SQRT3_DIV2 * vb) / vbus;
    float z = (-1.5f * va + SQRT3_DIV2 * vb) / vbus;

    switch (sector) {
    case 1: *t1 = -z; *t2 =  x; break;
    case 2: *t1 =  z; *t2 =  y; break;
    case 3: *t1 =  x; *t2 = -y; break;
    case 4: *t1 = -x; *t2 =  z; break;
    case 5: *t1 = -y; *t2 = -z; break;
    case 6: *t1 =  y; *t2 = -x; break;
    default: *t1 = 0.0f; *t2 = 0.0f; break;
    }
}

static void Svpwm_Assign_SevenSeg(uint8_t sector,
                                  float t1, float t2, float t0,
                                  float *du, float *dv, float *dw)
{
    /* 低电平累计时间（不折半） */
    float ta = t0 * 0.5f;      /* 零矢量一半 */
    float tb = ta + t1;        /* + T1 */
    float tc = tb + t2;        /* + T2 */

    /* du = 1 - 低电平时间；不同扇区相序映射不同 */
    switch (sector) {
    case 1: *du = 1.0f - ta; *dv = 1.0f - tb; *dw = 1.0f - tc; break;
    case 2: *du = 1.0f - tb; *dv = 1.0f - ta; *dw = 1.0f - tc; break;
    case 3: *du = 1.0f - tc; *dv = 1.0f - ta; *dw = 1.0f - tb; break;
    case 4: *du = 1.0f - tc; *dv = 1.0f - tb; *dw = 1.0f - ta; break;
    case 5: *du = 1.0f - tb; *dv = 1.0f - tc; *dw = 1.0f - ta; break;
    case 6: *du = 1.0f - ta; *dv = 1.0f - tc; *dw = 1.0f - tb; break;
    default: *du = 0.5f; *dv = 0.5f; *dw = 0.5f; break;
    }
}

void SVPWM_Calc(SvpwmCtx_t *ctx)
{
    float vmax, vmag, t1, t2, t0;

    if (ctx == 0) {
        return;
    }

    if (ctx->vbus < 1.0f) {
        ctx->duty_u = 0.5f;
        ctx->duty_v = 0.5f;
        ctx->duty_w = 0.5f;
        ctx->modulation = 0.0f;
        ctx->sector = 1;
        return;
    }

    vmax = ctx->vbus * ONE_DIV_SQRT3;
    vmag = sqrtf(ctx->valpha * ctx->valpha + ctx->vbeta * ctx->vbeta);

    if (vmag > vmax) {
        float k = vmax / vmag;
        ctx->valpha *= k;
        ctx->vbeta  *= k;
        vmag = vmax;
    }
    ctx->modulation = vmag / vmax;

    ctx->sector = Svpwm_Sector(ctx->valpha, ctx->vbeta);
    Svpwm_Times(ctx->sector, ctx->valpha, ctx->vbeta, ctx->vbus, &t1, &t2);

    if (t1 + t2 > 1.0f) {
        float k = 1.0f / (t1 + t2);
        t1 *= k;
        t2 *= k;
    }
    t0 = 1.0f - t1 - t2;

    Svpwm_Assign_SevenSeg(ctx->sector, t1, t2, t0,
                          &ctx->duty_u, &ctx->duty_v, &ctx->duty_w);

    ctx->duty_u = clampf(ctx->duty_u, 0.0f, 1.0f);
    ctx->duty_v = clampf(ctx->duty_v, 0.0f, 1.0f);
    ctx->duty_w = clampf(ctx->duty_w, 0.0f, 1.0f);
}
