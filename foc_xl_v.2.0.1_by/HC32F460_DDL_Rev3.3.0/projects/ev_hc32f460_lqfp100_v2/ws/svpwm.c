/*=============================================================================
 * svpwm.c  —  七段式 SVPWM 实现（无死区补偿版，最终修正版）
 *=============================================================================*/

#include "svpwm.h"
#include "foc_fx.h"
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

/*=============================================================================
 * Q15 定点版本
 *
 * 输入 va/vb 已经是标幺值（1.0 = 12/sqrt(3) V），所以浮点版里的
 * x/y/z 和扇区判断都变成与 vbus 无关的定点乘加。
 *=============================================================================*/
#define SQ15_0866   28378   /* 0.8660254 * 32768 */
#define SQ15_05     16384   /* 0.5      * 32768 */
#define SQ15_ONE    32768   /* 1.0      * 32768 */

static uint8_t Svpwm_SectorQ15(int16_t va, int16_t vb)
{
    int32_t v1 = (int32_t)vb;
    int32_t v2 = ((int32_t)SQ15_0866 * (int32_t)va -
                  (int32_t)SQ15_05   * (int32_t)vb) >> 15;
    int32_t v3 = (-(int32_t)SQ15_0866 * (int32_t)va -
                  (int32_t)SQ15_05   * (int32_t)vb) >> 15;

    uint8_t a = (v1 >= 0) ? 1u : 0u;
    uint8_t b = (v2 >= 0) ? 1u : 0u;
    uint8_t c = (v3 >= 0) ? 1u : 0u;
    uint8_t s = (uint8_t)(a + 2u * b + 4u * c);

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

static int16_t Svpwm_ClampOffset(int32_t v)
{
    if (v > 16384)  return 16384;
    if (v < -16384) return -16384;
    return (int16_t)v;
}

void SVPWM_CalcQ15(int16_t valpha_q15, int16_t vbeta_q15,
                   int16_t *duty_u, int16_t *duty_v, int16_t *duty_w)
{
    int16_t va = valpha_q15;
    int16_t vb = vbeta_q15;
    uint32_t vmag_sq;
    int32_t  x, y, z;
    int32_t  t1, t2, sum, t0, ta, tb, tc;
    int32_t  ou, ov, ow;
    uint8_t  sector;

    if ((duty_u == 0) || (duty_v == 0) || (duty_w == 0)) {
        return;
    }

    /* 超线性区：先把矢量缩到单位圆，和浮点版行为一致 */
    vmag_sq = (uint32_t)((int32_t)va * (int32_t)va) +
              (uint32_t)((int32_t)vb * (int32_t)vb);
    if (vmag_sq > (uint32_t)(SQ15_ONE * SQ15_ONE)) {
        uint32_t vmag = Fx_ISqrt32(vmag_sq);

        if (vmag > 0u) {
            int32_t k = (SQ15_ONE << 15) / (int32_t)vmag;

            va = (int16_t)(((int32_t)va * k) >> 15);
            vb = (int16_t)(((int32_t)vb * k) >> 15);
        }
    }

    sector = Svpwm_SectorQ15(va, vb);

    x = (int32_t)vb;
    y = ((int32_t)SQ15_0866 * (int32_t)va + (int32_t)SQ15_05 * (int32_t)vb) >> 15;
    z = (-(int32_t)SQ15_0866 * (int32_t)va + (int32_t)SQ15_05 * (int32_t)vb) >> 15;

    switch (sector) {
    case 1: t1 = -z; t2 =  x; break;
    case 2: t1 =  z; t2 =  y; break;
    case 3: t1 =  x; t2 = -y; break;
    case 4: t1 = -x; t2 =  z; break;
    case 5: t1 = -y; t2 = -z; break;
    case 6: t1 =  y; t2 = -x; break;
    default: t1 = 0; t2 = 0; break;
    }

    sum = t1 + t2;
    if (sum > SQ15_ONE) {
        int32_t k = (SQ15_ONE << 15) / sum;

        t1 = (t1 * k) >> 15;
        t2 = (t2 * k) >> 15;
    }

    t0 = SQ15_ONE - t1 - t2;
    if (t0 < 0) t0 = 0;

    ta = t0 >> 1;
    tb = ta + t1;
    tc = tb + t2;

    switch (sector) {
    case 1: ou = SQ15_05 - ta; ov = SQ15_05 - tb; ow = SQ15_05 - tc; break;
    case 2: ou = SQ15_05 - tb; ov = SQ15_05 - ta; ow = SQ15_05 - tc; break;
    case 3: ou = SQ15_05 - tc; ov = SQ15_05 - ta; ow = SQ15_05 - tb; break;
    case 4: ou = SQ15_05 - tc; ov = SQ15_05 - tb; ow = SQ15_05 - ta; break;
    case 5: ou = SQ15_05 - tb; ov = SQ15_05 - tc; ow = SQ15_05 - ta; break;
    case 6: ou = SQ15_05 - ta; ov = SQ15_05 - tc; ow = SQ15_05 - tb; break;
    default: ou = 0; ov = 0; ow = 0; break;
    }

    *duty_u = Svpwm_ClampOffset(ou);
    *duty_v = Svpwm_ClampOffset(ov);
    *duty_w = Svpwm_ClampOffset(ow);
}
