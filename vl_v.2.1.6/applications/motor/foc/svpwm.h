/**
 *******************************************************************************
 * @file  svpwm.h
 * @brief L4 七段式 SVPWM 调制层（无死区补偿）
 *
 *        输入 Vα/Vβ/vbus，输出三相中心占空比 duty_u/v/w（0~1，0.5 = 零电压）
 *        纯函数，无全局状态、无硬件依赖
 *******************************************************************************
 */

#ifndef SVPWM_H
#define SVPWM_H

#include <stdint.h>

typedef struct {
    /* ---- 输入 ---- */
    float valpha;       /* Vα (V) */
    float vbeta;        /* Vβ (V) */
    float vbus;         /* 母线电压 (V) */

    /* ---- 输出：中心占空比 0~1 ---- */
    float duty_u;
    float duty_v;
    float duty_w;

    /* ---- 只读中间量 ---- */
    float   modulation; /* |V| / (vbus/√3)，0~1 */
    uint8_t sector;     /* 扇区 1~6 */
} SvpwmCtx_t;

void SVPWM_Calc(SvpwmCtx_t *ctx);

#endif /* SVPWM_H */
