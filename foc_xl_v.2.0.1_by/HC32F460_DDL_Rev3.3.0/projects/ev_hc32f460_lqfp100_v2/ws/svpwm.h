/*=============================================================================
 * svpwm.h  —  FOC SVPWM 调制层（L4，无死区补偿版）
 *
 * 职责：
 *   - 输入 Vα/Vβ/vbus，输出三相中心占空比 duty_u/v/w（0~1，0.5 = 零电压）
 *   - 仅七段式 SVPWM
 *   - 仅线性区 + 最简边界保护（超线性区按比例缩、T1+T2>1 按比例缩、输出钳 0~1）
 *   - 不含五段式 / 六边形限幅 / 过调制 / 死区补偿
 *
 * 契约：
 *   - 输出为"中心占空比"语义，供 TMR4_PWM_SetDuty3Phase(center×100) 使用
 *   - 纯函数，无全局状态、无硬件依赖
 *=============================================================================*/

#ifndef SVPWM_H
#define SVPWM_H

#include <stdint.h>

typedef struct {
    /* ---- 输入 ---- */
    float valpha;       /* Vα (V) */
    float vbeta;        /* Vβ (V) */
    float vbus;         /* 母线电压 (V) */

    /* ---- 输出：中心占空比 0~1（0.5 = 零电压）---- */
    float duty_u;       /* U 相 */
    float duty_v;       /* V 相 */
    float duty_w;       /* W 相 */

    /* ---- 只读中间量（调试/弱磁预留）---- */
    float   modulation; /* |V| / (vbus/√3)，0~1 */
    uint8_t sector;     /* 扇区 1~6 */
} SvpwmCtx_t;

/* 计算：输入 valpha/vbeta/vbus → 输出 duty_u/v/w（中心占空比 0~1） */
void SVPWM_Calc(SvpwmCtx_t *ctx);

/*---------------------------------------------------------------------------
 * Q15 定点版本（固定 12V 母线归一化）
 *   输入：valpha/vbeta 为 Q15 标幺，1.0 = 12/sqrt(3) V
 *   输出：duty_u/v/w 为相对 50% 的偏移，Q15，16384 = 50%，范围 ±16384
 *---------------------------------------------------------------------------*/
void SVPWM_CalcQ15(int16_t valpha_q15, int16_t vbeta_q15,
                   int16_t *duty_u, int16_t *duty_v, int16_t *duty_w);

#endif /* SVPWM_H */
