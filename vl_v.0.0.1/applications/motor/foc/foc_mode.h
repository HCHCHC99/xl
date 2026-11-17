/**
 *******************************************************************************
 * @file  foc_mode.h
 * @brief FOC 模式层（IDLE / LOCK / IF / CALIB / CURRENT / SPEED / POSITION）
 *
 *        编号与 FOC 工程一致；参数辨识（11/14）不移植。
 *        mode 3/5/6/7 为手动触发的编码器标定，动作为阻塞式状态机，
 *        由 20kHz/1ms 分阶段推进。
 *******************************************************************************
 */

#ifndef __FOC_MODE_H__
#define __FOC_MODE_H__

#include <stdbool.h>
#include <stdint.h>

/* 1 = FOC 独占 TMR4_3：禁用旧六步路径的周期更新、占空比/使能/刹车写入 */
#ifndef FOC_DISABLE_LEGACY_6STEP
#define FOC_DISABLE_LEGACY_6STEP    1
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    FOC_MODE_IDLE     = 0,   /* 全低电平，安全停机 */
    FOC_MODE_LOCK     = 1,   /* 零矢量锁轴，默认 2%，可用 focmode 1 <duty> 修改 */
    FOC_MODE_IF       = 2,   /* q 轴加压 + 角度自增，开环拖动 */
    FOC_MODE_CALIB    = 3,   /* 编码器电角度对齐（mode 3，手动） */
    FOC_MODE_CURRENT  = 4,   /* 有感闭环电流环 */
    FOC_MODE_SWEEP    = 5,   /* 扫角测量（mode 5） */
    FOC_MODE_REV      = 6,   /* 一圈计数测量（mode 6） */
    FOC_MODE_ALIGN7   = 7,   /* 斜坡对齐（mode 7） */
    FOC_MODE_SPEED    = 9,   /* 速度外环 + 电流环 */
    FOC_MODE_POSITION = 10,  /* 位置外环 + 速度环 + 电流环 */
} foc_mode_t;

/* L6 每拍产出的控制指令 */
typedef struct {
    float vd;
    float vq;
    float theta_e;
    bool  enable;
} FocCmd_t;

void       Foc_Mode_Init(void);
void       Foc_Mode_SetMode(foc_mode_t mode);
foc_mode_t Foc_Mode_GetMode(void);

/* IF 参数设置（不改变当前模式） */
void Foc_Mode_SetOpenloop(float freq_hz, float vq_volts, float vd_volts);
void Foc_Mode_StartOpenloop(void);
void Foc_Mode_Stop(void);

/* 每拍调用（20kHz ISR） */
void Foc_Mode_Step(float ts, FocCmd_t *out);

/* LOCK 模式占空比（%）；若当前已在 LOCK，立即生效 */
void       Foc_Mode_SetLockDuty(float duty_pct);
float      Foc_Mode_GetLockDuty(void);

/* 1 = FOC 模式已接管 PWM（非 IDLE）；旧六步路径应停止写占空比 */
int        Foc_Mode_PwmOwned(void);

extern volatile foc_mode_t g_fm_mode;
extern volatile float      g_fm_theta_e;
extern volatile float      g_fm_freq_hz;
extern volatile float      g_fm_vq;
extern volatile float      g_fm_vd;
extern volatile uint32_t   g_fm_step_cnt;

#ifdef __cplusplus
}
#endif

#endif /* __FOC_MODE_H__ */
