/**
 * @file    foc_mode.h
 * @brief   FOC 模式层（当前仅 IDLE / LOCK，编号与参考工程一致）
 * @note    后续移植完整 foc_mode.c 时可直接扩展枚举与 Step 逻辑。
 */

#ifndef __FOC_MODE_H__
#define __FOC_MODE_H__

#include <stdint.h>

/* 1 = FOC 独占 TMR4_3：禁用旧六步路径的周期更新、占空比/使能/刹车写入。
 * 0 = 允许旧六步路径在 FOC IDLE 时运行。当前工程使用 FOC，默认 1。 */
#ifndef FOC_DISABLE_LEGACY_6STEP
#define FOC_DISABLE_LEGACY_6STEP    1
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    FOC_MODE_IDLE = 0,   /* 全低电平，上下管全关（安全停机） */
    FOC_MODE_LOCK = 1,   /* 零矢量锁轴：默认 2%，可用 focmode 1 <duty> 修改 */
} foc_mode_t;

void       Foc_Mode_Init(void);
void       Foc_Mode_SetMode(foc_mode_t mode);
foc_mode_t Foc_Mode_GetMode(void);

/* 设置 LOCK 模式占空比（%）；若当前已在 LOCK，立即生效 */
void       Foc_Mode_SetLockDuty(float duty_pct);
float      Foc_Mode_GetLockDuty(void);

/* 1 = FOC 模式已接管 PWM（非 IDLE）；旧六步路径应停止写占空比 */
int        Foc_Mode_PwmOwned(void);

#ifdef __cplusplus
}
#endif

#endif /* __FOC_MODE_H__ */
