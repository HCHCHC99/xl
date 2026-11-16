/**
 * @file    foc_mode.h
 * @brief   FOC 模式层（当前仅 IDLE / LOCK，编号与参考工程一致）
 * @note    后续移植完整 foc_mode.c 时可直接扩展枚举与 Step 逻辑。
 */

#ifndef __FOC_MODE_H__
#define __FOC_MODE_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    FOC_MODE_IDLE = 0,   /* 全低电平，上下管全关（安全停机） */
    FOC_MODE_LOCK = 1,   /* 2% 零矢量，锁死电机 */
} foc_mode_t;

void       Foc_Mode_Init(void);
void       Foc_Mode_SetMode(foc_mode_t mode);
foc_mode_t Foc_Mode_GetMode(void);

#ifdef __cplusplus
}
#endif

#endif /* __FOC_MODE_H__ */
