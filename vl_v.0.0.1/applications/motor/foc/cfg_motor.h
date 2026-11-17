/*=============================================================================
 * cfg_motor.h  —  FOC 电机参数（按 foc_xl_v.2.0.1）
 *=============================================================================*/

#ifndef CFG_MOTOR_H
#define CFG_MOTOR_H

/* 3505 电机：10 极对 */
#define CFG_MOTOR_POLE_PAIRS     10u

/* ABZ 编码器：1024 线 × 4 倍频 = 4096 CPR */
#define CFG_ENCODER_LINES        1024u
#define CFG_ENCODER_CPR          (CFG_ENCODER_LINES * 4u)

#endif /* CFG_MOTOR_H */
