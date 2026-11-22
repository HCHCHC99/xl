/**
 *******************************************************************************
 * @file  foc_feedback.h
 * @brief FOC 运行期反馈的唯一入口（编码器抽象）
 *
 *        签名与 hw_encoder 一一对应，调用方只做 1:1 函数改名，不改任何量纲。
 *        本阶段只收敛访问点：环路/控制宿主不再 include hw_encoder.h。
 *
 *        允许直连 drv 的例外：encoder_calib.c（标定需要原始计数与电角度
 *        对齐写入），它属于标定域，不属于运行期反馈。
 *******************************************************************************
 */

#ifndef __FOC_FEEDBACK_H__
#define __FOC_FEEDBACK_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化（当前无状态，保留占位以便后续缓存 CPR/极对数） */
void     Foc_Feedback_Init(void);

/* 累计计数（电机轴，counts） */
int32_t  Foc_Feedback_GetCount(void);

/* 机械角 [0, 2π) */
float    Foc_Feedback_GetMechAngleRad(void);

/* 电角度 [0, 2π)，已扣零点偏移并按电角度方向翻转 */
float    Foc_Feedback_GetElecAngleRad(uint8_t pole_pairs);

/* 转速（电机轴 rpm） */
float    Foc_Feedback_GetSpeedRpm(void);

/* 电角度方向：+1 / -1（原 g_enc_elec_dir） */
int8_t   Foc_Feedback_GetDir(void);

/* 机械角（度） */
float    Foc_Feedback_GetAngleDeg(void);

/* 圈计数 */
uint32_t Foc_Feedback_GetRevCount(void);

#ifdef __cplusplus
}
#endif

#endif /* __FOC_FEEDBACK_H__ */