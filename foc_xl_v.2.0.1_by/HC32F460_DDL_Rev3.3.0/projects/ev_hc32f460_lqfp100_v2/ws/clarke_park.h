/*=============================================================================
 * clarke_park.h  —  L3 变换层（坐标变换）
 *
 * 职责：
 *   - InvPark：dq 电压 → αβ 电压（输出路径）
 *   - Clarke：三相电流 → αβ 电流（反馈路径）
 *   - Park：  αβ 电流 → dq 电流（反馈路径）
 *   - Angle_Normalize：角度归一化到 [0, 2π)
 *
 * 契约：
 *   - 纯函数，无全局状态无硬件依赖
 *   - θe 原样使用，不偏移 90°不因正反转取反
 *   - 角度单位 rad
 *
 * 本版改动：
 *   - InvPark/Park 的 sin/cos 改为走 foc_sincos 的 LUT（ISR 安全）
 *   - Angle_Normalize 改为恒定时间（单次除法 + 条件修正）
 *   - 函数名签名公式均未改
 *
 * 当前调用点：
 *   - Clarke / Park 在 Foc_Control_Step() 第 3 步（20kHz ISR）
 *   - InvPark 在 Foc_Control_Step() 第 4 步（IF 分支）
 *   - Angle_Normalize 当前未被调用（Foc_Mode_Step 内部自折角度），备用
 *=============================================================================*/

#ifndef CLARKE_PARK_H
#define CLARKE_PARK_H

#include <stdint.h>

#define PI_F            3.14159265f
#define TWO_PI_F        6.28318530f
#define HALF_PI_F       1.57079633f
#define SQRT3_F         1.7320508f
#define ONE_DIV_SQRT3_F 0.57735027f

float Angle_Normalize(float theta);

void InvPark(float vd, float vq, float theta_e, float *valpha, float *vbeta);

void Clarke(float ia, float ib, float ic, float *ialpha, float *ibeta);

void Park(float ialpha, float ibeta, float theta_e, float *id, float *iq);

/*---------------------------------------------------------------------------
 * Q15 定点版本（phase: uint32，2^32 = 2π）
 *   输入输出都是 Q15 标幺；内部用 int64 中间量，避免 Park 求和溢出
 *---------------------------------------------------------------------------*/
void ClarkeQ15(int16_t ia, int16_t ib, int16_t ic,
               int16_t *ialpha, int16_t *ibeta);
void ParkQ15(int16_t ialpha, int16_t ibeta, uint32_t phase,
             int16_t *id, int16_t *iq);
void InvParkQ15(int16_t vd, int16_t vq, uint32_t phase,
                int16_t *valpha, int16_t *vbeta);

#endif /* CLARKE_PARK_H */
