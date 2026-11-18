/**
 *******************************************************************************
 * @file  clarke_park.h
 * @brief L3 变换层：InvPark / Clarke / Park / 角度归一化
 *
 *        契约：纯函数，无全局状态无硬件依赖；theta 单位 rad
 *******************************************************************************
 */

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

#endif /* CLARKE_PARK_H */
