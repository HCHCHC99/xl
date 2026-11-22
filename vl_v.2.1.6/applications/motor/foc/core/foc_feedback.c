/**
 *******************************************************************************
 * @file  foc_feedback.c
 * @brief FOC 运行期反馈唯一入口实现（Phase 4-lite）
 *
 *        纯转发，无状态、无量纲转换、无滤波。
 *        环路与控制宿主通过本模块取数，不再直接 include hw_encoder.h。
 *******************************************************************************
 */

#include "foc_feedback.h"

#include "hw_encoder.h"

/* 20kHz 热路径（电流环/位置环在 ISR 内取值）：与其它热路径文件一致开 O2 */
#pragma GCC optimize("O2")

void Foc_Feedback_Init(void)
{
    /* 当前无内部状态：编码器由 hw_encoder_init() 负责。
     * 保留入口，便于后续在此缓存 CPR/极对数/换算系数。 */
}

int32_t Foc_Feedback_GetCount(void)
{
    return hw_encoder_get_count();
}

float Foc_Feedback_GetMechAngleRad(void)
{
    return hw_encoder_get_mech_angle_rad();
}

float Foc_Feedback_GetElecAngleRad(uint8_t pole_pairs)
{
    return hw_encoder_get_elec_angle_rad(pole_pairs);
}

float Foc_Feedback_GetSpeedRpm(void)
{
    return hw_encoder_get_speed_rpm();
}

int8_t Foc_Feedback_GetDir(void)
{
    return g_enc_elec_dir;
}

float Foc_Feedback_GetAngleDeg(void)
{
    return hw_encoder_get_angle_deg();
}

uint32_t Foc_Feedback_GetRevCount(void)
{
    return hw_encoder_get_rev_count();
}