/**
 *******************************************************************************
 * @file  foc_output.c
 * @brief FOC 运行期桥臂输出唯一入口实现（Phase 4-lite）
 *
 *        只做转发与返回码归一化，不含任何策略：
 *          - 数值常量、调用顺序、失败处理全部由调用方决定；
 *          - 本文件之外不应再出现运行期 hw_tmr4_pwm_set_* / bsp_pwm_emergency_*。
 *******************************************************************************
 */

#include "foc_output.h"

#include "hw_tmr4_pwm.h"
#include "../../../bsp/bsp_pwm.h"

/* 20kHz 热路径（ISR 会调 AllLow/Resume/Duty3）：与其它热路径文件一致开 O2 */
#pragma GCC optimize("O2")

void Foc_Output_Init(void)
{
    (void)hw_tmr4_pwm_set_all_low();
    bsp_pwm_emergency_brake();
}

int Foc_Output_AllLow(void)
{
    return (hw_tmr4_pwm_set_all_low() == TMR4_PWM_OK) ? 0 : -1;
}

int Foc_Output_Resume(void)
{
    return (hw_tmr4_pwm_resume_output() == TMR4_PWM_OK) ? 0 : -1;
}

int Foc_Output_Duty3(float du, float dv, float dw)
{
    return (hw_tmr4_pwm_set_duty3(du, dv, dw) == TMR4_PWM_OK) ? 0 : -1;
}

void Foc_Output_Brake(void)
{
    bsp_pwm_emergency_brake();
}

void Foc_Output_Release(void)
{
    bsp_pwm_emergency_release();
}

void Foc_Output_SafeStop(void)
{
    (void)hw_tmr4_pwm_set_all_low();
    bsp_pwm_emergency_brake();
}