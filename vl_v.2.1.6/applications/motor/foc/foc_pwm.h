/**
 * @file    foc_pwm.h
 * @brief   TMR4_3 FOC PWM 配置表入口
 * @note    频率 / 死区 / 采样触发点由 cfg_foc_table.h 决定，
 *          本模块只负责把表项展开后交给 hw_tmr4_pwm 驱动。
 */

#ifndef __FOC_PWM_H__
#define __FOC_PWM_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  按 cfg_foc_table.h 当前选中的配置初始化 TMR4_3
 * @retval 0 成功；-1 失败
 * @note   配置完成后立即 set_all_low()，六路 OC 关闭。
 *         此时 20kHz 计数与 EVT0→ADC 触发链已在运行。
 */
int foc_pwm_apply_cfg(void);

/**
 * @brief  从全低状态恢复 PWM 输出（FOC 进入电流环时调用）
 * @retval 0 成功；-1 失败
 */
int foc_pwm_resume_output(void);

uint32_t foc_pwm_get_frequency(void);
uint16_t foc_pwm_get_period_ticks(void);
uint16_t foc_pwm_get_dead_ticks(void);
uint16_t foc_pwm_get_deadtime_ns(void);

#ifdef __cplusplus
}
#endif

#endif /* __FOC_PWM_H__ */
