/**
 * @file    foc_pwm.c
 * @brief   TMR4_3 FOC PWM 配置表入口实现
 */

#include "foc_pwm.h"
#include "cfg_foc_table.h"
#include "hw_tmr4_pwm.h"

#include <rtthread.h>

int foc_pwm_apply_cfg(void)
{
    tmr4_pwm_config_t cfg;
    tmr4_pwm_err_t    err;

    cfg.outputTypeU = FOC_CFG_OUTPUT_TYPE;
    cfg.outputTypeV = FOC_CFG_OUTPUT_TYPE;
    cfg.outputTypeW = FOC_CFG_OUTPUT_TYPE;
    cfg.freqHz      = FOC_CFG_PWM_FREQ;
    cfg.deadTimeNs  = FOC_CFG_DEADTIME_NS;
    cfg.activeHigh  = FOC_CFG_ACTIVE_HIGH;

    err = hw_tmr4_pwm_config(&cfg);
    if (err != TMR4_PWM_OK) {
        rt_kprintf("[FOC-PWM] config failed: %d\r\n", (int)err);
        return -1;
    }

    err = hw_tmr4_pwm_start_output();
    if (err != TMR4_PWM_OK) {
        rt_kprintf("[FOC-PWM] start failed: %d\r\n", (int)err);
        return -1;
    }

    /* 配置后先全低：OC 关闭 + 无效电平 LOW。
     * 功率级另有 EMB 封锁；FOC 真正输出前调用 foc_pwm_resume_output()。 */
    (void)hw_tmr4_pwm_set_all_low();

    rt_kprintf("[FOC-PWM] cfg=%s freq=%lu Hz dt=%u ns (%u ticks) sample=%s\r\n",
               FOC_CFG_NAME,
               (unsigned long)hw_tmr4_pwm_get_frequency(),
               (unsigned)hw_tmr4_pwm_get_deadtime(),
               (unsigned)hw_tmr4_pwm_get_dead_ticks(),
               (FOC_CFG_SAMPLE_POINT == CFG_SAMPLE_UPPER) ? "UPPER(peak)"
                                                          : "LOWER(valley)");
    return 0;
}

int foc_pwm_resume_output(void)
{
    tmr4_pwm_err_t err = hw_tmr4_pwm_resume_output();

    if (err != TMR4_PWM_OK) {
        rt_kprintf("[FOC-PWM] resume failed: %d\r\n", (int)err);
        return -1;
    }
    return 0;
}

uint32_t foc_pwm_get_frequency(void)
{
    return hw_tmr4_pwm_get_frequency();
}

uint16_t foc_pwm_get_period_ticks(void)
{
    return hw_tmr4_pwm_get_period_ticks();
}

uint16_t foc_pwm_get_dead_ticks(void)
{
    return hw_tmr4_pwm_get_dead_ticks();
}

uint16_t foc_pwm_get_deadtime_ns(void)
{
    return hw_tmr4_pwm_get_deadtime();
}
