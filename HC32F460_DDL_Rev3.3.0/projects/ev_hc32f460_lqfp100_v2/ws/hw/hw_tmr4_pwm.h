#ifndef __HW_TMR4_PWM_H__
#define __HW_TMR4_PWM_H__

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/*=============================================================================
 * hw_tmr4_pwm  —  TMR4 unit 3 三路 PWM 驱动（纯外设逻辑层）
 *
 * 引脚（由 hw_gpio_init.c 配置，本模块不碰 GPIO）：
 *   U 相上管/下管 = PB9 / PB8
 *   V 相上管/下管 = PB7 / PB6
 *   W 相上管/下管 = PB5 / PB4
 *
 * 职责：
 *   只操作 TMR4_3 外设：计数OC 通道死区PWM 模式比较值
 *   不操作 GPIO：引脚复用由 hw_gpio_init_all() 配好
 *
 * 波形约定：
 *   互补模式：高侧导通居中于峰值（比较值=关断点），低侧由硬件取反、居中于谷底
 *   同向模式：上下管同相，导通区居中于谷底（PB4/PB5 同高同低，比较值=导通宽度）
 *   同向模式下死区体现为下管比上管提前 deadTicks 关断；死区为 0 时两路完全相同
 *   前提：ACTIVE_HIGH 必须为 true（false 会使两路电平整体反相）
 *
 * 边界钳位（硬件边界，非业务）：
 *   OCCR ticks : 0 .. periodTicks-1
 *   dead-time  : 0 .. TMR4_DEADTIME_TICKS_MAX
 *
 * 单实例：一个 TMR4_3 单元，三相 U/V/W
 *=============================================================================*/

/*=============================================================================
 * Compile-time configuration
 *=============================================================================*/
#define TMR4_SHADOW_CPSR_ENABLE   1U
#define TMR4_SHADOW_OCCR_ENABLE   1U
#define TMR4_SHADOW_OCMR_ENABLE   1U

/* TMR4 dead-time hardware field is 10 bits -> max 0x3FF ticks */
#define TMR4_DEADTIME_TICKS_MAX   0x3FFU

/* Legacy 0..10000 integer duty range (SetDuty / PinSetDuty) */
#define TMR4_PWM_DUTY_MAX         10000U

/*=============================================================================
 * Error codes
 *=============================================================================*/
typedef enum {
    TMR4_PWM_OK              = 0,
    TMR4_PWM_ERR_NOT_INIT    = 1,
    TMR4_PWM_ERR_BAD_PARAM   = 2,
    TMR4_PWM_ERR_FREQ        = 3,
    TMR4_PWM_ERR_STATE       = 4,
} tmr4_pwm_err_t;

/*=============================================================================
 * Types
 *=============================================================================*/
typedef enum {
    TMR4_OUTPUT_COMPLEMENTARY = 0,
    TMR4_OUTPUT_SYNC          = 1,
} tmr4_output_type_t;

/* 采样臂（语义：采哪一桥臂的导通区），由 cfg_foc_table.h 的采样点映射而来
 *   高侧导通区居中于峰值，低侧导通区居中于谷底；
 *   触发点只与采样臂有关，与输出模式（互补/同向）无关 */
typedef enum {
    TMR4_SAMPLE_HIGH_SIDE = 0,   /* 峰值触发：采上桥臂导通区 */
    TMR4_SAMPLE_LOW_SIDE  = 1,   /* 谷底触发：采下桥臂导通区 */
} tmr4_sample_point_t;

typedef enum {
    TMR4_CHANNEL_U = 0,
    TMR4_CHANNEL_V = 1,
    TMR4_CHANNEL_W = 2,
} tmr4_pwm_channel_t;

typedef enum {
    TMR4_MODE_OFF       = 0,
    TMR4_MODE_HIGH_SIDE = 1,
    TMR4_MODE_LOW_SIDE  = 2,
} tmr4_channel_mode_t;

typedef enum {
    TMR4_SAFE_HIZ     = 0,
    TMR4_SAFE_ALL_OFF = 1,
} tmr4_safe_state_t;

typedef struct {
    tmr4_output_type_t outputTypeU;
    tmr4_output_type_t outputTypeV;
    tmr4_output_type_t outputTypeW;
    uint32_t           freqHz;
    uint16_t           deadTimeNs;
    bool               activeHigh;
} tmr4_pwm_config_t;

typedef struct {
    uint16_t periodTicks;
    uint32_t freqHz;
    uint16_t deadTicks;
    uint16_t deadTimeNs;
    float    deadTimePct;
} tmr4_pwm_context_t;

/*=============================================================================
 * Config / start / stop / resume / all-low / safe state
 *=============================================================================*/
tmr4_pwm_err_t hw_tmr4_pwm_config(const tmr4_pwm_config_t *cfg);
tmr4_pwm_err_t hw_tmr4_pwm_start_output(void);
tmr4_pwm_err_t hw_tmr4_pwm_stop_output(void);

/* 从 stop / all_low 恢复：按 s_outputType 重新初始化 PWM 模式 + OC + Start
 * 用途：mode 0（全低）切回 mode 1（50%）或 mode 2（IF）时调用 */
tmr4_pwm_err_t hw_tmr4_pwm_resume_output(void);

/* 六路全低（mode 0 安全停机）：
 *   停计数 + 关 OC + 高低侧 invalid 都 LOW + PWM 模式改 THROUGH/HOLD
 *   注意：切回 mode 1/2 必须调 resume_output 恢复 PWM 模式 */
tmr4_pwm_err_t hw_tmr4_pwm_set_all_low(void);

tmr4_pwm_err_t hw_tmr4_pwm_set_safe_state(tmr4_safe_state_t state);

/*=============================================================================
 * Frequency / dead-time
 *=============================================================================*/
tmr4_pwm_err_t hw_tmr4_pwm_set_frequency(uint32_t freqHz);
tmr4_pwm_err_t hw_tmr4_pwm_set_deadtime(uint16_t deadTimeNs);

/*=============================================================================
 * Duty (center semantics)
 *   centerPct = 命令值 = 高侧占空比（互补/同向两种模式语义一致）
 *=============================================================================*/
tmr4_pwm_err_t hw_tmr4_pwm_set_duty(tmr4_pwm_channel_t channel, uint16_t dutyU16);
tmr4_pwm_err_t hw_tmr4_pwm_set_duty_float(tmr4_pwm_channel_t channel, float centerPct);
tmr4_pwm_err_t hw_tmr4_pwm_set_duty3(float duPct, float dvPct, float dwPct);

/* Dead-time-aware; 输入 = 每相高侧有效占空比 */
tmr4_pwm_err_t hw_tmr4_pwm_set_duty3_deadtime(float highU, float highV, float highW);

/*=============================================================================
 * ADC 采样触发事件（EVT0 → AOS → ADC）
 *   CFG_SAMPLE_UPPER：SCCR=峰值，上溢匹配
 *   CFG_SAMPLE_LOWER：SCCR=0，下溢匹配
 *   与输出模式无关；同向模式下上下管同相，“高侧/低侧采样”没有物理区分
 *=============================================================================*/
tmr4_pwm_err_t hw_tmr4_pwm_config_adc_trigger(bool enable);

/*=============================================================================
 * Read-back
 *=============================================================================*/
void     hw_tmr4_pwm_get_context(tmr4_pwm_context_t *out);
uint16_t hw_tmr4_pwm_get_period_ticks(void);
uint16_t hw_tmr4_pwm_get_dead_ticks(void);
uint32_t hw_tmr4_pwm_get_frequency(void);
uint16_t hw_tmr4_pwm_get_deadtime(void);
uint16_t hw_tmr4_pwm_get_occr(tmr4_pwm_channel_t channel, bool highSide);
uint16_t hw_tmr4_pwm_get_counter(void);
void     hw_tmr4_pwm_get_duty3(float *duPct, float *dvPct, float *dwPct);

typedef struct {
    float centerPct;
    float highPct;
    float lowPct;
    float deadPct;
} tmr4_pwm_state_t;

void hw_tmr4_pwm_get_state(tmr4_pwm_channel_t channel, tmr4_pwm_state_t *out);

/*=============================================================================
 * Dead-time numeric helpers (pure)
 *=============================================================================*/
void hw_tmr4_pwm_center_to_actual(tmr4_pwm_channel_t channel,
                                  float centerPct,
                                  float *outHighPct,
                                  float *outLowPct);

tmr4_pwm_err_t hw_tmr4_pwm_actual_duty_to_center(tmr4_pwm_channel_t channel,
                                                 float highPct,
                                                 float lowPct,
                                                 float *outCenterPct);

/*=============================================================================
 * Channel / pin control
 *=============================================================================*/
tmr4_pwm_err_t hw_tmr4_pwm_channel_cmd(tmr4_pwm_channel_t channel, bool enable);
tmr4_pwm_err_t hw_tmr4_pwm_pin_cmd(tmr4_pwm_channel_t channel, bool highSide, bool enable);
tmr4_pwm_err_t hw_tmr4_pwm_pin_set_duty(tmr4_pwm_channel_t channel, bool highSide, uint16_t dutyU16);
tmr4_pwm_err_t hw_tmr4_pwm_pin_set_invalid_level(tmr4_pwm_channel_t channel, bool highSide, bool levelHigh);
tmr4_pwm_err_t hw_tmr4_pwm_set_channel_mode(tmr4_pwm_channel_t channel,
                                            tmr4_channel_mode_t mode, float centerPct);
tmr4_pwm_err_t hw_tmr4_pwm_set_foc_mode(uint16_t deadTimeNs);
tmr4_pwm_err_t hw_tmr4_pwm_set_output_type(tmr4_pwm_channel_t channel, tmr4_output_type_t type);
tmr4_output_type_t hw_tmr4_pwm_get_output_type(tmr4_pwm_channel_t channel);

#endif /* __HW_TMR4_PWM_H__ */
