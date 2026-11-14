/*=============================================================================
 * motor_sampling.h  —  L1 采样层（相电流 + 母线电压，TMR4 谷底触发）
 *
 * 硬件（来自 hw_board_pins.h）：
 *   PA3 = CH3  itotal  总电流
 *   PA4 = CH4  iw      W 相电流
 *   PA5 = CH5  iv      V 相电流
 *   PA6 = CH6  vbus    母线电压
 *   U 相电流由 iu = 0 - iw - iv 计算，不占 ADC 通道
 *
 * 职责边界：
 *   - ADC 序列配置 + TMR4 EVT0 触发（AOS 触发源在此绑定）
 *   - EOCA 回调里读取原始 ADC 值累加校准数据
 *   - 零偏校准（阻塞 / 非阻塞）
 *   - 电角度对齐已迁至 encoder_calib.h（本层不再涉及位置域标定）
 *   - 过流保护的 PushIsr / Task1ms 不在本层，属 L7 foc_protect
 *=============================================================================*/

#ifndef MOTOR_SAMPLING_H
#define MOTOR_SAMPLING_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float i_total;  /* A，母线电流（仅板型 0 有效，板型 1/2 恒为 0） */
    float iu;       /* A，U 相电流（板型 1 用 PA3 直接采样；板型 0/2 由 0 - iw - iv 计算） */
    float iv;       /* A，V 相电流（PA5） */
    float iw;       /* A，W 相电流（PA4） */
    float vbus;     /* V，母线电压（PA6） */
    uint32_t seq;   /* 采样序号，每次 EOCA +1 */
} MotorSample_t;

/* 定点快路径采样：相电流 Q15 标幺（1.0 = FX_I_BASE_A） */
typedef struct {
    int16_t  iu;    /* Q15 pu */
    int16_t  iv;    /* Q15 pu */
    int16_t  iw;    /* Q15 pu */
    float    vbus;  /* V，暂保留浮点 */
    uint32_t seq;
} MotorSampleQ15_t;

/* 标定系数：ADC 原始值 → 物理量
 *   current = (adc_raw - offset) * current_scale
 *   vbus    = adc_raw * vbus_scale
 */
typedef struct {
    float current_scale;   /* A / LSB */
    float current_offset[3];  /* LSB，零电流偏置 [0]=PA3(iu/itotal) [1]=iw(PA4) [2]=iv(PA5)，上电校准后更新 */
    float vbus_scale;      /* V / LSB */
    uint16_t vbus_ch;      /* 母线电压 ADC 通道 */
} MotorSamplingScale_t;

/* 初始化：配置 ADC1 序列A + TMR4 事件触发
 *   注意：必须在 hw_tmr4_pwm_config / hw_tmr4_pwm_start_output 之前调用
 *        （AOS 触发源 EVT_SRC_TMR4_3_SCMP0 依赖 TMR4 外设已使能，
 *          PWM config 内部还会再调 hw_tmr4_pwm_config_adc_trigger 覆盖 EVT0 配置）
 *   本模块不注册中断，EOCA 由 foc_control.c 注册 */
void Motor_Sampling_Init(const MotorSamplingScale_t *scale);

/* EOCA 中断里要执行的采集动作（由 foc_control.c 的复合回调调用） */
void Motor_Sampling_EocaCallback(void);

/* 取最近一帧采样（无临界区，可在 ISR 内调用） */
void Motor_GetSample(MotorSample_t *s);

/* 取最近一帧采样的 Q15 版本（定点快路径用，ISR 内可调用） */
void Motor_GetSampleQ15(MotorSampleQ15_t *s);

/* 查询是否有新帧（读取后自动清除） */
bool Motor_Sampling_HasNew(void);

/*---------------------------------------------------------------------------
 * 上电零电流偏置校准
 *---------------------------------------------------------------------------*/
void Motor_Sampling_StartOffsetCal(void);
bool Motor_Sampling_IsOffsetCalDone(void);
void Motor_Sampling_ApplyOffsetCal(void);

/* 非打印版：校准完成后静默更新零偏，可在 ISR 内调用。
 * 返回 true = 本次已应用（或应用成功）。 */
bool Motor_Sampling_ApplyOffsetCalSilent(void);

/* 
 * 阻塞式零电流校准：包含 启动校准等待采样满应用校准值超时保护
 * timeout_ms: 超时时间（建议 200ms，理论需要 100ms）
 * 返回: true = 校准成功, false = 超时失败
 */
bool Motor_Sampling_DoOffsetCalBlocking(uint32_t timeout_ms);

/* 校准结果观测量（Watch 可看） */
extern volatile uint16_t g_cal_offset_itotal;  /* 校准后 itotal 零电流码值 */
extern volatile uint16_t g_cal_offset_iw;      /* 校准后 iw 零电流码值 */
extern volatile uint16_t g_cal_offset_iv;      /* 校准后 iv 零电流码值 */
extern volatile uint32_t g_cal_sample_cnt;     /* 已累加点数 */

/* 全局校准状态标志 */
extern volatile uint8_t g_motor_is_offset_calibrated; /* 1 = 零偏已校准, 0 = 未校准 */
/* 注：g_motor_is_aligned 已迁至 encoder_calib.h */

#endif /* MOTOR_SAMPLING_H */
