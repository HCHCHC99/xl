/**
 *******************************************************************************
 * @file  foc_output.h
 * @brief FOC 运行期桥臂输出的唯一入口
 *
 *        约定：本模块是 FOC 运行期唯一直接操作 TMR4_3 桥臂与 EMB 的地方。
 *        模式层（foc_run）、流程层（foc_proc）、标定层（encoder_calib）、
 *        保护层（foc_protect）、控制宿主（foc_control）都只调这里的函数。
 *
 *        返回码统一为 int：0 = 成功，-1 = 失败（内部把 tmr4_pwm_err_t 归一化）。
 *
 *        注意：本阶段（Phase 4-lite）只做访问点收敛，不改变任何输出策略、
 *        数值常量或调用顺序。
 *******************************************************************************
 */

#ifndef __FOC_OUTPUT_H__
#define __FOC_OUTPUT_H__

#ifdef __cplusplus
extern "C" {
#endif

/* 本拍要执行的输出动作（由模式层 Foc_Run_Step20k 下发） */
typedef enum {
    FOC_OUT_NONE = 0,   /* 本拍不改变输出（执行器跳过） */
    FOC_OUT_VECTOR,     /* 用 vd/vq/theta_e 算 SVPWM */
    FOC_OUT_DUTY3,      /* 直接写三相中心占空比（LOCK / 静态矢量） */
    FOC_OUT_ALL_LOW,    /* 全低 + 抱闸（安全停机态） */
} foc_out_kind_t;

/* 上电默认输出：全低 + 抱闸 */
void Foc_Output_Init(void);

/* 六路全低（不抱闸） */
int  Foc_Output_AllLow(void);

/* 恢复 PWM 输出（把桥臂从全低状态切回比较输出） */
int  Foc_Output_Resume(void);

/* 三相中心占空比，单位 %（0~100） */
int  Foc_Output_Duty3(float du, float dv, float dw);

/* 抱闸：置 EMB 功率级封锁（BSP_PWM_EMB_ENABLE=0 时为等价空操作） */
void Foc_Output_Brake(void);

/* 解除抱闸 */
void Foc_Output_Release(void);

/* 唯一安全停机入口：全低 + 抱闸 */
void Foc_Output_SafeStop(void);

#ifdef __cplusplus
}
#endif

#endif /* __FOC_OUTPUT_H__ */