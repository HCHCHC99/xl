/**
 *******************************************************************************
 * @file  foc_proc.h
 * @brief FOC 一次性流程层（编码器对齐 / 扫角 / 测圈 / 斜坡对齐 / 零偏校准）
 *
 *        这些流程做完就结束，不占用运行模式编号：
 *          focmode 3/5/6/7/12/13 在 CLI 层映射到 Foc_Proc_Start()。
 *        流程执行期间 foc_run 的运行模式保持 IDLE，必须用
 *        Foc_Proc_GetActive() / Foc_Proc_IsBusy() 判断是否在标定。
 *******************************************************************************
 */

#ifndef __FOC_PROC_H__
#define __FOC_PROC_H__

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    FOC_PROC_NONE = 0,      /* 无流程（供 GetActive 使用） */
    FOC_PROC_ENC_ALIGN,     /* 编码器电角度对齐（旧 focmode 3） */
    FOC_PROC_SWEEP,         /* 扫角测量（旧 focmode 5） */
    FOC_PROC_REV,           /* 一圈计数测量（旧 focmode 6） */
    FOC_PROC_ALIGN7,        /* 斜坡对齐（旧 focmode 7） */
    FOC_PROC_OFFSET_NOPWM,  /* 无 PWM 电流零偏校准（旧 focmode 12） */
    FOC_PROC_OFFSET_PWM50,  /* PWM 50% 零矢量电流零偏校准（旧 focmode 13） */
} foc_proc_t;

/* 启动一次性流程。内部先收回运行模式（非 IDLE 时切 IDLE）并中止上一个流程。 */
void Foc_Proc_Start(foc_proc_t proc);

/* 20kHz 入口：1 = 本拍被流程占用，调用方应直接 return（不跑运行模式控制链）。 */
uint8_t Foc_Proc_Task20k(void);

/* 1ms 入口：推进标定状态机并回收已完成的流程。 */
void Foc_Proc_Poll1ms(void);

/* 中止当前流程（切运行模式、过流 trip 时调用）。 */
void Foc_Proc_Abort(void);

bool       Foc_Proc_IsBusy(void);
foc_proc_t Foc_Proc_GetActive(void);

/* 流程名（调试/CLI 用） */
const char *Foc_Proc_Name(foc_proc_t proc);

/* 兼容映射：流程 -> 原 focmode 用户命令编号（3/5/6/7/12/13），只用于日志/CLI。 */
int Foc_Proc_LegacyId(foc_proc_t proc);

/* 上电一次电流零偏校准（无 PWM，阻塞约 100ms，结果只写 RAM）
 * 由启动表在 foc_ctrl 步骤内调用；不改变运行模式（默认 IDLE）。 */
void Foc_Proc_PowerOnCal(void);

#ifdef __cplusplus
}
#endif

#endif /* __FOC_PROC_H__ */