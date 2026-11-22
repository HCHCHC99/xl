/**
 *******************************************************************************
 * @file  foc_run.h
 * @brief FOC 运行模式层（IDLE / LOCK / IF / CURRENT / SPEED / POSITION）
 *
 *        只包含「进入后保持」的持续运行模式，编号 0..5，无空洞。
 *        一次性流程（编码器对齐/扫角/测圈/斜坡对齐/零偏校准）在 foc_proc.h，
 *        不再占用运行模式编号，也不再出现在本文件的枚举里。
 *******************************************************************************
 */

#ifndef __FOC_RUN_H__
#define __FOC_RUN_H__

#include <stdbool.h>
#include <stdint.h>

#include "foc_output.h"   /* foc_out_kind_t：每拍输出动作 */

/* 1 = FOC 独占 TMR4_3：禁用旧六步路径的周期更新、占空比/使能/刹车写入 */
#ifndef FOC_DISABLE_LEGACY_6STEP
#define FOC_DISABLE_LEGACY_6STEP    1
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    FOC_RUN_IDLE = 0,   /* 全低电平，安全停机 */
    FOC_RUN_LOCK,       /* 零矢量锁轴，默认 2%，可用 focmode 1 <duty> 修改 */
    FOC_RUN_IF,         /* q 轴加压 + 角度自增，开环拖动 */
    FOC_RUN_CURRENT,    /* 有感闭环电流环 */
    FOC_RUN_SPEED,      /* 速度外环 + 电流环 */
    FOC_RUN_POSITION,   /* 位置外环 + 速度环 + 电流环 */
} foc_run_mode_t;

/* 每拍产出的控制指令：模式层决定「这一拍输出什么」，执行器只负责落实。
 * kind = VECTOR 时用 vd/vq/theta_e；kind = DUTY3 时用 duty_u/v/w；
 * kind = ALL_LOW 时忽略其余字段。 */
typedef struct {
    foc_out_kind_t kind;
    float vd;
    float vq;
    float theta_e;
    float duty_u;
    float duty_v;
    float duty_w;
} FocCmd_t;

void           Foc_Run_Init(void);
void           Foc_Run_SetMode(foc_run_mode_t mode);
foc_run_mode_t Foc_Run_GetMode(void);

/* 模式名（调试/CLI 用） */
const char *Foc_Run_Name(foc_run_mode_t mode);

/* 兼容映射：运行模式 -> 原 focmode 用户命令编号（0/1/2/4/9/10）。
 * 只用于日志与 CLI 显示；内部逻辑一律使用 foc_run_mode_t。 */
int Foc_Run_LegacyId(foc_run_mode_t mode);

/* mode 4 电流指令缓存：CLI 与核心共享的访问接口 */
float Foc_Run_GetCurrentCmdId(void);
float Foc_Run_GetCurrentCmdIq(void);
void  Foc_Run_SetCurrentCmd(float id, float iq);
void  Foc_Run_ReapplyCurrentCmd(void);

/* IF 参数设置（不改变当前模式） */
void Foc_Run_SetOpenloop(float freq_hz, float vq_volts, float vd_volts);
void Foc_Run_StartOpenloop(void);
void Foc_Run_Stop(void);

/* 每拍调用（20kHz ISR） */
void Foc_Run_Step20k(float ts, FocCmd_t *out);

/* LOCK 模式占空比（%）；若当前已在 LOCK，立即生效 */
void       Foc_Run_SetLockDuty(float duty_pct);
float      Foc_Run_GetLockDuty(void);

/* 1 = FOC 已接管 PWM；旧六步路径应停止写占空比 */
int        Foc_Run_PwmOwned(void);

extern volatile foc_run_mode_t g_fm_mode;
extern volatile float          g_fm_theta_e;
extern volatile float          g_fm_freq_hz;
extern volatile float          g_fm_vq;
extern volatile float          g_fm_vd;
extern volatile uint32_t       g_fm_step_cnt;

#ifdef __cplusplus
}
#endif

#endif /* __FOC_RUN_H__ */