/**
 *******************************************************************************
 * @file  foc_protect.h
 * @brief 过流保护层（两层窗口 + 恢复状态机）
 *
 *        20kHz 只搬运（PushIsr 写环形缓冲），1ms 集中判断（Task1ms）
 *******************************************************************************
 */

#ifndef FOC_PROTECT_H
#define FOC_PROTECT_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 缓冲区长度（必须是 2 的幂） */
#define FP_FAST_LEN             32u     /* 20kHz  1.6ms */
#define FP_SLOW_LEN             256u    /* 20kHz  12.8ms */

/* 快层：短时冲击，窗口内峰值超限即触发 */
#define FP_I_FAST_TRIP_A        999.0f

/* 慢层：持续过载，窗口平均超限 + 连续确认 */
#define FP_I_SLOW_TRIP_A        999.0f
#define FP_SLOW_CONFIRM_MS      500u

/* 恢复 */
#define FP_I_RECOVER_A          1.5f
#define FP_RECOVER_DELAY_MS     200u

/* 锁存 */
#define FP_LATCH_AFTER_TRIPS    3u

/* LATCHED 自动恢复：0 = 必须手动清故障 */
#define FP_LATCH_AUTO_RECOVER_MS   2000u

/* 启动屏蔽：上电后前 N ms 不判断 */
#define FP_STARTUP_BLANK_MS     150u

/* 故障码（位掩码） */
#define FOC_FAULT_NONE          0x00000000UL
#define FOC_FAULT_OC_FAST       0x00000001UL
#define FOC_FAULT_OC_SLOW       0x00000002UL
#define FOC_FAULT_OV            0x00000010UL
#define FOC_FAULT_UV            0x00000020UL
#define FOC_FAULT_OT            0x00000040UL

typedef enum {
    FP_STATE_NORMAL  = 0,
    FP_STATE_TRIP    = 1,
    FP_STATE_LATCHED = 2,
} fp_state_t;

typedef struct {
    float    i_fast_trip_a;
    float    i_slow_trip_a;
    float    i_recover_a;
    uint16_t slow_confirm_ms;
    uint16_t recover_delay_ms;
    uint8_t  latch_after_trips;
} FocProtectCfg_t;

void Foc_Protect_GetDefaultCfg(FocProtectCfg_t *cfg);
void Foc_Protect_Init(const FocProtectCfg_t *cfg);
void Foc_Protect_Enable(void);
void Foc_Protect_Disable(void);
void Foc_Protect_PushIsr(float iu, float iv, float iw);
void Foc_Protect_Task1ms(void);
void Foc_Protect_ClearFault(void);

extern volatile fp_state_t g_fp_state;
extern volatile uint32_t   g_fp_fault;
extern volatile float      g_fp_fast_max;
extern volatile float      g_fp_slow_avg;
extern volatile uint32_t   g_fp_trip_cnt;

#ifdef __cplusplus
}
#endif

#endif /* FOC_PROTECT_H */
