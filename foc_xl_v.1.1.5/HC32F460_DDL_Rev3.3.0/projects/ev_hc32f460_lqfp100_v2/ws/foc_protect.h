/*=============================================================================
 * foc_protect.h  —  过流保护层（两层窗口 + 恢复状态机，无堵转）
 *
 * 设计要点：
 *   - 20kHz EOCA 只搬运：Foc_Protect_PushIsr() 写两个环形缓冲区，不判断
 *   - 1ms 中断集中判断：Foc_Protect_Task1ms() 读缓冲区做快/慢判断
 *   - 触发动作复用现有 IDLE 全低机制：hw_tmr4_pwm_set_all_low + Foc_Mode_SetMode
 *   - 保护参数全部宏定义在本文件，改阈值只改这里
 *
 * 接入点：
 *   1) foc_control.c  Foc_Control_Step() 里 g_fc_iw=s.iw 之后 -> PushIsr
 *   2) Timer0_Unit2.c My_1ms_Task() 里 -> Foc_Protect_Task1ms
 *   3) main.c Foc_Mode_Init() 之后、Foc_Control_Init() 之前 -> Foc_Protect_Init
 *=============================================================================*/

#ifndef FOC_PROTECT_H
#define FOC_PROTECT_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*=============================================================================
 * 一、缓冲区长度（编译期固定，必须是 2 的幂，用掩码回绕）
 *=============================================================================*/
#define FP_FAST_LEN             32u     /* 20kHz  1.6ms */
#define FP_SLOW_LEN             256u    /* 20kHz  12.8ms */

/*=============================================================================
 * 二、保护阈值与时间参数（全部宏，改这里即可）
 *
 *   阈值起点按方案第九节建议：
 *     快层 = 硬件极限 80%
 *     慢层 = 电机额定 1.5 倍
 *     恢复 = 额定 1/4
 *   实际值需按 MOS 额定 / 电机额定 / 采样量程（满量程 62.5A）重新标定
 *=============================================================================*/

/* ---- 快层：短时冲击，窗口内峰值超限即触发 ---- */
#define FP_I_FAST_TRIP_A        999.0f   /* A，快层跳闸阈值 */

/* ---- 慢层：持续过载，窗口平均超限 + 连续确认 ---- */
#define FP_I_SLOW_TRIP_A        999.0f    /* A，慢层跳闸阈值 */
#define FP_SLOW_CONFIRM_MS      500u    /* ms，慢层连续确认时间 */

/* ---- 恢复 ---- */
#define FP_I_RECOVER_A          1.5f    /* A，恢复阈值（电流回落到此以下开始计时） */
#define FP_RECOVER_DELAY_MS     200u    /* ms，恢复延时 */

/* ---- 锁存 ---- */
#define FP_LATCH_AFTER_TRIPS    3u      /* 累计触发次数达到则进 LATCHED */

/* ---- LATCHED 自动恢复（新增）----
 *   0 = 关闭：LATCHED 后必须手动 Foc_Protect_ClearFault()
 *   >0 = 开启：LATCHED 后若电流持续 < FP_I_RECOVER_A 达到该 ms 数，
 *        自动清故障回 NORMAL（等价于自动调一次 ClearFault）
 *   注意：自动恢复默认清 fault/state、保留 trip_cnt 做诊断；
 *         需要连 trip_cnt 一起清见 .c 内注释
 */
#define FP_LATCH_AUTO_RECOVER_MS   2000u

/* ---- 启动屏蔽：上电后前 N ms 不判断（防上电电流跳变误跳） ---- */
#define FP_STARTUP_BLANK_MS     150u    /* ms，> 校准 100ms + 余量 */

/*=============================================================================
 * 三、故障码（位掩码）
 *=============================================================================*/
#define FOC_FAULT_NONE          0x00000000UL
#define FOC_FAULT_OC_FAST       0x00000001UL   /* 快层过流 */
#define FOC_FAULT_OC_SLOW       0x00000002UL   /* 慢层过流 */
/* 预留 */
#define FOC_FAULT_OV            0x00000010UL
#define FOC_FAULT_UV            0x00000020UL
#define FOC_FAULT_OT            0x00000040UL

/*=============================================================================
 * 四、状态机
 *=============================================================================*/
typedef enum {
    FP_STATE_NORMAL  = 0,   /* 正常 */
    FP_STATE_TRIP    = 1,   /* 已跳闸，等待恢复 */
    FP_STATE_LATCHED = 2,   /* 多次触发后锁存，需手动复位或自动恢复 */
} fp_state_t;

/*=============================================================================
 * 五、配置结构体（保留给未来运行期改参数用；当前阶段参数全走宏，
 *     本结构体只作为 Foc_Protect_Init 的入参占位，可传 NULL）
 *=============================================================================*/
typedef struct {
    float    i_fast_trip_a;
    float    i_slow_trip_a;
    float    i_recover_a;
    uint16_t slow_confirm_ms;
    uint16_t recover_delay_ms;
    uint8_t  latch_after_trips;
} FocProtectCfg_t;

/*=============================================================================
 * 六、API
 *=============================================================================*/

/* 用宏默认值填 cfg（可选，不填则 Init 内部直接用宏） */
void Foc_Protect_GetDefaultCfg(FocProtectCfg_t *cfg);

/* 初始化：清缓冲区、清状态、记录启动时刻；cfg 可传 NULL，表示全用宏 */
void Foc_Protect_Init(const FocProtectCfg_t *cfg);

/* 手动使能/禁用保护（默认 Init 后由启动屏蔽自动使能） */
void Foc_Protect_Enable(void);
void Foc_Protect_Disable(void);

/* 20kHz EOCA 里调用：只搬运，不判断 */
void Foc_Protect_PushIsr(float iu, float iv, float iw);

/* 1ms 中断里调用：集中判断 */
void Foc_Protect_Task1ms(void);

/* 手动复位（LATCHED -> NORMAL），同时清故障码和触发计数 */
void Foc_Protect_ClearFault(void);

/*=============================================================================
 * 七、Watch 观测量（全部 volatile）
 *=============================================================================*/
extern volatile fp_state_t g_fp_state;      /* 当前状态 */
extern volatile uint32_t   g_fp_fault;      /* 当前故障码（位掩码） */
extern volatile float      g_fp_fast_max;   /* 快层最近峰值 (A) */
extern volatile float      g_fp_slow_avg;   /* 慢层最近平均 (A) */
extern volatile uint32_t   g_fp_trip_cnt;   /* 累计触发次数 */

#ifdef __cplusplus
}
#endif

#endif /* FOC_PROTECT_H */
