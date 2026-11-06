/*=============================================================================
 * foc_current_loop.h  —  L6.5 电流闭环层（力矩模式，有感）
 *
 * 职责：
 *   - 持有 d/q 两路 PID，输入 id_ref/iq_ref 与反馈 g_fc_id/g_fc_iq
 *   - 读取编码器电角度（已扣对齐 offset），产出 vd/vq/theta_e
 *   - ref 斜坡（缓启动/缓停止），防电流阶跃冲击
 *   - 进入预热：进 CURRENT 后前 N 拍强制零输出，等 PWM/反馈稳定
 *   - 判断"物理受限"（自身限幅 / 电压不足 / 过流保护），对外输出 cur_sat
 *
 * 契约：
 *   - 本模块只做"给定→斜坡→PID→电压"，不碰 PWM/模式/保护
 *   - PID 输出限幅 ±(vbus/√3 * 0.9)，单位 V
 *   - 必须在 Foc_Control_Init() 之后、进入 CURRENT 之前调 Init
 *   - 依赖 g_motor_is_aligned（电角度已对齐），但本模块不检查，
 *     由 main 的入口检查负责
 *
 * 参数修改说明：
 *   所有可调参数的【初值】集中在下方 *_INIT 宏，改这里即可，编译后生效；
 *   运行时可在 Keil Watch 里改 g_cl_* 变量，立即或下次 Reset 生效：
 *     g_cl_ref_slew      A/拍，下一拍生效，0 关闭斜坡
 *     g_cl_vq_slew       V/拍，下一拍生效，0 关闭
 *     g_cl_warmup_ticks  拍，下次 Reset 生效
 *     g_cl_iref_max      A，下一次 SetRef 生效
 *     g_cl_vmax_watch    V，主循环检测变化后生效
 *=============================================================================*/

#ifndef FOC_CURRENT_LOOP_H
#define FOC_CURRENT_LOOP_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*=============================================================================
 * 一、可调参数初值（改这里，编译后成为变量初值）
 *=============================================================================*/

/* ref 斜坡：每拍 ref 最大变化量（A/拍）。
 *   20kHz 下 0.001A/拍 = 20A/s；0→1A 需 50ms。
 *   推杆可更慢：0.0005A/拍 = 10A/s；设 0 关闭斜坡。 */
#define CL_REF_SLEW_INIT      0.0001f

/* 输出斜率限制：每拍 vq 最大变化量（V/拍）。
 *   20kHz 下 0.005V/拍 = 100V/s；设 0 关闭。 */
#define CL_VQ_SLEW_INIT       0.005f

/* 进入预热拍数：100 拍 = 5ms，期间强制 vd=vq=0。
 *   运行中改只影响下一次 Foc_CurrentLoop_Reset 后的进入。 */
#define CL_WARMUP_INIT        4200u

/* 目标电流 clamp 上限（A），SetRef 内部用 */
#define CL_IREF_MAX_INIT      50.0f

/* 物理电压上限（V），默认 12/√3≈6.928V。
 *   主循环同步到 Foc_CurrentLoop_SetVmax() 生效。 */
#define CL_VMAX_INIT          3.9f//6.9282032f

/* PID 初值（保守，先能稳住再往上调；ki 量纲见 pid_ctrl.h 提示） */
#define CL_KP_INIT            0.1f
#define CL_KI_INIT            10.0f
#define CL_KD_INIT            0.0f
#define CL_DIFF_ALPHA_INIT    0.2f
#define CL_BACK_GAIN_INIT     0.8f

/*=============================================================================
 * 二、可调参数变量声明（初值在 .c 由上面的宏赋）
 *
 * 注意：extern 声明不带初值，避免多文件重复定义。Watch 可见、可改。
 *=============================================================================*/
extern volatile float    g_cl_ref_slew;      /* A/拍，下一拍生效 */
extern volatile float    g_cl_vq_slew;       /* V/拍，下一拍生效 */
extern volatile uint16_t g_cl_warmup_ticks;  /* 拍，下次 Reset 生效 */
extern volatile float    g_cl_iref_max;      /* A，下一次 SetRef 生效 */
extern volatile float    g_cl_vmax_watch;    /* V，主循环检测变化后生效 */

/*=============================================================================
 * 三、受限原因枚举
 *=============================================================================*/
typedef enum {
    CL_LIMIT_NONE     = 0,   /* 正常，未受限 */
    CL_LIMIT_SELF     = 1,   /* PID 输出撞自身限幅（±vlim） */
    CL_LIMIT_VOLTAGE  = 2,   /* vmag_raw > vmax，物理电压不够 */
    CL_LIMIT_PROTECT  = 3,   /* 过流保护非 NORMAL（TRIP/LATCHED） */
} cl_limit_reason_t;

/*=============================================================================
 * 四、API
 *=============================================================================*/
void Foc_CurrentLoop_Init(void);
void Foc_CurrentLoop_Reset(void);
void Foc_CurrentLoop_SetRef(float id_ref_a, float iq_ref_a);
void Foc_CurrentLoop_SetGains(float kp, float ki, float kd);
void Foc_CurrentLoop_SetVmax(float vmax_v);
void Foc_CurrentLoop_Step(float *vd, float *vq, float *theta_e);

bool Foc_CurrentLoop_IsReady(void);
bool Foc_CurrentLoop_IsLimited(void);
cl_limit_reason_t Foc_CurrentLoop_GetLimitReason(void);
uint16_t Foc_CurrentLoop_GetSatCnt(void);
float Foc_CurrentLoop_GetVmagRaw(void);
float Foc_CurrentLoop_GetVmax(void);
float Foc_CurrentLoop_GetIdRefRamped(void);
float Foc_CurrentLoop_GetIqRefRamped(void);

#ifdef __cplusplus
}
#endif

#endif /* FOC_CURRENT_LOOP_H */
