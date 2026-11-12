/*=============================================================================
 * foc_speed_loop.h  —  L7 速度外环（5ms 滑窗测速 + 1ms 速度 PI，输出 iq_ref）
 *
 * 职责：
 *   - 20kHz ISR 内每 20 拍（1ms）做一次速度测量/滤波/斜坡/速度 PI
 *   - 测速用 5ms 滑动窗口（100 拍）累加，窗口每拍滚动、每 1ms 取一次和，
 *     分辨率与纯 5ms 累加相同（1 count = 2.93 rpm），但更新率提高 5 倍
 *   - 速度 PI 输出直接写电流环 iq_ref，id_ref 恒为 0
 *   - 电流环未 active（预热 / 在线零偏校准中）时冻结斜坡不积分，iq_ref=0
 *
 * 数据流：
 *   编码器增量 -> 5ms 滑窗(±36/拍限幅) -> raw rpm -> ×0.5 滤波
 *              -> 目标斜坡(5000 rpm/s) -> 速度 PI -> iq_ref(A) -> 电流环
 *
 * 量纲：
 *   PID 反馈/给定 rpm，kp 单位 mA/rpm，ki 单位 mA/rpm/s，输出 mA
 *   pid_ctrl 的 integral += err*dt，I = ki*integral，ki 按每秒量纲直接填
 *
 * 参数初值集中在下方 *_INIT 宏；运行时可在 Keil Watch 改 g_spd_* 变量
 *=============================================================================*/

#ifndef FOC_SPEED_LOOP_H
#define FOC_SPEED_LOOP_H

#include <stdbool.h>
#include <stdint.h>

#include "pid_ctrl.h"   /* PID_AW_* 抗饱和措施位掩码 */

/*=============================================================================
 * 一速度环参数初值
 *=============================================================================*/

/* 速度环执行周期：20 拍 = 1ms（20kHz ISR） */
#define SPD_CTRL_DIV                (20u)

/* 测速滑动窗口长度：100 拍 = 5ms（窗口每拍滚动，1ms 取一次和）
 * 1 count/5ms = 2.93 rpm，分辨率与纯 5ms 累加一致 */
#define SPD_WIN_TICKS               (100u)

/* 速度 PI（mA/rpm, mA/rpm/s）
 *   kp=5 对应约 5~15Hz 速度环带宽（取决于负载折算惯量），现场从 3 往上加到不振荡
 *   ki=100 对应积分时间 50ms，带载掉速后 0.5s 量级补回 */
#define SPD_KP_INIT                 (5.0f)
#define SPD_KI_INIT                 (100.0f)

/* I 项抗饱和措施位掩码初值：钳位 + 反计算
 *   与改造前行为一致：反计算系数为 0 时反计算无实际作用，实际只有钳位生效
 *   0 = 全部关闭；可按位或组合 PID_AW_CLAMP / PID_AW_BACK_CALC / PID_AW_CONDITIONAL */
#define SPD_AW_MASK_INIT            (PID_AW_CLAMP | PID_AW_BACK_CALC)
/* 条件积分饱和判据裕度，mA；0 = 严格按输出限幅判断 */
#define SPD_AW_COND_THR_INIT        (0.0f)

/* 速度环输出限幅（iq_ref 上限，mA）：17A × 20% = 3400mA */
#define SPD_IQ_LIMIT_MA_INIT        (3400.0f)

/* 积分项（ki*integral）限幅，mA
 *   带丝杆负载时零速/低速保持力矩需要较大积分储备，默认 2000mA
 *   pid_ctrl 会把 integral 反算回 i_term/ki，等价于积分状态限幅 */
#define SPD_ITERM_MAX_MA_INIT       (2000.0f)

/* 库仑摩擦前馈，mA：速度指令方向明确时在 iq_ref 上叠一个固定偏置，
 *   让低速/起步不必靠速度误差慢慢憋力矩，0 = 关闭 */
#define SPD_FF_MA_INIT              (200.0f)
#define SPD_FF_DEADBAND_RPM         (2.0f)

/* 目标加速度斜坡，rpm/s（1000 时每 1ms 步进 1 rpm） */
#define SPD_ACCEL_RPM_S_INIT        (1000.0f)

/* 目标转速限幅，rpm */
#define SPD_TARGET_MAX_RPM          (7800.0f)

/* 速度反馈 EMA 系数初值（1ms 更新；滑窗本身已平滑，只补一阶滞后）
 *   运行时改 g_spd_fb_alpha，下一拍生效；1 = 不滤波 */
#define SPD_FB_ALPHA                (0.5f)

/* 编码器单拍增量限幅，counts（20kHz） */
#define SPD_ENC_DELTA_CLAMP         (36)

/*=============================================================================
 * 二速度模式下的电流环参数初值
 *   与 mode 4/8 的 g_cl_* 分开存放，避免互相覆盖
 *=============================================================================*/
#define SPD_CUR_KP_INIT             (0.1f)     /* V/A */
#define SPD_CUR_KI_INIT             (300.0f)   /* V/A/s */
#define SPD_CUR_VMAX_INIT           (6.2f)     /* V */
#define SPD_CUR_ITERM_MAX_INIT      (6.0f)     /* V */
#define SPD_CUR_FB_ALPHA_INIT       (1.0f)     /* 1 = 电流反馈不做 EMA */

/*=============================================================================
 * 三可调参数变量（Watch 可改）
 *=============================================================================*/
extern volatile float   g_spd_target_rpm;      /* 目标转速，rpm */
extern volatile float   g_spd_kp;              /* mA/rpm */
extern volatile float   g_spd_ki;              /* mA/rpm/s */
extern volatile float   g_spd_accel_rpm_s;     /* rpm/s，0 = 无斜坡 */
extern volatile float   g_spd_iq_limit_ma;     /* 输出限幅 mA */
extern volatile float   g_spd_iterm_max_ma;    /* 积分项限幅 mA，0 = 不限 */
extern volatile float   g_spd_ff_ma;           /* 库仑摩擦前馈 mA，0 = 关闭 */
extern volatile float   g_spd_back_gain;       /* 反计算抗饱和回退系数，0 = 关闭 */
extern volatile float   g_spd_fb_alpha;        /* 速度反馈 EMA 系数，1 = 不滤波 */
extern volatile uint32_t g_spd_aw_mask;        /* I 项抗饱和措施位掩码 */
extern volatile float   g_spd_aw_cond_thr;     /* 条件积分饱和判据裕度 mA */

/* 速度模式电流环参数 */
extern volatile float   g_spd_cur_kp;
extern volatile float   g_spd_cur_ki;
extern volatile float   g_spd_cur_vmax_v;
extern volatile float   g_spd_cur_iterm_v;
extern volatile float   g_spd_cur_fb_alpha;

/*=============================================================================
 * 四观测量
 *=============================================================================*/
extern volatile uint8_t g_spd_active;     /* 1 = 电流环已 active，速度环闭环中 */
extern volatile float   g_spd_raw_rpm;    /* 5ms 原始测量 */
extern volatile float   g_spd_filt_rpm;   /* 速度 PI 反馈 */
extern volatile float   g_spd_ramp_rpm;   /* 斜坡后的目标 */
extern volatile float   g_spd_err_rpm;    /* 误差 = ramp - filt */
extern volatile float   g_spd_out_ma;     /* 速度 PI 输出（限幅后） */
extern volatile float   g_spd_iterm_ma;   /* 当前积分项输出（ki × integral） */

/*=============================================================================
 * 五API
 *=============================================================================*/
void Foc_Speed_Init(void);

/* 启动：目标清 0，清滤波/PI/斜坡，iq_ref 置 0 */
void Foc_Speed_Start(void);

/* 停止：目标清 0，清状态，iq_ref 置 0 */
void Foc_Speed_Stop(void);

/* 只清速度 PI 积分，不清斜坡/滤波/观测量。位置环切段平滑用 */
void Foc_Speed_ResetIntegral(void);

/* 每拍调用（20kHz ISR，内部 5ms 抽稀） */
void Foc_Speed_Task20k(void);

/* 下发速度模式电流环参数（增益 / vmax / 积分项限幅 / 反馈 EMA），并关闭 dq 前馈 */
void Foc_Speed_ConfigCurrentLoop(void);

#endif /* FOC_SPEED_LOOP_H */
