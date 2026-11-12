/*=============================================================================
 * foc_position_loop.h  —  L8 位置外环（减速包络，2ms，输出速度指令）
 *
 * 结构：
 *   位置环 2ms -> 速度指令 rpm -> 速度环 1ms -> iq_ref -> 电流环 20kHz
 *
 * 控制律（连续，无分段、无刹车）：
 *   err   = clamp(g_pos_target_rev, ±limit) - pos            [圈]
 *   v_max_allow = sqrt(120 × dec × |err|)                    [rpm]
 *   vcmd  = sign(err) × min(v_max_allow, g_pos_v_max_rpm)
 *   vcmd -= kd × 实测转速                                    （可选阻尼）
 *   if |err| <= deadband: vcmd = 0                           （到位保持）
 *   g_spd_target_rpm  = vcmd
 *   g_spd_iq_limit_ma = g_pos_iq_limit_ma
 *
 *   为什么不用纯 P：速度环有 5000rpm/s 的目标斜坡，2500rpm 时 P 律要求
 *   的减速是 kp×v/60，kp=800 时高达 33000rpm/s，斜坡跟不上就会冲过目标。
 *   sqrt 包络的减速恒等于 dec，斜坡刚好能跟住，大行程也不会过冲。
 *   包络同时也是一条梯形曲线：远处 min() 取 v_max 为匀速段，近处自动减速。
 *
 *   - 稳态由速度环积分保持 0 速，带载不下沉
 *   - 到位窗口内指令清零，避免编码器量化引起的抖动
 *
 * 位置反馈：
 *   展开的多圈机械位置 = g_enc_count × g_enc_elec_dir / CFG_ENCODER_CPR，单位"圈"。
 *   正方向与速度环一致，可超出一圈，适合丝杆行程。
 *   写 g_pos_zero_req=1 可把当前位置设为 0 并保持。
 *
 * 参数初值集中在下方 *_INIT 宏；运行时可在 Keil Watch 改 g_pos_* 变量。
 *=============================================================================*/

#ifndef FOC_POSITION_LOOP_H
#define FOC_POSITION_LOOP_H

#include <stdbool.h>
#include <stdint.h>

/*=============================================================================
 * 一、位置环参数初值
 *=============================================================================*/

/* 位置环执行周期：40 拍 = 2ms（20kHz ISR） */
#define POS_CTRL_DIV                (40u)

/* 规划减速度，rpm/s：sqrt 包络按这个值规划减速段。
 *   必须 ≤ g_spd_accel_rpm_s（代码里会自动取小），否则速度环斜坡跟不上会过冲。
 *   负载重、到位前有机械冲击时往下调（2000~3000） */
#define POS_DEC_RPM_S_INIT          (5000.0f)

/* 速度阻尼增益，rpm/rpm：位置环输出里减去 kd×实测转速。
 *   包络本身已限制减速，默认 0；到位有振荡时可加到 0.1~0.3 */
#define POS_KD_RPM_PER_RPM_INIT     (0.0f)

/* 位置环输出（速度指令）限幅，rpm */
#define POS_V_MAX_RPM_INIT          (2500.0f)

/* 位置模式速度环 iq 限幅，mA */
#define POS_IQ_LIMIT_MA_INIT        (3400.0f)

/* 到位窗口，圈：窗口内速度指令清零，靠速度环保持 0 速。
 *   不要设太小，否则编码器量化会让指令在 0 附近抖动；
 *   0.001 圈 = 4 counts */
#define POS_DEADBAND_REV_INIT       (0.03f)

/* 位置误差限幅，圈：仅用于 g_pos_err_rev 观测 */
#define POS_ERR_MAX_REV_INIT        (3.0f)

/* 软行程限位，圈（对称 ±） */
#define POS_LIMIT_REV_INIT          (1000000.0f)

/*=============================================================================
 * 二、可调参数变量（Watch 可改）
 *=============================================================================*/
extern volatile float   g_pos_target_rev;        /* 目标位置，圈 */
extern volatile uint8_t g_pos_zero_req;          /* 写 1 把当前位置设为 0 并保持 */

extern volatile float   g_pos_dec_rpm_s;         /* 规划减速度 rpm/s */
extern volatile float   g_pos_kd_rpm_per_rpm;    /* 速度阻尼 rpm/rpm，0 = 关闭 */
extern volatile float   g_pos_v_max_rpm;         /* 位置环输出速度限幅 rpm */
extern volatile float   g_pos_iq_limit_ma;       /* 位置模式速度环 iq 限幅 mA */
extern volatile float   g_pos_deadband_rev;      /* 到位窗口 圈 */
extern volatile float   g_pos_err_max_rev;       /* 误差观测限幅 圈 */
extern volatile float   g_pos_limit_rev;         /* 软行程限位 圈 */

/*=============================================================================
 * 三、观测量
 *=============================================================================*/
extern volatile uint8_t g_pos_active;      /* 1 = 电流环已 active，位置环闭环中 */
extern volatile uint8_t g_pos_in_pos;      /* 1 = |err| <= 到位窗口 */
extern volatile float   g_pos_fb_rev;      /* 当前位置，圈 */
extern volatile float   g_pos_err_rev;     /* 限幅后的位置误差，圈（观测用） */
extern volatile float   g_pos_cmd_rpm;     /* 位置环输出速度指令，rpm */

/*=============================================================================
 * 四、API
 *=============================================================================*/
void Foc_Position_Init(void);

/* 进入位置模式：以当前位置为目标（保持不动），清位置环状态 */
void Foc_Position_Reset(void);

/* 离开位置模式：清位置环输出与观测量 */
void Foc_Position_Stop(void);

/* 每拍调用（20kHz ISR，内部 2ms 抽稀） */
void Foc_Position_Task20k(void);

#endif /* FOC_POSITION_LOOP_H */
