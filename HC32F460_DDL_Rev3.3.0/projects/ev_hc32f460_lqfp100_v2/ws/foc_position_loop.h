/*=============================================================================
 * foc_position_loop.h  —  L8 位置外环（10ms 位置 PI，输出速度指令）
 *
 * 结构：
 *   位置环 10ms -> 速度指令 rpm -> 速度环 5ms -> iq_ref -> 电流环 20kHz
 *
 * 位置反馈：
 *   展开的多圈机械位置 = g_enc_count × g_enc_elec_dir / CFG_ENCODER_CPR，单位“圈”。
 *   正方向与速度环一致，可超出一圈，适合丝杆行程。
 *   写 g_pos_zero_req=1 可把当前位置设为 0 并保持。
 *
 * 位置环控制律：
 *   err = target - pos，先限幅 ±g_pos_err_max_rev
 *   speed_cmd = kp × err + kd × d(-pos)/dt，输出限幅 ±g_pos_speed_limit_rpm
 *   输出直接写 g_spd_target_rpm，由速度环的 1950 rpm/s 斜坡平滑
 *
 * 参数初值集中在下方 *_INIT 宏；运行时可在 Keil Watch 改 g_pos_* 变量。
 *=============================================================================*/

#ifndef FOC_POSITION_LOOP_H
#define FOC_POSITION_LOOP_H

#include <stdbool.h>
#include <stdint.h>

/*=============================================================================
 * 一、位置环参数初值（保守启动值）
 *=============================================================================*/

/* 位置环执行周期：200 拍 = 10ms（20kHz ISR） */
#define POS_CTRL_DIV                (200u)

/* 位置 PD，输出单位 rpm，误差单位 圈
 *   速度内环自带积分，位置环用 PD 即可做到稳态无静差；
 *   D 项对位置反馈微分（微分先行），等效为速度阻尼。 */
#define POS_KP_INIT                 (60.0f)    /* rpm/rev */
#define POS_KI_INIT                 (0.0f)     /* rpm/rev/s，PD 默认关闭 */
#define POS_KD_INIT                 (20.0f)    /* rpm/(rev/s) */

/* 速度指令限幅，rpm（远距离段的最大接近速度） */
#define POS_SPEED_LIMIT_RPM_INIT    (300.0f)

/* 减速距离，圈：误差大于此值时按速度限幅全速接近，
 * 小于后按 (速度限幅 / 减速距离) 的比例线性减速。
 * 该值 > 0 时等效位置增益 kp = 速度限幅 / 减速距离，g_pos_kp 不再参与；
 * 设为 0 则退回纯 PD（用 g_pos_kp）。 */
#define POS_DECEL_REV_INIT          (5.0f)

/* 积分项（ki*integral）限幅，rpm */
#define POS_ITERM_MAX_RPM_INIT      (100.0f)

/* 位置误差限幅，圈（同时限制最大速度指令） */
#define POS_ERR_MAX_REV_INIT        (3.0f)

/* 位置死区，圈（约 8 counts）。带爬行速度后要略大于刹车距离 */
#define POS_DEADBAND_REV_INIT       (0.03f)

/* 爬行下限：误差超出死区且实际速度接近 0 时，强制最低速度，防静摩擦停住 */
#define POS_CREEP_RPM_INIT          (50.0f)
#define POS_STALL_RPM_INIT          (20.0f)    /* 实际速度低于此值才允许强制爬行 */
#define POS_CREEP_EN_INIT           (1u)       /* 1 = 开启爬行下限 */

/* 到位刹车：误差进入死区且实际速度低于门限后，输出三相零矢量短路制动；
 *   误差超出（死区 + 滞环）后退出刹车，恢复位置闭环。 */
#define POS_BRAKE_EN_INIT           (1u)
#define POS_BRAKE_SPEED_RPM_INIT    (5.0f)
#define POS_BRAKE_HYST_REV_INIT     (0.02f)

/* 软行程限位，圈（对称 ±） */
#define POS_LIMIT_REV_INIT          (100.0f)

/* 反计算抗饱和回退系数 */
#define POS_BACK_GAIN_INIT          (0.5f)

/*=============================================================================
 * 二、可调参数变量（Watch 可改）
 *=============================================================================*/
extern volatile float   g_pos_target_rev;        /* 目标位置，圈 */
extern volatile float   g_pos_kp;                /* rpm/rev */
extern volatile float   g_pos_ki;                /* rpm/rev/s，0 = 纯 PD */
extern volatile float   g_pos_kd;                /* rpm/(rev/s)，位置反馈微分阻尼 */
extern volatile float   g_pos_speed_limit_rpm;   /* 速度指令限幅 rpm */
extern volatile float   g_pos_decel_rev;         /* 减速距离 圈，0 = 用 g_pos_kp */
extern volatile float   g_pos_iterm_max_rpm;     /* 积分项限幅 rpm */
extern volatile float   g_pos_err_max_rev;       /* 误差限幅 圈 */
extern volatile float   g_pos_deadband_rev;      /* 位置死区 圈 */
extern volatile float   g_pos_creep_rpm;         /* 爬行下限 rpm */
extern volatile float   g_pos_stall_rpm;         /* 爬行生效的实际速度门限 rpm */
extern volatile uint8_t g_pos_creep_enable;      /* 1 = 开启爬行下限 */
extern volatile uint8_t g_pos_brake_enable;      /* 1 = 到位后短路制动 */
extern volatile float   g_pos_brake_speed_rpm;   /* 进入制动的速度门限 rpm */
extern volatile float   g_pos_brake_hyst_rev;    /* 退出制动的误差滞环 圈 */
extern volatile float   g_pos_limit_rev;         /* 软行程限位 圈（±） */
extern volatile float   g_pos_back_gain;         /* 抗饱和回退系数 */
extern volatile uint8_t g_pos_zero_req;          /* 写 1 把当前位置设为 0 并保持 */

/*=============================================================================
 * 三、观测量
 *=============================================================================*/
extern volatile uint8_t g_pos_active;      /* 1 = 电流环已 active，位置环闭环中 */
extern volatile uint8_t g_pos_brake;       /* 1 = 当前处于到位刹车 */
extern volatile float   g_pos_fb_rev;      /* 当前位置，圈 */
extern volatile float   g_pos_err_rev;     /* 限幅/死区后的位置误差，圈 */
extern volatile float   g_pos_cmd_rpm;     /* 位置 PI 输出速度指令，rpm */
extern volatile float   g_pos_kp_used;     /* 实际生效的位置增益 rpm/rev */

/*=============================================================================
 * 四、API
 *=============================================================================*/
void Foc_Position_Init(void);

/* 进入位置模式：以当前位置为目标（保持不动），清位置 PID */
void Foc_Position_Reset(void);

/* 离开位置模式：清位置 PID 与输出观测量 */
void Foc_Position_Stop(void);

/* 每拍调用（20kHz ISR，内部 10ms 抽稀） */
void Foc_Position_Task20k(void);

/* 是否处于到位刹车：true 时模式层输出零矢量，不跑速度/电流环 */
bool Foc_Position_IsBrake(void);

#endif /* FOC_POSITION_LOOP_H */
