/**
 *******************************************************************************
 * @file  foc_position_loop.h
 * @brief L8 位置外环（减速包络，2ms，输出速度指令）
 *
 *        控制律：
 *          err  = clamp(target, ±limit) - pos            [rev]
 *          vcmd = sign(err) × min(sqrt(120·dec·|err|), v_max) - kd × 实测转速
 *          |err| <= deadband 时 vcmd = 0（交给速度环闭环保持 0 速）
 *
 *        位置反馈：g_enc_count × g_enc_elec_dir / CFG_ENCODER_CPR - 零点
 *        参数初值在下方的 *_INIT 宏；运行时可改 g_pos_* 变量
 *******************************************************************************
 */

#ifndef FOC_POSITION_LOOP_H
#define FOC_POSITION_LOOP_H

#include <stdbool.h>
#include <stdint.h>

/* 位置环执行周期：40 拍 = 2ms（20kHz） */
#define POS_CTRL_DIV                (40u)

/* 规划减速度，rpm/s（代码会自动取 min(g_spd_accel_rpm_s)） */
#define POS_DEC_RPM_S_INIT          (5000.0f)

/* 速度阻尼增益，rpm/rpm；0 = 关闭 */
#define POS_KD_RPM_PER_RPM_INIT     (0.0f)

/* 位置环输出（速度指令）限幅，rpm */
#define POS_V_MAX_RPM_INIT          (2500.0f)

/* 位置模式速度环 iq 限幅，mA */
#define POS_IQ_LIMIT_MA_INIT        (3400.0f)

/* 到位窗口，圈 */
#define POS_DEADBAND_REV_INIT       (0.03f)

/* 到位判据速度门限，rpm */
#define POS_V_INPOS_RPM_INIT        (30.0f)

/* 位置误差观测限幅，圈 */
#define POS_ERR_MAX_REV_INIT        (3.0f)

/* 软行程限位，圈（对称 ±） */
#define POS_LIMIT_REV_INIT          (1000000.0f)

/*=============================================================================
 * 可调参数变量（Watch 可改）
 *=============================================================================*/
extern volatile float   g_pos_target_rev;
extern volatile uint8_t g_pos_zero_req;

extern volatile float   g_pos_dec_rpm_s;
extern volatile float   g_pos_kd_rpm_per_rpm;
extern volatile float   g_pos_v_max_rpm;
extern volatile float   g_pos_iq_limit_ma;
extern volatile float   g_pos_deadband_rev;
extern volatile float   g_pos_v_inpos_rpm;
extern volatile float   g_pos_err_max_rev;
extern volatile float   g_pos_limit_rev;

/*=============================================================================
 * 观测量
 *=============================================================================*/
extern volatile uint8_t g_pos_active;
extern volatile uint8_t g_pos_in_pos;
extern volatile float   g_pos_fb_rev;
extern volatile float   g_pos_err_rev;
extern volatile float   g_pos_cmd_rpm;

/*=============================================================================
 * API
 *=============================================================================*/
void Foc_Position_Init(void);

/* 进入位置模式：以当前位置为目标（保持不动），清位置环状态 */
void Foc_Position_Reset(void);

/* 离开位置模式：清位置环输出与观测量 */
void Foc_Position_Stop(void);

/* 每拍调用（20kHz ISR，内部 2ms 抽稀） */
void Foc_Position_Task20k(void);

#endif /* FOC_POSITION_LOOP_H */
