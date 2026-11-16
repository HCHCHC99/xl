/**
 * @file    motor_speed.h
 * @brief   转速 / 角速度计算（基于 Z0 角度差分 + 20kHz 硬件时基）
 *
 * ============================================================
 * 【为什么用 Z0 而不是霍尔】
 * ------------------------------------------------------------
 *   霍尔：6 状态/电周期，1 对极时 6 计数/转
 *         → 5ms 窗口下 1 LSB = 2000 RPM，完全不可用
 *   Z0  ：4096 计数/转
 *         → 5ms 窗口下 1 LSB = 2.93 RPM，可用
 *   精度提升 683 倍。这是采用编码器测速的根本原因。
 *
 * ============================================================
 * 【2026-11-03 重大修正：时间源改用 20kHz 硬件时基】
 * ------------------------------------------------------------
 * 原实现用 rt_tick_get_millisecond()，分辨率只有 1ms。
 * 5ms 窗口下 dt 只能是整数 5 或 6，量化误差：
 *     实际 5.3ms 读到 5ms  → 误差 -5.7%
 *     实际 5.5ms 读到 6ms  → 误差 +9.1%
 *     最坏情况            → ±20%
 * 这个误差比角度量化（±0.08%）大两个数量级，是原实现最大的误差源。
 *
 * 改用 bsp_pwm_get_cycle_cnt()（50μs 分辨率，与 PWM 同相）后：
 *     5ms 窗口 = 100 个 tick，dt 量化误差 ±0.5%
 *     改善 20 倍。
 *
 * 【为什么这个改善是必需的，不是锦上添花】
 *   实测曾看到 delta 稳定在 -798~-805、dt 恒为 5，看起来很好。
 *   但那是"软定时器碰巧准"的运气。一旦任务被阻塞（打印、SPI 抢占、
 *   将来加 FOC 计算），dt 跳到 6ms，同一转速立刻显示：
 *     dt=5ms delta=-800 → -2343.8 RPM
 *     dt=6ms delta=-800 → -1953.1 RPM   ← 差 16.7%
 *   系统负载一变就暴露，属于潜伏缺陷。
 *
 * ============================================================
 * 【闭环参考侧：输出轴（卸粮筒），不是 Z0】
 * ------------------------------------------------------------
 * 控制目标是卸粮筒转角/转速，所以速度环必须在输出轴侧闭环：
 *
 *   路 A（Z0 侧）：设定值 = 目标转速 × 1080
 *     ✗ 设定值不直观
 *     ✗ 1080 倍放大：输出轴 ±0.1RPM 的指令 = Z0 侧 ±108 RPM，PID 极敏感
 *     ✗ 单位不统一（限位用度、速度用 Z0 RPM）
 *
 *   路 B（输出轴侧，本实现）：
 *     ✓ 设定值就是用户要的物理量
 *     ✓ 与 out_x10 行程限位单位统一
 *
 * 【关键：换算只做一次】
 *   不要"先算 rpm_z0 再除 1080"——那会二次整数截断。
 *   直接从 Z0 计数差一步算到输出轴转速，单次除法：
 *     rpm_out_x100 = dcnt * 600000 * 100 / (4096 * i_total_x100 * dt_ticks)
 *   减速比从 mech_cfg 的 i_total_x100 取，不硬编码。
 *
 * ============================================================
 * 【单位选择：输出轴用 x100，不用 x10】
 * ------------------------------------------------------------
 * 实测输出轴转速仅 2.1 RPM 量级：
 *     x10 单位：1 LSB = 0.1 RPM → 占 2.1 RPM 的 4.8%，速度环会有量化死区
 *     x100 单位：1 LSB = 0.01 RPM → 占 0.48%，够用
 * 位宽无虞：输出轴最高 3.24 RPM = 324(x100)，±10RPM 也只有 ±1000。
 *
 * ============================================================
 * 【精度与滞后核算】
 * ------------------------------------------------------------
 *   5ms 窗口、3500 RPM（Z0 侧）：
 *     角度量化 ±1 LSB  → ±0.08%
 *     时间量化 ±0.5tick → ±0.50%
 *     合计(RSS)        → ±0.51%
 *
 *   低速端（角度量化主导，5ms 窗口）：
 *     3500 RPM → ±0.08%      1000 RPM → ±0.29%
 *      300 RPM → ±0.98%       100 RPM → ±2.93%
 *       30 RPM → ±9.77%
 *   100 RPM 以下若需高精度，须加长窗口或改用 M/T 法（本版未做）。
 *
 *   加速滞后（0→3500RPM/200ms，即 17500 RPM/s）：
 *     5ms 窗口 → 87.5 RPM（相对 3500 为 2.5%），速度环可接受。
 * ============================================================
 */

#ifndef __MOTOR_SPEED_H
#define __MOTOR_SPEED_H

#include <rtthread.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 速度计算窗口：速度环周期的 tick 数（5ms @ 20kHz = 100 tick） */
#define MOTOR_SPEED_WINDOW_TICKS    (100U)

/** 硬件时基频率（Hz），与 bsp_pwm 的 PWM 频率一致 */
#define MOTOR_SPEED_TICK_HZ         (20000U)

/** 每秒 tick 数（用于换算，避免魔法数字） */
#define MOTOR_SPEED_TICKS_PER_SEC   (MOTOR_SPEED_TICK_HZ)

/** 窗口过长判定：超过此值认为中断/任务被暂停，只重建基线不计算 */
#define MOTOR_SPEED_WINDOW_MAX_TICKS (10000U)   /* 500ms */

/* ============================================================
 * 状态
 * ============================================================ */
typedef struct {
    /* ---- 主输出（速度环用这个）---- */
    int32_t  rpm_out_x100;      /* 输出轴转速，0.01 RPM，带符号 ★ */
    int32_t  omega_out_mdeg_s;  /* 输出轴角速度，0.001 deg/s，带符号 */

    /* ---- 诊断输出 ---- */
    int32_t  rpm_z0_x10;        /* Z0（电机轴）转速，0.1 RPM */
    int32_t  delta_cnt;         /* 本窗口 Z0 计数增量 */
    uint32_t dt_ticks;          /* 本窗口实际时长（tick，50us/个）*/
    uint32_t dt_ms;             /* 同上换算成 ms，仅供人读 */

    uint32_t update_cnt;        /* 成功计算出转速的次数 */
    uint32_t reject_cnt;        /* dt 异常导致的丢弃（真故障） */
    uint32_t rebase_cnt;        /* 因角度不可用而重设基线的次数（非 dt 问题） */
    uint32_t dt_max_ticks;      /* 观察到的最大 dt（诊断抖动用） */
    uint8_t  valid;             /* 1 = 至少算出过一次有效值 */
} motor_speed_state_t;

/* ============================================================
 * API
 * ============================================================ */

void motor_speed_init(void);

/**
 * @brief 周期调用（与 motor_angle_update 同频，1ms，由软定时器驱动）
 *
 * @param cnt_now   当前 Z0 累计计数（来自 motor_angle_get_cnt）
 * @param now_tick  当前 20kHz 硬件 tick（来自 bsp_pwm_get_cycle_cnt）
 *
 * @note 【为什么传 tick 而不是 ms】
 *       调用方原先传 rt_tick_get_millisecond()，1ms 分辨率不够。
 *       改传硬件 tick 后，dt 量化从 ±10% 降到 ±0.5%。
 *       若某项测试不需要硬件时基，传 now_ms * (TICK_HZ/1000) 亦可，
 *       但会退回 ±10% 精度，生产路径必须用硬件 tick。
 *
 * @note 内部按 MOTOR_SPEED_WINDOW_TICKS 分频，调用方保持 1ms 节拍即可。
 */
void motor_speed_update(int32_t cnt_now, uint32_t now_tick);

/**
 * @brief 重设窗口基线（不计算速度）
 *
 * @param cnt_now   当前 Z0 计数
 * @param now_tick  当前硬件 tick
 *
 * @note 【用途】角度更新失败时调用。若不重设，tick 基线会停在很久以前，
 *       恢复后第一帧 dt 巨大（>500ms）被丢弃，再下一帧才能出数；
 *       期间若恰好在 500ms 内，还会用跨越大段时间的 delta 算出错误转速。
 *       每次失败都重设，保证恢复时窗口是干净的 5ms。
 */
void motor_speed_rebase(int32_t cnt_now, uint32_t now_tick);

/**
 * @brief 清空全部状态（含 valid），用于重新解算后重置
 */
void motor_speed_reset(void);

/**
 * @brief 取输出轴转速（速度环反馈）
 * @param[out] rpm_out_x100  0.01 RPM（可 NULL）
 * @retval 1 有效；0 尚未算出
 */
uint8_t motor_speed_get(int32_t *rpm_out_x100);

/**
 * @brief 取输出轴角速度
 * @param[out] omega_mdeg_s 0.001 deg/s（可 NULL）
 * @retval 1 有效；0 尚未算出
 */
uint8_t motor_speed_get_omega(int32_t *omega_mdeg_s);

/**
 * @brief 取 Z0 侧转速（诊断用）
 */
uint8_t motor_speed_get_z0(int32_t *rpm_z0_x10);

void motor_speed_get_state(motor_speed_state_t *out);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_SPEED_H */