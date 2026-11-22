/**
 * @file    bsp_pwm.h
 * @brief   三相 PWM BSP 层接口（TIM4_3）
 * @note    硬件配置：HC32F452 TIM4_3
 *          HU-PB9, LU-PB8, HV-PB7, LV-PB6, HW-PB5, LW-PB4
 */

#ifndef __BSP_PWM_H
#define __BSP_PWM_H

#include <stdint.h>
#include <rtthread.h>
#include "../hw/hw_pwm.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 私有数据类型
 * ============================================================ */
typedef struct {
    uint32_t period_counts;
    uint32_t max_duty;
    uint8_t  initialized;
    uint8_t  braked;
} bsp_pwm_priv_t;

/* ============================================================
 * 供 bsp_pwm_config.c / 应用层引用
 * ============================================================ */
extern const hw_pwm_ops_t g_bsp_pwm_ops;
extern bsp_pwm_priv_t     g_bsp_pwm_priv;

/* 应用层持有的 PWM 对象（定义在 app_hw.c） */
extern hw_pwm_t           g_swing_pwm;

/* ============================================================
 * 紧急刹车接口（ISR 安全）
 * ============================================================ */

/**
 * @brief 紧急刹车（直接操作寄存器，ISR 安全：直接写 EMB_SOE3 寄存器）
 * @note  用于三级过流保护，旁路所有软件逻辑
 */
void bsp_pwm_emergency_brake(void);

/**
 * @brief 解除紧急刹车（直接操作寄存器，ISR 安全：直接写 EMB_SOE3 寄存器）
 * @note  用于三级过流保护，旁路所有软件逻辑
 */
void bsp_pwm_emergency_release(void);

/* ============================================================
 * 【2026-11-02 新增】FOC 微小时基（TMR4_3 周期中断）
 * ------------------------------------------------------------
 * 通过三角波谷值中断产生 20 kHz 周期计数，与 PWM 严格同相。
 * 分辨率 50 μs —— 这是 FOC 控制周期的天然粒度。
 *
 * 【与其他时基的分工】
 *   bsp_pwm_get_cycle_cnt()  50μs，与 PWM 同相，供 FOC 外推/控制周期
 *   rt_tick_get()            1ms，供 RTOS 超时/延时
 *   两者不可互相替代：前者精度够但无日历语义，后者反之。
 *
 * 【时间差用法】
 *   uint32_t dcyc = bsp_pwm_get_cycle_cnt() - prev;   // 无符号减法自动处理回绕
 *   uint32_t dus  = bsp_pwm_cycle_to_us(dcyc);
 * ============================================================ */

/**
 * @brief 取 PWM 周期计数
 * @retval 自 PWM 启动以来经过的周期数，每 50 μs 加 1
 * @note   32bit 回绕周期约 59.6 小时（2^32 / 20kHz）
 */
uint32_t bsp_pwm_get_cycle_cnt(void);

/**
 * @brief 周期数换算为微秒
 */
uint32_t bsp_pwm_cycle_to_us(uint32_t cycles);

/**
 * @brief 微时基是否已使能（诊断用）
 * @retval 1 已使能；0 未使能（此时 cycle_cnt 恒为 0）
 */
uint8_t bsp_pwm_tb_is_enabled(void);

/**
 * @brief 时基寄存器直读快照（诊断用，2026-11-02 新增）
 *
 * 单次 t_pwmtime 只给 cycle_cnt 无法区分三种故障：
 *   a) 计数器没跑（STOP 位仍置位）→ CNT 不变化
 *   b) 计数器在跑但中断没来（标志不置位/中断源选错）→ CNT 变、cycle_cnt 不变
 *   c) 中断来了但被更高优先级长期压制 → 两者都变但抖动大
 * 本接口把判定所需的原始寄存器一次交出来，避免反复往返。
 *
 * @param out 长度 >= 8 的数组，含义见实现注释
 */
void bsp_pwm_tb_snapshot(uint32_t out[8]);

/**
 * @brief 读 TMR4 硬件计数器当前值（三角波 0..period）
 *
 * 与 bsp_pwm_get_cycle_cnt() 的区别：
 *   get_cycle_cnt  软件维护，50 μs 粒度，靠中断累加
 *   get_cntr       硬件真值，10 ns 粒度，不依赖中断
 *
 * 诊断时连续读两次本函数：
 *   - 值在变 → 计数器确实在跑，问题在中断路径
 *   - 值不变 → 计数器停着，问题在启动/时钟
 * @retval 0..period（period = 2500 @20kHz）
 */
uint16_t bsp_pwm_tb_get_cntr(void);

/**
 * @brief 中断链路完整快照（诊断用，2026-11-03 新增）
 * @param out 长度 >= 8，含义见实现注释
 */
void bsp_pwm_tb_irq_snapshot(uint32_t out[8]);

/**
 * @brief 事件计数快照（诊断用）
 * @param out [0] 谷值计数（主时基 20kHz） [1] 峰值计数 [2] 无误标志中断数
 */
void bsp_pwm_tb_event_cnt(uint32_t out[3]);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_PWM_H */
