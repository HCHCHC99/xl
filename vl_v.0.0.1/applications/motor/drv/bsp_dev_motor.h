/**
 * @file    bsp_dev_motor.h
 * @brief   电机 BSP 层接口
 * @note    本文件声明电机驱动所需的底层硬件操作接口
 *          具体实现在 bsp_dev_motor.c 中
 */

#ifndef __BSP_DEV_MOTOR_H
#define __BSP_DEV_MOTOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 三相 PWM 接口
 * ============================================================ */

/**
 * @brief 初始化三相 PWM
 * @param pwm_dev     PWM 设备句柄（如定时器句柄）
 * @param freq_hz     PWM 频率（Hz）
 * @param deadtime_ns 死区时间（ns）
 */
void bsp_pwm_init_3phase(void *pwm_dev, uint32_t freq_hz, uint32_t deadtime_ns);

/**
 * @brief 设置某相 PWM 占空比
 * @param pwm_dev PWM 设备句柄
 * @param phase   相号：1=A相, 2=B相, 3=C相
 * @param duty    占空比（0 ~ max_duty）
 */
void bsp_pwm_set_3phase(void *pwm_dev, uint8_t phase, uint32_t duty);

/**
 * @brief 关闭三相 PWM 输出
 */
void bsp_pwm_disable_3phase(void *pwm_dev);

/* ============================================================
 * GPIO 接口
 * ============================================================ */

void    bsp_gpio_init_input(void *port, uint16_t pin);
void    bsp_gpio_init_output(void *port, uint16_t pin);
uint8_t bsp_gpio_read(void *port, uint16_t pin);
void    bsp_gpio_write(void *port, uint16_t pin, uint8_t level);

/* ============================================================
 * 霍尔芯片读取（3路融合编码器）
 * ============================================================ */

int32_t bsp_hall_chip_read(void *dev, uint8_t channel);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_DEV_MOTOR_H */
