/**
 * @file    bsp_i2c_soft.h
 * @brief   软件模拟 I2C（GPIO 位操作）
 *
 * 【为什么用软件 I2C 而不是硬件 I2C】
 *   板级引脚：SCL = PB2，SDA = PB10
 *   数据手册功能表：
 *     PB2  / Func_Grp1 / Func49 = I2C1_SCL
 *     PB10 / Func_Grp2 / Func48 = I2C3_SDA
 *   两者分属 I2C1 与 I2C3 两个不同实例，物理上无法组成一对 SCL/SDA，
 *   因此硬件 I2C 不可用。EEPROM 为低频访问（页写 5ms 级），
 *   软件模拟 100kHz 完全够用。
 *
 * 【速率】约 100 kHz（半周期 5us），由 rt_hw_us_delay 控制
 * 【引脚】
 *   SCL : PB2   （开漏输出）
 *   SDA : PB10  （开漏双向）
 *   CS  : PB1   （I2C_SCL_WR，低电平有效，传输期间使能）
 */

#ifndef __BSP_I2C_SOFT_H
#define __BSP_I2C_SOFT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 引脚配置
 * ------------------------------------------------------------
 * 【2026-11-12 硬件变更 · Q32 用户裁定】
 *   原方案：IO 模拟，SDA=PB10 / CS=PB1
 *   新硬件：已改为
 *       I2C_SDA_24M02 = PB1   （数据）
 *       I2C_SCL_24M02 = PB2   （时钟）
 *       I2C_SCL_WR    = PB10  （写保护/片选，低有效）
 *
 * ⚠️ 注意 PB1 与 PB10 的用途相对旧版本【对调了】。
 *    改错会导致 EEPROM 完全读不到 ⇒ sys_param / sys_config 参数存不住
 *    （表现为机型设置丢失、零点标定丢失）。
 *
 * ⚠️ 硬件已在 SDA(=PB1) 与 SCL(=PB2) 外部各加 10k 上拉到 3.3V
 *    （Q34 用户确认）⇒ MCU 内部上拉【必须保持关闭】，避免并联降低阻值。
 * ============================================================ */
#define BSP_I2C_SCL_PORT        (GPIO_PORT_B)
#define BSP_I2C_SCL_PIN         (GPIO_PIN_02)       /* PB2  I2C_SCL_24M02 */

#define BSP_I2C_SDA_PORT        (GPIO_PORT_B)
#define BSP_I2C_SDA_PIN         (GPIO_PIN_01)       /* PB1  I2C_SDA_24M02 */

#define BSP_I2C_CS_PORT         (GPIO_PORT_B)
#define BSP_I2C_CS_PIN          (GPIO_PIN_10)       /* PB10 I2C_SCL_WR   */
#define BSP_I2C_CS_ACTIVE_LEVEL (0)     /* 低有效 */

/* 半周期延时（us）：5us → 约 100kHz */
#define BSP_I2C_DELAY_US        (5U)

/* ============================================================
 * API
 * ============================================================ */

/**
 * @brief 初始化 GPIO [单线程]
 * @return 0 成功
 */
int bsp_i2c_soft_init(void);

/**
 * @brief 向从机写数据 [单线程]
 * @param dev7 7 位从机地址（如 0x50）
 * @param buf  数据
 * @param len  长度
 * @return 0 成功；<0 失败（无应答/参数错误）
 * @note  内部自动处理 CS 使能与释放
 */
int bsp_i2c_soft_write(uint8_t dev7, const uint8_t *buf, uint16_t len);

/**
 * @brief 从从机读数据 [单线程]
 * @param dev7 7 位从机地址
 * @param buf  接收缓冲
 * @param len  长度
 * @return 0 成功；<0 失败
 */
int bsp_i2c_soft_read(uint8_t dev7, uint8_t *buf, uint16_t len);

/**
 * @brief 探测从机是否应答（用于写周期完成检测） [单线程]
 * @param dev7 7 位从机地址
 * @return 1 有应答；0 无应答
 */
uint8_t bsp_i2c_soft_probe(uint8_t dev7);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_I2C_SOFT_H */
