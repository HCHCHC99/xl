/**
 * @file    bsp_i2c_soft.c
 * @brief   软件模拟 I2C 实现（开漏 GPIO 位操作，约 100kHz）
 */

#include "bsp_i2c_soft.h"
#include "framework/state_engine/sm_assert.h"

#include <rtthread.h>
#include "hc32_ll.h"
#include "hc32_ll_gpio.h"

/* board.c 中实现 */
extern void rt_hw_us_delay(rt_uint32_t us);

/* ============================================================
 * 底层引脚操作
 * ============================================================ */
static inline void scl_high(void) { GPIO_SetPins(BSP_I2C_SCL_PORT, BSP_I2C_SCL_PIN); }
static inline void scl_low(void)  { GPIO_ResetPins(BSP_I2C_SCL_PORT, BSP_I2C_SCL_PIN); }

static inline void sda_high(void) { GPIO_SetPins(BSP_I2C_SDA_PORT, BSP_I2C_SDA_PIN); }
static inline void sda_low(void)  { GPIO_ResetPins(BSP_I2C_SDA_PORT, BSP_I2C_SDA_PIN); }

static inline uint8_t sda_read(void)
{
    return (GPIO_ReadInputPins(BSP_I2C_SDA_PORT, BSP_I2C_SDA_PIN) != PIN_RESET) ? 1U : 0U;
}

static inline void i2c_delay(void) { rt_hw_us_delay(BSP_I2C_DELAY_US); }

/* CS（I2C_SCL_WR）：低电平有效，传输期间使能 */
static inline void cs_enable(void)
{
#if (BSP_I2C_CS_ACTIVE_LEVEL != 0)
    GPIO_SetPins(BSP_I2C_CS_PORT, BSP_I2C_CS_PIN);
#else
    GPIO_ResetPins(BSP_I2C_CS_PORT, BSP_I2C_CS_PIN);
#endif
}

static inline void cs_disable(void)
{
#if (BSP_I2C_CS_ACTIVE_LEVEL != 0)
    GPIO_ResetPins(BSP_I2C_CS_PORT, BSP_I2C_CS_PIN);
#else
    GPIO_SetPins(BSP_I2C_CS_PORT, BSP_I2C_CS_PIN);
#endif
}

/* ============================================================
 * 时序
 * ============================================================ */
static void i2c_start(void)
{
    sda_high(); i2c_delay();
    scl_high(); i2c_delay();
    sda_low();  i2c_delay();     /* SCL 高时 SDA 下降 = 起始 */
    scl_low();  i2c_delay();
}

static void i2c_stop(void)
{
    sda_low();  i2c_delay();
    scl_high(); i2c_delay();
    sda_high(); i2c_delay();     /* SCL 高时 SDA 上升 = 停止 */
}

/**
 * @brief 写一个字节
 * @return 1 = 收到 ACK；0 = NACK
 */
static uint8_t i2c_write_byte(uint8_t b)
{
    uint8_t i;
    uint8_t ack;

    for (i = 0U; i < 8U; i++) {
        if ((b & 0x80U) != 0U) {
            sda_high();
        } else {
            sda_low();
        }
        b = (uint8_t)(b << 1);
        i2c_delay();

        scl_high(); i2c_delay();
        scl_low();  i2c_delay();
    }

    /* 第 9 拍：释放 SDA，采样 ACK */
    sda_high(); i2c_delay();
    scl_high(); i2c_delay();
    ack = (sda_read() == 0U) ? 1U : 0U;
    scl_low();  i2c_delay();

    return ack;
}

/**
 * @brief 读一个字节
 * @param ack 1 = 读完发 ACK（还有后续字节）；0 = 发 NACK（最后一字节）
 */
static uint8_t i2c_read_byte(uint8_t ack)
{
    uint8_t i;
    uint8_t b = 0U;

    sda_high();                  /* 释放 SDA，准备接收 */

    for (i = 0U; i < 8U; i++) {
        b = (uint8_t)(b << 1);
        scl_high(); i2c_delay();
        if (sda_read() != 0U) {
            b |= 1U;
        }
        scl_low();  i2c_delay();
    }

    /* 第 9 拍：主机回 ACK/NACK */
    if (ack != 0U) {
        sda_low();
    } else {
        sda_high();
    }
    i2c_delay();
    scl_high(); i2c_delay();
    scl_low();  i2c_delay();
    sda_high();

    return b;
}

/* ============================================================
 * 初始化
 * ============================================================ */
int bsp_i2c_soft_init(void)
{
    stc_gpio_init_t g;

    LL_PERIPH_WE(LL_PERIPH_GPIO);

    /* SCL：开漏输出，初始高（释放总线）
     * 【Q34 2026-11-12】硬件已在 SCL/SDA 外部 10k 上拉到 3.3V，
     *   内部上拉必须 OFF（u16PullUp = PIN_PU_OFF），否则并联拉低等效阻值、
     *   增加漏电。GPIO_StructInit 默认即 OFF，此处显式写死以防后人误改。 */
    (void)GPIO_StructInit(&g);
    g.u16PinDir        = PIN_DIR_OUT;
    g.u16PinOutputType = PIN_OUT_TYPE_NMOS;
    g.u16PinState      = PIN_STAT_SET;
    g.u16PullUp        = PIN_PU_OFF;
    (void)GPIO_Init(BSP_I2C_SCL_PORT, BSP_I2C_SCL_PIN, &g);

    /* SDA：开漏双向（写 1 释放后可读外部电平）；同样禁用内部上拉 */
    g.u16PullUp        = PIN_PU_OFF;
    (void)GPIO_Init(BSP_I2C_SDA_PORT, BSP_I2C_SDA_PIN, &g);

    /* CS：推挽输出，初始不使能 */
    (void)GPIO_StructInit(&g);
    g.u16PinDir        = PIN_DIR_OUT;
    g.u16PinOutputType = PIN_OUT_TYPE_CMOS;
#if (BSP_I2C_CS_ACTIVE_LEVEL != 0)
    g.u16PinState      = PIN_STAT_RST;
#else
    g.u16PinState      = PIN_STAT_SET;
#endif
    (void)GPIO_Init(BSP_I2C_CS_PORT, BSP_I2C_CS_PIN, &g);

    LL_PERIPH_WP(LL_PERIPH_GPIO);

    scl_high();
    sda_high();

    rt_kprintf("[BSP-I2C] soft i2c on PB2(SCL)/PB1(SDA), CS=PB10 active-low, ~100kHz\n");
    rt_kprintf("[BSP-I2C] external 10k pull-up on SDA/SCL, internal pull-up OFF\n");
    return 0;
}

/* ============================================================
 * 对外接口
 * ============================================================ */
int bsp_i2c_soft_write(uint8_t dev7, const uint8_t *buf, uint16_t len)
{
    uint16_t i;

    SM_ASSERT_PTR(buf);
    if (len == 0U) {
        return -1;
    }

    cs_enable();
    i2c_start();

    if (i2c_write_byte((uint8_t)(dev7 << 1)) == 0U) {
        i2c_stop();
        cs_disable();
        return -2;                       /* 地址 NACK */
    }

    for (i = 0U; i < len; i++) {
        if (i2c_write_byte(buf[i]) == 0U) {
            i2c_stop();
            cs_disable();
            return -3;                   /* 数据 NACK */
        }
    }

    i2c_stop();
    cs_disable();
    return 0;
}

int bsp_i2c_soft_read(uint8_t dev7, uint8_t *buf, uint16_t len)
{
    uint16_t i;

    SM_ASSERT_PTR(buf);
    if (len == 0U) {
        return -1;
    }

    cs_enable();
    i2c_start();

    if (i2c_write_byte((uint8_t)((dev7 << 1) | 0x01U)) == 0U) {
        i2c_stop();
        cs_disable();
        return -2;
    }

    for (i = 0U; i < len; i++) {
        buf[i] = i2c_read_byte((i < (uint16_t)(len - 1U)) ? 1U : 0U);
    }

    i2c_stop();
    cs_disable();
    return 0;
}

uint8_t bsp_i2c_soft_probe(uint8_t dev7)
{
    uint8_t ack;

    cs_enable();
    i2c_start();
    ack = i2c_write_byte((uint8_t)(dev7 << 1));
    i2c_stop();
    cs_disable();

    return ack;
}
