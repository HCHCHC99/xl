/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-01-28     gylocal       the first version
 */


//#include <drv_common.h>
#include "hc32_ll_gpio.h"
#include "board.h"

#include "bsp_drv_spi.h"
#include "drv_gpio.h"

#include "drv_spi.h"


#include "bsp_drv_spi.h"

#include "app.h"

#define KTH7812_SPI_DEVICE_NAME_41   "spi10"
#define KTH7812_SPI_DEVICE_NAME_42   "spi11"
#define KTH7812_SPI_DEVICE_NAME_43   "spi12"

#define ZD25Q_SPI_DEVICE_NAME     "spi30"



struct rt_spi_message *res;
//struct rt_spi_configuration cfg;

typedef struct{
    const char *spi_bus_name;
    const char *spi_device_name;


}DEV_SPI_CONFIG_DEF;


static rt_err_t bsp_dev_spi_kth7812_init(void)
{

#define USER_HC32_LL_SPI_DEV

#define BSP_SPI_UNIT                    CM_SPI1
#define BSP_SPI_PERIPH_CLK              FCG1_PERIPH_SPI1

#include "hc32_ll_gpio.h"
#include "board_config.h"

/* 【2026-10-29 D-59】CS 引脚实测纠正，必须与 bsp_dev_kth7812.c 的
 * KTH78_GetCS() 和 kth7812_config_data[] 三处保持一致：
 *     CS41 = PB15   （齿数 41）
 *     CS42 = PA8    （齿数 42）
 *     CS43 = PA9    （齿数 43）
 * 原值 PA9/PA8/PA15 与实际接线差两根，导致：
 *   - PA15 无 CS 走线 ⇒ 该"设备"永久读 0xFFFF
 *   - PB15 从未被驱动 ⇒ 齿数 41 那颗从未被选中 */
#define BSP_SPI_CS_PORT_41     (GPIO_PORT_B)
#define BSP_SPI_CS_PIN_41      (GPIO_PIN_15)

#define BSP_SPI_CS_PORT_42     (GPIO_PORT_A)
#define BSP_SPI_CS_PIN_42      (GPIO_PIN_08)

#define BSP_SPI_CS_PORT_43     (GPIO_PORT_A)
#define BSP_SPI_CS_PIN_43      (GPIO_PIN_09)

    stc_gpio_init_t stcGpioInit;
    stc_spi_init_t stcSpiInit;
    stc_spi_delay_t stcSpiDelayCfg;

    /* ============================================================
     * 【D-57，2026-10-29】必须先解锁 GPIO，否则下面所有配置静默失效
     * ------------------------------------------------------------
     * 事故现象：发送 t_enc_all 时示波器观察 SCK / MOSI / MISO 全部
     * 恒为高电平、无任何翻转。三颗编码器全寄存器读出 0x0000。
     *
     * 根因链：
     *   1) bsp_hw_init.c:86-90 步骤顺序为
     *        brake_pin → pwm_pin → tim4 → hall_exti → adc_unit → spi_enc
     *   2) bsp/bsp_hall.c:204 在初始化结尾执行
     *        LL_PERIPH_WP(... | LL_PERIPH_GPIO | ...)
     *      内部调 GPIO_REG_Lock()，锁住 PSPCR/PCCR/PINAER/PCRxy/PFSRxy
     *   3) 本函数此前**从未解锁**，直接调 GPIO_Init / GPIO_SetFunc，
     *      而 GPIO_SetFunc 的实现是直接写 PFSRx（hc32_ll_gpio.c:383），
     *      写受保护寄存器被硬件丢弃 → 引脚从未切到 SPI 复用功能
     *   4) DDL_ASSERT(IS_GPIO_UNLOCK()) 在发布编译下被编成 ((void)0U)
     *      （hc32_ll_utility.h:95），连断言都不报，故障完全静默
     *
     * 为什么 CLK 静态看起来"正常"：Mode 3（CPOL=1）下 SCK 空闲本就
     * 是高电平。真正的判据是**发送时是否翻转** —— 16 bit @ 1.5625MHz
     * 应有 16 个 640ns 周期，共 10.24us 的方波。
     * ============================================================ */
    LL_PERIPH_WE(LL_PERIPH_GPIO);

    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinDir = PIN_DIR_OUT;
    stcGpioInit.u16PinDrv = PIN_HIGH_DRV;

    (void)GPIO_Init(SPI_SCK_PORT, SPI_SCK_PIN, &stcGpioInit);
    (void)GPIO_Init(SPI_MOSI_PORT, SPI_MOSI_PIN, &stcGpioInit);

    /* 【D-58，2026-10-29】MISO 此前从未调用 GPIO_Init，只做了 SetFunc。
     * 后果：PCR 的 DDIS（数字输入使能）位保持复位值，若复位为"模拟"
     * 则数字输入通路断开，MISO 读回恒 0 或悬空电平。
     * 这里显式配成：输入方向 + 数字属性（DDIS=0）+ 上拉。
     * 上拉的理由：编码器断线时能读到确定的 0xFFFF，而不是随机电平 ——
     * 便于把"没接上"和"数据错误"区分开。 */
    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinDir   = PIN_DIR_IN;
    stcGpioInit.u16PinAttr  = PIN_ATTR_DIGITAL;   /* DDIS=0，开数字输入 */
    stcGpioInit.u16PullUp   = PIN_PU_ON;
    (void)GPIO_Init(SPI_MISO_PORT, SPI_MISO_PIN, &stcGpioInit);

    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinDir = PIN_DIR_OUT;
    stcGpioInit.u16PinState = PIN_STAT_SET;
    (void)GPIO_Init(BSP_SPI_CS_PORT_41,  BSP_SPI_CS_PIN_41,  &stcGpioInit);
    (void)GPIO_Init(BSP_SPI_CS_PORT_42,  BSP_SPI_CS_PIN_42,  &stcGpioInit);
    (void)GPIO_Init(BSP_SPI_CS_PORT_43,  BSP_SPI_CS_PIN_43,  &stcGpioInit);

    GPIO_SetFunc(SPI_SCK_PORT, SPI_SCK_PIN, SPI_SCK_FUNC);
    GPIO_SetFunc(SPI_MOSI_PORT, SPI_MOSI_PIN, SPI_MOSI_FUNC);
    GPIO_SetFunc(SPI_MISO_PORT, SPI_MISO_PIN, SPI_MISO_FUNC);

    /* 引脚配置完成，恢复寄存器保护（与 bsp_hall.c 的做法保持一致） */
    LL_PERIPH_WP(LL_PERIPH_GPIO);

    /* Clear initialize structure */
    (void)SPI_StructInit(&stcSpiInit);
    (void)SPI_DelayStructInit(&stcSpiDelayCfg);

    /* Configure peripheral clock */
    FCG_Fcg1PeriphClockCmd(BSP_SPI_PERIPH_CLK, ENABLE);

    /* SPI De-initialize */
    (void)SPI_DeInit(BSP_SPI_UNIT);
    /* Configuration SPI structure */
    stcSpiInit.u32WireMode          = SPI_4_WIRE;
    stcSpiInit.u32TransMode         = SPI_FULL_DUPLEX;
    stcSpiInit.u32MasterSlave       = SPI_MASTER;
    stcSpiInit.u32ModeFaultDetect   = SPI_MD_FAULT_DETECT_DISABLE;
    stcSpiInit.u32Parity            = SPI_PARITY_INVD;
    stcSpiInit.u32SpiMode           = SPI_MD_3;
    /* 【2026-11-02 降速排查】实测确认芯片为「带校验」版本（12bit + 4bit CRC4），
     * 但 CRC 通过率仅约 1%（100 次采样中 1 次 ok），说明通信质量有问题。
     *
     * 速度链：PCLK1 = 100 MHz（SYSCLK 200M / PCLK1_DIV2）
     *   DIV64  = 1.5625 MHz  T_SCK = 640 ns   ← 原值
     *   DIV128 = 0.7812 MHz  T_SCK = 1280 ns  ← 本次降到这档
     *
     * 手册要求 T_SCK >= 100 ns，DIV64 本已满足，故降速不是为了"满足手册"，
     * 而是为了排查**信号完整性问题**：
     *   - SCK 上升/下降沿过冲导致从机采样错误
     *   - MISO 建立时间不足（杜邦线较长 / 无屏蔽 / 上拉偏弱）
     *   - MOSI 与 SCK 走线串扰
     * 降速后若 CRC 通过率显著上升，即可确认是信号完整性问题，
     * 而非帧格式或 CRC 算法问题。
     *
     * 若 DIV128 仍不行，可继续试 DIV256（0.39 MHz, T_SCK=2560ns）。
     * 【降速只是诊断手段，定位后应按实测可用的最高速率回退】 */
    /* ============================================================
     * 【2026-11-02 回退到 6.25 MHz（DIV16）】
     * ------------------------------------------------------------
     * 12.5 MHz（DIV8）实测失败，用户报告"数据存在问题"，
     * 已按回退判据第 1 条处理 —— 与预期一致：T_SCK 80ns < 手册 100ns，
     * T_SCKL/T_SCKH 40ns < 手册 50ns，从机采样窗口不足。
     *
     * 【结论：6.25 MHz 是这颗芯片与当前布线下实测可用的最高速率】
     *   DIV64 = 1.5625 MHz  T_SCK = 640 ns  ✓ 已验证
     *   DIV32 = 3.125  MHz  T_SCK = 320 ns  ✓ 合规
     *   DIV16 = 6.25   MHz  T_SCK = 160 ns  ✓ 合规，当前
     *   DIV8  = 12.5   MHz  T_SCK =  80 ns  ✗ 实测失败，勿再用
     *
     * 【不要再尝试 DIV8 或更快】
     *   除非换更短/屏蔽更好的连接线并重做示波器验证，
     *   否则这已不是"再调调参数"能解决的问题，而是器件规格硬限。
     *
     * 【为什么不能靠提高 PCLK1 继续加速】
     *   T_SCK 只取决于最终 SCK 频率，与 PCLK1 无关。
     *   只要 T_SCK >= 100ns，SCK 上限就是 10 MHz。
     *   而 100 MHz 无法用 HC32 的 2 的幂分频得到 10 MHz。
     * ============================================================ */
    stcSpiInit.u32BaudRatePrescaler = SPI_BR_CLK_DIV16;
    stcSpiInit.u32DataBits          = SPI_DATA_SIZE_8BIT;
    stcSpiInit.u32FirstBit          = SPI_FIRST_MSB;
    (void)SPI_Init(BSP_SPI_UNIT, &stcSpiInit);

    /* 延时恢复 1 SCK。
     * DIV16 下 1 SCK = 160 ns，满足手册 T_H >= 120 ns。
     * （DIV8 时曾改 2 SCK，因为 1 SCK=80ns 不够；回退后 1 SCK 又够了，
     *   与分频一起恢复，保持与原验证版一致。） */
    stcSpiDelayCfg.u32IntervalDelay = SPI_INTERVAL_TIME_1SCK;
    stcSpiDelayCfg.u32ReleaseDelay  = SPI_RELEASE_TIME_1SCK;
    stcSpiDelayCfg.u32SetupDelay    = SPI_SETUP_TIME_1SCK;
    (void)SPI_DelayTimeConfig(BSP_SPI_UNIT, &stcSpiDelayCfg);
    SPI_Cmd(BSP_SPI_UNIT, ENABLE);
#ifdef USER_HC32_LL_SPI_DEV

#endif

#ifdef USER_RTT_SPI_DEV
    struct rt_spi_configuration cfg;


    //  KTH7812_SPI_DEVICE_NAME_41 ***************************************************************************************

//    rt_hw_spi_device_attach(SPI1_BUS_NAME, KTH7812_SPI_DEVICE_NAME_41, GPIO_PORT(GET_PIN(B,15)), GPIO_PIN(GET_PIN(B,15)));
    rt_hw_spi_device_attach(SPI1_BUS_NAME, KTH7812_SPI_DEVICE_NAME_41, GPIO_PORT_B, GPIO_PIN_15 );

//    spi_dev_kth7812_41 = (struct rt_spi_device *)rt_device_find(KTH7812_SPI_DEVICE_NAME_41);
    kth7812_config_data[KTH7812_DEV_41].spi_device = (struct rt_spi_device *)rt_device_find(KTH7812_SPI_DEVICE_NAME_41);
    if (!kth7812_config_data[KTH7812_DEV_41].spi_device)
    {
//        rt_kprintf("spi_dev_kth7812_41 run failed! can't find %s device!\n\n", KTH7812_SPI_DEVICE_NAME_41);
        return RT_ERROR;
    }
    else
    {
//        rt_kprintf("spi_dev_kth7812_41 run succeed\n\n");
    }

    // 配置 SPI 设备模式
    cfg.data_width = 8;
    cfg.mode = RT_SPI_MASTER | RT_SPI_MODE_3 | RT_SPI_MSB;
    cfg.max_hz = 0.1 * 1000 ;                           // 0.10MHz

    // 设置 SPI 设备配置
    if (rt_spi_configure(kth7812_config_data[KTH7812_DEV_41].spi_device, &cfg) != RT_EOK)
    {
//        rt_kprintf("Configure SPI device %s failed!\n", KTH7812_SPI_DEVICE_NAME_41);
        rt_free(kth7812_config_data[KTH7812_DEV_41].spi_device);
        return RT_ERROR;
    }

    // 打开 SPI 设备
    if (rt_device_open((rt_device_t)kth7812_config_data[KTH7812_DEV_41].spi_device, RT_DEVICE_FLAG_RDWR) != RT_EOK)
    {
//        rt_kprintf("Open SPI device %s failed!\n", KTH7812_SPI_DEVICE_NAME_41);
        return RT_ERROR;
    }
    else
    {
//        rt_kprintf("Open SPI device %s succeed!\n", KTH7812_SPI_DEVICE_NAME_41);
    }


    //  KTH7812_SPI_DEVICE_NAME_42 ***************************************************************************************

//    rt_hw_spi_device_attach(SPI1_BUS_NAME, KTH7812_SPI_DEVICE_NAME_42, GPIO_PORT(GET_PIN(A,8)), GPIO_PIN(GET_PIN(A,8)));
    rt_hw_spi_device_attach(SPI1_BUS_NAME, KTH7812_SPI_DEVICE_NAME_42, GPIO_PORT_A, GPIO_PIN_08 );

//    spi_dev_kth7812_42 = (struct rt_spi_device *)rt_device_find(KTH7812_SPI_DEVICE_NAME_42);
    kth7812_config_data[KTH7812_DEV_42].spi_device = (struct rt_spi_device *)rt_device_find(KTH7812_SPI_DEVICE_NAME_42);
    if (!kth7812_config_data[KTH7812_DEV_42].spi_device)
    {
//        rt_kprintf("spi_dev_kth7812_42 run failed! can't find %s device!\n\n", KTH7812_SPI_DEVICE_NAME_42);
        return RT_ERROR;
    }
    else
    {
//        rt_kprintf("spi_dev_kth7812_42 run succeed\n\n");
    }

    // 配置 SPI 设备模式
    cfg.data_width = 8;
    cfg.mode = RT_SPI_MASTER | RT_SPI_MODE_3 | RT_SPI_MSB;
    cfg.max_hz = 0.1 * 1000 ;                           // 0.10MHz

    // 设置 SPI 设备配置
    if (rt_spi_configure(kth7812_config_data[KTH7812_DEV_42].spi_device, &cfg) != RT_EOK)
    {
//        rt_kprintf("Configure SPI device %s failed!\n", KTH7812_SPI_DEVICE_NAME_42);
        rt_free(kth7812_config_data[KTH7812_DEV_42].spi_device);
        return RT_ERROR;
    }

    // 打开 SPI 设备
    if (rt_device_open((rt_device_t)kth7812_config_data[KTH7812_DEV_42].spi_device, RT_DEVICE_FLAG_RDWR) != RT_EOK)
    {
//        rt_kprintf("Open SPI device %s failed!\n", KTH7812_SPI_DEVICE_NAME_42);
        return RT_ERROR;
    }
    else
    {
//        rt_kprintf("Open SPI device %s succeed!\n", KTH7812_SPI_DEVICE_NAME_42);
    }

    //  KTH7812_SPI_DEVICE_NAME_41 ***************************************************************************************

//    rt_hw_spi_device_attach(SPI1_BUS_NAME, KTH7812_SPI_DEVICE_NAME_43, GPIO_PORT(GET_PIN(A,9)), GPIO_PIN(GET_PIN(A,9)));
    rt_hw_spi_device_attach(SPI1_BUS_NAME, KTH7812_SPI_DEVICE_NAME_43, GPIO_PORT_A, GPIO_PIN_09 );

//    spi_dev_kth7812_43 = (struct rt_spi_device *)rt_device_find(KTH7812_SPI_DEVICE_NAME_43);
    kth7812_config_data[KTH7812_DEV_43].spi_device = (struct rt_spi_device *)rt_device_find(KTH7812_SPI_DEVICE_NAME_43);
    if (!kth7812_config_data[KTH7812_DEV_43].spi_device)
    {
//        rt_kprintf("spi_dev_kth7812_43 run failed! can't find %s device!\n\n", KTH7812_SPI_DEVICE_NAME_43);
        return RT_ERROR;
    }
    else
    {
//        rt_kprintf("spi_dev_kth7812_43 run succeed\n\n");
    }

    // 配置 SPI 设备模式
    cfg.data_width = 8;
    cfg.mode = RT_SPI_MASTER | RT_SPI_MODE_3 | RT_SPI_MSB;
    cfg.max_hz = 0.1 * 1000 ;                           // 0.10MHz

    // 设置 SPI 设备配置
    if (rt_spi_configure(kth7812_config_data[KTH7812_DEV_43].spi_device, &cfg) != RT_EOK)
    {
//        rt_kprintf("Configure SPI device %s failed!\n", KTH7812_SPI_DEVICE_NAME_43);
        rt_free(kth7812_config_data[KTH7812_DEV_43].spi_device);
        return RT_ERROR;
    }

    // 打开 SPI 设备
    if (rt_device_open((rt_device_t)kth7812_config_data[KTH7812_DEV_43].spi_device, RT_DEVICE_FLAG_RDWR) != RT_EOK)
    {
//        rt_kprintf("Open SPI device %s failed!\n", KTH7812_SPI_DEVICE_NAME_43);
        return RT_ERROR;
    }
    else
    {
//        rt_kprintf("Open SPI device %s succeed!\n", KTH7812_SPI_DEVICE_NAME_43);
    }
#endif

    return RT_EOK;
}


void bsp_dev_spi_init(void)
{
   bsp_dev_spi_kth7812_init();
}
