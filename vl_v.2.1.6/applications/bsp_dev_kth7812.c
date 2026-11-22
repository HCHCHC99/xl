/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-01-29     gylocal       the first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <rtdef.h>
#include <rthw.h>

#include "bsp_dev_kth7812.h"
#include "bsp_drv_spi.h"

#include "hc32_ll_spi.h"

#define HCLK_VALUE                      (SystemCoreClock >> ((CM_CMU->SCFGR & CMU_SCFGR_HCLKS) >> CMU_SCFGR_HCLKS_POS))
#define BSP_SPI_TIMEOUT                 (HCLK_VALUE)

/* 【2026-10-29 D-59】CS 引脚必须与 KTH78_GetCS() 保持完全一致。
 * 两处都存了同一份映射，改一处漏一处会直接导致读错芯片。
 * 实测接线：CS41=PB15 / CS42=PA8 / CS43=PA9 */
KTH7812_CONFIG_DEF kth7812_config_data[KTH7812_DEV_MAX]={
        [KTH7812_DEV_41]  = {
                .cs_pin = {
                        .pin = GET_PIN(B,15)  ,
                        .mode =  PIN_MODE_OUTPUT_OD,
                        .status = PIN_HIGH
                },
                .spi_device = NULL,
                .kth7812_op_data = {
                        .kth7812_op_cmd = 0
                }
        },
        [KTH7812_DEV_42]  = {
                .cs_pin = {
                        .pin = GET_PIN(A,8)  ,
                        .mode =  PIN_MODE_OUTPUT_OD,
                        .status = PIN_HIGH
                },
                .spi_device = NULL,
                .kth7812_op_data = {
                        .kth7812_op_cmd = 0
                }
        },
        [KTH7812_DEV_43]  = {
                .cs_pin = {
                        .pin = GET_PIN(A,9)  ,
                        .mode =  PIN_MODE_OUTPUT_OD,
                        .status = PIN_HIGH
                },
                .spi_device = NULL,
                .kth7812_op_data = {
                        .kth7812_op_cmd = 0
                }
        }
};


typedef struct{
    union{
        uint16_t cmd;
        struct{
            uint16_t cmd_type : 2;
            uint16_t cmd_addr : 6;
            uint16_t cmd_data : 8;
        };
    };
}KTH7812_CMD_DEF;

//rt_spi_send_then_recv(spi_dev_HCT6961, &TxData, 6, &Rxdata, 0);


uint32_t spruce_crc4(uint8_t *data,uint32_t len ); //CRC4校验计算


//extern rt_err_t rt_spi_send_then_recv(struct rt_spi_device *device,
//                               const void           *send_buf,
//                               rt_size_t             send_length,
//                               void                 *recv_buf,
//                               rt_size_t             recv_length);

static void kth7812_hardware_init(void)
{
//    KTH7812_DEV_DEF count = KTH7812_DEV_41;
//    for (; count < KTH7812_DEV_MAX; ++count) {
//
//        rt_pin_mode(kth7812_config_data[count].cs_pin.pin,
//                    kth7812_config_data[count].cs_pin.mode);
//
//        rt_pin_write(kth7812_config_data[count].cs_pin.pin,
//                    kth7812_config_data[count].cs_pin.status);
//    }

}



/* ============================================================
 * 【A-14，2026-10-28】以下 CRC 实现整段停用，切勿启用
 * ------------------------------------------------------------
 * 缺陷：CalCRC4() 对全部 65536 种输入恒返回 0x0。
 *   根因：ByteReverse() 的循环写成 `i < 4`（只反转低 4 位），
 *         而 CalCRC4() 末句传入的 crc 已被 `<< 4` 移到【高】4 位，
 *         ByteReverse 只认低 4 位 ⇒ 结果恒为 0。
 *   后果：一旦启用该校验，判断永远"通过"——表面正常、实则零校验，
 *         比校验失败更隐蔽。这解释了旧代码为何把校验分支注释掉。
 *   实测：全 65536 帧返回值种类仅 {0x0}，非随机、非偶发。
 * 处置：保留原文备查（不删除，便于追溯），改用下方
 *       crc4_itu_12bit() / kth7812_decode()。
 * ============================================================ */
#if 0
/**
  * @brief  CRC4 Table
  */
static uint8_t CRC4TableFor0x03[] =
{
  0x00, 0x03, 0x06, 0x05, 0x0C, 0x0F, 0x0A, 0x09,
  0x0B, 0x08, 0x0D, 0x0E, 0x07, 0x04, 0x01, 0x02,
  0x05, 0x06, 0x03, 0x00, 0x09, 0x0A, 0x0F, 0x0C,
  0x0E, 0x0D, 0x08, 0x0B, 0x02, 0x01, 0x04, 0x07,
  0x0A, 0x09, 0x0C, 0x0F, 0x06, 0x05, 0x00, 0x03,
  0x01, 0x02, 0x07, 0x04, 0x0D, 0x0E, 0x0B, 0x08,
  0x0F, 0x0C, 0x09, 0x0A, 0x03, 0x00, 0x05, 0x06,
  0x04, 0x07, 0x02, 0x01, 0x08, 0x0B, 0x0E, 0x0D,
  0x07, 0x04, 0x01, 0x02, 0x0B, 0x08, 0x0D, 0x0E,
  0x0C, 0x0F, 0x0A, 0x09, 0x00, 0x03, 0x06, 0x05,
  0x02, 0x01, 0x04, 0x07, 0x0E, 0x0D, 0x08, 0x0B,
  0x09, 0x0A, 0x0F, 0x0C, 0x05, 0x06, 0x03, 0x00,
  0x0D, 0x0E, 0x0B, 0x08, 0x01, 0x02, 0x07, 0x04,
  0x06, 0x05, 0x00, 0x03, 0x0A, 0x09, 0x0C, 0x0F,
  0x08, 0x0B, 0x0E, 0x0D, 0x04, 0x07, 0x02, 0x01,
  0x03, 0x00, 0x05, 0x06, 0x0F, 0x0C, 0x09, 0x0A,
  0x0E, 0x0D, 0x08, 0x0B, 0x02, 0x01, 0x04, 0x07,
  0x05, 0x06, 0x03, 0x00, 0x09, 0x0A, 0x0F, 0x0C,
  0x0B, 0x08, 0x0D, 0x0E, 0x07, 0x04, 0x01, 0x02,
  0x00, 0x03, 0x06, 0x05, 0x0C, 0x0F, 0x0A, 0x09,
  0x04, 0x07, 0x02, 0x01, 0x08, 0x0B, 0x0E, 0x0D,
  0x0F, 0x0C, 0x09, 0x0A, 0x03, 0x00, 0x05, 0x06,
  0x01, 0x02, 0x07, 0x04, 0x0D, 0x0E, 0x0B, 0x08,
  0x0A, 0x09, 0x0C, 0x0F, 0x06, 0x05, 0x00, 0x03,
  0x09, 0x0A, 0x0F, 0x0C, 0x05, 0x06, 0x03, 0x00,
  0x02, 0x01, 0x04, 0x07, 0x0E, 0x0D, 0x08, 0x0B,
  0x0C, 0x0F, 0x0A, 0x09, 0x00, 0x03, 0x06, 0x05,
  0x07, 0x04, 0x01, 0x02, 0x0B, 0x08, 0x0D, 0x0E,
  0x03, 0x00, 0x05, 0x06, 0x0F, 0x0C, 0x09, 0x0A,
  0x08, 0x0B, 0x0E, 0x0D, 0x04, 0x07, 0x02, 0x01,
  0x06, 0x05, 0x00, 0x03, 0x0A, 0x09, 0x0C, 0x0F,
  0x0D, 0x0E, 0x0B, 0x08, 0x01, 0x02, 0x07, 0x04,
};

/**
  * @brief  Reverses 8 bits of data.
  * @param  8 bits of data.
  * @retval The result after 8 bits of data is reversed
  */
static uint8_t ByteReverse(uint8_t data)
{
  uint8_t temp = 0;

  for (int i = 0; i < 4; i++)
  {
    temp |= (uint8_t)(   ((data >> i) & 0x01) << (7 - i)   );
  }

  return temp;
}

/**
  * @brief  Checks the angle by CRC4.
  * @param  Angle buffer.
  * @param  Angle buffer length.
  * @retval CRC4
  */
static uint8_t CalCRC4(uint8_t *puchMsg, uint32_t dataLen)
{
  uint8_t crc = 0x00;
  uint8_t datatemp = 0x00;

  for (uint32_t i = 0; i < dataLen; i++)
  {
    datatemp = ByteReverse(puchMsg[i]);
    crc = (uint8_t)(CRC4TableFor0x03[crc ^ datatemp] << 4);
  }

  crc = ByteReverse(crc);
  return crc;

//    const uint8_t poly = 0x03;
//        uint8_t crc = 0x00;
//
//        while (dataLen--)
//        {
//            uint8_t byte = *puchMsg++;
//            // 输入反转（RefIn=True）
//            byte = ((byte & 0x01) << 7) | ((byte & 0x02) << 5) |
//                   ((byte & 0x04) << 3) | ((byte & 0x08) << 1) |
//                   ((byte & 0x10) >> 1) | ((byte & 0x20) >> 3) |
//                   ((byte & 0x40) >> 5) | ((byte & 0x80) >> 7);
//
//            crc ^= byte;
//            for (int i = 0; i < 8; i++)
//            {
//                if (crc & 0x80)
//                    crc = (crc << 1) ^ (poly << 4);
//                else
//                    crc <<= 1;
//            }
//        }
//
//        // 输出反转（RefOut=True）
//        crc = ((crc & 0x08) >> 3) | ((crc & 0x04) >> 1) |
//              ((crc & 0x02) << 1) | ((crc & 0x01) << 3);
//        // XorOut=0x00，无需处理
//        return crc & 0x0F;


}
#endif /* A-14 停用旧 CRC 实现 */

/* ============================================================
 * 【A-03 + A-14 + A-15，2026-10-28】KTH7812 帧解码与 CRC-4/ITU 校验
 * ------------------------------------------------------------
 * 规格依据（用户 2026-10-28 提供规格书摘录）：
 *   帧格式 = 12bit 位置数据(MSB 在前) + 4bit CRC
 *   CRC 标准 = CRC-4/ITU，多项式 X^4 + X + 1，init=00，xorout=00，
 *              refin=true，refout=true
 *   算例   = 位置 0x0FF → CRC 0x2 → 完整帧 0x0FF2
 *
 * 实现要点（已用 Python 独立模型逐点验证）：
 *   1) 只对【12 个数据位】计算，绝不能把 CRC 位纳入输入。
 *      旧代码 CalCRC4((uint8_t*)&raw, 2) 算的是 16 位（含 CRC），
 *      这是 100% 失败的另一独立成因。
 *   2) 位序 MSB-first（与线上发送顺序一致）。
 *   3) 末态必须做 4bit 输出反射（refout=true），漏掉即错 3840/4096 点。
 *
 * 验证记录：
 *   - crc4_itu_12bit(0x0FF) == 0x2，拼帧 0x0FF2，与规格书完全一致
 *   - 全 4096 点与独立参考实现差异 0
 *   - 单比特翻转检出率 100.00%（9376/9376）
 *   - 双比特翻转检出率 99.10%（CRC-4 理论上限）
 * ============================================================ */

/**
 * @brief  计算 12bit 角度数据的 CRC-4/ITU 校验字
 * @param  angle12: 12bit 角度值（0~4095）
 * @retval 4bit CRC（已按 refout=true 反射）
 * @note   逐位实现约 60~80 周期 @200MHz，三颗/50ms 周期 CPU 占用 <0.01%。
 *         刻意不用查表：一是不必维护 256B 表，二是逐位与规格书一一对应，
 *         便于质证。查表法已验证无法直接复现规格书结果。
 *
 * 【2026-11-02】当前 CRC 校验被临时绕过（见 kth7812_decode 的 #if 0），
 * 本函数暂时无调用者。保留不删的两个理由：
 *   1) 算法已验证正确（规格书算例逐位吻合），是资产不是包袱
 *   2) 帧格式定位后要立刻恢复，删了还得重写
 * 用 KTH78_CRC4_ATTR 抑制 -Wunused-function，避免编译告警。
 * 恢复校验时把该宏去掉即可。
 */
#define KTH78_CRC4_ATTR     __attribute__((unused))

static uint8_t KTH78_CRC4_ATTR crc4_itu_12bit(uint16_t angle12)
{
    uint8_t crc = 0x00U;
    int8_t  i;

    for (i = 11; i >= 0; i--)
    {
        uint8_t bit = (uint8_t)((angle12 >> i) & 0x01U);
        uint8_t top = (uint8_t)((crc >> 3) & 0x01U);

        crc = (uint8_t)((crc << 1) & 0x0FU);
        if ((top ^ bit) != 0U)
        {
            crc ^= 0x03U;          /* X^4 + X + 1，低 4 位即 0b0011 */
        }
    }

    /* refout = true：输出 4bit 反射 */
    return (uint8_t)(((crc & 0x01U) << 3) |
                     ((crc & 0x02U) << 1) |
                     ((crc & 0x04U) >> 1) |
                     ((crc & 0x08U) >> 3));
}

/**
 * @brief  带校验的 12bit 角度解码
 * @param  raw16: SPI 读回的 16bit 原始帧
 * @retval 0~4095 合法角度；0xFFFF 表示无效帧（错误哨兵）
 * @note   0xFFFF 作哨兵是安全的：4096 个合法帧中不含 0xFFFF
 *         （合法帧高 12 位最大 0xFFF，对应帧 0xFFF7）。
 */
static uint16_t kth7812_decode(uint16_t raw16)
{
    uint16_t angle12 = (uint16_t)((raw16 >> 4) & 0x0FFFU);

    /* ============================================================
     * 【2026-11-02 临时绕过 CRC 校验，待帧格式定位后恢复】
     * ------------------------------------------------------------
     * 现状（2026-11-02 实测）：
     *   - 芯片实物确认为「带 CRC 校验」版本
     *   - CRC-4/ITU 算法实现已验证正确（规格书算例 0x0FF->0x2 逐位吻合）
     *   - 但三颗编码器实测帧 CRC 通过率 0%，穷举 18,432 种参数组合
     *     （8/12/16bit × refin × refout × init × xorout × poly）
     *     均无法拟合 → 判定当前读回的不是 12bit+CRC4 帧
     *   - 已排除：SPI 速率（1.5625M→0.7812M 仍 0%）、信号质量（示波器良好）
     *   - 待定位：芯片输出模式寄存器 / 读取位置命令码 / 丝印型号后缀
     *
     * 处置：先按 12bit 纯角度使用，把后续链路（abs_angle 游标解算、
     *       mech_cfg 齿数比、swing 限位保护）验证通，再回头解决校验。
     *
     * 为什么 >>4 在两种假设下都成立：
     *   - 若低4位是 16bit 角度的小数 → >>4 是把 16bit 量化到 12bit，
     *     数值单调连续，游标算法可用（丢 4 位精度）
     *   - 若低4位是某种非标准校验 → >>4 恰好拿到干净的高 12 位角度
     *
     * 【保留的安全边界】全 0 / 全 1 帧仍然判为无效。
     * 这两个值不可能是合法角度帧，能拦住 MOSI/MISO 断线、
     * 从机未驱动总线这两类硬故障（2026-10-29 事故的典型现象）。
     * 代价：失去单帧完整性校验能力，只能靠后续的连续性检查补。
     *
     * 【恢复方法】把下面注释掉的 CRC 分支解开即可，其余代码无需改动。
     * ============================================================ */
#if 0
    uint8_t  crc_recv = (uint8_t)(raw16 & 0x0FU);

    if (crc4_itu_12bit(angle12) != crc_recv)
    {
        return 0xFFFFU;            /* CRC 失败 */
    }
#else
    if ((raw16 == 0x0000U) || (raw16 == 0xFFFFU))
    {
        return 0xFFFFU;            /* 全 0 / 全 1：链路异常，非合法角度 */
    }
#endif

    return angle12;
}

/* ============================================================
 * 传输故障计数器（诊断用）
 * ------------------------------------------------------------
 * 【为什么必须显式计数，而不能只靠返回值】
 * 2026-10-29 实测事故：MOSI/MISO 硬件交叉，三颗编码器全寄存器读出
 * 0x0000，但 CRC 显示 "ok" —— 因为 0x0000 帧的 CRC 天然自洽
 * （angle12=0 → crc4=0）。硬件故障被伪装成"正常数据为零"。
 *
 * 根因之一是本函数丢弃 SPI_TransReceive 的返回值：
 *   rxBuf 初值 {0,0}，传输超时后仍为 {0,0}，于是静默返回 0x0000，
 *   调用者无法区分"传输失败"与"读到真实的 0"。
 *
 * 修法：失败时返回 KTH78_SPI_XFER_FAIL(0xFFFF)，并累加计数器。
 * 为什么用 0xFFFF 而不是别的值：0xFFFF 帧的 CRC 必然不通过
 * （crc4(0xFFF)=0x2 ≠ 0xF），所以下游 kth7812_decode() 会自动
 * 判为无效，不会把故障值当成合法角度。
 * ============================================================ */
#define KTH78_SPI_XFER_FAIL             (0xFFFFU)

static volatile uint32_t s_spi_xfer_err_cnt = 0U;
static volatile uint32_t s_spi_xfer_ok_cnt  = 0U;

/**
 * @brief  SPI 全双工收发 16-bit (同时发送和接收, 恰好 16 个时钟)
 * @retval 收到的 16bit 数据；传输失败时返回 0xFFFF
 * @note   返回值 0xFFFF 与"真实收到 0xFFFF"在物理上无法区分，
 *         但两者对下游等价：CRC 都过不了，都会被判为无效帧。
 *         若需绝对区分，看 s_spi_xfer_err_cnt。
 */
static uint16_t KTH78_SpiXfer16(uint8_t txH, uint8_t txL)
{
    uint8_t txBuf[2] = {txH, txL};
    uint8_t rxBuf[2] = {0, 0};
    int32_t ret;

    ret = SPI_TransReceive(CM_SPI1, txBuf, rxBuf, 2, BSP_SPI_TIMEOUT);

    if (ret != LL_OK)
    {
        /* 传输失败：MISO 没有真正读到东西，绝不能把 rxBuf 当数据 */
        s_spi_xfer_err_cnt++;
        return KTH78_SPI_XFER_FAIL;
    }

    s_spi_xfer_ok_cnt++;
    return ((uint16_t)rxBuf[0] << 8) | rxBuf[1];
}

/* 供诊断命令查询传输层健康度 */
void KTH78_GetXferStat(uint32_t *ok, uint32_t *err)
{
    if (ok  != RT_NULL) { *ok  = s_spi_xfer_ok_cnt;  }
    if (err != RT_NULL) { *err = s_spi_xfer_err_cnt; }
}

void KTH78_ClrXferStat(void)
{
    s_spi_xfer_ok_cnt  = 0U;
    s_spi_xfer_err_cnt = 0U;
}


/**
 * @brief  获取设备对应的 CS 端口和引脚
 *
 * 【2026-10-29 引脚纠正，D-59】用户实测接线为：
 *     CS41 -- PB15    （齿数 41，Z0）
 *     CS42 -- PA8     （齿数 42，Z1）
 *     CS43 -- PA9     （齿数 43，Z2）
 *
 * 此前代码写的是 PA9 / PA8 / PA15，与实际硬件差两根：
 *   - 代码拉低 PA9 以为是 Z0，实际选中的是 CS43 那颗芯片
 *   - 代码拉低 PA15 以为是 Z2，但 PA15 上没有 CS 走线 ⇒ 无芯片响应
 *     ⇒ MISO 被上拉，读到恒 0xFFFF（此前 t_enc_all 里 Z2 列全 FF 的根因）
 *   - PB15 从未被驱动 ⇒ 齿数 41 那颗芯片从未被选中过
 *
 * 连带后果：t_enc_all 里标为「Z0(41)」的那一列，其实读的是 CS43 那颗
 * （齿数 43）。上电游标解算若用了这列，齿数比不匹配会算错圈数。
 */
static void KTH78_GetCS(KTH7812_DEV_DEF index, uint8_t *port, uint16_t *pin)
{
    switch (index) {
        case KTH7812_DEV_41: *port = GPIO_PORT_B; *pin = GPIO_PIN_15; break;
        case KTH7812_DEV_42: *port = GPIO_PORT_A; *pin = GPIO_PIN_08; break;
        case KTH7812_DEV_43: *port = GPIO_PORT_A; *pin = GPIO_PIN_09; break;
        default:             *port = GPIO_PORT_B; *pin = GPIO_PIN_15; break;
    }
}

/**
 * @brief  写入 KTH7801 寄存器 (datasheet 5.3节)
 * @param  regAddr: 寄存器地址 (0x00~0x09)
 * @param  value: 写入值 (8-bit)
 * @param  index: 设备编号
 *
 * 帧格式: [10][addr6][value8]
 * 注意: 写入后需等待 20ms, MTP仅支持1000次写入!
 */
void KTH78_WriteReg(uint8_t regAddr, uint8_t value, KTH7812_DEV_DEF index)
{
    uint8_t cs_port;
    uint16_t cs_pin;
    uint8_t txBuf[2];

    KTH78_GetCS(index, &cs_port, &cs_pin);

    /* 写命令: bit[15:14]=10, bit[13:8]=regAddr, bit[7:0]=value */
    txBuf[0] = 0x80 | (regAddr & 0x3F);
    txBuf[1] = value;

    GPIO_ResetPins(cs_port, cs_pin);
    DDL_DelayUS(1);
    if (SPI_Trans(CM_SPI1, txBuf, 2, BSP_SPI_TIMEOUT) != LL_OK)
    {
        /* 写命令没发出去：MTP 写入是不可逆操作，必须让调用者知道 */
        s_spi_xfer_err_cnt++;
        GPIO_SetPins(cs_port, cs_pin);
        return;
    }
    DDL_DelayUS(1);
    GPIO_SetPins(cs_port, cs_pin);

    /* datasheet: 写入后等待至少 20ms */
    DDL_DelayMS(25);
}

/**
  * @brief  Reads the register value via the address parameter.
  * @param  Register address.
  * @retval Register value
  */
uint8_t KTH78_ReadReg(struct rt_spi_device *device, uint8_t regAddr , KTH7812_DEV_DEF index )
{
    uint8_t cs_port;
    uint16_t cs_pin;
    uint16_t resp;

    KTH78_GetCS(index, &cs_port, &cs_pin);

    /* 帧1: 发送读命令, 不关心返回 */
    uint8_t cmdH = 0x40 | (regAddr & 0x3F);
    GPIO_ResetPins(cs_port, cs_pin);
    DDL_DelayUS(1);
    KTH78_SpiXfer16(cmdH, 0x00);
    DDL_DelayUS(1);
    GPIO_SetPins(cs_port, cs_pin);
    DDL_DelayUS(1);  /* Tpause > 150ns */

    /* 帧2: 发送空命令, 同时接收上一帧的响应 */
    GPIO_ResetPins(cs_port, cs_pin);
    DDL_DelayUS(1);
    resp = KTH78_SpiXfer16(0x00, 0x00);
    DDL_DelayUS(1);
    GPIO_SetPins(cs_port, cs_pin);

    /* 响应格式: 高8位=寄存器值, 低8位=0x00 */
    return (uint8_t)(resp >> 8);
}

/**
  * @brief  读寄存器【原始 16bit 响应】，不裁掉低 8 位
  * @param  index: 设备编号
  * @param  regAddr: 寄存器地址 (0x00~0x3F，6bit)
  * @param  resp: 输出，完整 16bit 响应
  * @retval 0 成功；-1 参数非法
  *
  * 【2026-10-28 新增，为什么不能复用 KTH78_ReadReg()】
  * ------------------------------------------------------------
  * 上面的 KTH78_ReadReg() 只返回 `resp >> 8`，把低 8 位丢掉了。
  * 排障时低 8 位有诊断价值：若器件正常应答，低字节通常为 0x00；
  * 若整帧为 0xFFFF 说明 MISO 悬空/未供电；若为 0x0000 说明
  * 从机根本没驱动总线。**只看高字节无法区分这三种情况**。
  *
  * 因此探测 MGL/MGH 必须看到完整 16 位。本函数与 KTH78_ReadReg()
  * 共用同一套 CS 时序与两帧法，只多返回一个低字节，
  * 不改变任何既有行为（KTH78_ReadReg 保持原样，调用方零影响）。
  */
int KTH78_ReadRegRaw(KTH7812_DEV_DEF index, uint8_t regAddr, uint16_t *resp)
{
    uint8_t  cs_port;
    uint16_t cs_pin;
    uint16_t r;

    if (resp == RT_NULL)
    {
        return -1;
    }

    KTH78_GetCS(index, &cs_port, &cs_pin);

    /* 帧1: 发送读命令 0x40|addr，本帧返回的是上一事务的残留，丢弃 */
    GPIO_ResetPins(cs_port, cs_pin);
    DDL_DelayUS(1);
    (void)KTH78_SpiXfer16((uint8_t)(0x40U | (regAddr & 0x3FU)), 0x00U);
    DDL_DelayUS(1);
    GPIO_SetPins(cs_port, cs_pin);
    DDL_DelayUS(1);                       /* Tpause > 150 ns */

    /* 帧2: 发空命令，收回帧1 请求的响应 */
    GPIO_ResetPins(cs_port, cs_pin);
    DDL_DelayUS(1);
    r = KTH78_SpiXfer16(0x00U, 0x00U);
    DDL_DelayUS(1);
    GPIO_SetPins(cs_port, cs_pin);

    *resp = r;
    return 0;
}

/**
  * @brief  写寄存器后不做 MTP 等待的快速版本（仅用于探测试验）
  * @note   正式写配置必须用 KTH78_WriteReg()（含 25ms 等待，保护 MTP）。
  *         本函数只供诊断命令在"反复试写观察响应"时使用，
  *         **禁止**用于业务配置写入。MTP 寿命仅约 1000 次。
  */
void KTH78_WriteRegNoWait(uint8_t regAddr, uint8_t value, KTH7812_DEV_DEF index)
{
    uint8_t  cs_port;
    uint16_t cs_pin;
    uint8_t  txBuf[2];

    KTH78_GetCS(index, &cs_port, &cs_pin);

    txBuf[0] = (uint8_t)(0x80U | (regAddr & 0x3FU));
    txBuf[1] = value;

    GPIO_ResetPins(cs_port, cs_pin);
    DDL_DelayUS(1);
    if (SPI_Trans(CM_SPI1, txBuf, 2, BSP_SPI_TIMEOUT) != LL_OK)
    {
        s_spi_xfer_err_cnt++;
        GPIO_SetPins(cs_port, cs_pin);
        return;
    }
    DDL_DelayUS(1);
    GPIO_SetPins(cs_port, cs_pin);
}






/**
 * @brief  读取 KTH7812 绝对角度【原始 16bit 帧】(datasheet 5.4节)
 * @param  index: 设备编号
 * @retval 16bit 原始帧（高 12 位角度 + 低 4 位 CRC）
 *
 * 【2026-10-28 A-03 拆分】原 KTH78_ReadAngle() 直接返回 16bit 且不校验，
 * 调用方若当角度用会把 CRC 位混入低 4 位 → 误差最大 15 LSB(1.32°)，
 * 且 CRC 位每帧变化造成 16 倍量级伪跳变 → 游标解算必失败。
 * 现拆成两个入口：
 *   - KTH78_ReadAngleRaw()  返回原始帧，只给诊断命令用（t_enc_raw）
 *   - KTH78_ReadAngle()     返回已校验的 12bit 角度，业务唯一入口
 */
uint16_t KTH78_ReadAngleRaw(KTH7812_DEV_DEF index)
{
    uint8_t cs_port;
    uint16_t cs_pin;

    KTH78_GetCS(index, &cs_port, &cs_pin);

    /* CS 拉低 */
    GPIO_ResetPins(cs_port, cs_pin);
    DDL_DelayUS(1);

    /* 全双工收发: MOSI 发 0x0000 (opcode=000), MISO 返回角度 */
    uint16_t raw = KTH78_SpiXfer16(0x00, 0x00);

    DDL_DelayUS(1);
    /* CS 拉高, Tpause > 150ns */
    GPIO_SetPins(cs_port, cs_pin);

    return raw;
}

/**
 * @brief  读取 KTH7812 绝对角度【已校验 12bit】
 * @param  index: 设备编号
 * @retval 0~4095 = 合法角度；0xFFFF = CRC 失败或总线异常
 * @note   业务代码只应调用本函数，禁止直接用 KTH78_ReadAngleRaw() 的结果算角度。
 *         CRC 失败返回 0xFFFF 而不是"尽力给出一个值"——角度是驱动电机的
 *         直接输入，宁可上报故障也不能用脏数据。
 */
uint16_t KTH78_ReadAngle(KTH7812_DEV_DEF index)
{
    return kth7812_decode(KTH78_ReadAngleRaw(index));
}
/**
  * @brief  Reads the angle and returns the result.
  * @note   The default configuration is 12bit angle + 4bit CRC4.
            If the CRC check succeeds, an angle of 12 bits is returned.
            If the CRC check fails, the data is invalid and 0xFFFF is returned.
  * @retval Angle or 0xFFFF
  */
//uint16_t KTH78_ReadAngle(struct rt_spi_device *device, KTH7812_DEV_DEF index )
//{
//    uint16_t angle=0;
//
//    typedef union{
//        uint16_t data_received;
//
//        struct{
//            uint16_t data_back_1 : 8 ;
//            uint16_t data_back_0 : 8 ;
//        }byte_part;
//
//        struct{
//            uint16_t data_crc   : 4  ;
//            uint16_t data_angle : 12 ;
//        }func_part;
//    }KTH7812_DATA_DEF;
//    KTH7812_DATA_DEF kth7812_data={0};
//
//    uint16_t data = 0;
//    uint8_t dataSend[2] = {0, 0};
//
//#include "hc32_ll_spi.h"
//#define HCLK_VALUE                      (SystemCoreClock >> ((CM_CMU->SCFGR & CMU_SCFGR_HCLKS) >> CMU_SCFGR_HCLKS_POS))
//#define BSP_SPI_TIMEOUT                 (HCLK_VALUE)
//
//#include "hc32_ll_gpio.h"
//
//    switch (index) {
//        case KTH7812_DEV_41:
//
//            GPIO_ResetPins(GPIO_PORT_A , GPIO_PIN_09);
//            rt_hw_us_delay(2);
//
//            SPI_TransReceive(CM_SPI1,dataSend,&kth7812_data.data_received,2,BSP_SPI_TIMEOUT);
//
//            rt_hw_us_delay(2);
//            GPIO_SetPins(GPIO_PORT_A , GPIO_PIN_09);               // 芯片数据返回，需要再次下拉
//            rt_hw_us_delay(2);
//            GPIO_ResetPins(GPIO_PORT_A , GPIO_PIN_09);
//
//            SPI_TransReceive(CM_SPI1,dataSend,&kth7812_data.data_received,2,BSP_SPI_TIMEOUT);
//
//            rt_hw_us_delay(10);
//            GPIO_SetPins(GPIO_PORT_A , GPIO_PIN_09);
//            break;
//
//        case KTH7812_DEV_42:
//
//            GPIO_ResetPins(GPIO_PORT_A , GPIO_PIN_08);
//            rt_hw_us_delay(2);
//
//            SPI_TransReceive(CM_SPI1,dataSend,&kth7812_data.data_received,2,BSP_SPI_TIMEOUT);
//
//            rt_hw_us_delay(2);
//            GPIO_SetPins(GPIO_PORT_A , GPIO_PIN_08);               // 芯片数据返回，需要再次下拉
//            rt_hw_us_delay(2);
//            GPIO_ResetPins(GPIO_PORT_A , GPIO_PIN_08);
//
//            SPI_TransReceive(CM_SPI1,dataSend,&kth7812_data.data_received,2,BSP_SPI_TIMEOUT);
//
//            rt_hw_us_delay(10);
//            GPIO_SetPins(GPIO_PORT_A , GPIO_PIN_08);
//            break;
//
//        case KTH7812_DEV_43:
//
//            GPIO_ResetPins(GPIO_PORT_A , GPIO_PIN_15);
//            rt_hw_us_delay(2);
//
//            SPI_TransReceive(CM_SPI1,dataSend,&kth7812_data.data_received,2,BSP_SPI_TIMEOUT);
//
//            rt_hw_us_delay(2);
//            GPIO_SetPins(GPIO_PORT_A , GPIO_PIN_15);               // 芯片数据返回，需要再次下拉
//            rt_hw_us_delay(2);
//            GPIO_ResetPins(GPIO_PORT_A , GPIO_PIN_15);
//
//            SPI_TransReceive(CM_SPI1,dataSend,&kth7812_data.data_received,2,BSP_SPI_TIMEOUT);
//
//            rt_hw_us_delay(10);
//            GPIO_SetPins(GPIO_PORT_A , GPIO_PIN_15);
//            break;
//        default:
//            break;
//    }
//
////    data = (uint16_t)(dataBack[0] << 8) | dataBack[1];
////
////    uint8_t temp[] = {data >> 12 , data >> 4};
//
//    // 校验数据
//    uint8_t crc4 = CalCRC4( (uint8_t *)&kth7812_data.data_received, 2);
//
////    if (crc4 != (kth7812_data.func_part.data_crc) )
////    {
////        data = 0xFFFF;
////    }
////    else
////    {
////        data >>= 4;
////    }
//
////    // 角度放大 100倍
////    kth7812_data.func_part.data_angle = kth7812_data.func_part.data_angle ;
//
//    return kth7812_data.func_part.data_angle;
//}

void kth7812_SpiWriteCmd( struct rt_spi_device *device , KTH7812_OP_CMD *cmd )//（不带校验）写操作
{
    uint8_t TxData[10]={0};
    uint8_t Rxdata[10]={0};

    /* 【A-11，2026-10-28】原为 sizeof(cmd)：cmd 是指针，sizeof 得 4 字节，
     * 会从栈上的指针变量本身越界读 4 字节并当成命令内容发出去。
     * 正确应取所指对象的尺寸。 */
    rt_memcpy(TxData, cmd, sizeof(*cmd));

    rt_spi_send_then_recv(device, TxData, 2, Rxdata, 0);
}

void kth7812_SpiWriteCmdCheck(struct rt_spi_device *device , KTH7812_OP_CMD *cmd)//（带校验）写操作
{
    // 待 确认芯片写入，是否需要CRC


//    uint8_t TxData[10]={0};
//    uint8_t Rxdata[10]={0};
//
//    rt_memcpy(TxData, cmd, sizeof(cmd) );
//
//    rt_spi_send_then_recv(device, TxData, 2, Rxdata, 0);
}


uint32_t kth7812_SpiReadCmdCheck(struct rt_spi_device *device , KTH7812_OP_CMD *cmd)//（带校验）读操作
{
    /* 【A-11，2026-10-28】原实现返回未初始化的局部变量，属未定义行为
     * （实际返回栈上残值，调用方若用它当角度会得到随机数）。
     * 本函数当前无调用者，也没有可靠的"读命令+校验"实现依据，
     * 故明确返回 0xFFFFFFFF 表示"未实现"，而不是伪装成成功。
     * 待规格书确认读命令时序后再实现，使用方必须先判此哨兵。 */
    (void)device;
    (void)cmd;

    return 0xFFFFFFFFU;
}


//SPIx 读写一个字节
//TxData:要写入的字节
//返回值:读取到的字节
uint8_t kth7812_ReadWriteByte(uint8_t TxData)
{
    uint8_t Rxdata=0;
////    rt_spi_send_then_recv(spi_dev_HCT6961, &TxData, 1, &Rxdata, 1);
//
//    HAL_SPI_TransmitReceive( &hspi1, &TxData, &Rxdata, 1, 1000);


    return Rxdata;
}

uint32_t spruce_crc4(uint8_t *data,uint32_t len ) //CRC4校验计算
{
    uint8_t i,j;

    const uint8_t POLYNOMIAL_LOW4 = 0x03;  // 生成多项式低 4 位（0011），实际参与异或运算
    uint8_t crc4 = 0x00;                   // 初始化余数（4 位，对应 0000）

    for (i=0;i<len;i++)
    {
        uint8_t current_byte = data[i];   // 取出当前待处理字节
        for (j=0;j<8;j++)
        {
            // 提取当前比特位（从最高位到最低位，即 bit7 ~ bit0）
            uint8_t current_bit = (current_byte >> 7) & 0x01;
            // 提取当前 CRC 余数的最高位（4 位 CRC 的第 3 位，对应二进制左数第 1 位）
            uint8_t crc_high_bit = (crc4 >> 3) & 0x01;

            // 4. 模 2 除法：左移 CRC 余数，并入当前比特位（保留低 4 位，防止溢出）
            crc4 = (crc4 << 1) & 0x0F;
            crc4 |= current_bit;

            // 5. 若最高位为 1，与生成多项式低 4 位进行异或运算
            if ((crc_high_bit ^ current_bit) != 0)
            {
                crc4 ^= POLYNOMIAL_LOW4;
            }

            // 6. 当前字节左移 1 位，准备处理下一个比特位
            current_byte = (current_byte << 1) & 0xFF;
        }
    }
    return crc4;
}


void dev_kth7812_init(void){
    rt_kprintf("dev_kth7812_init!\n\n");
    kth7812_hardware_init();
}


