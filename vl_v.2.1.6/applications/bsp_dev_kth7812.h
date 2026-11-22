/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-01-29     gylocal       the first version
 */
#ifndef APPLICATIONS_BSP_DEV_KTH7812_H_
#define APPLICATIONS_BSP_DEV_KTH7812_H_

#include "app.h"
typedef struct{
    uint16_t op_code : 2;
    uint16_t op_addr : 6;
    uint16_t op_var  : 8;
}KTH7812_OP_DEF;

typedef union{
    uint16_t        kth7812_op_cmd;
    KTH7812_OP_DEF  kth7812_op_def;
}KTH7812_OP_CMD;

typedef struct{
    rt_base_t  pin;       /* pin 引脚 */
    rt_uint8_t mode;      /* pin 模式*/
    rt_uint8_t status;    /* pin 状态*/
}GPIO_CS_DEF;

typedef struct{
    GPIO_CS_DEF            cs_pin;
    struct rt_spi_device *spi_device;
    KTH7812_OP_CMD       kth7812_op_data;
}KTH7812_CONFIG_DEF;

typedef enum{
    KTH7812_REG_ADD_Z7 = 0x00,
    KTH7812_REG_ADD_Z15,
    KTH7812_REG_ADD_GAIN_TRIM,
    KTH7812_REG_ADD_Y_X_TRIM,
    KTH7812_REG_ADD_PPT1_ZL_ZD,
    KTH7812_REG_ADD_PPT9,
    KTH7812_REG_ADD_MGH_MGL,
    KTH7812_REG_ADD_NPP,
    KTH7812_REG_ADD_ABZ_LIMIT,
    KTH7812_REG_ADD_RD,
    KTH7812_REG_ADD_MAX
}KTH7812_REG_ADD_DEF;

extern KTH7812_CONFIG_DEF kth7812_config_data[KTH7812_DEV_MAX];


void KTH78_WriteReg(uint8_t regAddr, uint8_t value, KTH7812_DEV_DEF index);
uint8_t KTH78_ReadReg(struct rt_spi_device *device, uint8_t regAddr , KTH7812_DEV_DEF index );
//uint8_t KTH78_ReadReg(uint8_t regAddr, KTH7812_DEV_DEF index);
//uint16_t KTH78_ReadAngle(struct rt_spi_device *device ,KTH7812_DEV_DEF index );

/* 【2026-10-28 新增，MGL/MGH 探测用】
 *
 * KTH78_ReadRegRaw() 与 KTH78_ReadReg() 做同一件事，唯一差别是
 * 返回**完整 16bit 响应**而不是只返回高 8 位。排障必须看低字节：
 *   0x0000 → 从机未驱动总线（未供电/CS 错/MISO 断）
 *   0xFFFF → MISO 被上拉悬空（器件无响应）
 *   0xXX00 → 正常应答，XX 即寄存器值
 * 只看高字节时，0x0000 与「寄存器值恰为 0」无法区分。
 *
 * 用途：t_enc_reg / t_enc_regscan 命令，以及 MGL/MGH 的拆装磁铁
 * 对照实验。业务代码不要用它读配置，仍走 KTH78_ReadReg()。
 */
int  KTH78_ReadRegRaw(KTH7812_DEV_DEF index, uint8_t regAddr, uint16_t *resp);

/* 【2026-10-28 新增】写寄存器但不等待 MTP 写入周期（约 20ms）。
 * 仅供诊断命令在探测试验中反复试写；正式写配置必须用
 * KTH78_WriteReg()。MTP 擦写寿命约 1000 次，滥用会让芯片报废。 */
void KTH78_WriteRegNoWait(uint8_t regAddr, uint8_t value, KTH7812_DEV_DEF index);

/* 【2026-10-28 A-03】角度读取拆成两个入口，语义必须分清：
 *
 *   KTH78_ReadAngle()     已校验的 12bit 角度（0~4095），CRC 失败返回 0xFFFF。
 *                         ★ 业务代码唯一允许调用的入口。
 *   KTH78_ReadAngleRaw()  原始 16bit 帧（高12位角度 + 低4位CRC），不校验。
 *                         ★ 仅限诊断命令（t_enc_raw）用于定标与排障，
 *                           禁止拿它的结果直接参与角度换算。
 */
uint16_t KTH78_ReadAngle(KTH7812_DEV_DEF index );
uint16_t KTH78_ReadAngleRaw(KTH7812_DEV_DEF index );

/* CRC 失败 / 总线异常时 KTH78_ReadAngle() 的返回值哨兵。
 * 依据：4096 个合法帧中不含 0xFFFF（合法帧最大 0xFFF7）。 */
#define KTH7812_ANGLE_INVALID       (0xFFFFU)

/* ============================================================
 * 【2026-10-29 A-01】SPI 传输层健康度统计
 * ------------------------------------------------------------
 * 背景：MOSI/MISO 硬件交叉事故中，三颗编码器全寄存器读出 0x0000，
 * 而 CRC 因 0x0000 帧自洽显示 "ok"，硬件故障被伪装成正常数据。
 * 根因之一是传输返回值被丢弃。
 *
 * 现在 KTH78_SpiXfer16() 会在 SPI_TransReceive 失败时返回 0xFFFF
 * 并累加错误计数。用这两个函数可以把"传输层失败"与"数据为 0"
 * 彻底分开：
 *   ok/err 均为 0         → 还没读过
 *   err 持续增长           → 物理层有问题（接线/时钟/CS）
 *   err=0 但数据全 0      → 传输成功但器件没驱动 MISO
 * ============================================================ */
void KTH78_GetXferStat(uint32_t *ok, uint32_t *err);
void KTH78_ClrXferStat(void);


//extern static KTH7812_CONFIG_DEF kth7812_config_data[KTH7812_DEV_MAX];

#endif /* APPLICATIONS_BSP_DEV_KTH7812_H_ */
