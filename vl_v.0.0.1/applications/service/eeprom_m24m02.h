/**
 * @file    eeprom_m24m02.h
 * @brief   M24M02-DRMN6TP EEPROM 访问接口（2 Mbit = 256 KB，18 位地址）
 *
 * 【器件要点】
 *   容量 256KB → 地址 18 位，**必须用 2 字节地址域**（AddrH/AddrL）。
 *   AddrH 的 bit7~bit3 保留为 0，bit2~0 为地址的 bit18~16。
 *   页大小 256 字节，跨页写必须拆分成多次（页内回卷，否则数据覆盖）。
 *   写周期 tW 最大 5ms，用 ACK 轮询等待完成。
 *
 * 【I2C 地址】
 *   Device Type Identifier = 1010，A2/A1/A0 由硬件接线决定。
 *   默认按 A2=A1=A0=0 → 7 位地址 0x50。
 */

#ifndef __EEPROM_M24M02_H
#define __EEPROM_M24M02_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 器件参数
 * ============================================================ */
#define M24M02_ADDR7            (0x50U)         /* 7 位从机地址（A2~A0 接地） */
#define M24M02_SIZE             (262144UL)      /* 256 KB */
#define M24M02_PAGE_SIZE        (256U)          /* 页大小 */
#define M24M02_WRITE_TIMEOUT_MS (20U)           /* 单页写等待上限（tW 最大 5ms） */
#define M24M02_CHUNK_MAX        (256U)          /* 单次读写最大字节数 */

/* ============================================================
 * API
 * ============================================================ */

/**
 * @brief 初始化 I2C 引脚并探测器件 [单线程]
 * @return 0 成功（器件在线）；-1 失败
 */
int eeprom_init(void);

/**
 * @brief 器件是否可用
 */
uint8_t eeprom_is_ready(void);

/**
 * @brief 写数据（自动按页拆分） [单线程]
 * @param addr 起始地址（0 ~ M24M02_SIZE-1）
 * @param data 数据
 * @param len  长度
 * @return 0 成功；<0 失败
 */
int eeprom_write(uint32_t addr, const uint8_t *data, uint16_t len);

/**
 * @brief 读数据（可跨页，长度上限 M24M02_CHUNK_MAX） [单线程]
 */
int eeprom_read(uint32_t addr, uint8_t *data, uint16_t len);

/**
 * @brief 擦除模拟：把指定区域写成 0xFF（用于日志区清空）
 * @param len 必须为 M24M02_PAGE_SIZE 的整数倍
 */
int eeprom_fill_ff(uint32_t addr, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* __EEPROM_M24M02_H */
