/**
 * @file    param_port_efm.c
 * @brief   参数存储 EFM 端口：扇区擦除 / 字编程 / 读取 / CRC32
 * @note    擦写临界区是否关中断由 PARAM_EFM_CRITICAL_DISABLE_IRQ 控制，
 *          默认 0（不关中断），见 param_manager.h。
 */

#include "param_manager.h"

#include <string.h>
#include <rtthread.h>
#include <rthw.h>

#include "hc32_ll.h"
#include "hc32_ll_efm.h"

/* ============================================================
 * EFM 解锁 / 加锁
 * ============================================================ */
static void efm_unlock(void)
{
    LL_PERIPH_WE(LL_PERIPH_EFM);
    while (SET != EFM_GetStatus(EFM_FLAG_RDY)) {
        /* 等待 EFM 空闲 */
    }
    EFM_FWMC_Cmd(ENABLE);
}

static void efm_lock(void)
{
    EFM_FWMC_Cmd(DISABLE);
    LL_PERIPH_WP(LL_PERIPH_EFM);
}

/* ============================================================
 * 读取
 * ============================================================ */
void Param_EfmRead(uint32_t addr, uint8_t *buf, uint32_t len)
{
    if ((buf == RT_NULL) || (len == 0U)) {
        return;
    }
    EFM_ReadByte(addr, buf, len);
}

/* ============================================================
 * 擦除一个 8KB 扇区
 * ============================================================ */
int Param_EfmEraseSector(uint32_t sector_addr)
{
    int32_t ret;
#if PARAM_EFM_CRITICAL_DISABLE_IRQ
    rt_base_t level = rt_hw_interrupt_disable();
#endif

    efm_unlock();
    EFM_ClearStatus(EFM_FLAG_ALL);
    ret = EFM_SectorErase(sector_addr);
    efm_lock();

#if PARAM_EFM_CRITICAL_DISABLE_IRQ
    rt_hw_interrupt_enable(level);
#endif

    if (ret != LL_OK) {
        rt_kprintf("[PARAM] erase FAILED @0x%08X (ret=%d)\n",
                   (unsigned)sector_addr, (int)ret);
        return PARAM_ERR_IO;
    }
    return PARAM_OK;
}

/* ============================================================
 * 写入（4 字节对齐，内部回读校验）
 * ============================================================ */
int Param_EfmWrite(uint32_t addr, const uint8_t *buf, uint32_t len)
{
    uint32_t i;
    int32_t  ret = LL_OK;
#if PARAM_EFM_CRITICAL_DISABLE_IRQ
    rt_base_t level = rt_hw_interrupt_disable();
#endif

    if ((buf == RT_NULL) ||
        ((addr % 4U) != 0U) ||
        ((len % 4U) != 0U)) {
#if PARAM_EFM_CRITICAL_DISABLE_IRQ
        rt_hw_interrupt_enable(level);
#endif
        return PARAM_ERR_PARAM;
    }

    efm_unlock();

    for (i = 0U; i < len; i += 4U) {
        uint32_t word;

        memcpy(&word, buf + i, 4U);
        ret = EFM_ProgramWordReadBack(addr + i, word);
        if (ret != LL_OK) {
            break;
        }
    }

    efm_lock();

#if PARAM_EFM_CRITICAL_DISABLE_IRQ
    rt_hw_interrupt_enable(level);
#endif

    if (ret != LL_OK) {
        rt_kprintf("[PARAM] program FAILED @0x%08X (ret=%d)\n",
                   (unsigned)(addr + i), (int)ret);
        return PARAM_ERR_VERIFY;
    }
    return PARAM_OK;
}

/* ============================================================
 * CRC32（多项式 0xEDB88320，初值/结果异或 0xFFFFFFFF）
 * ============================================================ */
uint32_t Param_Crc32(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFU;
    uint32_t i;
    uint8_t  j;

    if (data == RT_NULL) {
        return 0U;
    }

    for (i = 0U; i < len; i++) {
        crc ^= (uint32_t)data[i];
        for (j = 0U; j < 8U; j++) {
            if ((crc & 1U) != 0U) {
                crc = (crc >> 1) ^ 0xEDB88320U;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc ^ 0xFFFFFFFFU;
}
