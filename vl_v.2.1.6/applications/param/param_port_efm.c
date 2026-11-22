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
 * 应用镜像结束地址（防止擦写参数区时误伤代码）
 * ------------------------------------------------------------
 * GCC 链接脚本提供 _etext / __etext_ret_ram；
 * 弱引用未定义时地址为 0，表示当前工具链不做重叠检查。
 * ============================================================ */
#if defined(__GNUC__)
extern uint32_t _etext __attribute__((weak));
extern uint32_t __etext_ret_ram __attribute__((weak));
#endif

uint32_t Param_GetCodeEnd(void)
{
#if defined(__GNUC__)
    uint32_t a = (uint32_t)&_etext;
    uint32_t b = (uint32_t)&__etext_ret_ram;

    return (a > b) ? a : b;
#else
    return 0U;
#endif
}

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

    /* 【必须】单 bank Flash：擦写期间让总线停等（取指/数据访问挂起）。
     * 不设置时，擦写窗口内的取指（含 20kHz FOC/DMA 中断）会与 Flash
     * 操作冲突，表现为 EFM_ProgramWordReadBack 返回 PGMISMTCH(LL_ERR)，
     * 即运行时 program FAILED。参考 FA04A hc32_drv_flash.c 的做法。 */
    EFM_SetBusStatus(EFM_BUS_HOLD);

    EFM_ClearStatus(EFM_FLAG_ALL);
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
    int32_t  ret;
    uint32_t code_end = Param_GetCodeEnd();
#if PARAM_EFM_CRITICAL_DISABLE_IRQ
    rt_base_t level;
#endif

    /* 防误擦代码：参数区必须在应用镜像之后 */
    if ((code_end != 0U) && (sector_addr < code_end)) {
        rt_kprintf("[PARAM] refuse erase 0x%08X: overlaps code (code_end=0x%08X)\n",
                   (unsigned)sector_addr, (unsigned)code_end);
        return PARAM_ERR_PARAM;
    }

#if PARAM_EFM_CRITICAL_DISABLE_IRQ
    level = rt_hw_interrupt_disable();
#endif

    efm_unlock();
    EFM_ClearStatus(EFM_FLAG_ALL);
    ret = EFM_SectorErase(sector_addr);
    EFM_ClearStatus(EFM_FLAG_ALL);
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
    uint32_t code_end = Param_GetCodeEnd();
#if PARAM_EFM_CRITICAL_DISABLE_IRQ
    rt_base_t level;
#endif

    if ((buf == RT_NULL) ||
        ((addr % 4U) != 0U) ||
        ((len % 4U) != 0U)) {
        return PARAM_ERR_PARAM;
    }

    /* 防误写代码 */
    if ((code_end != 0U) && (addr < code_end)) {
        rt_kprintf("[PARAM] refuse write 0x%08X: overlaps code (code_end=0x%08X)\n",
                   (unsigned)addr, (unsigned)code_end);
        return PARAM_ERR_PARAM;
    }

#if PARAM_EFM_CRITICAL_DISABLE_IRQ
    level = rt_hw_interrupt_disable();
#endif

    efm_unlock();

    for (i = 0U; i < len; i += 4U) {
        uint32_t word;

        memcpy(&word, buf + i, 4U);
        ret = EFM_ProgramWordReadBack(addr + i, word);
        if (ret != LL_OK) {
            break;
        }
    }

    EFM_ClearStatus(EFM_FLAG_ALL);
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
