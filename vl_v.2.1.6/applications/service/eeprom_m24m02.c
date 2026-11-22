/**
 * @file    eeprom_m24m02.c
 * @brief   M24M02 EEPROM 访问实现（基于软件 I2C）
 */

#include "eeprom_m24m02.h"
#include "../bsp/bsp_i2c_soft.h"
#include "../mode/sys_caps.h"       /* 能力位门禁（2026-10-27） */
#include "framework/state_engine/sm_assert.h"

#include <string.h>
#include <stdlib.h>
#include <rtthread.h>
#include <finsh.h>

/* ============================================================
 * 模块状态
 * ============================================================ */
static uint8_t s_ready = 0U;

/* 页写缓冲：2 字节地址 + 最多 256 字节数据 */
static uint8_t s_page_buf[M24M02_PAGE_SIZE + 2U];

/* ============================================================
 * 内部工具
 * ============================================================ */
static inline uint8_t addr_h(uint32_t a)
{
    /* M24M02：AddrH 的 bit7~3 保留为 0，bit2~0 = 地址 bit18~16 */
    return (uint8_t)((a >> 16) & 0x07U);
}

static inline uint8_t addr_l(uint32_t a)
{
    return (uint8_t)((a >> 8) & 0xFFU);
}

/**
 * @brief 等待内部写周期结束（ACK 轮询）
 * @return 0 就绪；-1 超时
 */
static int eeprom_wait_ready(void)
{
    rt_tick_t t0 = rt_tick_get();

    while (1) {
        if (bsp_i2c_soft_probe(M24M02_ADDR7) != 0U) {
            return 0;
        }
        if ((rt_tick_get() - t0) > rt_tick_from_millisecond(M24M02_WRITE_TIMEOUT_MS)) {
            rt_kprintf("[EEPROM] write timeout\n");
            return -1;
        }
        rt_thread_mdelay(1);
    }
}

/* ============================================================
 * 初始化
 * ============================================================ */
int eeprom_init(void)
{
    bsp_i2c_soft_init();

    if (bsp_i2c_soft_probe(M24M02_ADDR7) == 0U) {
        /* 器件缺失不阻断启动，仅关闭 EEPROM 相关功能 */
        rt_kprintf("[EEPROM] WARN: M24M02 not found (addr7=0x%02X), logging disabled\n",
                   (unsigned)M24M02_ADDR7);
        s_ready = 0U;
        return 0;
    }

    s_ready = 1U;
    rt_kprintf("[EEPROM] M24M02 ready, %u bytes, page %u\n",
               (unsigned)M24M02_SIZE, (unsigned)M24M02_PAGE_SIZE);
    return 0;
}

uint8_t eeprom_is_ready(void)
{
    return s_ready;
}

/* ============================================================
 * 写
 * ============================================================ */
int eeprom_write(uint32_t addr, const uint8_t *data, uint16_t len)
{
    uint32_t cur = addr;

    SM_ASSERT_PTR(data);

    if (s_ready == 0U) {
        return -1;
    }
    if ((addr >= M24M02_SIZE) || ((addr + len) > M24M02_SIZE)) {
        return -2;
    }

    while (len > 0U) {
        uint16_t page_left;     /* 本页剩余空间 */
        uint16_t chunk;

        page_left = (uint16_t)(M24M02_PAGE_SIZE - (cur % M24M02_PAGE_SIZE));
        chunk     = (len < page_left) ? len : page_left;

        /* 组帧：AddrH + AddrL + Data */
        s_page_buf[0] = addr_h(cur);
        s_page_buf[1] = addr_l(cur);
        memcpy(&s_page_buf[2], data, chunk);

        if (bsp_i2c_soft_write(M24M02_ADDR7, s_page_buf, (uint16_t)(chunk + 2U)) != 0) {
            return -3;
        }

        if (eeprom_wait_ready() != 0) {
            return -4;
        }

        cur  += chunk;
        data += chunk;
        len  -= chunk;
    }

    return 0;
}

/* ============================================================
 * 读
 * ============================================================ */
int eeprom_read(uint32_t addr, uint8_t *data, uint16_t len)
{
    uint8_t ptr[2];

    SM_ASSERT_PTR(data);

    if (s_ready == 0U) {
        return -1;
    }
    if ((addr >= M24M02_SIZE) || ((addr + len) > M24M02_SIZE)) {
        return -2;
    }
    if (len > M24M02_CHUNK_MAX) {
        return -5;
    }

    /* 1. 设置地址指针（只写地址，不写数据） */
    ptr[0] = addr_h(addr);
    ptr[1] = addr_l(addr);
    if (bsp_i2c_soft_write(M24M02_ADDR7, ptr, 2U) != 0) {
        return -3;
    }

    /* 2. 重新起始，读数据 */
    if (bsp_i2c_soft_read(M24M02_ADDR7, data, len) != 0) {
        return -4;
    }

    return 0;
}

/* ============================================================
 * 区域填充 0xFF（模拟擦除）
 * ============================================================ */
int eeprom_fill_ff(uint32_t addr, uint32_t len)
{
    uint8_t  buf[M24M02_PAGE_SIZE];
    uint32_t done = 0U;

    if (s_ready == 0U) {
        return -1;
    }
    if (((addr % M24M02_PAGE_SIZE) != 0U) || ((len % M24M02_PAGE_SIZE) != 0U)) {
        return -2;      /* 必须按页对齐 */
    }

    memset(buf, 0xFF, sizeof(buf));

    while (done < len) {
        if (eeprom_write(addr + done, buf, (uint16_t)M24M02_PAGE_SIZE) != 0) {
            return -3;
        }
        done += M24M02_PAGE_SIZE;
    }

    return 0;
}

/* ============================================================
 * 调试命令
 * ============================================================ */
static void t_eeprom(int argc, char **argv)
{
    if ((argc >= 2) && (strcmp(argv[1], "r") == 0) && (argc >= 4)) {
        uint32_t addr = (uint32_t)strtoul(argv[2], RT_NULL, 0);
        uint32_t n    = (uint32_t)strtoul(argv[3], RT_NULL, 0);
        uint8_t  buf[32];
        uint32_t i;

        /* 读取只暴露存储内容，用 DIAG_READ（工作模式也允许，
         * 便于现场取证） */
        if (sys_cap_require(SYS_CAP_DIAG_READ, "t_eeprom r") == 0U) { return; }

        if (n > sizeof(buf)) n = sizeof(buf);

        if (eeprom_read(addr, buf, (uint16_t)n) != 0) {
            rt_kprintf("read failed\n");
            return;
        }
        rt_kprintf("0x%06X:", (unsigned)addr);
        for (i = 0U; i < n; i++) {
            rt_kprintf(" %02X", buf[i]);
        }
        rt_kprintf("\n");
        return;
    }

    if ((argc >= 2) && (strcmp(argv[1], "w") == 0) && (argc >= 4)) {
        uint32_t addr = (uint32_t)strtoul(argv[2], RT_NULL, 0);
        uint8_t  buf[32];
        uint16_t n = 0U;
        int      i;

        /* 裸写 EEPROM 会覆盖校准值/机型表/系统参数，
         * 是最高危的调试口，必须 DIAG_WRITE。 */
        if (sys_cap_require(SYS_CAP_DIAG_WRITE, "t_eeprom w") == 0U) { return; }

        for (i = 3; (i < argc) && (n < sizeof(buf)); i++) {
            buf[n++] = (uint8_t)strtoul(argv[i], RT_NULL, 16);
        }
        if (eeprom_write(addr, buf, n) != 0) {
            rt_kprintf("write failed\n");
            return;
        }
        rt_kprintf("wrote %u bytes @ 0x%06X\n", (unsigned)n, (unsigned)addr);
        return;
    }

    /* 默认：探测 + 信息 */
    rt_kprintf("\n===== M24M02 EEPROM =====\n");
    rt_kprintf("device   : %s\n", s_ready ? "ready" : "NOT FOUND");
    rt_kprintf("i2c addr : 0x%02X (7-bit)\n", (unsigned)M24M02_ADDR7);
    rt_kprintf("size     : %u bytes (%u KB)\n",
               (unsigned)M24M02_SIZE, (unsigned)(M24M02_SIZE / 1024UL));
    rt_kprintf("page     : %u bytes\n", (unsigned)M24M02_PAGE_SIZE);
    rt_kprintf("usage    : t_eeprom r <addr> <len>\n");
    rt_kprintf("           t_eeprom w <addr> <hex...>\n");
    rt_kprintf("=========================\n\n");
}

MSH_CMD_EXPORT_ALIAS(t_eeprom, t_eeprom, M24M02 EEPROM read/write test);
