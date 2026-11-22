/**
 * @file    flash_drv.c
 * @brief   Flash 物理层驱动实现（内部 Flash）
 * @note    支持多实例，追加写入策略
 */

#include "flash_drv.h"
#include "framework/state_engine/sm_assert.h"
#include "storage_config.h"
#include <string.h>
#include <rthw.h>
/* 【2026-11-03】rt_malloc/rt_free：blk_size 放开后栈缓冲改堆分配，需 rtthread.h */
#include <rtthread.h>

/* ============================================================
 * 硬件层头文件（根据 MCU 调整）
 * ============================================================ */
#include "hc32_ll.h"
#include "hc32_ll_efm.h"

/* ============================================================
 * Q-P3 / P-a（用户 2026-11-15 20:08 裁定）
 * ------------------------------------------------------------
 * 上电时若「剩余可写次数 < 3」，先擦除该扇区并把历史中最新数据写回，
 * 保证运行期有 >= 3 次纯追加写空间，掉电保存不必再走 compact + erase。
 *
 * [TEMP-P1] 外部电源断电后 3.3V 维持 >= 50ms —— 按「满足要求」执行，待硬件实测验证。
 * [TEMP-P2] 单次 append 写入耗时 <= 50ms   —— 按「可以满足」执行，待上机实测验证。
 * 两项均为临时假设（设计基线 V3.6 §7.1）；不达标须回 `待裁定基线 §3.1` 重新裁定。
 *
 * 已知边界（F2 / F3 / F4，本批按 R-01 只记录、不自行修）：
 *   F2 运行期写满仍会自动 compact；F3 storage.c 冷却期可能吞掉紧急保存；
 *   F4 全工程尚无掉电检测实现（bsp_mod_power.c 仍为死代码）。
 * ============================================================ */
#define FLASH_MIN_FREE_WRITES   3U

/* ============================================================
 * 1. 硬件操作函数（内部 Flash）
 * ============================================================ */
static void flash_hw_unlock(flash_t *self)
{
    self->priv.irq_level = rt_hw_interrupt_disable();
    LL_PERIPH_WE(LL_PERIPH_EFM);
    while (SET != EFM_GetStatus(EFM_FLAG_RDY));
    EFM_FWMC_Cmd(ENABLE);
}

static void flash_hw_lock(flash_t *self)
{
    EFM_FWMC_Cmd(DISABLE);
    LL_PERIPH_WP(LL_PERIPH_EFM);
    rt_hw_interrupt_enable(self->priv.irq_level);
}

static rt_err_t flash_hw_erase_sector(flash_t *self)
{
    int32_t ret;

    flash_hw_unlock(self);
    EFM_ClearStatus(EFM_FLAG_ALL);
    ret = EFM_SectorErase(self->priv.sector_addr);
    flash_hw_lock(self);

    if (ret != LL_OK) {
        /* 地址越界或 Flash 故障：必须暴露出来，否则上层以为擦除成功 */
        rt_kprintf("[FLASH] %s erase FAILED @0x%08X (ret=%d)\n",
                   self->priv.name, (unsigned)self->priv.sector_addr, (int)ret);
        return -RT_ERROR;
    }
    return RT_EOK;
}

/**
 * @brief 写入一个 32 位字并回读校验
 * @note  EFM_ProgramWordReadBack 内部已做 readback 比对，失败返回非 LL_OK。
 *        原实现丢弃该返回值，导致地址越界也"写成功"，问题被完全掩盖。
 */
static rt_err_t flash_hw_write_word(uint32_t addr, uint32_t data)
{
    int32_t ret = EFM_ProgramWordReadBack(addr, data);
    if (ret != LL_OK) {
        rt_kprintf("[FLASH] program FAILED @0x%08X (ret=%d)\n",
                   (unsigned)addr, (int)ret);
        return -RT_ERROR;
    }
    return RT_EOK;
}

/* ============================================================
 * 2. 内部辅助函数：查找最后有效块
 * ============================================================ */
static uint32_t flash_find_last_valid(flash_t *self, uint32_t blk_size)
{
    uint32_t addr = self->priv.sector_addr;
    uint32_t last = 0;
    uint32_t magic;

    while (addr < self->priv.sector_addr + self->priv.sector_size) {
        self->ops->read(self, addr, (uint8_t*)&magic, 4);
        if (magic != self->priv.magic) {
            break;
        }
        last = addr;
        addr += blk_size;
    }
    return last;
}

/* ============================================================
 * 3. 操作函数实现
 * ============================================================ */
static rt_err_t flash_ops_init(flash_t *self)
{
    uint32_t used, total;
    uint32_t blk_size = self->priv.blk_size;
    uint8_t *last_data;

    SM_ASSERT_PTR(self);
    SM_ASSERT_MSG(blk_size > 0, "blk_size must > 0");

    /* ============================================================
     * 【2026-11-03 Q9 方案 A】blk_size 上限 256 -> 1024
     * ------------------------------------------------------------
     * 原来的约束是两处：
     *   uint8_t last_data[256]                  栈数组，硬上限
     *   SM_ASSERT_MSG(blk_size <= 256, ...)     配套断言
     *
     * 为什么必须放开：
     *   sys_config 配置表按 8 字节/项设计，37 项就是 296B，
     *   加 8B 头 = 304B。256 上限装不下（248/8 = 31 项封顶）。
     *   硬压进 256 只能砍功能（不能存 U32 时间戳/纳伏精度）。
     *
     * 为什么 last_data 改成 rt_malloc：
     *   直接改成 uint8_t last_data[1024] 会占 1024B 栈。
     *   flash_ops_init 在 boot 线程调用，main 栈 1024B、峰值已 704B(68.8%)，
     *   再加 1024B 必然爆栈。堆分配规避该风险。
     *   释放路径覆盖所有 return（goto 收口），无泄漏。
     *
     * 向后兼容：
     *   现有 5 个分区 blk_size 均 <= 244，远小于新上限，
     *   行为与改动前完全一致，不产生任何回归。
     * ============================================================ */
    SM_ASSERT_MSG(blk_size <= FLASH_BLK_SIZE_MAX, "blk_size too large");

    /* 查找最后有效块 */
    self->priv.last_valid_addr = flash_find_last_valid(self, blk_size);

    if (self->priv.last_valid_addr == 0) {
        /* 无有效数据，初始化扇区 */
        if (flash_hw_erase_sector(self) != RT_EOK) {
            return -RT_ERROR;
        }
        /* ============================================================
         * 【2026-11-03 修复 DES-STORAGE-FIX-V1.0】从块0 起写，不跳块
         * ------------------------------------------------------------
         * 原代码：
         *     next_write_addr = sector_addr + blk_size;   // 跳到块1
         * 注释虽写"写入一个默认块（只写入 Magic）"，但实际从未写入任何数据。
         *
         * 后果链（实测复现）：
         *   块0 永远是 0xFF
         *   -> flash_find_last_valid 从块0 扫描，首个 magic 不匹配即 break
         *   -> 永远返回 0（"空扇区"）
         *   -> 每次上电都判空并擦除
         *   -> 刚写进块1 的数据被清掉
         *   => 全工程 6 个分区"写得进、读不出"，所有配置跨重启全部丢失。
         *
         * 修法：从块0 起写。
         *   块0 是【世代锚点】：块0 有效才说明该扇区有可信数据。
         *   这样"块0 有效"成为可依赖的不变式；
         *   历史数据（在块1起、块0 为空，如旧固件遗留）会被正确判为
         *   不可信并清除 —— 这是期望行为，不是数据丢失。
         *
         * 为什么不改成"全扫描取最后匹配"：
         *   块1..N 里可能混着不同世代（旧格式 / 上一轮 compact 前的残留），
         *   全扫描会把"物理最后一块"误当成"最新数据"，取到最旧的残留。
         *   故 flash_find_last_valid 保持"遇不匹配即停"，本函数只负责
         *   让第一次写入落在块0。
         *
         * 默认值由业务层（storage 之上）检测到空分区后主动写入，
         * 物理层不感知业务语义（甲-1 方案）。 */
        self->priv.last_valid_addr = 0;
        self->priv.next_write_addr = self->priv.sector_addr;
        return RT_EOK;
    }

    /* 计算已用块数 */
    total = self->priv.sector_size / blk_size;
    used = (self->priv.last_valid_addr - self->priv.sector_addr) / blk_size + 1;

    /* 【P-a / 2026-11-15】原判据 used >= total（扇区写满才整理）。
     * 现改为「剩余可写次数 < FLASH_MIN_FREE_WRITES(3) 即整理」：
     *   used + 3 > total  <=>  total - used < 3  <=>  剩余可写次数 < 3
     * 用加法而非减法，避免 uint32 下溢（used <= total 由上式保证）。
     * 效果：上电后至少留出 3 次纯追加写空间，擦除耗时从掉电路径移到启动路径。 */
    if (used + FLASH_MIN_FREE_WRITES > total) {
        rt_err_t ret = RT_EOK;

        /* 剩余空间不足 3 次，执行压缩整理：保留最后有效块 */
        last_data = rt_malloc(blk_size);
        if (last_data == RT_NULL) {
            rt_kprintf("[FLASH] %s compact: malloc(%u) failed\n",
                       self->priv.name, (unsigned)blk_size);
            return -RT_ENOMEM;
        }

        self->ops->read(self, self->priv.last_valid_addr, last_data, blk_size);

        if (flash_hw_erase_sector(self) != RT_EOK) {
            ret = -RT_ERROR;
        } else if (self->ops->write(self, self->priv.sector_addr,
                                    last_data, blk_size) != RT_EOK) {
            ret = -RT_ERROR;
        }

        rt_free(last_data);

        if (ret != RT_EOK) {
            return ret;
        }

        self->priv.last_valid_addr = self->priv.sector_addr;
        self->priv.next_write_addr = self->priv.sector_addr + blk_size;
    } else {
        self->priv.next_write_addr = self->priv.last_valid_addr + blk_size;
    }

    return RT_EOK;
}

static rt_err_t flash_ops_write(flash_t *self, uint32_t addr, const uint8_t *data, uint32_t len)
{
    uint32_t aligned_len;
    rt_err_t ret = RT_EOK;

    SM_ASSERT_PTR(self);
    SM_ASSERT_PTR(data);
    SM_ASSERT_MSG(len > 0, "len must > 0");

    flash_hw_unlock(self);

    /* 按 4 字节对齐写入 */
    aligned_len = (len + 3) & ~3;
    for (uint32_t i = 0; i < aligned_len; i += 4) {
        uint32_t val = 0xFFFFFFFF;
        if (i < len) {
            memcpy(&val, data + i, (len - i) >= 4 ? 4 : (len - i));
        }
        if (flash_hw_write_word(addr + i, val) != RT_EOK) {
            ret = -RT_ERROR;
            break;
        }
    }

    flash_hw_lock(self);
    return ret;
}

static void flash_ops_read(flash_t *self, uint32_t addr, uint8_t *data, uint32_t len)
{
    SM_ASSERT_PTR(self);
    SM_ASSERT_PTR(data);
    SM_ASSERT_MSG(len > 0, "len must > 0");

    EFM_ReadByte(addr, data, len);
}

static rt_err_t flash_ops_append_write(flash_t *self, const uint8_t *data, uint32_t len)
{
    uint32_t next_addr;
    uint32_t sector_end;
    uint32_t blk_size;

    SM_ASSERT_PTR(self);
    SM_ASSERT_PTR(data);
    SM_ASSERT_MSG(len > 0, "len must > 0");

    blk_size = self->priv.blk_size;
    next_addr = self->priv.next_write_addr;
    sector_end = self->priv.sector_addr + self->priv.sector_size;

    /* 检查剩余空间 */
    if (next_addr + blk_size > sector_end) {
        /* 【F2 已知边界 / 2026-11-15】上电已按 P-a 预留 >= 3 次写入空间，
         * 正常周期内不会走到这里；若走到，说明本周期保存超过 3 次，
         * 本次仍会触发 compact + 扇区擦除（掉电保存不再是纯 append）。
         * 本批按 R-01 只记录、不自行修，登记在 `待裁定基线 §3.1`。
         *
         * 空间不足：先压缩整理，压缩后必须重新取地址再写，
         * 否则本次数据会丢失（原实现压缩后直接返回）。 */
        rt_err_t ret = flash_compact(self, blk_size);
        if (ret != RT_EOK) {
            return ret;
        }

        next_addr = self->priv.next_write_addr;
        if (next_addr + blk_size > sector_end) {
            /* 压缩后仍无空间，属异常配置 */
            return -RT_ERROR;
        }
    }

    /* 写入数据 */
    if (self->ops->write(self, next_addr, data, len) != RT_EOK) {
        /* 失败时不推进地址，避免后续块错位覆盖 */
        return -RT_ERROR;
    }
    self->priv.last_valid_addr = next_addr;
    self->priv.next_write_addr = next_addr + blk_size;

    return RT_EOK;
}

static rt_err_t flash_ops_compact(flash_t *self, uint32_t blk_size)
{
    uint8_t *last_data;
    rt_err_t ret = RT_EOK;

    SM_ASSERT_PTR(self);
    SM_ASSERT_MSG(blk_size > 0, "blk_size must > 0");
    /* 【2026-11-03 Q9 方案 A】与 flash_ops_init 同批放开，见该函数注释 */
    SM_ASSERT_MSG(blk_size <= FLASH_BLK_SIZE_MAX, "blk_size too large");

    /* 读取最后有效块 */
    if (self->priv.last_valid_addr == 0) {
        /* 无有效数据，直接擦除 */
        if (flash_hw_erase_sector(self) != RT_EOK) {
            return -RT_ERROR;
        }
        self->priv.last_valid_addr = 0;
        self->priv.next_write_addr = self->priv.sector_addr + blk_size;
        return RT_EOK;
    }

    /* 栈缓冲改为堆分配：理由同 flash_ops_init，避免大 blk_size 爆栈 */
    last_data = rt_malloc(blk_size);
    if (last_data == RT_NULL) {
        rt_kprintf("[FLASH] %s compact: malloc(%u) failed\n",
                   self->priv.name, (unsigned)blk_size);
        return -RT_ENOMEM;
    }

    self->ops->read(self, self->priv.last_valid_addr, last_data, blk_size);

    /* 擦除扇区 */
    if (flash_hw_erase_sector(self) != RT_EOK) {
        ret = -RT_ERROR;
    }
    /* 写入最后有效块到扇区起始位置 */
    else if (self->ops->write(self, self->priv.sector_addr,
                              last_data, blk_size) != RT_EOK) {
        ret = -RT_ERROR;
    }
    else {
        self->priv.last_valid_addr = self->priv.sector_addr;
        self->priv.next_write_addr = self->priv.sector_addr + blk_size;
    }

    rt_free(last_data);
    return ret;
}

static uint32_t flash_ops_get_last_addr(flash_t *self)
{
    SM_ASSERT_PTR(self);
    return self->priv.last_valid_addr;
}

static uint32_t flash_ops_get_next_addr(flash_t *self)
{
    SM_ASSERT_PTR(self);
    return self->priv.next_write_addr;
}

/* ============================================================
 * 4. 操作函数表实例
 * ============================================================ */
static flash_ops_t g_flash_internal_ops = {
    .init = flash_ops_init,
    .write = flash_ops_write,
    .read = flash_ops_read,
    .append_write = flash_ops_append_write,
    .compact = flash_ops_compact,
    .get_last_addr = flash_ops_get_last_addr,
    .get_next_addr = flash_ops_get_next_addr,
};

/* ============================================================
 * 5. 内部 Flash 实例
 * ============================================================ */
flash_t g_flash_internal = {
    .priv = {
        .name = "internal",
        .type = FLASH_TYPE_INTERNAL,
        .sector_addr = 0,
        .sector_size = 0,
        .magic = 0,
        .last_valid_addr = 0,
        .next_write_addr = 0,
        .blk_size = 0,
    },
    .ops = &g_flash_internal_ops,
};

/* ============================================================
 * 6. 公开 API
 * ============================================================ */
rt_err_t flash_init(flash_t *self,
                    uint32_t sector_addr,
                    uint32_t sector_size,
                    uint32_t blk_size,
                    uint32_t magic)
{
    SM_ASSERT_PTR(self);
    SM_ASSERT_MSG(sector_size > 0, "sector_size must > 0");
    SM_ASSERT_MSG(blk_size > 0, "blk_size must > 0");
    /* 【2026-11-03 Q9 方案 A】256 -> 1024，与 flash_ops_init/compact 保持一致。
     * 三处必须同值，否则 flash_init 会先在这里断言失败。 */
    SM_ASSERT_MSG(blk_size <= FLASH_BLK_SIZE_MAX, "blk_size too large");

    /* 内部 Flash 驱动固定：显式赋值后，任意 flash_t 实例都能独立初始化。
     * 原实现依赖 g_flash_internal 的静态初始化给 ops，
     * 导致 4 个模块共用一个实例、扇区地址互相覆盖。 */
    self->ops = &g_flash_internal_ops;

    self->priv.sector_addr = sector_addr;
    self->priv.sector_size = sector_size;
    self->priv.blk_size = blk_size;
    self->priv.magic = magic;

    return self->ops->init(self);
}

rt_err_t flash_append_write(flash_t *self, const uint8_t *data, uint32_t len)
{
    SM_ASSERT_PTR(self);
    SM_ASSERT_PTR(data);
    return self->ops->append_write(self, data, len);
}

rt_err_t flash_compact(flash_t *self, uint32_t blk_size)
{
    SM_ASSERT_PTR(self);
    SM_ASSERT_MSG(blk_size > 0, "blk_size must > 0");
    return self->ops->compact(self, blk_size);
}

uint32_t flash_get_last_addr(flash_t *self)
{
    SM_ASSERT_PTR(self);
    return self->ops->get_last_addr(self);
}

uint32_t flash_get_next_addr(flash_t *self)
{
    SM_ASSERT_PTR(self);
    return self->ops->get_next_addr(self);
}
