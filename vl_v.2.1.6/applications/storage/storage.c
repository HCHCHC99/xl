/**
 * @file    storage.c
 * @brief   通用存储服务实现
 */

#include "storage.h"

/* 【2026-11-10 彩标 P3】存储层：读/写失败=ERR，其余为 INFO。
 * t_storage 的表格输出保持单色（Q4 裁定）。*/
#include "../log/log_color.h"
#include "framework/state_engine/sm_assert.h"
#include <string.h>
#include <rtthread.h>
#include <finsh.h>

/* 【2026-11-03 Q9 方案 A】块上限 1024 -> 4096
 *
 * 原值 1024 本身没错，但真正卡住的是 flash_drv.c 的 256 栈数组断言。
 * 该断言已在同批改动中放开到 1024（并改为 rt_malloc）。
 * 这里同步放宽到 4096，给 sys_config 配置表留出充足余量：
 *   sys_config 当前 37 项 x 8B + 头 = 304B，未来涨到数百项也不会撞墙。
 *
 * 注意：上限只是"允许到多大"，实际块大小由各分区按需指定。
 *       现有 5 个分区实际值均 <= 244B，行为零变化。 */
#define STORAGE_BLK_MAX_SIZE    4096

/* 每次真正落盘时打印一行（默认关闭；调试 Flash 寿命/写入频次时置 1） */
#define STORAGE_DEBUG_WRITE     0

/* ============================================================
 * 已初始化实例链表（仅用于调试统计，不参与业务逻辑）
 * ============================================================ */
static storage_t *s_storage_list = RT_NULL;

/* ============================================================
 * CRC32
 * ============================================================ */
static uint32_t calc_crc32(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFF;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) crc = (crc >> 1) ^ 0xEDB88320;
            else crc >>= 1;
        }
    }
    return ~crc;
}

static uint32_t calc_crc32_update(uint32_t crc, const uint8_t *data, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) crc = (crc >> 1) ^ 0xEDB88320;
            else crc >>= 1;
        }
    }
    return crc;
}

/* ============================================================
 * 初始化
 * ============================================================ */
rt_err_t storage_init(storage_t *storage,
                      flash_t *flash,
                      const char *name,
                      void *data_ptr,
                      uint32_t data_size,
                      uint32_t magic,
                      uint32_t cooldown_ms)
{
    SM_ASSERT_PTR(storage);
    SM_ASSERT_PTR(flash);
    SM_ASSERT_PTR(name);
    SM_ASSERT_PTR(data_ptr);
    SM_ASSERT_MSG(data_size > 0, "data_size must > 0");
    SM_ASSERT_MSG(data_size <= STORAGE_BLK_MAX_SIZE, "data_size too large");

    storage->flash = flash;
    storage->name = name;
    storage->data_ptr = data_ptr;
    storage->data_size = data_size;
    storage->magic = magic;
    storage->write_cooldown_ms = cooldown_ms;
    storage->is_loaded = 0;

    storage->last_data_buf = rt_malloc(data_size);
    SM_ASSERT_PTR(storage->last_data_buf);
    memset(storage->last_data_buf, 0, data_size);

    /* 初始时间基准回拨一个冷却周期，保证首次 save 不被冷却拦截 */
    storage->last_write_tick = rt_tick_get() - rt_tick_from_millisecond(cooldown_ms);
    storage->write_count   = 0;
    storage->skip_count    = 0;
    storage->pending_write = 0U;

    /* 挂入调试链表 */
    storage->next  = s_storage_list;
    s_storage_list = storage;

    return RT_EOK;
}

/* ============================================================
 * 加载
 * ============================================================ */
rt_err_t storage_load(storage_t *storage)
{
    uint8_t *read_buf;
    uint32_t last_addr, read_len;
    uint32_t magic, crc_stored, crc_calc;

    SM_ASSERT_PTR(storage);
    SM_ASSERT_PTR(storage->flash);
    SM_ASSERT_PTR(storage->data_ptr);
    SM_ASSERT_MSG(storage->data_size <= STORAGE_BLK_MAX_SIZE, "data_size too large");

    last_addr = flash_get_last_addr(storage->flash);

    if (last_addr == 0) {
        /* Flash 无有效记录：保留调用方当前值作为默认值 */
        storage->is_loaded = 1;
        memcpy(storage->last_data_buf, storage->data_ptr, storage->data_size);
        return RT_EOK;
    }

    read_len = 4 + 4 + storage->data_size;

    /* ★ 堆分配，不再占用栈：本函数可能在 can_parser(2048B) 等小栈线程中调用，
     *   原实现在栈上开 1024 字节缓冲会直接溢出并破坏相邻内存。 */
    read_buf = (uint8_t *)rt_malloc(read_len);
    if (read_buf == RT_NULL) {
        LOG_ERR("[STORAGE] %s load: alloc %u failed\n",
                   storage->name, (unsigned)read_len);
        return -RT_ENOMEM;
    }

    storage->flash->ops->read(storage->flash, last_addr, read_buf, read_len);

    memcpy(&magic, read_buf, 4);
    if (magic != storage->magic) {
        /* 不致命：返回错误由调用方决定是否采用默认值，禁止断言杀线程 */
        LOG_ERR("[STORAGE] %s magic mismatch @0x%08X: expect 0x%08X got 0x%08X\n",
                   storage->name, (unsigned)last_addr,
                   (unsigned)storage->magic, (unsigned)magic);
        rt_free(read_buf);
        return -RT_ERROR;
    }

    memcpy(&crc_stored, read_buf + 4, 4);
    crc_calc = calc_crc32(read_buf, 4);
    crc_calc = calc_crc32_update(crc_calc, read_buf + 8, storage->data_size);
    if (crc_calc != crc_stored) {
        LOG_ERR("[STORAGE] %s crc mismatch: calc 0x%08X stored 0x%08X\n",
                   storage->name, (unsigned)crc_calc, (unsigned)crc_stored);
        rt_free(read_buf);
        return -RT_ERROR;
    }

    memcpy(storage->data_ptr, read_buf + 8, storage->data_size);
    memcpy(storage->last_data_buf, storage->data_ptr, storage->data_size);
    storage->is_loaded = 1;

    rt_free(read_buf);
    return RT_EOK;
}

/* ============================================================
 * 保存
 * ============================================================ */
rt_err_t storage_save(storage_t *storage)
{
    uint8_t *write_buf;
    uint32_t crc, write_len;
    uint32_t now, elapsed_ms;
    rt_err_t ret;

    SM_ASSERT_PTR(storage);
    SM_ASSERT_PTR(storage->flash);
    SM_ASSERT_PTR(storage->data_ptr);
    SM_ASSERT_MSG(storage->is_loaded, "Storage not loaded");
    SM_ASSERT_MSG(storage->data_size <= STORAGE_BLK_MAX_SIZE, "data_size too large");

    /* ----- 1. 值去重（优先判定，不占用冷却窗口） -----
     * 内容与上次落盘完全一致时直接返回，绝不擦写 Flash。
     * 这是防止周期重发导致 Flash 写穿的第一道防线。 */
    if (rt_memcmp(storage->data_ptr, storage->last_data_buf, storage->data_size) == 0) {
        storage->skip_count++;
        storage->pending_write = 0U;     /* 与已落盘内容一致，撤销待写标记 */
        return RT_EOK;
    }

    /* ----- 2. 冷却检查 -----
     * 冷却期内发生真实变更时置 pending_write，下笔写放行，
     * 保证变更最终一定落盘（原实现会把冷却期内的变更丢弃）。
     *
     * ⚠️【F3 / 2026-11-15 Q-P3 核查发现，本批只记录不自行修（R-01）】
     * 若该分支被「掉电紧急保存」调用，且 pending_write==0 又落在冷却期内
     * ⇒ 只置 pending 并 return RT_EOK，本次**不落盘**；掉电窗口只有一次调用机会
     * ⇒ 数据可能丢失。修法需先定 F4 掉电检测手段，再单独成批。
     * 【TEMP-P1】3.3V 维持 >=50ms、【TEMP-P2】单次 append <=50ms 均为待验证前提
     * （设计基线 V3.6 §7.1 / 待裁定基线 §3.1）。 */
    if ((storage->pending_write == 0U) && (storage->write_cooldown_ms > 0U)) {
        now = rt_tick_get();
        elapsed_ms = (now - storage->last_write_tick) * 1000 / RT_TICK_PER_SECOND;
        if (elapsed_ms < storage->write_cooldown_ms) {
            storage->pending_write = 1U;
            storage->skip_count++;
            return RT_EOK;
        }
    }

    write_len = 4 + 4 + storage->data_size;

    /* ★ 堆分配，不再占用栈（同 storage_load 说明） */
    write_buf = (uint8_t *)rt_malloc(write_len);
    if (write_buf == RT_NULL) {
        LOG_ERR("[STORAGE] %s save: alloc %u failed\n",
                   storage->name, (unsigned)write_len);
        return -RT_ENOMEM;
    }

    /* 构造写入数据：Magic(4) + CRC(4) + Data */
    memcpy(write_buf, &storage->magic, 4);
    memset(write_buf + 4, 0, 4);

    crc = calc_crc32(write_buf, 4);
    crc = calc_crc32_update(crc, (const uint8_t *)storage->data_ptr, storage->data_size);
    memcpy(write_buf + 4, &crc, 4);

    memcpy(write_buf + 8, storage->data_ptr, storage->data_size);

    /* 追加写入 */
    ret = flash_append_write(storage->flash, write_buf, write_len);
    rt_free(write_buf);

    if (ret != RT_EOK) {
        LOG_ERR("[STORAGE] %s append_write failed (%d)\n", storage->name, ret);
        return ret;
    }

    /* 更新状态 */
    storage->last_write_tick = rt_tick_get();
    storage->write_count++;
    storage->pending_write = 0U;
    memcpy(storage->last_data_buf, storage->data_ptr, storage->data_size);

#if STORAGE_DEBUG_WRITE
    rt_kprintf("[STORAGE] %s write #%u (%u bytes)\n",
               storage->name, (unsigned)storage->write_count,
               (unsigned)storage->data_size);
#endif

    return RT_EOK;
}

/* ============================================================
 * 压缩整理
 * ============================================================ */
rt_err_t storage_compact(storage_t *storage)
{
    SM_ASSERT_PTR(storage);
    SM_ASSERT_PTR(storage->flash);
    return flash_compact(storage->flash, storage->data_size + 8);
}

/* ============================================================
 * 查询
 * ============================================================ */
uint8_t storage_is_changed(storage_t *storage)
{
    SM_ASSERT_PTR(storage);
    SM_ASSERT_PTR(storage->data_ptr);

    if (!storage->is_loaded) return 1;

    return (rt_memcmp(storage->data_ptr, storage->last_data_buf, storage->data_size) != 0);
}

/* ============================================================
 * 【2026-11-03 新增 / DES-STORAGE-FIX-V1.0】空分区查询
 * ------------------------------------------------------------
 * 直接问物理层"有没有有效块"，而不是靠 data_ptr 内容猜。
 * is_loaded 在两种情况下都是 1（有数据 / 空分区），不能用它判断。
 * ============================================================ */
uint8_t storage_is_empty(storage_t *storage)
{
    SM_ASSERT_PTR(storage);
    SM_ASSERT_PTR(storage->flash);

    return (flash_get_last_addr(storage->flash) == 0) ? 1U : 0U;
}

void storage_get_stats(storage_t *storage,
                       uint32_t *write_count,
                       uint32_t *skip_count)
{
    SM_ASSERT_PTR(storage);
    if (write_count) *write_count = storage->write_count;
    if (skip_count) *skip_count = storage->skip_count;
}

/* ============================================================
 * 调试：全部实例统计
 * ============================================================ */
void storage_dump_all(void)
{
    storage_t *p;
    uint32_t now = rt_tick_get();
    uint32_t since_ms;

    rt_kprintf("\n========== STORAGE STATS ==========\n");
    rt_kprintf("%-12s %6s %8s %8s %5s %10s\n",
               "name", "size", "write", "skip", "pend", "since_ms");
    rt_kprintf("-----------------------------------\n");

    for (p = s_storage_list; p != RT_NULL; p = p->next) {
        since_ms = (now - p->last_write_tick) * 1000 / RT_TICK_PER_SECOND;
        rt_kprintf("%-12s %6u %8u %8u %5u %10u\n",
                   (p->name != RT_NULL) ? p->name : "?",
                   (unsigned)p->data_size,
                   (unsigned)p->write_count,
                   (unsigned)p->skip_count,
                   (unsigned)p->pending_write,
                   (unsigned)since_ms);
    }
    rt_kprintf("===================================\n\n");
}
MSH_CMD_EXPORT_ALIAS(storage_dump_all, t_storage, dump storage write statistics);
