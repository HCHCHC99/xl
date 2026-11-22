/**
 * @file    storage.h
 * @brief   通用存储服务（不含任何业务数据）
 * @note    业务模块自建数据对象，通过本模块读写
 */

#ifndef __STORAGE_H
#define __STORAGE_H

#include <rtthread.h>
#include "flash_drv.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 存储服务对象
 * ============================================================ */
typedef struct storage_s {
    /* ----- 绑定关系 ----- */
    flash_t        *flash;
    const char     *name;
    void           *data_ptr;
    uint32_t        data_size;
    uint32_t        magic;

    /* ----- 状态 ----- */
    uint8_t         is_loaded;

    /* ----- 防反复写入 ----- */
    uint8_t        *last_data_buf;
    uint32_t        last_write_tick;
    uint32_t        write_cooldown_ms;
    uint32_t        write_count;
    uint32_t        skip_count;

    /* ----- 冷却期内的待写标记（防止真实变更被丢弃） ----- */
    uint8_t         pending_write;

    /* ----- 调试：已初始化实例链表 ----- */
    struct storage_s *next;
} storage_t;

/* ============================================================
 * API
 * ============================================================ */

rt_err_t storage_init(storage_t *storage,
                      flash_t *flash,
                      const char *name,
                      void *data_ptr,
                      uint32_t data_size,
                      uint32_t magic,
                      uint32_t cooldown_ms);

rt_err_t storage_load(storage_t *storage);
rt_err_t storage_save(storage_t *storage);
rt_err_t storage_compact(storage_t *storage);
uint8_t  storage_is_changed(storage_t *storage);

/**
 * @brief 查询分区是否为空（从未写入过有效数据）
 * @param storage 存储对象
 * @retval 1 空分区（Flash 中无任何有效块）
 * @retval 0 有数据
 * @note  应在 storage_load() 之后调用，仅作查询，不改变任何状态。
 *
 * 【为什么需要这个接口】（2026-11-03 / DES-STORAGE-FIX-V1.0）
 *   storage_load() 在空 Flash 时返回 RT_EOK，且【不填充 data_ptr】
 *   （保留调用方预先填好的默认值）——调用方无法通过返回值区分
 *   "加载成功" 和 "分区为空"。
 *   业务层需要前者来判断"是否要首次写入默认值"（甲-1 方案），
 *   故提供本显式查询接口，而不去改 storage_load 的返回值语义
 *   （改语义会影响现有 6 个调用方的错误处理分支）。
 */
uint8_t  storage_is_empty(storage_t *storage);

void storage_get_stats(storage_t *storage,
                       uint32_t *write_count,
                       uint32_t *skip_count);

/**
 * @brief 打印全部已初始化存储实例的统计信息 [线程安全-ish，仅调试]
 * @note  输出：名称、数据长度、实际写入次数、去重/冷却跳过次数、
 *        是否存在待写、距上次写入的时间
 */
void storage_dump_all(void);

#ifdef __cplusplus
}
#endif

#endif /* __STORAGE_H */
