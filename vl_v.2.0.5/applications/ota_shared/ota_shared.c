/**
 * @file    ota_shared.c
 * @brief   OTA 共享参数业务实现
 */

#include "ota_shared.h"
#include "../storage/storage.h"
#include "../storage/storage_config.h"
#include "framework/state_engine/sm_assert.h"
#include <string.h>
#include <rtthread.h>

/* ============================================================
 * 全局数据对象
 * ============================================================ */
ota_shared_t g_ota_shared = {
    /* 【2026-11-03】非零记录魔数 —— 见 ota_shared.h 的说明。
     * 本结构其余字段默认值全为 0，若无此字段，
     * "空分区写默认值"会被 storage_save 的值去重静默跳过。 */
    .rec_magic = OTA_SHARED_REC_MAGIC,
    .flag = OTA_FLAG_IDLE,
    .target_version = 0,
    .crc32 = 0,
    .size = 0,
    .progress = 0,
    .status = OTA_STATUS_IDLE,
};

static storage_t g_ota_storage;
/* 本模块独立的 Flash 实例（禁止与其他模块共用） */
static flash_t   s_flash;

/* ============================================================
 * 初始化
 * ============================================================ */
int ota_shared_init(void)
{
    const storage_config_t *cfg = storage_find_config("ota_shared");
    rt_err_t ret;
    uint32_t blk_size;

    SM_ASSERT_PTR(cfg);

    blk_size = sizeof(ota_shared_t) + 8;

    ret = flash_init(&s_flash,
                     cfg->sector_addr,
                     cfg->sector_size,
                     blk_size,
                     cfg->magic);
    if (ret != RT_EOK) return ret;

    ret = storage_init(&g_ota_storage,
                       &s_flash,
                       cfg->name,
                       &g_ota_shared,
                       sizeof(ota_shared_t),
                       cfg->magic,
                       cfg->cooldown_ms);
    if (ret != RT_EOK) return ret;

    ret = storage_load(&g_ota_storage);

    /* ============================================================
     * 【甲-1 / DES-STORAGE-FIX-V1.0】异常 / 空分区 / 记录损坏处理
     * ------------------------------------------------------------
     * 默认值来自 g_ota_shared 静态初值：
     *   rec_magic = OTA_SHARED_REC_MAGIC（非零，关键）
     *   flag = OTA_FLAG_IDLE, status = OTA_STATUS_IDLE，其余全 0
     * 在 storage_load 之前就已就位。
     *
     * ⚠️ ota_shared 记录 OTA 升级过程状态。写入默认值（IDLE）
     *    等价于"没有进行中的升级"，这是安全的初始语义。
     *
     * 为什么要清 last_data_buf（storage.c:185 的值去重）：
     *   storage_load 空分区分支执行 memcpy(last_data_buf, data_ptr)，
     *   两者相同 -> save 会被静默跳过。
     *   清零后与默认值不同 —— 因为 rec_magic 非零，这正是加该字段的目的。
     *
     * 三种情况都归一到"回默认 + 落盘"：
     *   ① magic/CRC 异常（ret != RT_EOK）
     *   ② 空分区
     *   ③ rec_magic 不符（记录损坏，例如旧固件格式或位翻转）
     * ============================================================ */
    if (ret != RT_EOK) {
        rt_kprintf("[ota_shared] flash invalid (%d), using defaults\n", (int)ret);
        memset(g_ota_storage.last_data_buf, 0x00, sizeof(ota_shared_t));
        if (storage_save(&g_ota_storage) != RT_EOK) {
            rt_kprintf("[ota_shared] WARN: default save FAILED\n");
        }
    }
    else if (storage_is_empty(&g_ota_storage) != 0U) {
        rt_kprintf("[ota_shared] empty partition, writing defaults...\n");
        memset(g_ota_storage.last_data_buf, 0x00, sizeof(ota_shared_t));
        if (storage_save(&g_ota_storage) == RT_EOK) {
            rt_kprintf("[ota_shared] defaults written to flash\n");
        } else {
            rt_kprintf("[ota_shared] WARN: default save FAILED\n");
        }
    }
    else if (g_ota_shared.rec_magic != OTA_SHARED_REC_MAGIC) {
        /* ---- 记录损坏：CRC 过了但内容不是本模块写的（旧格式/位翻转）---- */
        rt_kprintf("[ota_shared] bad rec_magic 0x%08X (expect 0x%08X), reset to default\n",
                   (unsigned)g_ota_shared.rec_magic,
                   (unsigned)OTA_SHARED_REC_MAGIC);

        g_ota_shared.rec_magic      = OTA_SHARED_REC_MAGIC;
        g_ota_shared.flag           = OTA_FLAG_IDLE;
        g_ota_shared.target_version = 0U;
        g_ota_shared.crc32          = 0U;
        g_ota_shared.size           = 0U;
        g_ota_shared.progress       = 0U;
        g_ota_shared.status         = OTA_STATUS_IDLE;

        memset(g_ota_storage.last_data_buf, 0x00, sizeof(ota_shared_t));
        if (storage_save(&g_ota_storage) == RT_EOK) {
            rt_kprintf("[ota_shared] reset written to flash\n");
        } else {
            rt_kprintf("[ota_shared] WARN: reset save FAILED\n");
        }
    }

    rt_kprintf("[ota_shared] Loaded: rec_magic=0x%08X flag=0x%08X, status=%d\n\n",
               (unsigned)g_ota_shared.rec_magic,
               g_ota_shared.flag, g_ota_shared.status);

    return RT_EOK;
}

int ota_shared_save(void)
{
    return storage_save(&g_ota_storage);
}

int ota_shared_load(void)
{
    return storage_load(&g_ota_storage);
}

ota_shared_t* ota_shared_get(void)
{
    return &g_ota_shared;
}

/* ============================================================
 * 便捷接口
 * ============================================================ */
int ota_shared_set_upgrade(uint32_t target_version, uint32_t crc32, uint32_t size)
{
    g_ota_shared.flag = OTA_FLAG_READY;
    g_ota_shared.target_version = target_version;
    g_ota_shared.crc32 = crc32;
    g_ota_shared.size = size;
    g_ota_shared.progress = 0;
    g_ota_shared.status = OTA_STATUS_READY;
    return ota_shared_save();
}

int ota_shared_set_status(ota_status_t status)
{
    g_ota_shared.status = status;
    return ota_shared_save();
}

int ota_shared_clear(void)
{
    g_ota_shared.flag = OTA_FLAG_IDLE;
    g_ota_shared.target_version = 0;
    g_ota_shared.crc32 = 0;
    g_ota_shared.size = 0;
    g_ota_shared.progress = 0;
    g_ota_shared.status = OTA_STATUS_IDLE;
    return ota_shared_save();
}
