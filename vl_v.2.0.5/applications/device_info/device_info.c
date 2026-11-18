/**
 * @file    device_info.c
 * @brief   设备信息业务实现
 */

#include "../device_info/device_info.h"
#include "../storage/storage.h"
#include "../storage/storage_config.h"
#include "framework/state_engine/sm_assert.h"

/* 【2026-11-10 彩标 P3】设备信息：失败=ERR，降级=WARN，成功=OK。
 * 上电打印的 Device Info 表格保持单色。*/
#include "../log/log_color.h"
#include <string.h>
#include <rtthread.h>

/* ============================================================
 * 全局数据对象
 * ============================================================ */
device_info_t g_device_info = {
    .sn = "JC_202610080001",
    .name = "JC_BLDC_Controller",
    .model = 1,
    .hw_version = 0x00000000,
    .sw_version = 0x00000000,
    .sw_version_str = "v0.0.0",
    .manufacture_date = 20261008,
    .first_boot_timestamp = 0,
    /* 【2026-11-03】非零记录魔数 —— 见 device_info.h 的说明。
     * 加在结构体末尾，不破坏 g_device_info_fields[] 的 offsetof 偏移。 */
    .rec_magic = DEVICE_INFO_REC_MAGIC,
};

/* ============================================================
 * 字段表单
 * ============================================================ */
const device_field_t g_device_info_fields[] = {
    {"SN",          offsetof(device_info_t, sn),                  DEVICE_SN_LEN,      FIELD_TYPE_STRING, 0},
    {"Name",        offsetof(device_info_t, name),                DEVICE_NAME_LEN,    FIELD_TYPE_STRING, 1},
    {"Model",       offsetof(device_info_t, model),               4,                  FIELD_TYPE_U32,    1},
    {"HwVersion",   offsetof(device_info_t, hw_version),          4,                  FIELD_TYPE_U32,    0},
    {"SwVersion",   offsetof(device_info_t, sw_version),          4,                  FIELD_TYPE_U32,    1},
    {"SwVerStr",    offsetof(device_info_t, sw_version_str),      4,                  FIELD_TYPE_STRING, 1},
    {"Manufacture", offsetof(device_info_t, manufacture_date),    4,                  FIELD_TYPE_U32,    0},
    {"FirstBoot",   offsetof(device_info_t, first_boot_timestamp),4,                  FIELD_TYPE_U32,    0},
};

const uint32_t g_device_info_field_count = sizeof(g_device_info_fields) / sizeof(g_device_info_fields[0]);

/* 内部存储服务实例 */
static storage_t g_dev_storage;
/* 本模块独立的 Flash 实例（禁止与其他模块共用） */
static flash_t   s_flash;

/* ============================================================
 * 初始化
 * ============================================================ */
int device_info_init(void)
{
    const storage_config_t *cfg = storage_find_config("device_info");
    rt_err_t ret;
    uint32_t blk_size;

    SM_ASSERT_PTR(cfg);

    blk_size = sizeof(device_info_t) + 8;

    ret = flash_init(&s_flash,
                     cfg->sector_addr,
                     cfg->sector_size,
                     blk_size,
                     cfg->magic);
    if (ret != RT_EOK) return ret;

    ret = storage_init(&g_dev_storage,
                       &s_flash,
                       cfg->name,
                       &g_device_info,
                       sizeof(device_info_t),
                       cfg->magic,
                       cfg->cooldown_ms);
    if (ret != RT_EOK) return ret;

    ret = storage_load(&g_dev_storage);

    /* ============================================================
     * 【甲-1 / DES-STORAGE-FIX-V1.0】异常 / 空分区处理
     * ------------------------------------------------------------
     * device_info 是【只读资产】（SN/型号/厂商日期），默认值来自
     * g_device_info 的静态初值（device_info.c:16-25），在 storage_load
     * 之前就已就位。
     *
     * 两种需要处理的情况：
     *   ① magic/CRC 异常（ret != RT_EOK）：回默认并重写
     *   ② 空分区（ret == RT_EOK 但无有效块）：写默认值
     *   storage_load 对空分区返回 RT_EOK 且不填 data_ptr，
     *   所以必须用 storage_is_empty() 显式判断，不能只看返回值。
     *
     * 两种都要清 last_data_buf：
     *   storage_load 空分区分支已执行
     *     memcpy(last_data_buf, data_ptr, size)      (storage.c:121)
     *   不清则 storage_save 的值去重会静默跳过（storage.c:185）。
     * ============================================================ */
    if (ret != RT_EOK) {
        LOG_WARN("[device_info] flash invalid (%d), using defaults\n", (int)ret);
        memset(g_dev_storage.last_data_buf, 0x00, sizeof(device_info_t));
        if (storage_save(&g_dev_storage) != RT_EOK) {
            rt_kprintf("[device_info] WARN: default save FAILED\n");
        }
    }
    else if (storage_is_empty(&g_dev_storage) != 0U) {
        LOG_INFO("[device_info] empty partition, writing defaults...\n");
        memset(g_dev_storage.last_data_buf, 0x00, sizeof(device_info_t));
        if (storage_save(&g_dev_storage) == RT_EOK) {
            rt_kprintf("[device_info] defaults written to flash\n");
        } else {
            rt_kprintf("[device_info] WARN: default save FAILED\n");
        }
    }
    else if (g_device_info.rec_magic != DEVICE_INFO_REC_MAGIC) {
        /* ---- 记录损坏：CRC 过了但内容不是本版本写的（旧格式/位翻转）----
         * device_info 是产品身份资产（SN/型号/厂商日期），
         * 若 rec_magic 不符说明整块不可信，必须回默认并重写，
         * 绝不能把乱码当成 SN 上报。 */
        LOG_WARN("[device_info] bad rec_magic 0x%08X (expect 0x%08X), reset to default\n",
                   (unsigned)g_device_info.rec_magic,
                   (unsigned)DEVICE_INFO_REC_MAGIC);

        /* 恢复静态默认值（与 g_device_info 初值一致） */
        memset(g_device_info.sn, 0, sizeof(g_device_info.sn));
        memset(g_device_info.name, 0, sizeof(g_device_info.name));
        rt_strncpy((char *)g_device_info.sn, "JC_202610080001",
                   sizeof(g_device_info.sn) - 1U);
        rt_strncpy((char *)g_device_info.name, "JC_BLDC_Controller",
                   sizeof(g_device_info.name) - 1U);
        g_device_info.model                 = 1U;
        g_device_info.hw_version            = 0U;
        g_device_info.sw_version            = 0U;
        memset(g_device_info.sw_version_str, 0, sizeof(g_device_info.sw_version_str));
        rt_strncpy((char *)g_device_info.sw_version_str, "v0.0.0",
                   sizeof(g_device_info.sw_version_str) - 1U);
        g_device_info.manufacture_date      = 20261008U;
        g_device_info.first_boot_timestamp  = 0U;
        memset(g_device_info.reserved, 0, sizeof(g_device_info.reserved));
        g_device_info.rec_magic             = DEVICE_INFO_REC_MAGIC;

        memset(g_dev_storage.last_data_buf, 0x00, sizeof(device_info_t));
        if (storage_save(&g_dev_storage) == RT_EOK) {
            rt_kprintf("[device_info] reset written to flash\n");
        } else {
            rt_kprintf("[device_info] WARN: reset save FAILED\n");
        }
    }

    device_info_dump();
    return RT_EOK;
}

int device_info_save(void)
{
    return storage_save(&g_dev_storage);
}

int device_info_load(void)
{
    return storage_load(&g_dev_storage);
}

device_info_t* device_info_get(void)
{
    return &g_device_info;
}

/* ============================================================
 * 字段访问
 * ============================================================ */
int device_info_get_field(const char *name, void *out, uint32_t *size)
{
    SM_ASSERT_PTR(name);
    SM_ASSERT_PTR(out);
    SM_ASSERT_PTR(size);

    for (uint32_t i = 0; i < g_device_info_field_count; i++) {
        const device_field_t *f = &g_device_info_fields[i];
        if (strcmp(f->name, name) == 0) {
            memcpy(out, (uint8_t*)&g_device_info + f->offset, f->size);
            *size = f->size;
            return 0;
        }
    }
    return -1;
}

int device_info_set_field(const char *name, const void *value, uint32_t size)
{
    SM_ASSERT_PTR(name);
    SM_ASSERT_PTR(value);

    for (uint32_t i = 0; i < g_device_info_field_count; i++) {
        const device_field_t *f = &g_device_info_fields[i];
        if (strcmp(f->name, name) == 0) {
            if (!f->writable) return -2;  /* 不可写 */
            if (size != f->size) return -3;  /* 大小不匹配 */
            memcpy((uint8_t*)&g_device_info + f->offset, value, f->size);
            return device_info_save();
        }
    }
    return -1;
}

/* ============================================================
 * 调试打印
 * ============================================================ */
void device_info_dump(void)
{
    rt_kprintf("========== Device Info ==========\n");
    for (uint32_t i = 0; i < g_device_info_field_count; i++) {
        const device_field_t *f = &g_device_info_fields[i];
        rt_kprintf("  %-12s: ", f->name);

        switch (f->type) {
            case FIELD_TYPE_STRING:
                rt_kprintf("%s\n", (char*)((uint8_t*)&g_device_info + f->offset));
                break;
            case FIELD_TYPE_U8:
                rt_kprintf("%d\n", *(uint8_t*)((uint8_t*)&g_device_info + f->offset));
                break;
            case FIELD_TYPE_U16:
                rt_kprintf("%d\n", *(uint16_t*)((uint8_t*)&g_device_info + f->offset));
                break;
            case FIELD_TYPE_U32:
                rt_kprintf("0x%08X\n", *(uint32_t*)((uint8_t*)&g_device_info + f->offset));
                break;
            default:
                rt_kprintf("(binary)\n");
                break;
        }
    }
    rt_kprintf("================================\n\n");
}
