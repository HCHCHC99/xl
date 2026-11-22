/**
 * @file    device_info.c
 * @brief   设备信息业务实现
 */

#include "../device_info/device_info.h"
#include "../param/param_store.h"
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

/* ============================================================
 * 初始化
 *
 * 新存储方案：只持久化 SN/型号/版本/生产日期等关键字段。
 * ============================================================ */
int device_info_init(void)
{
    (void)param_store_init();

    /* 上电默认记录只有 rec_magic，SN 为空；SN 空视为未初始化 */
    if ((param_store_dev_load(&g_device_info) != 0) ||
        (g_device_info.sn[0] == 0U)) {
        LOG_WARN("[device_info] no valid record, using defaults\n");

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

        (void)param_store_dev_save(&g_device_info);
    }

    device_info_dump();
    return RT_EOK;
}

int device_info_save(void)
{
    return param_store_dev_save(&g_device_info);
}

int device_info_load(void)
{
    return param_store_dev_load(&g_device_info);
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
