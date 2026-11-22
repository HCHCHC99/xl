/**
 * @file    storage_config.c
 * @brief   存储分区配置表实例
 */

#include "storage_config.h"
#include <string.h>

/* ============================================================
 * 分区配置表
 * ============================================================ */
const storage_config_t g_storage_configs[] = {
    /* V1.3：机械参数表（编码器齿数比/减速比多行 profile），独立扇区。
     * 放在表首并不影响其他分区；地址是显式指定的，与顺序无关。 */
    {
        .name        = "mech_param",
        .sector_addr = FLASH_ADDR_MECH_PARAM,
        .sector_size = FLASH_PARAM_SECTOR_SIZE,
        .magic       = 0x4D454348,   /* "MECH" */
        .cooldown_ms = 1000,
    },
    {
        .name        = "sys_param",
        .sector_addr = FLASH_ADDR_SYS_PARAM,
        .sector_size = FLASH_PARAM_SECTOR_SIZE,
        .magic       = 0x53595350,   /* "SYSP" */
        .cooldown_ms = 500,
    },
    {
        .name        = "device_info",
        .sector_addr = FLASH_ADDR_DEVICE_INFO,
        .sector_size = FLASH_PARAM_SECTOR_SIZE,
        .magic       = 0x44455649,   /* "DEVI" */
        .cooldown_ms = 1000,
    },
    {
        .name        = "ota_shared",
        .sector_addr = FLASH_ADDR_OTA_SHARED,
        .sector_size = FLASH_PARAM_SECTOR_SIZE,
        .magic       = 0x4F5441,     /* "OTA" */
        .cooldown_ms = 100,
    },
    {
        .name        = "mode_config",
        .sector_addr = FLASH_ADDR_MODE_CONFIG,
        .sector_size = FLASH_PARAM_SECTOR_SIZE,
        .magic       = 0x4D4F4445,   /* "MODE" */
        .cooldown_ms = 1000,
    },
    /* 【2026-11-03 新增】系统配置表：在线可改的运行参数。
     * cooldown 取 2000ms：配置项改动比零点标定频繁，
     * 加大冷却可防 MSH 连刷写坏 Flash（10 万次擦写 / 每天 10 次 = 27 年）。 */
    {
        .name        = "sys_config",
        .sector_addr = FLASH_ADDR_SYS_CONFIG,
        .sector_size = FLASH_PARAM_SECTOR_SIZE,
        .magic       = 0x43464731,   /* "CFG1" */
        .cooldown_ms = 2000,
    },
};

const uint32_t g_storage_config_count = sizeof(g_storage_configs) / sizeof(g_storage_configs[0]);

/* ============================================================
 * 查找配置
 * ============================================================ */
const storage_config_t* storage_find_config(const char *name)
{
    if (name == NULL) return NULL;

    for (uint32_t i = 0; i < g_storage_config_count; i++) {
        if (strcmp(g_storage_configs[i].name, name) == 0) {
            return &g_storage_configs[i];
        }
    }
    return NULL;
}
