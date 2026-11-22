/**
 * @file    ota_shared.h
 * @brief   OTA 共享参数数据对象（业务模块）
 */

#ifndef __OTA_SHARED_H
#define __OTA_SHARED_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * OTA 状态枚举
 * ============================================================ */
typedef enum {
    OTA_STATUS_IDLE = 0,
    OTA_STATUS_READY,
    OTA_STATUS_UPDATING,
    OTA_STATUS_SUCCESS,
    OTA_STATUS_FAILED,
    OTA_STATUS_ROLLBACK,
} ota_status_t;

/* ============================================================
 * OTA 共享参数结构体
 * ============================================================ */
typedef struct {
    uint32_t     rec_magic;         /* 【2026-11-03 新增】记录识别魔数 = OTA_SHARED_REC_MAGIC
                                     *
                                     * 为什么需要：
                                     *   本结构体所有业务字段的默认值都是 0
                                     *   （flag=OTA_FLAG_IDLE=0、status=OTC_STATUS_IDLE=0、
                                     *    其余全 0），导致"空分区首次写默认值"时
                                     *   memset(last_data_buf, 0) 后与 data_ptr 完全相同，
                                     *   被 storage_save 的值去重 (storage.c:185) 静默跳过，
                                     *   默认值永远写不进 Flash。
                                     *
                                     * 加此非零字段后：
                                     *   ① 默认值天然非零 -> memset 清零后必然不等 -> 可正常落盘
                                     *   ② 顺带提供一层记录完整性校验（magic 不符 = 记录损坏）
                                     *   ③ 把"默认值必须非零"从隐式前提变成显式约束
                                     *
                                     * 放在结构体首部，偏移 0。 */
    uint32_t     flag;              /* OTA_FLAG_IDLE / OTA_FLAG_READY */
    uint32_t     target_version;    /* 目标版本号 */
    uint32_t     crc32;             /* 升级包 CRC32 */
    uint32_t     size;              /* 升级包大小 */
    uint32_t     progress;          /* 0~100 */
    ota_status_t status;            /* OTA 状态 */
} ota_shared_t;

/* 记录识别魔数（与分区 magic 0x004F5441 "OTA\\0" 区分：本值是 "OTAS"）
 * 取值依据：ASCII 'O''T''A''S'，大端写法便于人工识别；落盘按小端。 */
#define OTA_SHARED_REC_MAGIC    0x4F544153U

/* 标志位定义 */
#define OTA_FLAG_IDLE       0x00000000
#define OTA_FLAG_READY      0xA5A5A5A5

/* ============================================================
 * 外部引用
 * ============================================================ */
extern ota_shared_t g_ota_shared;

/* ============================================================
 * API
 * ============================================================ */
int ota_shared_init(void);
int ota_shared_save(void);
int ota_shared_load(void);
ota_shared_t* ota_shared_get(void);

/* 便捷接口 */
int ota_shared_set_upgrade(uint32_t target_version, uint32_t crc32, uint32_t size);
int ota_shared_set_status(ota_status_t status);
int ota_shared_clear(void);

#ifdef __cplusplus
}
#endif

#endif /* __OTA_SHARED_H */
