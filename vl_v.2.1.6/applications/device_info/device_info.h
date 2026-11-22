/**
 * @file    device_info.h
 * @brief   设备信息块管理
 * @note    存储于参数区的 device_info 分区，地址由
 *          storage_config.h 的 FLASH_ADDR_DEVICE_INFO 统一定义（当前 0x0007A000）。
 *          bootloader 与 App 必须使用同一地址，修改时需同步。
 *          注意：EFM 驱动使用 0x00000000 起始的别名地址，不是 0x08000000。
 */

#ifndef __DEVICE_INFO_H
#define __DEVICE_INFO_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 常量定义
 * ============================================================ */
#define DEVICE_SN_LEN               16
#define DEVICE_NAME_LEN             32
#define DEVICE_SW_VERSION_LEN       16

/* ============================================================
 * 枚举定义
 * ============================================================ */

/* ============================================================
 * 设备信息块结构体
 * ============================================================ */
#pragma pack(push, 4)
typedef struct {
    /* ----- 产品身份信息 ----- */
    uint8_t  sn[DEVICE_SN_LEN];         /* 产品序列号 (16字节) */
    uint8_t  name[DEVICE_NAME_LEN];     /* 产品名称 (32字节) */
    uint32_t model;                     /* 设备型号 (device_model_t) */
    uint32_t hw_version;                /* 硬件版本 (0x00010000 = v1.0.0) */

    /* ----- 固件版本信息 ----- */
    uint32_t sw_version;                /* 当前固件版本 (0x00010000 = v1.0.0) */
    uint8_t  sw_version_str[DEVICE_SW_VERSION_LEN]; /* 版本字符串 "v1.0.0" */

    /* ----- 生产信息 ----- */
    uint32_t manufacture_date;          /* 生产日期 (YYYYMMDD) */
    uint32_t first_boot_timestamp;      /* 首次开机时间戳 */

    /* ----- 扩展与保留 ----- */
    uint32_t reserved[16];              /* 保留 16 个 uint32_t (64字节) */

    /* ============================================================
     * 【2026-11-03 新增】记录识别魔数
     * ------------------------------------------------------------
     * ⚠️ 必须加在【末尾】而非首部：
     *   本文件下方 g_device_info_fields[] 用 offsetof() 逐字段定位，
     *   首部插入会把所有字段偏移整体后移，导致字段表全部错位。
     *   加在末尾则既有偏移完全不变。
     *
     * 用途：保证默认值非零（其余字段如 first_boot_timestamp 可能为 0），
     *       使"空分区写默认值"不被 storage_save 的值去重跳过；
     *       同时提供记录完整性校验。
     * ============================================================ */
    uint32_t rec_magic;
} device_info_t;
#pragma pack(pop)

/* 记录识别魔数（"DEVR" = DEVice Record）
 * 与分区 magic 0x44455649 ("DEVI") 区分。 */
#define DEVICE_INFO_REC_MAGIC   0x44455652U

/* ============================================================
 * 字段类型枚举
 * ============================================================ */
typedef enum {
    FIELD_TYPE_U8,
    FIELD_TYPE_U16,
    FIELD_TYPE_U32,
    FIELD_TYPE_STRING,
    FIELD_TYPE_BYTES,
} field_type_t;

/* ============================================================
 * 字段元信息
 * ============================================================ */
typedef struct {
    const char  *name;       /* 字段名称 */
    uint32_t     offset;     /* 在结构体中的偏移 */
    uint32_t     size;       /* 字段大小 */
    field_type_t type;       /* 字段类型 */
    uint8_t      writable;   /* 是否可写 */
} device_field_t;

/* ============================================================
 * 外部引用
 * ============================================================ */
extern device_info_t g_device_info;
extern const device_field_t g_device_info_fields[];
extern const uint32_t g_device_info_field_count;

/* ============================================================
 * API
 * ============================================================ */

/**
 * @brief 初始化设备信息模块
 * @return 0 成功，-1 失败
 * @note  首次上电时创建默认设备信息
 */
int device_info_init(void);

/**
 * @brief 读取设备信息
 * @param info 输出指针
 * @return 0 成功，-1 失败
 */
int device_info_read(device_info_t *info);

/**
 * @brief 写入设备信息
 * @param info 输入指针
 * @return 0 成功，-1 失败
 * @note  写入前会自动计算 CRC32
 */
int device_info_write(const device_info_t *info);

/**
 * @brief 更新软件版本
 * @param version      版本号 (如 0x00010000)
 * @param version_str  版本字符串 (如 "v1.0.0")
 * @return 0 成功，-1 失败
 */
int device_info_update_sw_version(uint32_t version, const char *version_str);

/**
 * @brief 获取软件版本号
 * @return 版本号，0 表示读取失败
 */
uint32_t device_info_get_sw_version(void);

/**
 * @brief 获取软件版本字符串
 * @param buf  输出缓冲区
 * @param size 缓冲区大小
 */
void device_info_get_sw_version_str(char *buf, uint16_t size);

/**
 * @brief 获取设备序列号
 * @param buf  输出缓冲区
 * @param size 缓冲区大小
 */
void device_info_get_sn(char *buf, uint16_t size);

/**
 * @brief 调试打印设备信息
 */
void device_info_dump(void);

#ifdef __cplusplus
}
#endif

#endif /* __DEVICE_INFO_H */
