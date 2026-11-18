/**
 * @file    storage_config.h
 * @brief   存储分区配置结构 + 查找接口
 * @note    定义所有存储分区的地址、大小和块大小
 */

#ifndef __STORAGE_CONFIG_H
#define __STORAGE_CONFIG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 1. Flash 扇区大小（根据 MCU 调整）
 * ============================================================ */
#define FLASH_SECTOR_SIZE           8192        /* 8KB 扇区 */
#define FLASH_PARAM_SECTOR_SIZE     0x2000U     /* 同上，统一为无符号常量 */

/* ============================================================
 * 1.1 参数区地址规划 —— 唯一地址来源
 * ============================================================
 * 【为什么用 0x0000xxxx 而不是 0x0800xxxx】
 *   hc32_ll_efm.h:86  EFM_START_ADDR = 0x00000000
 *   hc32_ll_efm.h:88  EFM_END_ADDR   = 0x0007FFFF   （512KB Flash）
 *   EFM 库按「0x00000000 起始的别名地址」设计，
 *   IS_EFM_ADDR(x) 只校验 x <= EFM_END_ADDR。
 *   若传 0x08xxxxxx，必然越界；而 DDL_ASSERT 在非 __DEBUG 下是空宏，
 *   越界不会被发现，写入会静默失败。
 *
 * 【布局】取 512KB Flash 的最后 32KB
 *   0x00078000  sys_param      8KB
 *   0x0007A000  device_info    8KB  ← bootloader 也读此块，地址须与其一致
 *   0x0007C000  ota_shared     8KB
 *   0x0007E000  mode_config    8KB
 *
 * 【V1.3 追加】取 0x00076000 作为 mech_param（机械参数表），
 *   位于原 PARAM 区之前的独立 8KB 扇区，不动既有 4 个分区地址：
 *   0x00076000  mech_param     8KB  ← 编码器齿数比/减速比多行 profile 表
 *
 *   【为什么必须独立分区，不能扩 sys_param_t】
 *   Q-40 裁定为多行表后整表需 236 B。扩 sys_param_t 会破坏
 *   "机械参数(出厂级) / 零点(现场资产)" 解耦，机械参数改一次连带重写
 *   零点区；且 sys_param.h:36-38 明确"字段顺序与 sizeof 不得随意变更"，
 *   扩结构体等于让现场已标定设备的零点丢失。故独占一个扇区。
 *
 * App 当前约 194KB（_etext=0x00030834），距 0x00076000 余量约 285KB。
 * 若后续 OTA 分区调整，只需改 FLASH_PARAM_BASE 一处。
 * ============================================================ */
#define FLASH_PARAM_BASE            0x00078000U

#define FLASH_ADDR_SYS_PARAM        (FLASH_PARAM_BASE + 0x0000U)
#define FLASH_ADDR_DEVICE_INFO      (FLASH_PARAM_BASE + 0x2000U)
#define FLASH_ADDR_OTA_SHARED       (FLASH_PARAM_BASE + 0x4000U)
#define FLASH_ADDR_MODE_CONFIG      (FLASH_PARAM_BASE + 0x6000U)

/* V1.3：机械参数表独立分区（在原 4 区之前，8KB 对齐） */
#define FLASH_ADDR_MECH_PARAM       0x00076000U

/* 【2026-11-03 新增】系统配置表 sys_config
 *
 * 选址依据：0x00070000~0x00076000 为 24KB 空档，0x74000 与既有分区
 * 均不冲突。App 尾当前 0x00030834，到本地址余量约 270KB。
 *
 * ⚠️ BootLoader 尚未设计。等设计 OTA 双分区时必须重新核对本地址，
 *    届时分区表可能整体重排（记录为遗留项，非本次范围）。
 *
 * 内容：可在线读写的运行参数（整车状态阈值/看门狗/电流零点/标定标志等），
 *      用 cfg_item_t 自描述项数组承载，与 sys_param 的"现场资产"解耦。 */
#define FLASH_ADDR_SYS_CONFIG       0x00074000U

/* ============================================================
 * 2. 分区配置结构
 * ============================================================ */
typedef struct {
    const char *name;           /* 数据对象名称 */
    uint32_t    sector_addr;    /* 分区起始地址 */
    uint32_t    sector_size;    /* 分区大小 */
    uint32_t    magic;          /* 分区魔数 */
    uint32_t    cooldown_ms;    /* 写入冷却时间（毫秒） */
} storage_config_t;

/* ============================================================
 * 3.外部引用
 * ============================================================ */
extern const storage_config_t g_storage_configs[];
extern const uint32_t g_storage_config_count;

/* ============================================================
 * API
 * ============================================================ */

/**
 * @brief 按名称查找存储配置
 * @param name 数据对象名称
 * @return 配置指针，未找到返回 NULL
 */
const storage_config_t* storage_find_config(const char *name);

#ifdef __cplusplus
}
#endif

#endif /* __STORAGE_CONFIG_H */
//
///* ============================================================
// * 2. 内部 Flash 分区地址（HC32F460 布局）
// * ============================================================ */
//#define FLASH_PARTITION_SYS         0x080E0000  /* 系统参数 */
//#define FLASH_PARTITION_DEVICE      0x080E2000  /* 设备信息 */
//#define FLASH_PARTITION_OTA_SHARED  0x080E4000  /* OTA 共享参数 */
//
///* ============================================================
// * 3. 各分区块大小（业务数据结构大小，对齐到 4 字节）
// * ============================================================ */
//#define SYS_PARAM_BLK_SIZE          (sizeof(sys_param_t))
//#define DEVICE_INFO_BLK_SIZE        (sizeof(device_info_t))
//#define OTA_SHARED_BLK_SIZE         (sizeof(ota_shared_t))
//
///* ============================================================
// * 4. 各分区魔数（用于识别有效块）
// * ============================================================ */
//#define SYS_PARAM_MAGIC             0x53595350  /* "SYSP" */
//#define DEVICE_INFO_MAGIC           0x44455649  /* "DEVI" */
//#define OTA_SHARED_MAGIC            0x4F5441    /* "OTA" */
//
///* ============================================================
// * 5. 存储块最大大小（用于写入缓冲区）
// * ============================================================ */
//#define STORAGE_BLK_MAX_SIZE        256         /* 最大 256 字节 */
//
///* ============================================================
// * 6. 写入冷却时间（毫秒）
// * ============================================================ */
//#define STORAGE_COOLDOWN_SYS        500         /* 系统参数 500ms */
//#define STORAGE_COOLDOWN_DEVICE     1000        /* 设备信息 1000ms */
//#define STORAGE_COOLDOWN_OTA        100         /* OTA 共享 100ms */
//
//#ifdef __cplusplus
//}
//#endif
//
//#endif /* __STORAGE_CONFIG_H */
