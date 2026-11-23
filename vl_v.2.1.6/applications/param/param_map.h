/**
 * @file    param_map.h
 * @brief   参数存储分区映射、记录布局与编译期约束
 * @note    方案见 移植过程.md §10.15 / 业务.md 第 12 章
 *
 * ============================================================
 * 设计要点（改动前必读）
 * ============================================================
 * 1. 扇区 8KB，地址按 0x00000000 别名，扇区号 = 地址 / 0x2000。
 * 2. 扇区 62（0x7C000~0x7DFFF）被其他功能占用，本模块不得使用。
 * 3. 慢区（MOTION/ANGLE_MECH/SYSTEM/DEVICE）各占 1 个扇区，
 *    不设擦除次数上限；erase_count 仅递增用于诊断。
 * 4. 快区占 59~61 共 3 个扇区。每个扇区先原地擦除重用，擦除次数
 *    达到 10000 后，下一次该扇区写满才按 61 → 60 → 59 → 61 换扇。
 * 5. 记录只存关键量，单条记录 <= PARAM_MAX_RECORD_SIZE(256)。
 *    布局变化时换 magic，不设 version 字段：同一 magic 下按
 *    sequence_id 取最大者。
 * ============================================================
 */

#ifndef __PARAM_MAP_H
#define __PARAM_MAP_H

#include <stdint.h>
#include <stddef.h>

#include "../angle/mech_cfg.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 1. 物理分区
 * ============================================================ */
#define PARAM_SECTOR_SIZE           0x2000U      /* 8 KB */
#define PARAM_RECORD_ALIGN          4U

/* 扇区号 */
#define PARAM_SEC_MOTION            56U          /* 0x70000~0x71FFF */
#define PARAM_SEC_ANGLE_MECH        57U          /* 0x72000~0x73FFF */
#define PARAM_SEC_SYSTEM            58U          /* 0x74000~0x75FFF */
#define PARAM_SEC_FAST_START        61U          /* 快区最高扇区 0x7A000~0x7BFFF */
#define PARAM_SEC_FAST_END          59U          /* 快区最低扇区 0x76000~0x77FFF */
#define PARAM_SEC_RESERVED          62U          /* 0x7C000~0x7DFFF 禁用 */
#define PARAM_SEC_DEVICE            63U          /* 0x7E000~0x7FFFF */

#define PARAM_SEC_ADDR(sec)         ((uint32_t)(sec) * PARAM_SECTOR_SIZE)

/* 快区扇区数 */
#define PARAM_FAST_SECTOR_COUNT     (PARAM_SEC_FAST_START - PARAM_SEC_FAST_END + 1U)
/* 每扇区擦除次数上限（仅快区；0 = 不限） */
#define PARAM_FAST_MAX_ERASE_LIFE   10000U

/* 单条记录上限 */
#define PARAM_MAX_RECORD_SIZE       256U

/* 记录头/尾相对偏移（所有记录统一） */
#define PARAM_REC_OFF_HEAD          0U
#define PARAM_REC_OFF_SEQ           4U
#define PARAM_REC_OFF_ERASE         8U
#define PARAM_REC_HDR_SIZE          12U

/* ============================================================
 * 2. 记录 magic（成对出现，尾部必须等于头部取反的低 16 位模式）
 * ============================================================ */
#define PARAM_MAGIC_MOTION          0x5501A001U
#define PARAM_MAGIC_SYSTEM          0x5502A012U
#define PARAM_MAGIC_ANGLE_MECH      0x5503A003U
#define PARAM_MAGIC_DEVICE          0x5504A004U
#define PARAM_MAGIC_FAST            0x5505A015U

#define PARAM_TAIL_OF(m)            ((uint32_t)((m) ^ 0xA5A5A5A5U))

/* ============================================================
 * 3. 编译期断言
 * ============================================================ */
#define PARAM_STATIC_ASSERT(cond, tag) \
    typedef char param_static_assert_##tag[(cond) ? 1 : -1]

/* 记录大小必须 <= 256 且 4 字节对齐 */
#define PARAM_ASSERT_REC(name, type) \
    PARAM_STATIC_ASSERT(sizeof(type) <= PARAM_MAX_RECORD_SIZE, name##_size_le_256); \
    PARAM_STATIC_ASSERT((sizeof(type) % PARAM_RECORD_ALIGN) == 0U, name##_size_align4); \
    PARAM_STATIC_ASSERT(offsetof(type, head) == PARAM_REC_OFF_HEAD, name##_off_head); \
    PARAM_STATIC_ASSERT(offsetof(type, tail) == sizeof(type) - 4U, name##_off_tail)

/* 分区互不重叠：慢区 < 快区 < 预留 < DEVICE */
PARAM_STATIC_ASSERT(PARAM_SEC_MOTION < PARAM_SEC_ANGLE_MECH, sec_motion_angle);
PARAM_STATIC_ASSERT(PARAM_SEC_ANGLE_MECH < PARAM_SEC_SYSTEM, sec_angle_system);
PARAM_STATIC_ASSERT(PARAM_SEC_SYSTEM < PARAM_SEC_FAST_END, sec_system_fast);
PARAM_STATIC_ASSERT(PARAM_SEC_FAST_END < PARAM_SEC_FAST_START, sec_fast_order);
PARAM_STATIC_ASSERT(PARAM_SEC_FAST_START < PARAM_SEC_RESERVED, sec_fast_reserved);
PARAM_STATIC_ASSERT(PARAM_SEC_RESERVED < PARAM_SEC_DEVICE, sec_reserved_device);

/* ============================================================
 * 4. MOTION：电机 / PWM / 转速 / 堵转（24 项 sys_config）
 * ============================================================ */
#define PARAM_MOTION_CFG_COUNT      24U

typedef struct {
    uint32_t head;
    uint32_t seq;
    uint32_t erase_count;
    uint32_t cfg[PARAM_MOTION_CFG_COUNT];   /* 与 s_motion_ids[] 一一对应 */
    uint32_t checksum;                      /* CRC32 over [0, checksum) */
    uint32_t tail;
} param_motion_rec_t;

/* ============================================================
 * 5. SYSTEM：整车 / ACC / CAN / 看门狗 / 机械 / 欠压 / 举升传感器阈值 / 限位保持 / 过零停车（33 项）
 * ============================================================ */
#define PARAM_SYSTEM_CFG_COUNT      33U
#define PARAM_MODE_REC_MAGIC        0x4D4F4445U   /* "MODE" */

typedef struct {
    uint32_t rec_magic;
    uint8_t  default_mode;
    uint8_t  allowed_modes;
    uint8_t  factory_lock;
    uint8_t  cap_override;
    uint32_t test_timeout_ms;
    uint32_t reserved[4];
} param_mode_payload_t;

typedef struct {
    uint32_t head;
    uint32_t seq;
    uint32_t erase_count;
    uint32_t cfg[PARAM_SYSTEM_CFG_COUNT];
    param_mode_payload_t mode;
    uint32_t checksum;
    uint32_t tail;
} param_system_rec_t;

/* ============================================================
 * 6. ANGLE_MECH：机械 active 行 + 三选择子 + 系统标定量
 * ============================================================ */
#define PARAM_AM_REC_MAGIC          0x414D5231U   /* "AMR1" */

typedef struct {
    uint32_t    rec_magic;

    /* t_mech_set 可改的 active 行整行保存，避免只存 5 个字段丢配置 */
    mech_cfg_t  row;
    uint8_t     active;         /* 原表 active 行索引 */
    uint8_t     count;          /* 原表有效行数 */
    uint8_t     enc_sel;        /* 0x0A01 */
    uint8_t     iin_sel;        /* 0x0A02 */
    uint8_t     iout_sel;       /* 0x0A03 */

    /* sys_param 关键量 */
    uint8_t     enc_zero_valid;
    uint8_t     teeth_type;
    uint8_t     is_calibrated;
    uint8_t     teeth_clear_action;
    uint16_t    enc_zero_raw[3];   /* Z0/Z1/Z2 */
    uint16_t    lift_zero_voltage_mv;
} param_am_payload_t;

typedef struct {
    uint32_t head;
    uint32_t seq;
    uint32_t erase_count;
    param_am_payload_t payload;
    uint32_t checksum;
    uint32_t tail;
} param_am_rec_t;

/* ============================================================
 * 7. DEVICE：设备信息关键字段
 * ============================================================ */
#define PARAM_DEV_REC_MAGIC         0x44455652U   /* "DEVR" */

typedef struct {
    uint32_t rec_magic;
    uint8_t  sn[16];
    uint8_t  name[32];
    uint32_t model;
    uint32_t hw_version;
    uint32_t sw_version;
    uint8_t  sw_version_str[16];
    uint32_t manufacture_date;
    uint32_t first_boot_timestamp;
} param_dev_payload_t;

typedef struct {
    uint32_t head;
    uint32_t seq;
    uint32_t erase_count;
    param_dev_payload_t payload;
    uint32_t checksum;
    uint32_t tail;
} param_dev_rec_t;

/* ============================================================
 * 8. FAST：ABZ 电角度零点 / 筒角快照 / 对齐指纹
 *    （SPI 绝对编码器仅保留 source=1 的语义位置，本阶段恒 0）
 * ============================================================ */
#define PARAM_FAST_REC_MAGIC        0x46415332U   /* "FAS2" */

/* 快区差值容差：原始计数比较，±5 counts。
 * 4096 CPR / 10 对极下，5 counts ≈ 0.44° 机械角 ≈ 4.4° 电角度。
 * 差值在容差内视为一致，不重存，避免每次上电都擦写快区。 */
#ifndef PARAM_FAST_DIFF_TOL_COUNTS
#define PARAM_FAST_DIFF_TOL_COUNTS  5
#endif

typedef struct {
    uint32_t rec_magic;
    uint32_t calib_count;        /* mode3 / 一点式对齐累计次数（诊断） */
    uint32_t offset;             /* 电角度零点（counts，0..CPR-1） */
    uint32_t offset_fingerprint; /* 上电对齐指纹（同 offset 域，用于比对） */
    int32_t  tube_angle_x10;     /* 筒角快照 0.1° */
    uint32_t snapshot_seq;       /* 欠压存档次数（诊断） */
    uint16_t cpr;                /* 编码器 CPR（默认 4096） */
    uint8_t  pole_pairs;         /* 极对数（默认 10） */
    int8_t   elec_dir;           /* 电角度方向 ±1 */
    uint8_t  valid;              /* bit0=offset/dir 有效；bit1=筒角快照有效；bit2=指纹有效 */
    uint8_t  source;             /* 0=ABZ 存档；1=SPI 绝对（预留，本阶段恒 0） */
    uint8_t  reserved[2];
} param_fast_payload_t;

typedef struct {
    uint32_t head;
    uint32_t seq;
    uint32_t erase_count;
    param_fast_payload_t payload;
    uint32_t checksum;
    uint32_t tail;
} param_fast_rec_t;

/* ============================================================
 * 9. 记录大小断言
 * ============================================================ */
PARAM_ASSERT_REC(motion, param_motion_rec_t);
PARAM_ASSERT_REC(system, param_system_rec_t);
PARAM_ASSERT_REC(angle_mech, param_am_rec_t);
PARAM_ASSERT_REC(device, param_dev_rec_t);
PARAM_ASSERT_REC(fast, param_fast_rec_t);

/* 快区容量：每条记录至少能放 1 条 */
PARAM_STATIC_ASSERT((PARAM_SECTOR_SIZE / sizeof(param_fast_rec_t)) >= 2U, fast_capacity);

#ifdef __cplusplus
}
#endif

#endif /* __PARAM_MAP_H */
