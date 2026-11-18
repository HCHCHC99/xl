/**
 * @file    mode_config.h
 * @brief   模式配置（持久化到 Flash）
 */

#ifndef __MODE_CONFIG_H
#define __MODE_CONFIG_H

#include <stdint.h>
#include "sys_mode.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 模式允许位掩码
 * ============================================================ */
#define MODE_ALLOW_FACTORY   (1 << 0)
#define MODE_ALLOW_TEST      (1 << 1)
#define MODE_ALLOW_WORK      (1 << 2)

/* ============================================================
 * 模式配置结构体
 * ============================================================ */
typedef struct {
    uint8_t  default_mode;          /* 上电默认模式 */
    uint8_t  allowed_modes;         /* 允许的模式位掩码 */
    uint8_t  factory_lock;          /* 工厂模式锁定标志（1=锁） */
    uint8_t  cap_override;          /* 【原 reserved1】能力位收紧掩码
                                     *   0x00 = 不覆盖，用 sys_caps.c 的默认表
                                     *   其它 = 按位收紧（只能收紧不能越权）
                                     *   仅覆盖低 8 位能力，见 sys_caps.c 说明
                                     *   旧 flash 记录此字节为 0，天然兼容 */
    uint32_t test_timeout_ms;       /* 测试模式超时（0=不超时） */
    uint32_t reserved2[4];

    /* ============================================================
     * 【2026-11-03 新增】记录识别魔数
     * ------------------------------------------------------------
     * 加在末尾，不破坏既有字段偏移（与 device_info 同理，
     * 本结构虽无 offsetof 表，但保持"只追加不改序"的一致约定）。
     *
     * 用途：保证默认值非零。本结构 factory_lock / cap_override /
     *       test_timeout_ms / reserved2 默认均为 0，
     *       若无此字段，"空分区写默认值"会被 storage_save 的
     *       值去重 (storage.c:185) 静默跳过。
     *       同时提供记录完整性校验。
     * ============================================================ */
    uint32_t rec_magic;
} mode_config_t;

/* 记录识别魔数（"MODR" = MODe Record）
 * 与分区 magic 0x4D4F4445 ("MODE") 区分。 */
#define MODE_CONFIG_REC_MAGIC   0x4D4F4452U

/* ============================================================
 * API
 * ============================================================ */

/**
 * @brief 加载模式配置（从 Flash）
 * @param config 输出配置
 * @return 0 成功
 */
int mode_config_load(mode_config_t *config);

/**
 * @brief 保存模式配置（到 Flash）
 * @param config 输入配置
 * @return 0 成功
 */
int mode_config_save(const mode_config_t *config);

/**
 * @brief 恢复默认配置
 */
void mode_config_default(mode_config_t *config);

#ifdef __cplusplus
}
#endif

#endif /* __MODE_CONFIG_H */
