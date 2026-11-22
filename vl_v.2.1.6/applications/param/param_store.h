/**
 * @file    param_store.h
 * @brief   业务参数存取层：把 sys_param / mech_cfg / sys_config / device_info
 *          以及快区标定量打包成 param_manager 记录。
 *
 * 记录归属：
 *   MOTION     (56) 电机/PWM/转速/堵转 sys_config 项
 *   ANGLE_MECH (57) 机械 active 行 + 选择子 + sys_param 关键量
 *   SYSTEM     (58) 整车/ACC/看门狗/举升/机械选择子 sys_config 项
 *   FAST       (59~61) 绝对编码器—电角度标定（SPI 预留）
 *   DEVICE     (63) 设备信息关键字段
 */

#ifndef __PARAM_STORE_H
#define __PARAM_STORE_H

#include <stdint.h>
#include "param_map.h"
#include "../device_info/device_info.h"
#include "../mode/mode_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 保存请求位 */
#define PARAM_REQ_CFG       (1U << 0)
#define PARAM_REQ_AM        (1U << 1)
#define PARAM_REQ_DEV       (1U << 2)
#define PARAM_REQ_FAST      (1U << 3)
#define PARAM_REQ_ALL       (PARAM_REQ_CFG | PARAM_REQ_AM | PARAM_REQ_DEV | PARAM_REQ_FAST)

/* ============================================================
 * 初始化 / 保存
 * ============================================================ */

/**
 * @brief 初始化全部存储实例（可重复调用，内部只执行一次）。
 * @note  必须在 storage_all_init() 阶段、sys_config/mech_cfg 之前调用。
 */
int  param_store_init(void);

/**
 * @brief 同步保存指定记录（阻塞，直到写完）
 * @note  只用于上电默认值落盘、擦除等必须立刻完成的场合。
 */
int  param_store_save_now(uint32_t mask);

/**
 * @brief 请求保存（异步，交给低优先级线程；电机运行时自动延后）
 * @return 0 已排队
 */
int  param_store_request(uint32_t mask);

/* ============================================================
 * 诊断（MSH / 上电自检）
 * ============================================================ */
#define PARAM_DUMP_INFO     0x01U   /* 扇区/剩余槽位/擦除次数/seq */
#define PARAM_DUMP_VALUES   0x02U   /* 各组当前值（含 sys_config ID） */

/* 上电自检结束后自动打印一次；置 0 可关闭 */
#ifndef PARAM_BOOT_DUMP_ENABLE
#define PARAM_BOOT_DUMP_ENABLE  1
#endif

void param_store_dump(uint32_t flags);
void param_store_boot_dump(void);
void param_store_boot_prepare(void);   /* 上电快区预擦除 */

/* MSH 手动保存：mask 取 PARAM_REQ_*；电机运行中拒绝，返回 -2 */
int  param_store_save_cmd(uint32_t mask);

/* MSH 擦除：擦掉 mask 对应区域后，把当前 RAM 值作为新记录写回（seq 从 1 开始）。
 * 电机运行中拒绝，返回 -2。 */
int  param_store_erase_cmd(uint32_t mask);

/* MSH 填充：对 group（motion/system/angle_mech/device/fast）反复写入当前值，
 * 直到当前扇区只剩 leave 个槽位。leave 默认 2。 */
int  param_store_fill_cmd(const char *group, uint16_t leave);

/* MSH 精确写入：对 group 只写 count 次（默认 1），用于"某区只存一次"或手动触发轮转。 */
int  param_store_write_cmd(const char *group, uint16_t count);

/* MSH 设置某组当前扇区的擦除次数（只改 RAM，不写 Flash）。
 * fast 组设为 >= PARAM_FAST_MAX_ERASE_LIFE(10000) 后，下一次写入会换扇区。 */
int  param_store_set_erase_cmd(const char *group, uint32_t count);

/* ============================================================
 * ANGLE_MECH：sys_param + mech active 行
 * ============================================================ */
int  param_store_am_load(void);      /* 记录 -> g_sys_params / g_mech（mech 由调用方决定是否采用） */
int  param_store_am_save(void);      /* g_sys_params / g_mech -> 记录 */
int  param_store_am_was_default(void);
int  param_store_am_get_row(mech_cfg_t *row, uint8_t *active, uint8_t *count);
int  param_store_am_set_row(const mech_cfg_t *row, uint8_t active, uint8_t count);

/* 读取记录里保存的三个机械选择子（用于开机判断是否需要按 cfg 重选行） */
int  param_store_am_get_sel(uint8_t *enc_sel, uint8_t *iin_sel, uint8_t *iout_sel);

/* ============================================================
 * SYSTEM / MOTION：sys_config 关键项
 * ============================================================ */
int  param_store_cfg_load(void);     /* 记录 -> sys_config RAM（逐项范围校验） */
int  param_store_cfg_save(void);     /* sys_config RAM -> 记录 */
int  param_store_cfg_was_default(void);

/* MODE_CONFIG：与 SYSTEM 记录同扇区（扇区 58），共用打包 */
int  param_store_mode_load(mode_config_t *out);
int  param_store_mode_save(const mode_config_t *in);

/* ============================================================
 * DEVICE
 * ============================================================ */
int  param_store_dev_load(device_info_t *out);
int  param_store_dev_save(const device_info_t *in);
int  param_store_dev_was_default(void);

/* ============================================================
 * FAST（SPI 绝对编码器预留）
 * ============================================================ */
int  param_fast_load(param_fast_payload_t *out);
int  param_fast_save(const param_fast_payload_t *in);
/* 同步写快区（下电/欠压路径用）：更新 RAM payload 后立即落盘，只写一条，
 * 不投异步请求，避免重复记录占用槽位、在掉电瞬间触发擦除。 */
int  param_fast_save_now(const param_fast_payload_t *in);
uint8_t param_fast_offset_is_same(uint32_t old_raw, uint32_t new_raw);

#ifdef __cplusplus
}
#endif

#endif /* __PARAM_STORE_H */
