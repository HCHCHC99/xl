/**
 * @file    param_manager.h
 * @brief   参数存储引擎（记录头尾 magic + seq + erase_count + CRC32，多扇区轮转）
 * @note    参考 FA04A param_manager 方案，按本工程扇区布局重写。
 *
 * 记录格式（固定步长）：
 *   [head(4)][seq(4)][erase_count(4)][payload ...][checksum(4)][tail(4)]
 *
 * 上电扫描：逐扇区逐记录读 head/tail，校验 CRC32，取 sequence_id 最大者。
 * 写入：追加到当前扇区末尾；写满或擦除次数到限时换下一个扇区。
 */

#ifndef __PARAM_MANAGER_H
#define __PARAM_MANAGER_H

#include <stdint.h>
#include "param_map.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 返回码
 * ============================================================ */
#define PARAM_OK                    0
#define PARAM_ERR_PARAM            (-1)
#define PARAM_ERR_IO               (-2)
#define PARAM_ERR_NO_SPACE         (-3)
#define PARAM_ERR_VERIFY           (-4)

/* ============================================================
 * EFM 擦写临界区是否关中断
 * ------------------------------------------------------------
 * 0：不关中断，擦写期间 tick/CAN 可响应；
 * 1（默认）：擦写临界区 rt_hw_interrupt_disable()。
 * FA04A 参考工程在 param_manager 里对每次擦/写都关中断，
 * 本工程默认对齐该行为，避免运行期中断取指干扰 EFM。
 * ============================================================ */
#ifndef PARAM_EFM_CRITICAL_DISABLE_IRQ
#define PARAM_EFM_CRITICAL_DISABLE_IRQ   1
#endif

/* ============================================================
 * 配置 / 运行时
 * ============================================================ */
typedef struct {
    uint16_t sec_start;         /* 最高扇区号 */
    uint16_t sec_end;           /* 最低扇区号（单扇区时 == sec_start） */
    uint16_t record_size;       /* sizeof(记录)，必须 4 字节对齐且 <= 256 */
    uint16_t max_erase_life;    /* 0 = 不限；快区 10000 */
    uint32_t magic;             /* 记录头 magic，尾 = magic ^ 0xA5A5A5A5 */
    const char *name;           /* 诊断名 */
} param_config_t;

typedef struct {
    uint16_t curr_sec;          /* 当前写入扇区 */
    uint32_t curr_addr;         /* 下一条记录写入地址 */
    uint32_t last_seq;          /* 最后一条有效记录的 sequence_id */
    uint32_t erase_count;       /* 当前扇区已擦除次数 */
    uint32_t write_count;
    uint32_t skip_count;
    uint8_t  inited;
    uint8_t  defaults_written;  /* 上电未找到有效记录，已写默认值 */
} param_runtime_t;

/* 诊断快照：供 dump / msh 查询，不直接暴露内部字段 */
typedef struct {
    uint16_t record_size;
    uint16_t sec_first;         /* 最低扇区号 */
    uint16_t sec_last;          /* 最高扇区号 */
    uint16_t slots_per_sec;     /* 每扇区可存记录数 */
    uint16_t slots_left;        /* 当前扇区剩余槽位 */
    uint16_t curr_sec;
    uint32_t last_seq;
    uint32_t erase_count;       /* 当前扇区擦除次数 */
    uint32_t write_count;
    uint8_t  defaults_written;
} param_stat_t;

/* 默认值回调：buf 指向 record_size 的 RAM 记录，只需填 payload 区；
 * 头/尾/seq/erase/checksum 由引擎负责。 */
typedef void (*param_defaults_fn)(uint8_t *buf);

/* ============================================================
 * API
 * ============================================================ */

/**
 * @brief 初始化：扫描所有扇区，取 seq 最大的有效记录到 buf；
 *        无有效记录时调用 pfn 填默认值并写入 sec_start。
 * @param  cfg  静态配置
 * @param  rt   运行时状态
 * @param  buf  记录 RAM 缓冲（必须是静态/全局，不能是栈）
 * @param  pfn  默认值回调，可为 NULL（全 0）
 */
int  Param_Init(const param_config_t *cfg, param_runtime_t *rt,
                uint8_t *buf, param_defaults_fn pfn);

/**
 * @brief 追加保存 buf 中的记录（seq 自增并写入 checksum）。
 * @note  写满/到擦除上限时自动换扇区；写入后回读校验。
 */
int  Param_Save(const param_config_t *cfg, param_runtime_t *rt, uint8_t *buf);

/**
 * @brief 重新从 Flash 加载最新记录到 buf（不做默认值填充）
 * @return PARAM_OK；无有效记录返回 PARAM_ERR_NO_SPACE
 */
int  Param_Load(const param_config_t *cfg, param_runtime_t *rt, uint8_t *buf);

/**
 * @brief 擦除全部扇区并写入当前 buf（seq 从 1 开始）
 */
int  Param_EraseAll(const param_config_t *cfg, param_runtime_t *rt, uint8_t *buf);

/**
 * @brief 读取运行时统计
 */
void Param_GetStats(const param_runtime_t *rt,
                    uint32_t *write_count, uint32_t *skip_count,
                    uint32_t *erase_count);

/**
 * @brief 读取诊断快照（扇区/剩余槽位/擦除次数等）
 */
void Param_GetStat(const param_config_t *cfg, const param_runtime_t *rt,
                   param_stat_t *out);

/* ============================================================
 * EFM 端口（param_port_efm.c）
 * ============================================================ */
void     Param_EfmRead(uint32_t addr, uint8_t *buf, uint32_t len);
int      Param_EfmEraseSector(uint32_t sector_addr);
int      Param_EfmWrite(uint32_t addr, const uint8_t *buf, uint32_t len);
uint32_t Param_Crc32(const uint8_t *data, uint32_t len);

/**
 * @brief 应用镜像在 Flash 中的结束地址（用于防止擦写覆盖代码）
 * @return 0 = 当前工具链无法获取（不做重叠检查）
 */
uint32_t Param_GetCodeEnd(void);

#ifdef __cplusplus
}
#endif

#endif /* __PARAM_MANAGER_H */
