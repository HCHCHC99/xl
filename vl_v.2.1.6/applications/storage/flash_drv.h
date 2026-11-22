/**
 * @file    flash_drv.h
 * @brief   Flash 物理层驱动（支持多实例）
 * @note    提供统一的 Flash 读写接口，支持内部 Flash 和外部 Flash
 */

#ifndef __FLASH_DRV_H
#define __FLASH_DRV_H

#include <rtthread.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * blk_size 上限（2026-11-10 新增统一宏）
 * ------------------------------------------------------------
 * 【为什么要有这个宏】
 *   flash_drv.c 里共 3 处断言必须**同值**：
 *     flash_ops_init()   :127
 *     flash_compact()    :296
 *     flash_init()       :393
 *   原先是三处各自硬编码 1024，改一处漏两处会表现为
 *   "某个分区单独失败"，排查成本高。统一成宏后只改这一处。
 *
 * 【为什么从 1024 放到 2048】
 *   2026-11-10 三环模块新增 52 项配置后：
 *     CFG_MAX_ITEMS 96 -> 128
 *     blob     = 8 + 128×8 = 1032B
 *     blk_size = 1032 + 8(块头) = 1040B  > 1024  ← 被拒绝
 *   实机日志表现为：
 *     [CFG] ERROR: blk_size=1040 > 1024, refuse
 *     [BOOT] [ 4/29] sys_config FAIL
 *     [BOOT] FAIL ctrl_loop: dep 'sys_config' not ready
 *   即配置表整体退回默认值，且依赖它的模块全部 SKIP。
 *
 * 【2048 是否安全】
 *   ① flash_ops_init 的 last_data 已是 rt_malloc 动态分配
 *      （不再有栈数组硬上限），2048 不增加任何栈压力
 *   ② 扇区 8192B（FLASH_SECTOR_SIZE），单块 1040B 仍在一个扇区内
 *   ③ 各分区实际 blk_size：sys_config 1040、其余 ≤244，均远低于 2048
 * ============================================================ */
#define FLASH_BLK_SIZE_MAX      2048U

/* ============================================================
 * 1. Flash 设备类型枚举
 * ============================================================ */
typedef enum {
    FLASH_TYPE_INTERNAL = 0,   /* 内部 Flash */
    FLASH_TYPE_SPI,            /* SPI 接口外部 Flash */
    FLASH_TYPE_I2C,            /* I2C 接口外部 Flash */
    FLASH_TYPE_MAX
} flash_type_t;

/* ============================================================
 * 2. Flash 对象前向声明
 * ============================================================ */
typedef struct flash flash_t;

/* ============================================================
 * 3. Flash 操作函数表
 * ============================================================ */
typedef struct {
    rt_err_t  (*init)(flash_t *self);
    rt_err_t  (*write)(flash_t *self, uint32_t addr, const uint8_t *data, uint32_t len);
    void      (*read)(flash_t *self, uint32_t addr, uint8_t *data, uint32_t len);
    rt_err_t  (*append_write)(flash_t *self, const uint8_t *data, uint32_t len);
    rt_err_t  (*compact)(flash_t *self, uint32_t blk_size);
    uint32_t  (*get_last_addr)(flash_t *self);
    uint32_t  (*get_next_addr)(flash_t *self);
} flash_ops_t;

/* ============================================================
 * 4. Flash 对象结构体
 * ============================================================ */
struct flash {
    /* ----- 私有属性 ----- */
    struct {
        const char    *name;          /* 设备名称 */
        flash_type_t   type;          /* 设备类型 */
        uint32_t       sector_addr;   /* 分区起始地址 */
        uint32_t       sector_size;   /* 分区大小 */
        uint32_t       magic;         /* 分区魔数 */
        uint32_t       last_valid_addr;/* 最后有效块地址 */
        uint32_t       next_write_addr;/* 下一个写入地址 */
        uint32_t       blk_size;      /* 块大小 */
        rt_base_t      irq_level;     /* 中断状态保存 */
        /* 外部 Flash 特有 */
        void          *bus_handle;    /* SPI/I2C 总线句柄 */
        uint32_t       chip_select;   /* 片选或设备地址 */
    } priv;

    /* ----- 操作函数表 ----- */
    flash_ops_t *ops;
};

/* ============================================================
 * 5. 内部 Flash 实例（外部引用）
 * ============================================================ */
extern flash_t g_flash_internal;

/* ============================================================
 * 6. API
 * ============================================================ */

/**
 * @brief 初始化 Flash 实例
 * @param self        Flash 对象指针
 * @param sector_addr 分区起始地址
 * @param sector_size 分区大小
 * @param blk_size    块大小
 * @param magic       分区魔数
 * @return RT_EOK 成功，其他失败
 */
rt_err_t flash_init(flash_t *self,
                    uint32_t sector_addr,
                    uint32_t sector_size,
                    uint32_t blk_size,
                    uint32_t magic);

/**
 * @brief 追加写入数据
 * @param self Flash 对象指针
 * @param data 数据指针
 * @param len  数据长度
 * @return RT_EOK 成功，其他失败
 */
rt_err_t flash_append_write(flash_t *self, const uint8_t *data, uint32_t len);

/**
 * @brief 压缩整理扇区
 * @param self     Flash 对象指针
 * @param blk_size 块大小
 * @return RT_EOK 成功，其他失败
 */
rt_err_t flash_compact(flash_t *self, uint32_t blk_size);

/**
 * @brief 获取最后有效块地址
 */
uint32_t flash_get_last_addr(flash_t *self);

/**
 * @brief 获取下一个写入地址
 */
uint32_t flash_get_next_addr(flash_t *self);

#ifdef __cplusplus
}
#endif

#endif /* __FLASH_DRV_H */
