/**
 * @file    sys_param.c
 * @brief   系统参数业务实现（通过通用 storage 读写）
 */

#include "sys_param.h"

/* 【2026-11-10 彩标 P3】系统参数：保存失败=ERR，降级=WARN，成功=OK */
#include "../log/log_color.h"
#include "../storage/storage.h"
#include "../storage/storage_config.h"
#include "framework/state_engine/sm_assert.h"
#include <string.h>
#include <rtthread.h>

/* ============================================================
 * 全局数据对象
 * ============================================================ */
sys_param_t g_sys_params = {
    /* 【2026-11-03】非零记录魔数 —— 见 sys_param.h 的说明。
     * 无此字段时，若默认值恰好全 0，"空分区写默认值"会被值去重跳过。 */
    .rec_magic = SYS_PARAM_REC_MAGIC,
    .lift_zero_voltage_mv = 2203,
    .encoder_zero_raw = 0,
    .enc_zero_raw = {0U, 0U, 0U},
    .enc_zero_valid = 0,
    .teeth_type = TEETH_UNSET,
    .is_calibrated = 0,
    .teeth_clear_action = TEETH_CLEAR_KEEP,
    .reserved = 0,
};

/* 内部存储服务实例 */
static storage_t g_sys_storage;

/* 本模块独立的 Flash 实例（禁止与其他模块共用，否则扇区地址互相覆盖） */
static flash_t   s_flash;

/* ============================================================
 * 初始化
 * ============================================================ */
int sys_param_init(void)
{
    const storage_config_t *cfg = storage_find_config("sys_param");
    rt_err_t ret;
    uint32_t blk_size;

    SM_ASSERT_PTR(cfg);

    blk_size = sizeof(sys_param_t) + 8;  /* 数据 + Magic(4) + CRC(4) */

    ret = flash_init(&s_flash,
                     cfg->sector_addr,
                     cfg->sector_size,
                     blk_size,
                     cfg->magic);
    if (ret != RT_EOK) return ret;

    ret = storage_init(&g_sys_storage,
                       &s_flash,
                       cfg->name,
                       &g_sys_params,
                       sizeof(sys_param_t),
                       cfg->magic,
                       cfg->cooldown_ms);
    if (ret != RT_EOK) return ret;

    ret = storage_load(&g_sys_storage);
    if (ret != RT_EOK) {
        /* Flash 内容无效（magic/CRC 不符）不阻断启动：
         * 保留 g_sys_params 的静态默认值，并清除基线使下次 save 必然写入。
         * 注意 last_data_buf 必须是"与内存值不同"的内容，否则会被值去重跳过。
         * rec_magic 非零保证了这个"不同" —— 见 sys_param.h 说明。 */
        LOG_WARN("[sys_param] WARN: flash invalid (%d), using defaults\n", ret);
        g_sys_storage.is_loaded = 1U;
        memset(g_sys_storage.last_data_buf, 0x00, sizeof(sys_param_t));
    }
    else if (storage_is_empty(&g_sys_storage) != 0U) {
        /* ============================================================
         * 【甲-1 / DES-STORAGE-FIX-V1.0】空分区：写入静态默认值
         * ------------------------------------------------------------
         * storage_load() 空 Flash 时返回 RT_EOK 且不填充 data_ptr，
         * 上面的 ret!=RT_EOK 分支抓不到这种情况，必须单独判断。
         *
         * 必须先清 last_data_buf：storage_load 空分区分支已执行
         *   memcpy(last_data_buf, data_ptr, size)      (storage.c:121)
         * 而 storage_save 第一步是值去重
         *   if (memcmp(data_ptr, last_data_buf, size) == 0) return;  (storage.c:185)
         * 不清则 save 被静默跳过，默认值永远写不进去。
         * 清成 0 后能"不同"，靠的是 rec_magic 非零（2026-11-03 新增）。
         *
         * 写入内容 = g_sys_params 的静态初值，
         * 它在 storage_load 之前就已就位，正是我们要的"默认配置"。 */
        rt_kprintf("[sys_param] empty partition, writing defaults...\n");
        memset(g_sys_storage.last_data_buf, 0x00, sizeof(sys_param_t));

        if (sys_param_save() == RT_EOK) {
            LOG_OK("[sys_param] defaults written to flash\n");
        } else {
            LOG_ERR("[sys_param] WARN: default save FAILED\n");
        }
    }
    else if (g_sys_params.rec_magic != SYS_PARAM_REC_MAGIC) {
        /* ---- 记录损坏：CRC 过了但内容不是本版本写的（旧格式/位翻转）----
         * 回默认值并落盘，避免把脏数据当成有效零点/标定标志使用。
         * 这一点对本模块尤其重要：错误的 enc_zero_raw 会让游标解算偏移。 */
        LOG_WARN("[sys_param] bad rec_magic 0x%08X (expect 0x%08X), reset to default\n",
                   (unsigned)g_sys_params.rec_magic,
                   (unsigned)SYS_PARAM_REC_MAGIC);

        /* 恢复静态默认值：逐个字段复位，保持与 g_sys_params 初值一致 */
        g_sys_params.rec_magic              = SYS_PARAM_REC_MAGIC;
        g_sys_params.lift_zero_voltage_mv  = 2203U;
        g_sys_params.lift_encoder_zero_raw = 0;
        g_sys_params.encoder_zero_raw       = 0;
        g_sys_params.enc_zero_raw[0]        = 0U;
        g_sys_params.enc_zero_raw[1]        = 0U;
        g_sys_params.enc_zero_raw[2]        = 0U;
        g_sys_params.enc_zero_valid         = 0U;
        g_sys_params.teeth_type             = TEETH_UNSET;
        g_sys_params.is_calibrated          = 0U;
        g_sys_params.teeth_clear_action     = TEETH_CLEAR_KEEP;
        g_sys_params.reserved               = 0U;

        memset(g_sys_storage.last_data_buf, 0x00, sizeof(sys_param_t));
        if (sys_param_save() == RT_EOK) {
            LOG_OK("[sys_param] reset written to flash\n");
        } else {
            LOG_ERR("[sys_param] WARN: reset save FAILED\n");
        }
    }

    rt_kprintf("[sys_param] Loaded: zero=%dmV, teeth=%d, calib=%d\n\n",
               g_sys_params.lift_zero_voltage_mv,
               g_sys_params.teeth_type,
               g_sys_params.is_calibrated);

    return RT_EOK;
}

int sys_param_save(void)
{
    return storage_save(&g_sys_storage);
}

int sys_param_load(void)
{
    return storage_load(&g_sys_storage);
}

sys_param_t* sys_param_get(void)
{
    return &g_sys_params;
}

void sys_param_get_stats(uint32_t *write_count, uint32_t *skip_count)
{
    storage_get_stats(&g_sys_storage, write_count, skip_count);
}

/* ============================================================
 * 便捷接口
 * ============================================================
 * 所有 setter 先做值比较：值未变化直接返回，不触发 Flash 擦写。
 * 这是防止周期重发（HMI 每 100ms 下发同一机型）写穿 Flash 的
 * 第一道防线；storage_save() 内部另有整块 memcmp 作为兜底。
 * ============================================================ */
void sys_param_set_teeth(uint8_t type)
{
    if (g_sys_params.teeth_type == type) {
        return;                         /* 值去重：不写 Flash */
    }
    g_sys_params.teeth_type = type;
    sys_param_save();
}

void sys_param_set_lift_zero_voltage(uint16_t mv)
{
    if (g_sys_params.lift_zero_voltage_mv == mv) {
        return;
    }
    g_sys_params.lift_zero_voltage_mv = mv;
    sys_param_save();
}

void sys_param_set_calibrated(uint8_t calibrated)
{
    if (g_sys_params.is_calibrated == calibrated) {
        return;
    }
    g_sys_params.is_calibrated = calibrated;
    sys_param_save();
}

void sys_param_set_teeth_clear_action(uint8_t action)
{
    if (g_sys_params.teeth_clear_action == action) {
        return;
    }
    g_sys_params.teeth_clear_action = action;
    sys_param_save();
}

/**
 * @brief 清除回转零点并清校准标志（一次 Flash 写完成）
 * @note  【C-08】切换机械 profile 时必须调用。
 *        旧 profile 的 encoder_zero_raw 在新齿数模型下没有物理意义，
 *        若只清零点不清标志，status_tx 会继续上报"已校准"，
 *        上层据此放行运动 = 用脏零点驱动电机。
 *
 *        本函数把"清零点 + 清标志"合成一次 save()：
 *        分成两次会多一次擦写，且中间掉电会出现
 *        "零点已清但标志仍为已校准"的最坏组合（无法自愈）。
 */
void sys_param_clear_calibration(void)
{
    uint8_t changed = 0U;

    if (g_sys_params.encoder_zero_raw != 0) {
        g_sys_params.encoder_zero_raw = 0;
        changed = 1U;
    }
    if (g_sys_params.is_calibrated != 0U) {
        g_sys_params.is_calibrated = 0U;
        changed = 1U;
    }

    /* 举升零点一并清：切 profile 属于机械基准变更，两轴基准都需重标 */
    if (g_sys_params.lift_zero_voltage_mv != 0U) {
        g_sys_params.lift_zero_voltage_mv = 0U;
        changed = 1U;
    }

    if (changed != 0U) {
        sys_param_save();
        LOG_WARN("[SysParam] calibration CLEARED (zero+flag+mv)\n");
    }
}

/* ============================================================
 * 三颗编码器零点校准（方案 A，2026-11-02 新增）
 * ============================================================
 * 业务语义：回转臂处于**机械零位**时，记录三颗编码器各自的当前角度，
 *           作为后续解算的基准。每次上电解算前统一减掉。
 *
 * 与旧字段 encoder_zero_raw 的关系：
 *   旧字段只有一颗，且当前指向 motor->current_position 那条断链，
 *   实际无效。新逻辑全部走 enc_zero_raw[3]，旧字段保留仅为兼容
 *   已落盘的历史数据（不清零，避免破坏 layout）。
 * ============================================================ */

/**
 * @brief 保存三颗编码器零点
 */
int sys_param_set_enc_zero(uint16_t z0, uint16_t z1, uint16_t z2)
{
    uint8_t changed = 0U;

    /* 值去重：三颗都没变且标志已置位 → 不写 flash */
    if ((g_sys_params.enc_zero_raw[0] != z0) ||
        (g_sys_params.enc_zero_raw[1] != z1) ||
        (g_sys_params.enc_zero_raw[2] != z2)) {
        g_sys_params.enc_zero_raw[0] = z0;
        g_sys_params.enc_zero_raw[1] = z1;
        g_sys_params.enc_zero_raw[2] = z2;
        changed = 1U;
    }

    if (g_sys_params.enc_zero_valid != 1U) {
        g_sys_params.enc_zero_valid = 1U;
        changed = 1U;
    }
    /* 校准成功即置整体校准标志，status_tx 据此上报"已校准" */
    if (g_sys_params.is_calibrated != 1U) {
        g_sys_params.is_calibrated = 1U;
        changed = 1U;
    }

    if (changed == 0U) {
        rt_kprintf("[SysParam] enc zero unchanged, skip flash write\n");
        return 0;
    }

    if (sys_param_save() != 0) {
        LOG_ERR("[SysParam] FAIL: enc zero save failed\n");
        return -1;
    }

    LOG_OK("[SysParam] enc zero SAVED: Z0=%u Z1=%u Z2=%u (valid=1)\n",
               (unsigned)z0, (unsigned)z1, (unsigned)z2);
    return 0;
}

/**
 * @brief 读取三颗编码器零点
 */
uint8_t sys_param_get_enc_zero(uint16_t out[3])
{
    if (g_sys_params.enc_zero_valid != 1U) {
        if (out != RT_NULL) {
            out[0] = 0U; out[1] = 0U; out[2] = 0U;
        }
        return 0U;
    }
    if (out != RT_NULL) {
        out[0] = g_sys_params.enc_zero_raw[0];
        out[1] = g_sys_params.enc_zero_raw[1];
        out[2] = g_sys_params.enc_zero_raw[2];
    }
    return 1U;
}

/**
 * @brief 清除三颗编码器零点并复位校准标记
 *
 * 用途：重新校准前必须先清，否则上层会认为"已校准"而放行运动。
 * 清除后 enc_zero_valid=0、is_calibrated=0，系统回到"未校准"状态，
 * 允许再次执行 t_enc_cal。
 *
 * 合成一次 save()：分两次会多一次擦写，且中间掉电会留下
 * "零点已清但标志仍为已校准"的最坏组合（无法自愈）。
 */
void sys_param_clear_enc_zero(void)
{
    uint8_t changed = 0U;

    if ((g_sys_params.enc_zero_raw[0] != 0U) ||
        (g_sys_params.enc_zero_raw[1] != 0U) ||
        (g_sys_params.enc_zero_raw[2] != 0U)) {
        g_sys_params.enc_zero_raw[0] = 0U;
        g_sys_params.enc_zero_raw[1] = 0U;
        g_sys_params.enc_zero_raw[2] = 0U;
        changed = 1U;
    }
    if (g_sys_params.enc_zero_valid != 0U) {
        g_sys_params.enc_zero_valid = 0U;
        changed = 1U;
    }
    if (g_sys_params.is_calibrated != 0U) {
        g_sys_params.is_calibrated = 0U;
        changed = 1U;
    }

    if (changed != 0U) {
        sys_param_save();
        rt_kprintf("[SysParam] enc zero CLEARED, calibration flag reset\n");
    } else {
        rt_kprintf("[SysParam] enc zero already clear\n");
    }
}
