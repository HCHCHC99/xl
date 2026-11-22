/**
 * @file    mode_config.c
 * @brief   模式配置持久化实现（基于通用 storage 模块）
 */

#include "mode_config.h"

/* 【2026-11-10 彩标 P2】配置装载：失败=ERR/降级=WARN/成功=OK */
#include "../log/log_color.h"
#include "../param/param_store.h"
#include "framework/state_engine/sm_assert.h"
#include <string.h>
#include <rtthread.h>

/* ============================================================
 * 默认配置
 * ============================================================ */
void mode_config_default(mode_config_t *config)
{
    SM_ASSERT_PTR(config);

    memset(config, 0, sizeof(mode_config_t));

    /* 【2026-11-03】非零记录魔数 —— 见 mode_config.h 的说明。
     * 必须在 memset 之后设置，否则会被清成 0，
     * 导致"空分区写默认值"重新陷入被值去重跳过的老问题。 */
    config->rec_magic = MODE_CONFIG_REC_MAGIC;

    /* 【2026-10-27 用户裁定】调试验证期默认上电进 MAINT（维护模式功能最全）。
     * 量产前须改回 SYS_MODE_WORK。此项与 MODE_BOOT_FORCE_MAINT 配合：
     *   MODE_BOOT_FORCE_MAINT=1 → 上电无条件进 MAINT（忽略 flash 配置）
     *   MODE_BOOT_FORCE_MAINT=0 → 按本函数/default_mode 配置决定
     * 见 mode_init.c。 */
    config->default_mode = SYS_MODE_MAINT;

    config->allowed_modes = MODE_ALLOW_FACTORY | MODE_ALLOW_TEST | MODE_ALLOW_WORK;
    config->factory_lock = 0;

    /* 【2026-10-27】MAINT 超时置 0 = 不降级。
     * mode_tick_check()（mode_actions.c:104-128）只在 MODE_STATE_TEST(=MAINT)
     * 下检查超时，超时后投 MODE_EVT_TEST_TIMEOUT 自动降到 WORK。
     * 默认进 MAINT 做验证时，若保留 30min 会在半小时后静默掉到 WORK，
     * 一键/校准权限突然消失，极易误判为 bug。验证期必须为 0。
     * 量产若需"维护模式自动退出"，再改回 30*60*1000。 */
    config->test_timeout_ms = 0;
}

/* ============================================================
 * 公开 API
 * ============================================================ */
int mode_config_load(mode_config_t *config)
{
    SM_ASSERT_PTR(config);

    if (param_store_mode_load(config) != 0) {
        LOG_WARN("[ModeCfg] no valid record, using default\n");
        mode_config_default(config);
        return -1;
    }

    if (config->default_mode >= SYS_MODE_MAX) {
        rt_kprintf("[ModeCfg] invalid default_mode(%u), reset\n",
                   (unsigned)config->default_mode);
        mode_config_default(config);
        return -1;
    }

    /* 默认模式必须在允许位掩码内 */
    {
        uint8_t need_bit = (uint8_t)(1U << config->default_mode);
        if ((config->allowed_modes & need_bit) == 0U) {
            rt_kprintf("[ModeCfg] default_mode(%u) not in allowed_modes(0x%02X), reset\n",
                       (unsigned)config->default_mode,
                       (unsigned)config->allowed_modes);
            mode_config_default(config);
            return -1;
        }
    }

    rt_kprintf("[ModeCfg] Loaded: default=%d, allowed=0x%02X, timeout=%dms\n",
               config->default_mode,
               config->allowed_modes,
               config->test_timeout_ms);

    return 0;
}

int mode_config_save(const mode_config_t *config)
{
    SM_ASSERT_PTR(config);

    if (param_store_mode_save(config) != 0) {
        LOG_ERR("[ModeCfg] save request failed\n");
        return -1;
    }

    LOG_OK("[ModeCfg] Saved.\n");
    return 0;
}
