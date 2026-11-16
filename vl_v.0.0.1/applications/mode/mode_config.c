/**
 * @file    mode_config.c
 * @brief   模式配置持久化实现（基于通用 storage 模块）
 */

#include "mode_config.h"

/* 【2026-11-10 彩标 P2】配置装载：失败=ERR/降级=WARN/成功=OK */
#include "../log/log_color.h"
#include "../storage/storage.h"
#include "../storage/storage_config.h"
#include "framework/state_engine/sm_assert.h"
#include <string.h>
#include <rtthread.h>

/* ============================================================
 * 内部存储服务实例
 * ============================================================ */
static storage_t g_mode_storage;
/* 本模块独立的 Flash 实例（禁止与其他模块共用） */
static flash_t   s_flash;
static uint8_t   g_mode_storage_initialized = 0;

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
 * 内部：确保 storage 初始化
 * ============================================================ */
static int mode_storage_ensure_init(void)
{
    if (g_mode_storage_initialized) {
        return 0;
    }

    const storage_config_t *cfg = storage_find_config("mode_config");
    if (cfg == NULL) {
        LOG_ERR("[ModeCfg] config 'mode_config' not found!\n");
        return -1;
    }

    uint32_t blk_size = sizeof(mode_config_t) + 8;

    /* 初始化 Flash 分区 */
    rt_err_t ret = flash_init(&s_flash,
                              cfg->sector_addr,
                              cfg->sector_size,
                              blk_size,
                              cfg->magic);
    if (ret != RT_EOK) {
        LOG_ERR("[ModeCfg] flash_init failed (%d)\n", ret);
        return -1;
    }

    /* 初始化存储服务
     * 注：data_ptr 此处仅为满足 storage_init 的非空断言，
     *     实际在 mode_config_load/save 中会重新绑定到调用方提供的对象。 */
    ret = storage_init(&g_mode_storage,
                       &s_flash,
                       cfg->name,
                       &g_mode_storage,   /* 占位，load/save 时替换 */
                       sizeof(mode_config_t),
                       cfg->magic,
                       cfg->cooldown_ms);
    if (ret != RT_EOK) {
        LOG_ERR("[ModeCfg] storage_init failed (%d)\n", ret);
        return -1;
    }

    g_mode_storage_initialized = 1;
    return 0;
}

/* ============================================================
 * 公开 API
 * ============================================================ */
int mode_config_load(mode_config_t *config)
{
    SM_ASSERT_PTR(config);

    if (mode_storage_ensure_init() != 0) {
        mode_config_default(config);
        return -1;
    }

    /* 绑定到目标数据结构 */
    g_mode_storage.data_ptr = config;

    /* 从 Flash 加载 */
    rt_err_t ret = storage_load(&g_mode_storage);
    if (ret != RT_EOK) {
        LOG_WARN("[ModeCfg] storage_load failed, using default\n");
        mode_config_default(config);
        /* 清基线，使后续 save 能真正写入（否则被值去重跳过） */
        memset(g_mode_storage.last_data_buf, 0x00, sizeof(mode_config_t));
        return -1;
    }

    /* ============================================================
     * 【甲-1 / DES-STORAGE-FIX-V1.0】空分区：写入默认配置
     * ------------------------------------------------------------
     * 【替代原"全零记录"启发式（原 :131-137）】
     *   原启发式是当年因 storage_load 在空 Flash 时不填充 data_ptr
     *   才打的补丁：全零缓冲被当成有效配置通过校验，导致
     *   default_mode=0(FACTORY)、allowed_modes=0x00。
     *   现改用 storage_is_empty() 显式判断分区是否为空，更可靠：
     *   "全零"也可能是用户真的这么配置的，启发式会误判。
     *
     * 为什么在这里处理而不是在 mode_storage_ensure_init()：
     *   本模块的 data_ptr 是【运行时绑定】的（上面 g_mode_storage.data_ptr
     *   = config），初始化阶段 data_ptr 只是指向自己的占位符。
     *   且默认值由 mode_config_default() 生成，必须在知道
     *   "要填到哪个对象"之后才能写。
     *
     * 必须清 last_data_buf：storage_load 空分区分支执行了
     *   memcpy(last_data_buf, data_ptr, size)      (storage.c:121)
     * 不清则 storage_save 的值去重会静默跳过（storage.c:185）。
     * ============================================================ */
    if (storage_is_empty(&g_mode_storage) != 0U) {
        rt_kprintf("[ModeCfg] empty partition, writing defaults...\n");
        mode_config_default(config);
        memset(g_mode_storage.last_data_buf, 0x00, sizeof(mode_config_t));

        if (storage_save(&g_mode_storage) == RT_EOK) {
            LOG_OK("[ModeCfg] defaults written to flash\n");
        } else {
            LOG_ERR("[ModeCfg] WARN: default save FAILED\n");
        }
        /* 默认值已就位且合法，视为加载成功 */
        rt_kprintf("[ModeCfg] Loaded: default=%d, allowed=0x%02X, timeout=%dms\n",
                   config->default_mode,
                   config->allowed_modes,
                   config->test_timeout_ms);
        return 0;
    }

    /* ============================================================
     * 【2026-11-03 新增】记录完整性校验：rec_magic
     * ------------------------------------------------------------
     * 走到这里说明分区非空且 CRC 通过，但内容未必是本版本写的
     * （可能是旧固件格式，或存储位翻转恰好 CRC 仍匹配）。
     * magic 不符 => 整块不可信 => 回默认并重写。
     *
     * 为何放在 storage_is_empty 之后：
     *   空分区时 config 内容尚未定义，magic 校验无意义；
     *   上面的分支已处理并返回。 */
    if (config->rec_magic != MODE_CONFIG_REC_MAGIC) {
        LOG_WARN("[ModeCfg] bad rec_magic 0x%08X (expect 0x%08X), reset to default\n",
                   (unsigned)config->rec_magic,
                   (unsigned)MODE_CONFIG_REC_MAGIC);

        mode_config_default(config);
        memset(g_mode_storage.last_data_buf, 0x00, sizeof(mode_config_t));

        if (storage_save(&g_mode_storage) == RT_EOK) {
            rt_kprintf("[ModeCfg] reset written to flash\n");
        } else {
            LOG_ERR("[ModeCfg] WARN: reset save FAILED\n");
        }
        return 0;
    }

    /* 校验配置合法性
     * 【P0 修正 2026-10-27】原实现只校验上界 default_mode >= SYS_MODE_MAX，
     * 而 storage_load() 在 flash 无有效记录时返回 RT_EOK 却不填充 data_ptr
     * （storage.c:107-113），全零缓冲被当成有效配置通过校验：
     *   default_mode=0(FACTORY)、allowed_modes=0x00、timeout=0
     * ⇒ mode_init 取到 0，实际进 FACTORY；且 allowed_modes=0 会让所有模式不可用。
     * 现补两条判定：
     *   ① default_mode 必须是枚举内的有效值（0 是合法枚举，不能靠下界拦）
     *   ② allowed_modes 至少包含默认模式对应位，否则配置自相矛盾
     * 注：原"全零记录"判定已由上面的 storage_is_empty() 取代（2026-11-03）。 */
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

    if (mode_storage_ensure_init() != 0) {
        return -1;
    }

    /* 绑定到源数据结构 */
    g_mode_storage.data_ptr = (void*)config;

    /* 保存到 Flash */
    rt_err_t ret = storage_save(&g_mode_storage);
    if (ret != RT_EOK) {
        LOG_ERR("[ModeCfg] storage_save failed (%d)\n", ret);
        return -1;
    }

    LOG_OK("[ModeCfg] Saved.\n");
    return 0;
}
