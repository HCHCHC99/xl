/**
 * @file    storage_init.c
 * @brief   存储模块统一初始化入口
 * @note    本文件提供两种初始化方式，二选一，不可同时启用：
 *
 *          【方式A】使用 RT-Thread 自动初始化（默认启用）
 *            - 通过 INIT_APP_EXPORT 宏注册，无需在 main.c 中显式调用
 *            - 执行顺序由链接器决定，可能与预期不同
 *            - 适合：不依赖其他模块初始化顺序的场景
 *
 *          【方式B】在 main.c 中显式调用 storage_all_init()
 *            - 需注释掉下方的 INIT_APP_EXPORT(storage_all_init)
 *            - 执行顺序明确可控，便于调试
 *            - 适合：对初始化顺序有严格要求的场景
 *
 *          【方式C】使用 INIT_BOARD_EXPORT（更早执行）
 *            - 在 INIT_APP_EXPORT 之前执行，优先级更高
 *            - 适合：存储初始化必须在其他模块之前完成的场景
 *            - 需注释掉方式A的 INIT_APP_EXPORT，取消注释方式C
 *
 *          ⚠️ 实际调试时，请根据初始化顺序需求自行确认使用哪一种。
 *             如需在 main.c 中显式调用，请注释掉下方的 INIT_APP_EXPORT。
 */

#include "../sys_param/sys_param.h"
#include "../device_info/device_info.h"
#include "../ota_shared/ota_shared.h"
#include "framework/state_engine/sm_assert.h"
#include <rtthread.h>

/**
 * @brief 存储模块统一初始化
 * @return RT_EOK 成功，其他失败
 * @note  该函数为公开接口，既可作为自动初始化的入口，
 *        也可在 main.c 中被显式调用
 */
int storage_all_init(void)
{
    rt_err_t ret;

    ret = sys_param_init();
    SM_ASSERT_MSG(ret == RT_EOK, "sys_param init failed");

    ret = device_info_init();
    SM_ASSERT_MSG(ret == RT_EOK, "device_info init failed");

    ret = ota_shared_init();
    SM_ASSERT_MSG(ret == RT_EOK, "ota_shared init failed");

    rt_kprintf("[Storage] All modules initialized.\n\n");
    return RT_EOK;
}

/* ============================================================
 * 初始化方式选择（三选一，不可同时启用）
 * ============================================================ */

/* ============================================================
 * 【方式A】使用 INIT_APP_EXPORT（默认启用）
 * ============================================================
 * 特点：
 *   - 自动初始化，无需在 main.c 中显式调用
 *   - 执行顺序由链接器决定，通常在 main() 之前
 *   - 若存储模块被其他模块依赖，顺序可能不可控
 *
 * 使用方式：
 *   - 保持下方宏启用
 *   - main.c 中无需调用 storage_all_init()
 */
//INIT_PREV_EXPORT(storage_all_init);

/* ============================================================
 * 【方式B】在 main.c 中显式调用（默认注释）
 * ============================================================
 * 特点：
 *   - 执行顺序明确可控，便于调试
 *   - 需手动在 main() 中调用 storage_all_init()
 *   - 适合对初始化顺序有严格要求的场景
 *
 * 使用方式：
 *   1. 注释掉上方【方式A】的 INIT_APP_EXPORT
 *   2. 在 main.c 中：
 *        storage_all_init();  // 需 #include "sys_param.h" 等
 *   3. 确保 storage_all_init() 在其他模块初始化之前调用
 */
// 注意：不使用该方式时，请注释掉上方【方式A】的 INIT_APP_EXPORT

/* ============================================================
 * 【方式C】使用 INIT_BOARD_EXPORT（默认注释）
 * ============================================================
 * 特点：
 *   - 在 INIT_APP_EXPORT 之前执行，优先级更高
 *   - 比 APP 级初始化更早，适合存储必须在早期就绪的场景
 *   - 在 RT-Thread 启动流程中，BOARD 级初始化早于 APP 级
 *
 * 使用方式：
 *   1. 注释掉上方【方式A】的 INIT_APP_EXPORT
 *   2. 取消下方 INIT_BOARD_EXPORT 的注释
 *   3. main.c 中无需调用 storage_all_init()
 */
// INIT_BOARD_EXPORT(storage_all_init);

/* ============================================================
 * 初始化顺序说明（供参考）
 * ============================================================
 * RT-Thread 自动初始化顺序（从早到晚）：
 *   1. INIT_BOARD_EXPORT   —— 板级初始化
 *   2. INIT_PREV_EXPORT    —— 早期驱动
 *   3. INIT_DEVICE_EXPORT  —— 设备驱动
 *   4. INIT_COMPONENT_EXPORT —— 组件
 *   5. INIT_ENV_EXPORT     —— 环境
 *   6. INIT_APP_EXPORT     —— 应用
 *
 * 若存储模块需要被其他 INIT_APP_EXPORT 模块使用，
 * 建议使用 INIT_BOARD_EXPORT 或 INIT_DEVICE_EXPORT，
 * 或在 main.c 中显式调用，以确保顺序可控。
 * ============================================================ */
