/**
 * @file ut_can_send.c
 * @brief CAN 发送模块实现
 *
 * 核心逻辑：
 *   1. 初始化时遍历配置表，查找并打开所有启用的 CAN 设备
 *   2. 记录主通道索引（is_primary=1）
 *   3. 发送时通过主通道或指定索引发送
 *   4. 主通道不可用时直接报错（不降级）
 */

#include "ut_can_send.h"
#include "ut_can_config.h"
#include "framework/state_engine/sm_assert.h"
#include "ipc_center.h"
#include <rtthread.h>
#include <string.h>

/* -------------------- 静态资源 -------------------- */
can_device_entry_t g_devices[CAN_CHANNEL_MAX];
static uint8_t g_device_count = 0;
static int8_t g_primary_index = -1;      /* 主通道索引，-1 表示未找到 */

/* ============================================================
 * 内部函数：初始化设备表
 * ============================================================ */
static void can_devices_init(void)
{
    const can_channel_config_t *cfg;
    rt_device_t dev;
    rt_err_t res;

    g_device_count = 0;
    g_primary_index = -1;

    SM_ASSERT_PTR(g_can_channels);
    SM_ASSERT_MSG(g_can_channel_count > 0, "CAN config table empty");

    for (uint8_t i = 0; i < g_can_channel_count && i < CAN_CHANNEL_MAX; i++) {
        cfg = &g_can_channels[i];

        if (!cfg->enabled) {
            continue;   /* 跳过禁用的通道 */
        }

        SM_ASSERT_PTR(cfg->name);

        /* 查找设备 */
        dev = rt_device_find(cfg->name);
        if (dev == RT_NULL) {
            rt_kprintf("[CAN] Device '%s' not found, skip.\n", cfg->name);
            continue;
        }

        /* 设置 CAN 通信的波特率为 250kbit/s */
        res = rt_device_control(dev, RT_CAN_CMD_SET_BAUD, (void *)CAN250kBaud);
        if (res != RT_EOK) {
            rt_kprintf("[CAN] Device '%s' set baud failed, skip.\n", cfg->name);
        }

        /* 打开设备 */
        if (rt_device_open(dev, RT_DEVICE_FLAG_INT_TX | RT_DEVICE_FLAG_INT_RX) != RT_EOK) {
            rt_kprintf("[CAN] Device '%s' open failed, skip.\n", cfg->name);
            continue;
        }

        /* 设置 CAN 的工作模式为正常工作模式 */
        res = rt_device_control(dev, RT_CAN_CMD_SET_MODE, (void *)RT_CAN_MODE_NORMAL);
        RT_ASSERT(res == RT_EOK);

        /*滤波器：全收 */
        struct rt_can_filter_item items[1] = {
            RT_CAN_FILTER_ITEM_INIT(0x00000000, 1, 0, 0, 0x00000000, RT_NULL, RT_NULL),
        };
        struct rt_can_filter_config filter_cfg = { 1, 1, items };
        rt_device_control(dev, RT_CAN_CMD_SET_FILTER, &filter_cfg);

        /* 存入设备表 */
        g_devices[g_device_count].dev = dev;
        g_devices[g_device_count].is_ready = 1;
        g_devices[g_device_count].is_primary = cfg->is_primary;

        /* 记录主通道索引 */
        if (cfg->is_primary) {
            g_primary_index = g_device_count;
        }

        rt_kprintf("[CAN] Device '%s' initialized (index=%d, primary=%d)\n",
                   cfg->name, g_device_count, cfg->is_primary);

        g_device_count++;
    }

    /* 检查主通道是否就绪：如果配置表中标记了主通道，但未找到则报错 */
    if (g_primary_index == -1) {
        /* 检查是否配置表中存在 is_primary=1 的条目 */
        uint8_t has_primary_in_config = 0;
        for (uint8_t i = 0; i < g_can_channel_count && i < CAN_CHANNEL_MAX; i++) {
            if (g_can_channels[i].enabled && g_can_channels[i].is_primary) {
                has_primary_in_config = 1;
                break;
            }
        }
        if (has_primary_in_config) {
            rt_kprintf("[ERROR] CAN primary device not available!\n");
            /* 此处可触发断言或系统复位，根据项目需求决定 */
        }
    }
}

/* -------------------- 内部函数：获取主通道设备句柄 -------------------- */
static rt_device_t get_primary_device(void)
{
    if (g_primary_index < 0 || g_primary_index >= g_device_count) {
        return RT_NULL;
    }
    if (!g_devices[g_primary_index].is_ready) {
        return RT_NULL;
    }
    return g_devices[g_primary_index].dev;
}

/* -------------------- 内部函数：获取指定索引的设备句柄 -------------------- */
static rt_device_t get_device_by_index(uint8_t idx)
{
    if (idx >= g_device_count) {
        return RT_NULL;
    }
    if (!g_devices[idx].is_ready) {
        return RT_NULL;
    }
    return g_devices[idx].dev;
}

/* -------------------- 内部函数：核心发送逻辑 -------------------- */
static int can_send_core(rt_device_t dev, uint32_t id, const uint8_t *data, uint8_t len)
{
    struct rt_can_msg msg;

    /* 【2026-10-27 修复 · D-58】原为 SM_ASSERT_PTR / SM_ASSERT_RANGE，
     * 改为直接返回错误码。原因：本函数位于 can_tx 线程的**热路径**，
     * 每次发送都要经过。而 SM_ASSERT_MSG 在表达式为假时会调用
     * rt_assert_handler → rt_kprintf → rt_vsnprintf，实测该链需要：
     *
     *   rt_assert_handler(32) → rt_kprintf(32) → rt_vsnprintf(80)
     *     → print_number(72) / skip_atoi(24) ...
     *   ≈ 600 B（含 _vfiprintf_r 变体时 872 B）
     *
     * 即：为了「万一参数非法时能打印」，正常发送路径被迫预留 600 B 栈。
     * 实测 can_tx 正常路径仅需 240 B、含打印路径需 872 B —— 72% 的栈
     * 预算花在一个正常运行时永不发生的分支上。
     *
     * 参数校验的正确位置是调用方入口（一次性），运行期只返回错误码并
     * 由上层计数（can_tx_poll 已有 s_err[] 统计）。这样：
     *   - 非法参数不再让整机卡死在 while(level) 死循环
     *   - can_tx 栈需求从 872 B 降到约 350 B
     */
    if (dev == RT_NULL || data == RT_NULL) {
        return -1;
    }
    if (len > 8U) {
        len = 8U;       /* 调用方传超长时截断，不中断发送 */
    }

    msg.id = id;
    msg.ide = RT_CAN_EXTID;      /* 0x18FFxxxx 均为扩展帧 */
    msg.rtr = RT_CAN_DTR;        /* 数据帧（非远程帧） */
    msg.len = len;
    memcpy(msg.data, data, len);

    if (rt_device_write(dev, 0, &msg, sizeof(msg)) != sizeof(msg)) {
        return -2;
    }
    return 0;
}

/* ============================================================
 * 公开 API
 * ============================================================ */

/**
 * @brief 初始化 CAN 发送模块
 */
int can_send_init(void)
{
    can_devices_init();
    return 0;
}

/**
 * @brief 通过主通道发送 CAN 帧      (考虑 互斥量)
 */
int can_send(uint32_t id, const uint8_t *data, uint8_t len)
{
    rt_device_t dev = get_primary_device();
    if (dev == RT_NULL) {
        return -1;
    }
    return can_send_core(dev, id, data, len);
}

/**
 * @brief 通过指定索引的通道发送 CAN 帧   （考虑互斥量）
 */
int can_send_by_channel(uint8_t ch_idx, uint32_t id, uint8_t *data, uint8_t len)
{
    rt_device_t dev = get_device_by_index(ch_idx);
    if (dev == RT_NULL) {
        return -1;
    }
    return can_send_core(dev, id, data, len);
}
