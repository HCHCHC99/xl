/**
 * @file    ut_can_config.c
 * @brief   CAN 设备配置表实例
 * @note    用户在此文件中配置需要启用的 CAN 设备
 */

#include "ut_can_config.h"
#include "framework/state_engine/sm_assert.h"

/* ============================================================
 * 配置表实例
 * ============================================================ */
const can_channel_config_t g_can_channels[] = {
    /* 主通道：卸粮筒控制 CAN */
    {
        .name       = "can1",
        .enabled    = 1,
        .is_primary = 1
    },
    /* 备用/诊断通道（当前禁用） */
    {
        .name       = "can2",
        .enabled    = 0,
        .is_primary = 0
    },
};

/* 计算配置表条目数 */
const uint8_t g_can_channel_count = sizeof(g_can_channels) / sizeof(can_channel_config_t);

/* ============================================================
 * 配置表验证函数
 * ============================================================ */
void ut_can_config_verify(void)
{
    SM_ASSERT_PTR(g_can_channels);
    SM_ASSERT_MSG(g_can_channel_count > 0, "CAN config table empty");
    SM_ASSERT_MSG(g_can_channel_count <= CAN_CHANNEL_MAX, "CAN config table overflow");

    uint8_t primary_count = 0;

    for (uint8_t i = 0; i < g_can_channel_count; i++) {
        SM_ASSERT_MSG(g_can_channels[i].name != NULL, "Channel name is NULL");

        if (g_can_channels[i].enabled && g_can_channels[i].is_primary) {
            primary_count++;
        }
    }

    SM_ASSERT_MSG(primary_count <= 1, "Multiple primary channels defined");
}
