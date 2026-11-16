/**
 * @file    lift_adapter.c
 * @brief   举升角度转换实现
 */

#include "lift_adapter.h"
#include "framework/state_engine/sm_assert.h"
#include <string.h>


/* ============================================================
 * 转换实现
 * ============================================================ */

/**
 * @brief 原始值 → 角度（0.1°）
 */
int32_t lift_adapter_to_angle_x10(int32_t raw, const lift_adapter_cfg_t *cfg)
{
    int32_t delta;
    int32_t angle_x10;

    SM_ASSERT_PTR(cfg);
    if (cfg->scale_den == 0) return 0;

    /* 计算偏移量 */
    delta = raw - cfg->zero_raw;

    /* 定点转换 */
    angle_x10 = (delta * cfg->scale_num) / cfg->scale_den;

    /* 限幅 */
    if (angle_x10 < cfg->min_angle_x10) angle_x10 = cfg->min_angle_x10;
    if (angle_x10 > cfg->max_angle_x10) angle_x10 = cfg->max_angle_x10;

    return angle_x10;
}

/**
 * @brief 角度（0.1°）→ 原始值
 */
int32_t lift_adapter_to_raw(int32_t angle_x10, const lift_adapter_cfg_t *cfg)
{
    SM_ASSERT_PTR(cfg);
    if (cfg->scale_num == 0) return cfg->zero_raw;
    return cfg->zero_raw + (angle_x10 * cfg->scale_den) / cfg->scale_num;
}

/* ============================================================
 * 预设配置
 * ============================================================ */

/**
 * @brief 配置电压型适配器
 */
void lift_adapter_config_voltage(lift_adapter_cfg_t *cfg, int32_t zero_mv)
{
    SM_ASSERT_PTR(cfg);
    memset(cfg, 0, sizeof(lift_adapter_cfg_t));

    cfg->zero_raw      = zero_mv;
    cfg->scale_num     = LIFT_ADAPTER_SCALE_NUM;
    cfg->scale_den     = LIFT_ADAPTER_SCALE_DEN;
    cfg->min_angle_x10 = -300;      /* -30° */
    cfg->max_angle_x10 = 430;       /* +43° */
}

/**
 * @brief 配置电流型适配器
 */
void lift_adapter_config_current(lift_adapter_cfg_t *cfg,
                                   int32_t zero_ma10,
                                   int32_t span_ma10,
                                   int32_t span_deg)
{
    SM_ASSERT_PTR(cfg);
    SM_ASSERT_MSG(span_ma10 != 0, "span_ma10 must not be zero");

    memset(cfg, 0, sizeof(lift_adapter_cfg_t));

    cfg->zero_raw      = zero_ma10;
    cfg->scale_num     = span_deg * 10;
    cfg->scale_den     = span_ma10;
    cfg->min_angle_x10 = -300;
    cfg->max_angle_x10 = 430;
}

/**
 * @brief 配置角度型适配器
 */
void lift_adapter_config_angle(lift_adapter_cfg_t *cfg, int32_t zero_x10)
{
    SM_ASSERT_PTR(cfg);
    memset(cfg, 0, sizeof(lift_adapter_cfg_t));

    cfg->zero_raw      = zero_x10;
    cfg->scale_num     = 1;
    cfg->scale_den     = 1;
    cfg->min_angle_x10 = -300;
    cfg->max_angle_x10 = 430;
}
