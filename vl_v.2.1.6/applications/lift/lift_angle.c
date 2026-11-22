/**
 * @file    lift_angle.c
 * @brief   举升角对象实现
 */

#include "lift_angle.h"
#include "framework/state_engine/sm_assert.h"
#include <stddef.h>

/* ============================================================
 * 全局对象
 * ============================================================ */
lift_angle_t g_lift_angle;

///* ============================================================
// * 内部辅助函数：电压 → 角度 (0.1°精度)
// * 转换关系: 每 36.4mV 对应 1° → 乘以 10 转为 0.1°
// * ============================================================ */
//static int32_t calc_angle_from_voltage(uint16_t voltage_mv, uint16_t zero_mv)
//{
//    int32_t delta_mv = (int32_t)voltage_mv - (int32_t)zero_mv;
//    /* 使用定点运算: (delta_mv * 100) / 364 */
//    return (delta_mv * 100) / 364;
//}

/* ============================================================
 * 内部辅助函数：判断位置
 * ============================================================ */
static lift_position_t determine_position(lift_angle_t *angle)
{
    int32_t angle_x10 = angle->current_angle_x10;
    int32_t upper_bound = angle->upper_limit_x10 - angle->limit_tolerance_x10;
    int32_t lower_bound = angle->lower_limit_x10 + angle->limit_tolerance_x10;

    /* 判断上限位：角度 >= (+43° - 2°) */
    if (angle_x10 >= upper_bound) {
        return LIFT_POS_UPPER_LIMIT;
    }

    /* 判断下限位：角度 <= (-30° + 2°) */
    if (angle_x10 <= lower_bound) {
        return LIFT_POS_LOWER_LIMIT;
    }

    /* 中间位：在上下限之间（含零位） */
    return LIFT_POS_MIDDLE;
}

/* ============================================================
 * 内部辅助函数：更新缓存标志
 * ============================================================ */
static void update_position_flags(lift_angle_t *angle)
{
    angle->is_at_upper_limit = (angle->position == LIFT_POS_UPPER_LIMIT) ? 1 : 0;
    angle->is_at_lower_limit = (angle->position == LIFT_POS_LOWER_LIMIT) ? 1 : 0;
    angle->is_at_middle = (angle->position == LIFT_POS_MIDDLE) ? 1 : 0;
}

/* ============================================================
 * 内部辅助函数：处理位置变化 → 调用回调
 * ============================================================ */
static void handle_position_change(lift_angle_t *angle)
{
    /* 无变化，直接返回 */
    if (angle->last_position == angle->position) {
        return;
    }

    /* 调用回调函数（如果已注册） */
    if (angle->on_position_change != NULL) {
        angle->on_position_change(angle->last_position, angle->position);
    }

    /* 更新上次位置 */
    angle->last_position = angle->position;
}

/* ============================================================
 * 公开 API
 * ============================================================ */

void lift_angle_init(lift_angle_t *angle, lift_sensor_t *sensor)
{
    SM_ASSERT_PTR(angle);
    SM_ASSERT_PTR(sensor);

    /* 设置限位常量（符合 REQ001-FUN001） */
    angle->upper_limit_x10 = 430;          /* +43.0° */
    angle->lower_limit_x10 = -300;         /* -30.0° */
//    angle->middle_center_x10 = 0;          /* 0.0° */
    angle->limit_tolerance_x10 = 20;       /* ±2° */
//    angle->middle_tolerance_x10 = 5;       /* ±0.5° */

    /* 传感器绑定 */
    angle->sensor = sensor;

    /* 默认适配器配置 */
    lift_adapter_config_voltage(&angle->adapter, 2203);

    /* 初始化物理量 */
    angle->current_angle_x10 = 0;
    angle->current_raw_value = 0;

    /* 初始化位置状态 */
    angle->position = LIFT_POS_UNKNOWN;
    angle->last_position = LIFT_POS_UNKNOWN;
    angle->is_at_upper_limit = 0;
    angle->is_at_lower_limit = 0;
    angle->is_at_middle = 0;

    /* 初始化依赖注入 */
    angle->on_position_change = NULL;

    /* 标记已初始化 */
    angle->is_initialized = 1;
}

void lift_angle_bind_sensor(lift_angle_t *angle, lift_sensor_t *sensor)
{
    SM_ASSERT_PTR(angle);
    SM_ASSERT_PTR(sensor);
    angle->sensor = sensor;
}

void lift_angle_set_adapter(lift_angle_t *angle, const lift_adapter_cfg_t *cfg)
{
    SM_ASSERT_PTR(angle);
    SM_ASSERT_PTR(cfg);
    angle->adapter = *cfg;
}

void lift_angle_update(lift_angle_t *angle)
{
    SM_ASSERT_PTR(angle);
    SM_ASSERT_MSG(angle->is_initialized, "Lift angle not initialized");
    SM_ASSERT_PTR(angle->sensor);

    /* 1. 更新传感器 */
    lift_sensor_update(angle->sensor);

    /* 2. 检查传感器故障 */
    if (lift_sensor_get_fault(angle->sensor)) {
        angle->last_position = angle->position;
        angle->position = LIFT_POS_UNKNOWN;
        update_position_flags(angle);
        return;
    }

    /* 3. 读取原始值 */
    angle->current_raw_value = lift_sensor_get_raw(angle->sensor);

    /* 4. 使用适配器转换 */
    angle->current_angle_x10 = lift_adapter_to_angle_x10(
                                    angle->current_raw_value,
                                    &angle->adapter);

    /* 5. 更新位置 */
    angle->last_position = angle->position;
    angle->position = determine_position(angle);
    update_position_flags(angle);

    /* 6. 处理位置变化 */
    handle_position_change(angle);
}

void lift_angle_calibrate(lift_angle_t *angle, int32_t  raw_zero)
{
    SM_ASSERT_PTR(angle);
    SM_ASSERT_MSG(angle->is_initialized, "Lift angle not initialized");

    angle->adapter.zero_raw = raw_zero;
    angle->is_calibrated = 1;

    /* 校准后立即更新位置 */
    lift_angle_update(angle);
}

/* ============================================================
 * 查询接口
 * ============================================================ */

lift_position_t lift_angle_get_position(const lift_angle_t *angle)
{
    SM_ASSERT_PTR(angle);
    SM_ASSERT_MSG(angle->is_initialized, "Lift angle not initialized");
    return angle->position;
}

uint8_t lift_angle_is_upper_limit(const lift_angle_t *angle)
{
    SM_ASSERT_PTR(angle);
    SM_ASSERT_MSG(angle->is_initialized, "Lift angle not initialized");
    return angle->is_at_upper_limit;
}

uint8_t lift_angle_is_lower_limit(const lift_angle_t *angle)
{
    SM_ASSERT_PTR(angle);
    SM_ASSERT_MSG(angle->is_initialized, "Lift angle not initialized");
    return angle->is_at_lower_limit;
}

uint8_t lift_angle_is_middle(const lift_angle_t *angle)
{
    SM_ASSERT_PTR(angle);
    SM_ASSERT_MSG(angle->is_initialized, "Lift angle not initialized");
    return angle->is_at_middle;
}

int32_t lift_angle_get_value(const lift_angle_t *angle)
{
    SM_ASSERT_PTR(angle);
    SM_ASSERT_MSG(angle->is_initialized, "Lift angle not initialized");
    return angle->current_angle_x10;
}

uint8_t lift_angle_is_safe(const lift_angle_t *angle)
{
    SM_ASSERT_PTR(angle);
    SM_ASSERT_MSG(angle->is_initialized, "Lift angle not initialized");
    return (angle->position == LIFT_POS_MIDDLE) && angle->is_calibrated;
}

lift_angle_t* lift_angle_get(void)
{
    return &g_lift_angle;
}
