/**
 * @file    lift_sensor.c
 * @brief   举升传感器实现（电压型 + 电流型）
 */

#include "lift_sensor.h"
#include "lift_sensor_config.h"
#include "../config/sys_config.h"      /* P5：阈值与确认时间走 SYSTEM 慢区 */
#include "framework/state_engine/sm_assert.h"
#include <string.h>
#include <rtthread.h>

/* ============================================================
 * 通用 API
 * ============================================================ */
/**
 * @brief 初始化传感器对象
 */
rt_err_t lift_sensor_init(lift_sensor_t *sensor,
                           const char *name,
                           lift_sensor_type_t type,
                           const lift_sensor_ops_t *ops,
                           void *data)
{
    SM_ASSERT_PTR(sensor);
    SM_ASSERT_PTR(name);
    SM_ASSERT_PTR(ops);

    memset(sensor, 0, sizeof(lift_sensor_t));
    sensor->name = name;
    sensor->type = type;
    sensor->ops = ops;
    sensor->data = data;

    if (ops->init) {
        rt_err_t ret = ops->init(sensor);
        if (ret != RT_EOK) {
            rt_kprintf("[LiftSensor] %s init failed (%d)\n", name, ret);
            return ret;
        }
    }

    sensor->is_initialized = 1;
    rt_kprintf("[LiftSensor] %s initialized (type=%d)\n", name, type);
    return RT_EOK;
}

void lift_sensor_update(lift_sensor_t *sensor)
{
    SM_ASSERT_PTR(sensor);
    if (!sensor->is_initialized) return;

    if (sensor->ops && sensor->ops->update) {
        sensor->ops->update(sensor);
    }
}

int32_t lift_sensor_get_raw(lift_sensor_t *sensor)
{
    SM_ASSERT_PTR(sensor);
    if (sensor->ops && sensor->ops->get_raw) {
        return sensor->ops->get_raw(sensor);
    }
    return sensor->raw_value;
}

uint8_t lift_sensor_get_fault(lift_sensor_t *sensor)
{
    SM_ASSERT_PTR(sensor);
    if (sensor->ops && sensor->ops->get_fault) {
        return sensor->ops->get_fault(sensor);
    }
    return sensor->fault;
}

/* ============================================================
 * 配置驱动初始化
 * ============================================================ */

/**
 * @brief 从配置表初始化传感器
 * @note  根据类型动态分配传感器私有数据
 */
rt_err_t lift_sensor_init_from_config(lift_sensor_t *sensor,
                                        const struct lift_sensor_config *config)
{
    SM_ASSERT_PTR(sensor);
    SM_ASSERT_PTR(config);

    memset(sensor, 0, sizeof(lift_sensor_t));
    sensor->name = config->name;
    sensor->type = config->type;

    switch (config->type) {
        /* ----- 电压型 ----- */
        case LIFT_SENSOR_TYPE_VOLTAGE: {
            lift_sensor_voltage_cfg_t *cfg = rt_malloc(sizeof(lift_sensor_voltage_cfg_t));
            if (cfg == NULL) return -RT_ENOMEM;

            /* 必须整体清零：新增的 exit 阈值 / 确认时间 / 计时器字段
             * 若留堆垃圾，voltage_init 的兜底判断会被随机值命中。 */
            memset(cfg, 0, sizeof(*cfg));
            cfg->read_adc_mv   = config->params.voltage.read_adc_mv;
            cfg->user_data     = config->params.voltage.user_data;
            cfg->fault_high_mv = config->params.voltage.fault_high_mv;
            cfg->fault_low_mv  = config->params.voltage.fault_low_mv;
            /* P5：表里的迟滞阈值/时间也要带过来，作为 CFG 读失败时的兜底 */
            cfg->fault_high_exit_mv = config->params.voltage.fault_high_exit_mv;
            cfg->fault_low_exit_mv  = config->params.voltage.fault_low_exit_mv;
            cfg->confirm_ms         = config->params.voltage.confirm_ms;
            cfg->recover_ms         = config->params.voltage.recover_ms;

            sensor->ops = &lift_sensor_voltage_ops;
            sensor->data = cfg;
            break;
        }

        /* ----- 电流型 ----- */
        case LIFT_SENSOR_TYPE_CURRENT: {
            lift_sensor_current_cfg_t *cfg = rt_malloc(sizeof(lift_sensor_current_cfg_t));
            if (cfg == NULL) return -RT_ENOMEM;

            cfg->read_adc_mv         = config->params.current.read_adc_mv;
            cfg->user_data           = config->params.current.user_data;
            cfg->sample_resistor_ohm = config->params.current.sample_resistor_ohm;
            cfg->fault_high_ma10     = config->params.current.fault_high_ma10;
            cfg->fault_low_ma10      = config->params.current.fault_low_ma10;

            sensor->ops = &lift_sensor_current_ops;
            sensor->data = cfg;
            break;
        }

        /* ----- 角度型 ----- */
        case LIFT_SENSOR_TYPE_ANGLE:
            sensor->ops = &lift_sensor_angle_ops;
            sensor->data = NULL;
            break;

        /* ----- CAN 型 ----- */
        case LIFT_SENSOR_TYPE_CAN:
            sensor->ops = &lift_sensor_can_ops;
            sensor->data = NULL;
            break;

        default:
            return -RT_EINVAL;
    }

    if (sensor->ops->init) {
        rt_err_t ret = sensor->ops->init(sensor);
        if (ret != RT_EOK) {
            if (sensor->data) rt_free(sensor->data);
            return ret;
        }
    }

    sensor->is_initialized = 1;
    rt_kprintf("[LiftSensor] %s initialized (type=%d, from config)\n",
               sensor->name, sensor->type);
    return RT_EOK;
}



static rt_err_t voltage_init(lift_sensor_t *self)
{
    lift_sensor_voltage_cfg_t *cfg = (lift_sensor_voltage_cfg_t*)self->data;
    SM_ASSERT_PTR(cfg);
    SM_ASSERT_PTR(cfg->read_adc_mv);

    if (cfg->fault_high_mv == 0U) cfg->fault_high_mv = 4700U;
    if (cfg->fault_low_mv  == 0U) cfg->fault_low_mv  = 300U;

    /* 【P5 / B-6】阈值与确认时间以 sys_config 为准（SYSTEM 慢区），
     * 表里的默认值只作为 CFG 未就绪时的兜底。
     * 只在 init 读一次：运行期改参数需重启（对应 CFG_F_REQ_RESTART）。 */
    {
        int32_t v;
        if (sys_config_get_i32(CFG_ID_LIFT_SENS_HI_ENTER_MV, &v) == 0) cfg->fault_high_mv      = (uint16_t)v;
        if (sys_config_get_i32(CFG_ID_LIFT_SENS_HI_EXIT_MV,  &v) == 0) cfg->fault_high_exit_mv = (uint16_t)v;
        if (sys_config_get_i32(CFG_ID_LIFT_SENS_LO_ENTER_MV, &v) == 0) cfg->fault_low_mv       = (uint16_t)v;
        if (sys_config_get_i32(CFG_ID_LIFT_SENS_LO_EXIT_MV,  &v) == 0) cfg->fault_low_exit_mv  = (uint16_t)v;
        if (sys_config_get_i32(CFG_ID_LIFT_SENS_CONFIRM_MS,  &v) == 0) cfg->confirm_ms          = (uint16_t)v;
        if (sys_config_get_i32(CFG_ID_LIFT_SENS_RECOVER_MS,  &v) == 0) cfg->recover_ms          = (uint16_t)v;
    }
    if (cfg->fault_high_exit_mv == 0U || cfg->fault_high_exit_mv >= cfg->fault_high_mv) {
        cfg->fault_high_exit_mv = (cfg->fault_high_mv > 400U)
                                  ? (uint16_t)(cfg->fault_high_mv - 400U) : 0U;
    }
    if (cfg->fault_low_exit_mv == 0U) {
        cfg->fault_low_exit_mv = (uint16_t)(cfg->fault_low_mv + 270U);
    }
    if (cfg->confirm_ms == 0U) cfg->confirm_ms = 2000U;
    if (cfg->recover_ms == 0U) cfg->recover_ms = 2000U;

    cfg->last_voltage_mv = 0U;
    cfg->timer_tick      = 0U;
    cfg->timer_active    = 0U;

    rt_kprintf("[LiftSensor] thresh: hi %u/%u lo %u/%u confirm=%ums recover=%ums\n",
               (unsigned)cfg->fault_high_mv, (unsigned)cfg->fault_high_exit_mv,
               (unsigned)cfg->fault_low_mv,  (unsigned)cfg->fault_low_exit_mv,
               (unsigned)cfg->confirm_ms,    (unsigned)cfg->recover_ms);
    return RT_EOK;
}

/* 结束计时并返回本次持续时长（ms） */
static uint32_t voltage_timer_stop(lift_sensor_voltage_cfg_t *cfg)
{
    uint32_t dt = 0U;

    if (cfg->timer_active != 0U) {
        dt = ((uint32_t)(rt_tick_get() - cfg->timer_tick) * 1000U) / RT_TICK_PER_SECOND;
        cfg->timer_active = 0U;
    }
    return dt;
}

static void voltage_update(lift_sensor_t *self)
{
    lift_sensor_voltage_cfg_t *cfg = (lift_sensor_voltage_cfg_t*)self->data;
    uint32_t now = rt_tick_get();
    uint32_t dt;
    uint8_t  trigger;

    SM_ASSERT_PTR(cfg);
    SM_ASSERT_PTR(cfg->read_adc_mv);

    cfg->last_voltage_mv = cfg->read_adc_mv(cfg->user_data);
    self->raw_value = (int32_t)cfg->last_voltage_mv;

    /* 【P5 / B-6】迟滞 + 连续确认：
     *   未故障：越界（>hi 或 <lo）持续 confirm_ms 才置位；
     *   已故障：回到区间 [lo_exit, hi_exit] 持续 recover_ms 才清除。
     * 中途恢复/再越界都清零重计（判连续，不判累计）。 */
    if (self->fault_latched == 0U) {
        /* 进入判定用 >= / <=：用例就是"注入 4800 mV"即算越界，
         * ADC 读回正好 4800，用严格 > 会永远不触发。 */
        trigger = (uint8_t)((cfg->last_voltage_mv >= cfg->fault_high_mv) ||
                            (cfg->last_voltage_mv <= cfg->fault_low_mv));
        if (trigger == 0U) {
            (void)voltage_timer_stop(cfg);
            self->fault_elapsed_ms = 0U;
            self->fault_timing     = 0U;
        } else {
            if (cfg->timer_active == 0U) {
                cfg->timer_tick   = now;
                cfg->timer_active = 1U;
            }
            self->fault_timing     = 1U;
            self->fault_elapsed_ms = ((uint32_t)(now - cfg->timer_tick) * 1000U) / RT_TICK_PER_SECOND;
            if (self->fault_elapsed_ms >= (uint32_t)cfg->confirm_ms) {
                (void)voltage_timer_stop(cfg);
                self->fault_latched = 1U;
                rt_kprintf("[LiftSensor] FAULT: %umV held %ums (>=%u or <=%u)\n",
                           (unsigned)cfg->last_voltage_mv,
                           (unsigned)self->fault_elapsed_ms,
                           (unsigned)cfg->fault_high_mv,
                           (unsigned)cfg->fault_low_mv);
            }
        }
    } else {
        trigger = (uint8_t)((cfg->last_voltage_mv <= cfg->fault_high_exit_mv) &&
                            (cfg->last_voltage_mv >= cfg->fault_low_exit_mv));
        if (trigger == 0U) {
            (void)voltage_timer_stop(cfg);
            self->fault_elapsed_ms = 0U;
            self->fault_timing     = 0U;
        } else {
            if (cfg->timer_active == 0U) {
                cfg->timer_tick   = now;
                cfg->timer_active = 1U;
            }
            self->fault_timing     = 1U;
            self->fault_elapsed_ms = ((uint32_t)(now - cfg->timer_tick) * 1000U) / RT_TICK_PER_SECOND;
            if (self->fault_elapsed_ms >= (uint32_t)cfg->recover_ms) {
                dt = voltage_timer_stop(cfg);
                self->fault_latched = 0U;
                rt_kprintf("[LiftSensor] RECOVER: %umV held %ums\n",
                           (unsigned)cfg->last_voltage_mv, (unsigned)dt);
            }
        }
    }

    self->fault = self->fault_latched;
}

static int32_t voltage_get_raw(lift_sensor_t *self)
{
    return self->raw_value;
}

static uint8_t voltage_get_fault(lift_sensor_t *self)
{
    return self->fault;
}

static void voltage_deinit(lift_sensor_t *self)
{
    (void)self;
}

const lift_sensor_ops_t lift_sensor_voltage_ops = {
    .init      = voltage_init,
    .update    = voltage_update,
    .get_raw   = voltage_get_raw,
    .get_fault = voltage_get_fault,
    .deinit    = voltage_deinit,
};

/* ============================================================
 * 电流型传感器实现（4~20mA）
 * ============================================================ */
typedef struct {
    uint16_t (*read_adc_mv)(void *user_data);
    void    *user_data;
    uint16_t sample_resistor_ohm;
    uint16_t fault_high_ma10;
    uint16_t fault_low_ma10;
    uint16_t last_current_ma10;
} lift_sensor_current_data_t;

static rt_err_t current_init(lift_sensor_t *self)
{
    lift_sensor_current_data_t *data = (lift_sensor_current_data_t*)self->data;

    SM_ASSERT_PTR(data);
    SM_ASSERT_PTR(data->read_adc_mv);

    /* 默认参数 */
    if (data->sample_resistor_ohm == 0) data->sample_resistor_ohm = 100;
    if (data->fault_high_ma10 == 0)     data->fault_high_ma10 = 220;
    if (data->fault_low_ma10 == 0)      data->fault_low_ma10  = 35;

    data->last_current_ma10 = 0;
    return RT_EOK;
}

static void current_update(lift_sensor_t *self)
{
    lift_sensor_current_data_t *data = (lift_sensor_current_data_t*)self->data;
    uint32_t voltage_mv;
    uint32_t current_ma10;

    SM_ASSERT_PTR(data);
    SM_ASSERT_PTR(data->read_adc_mv);

    /* 1. 读取采样电阻上的电压 */
    voltage_mv = data->read_adc_mv(data->user_data);

    /* 2. 电压 → 电流（0.1mA）: I(0.1mA) = V(mV) * 10 / R(Ω) */
    if (data->sample_resistor_ohm > 0) {
        current_ma10 = (voltage_mv * 10) / data->sample_resistor_ohm;
    } else {
        current_ma10 = 0;
    }

    if (current_ma10 > 0xFFFF) current_ma10 = 0xFFFF;

    data->last_current_ma10 = (uint16_t)current_ma10;
    self->raw_value = (int32_t)current_ma10;

    /* 3. 故障检测 */
    self->fault = 0;
    if (current_ma10 > data->fault_high_ma10) {
        self->fault = 1;    /* 过流/短路 */
    } else if (current_ma10 < data->fault_low_ma10) {
        self->fault = 1;    /* 欠流/开路 */
    }
}

static int32_t current_get_raw(lift_sensor_t *self)
{
    return self->raw_value;
}

static uint8_t current_get_fault(lift_sensor_t *self)
{
    return self->fault;
}

static void current_deinit(lift_sensor_t *self)
{
    (void)self;
}

const lift_sensor_ops_t lift_sensor_current_ops = {
    .init      = current_init,
    .update    = current_update,
    .get_raw   = current_get_raw,
    .get_fault = current_get_fault,
    .deinit    = current_deinit,
};

/* ============================================================
 * 角度型传感器（预留）
 * ============================================================ */
static rt_err_t angle_sensor_init(lift_sensor_t *self) { (void)self; return RT_EOK; }
static void     angle_sensor_update(lift_sensor_t *self) { (void)self; }
static int32_t  angle_sensor_get_raw(lift_sensor_t *self) { return self->raw_value; }
static uint8_t  angle_sensor_get_fault(lift_sensor_t *self) { return self->fault; }
static void     angle_sensor_deinit(lift_sensor_t *self) { (void)self; }

const lift_sensor_ops_t lift_sensor_angle_ops = {
    .init      = angle_sensor_init,
    .update    = angle_sensor_update,
    .get_raw   = angle_sensor_get_raw,
    .get_fault = angle_sensor_get_fault,
    .deinit    = angle_sensor_deinit,
};

/* ============================================================
 * CAN 型传感器（预留）
 * ============================================================ */
const lift_sensor_ops_t lift_sensor_can_ops = {
    .init      = NULL,
    .update    = NULL,
    .get_raw   = NULL,
    .get_fault = NULL,
    .deinit    = NULL,
};
