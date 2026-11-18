/**
 * @file    motor_drv_bldc_6step.c
 * @brief   BLDC 六步换相驱动实现
 * @note    换相由霍尔中断触发；上电先开环强拖，稳定后交给霍尔
 */
#include "motor_drv_bldc_6step.h"
#include "../../bsp/bsp_hall.h"
#include "../../bsp/bsp_pwm.h"
#include "../../hw/hw_pwm.h"
#include "../foc/foc_mode.h"   /* FOC_DISABLE_LEGACY_6STEP / Foc_Mode_PwmOwned */
#include "framework/state_engine/sm_assert.h"
#include <string.h>
#include <rtthread.h>

/* ============================================================
 * 外部引用：PWM 对象（在 app_hw.c 中定义）
 * ============================================================ */
extern hw_pwm_t g_swing_pwm;

/* ============================================================
 * 开环强拖参数（可用宏覆盖）
 * ============================================================ */
#ifndef BSP_PWM_FORCESTART_STEP_MS
#define BSP_PWM_FORCESTART_STEP_MS      (30U)   /* 每步时长 ms */
#endif
#ifndef BSP_PWM_FORCESTART_MAX_MS
#define BSP_PWM_FORCESTART_MAX_MS       (3000U) /* 强拖超时 ms */
#endif

/* ============================================================
 * 驱动私有数据
 * ============================================================ */
typedef struct {
    const motor_pwm_cfg_t  *pwm_cfg;
    const motor_hall_cfg_t *hall_cfg;
    uint8_t  current_step;              /* 当前换相步骤 1~6 */
    uint32_t duty;                      /* 当前占空比 */
    uint8_t  hall_state;                /* 当前霍尔状态 */
    uint8_t  enabled;
    uint8_t  reverse;                   /* 1 = 反转 */

    /* 开环强拖 */
    uint8_t  forcestart_active;         /* 1 = 正在强拖 */
    uint8_t  forcestart_done;           /* 1 = 强拖完成，交给霍尔 */
    uint8_t  forcestart_step;           /* 强拖当前步 */
    uint32_t forcestart_elapsed_ms;     /* 强拖单步计时 */
    uint32_t forcestart_total_ms;       /* 强拖总计时 */
    volatile uint8_t hall_changed;      /* 霍尔中断里置位 */
} bldc_6step_data_t;

/* ============================================================
 * 六步换相表
 * 霍尔状态 (HA HB HC) → 换相步骤
 * ============================================================ */
static const uint8_t g_default_hall_to_step[8] = {
    0,      /* 000: 无效 */
    1,      /* 001: 步骤1 */
    5,      /* 010: 步骤5 */
    4,      /* 011: 步骤4 */
    3,      /* 100: 步骤3 */
    2,      /* 101: 步骤2 */
    6,      /* 110: 步骤6 */
    0,      /* 111: 无效 */
};

/* ============================================================
 * 换相表：步骤 → 上桥臂/下桥臂导通相
 * ============================================================ */
static const uint8_t g_step_phase[7][2] = {
    {0, 0},     /* 索引0占位 */
    {1, 2},     /* 步骤1: U上, V下 */
    {1, 3},     /* 步骤2: U上, W下 */
    {2, 3},     /* 步骤3: V上, W下 */
    {2, 1},     /* 步骤4: V上, U下 */
    {3, 1},     /* 步骤5: W上, U下 */
    {3, 2},     /* 步骤6: W上, V下 */
};

/* ============================================================
 * 前向声明
 * ============================================================ */
static void bldc_hall_callback(hw_hall_t *hall, void *user_data);

/* ============================================================
 * 内部辅助：设置三相桥臂输出
 * ============================================================ */
static void set_bridge_output(bldc_6step_data_t *data, uint8_t step, uint32_t duty)
{
    if (Foc_Mode_PwmOwned()) return;   /* FOC 独占 PWM，旧六步禁止写占空比 */

    if (step < 1 || step > 6) return;

    uint8_t upper = g_step_phase[step][0];
    uint8_t lower = g_step_phase[step][1];

    /* 反转时交换上下桥 */
    if (data->reverse) {
        uint8_t tmp = upper;
        upper = lower;
        lower = tmp;
    }

    for (uint8_t phase = 1; phase <= 3; phase++) {
        if (phase == upper) {
            hw_pwm_set_duty(&g_swing_pwm, phase, duty);
        } else if (phase == lower) {
            hw_pwm_set_duty(&g_swing_pwm, phase, 0);
        } else {
            /* 悬空相下桥导通，作为续流路径 */
            hw_pwm_set_duty(&g_swing_pwm, phase, 0);
        }
    }
}

/* ============================================================
 * 驱动层接口实现
 * ============================================================ */
static rt_err_t bldc_6step_init(motor_t *self)
{
    SM_ASSERT_PTR(self);
    SM_ASSERT_PTR(self->config);

    bldc_6step_data_t *data = rt_malloc(sizeof(bldc_6step_data_t));
    if (data == RT_NULL) return -RT_ENOMEM;

    memset(data, 0, sizeof(bldc_6step_data_t));
    data->pwm_cfg = (const motor_pwm_cfg_t*)self->config->drv_hw_cfg;
    data->hall_cfg = (const motor_hall_cfg_t*)self->config->comm_cfg;
    SM_ASSERT_PTR(data->pwm_cfg);
    SM_ASSERT_PTR(data->hall_cfg);


    /* ============================================================
     * 1. 校验硬件对象已就绪
     * ============================================================
     * 对象初始化由 boot 表 app_hw 步统一完成（hw_pwm_init / hw_hall_init）。
     * 驱动层不再重复初始化，原因：
     *   - hw_hall_init 二次调用会清空 callback / user_data / capture_count
     *   - 破坏 boot 表 "app_hw → motor" 的分层依赖语义
     * ============================================================ */
    if (!g_swing_pwm.is_ready) {
        rt_kprintf("[BLDC] pwm not ready, abort\n");
        rt_free(data);
        return -RT_ERROR;
    }
    if (!g_swing_hall.is_ready) {
        rt_kprintf("[BLDC] hall not ready, abort\n");
        rt_free(data);
        return -RT_ERROR;
    }

    /* ============================================================
     * 2. 注册 Hall 回调（关键：绑定电机对象作为 user_data）
     * ============================================================ */
    rt_err_t ret = hw_hall_set_callback(&g_swing_hall,
                                        bldc_hall_callback,
                                        self);      /* user_data = motor_t */
    if (ret != RT_EOK) {
        rt_free(data);
        return ret;
    }

    /* ============================================================
     * 3. 读初始霍尔，设置初始换相步骤
     * ============================================================ */
    data->hall_state = hw_hall_read_state(&g_swing_hall);
    uint8_t step = data->hall_cfg->hall_state_table[data->hall_state];
    if (step < 1 || step > 6) step = 1;   /* 无效霍尔兜底 */
    data->current_step     = step;
    data->forcestart_step  = step;

    /* ============================================================
     * 4. 预置初始换向状态（占空比 0），防止 enable 瞬间意外导通
     * ============================================================ */
    set_bridge_output(data, step, 0);

    self->drv_data = data;
    data->enabled = 0;

    rt_kprintf("[BLDC] Initialized (pole_pairs=%d, pwm=%dHz)\n",
               self->pole_pairs, data->pwm_cfg->pwm_freq_hz);

    return RT_EOK;
}

/* ============================================================
 * Hall 中断回调（由 BSP 层 ISR 调用）
 * ============================================================ */
static void bldc_hall_callback(hw_hall_t *hall, void *user_data)
{
    motor_t *motor = (motor_t*)user_data;
    SM_ASSERT_PTR(motor);
    (void)hall;

    /* 转发到驱动层更新函数 */
    if (motor->drv_ops && motor->drv_ops->update) {
        motor->drv_ops->update(motor);
    }
}

/* ============================================================
 * 设置输出（占空比）
 * ============================================================ */
static void bldc_6step_set_output(motor_t *self, int32_t output)
{
    bldc_6step_data_t *data = (bldc_6step_data_t*)self->drv_data;
    SM_ASSERT_PTR(data);

    /* FOC 独占 TMR4_3 时，旧六步路径不得使能/写占空比 */
    if (Foc_Mode_PwmOwned()) return;

    /* 方向跟随 motor->direction */
    data->reverse = (self->direction == MOTOR_DIR_CCW) ? 1U : 0U;

    /* 限幅 */
    if (output < 0) output = 0;
    if (output > (int32_t)data->pwm_cfg->max_duty) {
        output = (int32_t)data->pwm_cfg->max_duty;
    }

    data->duty = (uint32_t)output;

    /* 首次非零输出：先写初始换向，再使能 PWM，然后进入强拖 */
    if (output > 0 && !data->enabled) {
        set_bridge_output(data, data->current_step, 0);
        hw_pwm_enable(&g_swing_pwm);
        data->enabled = 1;

        data->forcestart_active     = 1;
        data->forcestart_done       = 0;
        data->forcestart_step       = data->current_step;
        data->forcestart_elapsed_ms = 0;
        data->forcestart_total_ms   = 0;
        data->hall_changed          = 0;

        rt_kprintf("[BLDC] Force start from step %d\n", data->forcestart_step);
        return;
    }

    /* 停止输出：立即封锁 */
    if (output == 0) {
        set_bridge_output(data, data->current_step, 0);
        return;
    }

    /* 非强拖模式：按 current_step 直接输出 */
    if (!data->forcestart_active &&
            data->current_step >= 1 &&
            data->current_step <= 6) {
        set_bridge_output(data, data->current_step, data->duty);
    }
}

/* ============================================================
 * 刹车
 * ============================================================ */
static void bldc_6step_brake(motor_t *self)
{
    if (Foc_Mode_PwmOwned()) return;

    bldc_6step_data_t *data = (bldc_6step_data_t*)self->drv_data;
    SM_ASSERT_PTR(data);

    hw_pwm_brake(&g_swing_pwm);   /* EMB 软件触发，ISR 安全 */
    data->duty = 0;
    data->forcestart_active = 0;
    data->forcestart_done   = 0;
}

/* ============================================================
 * 滑行
 * ============================================================ */
static void bldc_6step_coast(motor_t *self)
{
    bldc_6step_data_t *data = (bldc_6step_data_t*)self->drv_data;
    SM_ASSERT_PTR(data);

    if (Foc_Mode_PwmOwned()) return;   /* FOC 独占 PWM，旧六步禁止关闭通道 */

    hw_pwm_disable(&g_swing_pwm);
    data->duty              = 0;
    data->enabled           = 0;
    data->forcestart_active = 0;
    data->forcestart_done   = 0;
}

/* ============================================================
 * 故障读取
 * ============================================================ */
static uint8_t bldc_6step_get_fault(motor_t *self)
{
    /* TODO: 读取硬件故障信号（如驱动芯片的 nFAULT 引脚） */
    (void)self;
    return 0;
}

/**
 * @brief 换相更新（由霍尔中断调用）
 * @note  此函数响应霍尔状态变化，执行换相
 */
static void bldc_6step_update(motor_t *self)
{
    bldc_6step_data_t *data = (bldc_6step_data_t*)self->drv_data;
    SM_ASSERT_PTR(data);

    if (Foc_Mode_PwmOwned()) return;

    uint8_t hall = hw_hall_read_state(&g_swing_hall);
    if (hall == data->hall_state) return;

    data->hall_state   = hall;
    data->hall_changed = 1;

    /* 强拖期间不动输出，由 tick 驱动换相 */
    if (data->forcestart_active) return;

    uint8_t step = data->hall_cfg->hall_state_table[hall];
    if (step < 1 || step > 6) return;

    data->current_step = step;
    set_bridge_output(data, step, data->duty);
}

/* ============================================================
 * 后台 tick：驱动开环强拖
 * ============================================================ */
static void bldc_6step_tick(motor_t *self, uint32_t dt_ms)
{
    bldc_6step_data_t *data = (bldc_6step_data_t*)self->drv_data;
    SM_ASSERT_PTR(data);

    if (Foc_Mode_PwmOwned()) return;

    if (!data->forcestart_active) return;

    /* 霍尔已变化：退出强拖，交回霍尔换相 */
    if (data->hall_changed) {
        data->hall_changed      = 0;
        data->forcestart_active = 0;
        data->forcestart_done   = 1;

        uint8_t step = data->hall_cfg->hall_state_table[data->hall_state];
        if (step >= 1 && step <= 6) {
            data->current_step = step;
            set_bridge_output(data, step, data->duty);
        }
        rt_kprintf("[BLDC] Force start done, hall=0x%X, step=%d\n",
                   data->hall_state, data->current_step);
        return;
    }

    /* 强拖超时保护 */
    data->forcestart_total_ms += dt_ms;
    if (data->forcestart_total_ms > BSP_PWM_FORCESTART_MAX_MS) {
        data->forcestart_active = 0;
        data->forcestart_done   = 0;
        set_bridge_output(data, data->current_step, 0);
        rt_kprintf("[BLDC] Force start timeout, output disabled\n");
        return;
    }

    /* 强拖换相：按固定时间步进 */
    data->forcestart_elapsed_ms += dt_ms;
    if (data->forcestart_elapsed_ms >= BSP_PWM_FORCESTART_STEP_MS) {
        data->forcestart_elapsed_ms = 0;
        data->forcestart_step       = (data->forcestart_step % 6) + 1;
        set_bridge_output(data, data->forcestart_step, data->duty);
    }
}


static void bldc_6step_deinit(motor_t *self)
{
    SM_ASSERT_PTR(self);

    if (self->drv_data) {
        bldc_6step_coast(self);
        hw_hall_deinit(&g_swing_hall);
        hw_pwm_deinit(&g_swing_pwm);

        rt_free(self->drv_data);
        self->drv_data = NULL;
    }
}

/* ============================================================
 * 接口表
 * ============================================================ */
const motor_drv_ops_t motor_drv_bldc_6step_ops = {
    .init       = bldc_6step_init,
    .set_output = bldc_6step_set_output,
    .brake      = bldc_6step_brake,
    .coast      = bldc_6step_coast,
    .get_fault  = bldc_6step_get_fault,
    .update     = bldc_6step_update,
    .deinit     = bldc_6step_deinit,
    .tick       = bldc_6step_tick,
    .deinit     = bldc_6step_deinit,
};

/* ============================================================
 * 霍尔中断处理（供外部中断调用）
 * ============================================================ */
void motor_bldc_6step_hall_isr(motor_t *motor)
{
    SM_ASSERT_PTR(motor);
    if (motor->drv_ops && motor->drv_ops->update) {
        motor->drv_ops->update(motor);
    }
}
