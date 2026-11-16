/**
 * @file    hw_brake.c
 * @brief   刹车盘设备接口实现
 * @note    状态机 + 状态源调度，不涉及具体硬件
 */

#include "hw_brake.h"
#include "framework/state_engine/sm_assert.h"
#include <string.h>
#include <rtthread.h>

/* 【2026-11-10】彩色日志：紧急锁定属 FAULT 级，与 MOTOR-OC 同色，
 * 两条连在一起构成完整的"过流 -> 刹车"告警块。 */
#include "../log/log_color.h"

/* ============================================================
 * 前向声明声明
 * ============================================================ */
static void _brake_state_timer_cb(void *param);

/* 中断上下文挂起的急停日志标志（见 hw_brake_emergency_lock 注释）*/
static volatile uint8_t s_log_pending = 0U;


/* ============================================================
 * 标准 API 实现
 * ============================================================ */
rt_err_t hw_brake_init(hw_brake_t *self,
                       const hw_brake_cfg_t *cfg,
                       const hw_brake_ops_t *ops,
                       const brake_provider_ops_t *provider)
{
    SM_ASSERT_PTR(self);
    SM_ASSERT_PTR(cfg);
    SM_ASSERT_PTR(ops);
    SM_ASSERT_PTR(provider);


    self->cfg        = cfg;
    self->ops        = ops;
    self->provider   = provider;
    self->state      = BRAKE_STATE_LOCKED;
    self->elapsed_ms = 0;
    self->is_ready   = 0;

    if (ops->init) {
        rt_err_t ret = ops->init(self);
        if (ret != RT_EOK) {
            LOG_ERR("[HW-BRAKE] %s init failed (%d)\n", cfg->name, ret);
            return ret;
        }
    }

    /* 创建一次性软定时器 */
    self->state_timer = rt_timer_create("bkt",
                                        _brake_state_timer_cb,
                                        self,
                                        rt_tick_from_millisecond(cfg->unlock_ms),
                                        RT_TIMER_FLAG_ONE_SHOT | RT_TIMER_FLAG_SOFT_TIMER);
    if (self->state_timer == RT_NULL) {
        LOG_ERR("[HW-BRAKE] %s timer create failed\n", cfg->name);
        return -RT_ERROR;
    }

    self->is_ready = 1;
    LOG_OK("[HW-BRAKE] %s initialized\n", cfg->name);
    return RT_EOK;
}

rt_err_t hw_brake_unlock(hw_brake_t *self)
{
    SM_ASSERT_PTR(self);
    if (!self->is_ready)                     return -RT_ERROR;
    if (self->state == BRAKE_STATE_UNLOCKED) return RT_EOK;
    if (self->state == BRAKE_STATE_FAULT)    return -RT_ERROR;

    if (self->ops->unlock) {
        self->ops->unlock(self);
    }
    if (self->provider->reset) {
        self->provider->reset(self);
    }
    self->elapsed_ms = 0;
    self->state      = BRAKE_STATE_UNLOCKING;

    rt_tick_t ticks = rt_tick_from_millisecond(self->cfg->unlock_ms);
    rt_timer_control(self->state_timer, RT_TIMER_CTRL_SET_TIME, &ticks);
    rt_timer_start(self->state_timer);

    LOG_INFO("[HW-BRAKE] %s unlocking\n", self->cfg->name);

    return RT_EOK;
}

rt_err_t hw_brake_lock(hw_brake_t *self)
{
    SM_ASSERT_PTR(self);
    if (!self->is_ready)                   return -RT_ERROR;
    if (self->state == BRAKE_STATE_LOCKED) return RT_EOK;

    if (self->ops->lock) {
        self->ops->lock(self);
    }
    if (self->provider->reset) {
        self->provider->reset(self);
    }
    self->elapsed_ms = 0;
    self->state      = BRAKE_STATE_LOCKING;

    rt_tick_t ticks = rt_tick_from_millisecond(self->cfg->lock_ms);
    rt_timer_control(self->state_timer, RT_TIMER_CTRL_SET_TIME, &ticks);
    rt_timer_start(self->state_timer);

    LOG_INFO("[HW-BRAKE] %s locking\n", self->cfg->name);

    return RT_EOK;
}

rt_err_t hw_brake_emergency_lock(hw_brake_t *self)
{
    SM_ASSERT_PTR(self);
    if (!self->is_ready) return -RT_ERROR;

    /* ============================================================
     * 【2026-11-09 幂等化 + 日志节流】
     * ------------------------------------------------------------
     * 原实现每次都完整执行并打印。当上层（motor_update → motor_oc）
     * 在 LEVEL3 状态下每 5ms 重复调用时，就变成刷屏，
     * 把 MSH 的调试输出冲掉。
     *
     * 两层修复：
     *   ① 幂等：已经在 LOCKED 就直接返回，不重置 elapsed_ms、
     *      不重复调 ops->emergency_lock（那会不断重置刹车状态机）。
     *   ② 记忆化：即使需要动作，同一原因 500ms 内只打印一次。
     *
     * 为什么保留打印而不是全删：
     *   紧急刹车是安全事件，日志有排查价值；
     *   500ms 节流后既能看到发生过，又不会淹掉命令回显。
     * ============================================================ */
    if (self->state == BRAKE_STATE_LOCKED) {
        return RT_EOK;              /* 已锁定：幂等返回，零副作用 */
    }

    /* 停掉正在等待的推进定时器 */
    if (self->state_timer) {
        rt_timer_stop(self->state_timer);
    }

    if (self->ops->emergency_lock) {
        self->ops->emergency_lock(self);
    }
    self->state      = BRAKE_STATE_LOCKED;
    self->elapsed_ms = 0;

    /* 500ms 日志节流（用静态变量，单实例够用；多实例场景影响仅限日志）
     *
     * 【2026-11-10 修正：中断上下文不得打印】
     * ------------------------------------------------------------
     * 本函数可能由 20kHz 电流环 ISR 直接调用（调用链：
     *   foc_adc_dma_isr -> stall_on_current -> stall_trigger
     *   -> hw_brake_emergency_lock）
     * 在 20kHz 中断里做 vsnprintf + 3 次 device_write，
     * 单次可达数十微秒，周期只有 50μs，必然导致 ISR 超时、
     * 丢帧、甚至看门狗复位。
     *
     * 原来的 rt_kprintf 也有这个问题，但影响稍小；
     * 换成带颜色码的 log_printf 后开销更大（多一次 memcpy
     * 与两次额外的 device_write），所以必须在中断里彻底拦住。
     *
     * 处理方式：中断里只置"待上报"标志，打印交给
     * bg_sched 的 5ms 任务（hw_brake_flush_log）补打。
     * 这样既保住安全事件的可观测性，又不在 ISR 里做重活。
     * ============================================================ */
    {
        static uint32_t s_last_log_tick = 0U;
        uint32_t now = rt_tick_get();

        if ((s_last_log_tick == 0U) ||
            ((now - s_last_log_tick) >= (RT_TICK_PER_SECOND / 2)))
        {
            s_last_log_tick = now;

            if (rt_interrupt_get_nest() > 0U)
            {
                /* 中断上下文：只登记，不打印 */
                s_log_pending = 1U;
            }
            else
            {
                LOG_FAULT("[HW-BRAKE] %s emergency_locked\n",
                          self->cfg->name);
            }
        }
    }

    return RT_EOK;
}

/**
 * @brief  补打中断里挂起的急停日志
 * @note   由 bg_sched 的 5ms 任务调用（见 bg_modules.c）。
 *         之所以单独抽出来：中断上下文不能打印，
 *         但安全事件必须留痕，只能延迟到线程里补。
 *         标志用 static 而非实例字段，因为急停日志是
 *         "系统级一次性事件"，不需要区分哪个刹车实例。
 */
void hw_brake_flush_log(void)
{
    if (s_log_pending == 0U) {
        return;
    }
    s_log_pending = 0U;

    LOG_FAULT("[HW-BRAKE] emergency_locked (deferred from ISR)\n");
}

rt_err_t hw_brake_clear_fault(hw_brake_t *self)
{
    SM_ASSERT_PTR(self);
    if (self->state != BRAKE_STATE_FAULT) return RT_EOK;

    if (self->ops->emergency_lock) {
        self->ops->emergency_lock(self);
    }
    self->state      = BRAKE_STATE_LOCKED;
    self->elapsed_ms = 0;
    LOG_OK("[HW-BRAKE] %s fault cleared\n", self->cfg->name);
    return RT_EOK;
}

void hw_brake_tick(hw_brake_t *self, uint32_t dt_ms)
{
//    SM_ASSERT_PTR(self);
//    if (!self->is_ready) return;
//
//    if (self->state == BRAKE_STATE_LOCKED ||
//        self->state == BRAKE_STATE_UNLOCKED ||
//        self->state == BRAKE_STATE_FAULT) {
//        return;
//    }
//
//    self->elapsed_ms += dt_ms;
//
//    if (self->elapsed_ms > self->cfg->timeout_ms) {
//        rt_kprintf("[HW-BRAKE] %s timeout @%ums -> FAULT\n",
//                   self->cfg->name, self->elapsed_ms);
//        if (self->cfg->ops->emergency_lock) {
//            self->cfg->ops->emergency_lock(self);
//        }
//        self->state = BRAKE_STATE_FAULT;
//        return;
//    }
//
//    if (self->cfg->provider->tick) {
//        self->cfg->provider->tick(self, dt_ms);
//    }
//
//    if (self->state == BRAKE_STATE_UNLOCKING) {
//        if (self->cfg->provider->is_unlock_done(self)) {
//            self->state = BRAKE_STATE_UNLOCKED;
//            rt_kprintf("[HW-BRAKE] %s unlocked @%ums\n",
//                       self->cfg->name, self->elapsed_ms);
//        }
//    } else if (self->state == BRAKE_STATE_LOCKING) {
//        if (self->cfg->provider->is_lock_done(self)) {
//            self->state = BRAKE_STATE_LOCKED;
//            rt_kprintf("[HW-BRAKE] %s locked @%ums\n",
//                       self->cfg->name, self->elapsed_ms);
//        }
//    }
}

hw_brake_state_t hw_brake_get_state(const hw_brake_t *self)
{
    if (self == NULL) return BRAKE_STATE_FAULT;
    return self->state;
}

/**
 * @brief 状态推进定时器回调
 * @note  运行在 RT-Thread timer 线程上下文，非 ISR
 */
static void _brake_state_timer_cb(void *param)
{
    hw_brake_t *self = (hw_brake_t *)param;

    switch (self->state) {
    case BRAKE_STATE_UNLOCKING:
        self->state = BRAKE_STATE_UNLOCKED;
        LOG_INFO("[HW-BRAKE] %s unlocked\n", self->cfg->name);
        break;

    case BRAKE_STATE_LOCKING:
        self->state = BRAKE_STATE_LOCKED;
        LOG_INFO("[HW-BRAKE] %s locked\n", self->cfg->name);
        break;

    default:
        /* 状态已被其它路径改掉，忽略 */
        break;
    }
}
