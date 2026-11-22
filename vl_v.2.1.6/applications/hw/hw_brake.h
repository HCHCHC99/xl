/**
 * @file    hw_brake.h
 * @brief   刹车盘设备标准接口
 * @note    支持状态查询、可插拔状态源
 */

#ifndef __HW_BRAKE_H
#define __HW_BRAKE_H

#include <stdint.h>
#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 前置声明
 * ============================================================ */
typedef struct hw_brake     hw_brake_t;
typedef struct hw_brake_cfg hw_brake_cfg_t;

/* ============================================================
 * 刹车盘状态
 * ============================================================ */
typedef enum {
    BRAKE_STATE_LOCKED = 0,     /* 已锁止 */
    BRAKE_STATE_UNLOCKING,      /* 解锁请求中，UI 可显示"动作中" */
    BRAKE_STATE_UNLOCKED,       /* 解锁完成，允许运动 */
    BRAKE_STATE_LOCKING,        /* 锁止请求中 */
    BRAKE_STATE_FAULT,          /* 超时/异常，需 clear_fault */
} hw_brake_state_t;

/* ============================================================
 * 状态源接口（可插拔）
 * ============================================================
 * 默认实现：brake_provider_timer.c
 * 扩展实现：GPIO 到位反馈、ADC 电流检测等
 * ============================================================ */
typedef struct {
    /**
     * @brief 状态源复位 [单线程]
     * @note  进入 UNLOCKING/LOCKING 时调用，重置内部计时
     */
    void (*reset)(hw_brake_t *self);

    /**
     * @brief 状态源周期驱动 [单线程]
     */
    void (*tick)(hw_brake_t *self, uint32_t dt_ms);

    /**
     * @brief 查询解锁是否完成
     * @return 1 已到位，0 未到位
     */
    uint8_t (*is_unlock_done)(hw_brake_t *self);

    /**
     * @brief 查询锁止是否完成
     * @return 1 已到位，0 未到位
     */
    uint8_t (*is_lock_done)(hw_brake_t *self);
} brake_provider_ops_t;

/* ============================================================
 * 设备操作接口
 * ============================================================ */
typedef struct {
    /**
     * @brief 初始化 [单线程]
     */
    rt_err_t (*init)(hw_brake_t *self);

    /**
     * @brief 请求解锁 [单线程]
     * @note  非阻塞，实际到位由 tick 推进
     */
    rt_err_t (*unlock)(hw_brake_t *self);

    /**
     * @brief 请求锁止 [单线程]
     */
    rt_err_t (*lock)(hw_brake_t *self);

    /**
     * @brief 紧急锁止 [ISR 安全]
     * @note  直接操作寄存器，不动状态机
     */
    rt_err_t (*emergency_lock)(hw_brake_t *self);

    /**
     * @brief 清除故障 [单线程]
     */
    rt_err_t (*clear_fault)(hw_brake_t *self);

    /**
     * @brief 反初始化 [单线程]
     */
    rt_err_t (*deinit)(hw_brake_t *self);
} hw_brake_ops_t;


/* ============================================================
 * 配置结构体（只含静态配置）
 * ============================================================ */
struct hw_brake_cfg {
    const char                 *name;
    void                       *priv;
    uint32_t                    unlock_ms;
    uint32_t                    lock_ms;
    uint32_t                    timeout_ms;
};

/* ============================================================
 * 设备对象（cfg 指针 + 运行时状态）
 * ============================================================ */
struct hw_brake {
    const hw_brake_cfg_t *cfg;              /* ★ 指向配置表 */
    const hw_brake_ops_t       *ops;        /* ★ 编译期绑定 */
    const brake_provider_ops_t *provider;   /* ★ 编译期绑定 */
    hw_brake_state_t      state;
    uint32_t              elapsed_ms;       /* 当前状态已历时 */
    uint8_t               is_ready;

    rt_timer_t            state_timer;      /* 状态推进定时器（一次性） */
};


/* ============================================================
 * 标准 API
 * ============================================================ */

/**
 * @brief 初始化刹车盘 [单线程]
 */
rt_err_t hw_brake_init(hw_brake_t *self,
                       const hw_brake_cfg_t *cfg,
                       const hw_brake_ops_t *ops,
                       const brake_provider_ops_t *provider);

/**
 * @brief 请求解锁 [单线程]
 */
rt_err_t hw_brake_unlock(hw_brake_t *self);

/**
 * @brief 请求锁止 [单线程]
 */
rt_err_t hw_brake_lock(hw_brake_t *self);

/**
 * @brief 紧急锁止 [ISR 安全]
 */
rt_err_t hw_brake_emergency_lock(hw_brake_t *self);

/**
 * @brief 清除故障 [单线程]
 */
rt_err_t hw_brake_clear_fault(hw_brake_t *self);

/**
 * @brief 周期驱动 [单线程]
 */
void hw_brake_tick(hw_brake_t *self, uint32_t dt_ms);

/**
 * @brief 查询状态
 */
hw_brake_state_t hw_brake_get_state(const hw_brake_t *self);

/* 便捷宏：访问配置 */
#define HW_BRAKE_UNLOCK_MS(self)   ((self)->cfg->unlock_ms)
#define HW_BRAKE_LOCK_MS(self)     ((self)->cfg->lock_ms)
#define HW_BRAKE_TIMEOUT_MS(self)  ((self)->cfg->timeout_ms)


/* ============================================================
 * 默认状态源
 * ============================================================ */
extern const brake_provider_ops_t g_brake_provider_timer;

/* ============================================================
 * 配置表
 * ============================================================ */
extern const hw_brake_cfg_t g_swing_brake_cfg;

/**
 * @brief  补打中断上下文挂起的急停日志
 * @note   【2026-11-10 新增】
 *
 *         背景：hw_brake_emergency_lock() 可被 20kHz 电流环 ISR
 *         直接调用（stall_trigger 路径）。在 50μs 周期里做
 *         vsnprintf + device_write 会必然超时，因此中断里只置标志，
 *         由 bg_sched 的 5ms 任务调本函数补打。
 *
 *         调用点：bg_modules.c 的 motor_periodic_task()。
 *         未挂起时立即返回（一次字节读，可忽略）。
 */
void hw_brake_flush_log(void);

#ifdef __cplusplus
}
#endif

#endif /* __HW_BRAKE_H */
