/**
 * @file    hw_pwm.h
 * @brief   三相 PWM 设备标准接口
 * @note    配置表 + ops + 对象，对象仅持有 cfg 指针（零拷贝）
 */

#ifndef __HW_PWM_H
#define __HW_PWM_H

#include <stdint.h>
#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 前置声明
 * ============================================================ */
typedef struct hw_pwm     hw_pwm_t;
typedef struct hw_pwm_cfg hw_pwm_cfg_t;

/* ============================================================
 * 相号枚举
 * ============================================================ */
typedef enum {
    HW_PWM_PHASE_U = 1,     /* U相（A相） */
    HW_PWM_PHASE_V = 2,     /* V相（B相） */
    HW_PWM_PHASE_W = 3,     /* W相（C相） */
} hw_pwm_phase_t;

/* ============================================================
 * 设备操作接口
 * ============================================================ */
typedef struct {
    rt_err_t (*init)(hw_pwm_t *self);
    rt_err_t (*deinit)(hw_pwm_t *self);
    rt_err_t (*set_duty)(hw_pwm_t *self, uint8_t phase, uint32_t duty);
    rt_err_t (*enable)(hw_pwm_t *self);
    rt_err_t (*disable)(hw_pwm_t *self);
    rt_err_t (*brake)(hw_pwm_t *self);
    rt_err_t (*brake_release)(hw_pwm_t *self);
} hw_pwm_ops_t;


/* ============================================================
 * 配置结构体（只含静态配置）
 * ============================================================ */
struct hw_pwm_cfg {
    const char         *name;
    void               *priv;
    uint32_t            freq_hz;
    uint32_t            deadtime_ns;
    uint32_t            max_duty;
};

/* ============================================================
 * 设备对象（仅持有 cfg 指针 + 运行时状态）
 * ============================================================ */
struct hw_pwm {
    const hw_pwm_cfg_t *cfg;            /* ★ 指向配置表 */
    const hw_pwm_ops_t *ops;            /* ★ 编译期绑定 */
    uint8_t             is_ready;
};

/* ============================================================
 * 标准 API
 * ============================================================ */

/**
 * @brief 初始化三相 PWM [单线程]
 */
rt_err_t hw_pwm_init(hw_pwm_t *self,
                     const hw_pwm_cfg_t *cfg,
                     const hw_pwm_ops_t *ops);

/**
 * @brief 反初始化 [单线程]
 */
rt_err_t hw_pwm_deinit(hw_pwm_t *self);

/**
 * @brief 设置某相占空比 [线程安全]
 */
rt_err_t hw_pwm_set_duty(hw_pwm_t *self, uint8_t phase, uint32_t duty);

/**
 * @brief 使能输出 [线程安全]
 */
rt_err_t hw_pwm_enable(hw_pwm_t *self);

/**
 * @brief 关闭输出 [线程安全]
 */
rt_err_t hw_pwm_disable(hw_pwm_t *self);

/**
 * @brief 刹车（三相短接） [ISR 安全]
 */
rt_err_t hw_pwm_brake(hw_pwm_t *self);

/**
 * @brief 解除 EMB 刹车
 */
rt_err_t hw_pwm_brake_release(hw_pwm_t *self);

/* ============================================================
 * 配置表（由 BSP 层提供）
 * ============================================================ */
extern const hw_pwm_cfg_t g_swing_pwm_cfg;

#ifdef __cplusplus
}
#endif

#endif /* __HW_PWM_H */
