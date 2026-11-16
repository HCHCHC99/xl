/**
 * @file    motor_drv.h
 * @brief   驱动层接口（直接操作功率级硬件）
 */

#ifndef __MOTOR_DRV_H
#define __MOTOR_DRV_H

#include <stdint.h>
#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 前置声明
 * ============================================================ */
struct motor;

/* ============================================================
 * 驱动层接口
 * ============================================================ */
typedef struct motor_drv_ops {
    /**
     * @brief 初始化驱动硬件
     */
    rt_err_t (*init)(struct motor *self);

    /**
     * @brief 设置输出量
     * @param output 输出语义由驱动实现决定：
     *               BDC: PWM占空比+符号表示方向
     *               BLDC-6step: PWM占空比
     *               BLDC-FOC: q轴电流指令
     */
    void (*set_output)(struct motor *self, int32_t output);

    /**
     * @brief 刹车（三相短接）
     */
    void (*brake)(struct motor *self);

    /**
     * @brief 滑行（三相悬空）
     */
    void (*coast)(struct motor *self);

    /**
     * @brief 读取硬件故障
     */
    uint8_t (*get_fault)(struct motor *self);

    /**
     * @brief 后台周期上下文
     */
    void (*tick)(motor_t *self, uint32_t dt_ms);
    /**
     * @brief 换相更新（BLDC专用，由霍尔中断或高频定时器调用）
     */
    void (*update)(struct motor *self);

    /**
     * @brief 反初始化
     */
    void (*deinit)(struct motor *self);
}motor_drv_ops_t;

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_DRV_H */
