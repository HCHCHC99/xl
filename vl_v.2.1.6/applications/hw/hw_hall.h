/**
 * @file    hw_hall.h
 * @brief   Hall 传感器设备标准接口
 * @note    支持 3 路 Hall 信号采集 + 双边沿中断换相回调
 *          通过回调注册机制解耦 BSP 层与业务层
 */

#ifndef __HW_HALL_H
#define __HW_HALL_H

#include <stdint.h>
#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 前置声明
 * ============================================================ */
typedef struct hw_hall     hw_hall_t;
typedef struct hw_hall_cfg hw_hall_cfg_t;

/* ============================================================
 * Hall 状态枚举（3 位编码）
 * ============================================================
 *
 * 有效状态：1~6
 * 非法状态：0（000）、7（111）
 *
 * 位序：bit2=HALL_A, bit1=HALL_B, bit0=HALL_C
 * ============================================================ */
typedef enum {
    HW_HALL_STATE_INVALID_LOW  = 0,     /* 000: 非法 */
    HW_HALL_STATE_1            = 1,     /* 001 */
    HW_HALL_STATE_2            = 2,     /* 010 */
    HW_HALL_STATE_3            = 3,     /* 011 */
    HW_HALL_STATE_4            = 4,     /* 100 */
    HW_HALL_STATE_5            = 5,     /* 101 */
    HW_HALL_STATE_6            = 6,     /* 110 */
    HW_HALL_STATE_INVALID_HIGH = 7,     /* 111: 非法 */
} hw_hall_state_t;

/* ============================================================
 * 回调函数类型
 * ============================================================
 *
 * @param self      Hall 设备对象
 * @param user_data 用户数据（由注册时提供）
 *
 * 注意：回调在 ISR 中执行，必须极快（< 10μs），禁止阻塞
 * ============================================================ */
typedef void (*hw_hall_callback_t)(hw_hall_t *self, void *user_data);

/* ============================================================
 * 设备操作接口
 * ============================================================ */
typedef struct {
    /**
     * @brief 初始化 Hall 设备 [单线程]
     */
    rt_err_t (*init)(hw_hall_t *self);

    /**
     * @brief 反初始化 [单线程]
     */
    rt_err_t (*deinit)(hw_hall_t *self);

    /**
     * @brief 读取当前 Hall 状态 [ISR 安全]
     * @return 0~7
     */
    uint8_t (*read_state)(hw_hall_t *self);
} hw_hall_ops_t;

/* ============================================================
 * 配置结构体（只含静态配置）
 * ============================================================ */
struct hw_hall_cfg {
    const char          *name;
    void                *priv;
};
/* ============================================================
 * 设备对象
 * ============================================================ */
struct hw_hall {
    const hw_hall_cfg_t *cfg;              /* ★ 指向配置表 */
    const hw_hall_ops_t *ops;
    uint8_t             is_ready;

    /* ----- 回调（中断触发时调用） ----- */
    hw_hall_callback_t  callback;
    void               *user_data;

    /* ----- 捕获信息（用于转速计算） ----- */
    volatile uint32_t   last_capture_tick;  /* 上次变化时刻 */
    volatile uint32_t   capture_count;      /* 累计变化次数 */
    volatile uint8_t    last_state;         /* 上次状态（去抖用） */
};

/* ============================================================
 * 标准 API
 * ============================================================ */

/**
 * @brief 初始化 Hall 设备 [单线程]
 */
rt_err_t hw_hall_init(hw_hall_t *self,
                      const hw_hall_cfg_t *cfg,
                      const hw_hall_ops_t *ops);

/**
 * @brief 反初始化 [单线程]
 */
rt_err_t hw_hall_deinit(hw_hall_t *self);

/**
 * @brief 读取当前 Hall 状态 [ISR 安全]
 * @return 0~7
 */
uint8_t hw_hall_read_state(hw_hall_t *self);

/**
 * @brief 注册中断回调 [单线程]
 * @param self      Hall 设备对象
 * @param cb        回调函数
 * @param user_data 用户数据（通常指向 motor_t）
 */
rt_err_t hw_hall_set_callback(hw_hall_t *self,
                               hw_hall_callback_t cb,
                               void *user_data);

/**
 * @brief 获取上次变化时刻 [ISR 安全]
 */
uint32_t hw_hall_get_last_tick(const hw_hall_t *self);

/**
 * @brief 获取累计变化次数 [ISR 安全]
 */
uint32_t hw_hall_get_count(const hw_hall_t *self);

/* ============================================================
 * 内部接口（供 BSP 层 ISR 调用）
 * ============================================================ */

/**
 * @brief Hall 状态变化处理（BSP ISR 中调用）
 * @param self      Hall 设备对象
 * @param new_state 新状态（0~7）
 * @note  [ISR 安全] 记录 tick、去抖、调用回调
 */
void hw_hall_on_edge(hw_hall_t *self, uint8_t new_state);

/* 便捷宏：访问配置 */
#define HW_HALL_NAME(self)  ((self)->cfg->name)

#ifdef __cplusplus
}
#endif

#endif /* __HW_HALL_H */
