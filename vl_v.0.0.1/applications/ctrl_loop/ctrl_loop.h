/**
 * @file    ctrl_loop.h
 * @brief   通用三环控制模块（位置/速度/电流），支持多实例
 *
 * @note    【设计目标】
 *          1. 一个模块可创建多个实例，分别控制不同对象（回转电机、举升机构…）
 *          2. 模块本身不关心"控制对象是什么"，通过函数指针注入反馈与输出
 *          3. PID 参数、使能、调度周期全部走 sys_config，可存 flash、可 MSH 读写
 *          4. 三环按各自周期独立调度，通过 target 链式传递（上一环输出→下一环目标）
 *
 *          【为什么不在 bg_scheduler 里注册】
 *            bg_scheduler 只遍历静态数组 g_bg_modules[]（bg_scheduler.c:96），
 *            无法运行时动态注册实例。且 bg_sched 线程优先级 15（最低），
 *            速度环 1ms 要求会被其他任务抢占。
 *            故本模块自带 tick 入口，由调用方在合适的时基里驱动
 *            （建议：独立线程 prio 6，1ms 周期）。
 *
 *          【定点运算】
 *            本模块不使用 float —— Q-63 的 float 豁免范围仅
 *            motor/foc/ 等 3 处，本模块不在豁免内，
 *            必须遵守 STD-FIXEDPOINT。所有系数用 *100 定点，
 *            与工程已有的 cur_gain_x100 / i_total_x100 约定一致。
 */

#ifndef __CTRL_LOOP_H
#define __CTRL_LOOP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 1. 环类型
 * ============================================================ */
typedef enum {
    CTRL_LOOP_POSITION = 0,     /* 位置环，最低频 */
    CTRL_LOOP_SPEED,            /* 速度环 */
    CTRL_LOOP_CURRENT,          /* 电流环，最高频（当前留接口不接） */
    CTRL_LOOP_RING_MAX
} ctrl_loop_kind_t;

/* ============================================================
 * 2. 实例数量
 * ------------------------------------------------------------
 * 【为什么是 4】
 *   当前需求：回转电机 + 举升机构 = 2 个
 *   预留：将来可能加搅龙、卸粮离合 = 共 4 个
 *   配置表占用：4 × 13 = 52 项（见 ctrl_loop_cfg.h）
 * ============================================================ */
#define CTRL_LOOP_MAX_INST      4U

/* ============================================================
 * 3. 定点 PID 参数
 * ------------------------------------------------------------
 * kp_x100 = 100 表示 kp = 1.00
 * 为什么 *100 而不是 *1000：与工程既有约定一致
 *   （cur_gain_x100=2640 表示 26.40、i_total_x100、tol_override_x1000）
 *   现场调参时不用额外换算。
 * ============================================================ */
typedef struct {
    int32_t kp_x100;        /* 比例系数 ×100 */
    int32_t ki_x100;        /* 积分系数 ×100 */
    int32_t kd_x100;        /* 微分系数 ×100 */

    int32_t out_max;        /* 输出上限（工程量，与 set_output 同单位） */
    int32_t out_min;        /* 输出下限 */
    int32_t integ_max;      /* 积分限幅（防积分饱和） */

    uint8_t enable;         /* 0=旁路（目标直接透传到下一环），1=启用 */
    uint8_t d_on_measure;   /* 1=微分作用在测量值（避免设定值阶跃冲击） */
    uint8_t rsv[2];
} ctrl_pid_cfg_t;

/* ============================================================
 * 4. 单环运行时状态
 * ============================================================ */
typedef struct {
    ctrl_pid_cfg_t cfg;     /* PID 参数（从 sys_config 装载） */

    /* ---- 运行时量 ---- */
    int32_t  integ;         /* 积分累积值 */
    int32_t  last_meas;     /* 上次测量值（微分用） */
    int32_t  last_err;      /* 上次误差（d_on_measure=0 时用） */
    int32_t  last_out;      /* 上次输出 */
    int32_t  err;           /* 当前误差（诊断） */

    /* ---- 调度 ---- */
    uint32_t period_ms;     /* 本环周期 */
    uint32_t last_tick;     /* 上次执行时刻（tick） */

    /* ---- 统计（诊断整定用）---- */
    uint32_t run_cnt;       /* 执行次数 */
    uint32_t sat_cnt;       /* 输出饱和次数 */
    uint32_t skip_cnt;      /* 因未到期跳过次数 */
} ctrl_loop_ring_t;

/* ============================================================
 * 5. 三环实例
 * ============================================================ */
struct ctrl_loop;

/**
 * @brief 读反馈值
 * @param self  实例指针
 * @param kind  哪个环（各环反馈量不同：位置=角度、速度=rpm、电流=A）
 * @retval 反馈值，单位为该环的工程量
 * @note   返回 0 表示无数据；模块不区分"真 0"与"无数据"，
 *         如需区分请在实现里通过 out 参数或 user_data 传递状态。
 */
typedef int32_t (*ctrl_loop_get_fb_t)(struct ctrl_loop *self,
                                      ctrl_loop_kind_t kind);

/**
 * @brief 写输出值
 * @param self  实例指针
 * @param kind  哪个环
 * @param out   输出值（已限幅），含义由实现决定（占空比 / 目标速度 / 目标电流）
 */
typedef void (*ctrl_loop_set_out_t)(struct ctrl_loop *self,
                                    ctrl_loop_kind_t kind,
                                    int32_t out);

typedef struct ctrl_loop {
    const char *name;           /* 实例名（诊断用） */

    /* ---- 注入的钩子 ---- */
    ctrl_loop_get_fb_t  get_fb;
    ctrl_loop_set_out_t set_out;
    void               *user_data;  /* 使用者私有上下文 */

    /* ---- 三环 ---- */
    ctrl_loop_ring_t    ring[CTRL_LOOP_RING_MAX];

    /* ---- 目标值（外部写入，位置环通常被上层写；速度/电流环由上一环写） ---- */
    int32_t             target[CTRL_LOOP_RING_MAX];

    /* ---- 前馈（叠加在 PID 输出上，通常 0） ---- */
    int32_t             ff[CTRL_LOOP_RING_MAX];

    /* ---- 状态 ---- */
    uint8_t  enabled;           /* 总开关：0=全部旁路 */
    uint8_t  inited;
    uint8_t  rsv[2];
} ctrl_loop_t;

/* ============================================================
 * 6. API
 * ============================================================ */

/**
 * @brief 初始化一个实例
 *
 * @param loop      实例（由调用方静态分配，模块不 malloc）
 * @param name      实例名
 * @param get_fb    读反馈钩子
 * @param set_out   写输出钩子
 * @param user_data 私有上下文（会传给钩子）
 * @retval 0 成功；-1 参数错
 *
 * @note  初始化后各环参数为 0、enable=0（旁路），
 *        必须再调 ctrl_loop_load_cfg() 从 sys_config 装载参数，
 *        或通过 ctrl_loop_set_pid() 手工设置。
 */
int ctrl_loop_init(ctrl_loop_t *loop,
                   const char *name,
                   ctrl_loop_get_fb_t get_fb,
                   ctrl_loop_set_out_t set_out,
                   void *user_data);

/**
 * @brief 周期驱动：按各环 period_ms 分频执行三环
 *
 * @param loop  实例
 * @param now_tick 当前 tick（1ms 分辨率）
 *
 * @note  调用方保证 1ms 调用一次（或用更快时基配合分频）。
 *        内部对每个环判 (now_tick - last_tick >= period_ms)，
 *        未到期直接返回，开销约 3 次比较。
 *
 *        ⚠ 本函数在**线程上下文**执行，不在中断里。
 *          电流环若要走 20kHz，应由 PWM ISR 单独调 ctrl_loop_current_step()。
 */
void ctrl_loop_tick(ctrl_loop_t *loop, uint32_t now_tick);

/**
 * @brief 电流环单步（供 20kHz ISR 调用，不经 ctrl_loop_tick 分频）
 *
 * @param loop  实例
 *
 * @note  当前工程是六步方波驱动，无 SVPWM，本函数为**预留接口**。
 *        由 ctrl_loop_tick 判定 CTRL_LOOP_CURRENT 是否启用；
 *        若启用则本函数由 PWM ISR 调，tick 里跳过电流环。
 *        ⚠ 见文件头"电流环预留"标记。
 */
void ctrl_loop_current_step(ctrl_loop_t *loop);

/**
 * @brief 设置某环的 PID 参数（不落盘）
 */
void ctrl_loop_set_pid(ctrl_loop_t *loop, ctrl_loop_kind_t kind,
                       const ctrl_pid_cfg_t *cfg);

/**
 * @brief 读取某环的 PID 参数
 */
void ctrl_loop_get_pid(const ctrl_loop_t *loop, ctrl_loop_kind_t kind,
                       ctrl_pid_cfg_t *out);

/**
 * @brief 设置某环周期（ms）
 * @retval 0 成功；-1 非法（周期必须 >=1）
 */
int ctrl_loop_set_period(ctrl_loop_t *loop, ctrl_loop_kind_t kind,
                         uint32_t period_ms);

/**
 * @brief 清积分与统计（换目标/换模式后调用，避免残留积分冲击）
 */
void ctrl_loop_reset(ctrl_loop_t *loop, ctrl_loop_kind_t kind);

/**
 * @brief 取实例指针（供 MSH 按序号访问）
 * @retval NULL 表示该序号未注册
 */
ctrl_loop_t *ctrl_loop_get(uint8_t inst);
uint8_t      ctrl_loop_inst_count(void);

/**
 * @brief 环类型名（诊断用）
 */
const char *ctrl_loop_kind_name(ctrl_loop_kind_t kind);

/**
 * @brief 定点 PID 单步（公开以便单元测试）
 *
 * @param ring    环状态
 * @param target  目标
 * @param meas    测量
 * @param dt_ms   距上次的间隔（ms）
 * @retval 限幅后的输出
 */
int32_t ctrl_loop_pid_step(ctrl_loop_ring_t *ring,
                           int32_t target, int32_t meas, uint32_t dt_ms);

#ifdef __cplusplus
}
#endif

#endif /* __CTRL_LOOP_H */