/**
 * @file    motor_angle.h
 * @brief   电机轴虚拟角度（0.001 度，int32）
 *
 * ============================================================
 * 【定位】
 * 把 Z0 磁编码器的 12bit 角度，累积成一个**连续、多圈、带符号**
 * 的电机轴角度变量，供测速、位置环、FOC 电角度使用。
 *
 * ============================================================
 * 【上电 vs 运行期，两条不同的数据来源】
 * ------------------------------------------------------------
 *   上电（一次）：
 *     三颗编码器游标解算 → abs_angle_solve()
 *     → 得到 Z0 绝对圈数 k → cnt_accum = k*4096 + raw0
 *     → 换算成电机轴角度
 *
 *   运行期（周期）：
 *     只读 Z0 一颗 → 与上次比较得增量 → 累加
 *     **不再用三颗解算**（用户 2026-11-02 明确要求）
 *
 * 为什么运行期不用三颗：
 *   三颗解算要 6 次 SPI 读 + 游标搜索，耗时约 100μs；
 *   单读 Z0 只要 11μs。且游标解算依赖齿数比，任一齿轮打滑就会
 *   跳变；增量法只依赖单颗的连续性，更鲁棒。
 *
 * ============================================================
 * 【单位与量程】
 * ------------------------------------------------------------
 *   单位：0.001 度（mdeg），与 mech_cfg 的 mdeg 口径一致
 *   Z0 与电机轴 1:1 直连（用户确认），故 1 Z0 转 = 1 电机转
 *
 *   量程核查：
 *     卸粮筒行程 360°，经 i_total=1080 减速
 *     → 电机轴最多 360 × 1080 = 388,800 度 = 388,800,000 mdeg
 *     → int32 上限 2,147,483,647，占 18%，余量充足
 *
 *   1 LSB (12bit) = 360000/4096 = 87.89 mdeg
 *
 * ============================================================
 * 【过圈保护 —— 本模块最关键的安全设计】
 * ------------------------------------------------------------
 * 增量法靠"本次读数 - 上次读数"，用 ±2048 环绕归一判断方向。
 * 但若一个采样周期内转过超过半圈（2048 LSB），方向判断出现
 * 二义性（+190° 与 -170° 无法区分），累加就会错。
 *
 * 采样周期 1ms 时：
 *     半圈上限 rpm = (2048/4096) / 0.001 × 60 = 30000 RPM
 *     实际最高 3500 RPM（14V）→ 每周期仅 239 LSB → 余量 8.6 倍 ✓
 *
 * 即便如此，代码仍保留显式检测：|delta| 接近 2047 时不累加，
 * 计入 wrap_risk 计数并返回错误。因为采样周期可能被以下原因拉长：
 *   - 高优先级中断长时间占用 CPU
 *   - 调试器暂停
 *   - 任务调度异常
 * 一旦出现，宁可报错也不能污染角度累加。
 * ============================================================
 */

#ifndef __MOTOR_ANGLE_H
#define __MOTOR_ANGLE_H

#include <rtthread.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 常量
 * ============================================================ */

/** 编码器单圈计数（12bit） */
#define MOTOR_ANGLE_ENC_FULL        (4096)

/** 单圈对应毫度 */
#define MOTOR_ANGLE_DEG_PER_REV     (360000)

/** 12bit LSB 对应的毫度（360000/4096 = 87.890625）
 *  用整数表示：mdeg = cnt * 360000 / 4096 */
#define MOTOR_ANGLE_MDEG_NUM        (360000L)
#define MOTOR_ANGLE_MDEG_DEN        (4096L)

/** 增量更新周期（毫秒）
 *
 * 【为什么是 1ms】
 *   过圈余量：30000 RPM 上限 vs 3500 RPM 实际 = 8.6 倍
 *   CPU 开销：11μs/1ms = 1.1%（含 CS 延时约 1.4%）
 *   比 2.5ms 更安全，比 50μs(FOC) 更省 CPU 且精度更高
 *
 * 【为什么不跟 FOC 的 50μs】
 *   50μs 下每周期仅转 12 LSB，±1 LSB 量化误差 = ±292 RPM，
 *   转速精度不可用；且 SPI 占 22% CPU。
 *   FOC 需要的电角度用本模块的角度 + 角速度外推即可，
 *   不必在 PWM 中断里读 SPI。 */
#define MOTOR_ANGLE_PERIOD_MS       (1U)

/* ============================================================
 * 错误码
 * ============================================================ */
typedef enum {
    MOTOR_ANGLE_OK          = 0,
    MOTOR_ANGLE_ERR_NOT_INIT = 1,   /* 未初始化 / 未解算 */
    MOTOR_ANGLE_ERR_NOT_CAL  = 2,   /* 未校准 */
    MOTOR_ANGLE_ERR_ENC_READ = 3,   /* Z0 读失败 */
    MOTOR_ANGLE_ERR_WRAP_RISK= 4,   /* 单周期转过半圈，过圈风险 */
} motor_angle_err_t;

/* ============================================================
 * 状态（诊断用）
 * ============================================================ */
typedef struct {
    int32_t  angle_mdeg;        /* 当前电机轴角度，0.001 度 */
    int32_t  cnt_accum;         /* Z0 累计计数（含多圈） */
    int32_t  last_raw;          /* 上次 Z0 原始值，-1 表示未采 */
    uint32_t update_cnt;        /* 更新次数 */
    uint32_t err_cnt;           /* 读失败次数 */
    uint32_t wrap_risk_cnt;     /* 过圈风险次数 */
    uint32_t max_delta;         /* 观察到的最大单周期增量（绝对值） */
    uint8_t  inited;            /* 1 = 已建立基线 */

    /* ---- 【2026-11-02 新增】静默失败不再静默 ---- */
    uint32_t not_init_cnt;      /* 因未解算而被跳过的次数（原是静默 return） */
    uint32_t call_cnt;          /* update 被调用总次数 */
    uint32_t last_err;          /* 最近一次错误码（motor_angle_err_t） */
    uint32_t still_cnt;         /* delta==0 的次数（静止/量化不足） */

    /* ---- 【2026-11-03 新增】零基点降级标记 ---- */
    uint8_t  degenerate;        /* 1 = 解算失败，当前是 0°相对基准，绝对角不可信 */
    int32_t  z0_raw0;           /* 建立基线时的 Z0 原始值，便于事后追溯 */
} motor_angle_state_t;

/* ============================================================
 * API
 * ============================================================ */

/**
 * @brief 初始化模块（不碰硬件，只清状态）
 * @note  必须在 app 启动早期调用，早于首次 motor_angle_update
 */
void motor_angle_init(void);

/**
 * @brief 上电解算：用三颗编码器建立绝对角度基线
 * @retval MOTOR_ANGLE_OK 成功
 * @note   内部调 abs_angle_solve()，再取 cnt_accum 换算角度。
 *         依赖 mech_cfg 已加载、三颗编码器可读。
 *         必须在 motor_angle_init() 之后调用。
 */
motor_angle_err_t motor_angle_solve(void);

/**
 * @brief 周期增量更新（只读 Z0 一颗）
 * @retval 错误码；MOTOR_ANGLE_OK 表示本次更新成功
 * @note   由 1ms 定时器调用。
 *         未初始化时返回 NOT_INIT，不读硬件。
 *         检测到过圈风险时返回 WRAP_RISK 且**不累加**。
 */
motor_angle_err_t motor_angle_update(void);

/**
 * @brief 取当前电机轴角度
 * @param[out] angle_mdeg  0.001 度
 * @retval 错误码
 */
motor_angle_err_t motor_angle_get(int32_t *angle_mdeg);

/**
 * @brief 取状态快照
 */
void motor_angle_get_state(motor_angle_state_t *out);

/**
 * @brief 取 Z0 累计计数（供 FOC 电角度计算）
 */
int32_t motor_angle_get_cnt(void);

/**
 * @brief 计数器 → 毫度（纯换算，无副作用）
 */
static inline int32_t motor_angle_cnt_to_mdeg(int32_t cnt)
{
    /* 用 int64 防溢出：cnt 最大近 4.4e6，乘 360000 = 1.6e12 */
    return (int32_t)(((int64_t)cnt * (int64_t)MOTOR_ANGLE_MDEG_NUM)
                     / (int64_t)MOTOR_ANGLE_MDEG_DEN);
}

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_ANGLE_H */