/**
 * @file    motor_stall.h
 * @brief   电机堵转保护（电流三档 + 低速判据，OR 逻辑）
 *
 * ============================================================
 * 【为什么需要本模块 —— 与现有 motor_oc.c 的分工】
 * ------------------------------------------------------------
 * motor_oc.c 已实现「三级过流」（警告 / 降功率 / 紧急停机），
 * 但它只做**单一电流维度**判定，且由 motor_update() 在 50ms 周期调用。
 * 本项目的堵转需求比它宽：
 *
 *   ① 电流三档时间不同（55A/10ms、50A/100ms、45A/500ms）
 *      → motor_oc 支持，但阈值/时间来自 motor_oc_cfg_t 硬编码，
 *        不读 sys_config 的 stall_* 项；
 *   ② **转速低于 36r/min 持续 2.5s** 也算堵转
 *      → motor_oc 完全没有这个维度；
 *   ③ 三者是 **OR** 关系，任一满足即停机
 *      → 需要统一收敛点。
 *
 * 因此本模块定位为「堵转判定的唯一入口」：
 *   - 从 sys_config 读 11 项参数（0x0901~0x090B），带硬编码默认值兜底
 *   - 电流档来自 foc_adc（20kHz 采样，本模块按 dt 累加计数器）
 *   - 转速档来自 motor_speed_get()（5ms 窗口）
 *   - 触发后：停机 + 紧急锁刹车 + 置 FAULT
 *   - 松开手柄后自动清故障，再次按键可执行
 *
 * ============================================================
 * 【电流采样的时间基准 —— 为什么用"计数"而不是"毫秒"】
 * ------------------------------------------------------------
 * 需求给的是时间（10ms / 100ms / 500ms / 2500ms），但电流源是
 * 20kHz（50μs 一次）。若用 tick 计时，10ms 只能分辨 2~10 个 tick，
 * 量化误差极大。
 *
 * 做法：把时间换算成"20kHz 采样点个数"
 *   N = ms * 20        （因为 1ms = 20 个采样点）
 *   55A / 10ms  → 200   点
 *   50A / 100ms → 2000  点
 *   45A / 500ms → 10000 点
 * 每采到一个超过阈值的样本，对应计数器 +1；低于阈值则清零。
 * 计数器达到 N 即判定该档触发。
 *
 * 【为什么"低于阈值就清零"而不是"递减"】
 *   需求原文是"持续 X ms"。任何一次低于阈值就说明不是连续的，
 *   必须重新计时（否则间歇性尖峰会被累加成误触发）。
 *
 * ============================================================
 * 【低速判据的分段设计】
 * ------------------------------------------------------------
 * 2500ms 若用 20kHz 计数就是 50000 点，单独一个 uint16 存不下，
 * 且低速判定本身不需要 50μs 精度（机械转速变化远慢于此）。
 *
 * 做法：低速用独立的分段计数，每段 250ms（= 5000 点 @20kHz）
 *   10 段 × 250ms = 2500ms
 * 每段内统计"转速低于阈值"的样本数，超过段内 90% 则本段成立，
 * 连续 10 段成立即触发。这样：
 *   - 单段计数 5000 < 65535，uint16 够用
 *   - 相邻段之间允许短暂超速（机械抖动/换向瞬间），不会误清零
 *
 * ============================================================
 * 【门控条件 —— 防止静止时误报】
 * ------------------------------------------------------------
 * 电机不转且无电流（待机）时，转速 = 0 满足低速判据。
 * 若不加门控，上电 2.5s 后必然误报堵转。
 *
 * 门控（三者同时满足才允许低速判据生效）：
 *   ① PWM 有输出（占空比 > 0，即"正在驱动"）
 *   ② 母线电流 >= stall_en_min_a10（默认 5.0A，说明确实在出力）
 *   ③ 不在缓停止过程（主动减速时转速低是正常的）
 *
 * 电流三档不受门控限制 —— 电流超阈值本身就说明在出力。
 *
 * ============================================================
 * 【本次实现范围（2026-11-05，S1-a/b/c/d 一次性落地）】
 *   ✅ stall_load_cfg()    从 sys_config 读 11 项 + 默认值兜底
 *   ✅ stall_on_current()  20kHz 电流计数（三档独立）
 *   ✅ stall_on_tick()     5ms 周期：低速分段 + 电压门控 + 自动清除
 *   ✅ stall_trigger()     停机 + 锁刹车 + FAULT
 *   ✅ t_stall             诊断打印
 *
 *   ⏸ 与键盘/事件总线的联动（HORI_EVT_STALL_FAULT / kb_request_rebaseline）
 *      留待 M 批统一接入 —— 本模块只提供 motor_stall_is_faulted() 查询，
 *      不主动向事件总线发消息，避免与现有 ctrl_manager 的路由缺口冲突。
 * ============================================================
 */

#ifndef __MOTOR_STALL_H
#define __MOTOR_STALL_H

#include <stdint.h>
#include <rtthread.h>

#include "motor.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 一、硬编码默认值（sys_config 读不到时兜底）
 * ------------------------------------------------------------
 * 数值来源：2026-11-04 用户给定的堵转保护策略
 *   55A/10ms、50A/100ms、45A/500ms、36r/min 持续 2.5s
 *   额定 3000r/min、额定电流 37A
 * 全部为 0.1A / 整数 r/min 单位，禁浮点（Q-63 未批准）。
 * ============================================================ */
#define STALL_DEF_OC3_A10           (550U)      /* 55.0 A */
#define STALL_DEF_OC3_MS            (10U)
#define STALL_DEF_OC2_A10           (500U)      /* 50.0 A */
#define STALL_DEF_OC2_MS            (100U)
#define STALL_DEF_OC1_A10           (450U)      /* 45.0 A */
#define STALL_DEF_OC1_MS            (500U)
#define STALL_DEF_LOW_RPM           (36U)       /* 低于 36 r/min */
#define STALL_DEF_LOW_MS            (2500U)     /* 持续 2.5 s */
#define STALL_DEF_EN_MIN_A10        (50U)       /* 5.0 A 门控 */
#define STALL_DEF_MOTOR_RATED_A10   (370U)      /* 37.0 A */
#define STALL_DEF_AUTO_CLEAR_MS     (0U)        /* 0 = 立即清除 */

/* 20kHz 采样率：1ms = 20 个样本点 */
#define STALL_SAMPLE_PER_MS         (20U)

/* 低速判据分段：10 段 × 250ms = 2500ms（可在 low_ms 变化时自适应） */
#define STALL_LOW_SEG_MAX           (16U)
#define STALL_LOW_SEG_MS            (250U)

/* 定时器周期（ms）。由 bg_sched / motor_periodic_task 提供 */
#define STALL_TICK_MS               (5U)

/* ============================================================
 * 0.1A 阈值 → raw 域"偏离量"阈值的编译期换算
 * ------------------------------------------------------------
 * 【2026-11-09 重算】原宏基于错误的 a10 公式（*1000）推导，
 * 得到 55A→dev 181，比正确值小 10 倍，会导致堵转保护在
 * 极低电流（约 5.5A）就误触发。必须按正确公式重推。
 *
 * 正确的前向式（bsp_adc_foc.c:160）：
 *   a10 = (raw*3300 - 6758400) * 100 / 1081344
 *
 * 反解：
 *   raw*3300 - 6758400 = a10*1081344 / 100
 *   raw = (a10*1081344 + 100*6758400) / (100*3300)
 *       = (a10*1081344 + 675840000) / 330000
 *   三者 gcd = 528，约分后：
 *     raw = (2048*a10 + 1280000) / 625
 *
 * 溢出：最坏分子 2,406,400 ≪ 2^32 ✓
 * 向上取整（+624 再除）：保证阈值不偏松，宁可早触发不漏判。
 *
 * 校验（Python 核算，2026-11-09，与正确公式逐点对齐）：
 *   55A → raw 3851 → dev 1803
 *   50A → raw 3687 → dev 1639
 *   45A → raw 3523 → dev 1475
 *   37A → raw 3261 → dev 1213
 *    5A → raw 2212 → dev  164
 * ============================================================ */
#define STALL_A10_TO_DEV_RAW(a10)                                   \
    ((uint16_t)((((uint32_t)(a10) * 2048UL) + 1280000UL + 624UL) /  \
                625UL) - 2048U)

/* ============================================================
 * 二、电路/触发原因位（诊断用，可组合）
 * ============================================================ */
#define STALL_CAUSE_NONE            (0x00U)
#define STALL_CAUSE_OC3             (0x01U)     /* 55A/10ms   */
#define STALL_CAUSE_OC2             (0x02U)     /* 50A/100ms  */
#define STALL_CAUSE_OC1             (0x04U)     /* 45A/500ms  */
#define STALL_CAUSE_LOW_RPM         (0x08U)     /* 36rpm/2.5s */

/* ============================================================
 * 三、运行时配置（从 sys_config 装载）
 * ============================================================ */
typedef struct
{
    uint16_t oc3_a10;
    uint16_t oc3_ms;
    uint16_t oc2_a10;
    uint16_t oc2_ms;
    uint16_t oc1_a10;
    uint16_t oc1_ms;
    uint16_t low_rpm;
    uint16_t low_ms;
    uint16_t en_min_a10;
    uint16_t rated_a10;
    uint16_t auto_clear_ms;

    /* ---- 派生量 ①：ms → 20kHz 样本数 ----
     * 避免运行时反复乘除 */
    uint16_t oc3_cnt_max;
    uint16_t oc2_cnt_max;
    uint16_t oc1_cnt_max;
    uint16_t low_seg_cnt_max;   /* 单段样本数上限 = low_seg_ms * 20 */
    uint8_t  low_seg_total;     /* 需要的连续段数 = low_ms / low_seg_ms */

    /* ---- 派生量 ②：a10 阈值 → raw 域阈值 ----
     * 【为什么必须有这一组】
     *   stall_on_current() 在 **20kHz 中断**里调用。若每帧都调
     *   foc_adc_get_bus_current_a10()，内部那次 int64 除法要 60~100 周期，
     *   20kHz × 80 周期 ≈ 1.6% CPU，且拉长 ISR。
     *   因此把阈值预先换算到 raw 域，ISR 内只做整数比较、零除法。
     *
     * 换算公式（由 foc_adc_raw_to_a10 反解，向上取整）：
     *   raw = (a10*1081344 + 1000*6758400) / (1000*3300)
     *        = (a10*1081344 + 6758400000) / 3300000
     * 再减去零点 FOC_CUR_ZERO_RAW(=2048)，得到"偏离量阈值"，
     * 与 foc_adc_get_bus_current_dev_raw() 的返回值域一致。 */
    uint16_t oc3_dev_raw;       /* 55A → 偏离 181 */
    uint16_t oc2_dev_raw;       /* 50A → 偏离 164 */
    uint16_t oc1_dev_raw;       /* 45A → 偏离 148 */
    uint16_t en_min_dev_raw;    /* 5A  → 偏离 17  */
} motor_stall_cfg_t;

/* ============================================================
 * 四、运行时状态 / 统计（供 t_stall 打印）
 * ============================================================ */
typedef struct
{
    /* 电流三档计数器（20kHz 域） */
    uint16_t oc3_cnt;
    uint16_t oc2_cnt;
    uint16_t oc1_cnt;

    /* 低速分段 */
    uint16_t low_seg_cnt;       /* 本段内"低于阈值"的样本数 */
    uint16_t low_seg_samples;   /* 本段已累计的总样本数 */
    uint8_t  low_seg_ok;        /* 本段已成立的连续段数 */

    /* 门控状态 */
    uint8_t  gate_driving;      /* 1 = 占空比 > 0 */
    uint8_t  gate_current;      /* 1 = 电流 >= en_min */
    uint8_t  gate_not_ramping;  /* 1 = 不在缓停止 */

    /* 故障与清除 */
    uint8_t  faulted;           /* 1 = 已触发堵转 */
    uint8_t  cause;             /* STALL_CAUSE_xxx 位组合 */
    uint32_t trigger_count;     /* 累计触发次数 */
    uint32_t last_trigger_tick; /* 最近触发时刻 */
    uint32_t clear_wait_tick;   /* 自动清除等待起点，0=未在等 */

    /* 本次触发时的快照（便于事后分析） */
    uint16_t snap_current_a10;  /* 触发瞬间的母线电流（母线单向，无符号） */
    int32_t  snap_rpm_x100;
    uint32_t snap_tick;
} motor_stall_ctx_t;

/* ============================================================
 * 五、API
 * ============================================================ */

/**
 * @brief  初始化堵转保护（从 sys_config 装载参数）
 *
 * @param  motor  关联的电机对象（触发时用于 motor_stop）。可为 NULL。
 *                类型为 void* 是为了让 boot_table.c 不必 include motor.h。
 * @retval 0 成功
 *
 * @note   必须在 sys_config_init（boot 第 4 步）之后调用。
 *         本函数内部逐项 try-read，任何一项读失败都回退到 STALL_DEF_*，
 *         保证即使配置表损坏也能保护（fail-safe）。
 *
 *         重复调用安全（内部有 inited 短路，仅更新 motor 指针）。
 */
int motor_stall_init(void *motor);

/**
 * @brief  从 sys_config 重新装载参数（热更新）
 *
 * @note   t_cfg set 之后调用即可生效，无需重启。
 *         只重算派生量，不改变当前故障状态与计数器。
 */
void stall_load_cfg(void);

/**
 * @brief  20kHz 电流采样点（由 ADC 通路每帧调用一次）
 *
 * @param  i_bus_dev_raw  母线电流正向偏离量（raw - zero，**无符号**）
 *                        由 foc_adc_get_bus_current_dev_raw() 取得
 *
 * @note   调用上下文：**20kHz 中断**。内部只做整数比较与自增，
 *         无除法、无打印、无阻塞，最坏路径约 20 周期。
 *
 *         【2026-11-09 类型裁定：无符号】
 *           母线电流物理上不反向（直流母线经前级整流，本机无回馈通路），
 *           故偏离量天然非负。foc_adc_get_bus_current_dev_raw()
 *           在 raw <= zero 时返回 0，本函数无需再取绝对值。
 *
 *           需要符号的是 IV/IW/IU（相/线电流），
 *           它们在 foc_adc_read() 里保持 int16，与本函数无关。
 *
 *         三档独立计数：某一档满足即触发，不要求其他档满足。
 *         任一样本低于该档阈值 → 该档计数器**清零**（"持续"语义）。
 *
 *         未初始化时安全返回（PWM ISR 可能早于本模块初始化就进来）。
 */
void stall_on_current(uint16_t i_bus_dev_raw);

/**
 * @brief  周期任务（建议 5ms，由 bg_sched 或 motor_periodic_task 调用）
 *
 * @note   职责：
 *         ① 低速判据的分段累计与成立判定
 *         ② 门控条件刷新（占空比 / 电流 / 缓停止）
 *         ③ 自动清除故障（松开手柄后）
 *
 *         为什么低速判据不放在 20kHz 里：
 *           转速源本身是 5ms 窗口（motor_speed.c），
 *           在 20kHz 里读只会得到同一个值的 100 次重复。
 */
void stall_on_tick(void);

/**
 * @brief  查询是否处于堵转故障
 * @retval 1 = 故障中，0 = 正常
 */
uint8_t motor_stall_is_faulted(void);

/**
 * @brief  取触发原因位（STALL_CAUSE_xxx）
 */
uint8_t motor_stall_get_cause(void);

/**
 * @brief  取完整上下文（只读，诊断用）
 */
const motor_stall_ctx_t *motor_stall_get_ctx(void);

/**
 * @brief  手动清除故障（MSH / 上层业务调用）
 * @note   会同时复位所有计数器与门控状态，回到"未触发"初态。
 *         保留 trigger_count / last_trigger_tick 等历史统计，便于追溯。
 *         若电机处于 MOTOR_STATE_FAULT，会一并恢复为 IDLE，
 *         使"松开手柄后故障清除、再次按键可执行"的流程成立。
 */
void motor_stall_clear(void);

/**
 * @brief  取最近一次母线电流（0.1A，无符号），供触发快照使用
 * @note   内部转调 foc_adc_get_bus_current_a10()。
 *         母线电流物理上不反向，故为无符号；相/线电流才有符号。
 */
uint16_t motor_stall_get_last_current(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_STALL_H */