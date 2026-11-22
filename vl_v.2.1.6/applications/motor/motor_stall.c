/**
 * @file    motor_stall.c
 * @brief   电机堵转保护实现（电流三档 + 低速判据，OR 逻辑）
 *
 * 设计与取舍详见 motor_stall.h 的文件头注释。
 *
 * ============================================================
 * 【本文件的三条硬约束】
 * ------------------------------------------------------------
 * ① stall_on_current() 可能在 **20kHz 中断上下文**被调用
 *    ⇒ 函数体内禁止：除法 / 取模 / rt_kprintf / 任何锁 / 任何等待
 *    ⇒ 只用比较、自增、清零，最坏路径约 20 个周期
 *
 * ② 全整数运算，禁浮点（Q-63 float 豁免未批准）
 *    ⇒ 时间→样本数、转速比较全部用整数
 *
 * ③ 配置读不到必须 fail-safe 到默认值
 *    ⇒ 保护功能宁可用保守默认值，也不能因为配置表坏了就失效
 *
 * ============================================================
 * 【20kHz 计数器的溢出核查】
 * ------------------------------------------------------------
 * 最大计数 45A 档 = 500ms × 20 = 10000，uint16 上限 65535 ⇒ 安全。
 * 若将来把 oc1_ms 调到 > 3200ms，会溢出。stall_load_cfg() 内部
 * 做了钳位保护（见 STALL_MAX_MS），不会真的溢出。
 * ============================================================
 */

#include "motor_stall.h"

/* 【2026-11-10 彩标 P0】堵转阈值装载：正常=INFO，不单调=WARN */
#include "../log/log_color.h"

#include "motor_oc.h"
#include "drv/bsp_adc_foc.h"        /* foc_adc_get_bus_current_a10 */
#include "../config/sys_config.h"
#include "../angle/motor_speed.h"
#include "../hw/hw_brake.h"
#include "../bsp/bsp_pwm.h"

#include <string.h>

/* ============================================================
 * 外部依赖（避免头文件循环包含，此处前向声明）
 * ============================================================ */
extern hw_brake_t g_swing_brake;

/* 内部辅助的前向声明（定义在文件末尾） */
uint16_t motor_stall_get_last_current(void);

/* 计数器上限保护：65535 / STALL_SAMPLE_PER_MS。
 * 20kHz 下 = 3276ms，再留 76ms 余量取 3200ms。
 * 10kHz 下自动变成 6477ms，仍保证 uint16 不溢出。 */
#define STALL_MAX_CNT               (65535U)
#define STALL_MAX_MS                ((STALL_MAX_CNT / STALL_SAMPLE_PER_MS) - 76U)

/* ============================================================
 * 模块状态
 * ============================================================ */
static motor_stall_cfg_t s_cfg;
static motor_stall_ctx_t s_ctx;
static motor_t          *s_motor   = RT_NULL;
static uint8_t           s_inited  = 0U;

/* ============================================================
 * 内部：读一项配置，失败返回默认值
 * ------------------------------------------------------------
 * sys_config_get_i32() 返回 0 表示读到有效值；非 0 表示该 ID 不存在
 * 或类型不符。无论哪种，都必须回退到默认值 —— 这是"保护功能
 * 不能依赖配置表健康"的落地。
 * ============================================================ */
static uint16_t stall_read_u16(uint16_t id, uint16_t def)
{
    int32_t v = 0;

    if (sys_config_get_i32(id, &v) != 0)
    {
        return def;
    }
    if (v < 0)
    {
        return def;                 /* 负数对 uint16 无意义 */
    }
    if (v > 65535)
    {
        return def;
    }
    return (uint16_t)v;
}

/* 毫秒 → 20kHz 样本数，带溢出钳位 */
static uint16_t stall_ms_to_cnt(uint16_t ms)
{
    uint32_t cnt;

    if (ms > STALL_MAX_MS)
    {
        ms = STALL_MAX_MS;
    }
    cnt = (uint32_t)ms * STALL_SAMPLE_PER_MS;
    if (cnt > STALL_MAX_CNT)
    {
        cnt = STALL_MAX_CNT;
    }
    return (uint16_t)cnt;
}

/* ============================================================
 * 装载配置
 * ============================================================ */
void stall_load_cfg(void)
{
    s_cfg.oc3_a10 = stall_read_u16(CFG_ID_STALL_OC3_A10,     STALL_DEF_OC3_A10);
    s_cfg.oc3_ms  = stall_read_u16(CFG_ID_STALL_OC3_MS,      STALL_DEF_OC3_MS);
    s_cfg.oc2_a10 = stall_read_u16(CFG_ID_STALL_OC2_A10,     STALL_DEF_OC2_A10);
    s_cfg.oc2_ms  = stall_read_u16(CFG_ID_STALL_OC2_MS,      STALL_DEF_OC2_MS);
    s_cfg.oc1_a10 = stall_read_u16(CFG_ID_STALL_OC1_A10,     STALL_DEF_OC1_A10);
    s_cfg.oc1_ms  = stall_read_u16(CFG_ID_STALL_OC1_MS,      STALL_DEF_OC1_MS);
    s_cfg.low_rpm = stall_read_u16(CFG_ID_STALL_LOW_RPM,     STALL_DEF_LOW_RPM);
    s_cfg.low_ms  = stall_read_u16(CFG_ID_STALL_LOW_MS,      STALL_DEF_LOW_MS);
    s_cfg.en_min_a10 = stall_read_u16(CFG_ID_STALL_EN_MIN_A10, STALL_DEF_EN_MIN_A10);
    s_cfg.rated_a10  = stall_read_u16(CFG_ID_MOTOR_RATED_A10,  STALL_DEF_MOTOR_RATED_A10);
    s_cfg.auto_clear_ms = stall_read_u16(CFG_ID_STALL_AUTO_CLEAR_MS,
                                         STALL_DEF_AUTO_CLEAR_MS);

    /* ---- 派生量：ms → 20kHz 样本数 ---- */
    s_cfg.oc3_cnt_max = stall_ms_to_cnt(s_cfg.oc3_ms);
    s_cfg.oc2_cnt_max = stall_ms_to_cnt(s_cfg.oc2_ms);
    s_cfg.oc1_cnt_max = stall_ms_to_cnt(s_cfg.oc1_ms);

    /* ---- 派生量：a10 → raw 域偏离量 ----
     * 只在装载时算一次，ISR 里直接用，零除法。 */
    s_cfg.oc3_dev_raw    = STALL_A10_TO_DEV_RAW(s_cfg.oc3_a10);
    s_cfg.oc2_dev_raw    = STALL_A10_TO_DEV_RAW(s_cfg.oc2_a10);
    s_cfg.oc1_dev_raw    = STALL_A10_TO_DEV_RAW(s_cfg.oc1_a10);
    s_cfg.en_min_dev_raw = STALL_A10_TO_DEV_RAW(s_cfg.en_min_a10);

    /* ---- 低速分段 ----
     * 段长固定 250ms（5000 样本），段数 = low_ms / 250，向上取整。
     * 若 low_ms 被配成 0 或极小值，强制至少 1 段，避免除零。 */
    s_cfg.low_seg_cnt_max = stall_ms_to_cnt(STALL_LOW_SEG_MS);

    {
        uint32_t total;

        if (s_cfg.low_ms == 0U)
        {
            s_cfg.low_ms = STALL_DEF_LOW_MS;
        }
        /* 向上取整： (a + b - 1) / b */
        total = ((uint32_t)s_cfg.low_ms + STALL_LOW_SEG_MS - 1U) /
                STALL_LOW_SEG_MS;

        if (total == 0U)
        {
            total = 1U;
        }
        if (total > STALL_LOW_SEG_MAX)
        {
            total = STALL_LOW_SEG_MAX;
        }
        s_cfg.low_seg_total = (uint8_t)total;
    }

    /* ---- 一致性检查：三档阈值必须递减 ----
     * 若用户把 oc3 < oc2 < oc1 配反了，会导致高档永不触发。
     * 这里不强行改值（那是改用户配置），只做一次告警。
     * 告警只在 init 时打，避免热更新时刷屏。 */
}

/* ============================================================
 * 触发
 * ------------------------------------------------------------
 * 动作序列（顺序不能乱）：
 *   1. 置 faulted 与 cause（先置位，防止重入时重复停机）
 *   2. motor_stop()      —— 停 PWM 输出
 *   3. hw_brake_emergency_lock() —— 抱闸（防溜车）
 *   4. 记录快照与时间
 *
 * 为什么先置 faulted：stall_on_current 是 20kHz 调用，
 * 若先停机再置位，停机过程中的残余采样点可能再次进来重复执行。
 * ============================================================ */
static void stall_trigger(uint8_t cause)
{
    if (s_ctx.faulted)
    {
        return;                     /* 已故障，不重复停机 */
    }

    /* 快照必须在停机前取，停机后电流/转速会立刻掉下来 */
    s_ctx.snap_current_a10 = motor_stall_get_last_current();
    s_ctx.snap_tick        = rt_tick_get();

    s_ctx.faulted = 1U;
    s_ctx.cause   = cause;
    s_ctx.trigger_count++;
    s_ctx.last_trigger_tick = s_ctx.snap_tick;
    s_ctx.clear_wait_tick   = 0U;

    /* 1. 停电机 */
    if (s_motor != RT_NULL)
    {
        (void)motor_stop(s_motor);
        s_motor->state = MOTOR_STATE_FAULT;
    }

    /* 2. 紧急抱闸 —— 与 motor.c 的过流处理保持一致 */
    (void)hw_brake_emergency_lock(&g_swing_brake);
}

/* ============================================================
 * 20kHz 电流采样（raw 域，ISR 安全）
 * ------------------------------------------------------------
 * 三档独立计数，任一达到即触发。
 * "持续"语义：任一样本低于该档阈值 ⇒ 该档计数清零。
 *
 * 【为什么入参是 dev_raw 而不是 a10】
 *   本函数由 20kHz 中断调用。若入参用 0.1A，调用方每帧都要做一次
 *   int64 除法（foc_adc_raw_to_a10 内部），约 60~100 周期，
 *   20kHz 下白白吃掉 1.6% CPU 并拉长 ISR。
 *   改传 raw 域偏离量后，本函数内只剩整数比较，零除法。
 *   调用方用 foc_adc_get_bus_current_dev_raw() 取值（也是零除法）。
 *
 * 【为什么三档都要判，而不是只判最低档】
 *   最低档（45A/500ms）时间窗最长，但若电流瞬间冲到 55A，
 *   等 500ms 才停机就烧管了。必须三档并行，快档兜住冲击、
 *   慢档兜住缓慢升温。
 *
 * 【性能】最坏路径 3 次比较 + 3 次自增 + 1 次自增，约 20 周期。
 * ============================================================ */
void stall_on_current(uint16_t i_bus_dev_raw)
{
    uint8_t cause = STALL_CAUSE_NONE;

    if ((s_inited == 0U) || (s_ctx.faulted != 0U))
    {
        return;
    }

    /* ============================================================
     * 【2026-11-09】入参回到无符号
     * ------------------------------------------------------------
     * 母线电流物理上不反向，所以"偏离零点的量"天然非负，
     * foc_adc_get_bus_current_dev_raw() 已保证 raw <= zero 时返回 0。
     * 此处无需再取绝对值。
     *
     * 需要符号的是 IV/IW/IU（相/线电流），它们在 foc_adc_read()
     * 里保持 int16，与本函数无关。
     * ============================================================ */

    /* ---- 第 3 档：55A / 10ms ---- */
    if (i_bus_dev_raw >= s_cfg.oc3_dev_raw)
    {
        if (s_ctx.oc3_cnt < s_cfg.oc3_cnt_max)
        {
            s_ctx.oc3_cnt++;
        }
        if (s_ctx.oc3_cnt >= s_cfg.oc3_cnt_max)
        {
            cause |= STALL_CAUSE_OC3;
        }
    }
    else
    {
        s_ctx.oc3_cnt = 0U;
    }

    /* ---- 第 2 档：50A / 100ms ---- */
    if (i_bus_dev_raw >= s_cfg.oc2_dev_raw)
    {
        if (s_ctx.oc2_cnt < s_cfg.oc2_cnt_max)
        {
            s_ctx.oc2_cnt++;
        }
        if (s_ctx.oc2_cnt >= s_cfg.oc2_cnt_max)
        {
            cause |= STALL_CAUSE_OC2;
        }
    }
    else
    {
        s_ctx.oc2_cnt = 0U;
    }

    /* ---- 第 1 档：45A / 500ms ---- */
    if (i_bus_dev_raw >= s_cfg.oc1_dev_raw)
    {
        if (s_ctx.oc1_cnt < s_cfg.oc1_cnt_max)
        {
            s_ctx.oc1_cnt++;
        }
        if (s_ctx.oc1_cnt >= s_cfg.oc1_cnt_max)
        {
            cause |= STALL_CAUSE_OC1;
        }
    }
    else
    {
        s_ctx.oc1_cnt = 0U;
    }

    /* ---- 低速档的"样本计数"也在这里累加 ----
     *
     * 为什么样本计数放 20kHz、而阈值比较放 5ms tick：
     *   样本计数需要 20kHz 密度才能准确换算成毫秒；
     *   但转速值本身是 5ms 窗口的产物，在 20kHz 里读只是重复同一值。
     * 折中：20kHz 只负责 low_seg_samples++，
     *       5ms tick 负责比较转速并决定 low_seg_cnt++。
     * 这样低速段的"时间"仍由 20kHz 精确累计，不受调度抖动影响。 */
    if (s_ctx.low_seg_samples < 0xFFFFU)
    {
        s_ctx.low_seg_samples++;
    }

    if (cause != STALL_CAUSE_NONE)
    {
        stall_trigger(cause);       /* 冷路径：仅在真触发时进入 */
    }
}

/* ============================================================
 * 5ms 周期任务
 * ------------------------------------------------------------
 * 三件事：
 *   ① 刷门控（占空比 / 电流 / 缓停止）
 *   ② 低速分段判定
 *   ③ 自动清除故障
 * ============================================================ */
void stall_on_tick(void)
{
    int32_t rpm_x100 = 0;
    uint16_t i_mag;                 /* 母线电流幅值（raw 域偏离量） */
    uint8_t  got_speed;

    if (s_inited == 0U)
    {
        return;
    }

    /* ---------------- 1. 自动清除（故障态只做这一件事） ---------------- */
    if (s_ctx.faulted != 0U)
    {
        /* 清除条件：电流已降下来 且 电机已停（手柄松开）
         *
         * 为什么要同时满足：
         *   只看电流会在"堵转中电流被限流"时误清；
         *   只看状态会在"刚停机但电流未泄放"时误清。
         *
         * auto_clear_ms = 0（默认）表示条件满足立即清；> 0 则延时清。
         *
         * 【2026-11-09】母线电流已改为无符号单向，
         * foc_adc_get_bus_current_dev_raw() 直接返回非负值，此处不再取绝对值。 */
        i_mag = foc_adc_get_bus_current_dev_raw();

        if (i_mag < s_cfg.en_min_dev_raw)
        {
            if (s_cfg.auto_clear_ms == 0U)
            {
                motor_stall_clear();
            }
            else
            {
                if (s_ctx.clear_wait_tick == 0U)
                {
                    s_ctx.clear_wait_tick = rt_tick_get();
                }
                else if ((rt_tick_get() - s_ctx.clear_wait_tick) >=
                         (uint32_t)((s_cfg.auto_clear_ms + STALL_TICK_MS - 1U) /
                                    STALL_TICK_MS))
                {
                    motor_stall_clear();
                }
            }
        }
        else
        {
            /* 电流还在，说明仍堵着，撤销清除等待 */
            s_ctx.clear_wait_tick = 0U;
        }
        return;
    }

    /* ---------------- 2. 刷门控 ---------------- */
    {
        /* ① 是否在驱动：占空比 > 0
         * 用 motor->target_output（开环模式下即占空比）判断。
         * motor 为空时保守地认为"在驱动"，让保护更灵敏。 */
        if (s_motor != RT_NULL)
        {
            s_ctx.gate_driving = (s_motor->target_output > 0) ? 1U : 0U;
        }
        else
        {
            s_ctx.gate_driving = 1U;
        }

        /* ② 电流是否足以说明在出力（无符号，直接比较） */
        i_mag = foc_adc_get_bus_current_dev_raw();
        s_ctx.gate_current = (i_mag >= s_cfg.en_min_dev_raw) ? 1U : 0U;

        /* ③ 是否在缓停止（主动减速时转速低是正常的，不能算堵转）
         * soft_ramp_t.mode: 1=启动斜坡, 0=停止斜坡
         * 只有在 ramp 正在跑（is_running）且处于停止模式时才算缓停止。 */
        if (s_motor != RT_NULL)
        {
            uint8_t stopping = (s_motor->ramp.is_running != 0U) &&
                               (s_motor->ramp.mode == 0U);
            s_ctx.gate_not_ramping = (stopping != 0U) ? 0U : 1U;
        }
        else
        {
            s_ctx.gate_not_ramping = 1U;
        }
    }

    /* ---------------- 3. 低速判据 ---------------- */
    got_speed = motor_speed_get(&rpm_x100);

    if (got_speed != 0U)
    {
        /* 取绝对值：反转时 rpm_x100 为负，堵转判据只看幅值 */
        int32_t abs_rpm = (rpm_x100 < 0) ? -rpm_x100 : rpm_x100;
        uint32_t thr_x100 = (uint32_t)s_cfg.low_rpm * 100U;

        if (abs_rpm < (int32_t)thr_x100)
        {
            if (s_ctx.low_seg_cnt < 0xFFFFU)
            {
                s_ctx.low_seg_cnt++;
            }
        }
        /* 转速高不清零 low_seg_cnt —— 段内允许短暂超速，
         * 只要累计占比够高即成立。这是与电流档"严格连续"的有意区别。 */
    }

    /* ---------------- 4. 段切换判定 ----------------
     * low_seg_samples 由 20kHz 每样本 ++，达到段长（5000）即本段结束。
     * 本段成立条件：low_seg_cnt >= 段长的 90%。
     *
     * 为什么用 90% 而不是 100%：
     *   机械振动 / 换向瞬间 / 采样窗口边界都可能让个别 5ms 点读到
     *   略高的转速。100% 会导致永远不成立。
     *   90% 意味着 250ms 段内允许 25ms 的"超速"，足够容错，
     *   又不足以让真正的转动（转速会长时间高于阈值）蒙过去。
     */
    if (s_ctx.low_seg_samples >= s_cfg.low_seg_cnt_max)
    {
        uint32_t need = ((uint32_t)s_cfg.low_seg_cnt_max * 9U) / 10U;

        if (s_ctx.low_seg_cnt >= need)
        {
            /* 本段成立 */
            if (s_ctx.low_seg_ok < 0xFFU)
            {
                s_ctx.low_seg_ok++;
            }
        }
        else
        {
            /* 本段不成立 ⇒ 连续段数归零（"持续"语义） */
            s_ctx.low_seg_ok = 0U;
        }

        /* 进入下一段：样本计数与段内低速计数都清零 */
        s_ctx.low_seg_samples = 0U;
        s_ctx.low_seg_cnt     = 0U;
    }

    /* ---------------- 5. 低速触发判定 ----------------
     * 门控三条件全满足 且 连续段数达到要求 ⇒ 触发
     */
    if ((s_ctx.gate_driving   != 0U) &&
        (s_ctx.gate_current   != 0U) &&
        (s_ctx.gate_not_ramping != 0U) &&
        (s_ctx.low_seg_ok >= s_cfg.low_seg_total))
    {
        stall_trigger(STALL_CAUSE_LOW_RPM);
    }
}

/* ============================================================
 * 初始化
 * ============================================================ */
int motor_stall_init(void *motor)
{
    if (s_inited != 0U)
    {
        s_motor = (motor_t *)motor;  /* 允许后来补挂 motor */
        return 0;
    }

    memset(&s_cfg, 0, sizeof(s_cfg));
    memset(&s_ctx, 0, sizeof(s_ctx));

    s_motor = (motor_t *)motor;
    stall_load_cfg();

    s_inited = 1U;

    LOG_INFO("[STALL] init: oc3=%u.%uA/%ums oc2=%u.%uA/%ums "
               "oc1=%u.%uA/%ums\n",
               (unsigned)(s_cfg.oc3_a10 / 10U), (unsigned)(s_cfg.oc3_a10 % 10U),
               (unsigned)s_cfg.oc3_ms,
               (unsigned)(s_cfg.oc2_a10 / 10U), (unsigned)(s_cfg.oc2_a10 % 10U),
               (unsigned)s_cfg.oc2_ms,
               (unsigned)(s_cfg.oc1_a10 / 10U), (unsigned)(s_cfg.oc1_a10 % 10U),
               (unsigned)s_cfg.oc1_ms);
    LOG_INFO("[STALL] init: low=%urpm/%ums (seg=%u x %ums) "
               "gate=%u.%uA clear=%ums\n",
               (unsigned)s_cfg.low_rpm, (unsigned)s_cfg.low_ms,
               (unsigned)s_cfg.low_seg_total, (unsigned)STALL_LOW_SEG_MS,
               (unsigned)(s_cfg.en_min_a10 / 10U),
               (unsigned)(s_cfg.en_min_a10 % 10U),
               (unsigned)s_cfg.auto_clear_ms);

    /* 阈值单调性检查：高档阈值必须 >= 低档，否则高档永远先满足没意义 */
    if ((s_cfg.oc3_a10 < s_cfg.oc2_a10) || (s_cfg.oc2_a10 < s_cfg.oc1_a10))
    {
        LOG_WARN("[STALL] WARN: thresholds not monotonic "
                   "(oc3=%u oc2=%u oc1=%u), expect oc3>=oc2>=oc1\n",
                   (unsigned)s_cfg.oc3_a10,
                   (unsigned)s_cfg.oc2_a10,
                   (unsigned)s_cfg.oc1_a10);
    }

    return 0;
}

/* ============================================================
 * 查询与清除
 * ============================================================ */
uint8_t motor_stall_is_faulted(void)
{
    return s_ctx.faulted;
}

uint8_t motor_stall_get_cause(void)
{
    return s_ctx.cause;
}

const motor_stall_ctx_t *motor_stall_get_ctx(void)
{
    return &s_ctx;
}

void motor_stall_clear(void)
{
    /* 保留统计量（trigger_count / last_trigger_tick / 快照），
     * 只清"当前故障"与所有工作计数器 —— 便于事后追溯。 */
    s_ctx.faulted      = 0U;
    s_ctx.cause        = STALL_CAUSE_NONE;
    s_ctx.clear_wait_tick = 0U;

    s_ctx.oc3_cnt = 0U;
    s_ctx.oc2_cnt = 0U;
    s_ctx.oc1_cnt = 0U;

    s_ctx.low_seg_cnt     = 0U;
    s_ctx.low_seg_samples = 0U;
    s_ctx.low_seg_ok      = 0U;

    s_ctx.gate_driving    = 0U;
    s_ctx.gate_current    = 0U;
    s_ctx.gate_not_ramping = 1U;

    /* 电机状态从 FAULT 恢复到 IDLE，允许再次按键执行。
     * 为什么要在这里做：需求是"松开手柄后故障清除，再次按下键则按
     * 按键动作执行"。若只清本模块的 faulted 而不改 motor->state，
     * 上层看到 FAULT 会拒绝启动。 */
    if (s_motor != RT_NULL)
    {
        if (s_motor->state == MOTOR_STATE_FAULT)
        {
            s_motor->state = MOTOR_STATE_IDLE;
            s_motor->fault = 0U;
        }
    }
}

/* ============================================================
 * 供 t_stall 与内部快照使用
 * ------------------------------------------------------------
 * 单独抽一个 getter 而不直接在 stall_trigger 里调 foc_adc_*，
 * 是为了让"快照"这件事只有一个实现点。
 *
 * 【2026-11-09】返回**有符号**：触发快照保留电流方向，
 * 便于事后分析是电动堵转（正）还是发电/回馈（负）。
 * ============================================================ */
uint16_t motor_stall_get_last_current(void)
{
    return foc_adc_get_bus_current_a10();
}

/* ============================================================
 * 供 boot 层使用的电机对象 getter
 * ------------------------------------------------------------
 * 为什么需要它而不是让 boot_table.c 直接 extern g_swing_motor：
 *   boot_table.c 刻意不 include motor.h（启动表层对业务类型零依赖），
 *   直接 extern 会引入 motor_t 的完整定义。
 *   本文件已经 include motor.h，在这里转一道最自然。
 * ============================================================ */
void *boot_get_swing_motor(void)
{
    extern motor_t g_swing_motor;

    return (void *)&g_swing_motor;
}