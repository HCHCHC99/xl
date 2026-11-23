/**
 * @file    sys_voltage.c
 * @brief   系统（母线）电压采集实现
 *
 *          数据源：FOC 的 ADC1 CH6（PA6）20kHz 同步采样。
 *          FOC 的 DMA 中断每帧调用 sys_voltage_vbus_sample_isr(raw)，
 *          本模块维护 20 点环形缓冲区，每满 1ms 求平均并按板型分压换算。
 */

#include "sys_voltage.h"

#include <rtthread.h>

/* 欠压参数（CFG_ID_UV_* / CFG_DEF_UV_*）与事件投递 */
#include "../config/sys_config.h"
#include "../app_event.h"
#include "input_sim.h"              /* 台架模拟（任务清单 §11） */

/* 分压系数随板型选择：
 *   板型 0：31 倍（100k/3.3k，1033/33）
 *   板型 1/2：25 倍
 * 板型宏来自 FOC 采样配置，保持"一块板一份配置"的唯一来源。 */
#include "../motor/foc/cfg_sampling.h"

#if (CFG_SAMPLING_BOARD_IU_SENSOR == 0)
#define SYS_VOLT_DIV_NUM        (1033U)   /* ≈ 31.03 倍 */
#define SYS_VOLT_DIV_DEN        (33U)
#else
#define SYS_VOLT_DIV_NUM        (25U)
#define SYS_VOLT_DIV_DEN        (1U)
#endif

#define SYS_VOLT_DIV_X100       ((SYS_VOLT_DIV_NUM * 100U) / SYS_VOLT_DIV_DEN)

/* ============================================================
 * 1ms 环形缓冲区
 * ------------------------------------------------------------
 * 20kHz 下 20 点正好 1ms。写入与平均都在 ADC 中断上下文，
 * 无需临界区；对外只暴露一个 32 位 mV 值。
 * ============================================================ */
#define SYS_VOLT_VBUS_RING_LEN      20U

static uint16_t s_vbus_ring[SYS_VOLT_VBUS_RING_LEN];
static uint8_t  s_vbus_ring_idx = 0U;
static uint8_t  s_vbus_ring_cnt = 0U;

static volatile uint16_t s_vbus_avg_raw = 0U;   /* 1ms 平均原始码（诊断用） */
static volatile uint32_t s_vbus_meas_mv = 0U;   /* 标定前的母线电压 mV（诊断用） */
static volatile uint32_t s_bus_mv_1ms   = 0U;   /* 1ms 平均母线电压 mV（标定后） */
static volatile uint32_t s_bus_mv_cached = 0U;  /* 最近一次读取值 mV */

/* ============================================================
 * 欠压状态机（业务.md §13.6-A / 任务清单 §10.5-A）
 * ------------------------------------------------------------
 * sys_voltage_init() 末尾启动 10 ms 软定时器：
 *   NORMAL --(bus < enter_mv 连续 confirm_ms，且已过 blank_ms 消隐)--> UV
 *   UV     --(bus > recover_mv 连续 recover_ms)--> NORMAL
 * 状态变化各投递一次事件，不重复投递。
 *
 * 【为什么参数不在 init 里读】init 早于启动流程收尾；首次定时器回调
 * （+10 ms）时 sys_config 已加载完成，此时读一次并缓存，之后只用缓存。
 * 【为什么回调内不打印】10 ms 周期回调；RTT 输出会拖慢 timer 线程，
 * 参数打印统一放到 sys_voltage_monitor_start()。
 * ============================================================ */
#define SYS_UV_POLL_PERIOD_MS       10U

static struct rt_timer s_uv_timer;
static uint8_t         s_uv_timer_started = 0U;

/* 配置缓存（首次回调从 sys_config 读，失败保留 CFG_DEF_UV_* 兜底） */
static uint8_t s_uv_cfg_loaded    = 0U;
static int32_t s_uv_enter_mv      = CFG_DEF_UV_ENTER_MV;
static int32_t s_uv_recover_mv    = CFG_DEF_UV_RECOVER_MV;
static int32_t s_uv_confirm_ms    = CFG_DEF_UV_CONFIRM_MS;
static int32_t s_uv_recover_ms    = CFG_DEF_UV_RECOVER_MS;
static int32_t s_uv_blank_ms      = CFG_DEF_UV_BLANK_MS;

/* 运行时状态 */
static uint32_t s_uv_blank_start_tick = 0U;     /* 消隐起点（init 时刻） */
static uint32_t s_uv_enter_acc_ms     = 0U;     /* 欠压条件连续成立累计 */
static uint32_t s_uv_recover_acc_ms   = 0U;     /* 恢复条件连续成立累计 */
static volatile sys_power_state_t s_uv_state = SYS_POWER_NORMAL;

/* 读 5 个 UV 参数（sys_config 返回 0 = 成功；失败或非法值用宏兜底） */
static void sys_voltage_uv_load_cfg(void)
{
    int32_t v = 0;

    if ((sys_config_get_i32(CFG_ID_UV_ENTER_MV, &v) == 0) && (v > 0)) {
        s_uv_enter_mv = v;
    }
    if ((sys_config_get_i32(CFG_ID_UV_RECOVER_MV, &v) == 0) && (v > 0)) {
        s_uv_recover_mv = v;
    }
    if ((sys_config_get_i32(CFG_ID_UV_CONFIRM_MS, &v) == 0) && (v > 0)) {
        s_uv_confirm_ms = v;
    }
    if ((sys_config_get_i32(CFG_ID_UV_RECOVER_MS, &v) == 0) && (v > 0)) {
        s_uv_recover_ms = v;
    }
    if ((sys_config_get_i32(CFG_ID_UV_BLANK_MS, &v) == 0) && (v >= 0)) {
        s_uv_blank_ms = v;
    }

    s_uv_cfg_loaded = 1U;
}

/* 10 ms 软定时器回调：只做采样/比较/计时/投事件，不打印 */
static void sys_voltage_uv_timer_cb(void *parameter)
{
    uint32_t bus_mv;

    (void)parameter;

    /* 首次回调：启动流程已完成，读取并缓存 5 个门限参数 */
    if (s_uv_cfg_loaded == 0U) {
        sys_voltage_uv_load_cfg();
    }

    /* ---- 上电消隐：自 sys_voltage_init() 起算，期间不做任何判定 ---- */
    if ((rt_tick_get() - s_uv_blank_start_tick) <
        rt_tick_from_millisecond((rt_int32_t)s_uv_blank_ms)) {
        return;
    }

    /* ---- 采样：1 ms 平均母线电压 ---- */
    bus_mv = sys_voltage_get_bus_mv_1ms();
    if (bus_mv == 0U) {
        /* 采集尚未就绪（PWM/ADC 未运行）：不判欠压，避免上电误急停。
         * 采集丢失由 vehicle 的 VEH_FAULT_VOLT_MISSING 负责，不在此处理。 */
        s_uv_enter_acc_ms   = 0U;
        s_uv_recover_acc_ms = 0U;
        return;
    }

    /* ---- 状态机：只比较/计时，状态变化各投一次事件 ---- */
    if (s_uv_state == SYS_POWER_NORMAL) {
        s_uv_recover_acc_ms = 0U;

        if (bus_mv < (uint32_t)s_uv_enter_mv) {
            s_uv_enter_acc_ms += SYS_UV_POLL_PERIOD_MS;
            if (s_uv_enter_acc_ms >= (uint32_t)s_uv_confirm_ms) {
                s_uv_enter_acc_ms = 0U;
                s_uv_state = SYS_POWER_UV;
                /* 【P1 / 2026-11-23】停止类事件不丢：队列满时进待处理环 */
                post_event_stop(SYS_EVT_UV_ENTER);
            }
        } else {
            s_uv_enter_acc_ms = 0U;
        }
    } else {
        s_uv_enter_acc_ms = 0U;

        if (bus_mv > (uint32_t)s_uv_recover_mv) {
            s_uv_recover_acc_ms += SYS_UV_POLL_PERIOD_MS;
            if (s_uv_recover_acc_ms >= (uint32_t)s_uv_recover_ms) {
                s_uv_recover_acc_ms = 0U;
                s_uv_state = SYS_POWER_NORMAL;
                post_event(SYS_EVT_UV_RECOVER);
            }
        } else {
            s_uv_recover_acc_ms = 0U;
        }
    }
}

/* 10 ms 软定时器创建/启动（幂等，重复调用不会重复启动） */
static void sys_voltage_uv_timer_setup(void)
{
    if (s_uv_timer_started != 0U) {
        return;
    }

    rt_timer_init(&s_uv_timer,
                  "uv_mon",
                  sys_voltage_uv_timer_cb,
                  RT_NULL,
                  rt_tick_from_millisecond((rt_int32_t)SYS_UV_POLL_PERIOD_MS),
                  RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    (void)rt_timer_start(&s_uv_timer);
    s_uv_timer_started = 1U;
}

int sys_voltage_init(void)
{
    s_vbus_ring_idx = 0U;
    s_vbus_ring_cnt = 0U;
    s_vbus_avg_raw  = 0U;
    s_bus_mv_1ms    = 0U;
    s_bus_mv_cached = 0U;

    rt_kprintf("[SYS-VOLT] PA6 via FOC ADC1 CH6, 1ms avg, div=%u.%02ux (board=%d)\n",
               (unsigned)(SYS_VOLT_DIV_X100 / 100U),
               (unsigned)(SYS_VOLT_DIV_X100 % 100U),
               (int)CFG_SAMPLING_BOARD_IU_SENSOR);

    /* ============================================================
     * 欠压监视启动（任务清单 §10.5-A）
     * ------------------------------------------------------------
     * 10 ms 周期软定时器跑在 timer 线程，回调只做比较/计时/投事件。
     * 消隐起点取此刻，5 个门限参数留到首次回调再读（见 uv_timer_cb）。
     * ============================================================ */
    s_uv_state            = SYS_POWER_NORMAL;
    s_uv_enter_acc_ms     = 0U;
    s_uv_recover_acc_ms   = 0U;
    s_uv_cfg_loaded       = 0U;          /* 首次回调再读配置 */
    s_uv_blank_start_tick = rt_tick_get();

    sys_voltage_uv_timer_setup();

    return 0;
}

uint8_t sys_voltage_is_ready(void)
{
#if (INPUT_SIM_ENABLE && SYS_VOLT_SIM_ENABLE)
    if (input_sim_active(INPUT_SIM_BUS) != 0U) {
        return 1U;      /* 模拟值即有效值，欠压/启车判定可直接用 */
    }
#endif
    return (s_bus_mv_1ms != 0U) ? 1U : 0U;
}

/* ============================================================
 * 1ms 平均（20kHz 中断喂数据）
 * ------------------------------------------------------------
 * 每帧写入环形缓冲区；每满 20 点（20kHz → 1ms）求一次平均。
 * 平均值先还原成引脚 mV，再按板型分压系数还原成母线 mV。
 * ============================================================ */
void sys_voltage_vbus_sample_isr(uint16_t raw)
{
    uint32_t sum = 0U;
    uint32_t pin_mv;
    uint8_t  i;

    s_vbus_ring[s_vbus_ring_idx] = raw;
    s_vbus_ring_idx++;
    if (s_vbus_ring_idx >= SYS_VOLT_VBUS_RING_LEN) {
        s_vbus_ring_idx = 0U;
    }

    if (s_vbus_ring_cnt < SYS_VOLT_VBUS_RING_LEN) {
        s_vbus_ring_cnt++;
        if (s_vbus_ring_cnt < SYS_VOLT_VBUS_RING_LEN) {
            return;                 /* 未满 1ms，本拍不更新 */
        }
    }

    for (i = 0U; i < SYS_VOLT_VBUS_RING_LEN; i++) {
        sum += s_vbus_ring[i];
    }

    s_vbus_avg_raw = (uint16_t)((sum + (SYS_VOLT_VBUS_RING_LEN / 2U)) /
                                SYS_VOLT_VBUS_RING_LEN);

    /* 先算引脚 mV（12bit / 3.3V），再乘分压比，避免乘法溢出 */
    pin_mv = ((uint32_t)s_vbus_avg_raw * SYS_VOLT_ADC_VREF_MV) /
             SYS_VOLT_ADC_FULL_SCALE;
    s_vbus_meas_mv = (pin_mv * SYS_VOLT_DIV_NUM) / SYS_VOLT_DIV_DEN;

    /* 板型 1/2 两点标定：bus_true = (bus_measured + offset) / gain。
     * 板型 0 的宏为恒等（gain=1、offset=0）。 */
    {
        int32_t mv = (int32_t)s_vbus_meas_mv + CFG_SAMPLING_VBUS_CAL_OFFSET_MV;

        if (mv < 0) {
            mv = 0;
        }
        s_bus_mv_1ms = (uint32_t)((mv * 10000) / CFG_SAMPLING_VBUS_CAL_GAIN_X10000);
    }
}

uint32_t sys_voltage_get_bus_mv_1ms(void)
{
#if (INPUT_SIM_ENABLE && SYS_VOLT_SIM_ENABLE)
    /* 台架模拟：t_sim bus <mV>。唯一读点，欠压状态机 / 启车判定 /
     * 状态帧 / FOC 的 vbus 全部随之变化。 */
    if (input_sim_active(INPUT_SIM_BUS) != 0U) {
        int32_t mv = input_sim_value(INPUT_SIM_BUS);

        return (mv > 0) ? (uint32_t)mv : 0U;
    }
#endif
    return s_bus_mv_1ms;
}

uint32_t sys_voltage_read_bus_mv(void)
{
    /* 走 getter，模拟值同样生效 */
    uint32_t bus_mv = sys_voltage_get_bus_mv_1ms();

    s_bus_mv_cached = bus_mv;

    return bus_mv;
}

uint32_t sys_voltage_get_bus_mv_cached(void)
{
    return s_bus_mv_cached;
}

uint16_t sys_voltage_get_dv(void)
{
    uint32_t bus_mv = sys_voltage_read_bus_mv();

    return (uint16_t)(bus_mv / 100U);
}

/* ============================================================
 * 欠压状态查询 / 诊断（任务清单 §10.5-A）
 * ============================================================ */
sys_power_state_t sys_voltage_get_power_state(void)
{
    return s_uv_state;
}

uint8_t sys_voltage_is_uv(void)
{
    return (sys_voltage_get_power_state() == SYS_POWER_UV) ? 1U : 0U;
}

void sys_voltage_monitor_start(void)
{
    /* 10 ms 轮询已在 sys_voltage_init() 启动；本函数只打印生效参数
     * （定时器回调内禁止打印）并做幂等保护，允许重复调用。 */
    if (s_uv_cfg_loaded == 0U) {
        sys_voltage_uv_load_cfg();
    }
    sys_voltage_uv_timer_setup();

    /* 与验收日志格式对齐：enter / confirm / blank 一行 */
    rt_kprintf("[SYS-VOLT] UV monitor: enter=%dmV confirm=%dms blank=%dms\n",
               (int)s_uv_enter_mv, (int)s_uv_confirm_ms, (int)s_uv_blank_ms);
    rt_kprintf("[SYS-VOLT] UV hysteresis: recover=%dmV after %dms\n",
               (int)s_uv_recover_mv, (int)s_uv_recover_ms);
}
