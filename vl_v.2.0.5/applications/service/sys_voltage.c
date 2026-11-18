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
    return 0;
}

uint8_t sys_voltage_is_ready(void)
{
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
    return s_bus_mv_1ms;
}

uint32_t sys_voltage_read_bus_mv(void)
{
    uint32_t bus_mv = s_bus_mv_1ms;

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
