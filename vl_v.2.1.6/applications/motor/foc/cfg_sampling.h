/**
 *******************************************************************************
 * @file  cfg_sampling.h
 * @brief 采样标定配置（与 FOC 工程 ws/cfg/cfg_sampling.h 一致）
 *
 *        换采样板/运放/分压电阻只改这里。
 *******************************************************************************
 */

#ifndef CFG_SAMPLING_H
#define CFG_SAMPLING_H

/*---------------------------------------------------------------------------
 * 板型选择
 *   0 = 母线电流板：PA3 = 母线电流，26.4 mV/A，零点 1.65V
 *                    iu = -iv - iw
 *   1 = IU 板：PA3/PA5/PA4 = iu/iv/iw，0.12 V/A，零点 1.25V，三路取反
 *   2 = IU 重构板：硬件同 1，PA3 只采样观测，iu = -iv - iw
 *---------------------------------------------------------------------------*/
#define CFG_SAMPLING_BOARD_IU_SENSOR    2

/*---------------------------------------------------------------------------
 * 母线电压
 *   FOC 的 ADC1 CH6 每帧采样 PA6，原始值送入 sys_voltage 的 1ms 环形缓冲区，
 *   由 sys_voltage 求平均并按板型分压系数换算：
 *     板型 0 → 31 倍，板型 1/2 → 25 倍
 *   SVPWM 使用换算后的真实 vbus；低于 12V 按 12V 处理。
 *   下面的常量仅作为 vbus 未就绪时的兜底值。
 *---------------------------------------------------------------------------*/
#define CFG_SAMPLING_SIM_VBUS_V      12.0f

/*---------------------------------------------------------------------------
 * 母线电压两点标定（仅板型 1/2；板型 0 为恒等，不做修正）
 *   实测三点（供电 12.33/13.85/15.06V，读数 13.87/15.70/17.60V）拟合：
 *     bus_measured = 1.3595 × bus_true - 2.965V
 *   反解：
 *     bus_true = (bus_measured + 2965mV) / 1.3595
 *   该增益相当于真实分压约 34 倍（25 × 1.3595），另叠加约 87mV 的引脚偏置。
 *   若只想用纯偏置：GAIN_X10000 设为 10000，OFFSET_MV 设为 1977。
 *---------------------------------------------------------------------------*/
#if (CFG_SAMPLING_BOARD_IU_SENSOR != 0)
#define CFG_SAMPLING_VBUS_CAL_GAIN_X10000   13595
#define CFG_SAMPLING_VBUS_CAL_OFFSET_MV     2965
#else
#define CFG_SAMPLING_VBUS_CAL_GAIN_X10000   10000
#define CFG_SAMPLING_VBUS_CAL_OFFSET_MV     0
#endif

/*---------------------------------------------------------------------------
 * ADC 采样时间与零偏校准
 *---------------------------------------------------------------------------*/
#define CFG_SAMPLING_ADC_SAMPLE_TIME        50u

/* 上电/模式3 ZERO 阶段零偏校准平均点数：20kHz 下 2000 点 = 100ms */
#define CFG_SAMPLING_OFFSET_CAL_SAMPLES     2000u

/*---------------------------------------------------------------------------
 * 电流标定
 *   板型 0：  VOUT = 1.65V + I × 26.4mV/A
 *   板型 1/2：VOUT = 1.25V + I × (6 × 0.02Ω) = 1.25V + I × 0.12V/A
 *   ADC 12bit / VREF = 3.3V
 *---------------------------------------------------------------------------*/
#if (CFG_SAMPLING_BOARD_IU_SENSOR != 0)
#define CFG_SAMPLING_IU_GAIN          6.0f
#define CFG_SAMPLING_IU_SHUNT_OHM     0.02f
#define CFG_SAMPLING_IU_ZERO_V        1.25f
/* 低侧 shunt：运放输出方向与 FOC 相电流正方向相反，三路统一取反 */
#define CFG_SAMPLING_CURRENT_SIGN     (-1.0f)
#define CFG_SAMPLING_CURRENT_SCALE    ((CFG_SAMPLING_CURRENT_SIGN) * (3.3f / 4095.0f) / \
                                       (CFG_SAMPLING_IU_GAIN * CFG_SAMPLING_IU_SHUNT_OHM))
#define CFG_SAMPLING_CURRENT_OFFSET   (CFG_SAMPLING_IU_ZERO_V * 4095.0f / 3.3f)
#else
#define CFG_SAMPLING_CURRENT_SCALE    (3.3f / 4096.0f / 0.0264f)
#define CFG_SAMPLING_CURRENT_OFFSET   2048.0f
#endif

#endif /* CFG_SAMPLING_H */
