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
 *   1 = 真实模式：从 ADC PA6/CH6 读取
 *   0 = 模拟模式：固定 CFG_SAMPLING_SIM_VBUS_V
 *---------------------------------------------------------------------------*/
#define CFG_SAMPLING_USE_REAL_ADC    1

#define CFG_SAMPLING_SIM_VBUS_V      12.0f

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

/*---------------------------------------------------------------------------
 * 母线电压标定：vbus = adc_raw × scale
 *   待实机标定，先填 1.0f 看原始值
 *---------------------------------------------------------------------------*/
#define CFG_SAMPLING_VBUS_SCALE      1.0f

#endif /* CFG_SAMPLING_H */
