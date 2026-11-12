/*=============================================================================
 * cfg_sampling.h  —  采样标定配置
 *
 * 换采样电阻/运放/分压电阻只改这里
 *=============================================================================*/

#ifndef CFG_SAMPLING_H
#define CFG_SAMPLING_H

/*---------------------------------------------------------------------------
 * 板型选择（换板子只改这里）
 *   0 = 现板：PA3 = 母线电流 itotal，U 相电流由 iu = -iv - iw 计算
 *             电流传感器灵敏度 26.4 mV/A，零点 1.65V
 *   1 = IU 板：PA3/PA4/PA5 = iu/iv/iw，三相差分电流传感器
 *             U 相电流直接用 PA3 采样值
 *             灵敏度 6 × 0.02Ω = 0.12 V/A，零点 1.25V
 *   2 = IU 重构板：硬件与 1 相同（PA3/PA4/PA5 = iu/iw/iv），
 *             但 PA3 只采样不使用（仅供观测），
 *             U 相电流由 iu = -iv - iw 重构
 *             传感器参数与板型 1 相同
 *---------------------------------------------------------------------------*/
#define CFG_SAMPLING_BOARD_IU_SENSOR    2

/*---------------------------------------------------------------------------
 * 采样模式
 *   1 = 真实模式：vbus 从 ADC 读，按 vbus_scale 换算
 *   0 = 模拟模式：vbus 固定 12.0V
 *---------------------------------------------------------------------------*/
#define CFG_SAMPLING_USE_REAL_ADC    1

/* 模拟模式下的固定母线电压（V） */
#define CFG_SAMPLING_SIM_VBUS_V      12.0f

/*---------------------------------------------------------------------------
 * ADC 采样时间与零偏校准
 *---------------------------------------------------------------------------*/
/* ADC 采样时间（SSTR 值，ADC 时钟周期数），三相电流通道统一使用。
 * 原值 7U，暂定 20U；DDL 只要求 >=5U，SSTR 为 8bit。 */
#define CFG_SAMPLING_ADC_SAMPLE_TIME        20u

/* 上电零偏校准平均点数：20kHz 下 2000 点 = 100ms。
 * main.c 的 DoOffsetCalBlocking 超时 300ms，需大于该时间。 */
#define CFG_SAMPLING_OFFSET_CAL_SAMPLES     2000u

/* 除上电零偏校准外，是否允许运行中的在线重校：
 *   1 = 允许（CURRENT/SPEED/POSITION/PARAM_IDENT 进入时重校，
 *           mode3/7 的 ZERO 段也做零偏观测）
 *   0 = 只使用上电校准值，不再做任何在线重校 */
#define CFG_SAMPLING_RUNTIME_OFFSET_CAL_EN  1

/*---------------------------------------------------------------------------
 * 电流标定
 *   现板：   VOUT = 1.65V + I × 26.4mV/A
 *   IU 板：  VOUT = 1.25V + I × (6 × 0.02Ω) = 1.25V + I × 0.12V/A
 *
 *   ADC 12bit / VREF = VDDA = 3.3V
 *     1 LSB   = 3.3/4095 V
 *     scale   = (VREF/4095) / 灵敏度        [A/LSB]
 *     offset  = 零点电压 × 4095 / VREF     [LSB]
 *---------------------------------------------------------------------------*/
#if (CFG_SAMPLING_BOARD_IU_SENSOR != 0)
#define CFG_SAMPLING_IU_GAIN          6.0f
#define CFG_SAMPLING_IU_SHUNT_OHM     0.02f
#define CFG_SAMPLING_IU_ZERO_V        1.25f
/* 低侧 shunt 采样：运放输出方向与 FOC 相电流正方向（流出逆变器）相反，
 * mode5 实测三相电流矢量整体差 180°，故三路统一取反 */
#define CFG_SAMPLING_CURRENT_SIGN     (-1.0f)
#define CFG_SAMPLING_CURRENT_SCALE    ((CFG_SAMPLING_CURRENT_SIGN) * (3.3f / 4095.0f) / (CFG_SAMPLING_IU_GAIN * CFG_SAMPLING_IU_SHUNT_OHM))  /* ≈-0.006716 A/LSB */
#define CFG_SAMPLING_CURRENT_OFFSET   (CFG_SAMPLING_IU_ZERO_V * 4095.0f / 3.3f)                                /* ≈1551 LSB */
#else
#define CFG_SAMPLING_CURRENT_SCALE    (3.3f / 4096.0f / 0.0264f)   /* A / LSB */
#define CFG_SAMPLING_CURRENT_OFFSET   2048.0f                       /* LSB = 1650 mV */
#endif

/*---------------------------------------------------------------------------
 * 母线电压标定
 *   vbus = adc_raw × scale
 *   待实机标定，先填 1.0f 看原始值
 *---------------------------------------------------------------------------*/
#define CFG_SAMPLING_VBUS_SCALE      1.0f

#endif /* CFG_SAMPLING_H */
