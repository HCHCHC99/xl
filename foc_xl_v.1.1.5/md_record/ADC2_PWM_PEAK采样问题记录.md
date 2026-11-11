# ADC2 PWM PEAK 电流采样问题记录

日期：2026-09-16

## 当前结论

`I_SAMPLE_ADC2_PWM_PEAK` 目前不可用。

进入 mode 29 后会立刻触发过流保护。已经尝试把 ADC2 的采样时间从默认值增加到：

```c
#define I_ADC2_PWM_PEAK_SAMPLE_TIME (64U)
```

但问题仍然存在，因此这不是单纯通过延长 ADC 采样保持时间就能解决的。

## 当前可用模式

当前建议继续使用：

```c
#define I_SAMPLE_MODE I_SAMPLE_ADC2_PWM_VALLEY
```

之前实测 `I_SAMPLE_ADC2_PWM_PEAK_VALLEY` 也可以正常进行电流 PI 环。

## 现象

1. `I_SAMPLE_ADC2_PWM_PEAK_VALLEY`：电流 PI 环可以正常运行。
2. `I_SAMPLE_ADC2_PWM_VALLEY`：修复 AOS 路由后可用。
3. `I_SAMPLE_ADC2_PWM_PEAK`：进入 mode 29 后立即报过流。
4. PEAK 模式下将 ADC2 CH1/CH2/CH3 采样时间增大到 64 个 ADCLK 周期后，问题仍然存在。

## 当前 PEAK 配置

PEAK 模式使用：

```text
TMR4_3 三角波峰值
-> SCMP0
-> AOS_ADC2_0
-> ADC2 SEQ_A
-> ADC2_EOCA 中断
```

采样通道为：

```text
PA5 / ADC2_CH1 / IU
PA6 / ADC2_CH2 / IV
PA7 / ADC2_CH3 / IW
```

## 可能方向

PEAK 点靠近 PWM reload/update 时刻，功率级开关和驱动耦合可能会影响电流传感器输出或 ADC 采样。后续可以排查：

1. 进入 mode 29 前后在 mode 0 下观察 `g_i_iu_raw`、`g_i_iv_raw`、`g_i_iw_raw` 是否已经有 PEAK 时刻尖峰；
2. 记录过流瞬间的 `g_foc_fault_i_ma`、`g_foc_fault_iu_ma`、`g_foc_fault_iv_ma`、`g_foc_fault_iw_ma`；
3. 将 `g_drun29_i_ref_ma` 降到 0 或很小的正值，确认过流是否仍与电流环输出有关；
4. 增大电流传感器输出端的 RC 滤波或检查 ADC 输入阻抗；
5. 尝试将 PEAK 触发点从 PWM reload 边沿向后延迟若干 ADCLK/PCLK 周期；
6. 对比示波器下 PA5/PA6/PA7 传感器输出与 ADC 实际采样波形。

## 代码位置

采样模式选择：

```c
ws/I.h
```

PEAK 模式的加长采样时间：

```c
ws/I.c
I_AdcConfig()
```

当前默认模式为：

```c
I_SAMPLE_MODE = I_SAMPLE_ADC2_PWM_VALLEY
```
