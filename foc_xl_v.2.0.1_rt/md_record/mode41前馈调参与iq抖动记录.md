# mode41 前馈调参与 iq 抖动记录

日期：2026-09-19

## 当前现象

在 mode41 中打开前馈后：

```text
iq_ref = 200mA：基本可以跟随；
iq_ref >= 300mA：平均值仍跟随，但抖动明显变大；
iq_ref 越大，抖动越大。
```

VOFA 观察通道：

```text
CH5  = g_drun41_iq_ma        原始 iq，电流 PI 实际使用；
CH7  = g_drun41_iq_ref_ma    iq 目标；
CH16 = g_drun41_iq_filt_ma   显示用 EMA 滤波 iq，不进入 PI。
```

这个现象说明电流环平均值可以跟随目标，但高频误差或噪声随电流目标增大。需要区分是实际电流纹波变大，还是前馈把测量噪声重新注入输出。

## mode41 前馈公式

当前前馈为：

```text
vd_ff = -cross_gain * we * L * iq

vq_ff =  cross_gain * we * L * id
       + bemf_gain  * we * psi_f
       + res_gain   * R  * iq_ref
```

对应参数：

```c
g_drun41_ff_enable
g_drun41_ff_bemf_gain
g_drun41_ff_cross_gain
g_drun41_ff_res_gain
g_drun41_vmax_v
```

注意：

```text
交叉项使用实测 id/iq；
BEMF 项使用 200ms 窗口速度；
电阻项使用 iq_ref。
```

因此 `cross_gain` 最容易把原始电流采样噪声重新注入 `vd/vq` 输出。

## 重点观察变量

```c
g_drun41_iq_ref_ma
g_drun41_iq_ma
g_drun41_iq_filt_ma

g_drun41_eq_mean_ma
g_drun41_eq_pp_ma

g_drun41_vd_pi_v
g_drun41_vq_pi_v
g_drun41_vd_ff_v
g_drun41_vq_ff_v

g_drun41_omega_e_rad_s
g_drun41_vsat
```

判断原则：

```text
好的前馈：eq_mean_ma 下降，eq_pp_ma 不恶化或下降，PI 分量变小，vsat 不更容易触发；
坏的前馈：iq 抖动增大，vd/vq 振荡，id/iq 互串加重，vsat 更容易触发。
```

## 建议分项调参顺序

不要一次性打开全部前馈。按下面的顺序隔离：

### 1. 基线

```c
g_drun41_ff_enable      = 0;
```

记录 `iq_ref = 200 / 300 / 400mA` 时的：

```text
eq_mean_ma
eq_pp_ma
CH5 原始 iq 抖动
CH16 滤波 iq 抖动
vd/vq PI 分量
```

### 2. 只开 BEMF 前馈

```c
g_drun41_ff_enable      = 1;
g_drun41_ff_bemf_gain   = 1.0f;
g_drun41_ff_cross_gain  = 0.0f;
g_drun41_ff_res_gain    = 0.0f;
```

BEMF 主要补偿反电动势。方向正确时，`g_drun41_vq_pi_v` 应该变小。

如果 `vq_pi` 反而变大，说明 BEMF 符号、磁链参数或速度方向有问题。

### 3. 加入电阻项

```c
g_drun41_ff_enable      = 1;
g_drun41_ff_bemf_gain   = 1.0f;
g_drun41_ff_cross_gain  = 0.0f;
g_drun41_ff_res_gain    = 1.0f;
```

电阻项通常很小，例如：

```text
R * iq_ref = 0.1 * 0.3 = 0.03V
```

这项一般不会造成大抖动。

### 4. 最后单独加交叉项

从很小的值开始：

```c
g_drun41_ff_cross_gain = 0.0f -> 0.2f -> 0.5f -> 1.0f;
```

每改一次观察 `eq_pp_ma` 和 CH16 抖动。

如果 `cross_gain` 打开后抖动明显变大，而平均误差没有改善，应先降低或置零。当前最可疑的就是交叉项使用原始 `iq` 计算 `vd_ff`，把电流采样噪声注入输出。

## 可选改进方向

如果确认交叉项导致抖动，可以改用低噪声来源：

```text
方案 A：交叉项使用 id_ref/iq_ref，而不是实测 id/iq；
方案 B：交叉项使用低通滤波后的 id/iq；
方案 C：保留很小的 cross_gain，例如 0.2~0.5；
方案 D：先关闭交叉项，只使用 BEMF 前馈。
```

## PI 配合调整

如果前馈已经承担一部分模型电压，可以适当降低电流环 P，避免原始电流噪声被放大：

```c
g_drun41_pid_id_cfg.kp = 0.15f ~ 0.25f;
g_drun41_pid_id_cfg.ki = 300.0f ~ 400.0f;

g_drun41_pid_iq_cfg.kp = 0.15f ~ 0.25f;
g_drun41_pid_iq_cfg.ki = 300.0f ~ 400.0f;
```

当前若仍使用：

```c
kp = 0.5
ki = 300
```

则在电流采样噪声较大时，P 项会把噪声较快映射到电压输出。

## 当前结论

200mA 时前馈效果可以接受；300mA 以上抖动随目标增大，优先怀疑：

```text
1. cross_gain 使用原始实测 iq/id，导致噪声注入；
2. 电流纹波本身随目标电流增大；
3. kp 偏大，放大高频电流误差；
4. 高电流下速度升高，BEMF/交叉项幅值同步变大；
5. 电压余量或 SVPWM 限幅开始影响动态。
```

下一步建议先测试：

```c
g_drun41_ff_enable      = 1;
g_drun41_ff_bemf_gain   = 1.0f;
g_drun41_ff_res_gain    = 1.0f;
g_drun41_ff_cross_gain  = 0.0f;
```

如果 300mA 以上抖动明显下降，就基本确认交叉前馈是主要噪声来源。
