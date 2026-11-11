# 电流 PI 环不跟随问题记录

日期：2026-09-17

## 现象

仅启用电流 PI 环时，电机转动过程中：

```text
iq 不能较好跟随 iq_ref
```

该问题影响 mode 29 / mode 40 的电流内环调试。转速越高或负载/电压条件变化时，跟随误差可能更明显。

## 当前相关配置

mode 29 当前电流 PI 参数：

```c
g_drun29_pid_id_cfg.i_valid = 1;
g_drun29_pid_iq_cfg.i_valid = 1;

g_drun29_pid_id_cfg.kp = 0.5f;
g_drun29_pid_iq_cfg.kp = 0.5f;

g_drun29_pid_id_cfg.ki = 300.0f;
g_drun29_pid_iq_cfg.ki = 300.0f;
```

mode 40 电流内环参数目前相同：

```c
g_speed40_pid_id_cfg.kp = 0.5f;
g_speed40_pid_iq_cfg.kp = 0.5f;

g_speed40_pid_id_cfg.ki = 300.0f;
g_speed40_pid_iq_cfg.ki = 300.0f;
```

当前电流采样模式为 ADC2 PWM 触发模式。若选择 `PEAK` 或 `VALLEY`，电流环实际频率为 PWM 频率；若选择 `PEAK_VALLEY`，电流环实际频率为 PWM 频率的两倍。

## 可能原因

### 1. 反电动势随转速上升

电机旋转后 q 轴电压平衡中包含反电动势项：

```text
vq = R * iq + L * diq/dt + e
```

其中：

```text
e ≈ ω * ψf
```

转速越高，反电动势越大。如果电流环 PI 输出不足以抵消反电动势，iq 就无法跟随 `iq_ref`。

排查时重点看：

```c
g_speed40_speed_meas_rpm
g_speed40_iq_ref_ma
g_speed40_iq_ma
g_speed40_vq
g_speed40_vsat
```

如果 `vq` 已经接近限幅，或 `g_speed40_vsat = 1`，说明电压余量不足。

### 2. dq 轴交叉耦合

旋转时 dq 轴之间存在交叉耦合项：

```text
vd 包含 -ω * L * iq
vq 包含 +ω * L * id
```

如果没有解耦，id/iq 的 PI 会互相影响，动态过程中 iq 跟随误差会增大。

### 3. 电流环带宽不足

当前电机电感较小：

```text
L ≈ 42.3µH
```

电流环 P 墊决定理论带宽：

```text
fc ≈ kp / (2π * L)
```

但是实际带宽还受以下因素限制：

```text
ADC 采样延迟；
PWM 更新延迟；
计算延迟；
电流噪声；
电流采样模式。
```

如果 `kp` 太小，电流环响应慢，动态过程中 iq 跟不上 `iq_ref`。

### 4. 电机转动时角度误差被放大

如果编码器零点或电角度方向有误差，旋转时 dq 轴电流会互相耦合，表现为 iq 跟随变差、id 出现非零分量。

重点检查：

```c
g_foc_enc_dir
g_foc_cur_sign
mode24 校准结果
g_dcal24_offset
```

## 排查方案

### 方案 1：提高电流环带宽

先锁转子或轻载，给小电流阶跃，观察：

```c
g_drun29_iq_ref_ma
g_drun29_iq_ma
```

逐步提高电流环 P：

```c
g_drun29_pid_iq_cfg.kp = 0.5f;
g_drun29_pid_iq_cfg.kp = 0.7f;
g_drun29_pid_iq_cfg.kp = 1.0f;
```

注意每次变化后观察：

```text
iq 是否振荡；
vd/vq 是否频繁饱和；
三相电流是否噪声过大；
是否触发过流。
```

如果电流环带宽提高后动态跟随改善，说明原电流环带宽不足。

### 方案 2：加入反电动势前馈

根据当前电角速度计算反电动势：

```text
bemf = ω_e * ψf
```

然后在电流环输出后加入 q 轴前馈：

```text
vq = PI_iq + ω_e * ψf
```

其中：

```c
ψf = FOC_MOTOR_FLUX_VS
```

该前馈可以减少速度环/电流环积分器需要长时间建立的反电动势补偿量，提高动态跟随能力。

### 方案 3：加入 dq 交叉耦合解耦

按当前转速和电流计算解耦电压：

```text
vd_ff = -ω_e * L * iq
vq_ff = +ω_e * L * id
```

然后：

```text
vd = PI_id + vd_ff
vq = PI_iq + vq_ff + bemf_ff
```

`L` 可先使用：

```c
FOC_MOTOR_LS_UH
```

如果实际电感与手册值差异较大，应先通过实验辨识实际 L。

### 方案 4：确认电角度和相序

电机旋转时如果角度框架不准，id/iq 会耦合。可通过以下现象判断：

```text
iq_ref 固定时 id 明显非零；
iq 跟随误差随转速增加；
正反转行为不一致；
同电流下转速异常偏低。
```

必要时重新执行 mode24 校准，并确认：

```c
g_foc_enc_dir
g_foc_cur_sign
```

## 临时建议

在问题解决前：

```text
1. 不要在高速下给大幅值 iq 阶跃；
2. 保持电流限幅有效；
3. 优先用 VOFA 观察 iq_ref/iq/id/vd/vq/vsat；
4. 若 vq 饱和，应先降低转速或电流目标。
```
