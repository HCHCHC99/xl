# mode 53 磁链 ψf 辨识 — 实现方案（v2：纳入电流采样噪声前提）

## Context（为什么做、目标）

`mode 51`（Rs/R_eff）与 `mode 52`（Ld/Lq）已实现并上机。现在补第三个参数辨识模式
**`mode 53 = ψf（永磁磁链）`**，完成 `md_record/参数辨识mode方案_51_52_53.md` 的三件套。

- **目标 = 验证辨识思路与方法本身**：结果**只放 Watch / VOFA**，**不写 Flash、不改 `motor_config.h`**，
  与手册值 `FOC_MOTOR_FLUX_VS = 0.00084`（0.84 mWb）、以及 mode 45 的 SMO 反电动势数据交叉对比。
- **机械条件已确认**：转子自由、空载、可自转 ⇒ 采用 **(a) 自带速度环**（复制 mode 40 的
  "5 ms 速度 PI + 每拍电流 PI"），电机自己爬到各转速点，**不需要任何外力**。
- **前置**：必须先跑 **mode 24**（编码器零点），用 `Foc_Dcal_GetResult()` 校验。
- **新增前提（本次）**：`Vcc 3.3V 有抖动 ⇒ 电流采样不准`。方案需在此前提下重做误差预算与滤波设计。

## 一、辨识原理与公式

稳态、`id≈0`、`diq/dt≈0`：

```
vq_cmd = R_eff · iq + ωe · ψf + V_dt        ωe = rpm/60 · 2π · FOC_POLE_PAIRS(=10)
```

- `R_eff ≈ 0.098 Ω`、`V_dt ≈ 0.160 V` 都取 **mode 51 实测值**。
- ⚠ **对原文档的修正**：原 §3 的 `vq = R·iq + ωe·ψf` 漏了死区项。旋转电流下死区电压在 dq 系
  是"锁在电流矢量上的 6 步波形"，含**直流分量**（非零均值），低速点（800 rpm 时 `ωe·ψf≈0.70 V`）
  占约 20%，不可忽略。
- **求解 = 取斜率，不是取单点**：`iq` 与 `ωe` 近似共线 ⇒ `R` 不能与 `ψf` 同辨识，故 **`R` 固定取 mode 51 值**。
  对 `y_k = vq_k − R_eff·iq_k` 做**二参数最小二乘** `y_k = ψf·ωe_k + V_dt`：
  **斜率 = ψf（主结果）**，截距 = `V_dt`（与 mode 51 的 160 mV 互证）。
- ⭐ **斜率对常数偏置免疫**：`R·iq` 偏置、`V_dt`、电流零偏、Vcc 增益误差里凡是**不随转速变化**的部分，
  都被截距吸收，**不进斜率**。这是本方案抗采样噪声的核心。

## 二、电流采样噪声 / Vcc 抖动的处置（本次新增重点）

### 2.1 误差预算（先看清"怕什么、不怕什么"）

ψf 的两个来源是 **vq（数字指令，无 ADC 噪声）** 与 **ωe（编码器）**；电流只经 `R·iq` 进入。

| 扰动 | 传播 | 对 ψf 的影响（0.84 mWb 基准） |
|---|---|---|
| iq 误差 ±50 mA | `∂ψf/∂iq = −R/ωe` | 800 rpm 时 **−0.006 mWb（0.7%）**；3200 rpm 时 0.17% |
| Vcc 增益误差 ±1%（iq=300 mA） | 环路把"测得 iq"逼到给定，实际 iq 差 1% | **0.04%**（二阶小量） |
| Vcc 抖动（交流） | 对三相**共模**（同一个 ADC 基准）⇒ Clarke/Park 后是纯增益抖动，不产生 dq 交叉 | 被 avg 窗平均掉，趋近 0 |
| **Vbus 假设误差**（`FOC_Svpwm` 用固定 `FOC_VBUS_V=12.0`） | 占空比按假设算，实际电压 = 指令 × `Vbus_actual/12.0` | **±2.5% 的 Vbus 偏差 ⇒ ψf ±2.5%（系统性，最大项）** |

> 结论：**电流采样噪声不是本模式的主导误差**（<1%）；真正要紧的是 Vbus 的绝对准确度（系统性）
> 与"iq 是否随转速变化"（会污染斜率）。方案因此把力气花在**平均 + 斜率免疫**上，而不是硬怼 ADC。

### 2.2 具体滤波与算法措施（放进 AVG 状态）

1. **长窗平均（主滤波）**：`AVG` 窗默认 **300 ms**（可调 100~1000 ms），@20 kHz ≈ 6000 个样本，
   随机噪声按 √N 衰减（√6000≈77），远强于任何单级 EMA。
2. **测量侧 EMA 预滤（不进环路）**：对**用于测量**的 `iq`、`vq` 各加一级 EMA（`α=0.05`，τ≈1 ms），
   在累加前抑制开关纹波与 6 次谐波。⚠ **只滤测量副本，绝不插入电流 PI 的反馈链**（避免给环路加相位滞后）。
3. **ωe 不做差值估计，用窗内编码器计数直算**：`ωe = (Δcounts/ENCODER_CPR)·2π·P / Δt`，
   `Δt` 用 **Timer6 µs 时基**（0.32 µs/tick）测得。这样分母是"整窗平均转速"，
   没有 5 ms 速度窗的量化噪声，也避免 α=0.25 滤波的滞后。
4. **稳定性诊断（判"噪声是否真的限制了精度"）**：
   - `g_flx53_win_split_pct`：窗内**前半段均值 vs 后半段均值**的相对差；>2% 说明有漂移/噪声。
   - 建议上机时把 `g_flx53_avg_ms` 由 300 → 600 再测一次：**若 ψf 变化 <1%，说明噪声已不是瓶颈**。
5. **离群剔除**：`AVG` 窗**前 20 ms 不计**（切入新转速点、电流环尚未完全稳态）；
   EMA 后的 `iq` 若偏离窗均值 >3σ 的单拍不累加（可选，默认关，用 `g_flx53_reject_en`）。
6. **out-of-mode 可选**：mode-40 文档已给出根治法——电流传感器输出加 **1 kΩ+100 nF 硬件 RC**
   （fc≈1.6 kHz），在环路之外切断开关纹波。本模式不改硬件，只记录该建议。

## 三、状态机（`g_flx53_state`）

```
IDLE(0)         : PWM 关；Start 校验 mode 24，失败则 running=0 + evt=NO_CAL
SPINUP(1)       : 速度环爬向 g_flx53_rpm[k]（斜坡限幅），同时进稳态计时
SETTLE(2)       : 判稳态 |ramp-filt|<g_flx53_rpm_tol 且 |Δiq|<20mA 连续 settle_ms；
                  超时 5s 未稳 -> evt=TIMEOUT，跳过该点
AVG(3)          : avg_ms 窗：前 20ms 丢弃；EMA 后累加 iq/vq；用 Timer6+编码器 delta 算 ωe
NEXT(4)         : k++；未到 points 则回 SPINUP，否则进 FIT
FIT(5)          : 二参数最小二乘(斜率=ψf) + 逐点 ψf + 离散度 + r² + ratio + 差分校验
RAMPDOWN(6)     : 目标置 0，等 |filt rpm| < 30 后关 PWM
DONE(7)         : 完成（PWM 已关）；running 保持 1 供 VOFA/Watch 回看
FAULT_OC(8)     : 过流停机
FAULT_OVRPM(9)  : 超速（filt rpm > g_flx53_rpm_max）
```

- 过流一律 `Foc_Core_OverCurrent(pData)` → `Foc_Core_FaultStop(1)`（沿用 mode 40/51/52）。
- **差分校验**：另有 `ψf_diff = (vq_last−vq_first)/(ωe_last−ωe_first)`（端点差分），
  与最小二乘斜率对比；两者接近 ⇒ 斜率法自洽（对常数偏置免疫得到二次确认）。

## 四、环路参数（直接复用 mode 40 已验证值，同电机同硬件）

| 项 | 值 | 来源 |
|---|---|---|
| 电流环 kp / ki | 0.1 V/A / 300 V/A/s | `FOC40_PI_KP/KI`（kp=0.1 本就是压静止噪声的定稿）|
| 电流环 UMAX / ITERM | 6.2 V / 6.0 V | `FOC40_PI_UMAX_V/ITERM_MAX_V` |
| 速度环 kp / ki | 0.8 mA/rpm / 0.03 mA/rpm/s | `FOC40_SPD_KP/KI` |
| 速度环 iq 限幅 / 加速度限幅 | 3400 mA / 1950 rpm/s | `FOC40_SPD_IQ_LIMIT_MA` / `_ACCEL_LIMIT_RPM_S` |
| 速度窗口 / 滤波 / 显示滤波 | 5 ms / 0.25 / 0.05 | `FOC40_SPD_*` |
| 编码器增量限幅 | 36 counts/拍 | `FOC40_ENC_DELTA_MAX` |

实现时以 `FOC53_*` 前缀镜像这些宏（模块自持，便于将来单独调）。

## 五、Watch 可调参数

| 变量 | 默认 | 单位 | 含义 |
|---|---|---|---|
| `g_flx53_rpm[6]` | 800/1600/2400/3200/0/0 | rpm | 转速序列（前 `points` 个有效） |
| `g_flx53_points` | 4 | - | 取点数 2~6 |
| `g_flx53_r_used_ohm` | 0.098 | Ω | 算 ψf 用的 R（填 mode 51 实测值） |
| `g_flx53_vdead_v` | 0.160 | V | 死区先验（`fit_vdt=0` 时使用） |
| `g_flx53_fit_vdt` | 1 | - | 1=把 V_dt 与 ψf 一起拟合；0=用固定值 |
| `g_flx53_avg_ms` | 300 | ms | 每点平均窗口（100~1000，加大以压噪声） |
| `g_flx53_settle_ms` | 300 | ms | 稳态持续判据 |
| `g_flx53_rpm_tol` | 15 | rpm | 稳态转速容差 |
| `g_flx53_iq_filt_alpha` | 0.05 | - | 测量侧 iq EMA（τ≈1ms，不进环路） |
| `g_flx53_reject_en` | 0 | - | 3σ 离群剔除开关（默认关） |
| `g_flx53_rpm_max` | 4000 | rpm | 超速保护阈值 |
| `g_flx53_vbus_scale` | 1.000 | - | Vbus 校正系数（实测 Vbus ÷ 12.0，修系统性） |

## 六、观测量

| 变量 | 含义 |
|---|---|
| `g_flx53_psi_mwb` | **ψf 主结果**（斜率，mWb；0.84 显示 0.840） |
| `g_flx53_psi_diff_mwb` | ψf 端点差分校验值 |
| `g_flx53_vdt_fit_mv` | 拟合截距 V_dt（mV，与 mode 51 的 160 mV 互证） |
| `g_flx53_ratio` | `ψf / FOC_MOTOR_FLUX_VS`（x1000） |
| `g_flx53_psi_alt_mwb` | 用 `FOC_MOTOR_RS_OHM`(0.1) 重算的 ψf（R 敏感度） |
| `g_flx53_psi_spread_mwb` | 各点 ψf 离散度（max−min，<10% 合格） |
| `g_flx53_r2` | 二参数拟合优度（x1000） |
| `g_flx53_win_split_pct` | 窗内前半/后半均值相对差（x10，诊断噪声/漂移） |
| `g_flx53_psi_pts[6]` / `_pts_rpm[6]` / `_pts_vq[6]` / `_pts_iq[6]` / `_pts_we[6]` | 各点原始数据 |
| `g_flx53_psi_live_mwb` | 当前点实时 ψf |
| `g_flx53_target_rpm` / `_rpm` / `_rpm_disp` | 目标 / PI 反馈 / 显示转速 |
| `g_flx53_vq` / `_iq` (EMA 前后) / `_vsat` / `_omega_e` | q 轴电压 / 电流 / 饱和 / 电角频率 |
| `g_flx53_pts_done` / `_elapsed_ms` | 已完成点数 / 累计时间（ms） |
| `g_flx53_state` / `_evt` / `_running` | 状态 / 事件 / 运行标志 |

事件码：`1 OC / 2 DONE_OK / 3 DONE_POOR(离散度大) / 4 TIMEOUT(某点未稳) / 5 OVRPM / 6 NO_CAL`。

## 七、VOFA 布局（16ch，`Foc_Flx53_VofaFill`）

```
ch0 状态 | ch1 目标转速(rpm) | ch2 实际转速-PI反馈(rpm) | ch3 vq(V) | ch4 iq(A)
ch5 ψf 实时估计(mWb) | ch6 ψf 最终值(mWb) | ch7 离散度(mWb) | ch8 已完成点数
ch9 r²(x1000) | ch10 窗内前后半差(x10) | ch11 Vdead 用值(mV) | ch12 ψf 差分校验(mWb)
ch13 ψf/手册(x1000) | ch14 Vsat(0/1) | ch15 累计时间(ms)
```

## 八、需要改动的文件

| 文件 | 改动 |
|---|---|
| **新增** `ws/foc_53_flx.h` | 顶部【模式速览卡】+ 宏 + `extern` + API 原型；`FLX53_DBG` 开关 |
| **新增** `ws/foc_53_flx.c` | 复制 mode 40 骨架（速度/电流 PI、编码器测速、Step、VofaFill）+ 本方案测量/滤波/拟合逻辑；`#include "timer6_timebase.h"` |
| `ws/dev_comm_runner.h` | 枚举加 `COMM_RUNNER_FLX53 = 53`；通道一览表加 `mode 53 16ch` |
| `ws/dev_comm_runner.c` | `CommRunner_StopFocModes()` 加 `g_flx53_running -> Foc_Flx53_Stop()`；`SetMode` 加 `case`（Stop + Commutation_Stop + `Foc_Flx53_Start()` + 日志） |
| `ws/foc.c` | ALIGN 分发链末尾加 `else if (g_flx53_running) { Foc_Flx53_Step(pData); }` |
| `ws/foc.h` | 加 `#include "foc_53_flx.h"` 与文件头索引行 |
| `template/source/main.c` | VOFA 派发链加 `Foc_Flx53_VofaFill`；更新两处通道数注释表 |
| `ws/foc_obs.c` | 事件段加 `[FLX53]` 打印（进入配置 / 每点结果 / 汇总 / 各 warn；ISR 内不打印，禁 `%f` 与中文） |
| `template/MDK/template - 副本.uvprojx` | **两个 target 各加 1 个 `<File>` 条目**（仿 `_cardgen/add_mode52_uv.mjs`：先收集完整 `<File>` 区间再一次性插入，校验 `</File>` 配对） |
| `d:/WS_L_re/_cardgen/cards/mode53.json` | 新建卡片数据，`node cardbuild.mjs --splice` 拼进 `foc_53_flx.h`（**不手改头文件里的卡**） |

命名沿用约定：`Foc_Flx53_Start/Step/Stop/VofaFill`、全局 `g_flx53_*`、宏 `FOC53_*`。

## 九、验证

1. **静态自检**（无 CLI ARMCC，先跑脚本）：`cardbuild.mjs`、`verify_sync.mjs`、
   `check_vofafill.mjs`（`cur[]` 下标恰为 0..15、return ≤ 24、符号真实存在）、`check_braces.mjs`、`check_externs.mjs`。
2. **Keil 编译**（两个 target 各 Build 一次）——需你在本机执行。
3. **上机流程**：`mode 24` → `comm_mode = 53` → Watch 观察 `g_flx53_psi_mwb`、
   `g_flx53_psi_diff_mwb`、`g_flx53_vdt_fit_mv`、`g_flx53_psi_spread_mwb`、`g_flx53_r2`、
   `g_flx53_win_split_pct`、`g_flx53_ratio`；RTT 看 `[FLX53]` 汇总行。
4. **判据**：
   - 各点 `ψf_k` 离散度 < 10%（<0.08 mWb），且 `win_split_pct` 小 ⇒ 数据合格；
   - **噪声自证**：`avg_ms` 300 → 600 重测，ψf 变化 <1% ⇒ 采样噪声不是瓶颈；
   - 期望 `ψf ≈ 0.84 mWb`（≈ 每 1000 rpm 升 0.88 V），`ratio ≈ 0.87~1.0`；
   - `psi_mwb` 与 `psi_diff_mwb` 接近 ⇒ 斜率法自洽；
   - 拟合 `V_dt` 与 mode 51 的 160 mV 同量级 ⇒ R 与死区模型自洽；
   - 稳态段 `g_flx53_vsat == 0`；与 mode 45 的 SMO 数据交叉对比（判 0.87 因子是真实偏差还是投影）。