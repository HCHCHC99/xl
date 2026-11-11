# mode 27 功角闭环拖动设计方案

> 状态：已实现（foc_dcl.c/.h，comm_mode 27）
> 日期：2026-09-12
> 配套：FOC模式27流程图.canvas（ob 目录）、交接.md 十三节

---

## 一、背景与动机

mode 26（开环 VF 负载角实验）实测发现：磁场按时间自增拖动时，功角 delta 存在
±10° 左右的持续抖动（VOFA CH13 呈马鞍波）。诊断结论：**欠阻尼猎振（hunting）**
——转子是被磁场吊着的钟摆，PMSM 在同步转速下没有电磁阻尼，齿槽/量化踢步持续
激励，摩擦阻尼压不住。

由此引出锁相设想：**如果磁场角不按时间走，而是实时跟随转子角 + 固定功角，
功角就被"钉死"了**——这是 mode 27 的出发点。

## 二、方案取舍过程

讨论中否决/搁置的备选：

| 备选 | 结论 |
|---|---|
| 时间开环 + freq/step/delta 三组各自爬坡（初版需求） | freq（时间源）与 delta（锁相源）互斥，同组无意义 → 用户拍板：**砍掉时间开环，纯 delta 闭环**。mode 26 保留作开环对照 |
| 转速 PI → 调 delta（同步发电机调速器式） | 强但复杂一个量级，v2 备选 |
| 跟踪观测器（PLL）滤波编码器 | 量化噪声 ±0.44° 电角远小于实验精度；PLL 防不了相对框架永久污染（跟的是同一个被污染计数器）→ v1 不做，v2 纯加法 |
| 半拍提前补偿（实际 δ = δ_set + ω·Ts/2） | 偏差仅 1~3°@100-300Hz，手动把 targ 少设 1~2° 即可 → v1 不做，v2 备选 |

## 三、核心原理

1. **锁相 = 功角钉死**：磁场角 `fa = 转子实测电角 + δ_set`，每 50µs 重新计算。
   δ 不再是力矩平衡"漂"出来的自由变量，而是代数定义 → ±0.1° 级稳定，且
   **理论上不可能失步**（磁场天涯海角跟着转子，没有牵出边界）。
2. **电压定转速（直流电机式）**：δ 钉死后力矩近似恒定，转速成为新的状态变量：
   转速↑ → BEMF↑ → 电流↓ → 力矩↓ → 减速，稳态停在"BEMF 吃掉足够电压"的
   平衡点。**调速旋钮 = g_dcl_volt_v**（所以本模式没有 freq 变量）。
   实测参考：δ=90°、0.6V 时稳态 ≈ 460Hz 电频（≈2760rpm），id 转负（电压饥饿
   去磁），iq = 摩擦级电流——全部自洽。
3. **MTPA 闭环验证**：扫 `g_dcl_dlt_targ_deg`（5→45→89），转速应在 90° 附近
   最高（id 最小、电压预算全给 BEMF 对抗）——功角特性的闭环版实验。

## 四、状态机与流程

```
IDLE(0) → CAL_BETA(1, 磁场90° 2s) → CAL_ALPHA(2, 磁场0° 2s)
        → 锁零点 → RUN(3, 锁相拖动) ；任意状态过流 → FAULT_OC(4)
```

- 校准与 mode 25/26 完全相同（吸附式，无 foc_calib 零偏窗）。
- **锁零点双帧**：
  - 控制帧 `s_off_rel` = 锁相瞬间相对计数，此后转子电角 =
    `mod((s_enc_pos − s_off_rel) × enc_dir, CPR)`（锁相瞬间 = 0）
  - 显示帧 `g_dcl_offset` = hw 绝对帧（与 mode 20/25/26 同框架，跨模式对比），
    并调 `Foc_Core_SetAlignOffset` 同步 core 观测。
- 事件：1 BETA_DONE / 2 LOCKED / 3 RAMP_DONE / 4 OC（仅 OC 自动回 mode 0）。

## 五、delta 爬坡设计

- 公式：`value = init + (targ − init) × clamp(elapsed/tr, 0, 1)`
- **计时用 ISR tick 计数**（50µs 分辨率），不用 timer6/TickTimer：爬坡消费者
  就是 20kHz ISR 自己，本地计数零共享、确定性最好；秒级过渡用 µs 时间戳是浪费。
- `init/targ/tr` 每拍实时读 Watch：运行中改 = 按新参数重算轨迹（会跳变，
  与 mode 26 直改 freq 同性质，实验时建议启动前设好）；`tr=0` 立即到目标。
- 到点置 `RAMP_DONE` 每次**运行一次**（s_ramp_done 标志）。
- 三参数 **Start 不复位**（便于预设后启动）；`g_dcl_volt_v` Start 复位 0.6V。

## 六、编码器路径设计（本模式编码器 = 控制路径）

mode 26 里编码器只做观察，mode 27 它直接决定力矩方向，丢计数 = 框架错乱。

1. **增量累积式读取**：wrap-safe 差分 → **±32 counts 限幅** → 累加 `s_enc_pos`。
   - 物理依据：7800rpm 极限下单拍真实增量上限 26.6 counts，超过必是毛刺；
   - 限幅把单次毛刺对功角框架的**永久**污染封顶 ±32 counts ≈ ±2.8° 电角
     （1 count = 0.879° 电角）。
   - 注意：限幅不是"滤波"——硬件计数器本身会被噪声边沿永久顶偏，
     限幅只保证"每次污染有界"。
2. **为什么不上普通低通**：滞后 = ω×τ。100Hz 电频 + 1ms EMA = 32° 滞后，
   磁场永远落后——滤波后的角度只能进 Watch/VOFA 显示，**绝不能进控制**。
3. **量化噪声**：±0.5 count = ±0.44° 电角，远小于实验精度，不处理。

## 七、输出与观测约定

- **q 轴电压约定**（与 mode 26/30 统一）：`Dcl_OutputField` 入参是磁场角 fa
  （主控制变量）：`valpha=V·cos(fa), vbeta=V·sin(fa)`；等效 d 轴角
  `theta = fa − 90°` 存入 g_foc_theta_rad；锁相稳态 `g_foc_id_ma≈0, g_foc_iq_ma≈I`。
- **diff 是恒等式**：`diff = fold(field − rotor) ≡ dlt_now`（两者同源构造），
  只验证数学没错，**不是独立测量的真功角**（mode 26 才是）。真实功角看
  id/iq 的电流矢量角 `atan2(iq, id)`。
- 电流用原始 pData（与 25/26 同策略，无零偏窗，读数含传感器偏置）。
- 转速 `g_dcl_speed_hz`：200ms 窗口计数 × 极对数 × 窗口率 / CPR，**带符号**，
  符号 = 原始计数方向（未乘 enc_dir，与 mode 31 win 同约定）。

## 八、Watch 变量表

| 变量 | 默认 | 属性 | 说明 |
|---|---|---|---|
| `g_dcl_volt_v` | 0.6 | 可调，Start 复位 | 电压 = 调速旋钮（0.1V ≈ 1A） |
| `g_dcl_dlt_init_deg` | 5 | 可调，Start 不复位 | 爬坡起点（软吸附） |
| `g_dcl_dlt_targ_deg` | 45 | 可调，Start 不复位 | 爬坡终点（MTPA 扫描用） |
| `g_dcl_dlt_tr_ms` | 2000 | 可调，Start 不复位 | 过渡时间，0 = 立即 |
| `g_dcl_dlt_now_deg` | — | 观测 | 当前 δ 指令 |
| `g_dcl_speed_hz` | — | 观测 | 实测电频率（200ms 窗口，带符号） |
| `g_dcl_diff_deg` | — | 观测 | ≡ dlt_now（恒等验证） |
| `g_dcl_id_ma / g_dcl_iq_ma` | — | 观测 | 真实转子系电流 |
| `g_dcl_state / g_dcl_evt` | — | 观测 | 状态机 / 事件 |
| `g_dcl_offset` | — | 观测 | 锁零点（绝对帧 counts） |
| `g_dcl_enc_pos` | — | 观测 | 相对计数镜像（毛刺排查） |
| `g_dcl_field_deg / g_dcl_rotor_deg` | — | 观测 | 磁场角 / 转子角（0~359） |
| `g_dcl_du/dv/dw` | — | 观测 | 三相占空比 % |

编译期常量（foc_dcl.h）：`DCL_BETA_MS/ALPHA_MS`=2000、`DCL_SPEED_WIN_MS`=200、
`DCL_ENC_DELTA_MAX`=32。

## 九、RTT 打印（[DCL] 前缀，全整型）

- 事件：`LOCKED off=%d deg -> run dlt %d->%d deg tr=%d ms` /
  `ramp done dlt=%d deg spd=%d mHz` / `FAULT_OC i=%d mA`（自动回 mode 0）
- 200ms 周期：`dlt=%d deg spd=%d mHz fld=%d deg rot=%d deg diff=%d deg id=%d iq=%d mA`

## 十、实验方法

1. Watch 写 `comm_mode = 27`（main.c 零改动，不占按键）→ 等 4s 校准 → 自动 RUN。
2. 确认 `LOCKED` / `ramp done` 事件，`diff ≈ 45`。
3. **MTPA 扫描**：`g_dcl_dlt_targ_deg` 依次 5/30/45/60/75/89，每档记
   `g_dcl_speed_hz` 稳态值 → 转速峰值应在 90° 附近。
4. **调速**：改 `g_dcl_volt_v`（直流电机式）。
5. 注意：delta 小角度时 id 大（铜损 ~2W 量级），长时间运行留意电机温升；
   高速段注意机械安全（实测可到 ~2760rpm）。

## 十一、已知边界与 v2 待办

| 项 | 现状 | v2 方向 |
|---|---|---|
| 半拍提前补偿 | 不做，手动 targ 少设 1~2° | 自动 `−ω·Ts/2`，需平滑转速 |
| PLL 跟踪观测器 | 不做（纯加法位，随时可插） | 角度+转速联合跟踪，兼做半拍补偿的转速源 |
| speed 符号 | 原始计数方向（未乘 enc_dir） | 若要电角度方向，测速公式乘 `g_foc_enc_dir` |
| 电流零偏 | 原始 pData（含偏置） | 可复用 foc_calib 窗 |
| 转速 PI → 调 δ | 未做 | 同步发电机调速器式闭环调速 |

## 十二、集成清单与验证

- 新文件：`ws/foc_dcl.c/.h`
- `dev_comm_runner.h`：`COMM_RUNNER_DCL=27`；`.c`：case 27 启动 + 其余 8 case 停止
- `foc.c`：ALIGN 分发第四优先级 `g_cal → g_calang → g_olf → g_dcl → Foc_Align_Step`
- `foc.h`：聚合 foc_dcl.h；`foc_obs.c`：[DCL] 事件 + 200ms 周期打印
- main.c **零改动**（Watch 写 comm_mode=27 进入）
- **foc_dcl.c 需手动添加到 Keil 工程**（ws 组）
- 验证点：LOCKED 事件 → ramp done → diff ≈ targ → speed_hz 稳定 →
  改 volt 转速跟随 → 扫 targ 看 MTPA 峰
