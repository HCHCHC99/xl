# FOC 模式25 设计文档 — foc_cal_angle（手动角度吸附 / SVPWM 教学）

## 目的

教学用模式：手动输入电角度，让磁场指向该角度，把转子吸过去并验证吸稳，
用于直观理解 `电角度 → valpha/vbeta → SVPWM 三相占空比 → 转子物理位置` 这条链路。

## 用户操作流

1. 进入 mode 25 → **自动跑校准**（每次进入都跑，BETA 90° 吸 2s → ALPHA 0° 吸 2s，锁零点 offset）
2. 校准完成 → 切 **50/50/50 零矢量刹车**（不出力），**模式停在 25**
3. 用户在 Keil Watch 修改 `g_foc_angle_input`（**单位 0.1°，0~3599，写 900 = 90.0°**）
   → 程序检测到变化（x → y）
4. 吸附 **2s，三段子阶段**：
   - **引导点 300ms**：触发瞬间实测转子电角度判来向，先吸到"转子一侧 15°"（同侧预拉，最后一段拉程只有 15°，到站动量小）；转子已在目标 ±5° 内则跳过
   - **目标角 ±3° 抖动 400ms**（50Hz 正弦）：拔齿槽 / 破静摩擦
   - **纯目标角 1.3s**：落座
5. **500ms 校验窗（磁场仍吸在 y）**：窗口内编码器位移 `max−min ≤ g_calang_stable_cnts`（默认 8 counts ≈ 0.7° 机械角）→ 成功；还在动 → 失败
6. **无论成败都回刹车 50/50/50**，停在 mode 25 等下一次输入；退出靠切 mode 0

## 状态机

| 状态 | 值 | 行为 | 出口 |
|---|---|---|---|
| IDLE | 0 | 未运行 | — |
| CAL_BETA | 1 | 磁场定 90°，计时 2s | → CAL_ALPHA，evt=BETA_DONE |
| CAL_ALPHA | 2 | 磁场定 0°，计时 2s；结束时 `offset = mod(原始计数×方向, CPR)` → `Foc_Core_SetAlignOffset` | → BRAKE_WAIT，evt=LOCKED |
| BRAKE_WAIT | 3 | 每拍 50/50/50；监视 `g_foc_angle_input` 改值 | 改值 → 锁存目标 → HOLD_ATTRACT |
| HOLD_ATTRACT | 4 | 三段子阶段（s_hold_sub：0 引导点 / 1 抖动 / 2 落座）；中途改值 → 重新判向锁存重计时 | 落座完成 → HOLD_VERIFY |
| HOLD_VERIFY | 5 | 磁场保持 y；500ms 窗口记录编码器 min/max 位移；结束时快照实测角/误差 | → BRAKE_WAIT，evt=DONE_OK 或 DONE_FAIL |
| FAULT_OC | 6 | FaultStop | evt=OC → obs 打印 → 自动回 mode 0 |

## g_foc_angle_input 语义与数学陷阱

- **电角度，单位 0.1°（deci-degree）**，0~3599，静止系（校准后 0° = 转子 N 极零位）；
  int32，代码内 wrap 到 [0,3600)；写 900 = 90.0°
- **input/target/meas/err 全部同一单位（×0.1° 整型）**，Watch 里直接对比无需换算
  （target_deg/meas_deg/err_deg 变量名带 deg 是历史命名，实际存 ×0.1°）
- 渐进扫描（每步 ≤900 即 90°电）：900→mech 9°、1800→18°、2700→27°、0/3600→36°，
  每步 +9° 机械（360°/10 对极），四步走完一个电周期 = 36° 机械
- **陷阱 1**：只对渐进扫描成立。跳变 >180°电（>1800）时转子走短路径
  （如 0 直接设 2700 → 转子倒退到 mech −9°=351°）
- **陷阱 2**：恰好 180° 跳变（输入 1800）转矩为零（sin180°=0），死点，转子不动或乱摆，实验时避开
- 0 与 3600 等价（同一磁场方向）
- 输入规则：改值才触发；HOLD 中途改值 → 重新计时吸新目标；
  **校准 4s 期间预设的值会在校准完成后立即执行**（s_last_input 在 Start 时刻快照）

## 精度优化记录（实测数据驱动）

首轮实测（v1：直接跳目标角，0.4V）：

| target | 270 | 90 | 0 | 180 | 0 | 50 | 45 |
|---|---|---|---|---|---|---|---|
| meas | 267 | 87 | 0 | 186 | 4 | 42 | 36 |
| err | −3 | −3 | 0 | +6 | +4 | −8 | −9 |

误差来源分析（全部 ±9° 电角度 = ±0.9° 机械角以内）：

1. **编码器量化底噪**：1 count = 0.88° 电角度，测量极限
2. **静摩擦死区（主项）**：转子停在吸引转矩 = 摩擦转矩处，Δ ≈ asin(T_f/T_max)
3. **齿槽/漂移**：BRAKE 期间转子无保持力矩、自由漂进齿槽（50→45 组的 42→36 倒退即此），下次吸引要先"拔出"齿槽
4. **校准偏置**：ALPHA 校准同样带死区，锁进的 offset 自带 1~2° 公共偏移（首轮数据均值 ≈ −2°）

优化手段（v2 实现，吸附三段化）：

- 引导点（治过冲 + 预拔齿槽）：先吸到转子一侧 15°，最后一段拉程短、到站动量小
- 抖动 dither（治静摩擦 + 齿槽）：目标角 ±3°、50Hz、400ms
- 提电压（Watch 手调）：g_calang_volt_v 0.4→0.5V（4→5A，死区约缩 20%）；上限 0.55V（OC 5.5A）
- 预期：±9° → ±4~5° 电角度；物理下限 ±1~2°（量化 + 死区）

## 变量清单

| 变量 | 类型 | 说明 |
|---|---|---|
| `g_foc_angle_input` | volatile int32_t | 用户输入电角度 ×0.1°（0~3599，Watch 写） |
| `g_calang_volt_v` | volatile float | 吸附电压，默认 0.4V（≈4A，与 mode 32 一致） |
| `g_calang_state` | volatile uint8_t | 状态机当前状态 |
| `g_calang_evt` | volatile uint8_t | 事件锁存（obs 消费后清 0） |
| `g_calang_running` | volatile uint8_t | 1 = 运行中 |
| `g_calang_stable_cnts` | volatile int32_t | 校验窗静止判据，默认 8 counts |
| `g_calang_target_deg` | volatile int32_t | 快照：锁存的目标角 ×0.1°（0~3599） |
| `g_calang_meas_deg` | volatile int32_t | 快照：校验结束时编码器实测电角度 ×0.1°（0~3599） |
| `g_calang_err_deg` | volatile int32_t | 快照：meas−target 折叠到 (−1800,1800] ×0.1° |
| `g_calang_win_moved` | volatile int32_t | 快照：校验窗内 max−min 位移（counts） |
| `g_calang_offset` | volatile int32_t | 快照：校准锁定的零点（counts） |
| `g_calang_du/dv/dw` | volatile float | 三相占空比观测（%），看 SVPWM 用 |

## 打印规范（[CALANG] 前缀，.h 内 0/1 开关，整数缩放，无浮点/中文）

- `BETA done -> ALPHA 0deg`
- `LOCKED offset=%d deg`（零点，校准框架基准，保持整度）
- `HOLD ok target=%d meas=%d err=%d x0.1deg`（吸稳；target/meas/err 单位 0.1°）
- `HOLD FAIL moved=%d cnts target=%d meas=%d x0.1deg`（校验窗还在动）
- `FAULT_OC i=%d mA`

事件由 foc_obs.c 的 Foc_Obs_Task 消费打印（ISR 内不打印）；只有 OC 自动回 mode 0，
LOCKED / DONE 均保持在 mode 25。

## 集成点（mode 20 零改动）

- 新文件：`ws/foc_cal_angle.c/.h`（.h 顶部傻瓜式讲解；校准逻辑自包含，不依赖 foc_cal.c）
- foc.c：`Foc_Isr` 加 mode 25 分发 → `Foc_CalAngle_Step(pData)`；`Foc_Stop` 调 `Foc_CalAngle_Stop`
- foc.h：聚合 include foc_cal_angle.h
- dev_comm_runner.h：`COMM_RUNNER_CAL_ANGLE = 25`（.c 不动）
- foc_obs.c/.h：[CALANG] 事件处理
- Keil 工程：手动添加 foc_cal_angle.c

## 错误处理

- OC：原始 pData 三相电流，4 拍去抖 → FaultStop → evt → obs 打印 → 自动回 mode 0
- 编码器读取：TMRA_1 原始计数；位移用 int16 回绕差值逐拍累加（无 wrap 风险）

## 验证清单

1. 渐进 900/1800/2700/0：`g_foc_mech_deg` 每步 +9°，四步共 36°
2. 大跳变：0 直接设 2700 → 转子倒退到 351°
3. HOLD ok 时 err≈0；手扳转子 → FAIL（moved 超阈值）
4. VOFA 看 `g_calang_du/dv/dw` 三相正弦、互差 120°、随输入角变化；0.1° 步进扫描可见占空比平滑变化
5. mode 20 回归不受影响
