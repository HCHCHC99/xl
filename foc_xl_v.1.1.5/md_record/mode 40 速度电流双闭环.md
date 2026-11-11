# mode 40 编码器 FOC 速度/电流双闭环

> 记录日期：2026-09-21
> 状态：已实现并入轨（foc_speed40.c/h），内环参数沿用 mode 29 验证值，外环已初调
> 关联文档：`md_record/PI电流环mode 28记录.md`（内环物理结论）、`md_record/mode 24+29 校准闭环拆分.md`（校准来源）、`md_record/电流环PI理论计算.md`（内环 PI 理论）
> 关联流程图：`ob/FOC模式40流程图.canvas`

---

## 一、mode 40 是什么

**编码器 FOC 速度/电流双闭环**：在 mode 29 验证过的电流内环上面套一层速度外环，
构成经典的 **速度环 → 电流环 → SVPWM** 串级结构。

```
速度指令 → [速度 PI 5ms] → iq_ref (mA) ──┐
                            id_ref = 0  ──┤
iq 反馈 ←─ Park(转子角) ─────────────────┤
id 反馈 ←─ Park(转子角) ─────────────────┤
                            [电流 PI 20kHz] → vd/vq → 反Park → SVPWM
```

- 外环速度 PI 输出 = q 轴电流参考（mA），每 5ms 跑一次（1000 倍内环的抽稀）
- 内环电流 PI 每拍（20kHz）执行，id_ref 恒为 0（id=0 MTPA）
- 转子角来自 TMRA_1 编码器（与 mode 24/29 同框架：增量 ±32 限幅累积式）
- 前置：必须先跑 mode 24 校准（零偏 + offset），启动时校验 `Foc_Dcal24_GetResult()`

## 二、与 mode 28/29 的关系

| 维度 | mode 28/29（foc_dci） | mode 40（foc_speed40） |
|---|---|---|
| 控制对象 | 电流矢量（I_ref·sinδ） | 转速 |
| iq_ref 来源 | `g_dci_i_ref_ma·sinδ`（Watch 手动给） | **速度 PI 输出**（外环自动生成） |
| id_ref | `I_ref·cosδ`（δ 可调功角实验） | 恒 0（经典 FOC） |
| 外环 | 无 | 速度 PI（5ms 抽稀） |
| 校准来源 | mode 24/28 内置校准段 | **复用 mode 24 的 `foc_dcal24_result_t`** |
| 典型工况 | 调内环参数、功角实验 | 真实驱动应用 |

mode 40 的内环三参数（kp=0.5, ki=300, UMAX=3.5V, ITERM=3.2V）直接从 mode 29 验证值
复制过来（`SPEED40_PI_KP` 等宏），**内环物理结论全部继承 mode 28 记录**。

## 三、实现细节

### 3.1 文件结构

| 文件 | 职责 |
|---|---|
| `foc_speed40.h` | 接口、宏、观测量声明 |
| `foc_speed40.c` | 状态机、双环 ISR、编码器帧 |
| `foc_dcal24.h/c` | mode 24 独立校准模块（mode 40 启动时读结果） |
| `dev_comm_runner.c` | `COMM_RUNNER_SPEED_FOC=40` 分发 |
| `main.c` | VOFA CH4-9/14-16 通道映射 |

### 3.2 启动流程（Foc_Speed40_Start）

```
1. 前置检查：Foc_Dcal24_GetResult() → 无校准 → ERROR 拒绝启动
2. 复用校准快照：零偏 (zero_u/v/w_ma) + offset（按值拷贝，无运行时共享）
3. 编码器帧锚定：读 TMRA_1 当前计数 → s_rotor_count = mod(hw×dir − offset, CPR)
4. 清状态 + PID_Reset（三路：速度/id/iq）
5. 速度目标 = 0（安全起步，Watch 再给）→ PwmStart → 进 RUN
```

### 3.3 RUN 步骤（Foc_Speed40_Step，20kHz ISR）

```
1. OC 检查（原始采样）→ 过流则 FAULT_OC 停机
2. 编码器增量：hw 差分 → ±32 限幅 → ×dir → s_rotor_count 累积
3. 速度计算（Speed40_UpdateSpeed）：
   - 5ms 窗口累积 corrected_delta → raw_rpm = cnt×60×(1000/win_ms)/CPR
   - 一阶滤波：filt += 0.25×(raw − filt)（SPEED40_SPD_FILT_ALPHA）
4. 外环（Speed40_UpdateOuterLoop，5ms 一次）：
   - 目标斜率限制：±ACCEL_LIMIT_RPM_S（=MAX_SPEED×25% = 1950 rpm/s）
   - speed_error = ramp_rpm − filt_rpm
   - speed_out = PID_UpdateUs(速度环, ramp, filt, 5ms) → iq_ref (mA)
   - id_ref = 0
5. 转子电角：rotor_count × 2π×p/CPR → [0, 2π)
6. 电流反馈：零偏校正副本 → Clarke/Park(rotor_rad) → id/iq
7. iq 滤波（EMA α=0.10，仅显示用，PI 仍用原始 iq）
8. 内环 PI：vd = PID(id_ref=0, id), vq = PID(iq_ref, iq)
   - 限幅 ±UMAX=3.5V，积分限 ±ITERM=3.2V
9. 反 Park(rotor_rad) → SVPWM → TMR4 占空比
```

### 3.4 速度环参数（foc_speed40.h）

| 宏 | 值 | 含义 |
|---|---|---|
| `SPEED40_SPD_KP_MA_PER_RPM` | 1.8 | 速度环 P（mA/rpm） |
| `SPEED40_SPD_KI_MA_PER_RPM_S` | 0.2 | 速度环 I（mA/rpm/s） |
| `SPEED40_SPD_IQ_LIMIT_MA` | 3400 | 速度环输出限幅 = 额定电流×20%（17A×0.2） |
| `SPEED40_SPEED_REF_LIMIT_RPM` | 7800 | 转速指令上限（= MAX_SPEED） |
| `SPEED40_ACCEL_LIMIT_RPM_S` | 1950 | 加速度限幅（= MAX_SPEED×25%） |
| `SPEED40_SPD_WIN_MS` | 5 | 速度窗口（= 100 抽稀比 vs 20kHz ISR） |
| `SPEED40_SPD_FILT_ALPHA` | 0.25 | 速度反馈一阶滤波系数 |

### 3.5 内环参数（沿用 mode 29）

| 宏 | 值 | 来源 |
|---|---|---|
| `SPEED40_PI_KP` | 0.5 | mode 29 验证值（= R_eff 量级） |
| `SPEED40_PI_KI` | 300 | mode 29 验证值（≈ kp×R_eff/L_true） |
| `SPEED40_PI_UMAX_V` | 3.5 | 与 mode 28/29 同 |
| `SPEED40_ITERM_MAX_V` | 3.2 | > 天花板 BEMF 3.05V |
| `SPEED40_IQ_FILT_ALPHA` | 0.10 | iq 显示滤波（PI 不用） |

## 四、VOFA 通道映射（mode 40 运行时）

| CH | 内容 | 单位 |
|---|---|---|
| CH4 | id 反馈 (`g_speed40_id_ma`) | mA→A |
| CH5 | iq 反馈 (`g_speed40_iq_ma`) | mA→A |
| CH6 | id 参考 (=0) | mA→A |
| CH7 | iq 参考（=速度环输出） | mA→A |
| CH8 | vd 输出 | V |
| CH9 | vq 输出 | V |
| CH14 | 目标转速 (`g_speed40_speed_target_rpm`) | rpm |
| CH15 | 实际转速（滤波后，`g_speed40_speed_filt_rpm`） | rpm |
| CH16 | 滤波 iq（`g_speed40_iq_filt_ma`，EMA α=0.10） | mA→A |

## 五、Watch 变量速查

| 变量 | 说明 |
|---|---|
| `g_speed40_speed_target_rpm` | 速度目标（rpm），Watch 可调，自动限幅 ±7800 |
| `g_speed40_speed_filt_rpm` | 速度反馈滤波值（rpm） |
| `g_speed40_speed_meas_rpm` | 速度原始测量值（rpm，未滤波） |
| `g_speed40_speed_ramp_rpm` | 斜坡后的速度目标（rpm） |
| `g_speed40_speed_err_rpm` | 速度误差（rpm） |
| `g_speed40_speed_out_ma` | 速度环输出 = iq_ref（mA） |
| `g_speed40_iq_ref_ma` | q 轴电流参考（mA，= 速度环输出） |
| `g_speed40_iq_ma` / `g_speed40_id_ma` | 电流反馈（mA） |
| `g_speed40_iq_filt_ma` | 滤波 iq（mA，仅显示） |
| `g_speed40_iq_filt_alpha` | iq 滤波系数（Watch 可调，默认 0.10） |
| `g_speed40_vd` / `g_speed40_vq` | PI 输出电压（V） |
| `g_speed40_vsat` | 电压饱和标志（1=任一轴顶到 UMAX） |
| `g_speed40_rotor_deg` | 转子电角（deg，[0,360)） |
| `g_speed40_pid_speed_cfg .kp/.ki` | 速度环 PI（volatile，Watch 实时可调） |
| `g_speed40_pid_id_cfg / iq_cfg .kp/.ki` | 电流环 PI（volatile，Watch 实时可调） |
| `g_speed40_state` | 0 空闲 1 运行 2 过流 |
| `g_speed40_running` | 运行标志 |

## 六、标准工作流

```
1. comm_mode=24          → 等 RTT "cal-only done"（校准零偏 + offset）
2. mode 0 下确认转子自由（可选：轻拨看 g_foc_elec_deg 跟随）
3. comm_mode=40          → 速度/电流双闭环启动（目标=0，安全起步）
4. Watch 置 g_speed40_speed_target_rpm = 500   → 电机加速到 500rpm
5. 观察：
   - CH15（实际转速）追踪 CH14（目标转速），无超调/不振铃
   - CH5（iq）跟随 CH7（iq_ref），内环跟踪良好
   - CH8/CH9（vd/vq）不顶到 UMAX（g_speed40_vsat=0）
6. 调外环：Watch 改 g_speed40_pid_speed_cfg.kp/.ki
   - 超调大 → 降 kp 或降 ki
   - 跟踪慢 → 升 kp
7. 调内环（通常不动）：g_speed40_pid_iq_cfg.kp/.ki（沿用 mode 29 值）
```

## 七、与 mode 22（foc_curloop）的对比

| 维度 | mode 22（foc_curloop） | mode 40（foc_speed40） |
|---|---|---|
| 启动 | I-F 启动（合成角拖动→同步→切编码器） | 直接编码器角（需先 mode 24 校准） |
| 外环 | 速度 PI | 速度 PI |
| 内环 | 电流 PI（id=0） | 电流 PI（id=0） |
| 校准 | 内置对齐校准 | 独立 mode 24 模块 |
| 适用 | 无编码器零点场景 | 已有 mode 24 校准的精确控制 |

mode 40 省去了 I-F 启动的同步切换复杂度，直接用 mode 24 锁定的编码器零点，
内环参数有 mode 28/29 的完整理论+实验支撑。

## 八、当前状态与待办

- [x] foc_speed40.c/h 实现（双环 ISR、编码器帧、OC 保护）
- [x] dev_comm_runner 集成（COMM_RUNNER_SPEED_FOC=40）
- [x] VOFA 通道映射（CH4-9/14-16）
- [x] 内环参数沿用 mode 29 验证值
- [x] 速度环初调（kp=1.8, ki=0.2）
- [ ] 速度环参数精调（阶跃响应、抗扰）
- [ ] 全转速范围验证（0→7800rpm）
- [ ] 加减速斜率优化（当前 1950 rpm/s）
