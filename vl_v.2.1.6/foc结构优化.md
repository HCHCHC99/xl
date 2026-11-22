# FOC 结构优化方案

- 代码基线：`D:\xl\vl_v.2.0.5`（HC32F460 + RT-Thread）；本次实际工作副本 `F:\xl_v\code\vl_v.2.1.5`
- 范围：仅重构 `applications/motor/foc/` 的内部结构与文件边界，**不接入业务层、不改控制算法、不改 PWM 底层配置**
- 日期：2026-11-20（Phase 1~4 全部完成后更新）
- 前置结论：当前 FOC 核心已可独立运行（msh 驱动），是重构成本最低的窗口期；一旦业务层通过 `swing_axis` 依赖 FOC 接口，再改代价会明显上升。

---

## 0. 实施进度（2026-11-20）

**Phase 1 + Phase 2 + Phase 3 + Phase 4-lite + 完整 Phase 4 已完成，工程全量编译通过。**

### 0.1 各阶段结果

| 阶段 | 内容 | 结果 |
| --- | --- | --- |
| Phase 1 | 驱动/调试搬迁 | `hw_tmr4_pwm.*`、`hw_encoder.*` → `drv/`；`foc_vofa.*`、`foc_eoca_probe.*` → `diag/` |
| Phase 2 | 拆 CLI | `foc_mode.c` 1219 → 414 行；新增 `foc/cli/foc_cli.c` |
| Phase 3 | run/proc 拆分 | `foc_mode.c/.h` 删除；新增 `foc/core/foc_run.*`、`foc/proc/foc_proc.*`；**实机通过** |
| Phase 4-lite | 访问点收敛 | 新增 `foc/core/foc_output.*`、`foc/core/foc_feedback.*`；50 处写 / 12 处读收敛；**实机通过** |
| 完整 Phase 4 | 执行器化 | `FocCmd_t.kind` + `foc_control.c` 纯执行器；删 2%/`s_last_static_mode`/`ReapplyStatic`；构建通过，**实机待验证** |
| 代码尺寸 | — | **GCC 10.3.1（env tools，本次验证用）：text 386392 / data 3548 / bss 34448**；**GCC 5.4.1（RT-Thread Studio 构建，烧录用）：text 376512 / data 3552 / bss 34436**。两个工具链差异约 9.8 KB 属正常 codegen 差异，不要混用基线条目（详见 §15.5） |
| 构建方式 | — | 本机必须**串行** `make`（`-j` 会 100% CPU 自旋）；`make all` 的 hex 递归 make 会报 `Error 127`（elf/hex 实际已生成），可改用 `make rtthread.elf rtthread.siz` |

### 0.2 当前文件状态

| 文件 | 行数 | 位置 | 说明 |
| --- | ---: | --- | --- |
| `foc_run.c` / `.h` | 304 / 94 | `foc/core/` | 6 个运行模式、`SetMode`（只改状态）、`Step20k` 每拍下发 `kind`、LOCK 占空比 |
| `foc_output.c` / `.h` | 55 / 57 | `foc/core/` | 运行期写桥臂的唯一入口（含 `Foc_Output_SafeStop()`、`foc_out_kind_t`） |
| `foc_feedback.c` / `.h` | 57 / 51 | `foc/core/` | 运行期读编码器的唯一入口 |
| `foc_proc.c` / `.h` | 238 / 62 | `foc/proc/` | 3/5/6/7 编排 + 12/13 零偏校准 + 上电零偏 |
| `foc_control.c` | 422 | `foc/` | 20 kHz 宿主 + **纯输出执行器**（`s_out_state` 去重）；1 ms 里仍有过流判断/流程轮询/调试打印 |
| `encoder_calib.c` | 976 | `foc/` | 标定状态机；按设计仍直连 `hw_encoder`/`foc_output`（标定域） |
| `foc_protect.c` | 311 | `foc/` | 过流保护；trip → `Foc_Output_SafeStop()` |
| `foc_current_loop.c` | 534 | `foc/` | 电流环；已改走 `Foc_Feedback_*` |
| `foc_speed_loop.c` | 345 | `foc/` | 速度环；已改走 `Foc_Feedback_GetDir()` |
| `foc_position_loop.c` | 276 | `foc/` | 位置环；已改走 `Foc_Feedback_*` |
| `foc_cli.c` | 856 | `foc/cli/` | `focmode`、短命令、参数表、dump、`fstat`（旧数字只在此映射） |
| `hw_tmr4_pwm.c` | 1181 | `drv/` | 硬件驱动（**本次未改**） |
| `hw_encoder.c` | 344 | `drv/` | ABZ 驱动（**本次未改**） |
| `foc_vofa.c` | 461 | `diag/` | VOFA 调试输出（Phase 5 才动） |
| `foc_eoca_probe.c` | — | `diag/` | EOCA 观测探针 |
| `foc_mode.c` / `.h` | — | — | **已删除**（Phase 3） |

### 0.3 Phase 1/2 已解决的问题

- `foc/` 顶层不再混放 `hw_*` 驱动文件。
- `foc/` 顶层不再混放 VOFA/EOCA 调试文件。
- `foc_mode.c` 不再是“模式 + 参数表 + 打印 + msh 命令”的上帝文件。
- `focmode` 命令、短命令、参数表、打印格式保持不变。
- IDE 构建已能正确找到 `motor/drv`、`motor/foc`、`motor/foc/cli`、`motor/diag`。

### 0.4 仍然存在的问题

Phase 1/2 只解决了“文件放在哪”和“CLI 混在核心里”，以下架构问题仍在：

1. ~~`mode 几` 仍然混淆运行模式和一次性标定流程（3/5/6/7/12/13）。~~ **已在 Phase 3 解决**：拆成 `foc_run_mode_t`（6 值）与 `foc_proc_t`，`foc_mode.c/.h` 已删除。
2. ~~`foc_control.c` 仍然自己判断 IDLE/LOCK 并写 2% 静态占空比。~~ **已在完整 Phase 4 解决**：`foc_control.c` 只按 `FocCmd_t.kind` 执行，2% 魔数删除。
3. ~~电流/速度/位置环仍直接读 `hw_encoder` 全局。~~ **已在 Phase 4-lite 解决**：统一走 `Foc_Feedback_*`；`encoder_calib.c` 按设计保留直连。
4. `foc_vofa.c` 虽然搬到了 `diag/`，但仍直接依赖 6 个 FOC 模块的全局。（Phase 5 处理）
5. FOC 仍然知道旧六步（`FOC_DISABLE_LEGACY_6STEP`、`Foc_Run_PwmOwned`）。**仅部分解决**：旧六步的三个桥臂写点已加 `Foc_Run_PwmOwned()` 门控，但门控本身仍在 `foc_run.h` 里（Phase 4 有意不扩散改动面）。
6. `foc_protect` 与 `motor_stall` 仍是两个保护所有者。（Phase 4 之后的独立项，见 §14.9 审计 M1）
7. ~~模式切换仍是线程直接改 `s_mode` + 写 PWM，缺少 ISR 侧的所有权约定。~~ **输出侧已解决**：`SetMode` 不再写任何桥臂，输出由 ISR 按 kind 落实；线程仍改模式*状态*，但不再碰寄存器。
8. `foc_control.c` 的 1 ms 定时器里仍有 IF/overrun 调试打印。（Phase 5 处理）

### 0.5 Phase 4-lite 已完成（2026-11-20）

在完整 Phase 4 之前先做了低风险的「访问点收敛」，详见 §13：

- 新增 `foc/core/foc_output.{c,h}`（运行期写桥臂的唯一入口，含 `Foc_Output_SafeStop()`）和 `foc/core/foc_feedback.{c,h}`（运行期读编码器的唯一入口）。
- **50 处**运行期写桥臂调用收敛到 `foc_output`（`foc_run.c` 22、`foc_proc.c` 11、`encoder_calib.c` 9、`foc_control.c` 5、`foc_protect.c` 1、`foc_pwm.c` 2 保留为配置期）。
- **12 处**环路/宿主读编码器收敛到 `foc_feedback`（`foc_control.c` 6、`foc_current_loop.c` 3、`foc_position_loop.c` 2、`foc_speed_loop.c` 1）；`encoder_calib.c` 的 14 处按设计保留（标定域需要原始计数）。
- 行为零变化：数值常量、分支、错误处理、打印文本全部原样；全量构建通过，text 386696（+104）。
- **实机验证通过（2026-11-20，用户确认）**。

**完整 Phase 4 已完成（2026-11-20，§14）**：`FocCmd_t` 带 `kind`、`foc_control.c` 变纯执行器、删除 `s_last_static_mode`/2% 魔数/`Foc_Run_ReapplyStatic()`；独立审计后补了旧六步门控与 trip 兜底，详见 §14.9。

---

## 1. 结论

建议做，但按“**保行为、分阶段、可回退**”的方式做，不要一次性重写。

- 你的直觉是对的：`mode 几` 的写法不好，`foc/` 目录也存在职责越界。
- 问题不在编号本身，而在于**把“持续运行模式、一次性标定流程、调试命令”三类东西塞进了同一个枚举和同一个文件**。
- 最大收益、最低风险的两步（Phase 1/2）已经完成：驱动/调试文件搬出 `foc/`，CLI 拆到 `foc_cli.c`，工程全量编译通过。
- **Phase 3（run/proc 拆分）已完成**：`foc_mode.c/.h` 删除，新增 `foc/core/foc_run.c/.h` 与 `foc/proc/foc_proc.c/.h`；全量构建通过，详见 §12.10。
- **Phase 4-lite（输出/反馈收敛）已完成**：新增 `foc/core/foc_output.c/.h`、`foc/core/foc_feedback.c/.h`，48 处运行期写桥臂 + 12 处读编码器收敛到两个模块；行为零变化，详见 §13.10。
- **完整 Phase 4 已完成**：`FocCmd_t` 增加 `kind`、`foc_control.c` 变纯执行器、删除 `s_last_static_mode`/`s_if_resumed`/2% 魔数/`Foc_Run_ReapplyStatic()`；独立审计后补了旧六步门控与 trip 的 `SafeStop` 兜底，详见 §14.9。
- **FOC 侧重构到此收尾**：下一步回到业务层——P0-1 绝对角路线决策 → P0-2 `swing_axis`（直接调 `Foc_Feedback_*` / `Foc_Run_SetMode`）。
- Phase 5（参数/遥测集中）在 Phase 4 之后再做；不建议一次性重写。

---

## 2. 重构前基线数据（2026-11-19）

> 下表是 Phase 1/2 之前的基线，保留用于对照。`foc_mode.c` 在 Phase 2 降到 414 行、**Phase 3 已删除**；CLI 在 `foc/cli/foc_cli.c`，运行模式在 `foc/core/foc_run.c`，流程在 `foc/proc/foc_proc.c`，驱动在 `motor/drv/`，调试在 `motor/diag/`。

| 文件 | 行数 | 大小 | 说明 |
| --- | ---: | ---: | --- |
| `foc_mode.c` | 1219 | 42.9 KB | 上帝文件：核心模式 + 参数表 + 打印 + msh 命令 + 硬件动作 |
| `hw_tmr4_pwm.c` | 1181 | 43.0 KB | 全目录最大；本质是硬件驱动，不属于 FOC 控制 |
| `encoder_calib.c` | 976 | 28.6 KB | 标定流程状态机 |
| `foc_current_loop.c` | 534 | 14.8 KB | 电流环 |
| `foc_vofa.c` | 461 | 19.7 KB | 调试输出，不属于 FOC 控制 |
| `foc_control.c` | 411 | 13.0 KB | 20 kHz 宿主，但混入了模式输出策略和调试打印 |
| `foc_speed_loop.c` | 345 | 10.2 KB | 速度环 |
| `hw_encoder.c` | 344 | 11.0 KB | 硬件编码器驱动 |
| `foc_protect.c` | 308 | 9.4 KB | FOC 过流保护 |
| `foc_position_loop.c` | 276 | 7.7 KB | 位置环 |

`foc_mode.c` 中：

- 约 1～400 行是真正的模式核心（Init/SetMode/Step/Stop/SetLockDuty/PwmOwned）；
- 约 402～1219 行是参数表、格式化打印、msh 命令、短命令、诊断；
- 也就是说，**约 2/3 的文件不是控制逻辑**。

Phase 2 已把上述 402～1219 行拆到 `foc/cli/foc_cli.c`，现在 `foc_mode.c` 只剩控制核心。

---

## 3. 重构前问题诊断（保留对照）

> 本节是 2026-11-19 重构前的诊断，保留作历史对照。**Phase 3/4 之后的实际状态见 §0.2/§0.4/§9**：3.1/3.2/3.3/3.4/3.7/3.9 已解决或大幅改善，3.5（VOFA 依赖）、3.8（保护两个所有者）、3.10（控制宿主里的打印）留待 Phase 5 或独立项。

### 3.1 `mode 几` 混淆了三类语义

| 类别 | 当前编号 | 实际语义 | 问题 |
| --- | --- | --- | --- |
| 持续运行模式 | 0/1/2/4/9/10 | IDLE、LOCK、IF、CURRENT、SPEED、POSITION | 进入后保持，适合作为 `run_mode` |
| 一次性标定流程 | 3/5/6/7/12/13 | 编码器对齐、扫角、测圈、斜坡对齐、零偏校准 | 做完应回 IDLE，不应长期占据 `s_mode` |
| 调试命令 | `stat`、`set`、`dump` | 只读诊断或改参数 | 根本不是模式 |

证据：

- mode 12/13 在 `foc_mode.c:247-288` 里先把 `s_mode` 强制改回 IDLE，再阻塞采样；这是流程，不是模式。
- mode 3 进入后由 `encoder_calib` 状态机推进，完成后回 IDLE；它也不该长期作为运行模式。
- 编号 8/11/14 空缺，是参考工程遗留，不必强行补齐；内部类型不应带空洞。

### 3.2 `foc_mode.c` 是上帝文件

- 一个文件同时负责：模式 FSM、模式切换、PWM/EMB 硬件动作、参数表、格式化打印、msh 命令分发、短命令、诊断 dump。
- `Foc_Mode_SetMode()`（`foc_mode.c:150-288`）直接调用：
  - `hw_tmr4_pwm_set_all_low()`
  - `hw_tmr4_pwm_resume_output()`
  - `hw_tmr4_pwm_set_duty3()`
  - `bsp_pwm_emergency_brake()`
  - `bsp_pwm_emergency_release()`
  - `rt_thread_mdelay()`
- 结果是：模式层既做策略，又做硬件，又做阻塞等待，还做打印。

### 3.3 控制层和模式层重复决定输出

`foc_control.c:250-265` 里有第二套“模式输出策略”：

- `g_fm_mode == FOC_MODE_IDLE` → 全低；
- `g_fm_mode != FOC_MODE_LOCK` → 恢复输出并写 2% 零矢量；
- 用 `s_last_static_mode` 记录上一次模式。

而 `Foc_Mode_SetMode()` 里已经处理过 IDLE/LOCK/恢复输出。两层都在决定“这一拍该输出什么”，职责重叠。正确做法是：**模式层产出 `FocCmd_t`，控制层只执行**。

### 3.4 环路直接依赖驱动/全局

- `foc_position_loop.c:59-65` 直接读 `hw_encoder_get_count()` 和 `g_enc_elec_dir`。
- `foc_current_loop.c:10-15` 同时包含 `hw_encoder.h`、`foc_protect.h`、`foc_control.h`，并读 `g_fc_id/g_fc_iq`。
- `foc_speed_loop.c` 也依赖编码器增量/全局观测。

后果：环路无法脱离硬件单测；换编码器或换采样来源要改环路文件。

### 3.5 调试文件反向依赖 6 个 FOC 模块

`foc_vofa.c:24-30` 同时包含：

- `foc_control.h`
- `foc_current_loop.h`
- `hw_encoder.h`
- `foc_mode.h`
- `foc_speed_loop.h`
- `foc_position_loop.h`

它是调试输出，不是 FOC 控制。放在 `foc/` 里会让“FOC 核心”看起来依赖一大片全局变量，也让量产裁剪不干净。

### 3.6 驱动文件混在 `foc/`

- `hw_tmr4_pwm.c/.h`：TMR4 PWM/死区/EVT/寄存器诊断，是硬件驱动。
- `hw_encoder.c/.h`：TMRA1 ABZ 计数/测速/诊断，是硬件驱动。

它们放在 `foc/` 下，导致目录名是“算法域”，内容却是“驱动 + 算法 + 调试 + CLI”的混合。

### 3.7 FOC 知道旧六步

- `foc_mode.h:19-21` 定义 `FOC_DISABLE_LEGACY_6STEP`。
- `foc_mode.h:70-71` 暴露 `Foc_Mode_PwmOwned()`。

FOC 不应该知道“旧六步电机对象”是否存在。这个门控属于 `motor` 层的兼容逻辑，或者直接把旧路径编译移除。

### 3.8 保护有两个所有者

- `foc_protect.c`：FOC 相电流保护，trip 后切 IDLE。
- `motor_stall.c`：母线电流/低速堵转，trigger 后调 `motor_stop()`（FOC 下是空操作）+ 抱闸。

两边都叫“保护”，但停机路径不统一；FOC 下堵转甚至停不了 PWM。应该统一到一个 `foc_safe_stop()`，两个检测源都调用它。

### 3.9 线程/ISR 共享状态缺少所有权

- `focmode` 线程写 `s_mode`、`g_align_active`、PWM 寄存器；20 kHz ISR 同时读/写同一批状态。
- 文档里已经记录过 `g_align_active` 的启动竞态（`移植过程.md` 10.9），说明这类“线程直接改 ISR 状态”的模式是脆弱的。
- 重构时应明确：**线程只发请求，ISR 在安全点完成模式切换**，或者切换期间短暂屏蔽 ADC ISR。

### 3.10 控制宿主里做调试打印

`foc_control.c:272-312` 的 1 ms 定时器里打印 IF 状态和 ISR overrun。打印本身不在 20 kHz 里，但把调试逻辑放进了控制宿主，增加了控制文件的职责。

---

## 4. 目标结构

```
applications/motor/
├─ drv/
│  ├─ hw_tmr4_pwm.c/.h        // 从 foc/ 搬出
│  ├─ hw_encoder.c/.h         // 从 foc/ 搬出
│  └─ bsp_adc_foc.c/.h        // 已有
├─ foc/
│  ├─ core/
│  │  ├─ foc_control.c/.h     // 20kHz 宿主：采样→变换→执行指令
│  │  ├─ foc_run.c/.h         // 六个运行模式
│  │  ├─ foc_output.c/.h      // all-low / resume / duty3 / brake
│  │  ├─ foc_feedback.c/.h    // 编码器抽象
│  │  ├─ foc_protect.c/.h     // 过流保护
│  │  └─ foc_params.c/.h      // 参数集中（可选）
│  ├─ loops/
│  │  ├─ foc_current_loop.c/.h
│  │  ├─ foc_speed_loop.c/.h
│  │  ├─ foc_position_loop.c/.h
│  │  └─ pid_ctrl.c/.h
│  ├─ math/
│  │  ├─ clarke_park.c/.h
│  │  ├─ svpwm.c/.h
│  │  └─ foc_sincos.c/.h
│  ├─ proc/
│  │  ├─ encoder_calib.c/.h   // 原 3/5/6/7
│  │  └─ foc_offset_cal.c/.h  // 原 12/13（可从 foc_mode.c 拆出）
│  ├─ config/
│  │  ├─ cfg_foc_table.h
│  │  ├─ cfg_motor.h
│  │  └─ cfg_sampling.h
│  └─ cli/
│     └─ foc_cli.c            // focmode、短命令、dump、参数表
└─ diag/
   ├─ foc_vofa.c/.h
   └─ foc_eoca_probe.c/.h
```

依赖方向：

```
diag → cli → core → loops/math → drv
                 ↑
               proc
```

`core` 不包含 msh/打印/参数表；`drv` 不包含模式语义；`diag` 只读遥测快照。

### 4.1 文件搬迁映射

| 当前文件 | 目标位置 | 处理方式 |
| --- | --- | --- |
| `foc/hw_tmr4_pwm.c/.h` | `motor/drv/hw_tmr4_pwm.c/.h` | 纯搬迁 |
| `foc/hw_encoder.c/.h` | `motor/drv/hw_encoder.c/.h` | 纯搬迁 |
| `foc/foc_vofa.c/.h` | `motor/diag/foc_vofa.c/.h` | 纯搬迁 |
| `foc/foc_eoca_probe.c/.h` | `motor/diag/foc_eoca_probe.c/.h` | 纯搬迁 |
| `foc/encoder_calib.c/.h` | `foc/proc/encoder_calib.c/.h` | 纯搬迁 |
| `foc/clarke_park.c/.h` | `foc/math/clarke_park.c/.h` | 纯搬迁 |
| `foc/svpwm.c/.h` | `foc/math/svpwm.c/.h` | 纯搬迁 |
| `foc/foc_sincos.c/.h` | `foc/math/foc_sincos.c/.h` | 纯搬迁 |
| `foc/pid_ctrl.c/.h` | `foc/loops/pid_ctrl.c/.h` | 纯搬迁 |
| `foc/foc_current_loop.c/.h` | `foc/loops/` | 纯搬迁 |
| `foc/foc_speed_loop.c/.h` | `foc/loops/` | 纯搬迁 |
| `foc/foc_position_loop.c/.h` | `foc/loops/` | 纯搬迁 |
| `foc/foc_protect.c/.h` | `foc/core/` | 纯搬迁 |
| `foc/foc_control.c/.h` | `foc/core/` | 搬迁 + 后续去模式策略 |
| `foc/foc_pwm.c/.h` | `foc/core/` | 纯搬迁 |
| `foc/cfg_*.h` | `foc/config/` | 纯搬迁 |
| `foc/foc_mode.c` | 拆成 `foc/core/foc_run.c`、`foc/proc/*`、`foc/cli/foc_cli.c` | 逻辑拆分 |
| 新增 | `foc/core/foc_output.c/.h`、`foc_feedback.c/.h`、`foc_params.c/.h` | 新接口 |

### 4.2 构建脚本同步

当前 `applications/motor/SConscript` 只匹配：

```python
src = Glob('*.c') + Glob('drv/*.c') + Glob('fb/*.c') + Glob('ctrl/*.c') + Glob('foc/*.c')
```

搬迁后必须增加：

```python
src = (Glob('*.c') + Glob('drv/*.c') + Glob('fb/*.c') + Glob('ctrl/*.c')
       + Glob('foc/*/*.c') + Glob('diag/*.c'))
```

并补充 `foc/core`、`foc/loops`、`foc/math`、`foc/proc`、`foc/config`、`foc/cli`、`diag` 的 `CPPPATH`。

Eclipse/RT-Thread Studio 的 `Debug/applications/motor/foc/subdir.mk` 是手工维护的（`移植过程.md` 已提到），搬迁后也要同步，否则 IDE 构建会出现“源文件不存在/缺少符号”。

---

## 5. 模式重构

### 5.1 内部类型拆成两个

```c
typedef enum {
    FOC_RUN_IDLE = 0,
    FOC_RUN_LOCK,
    FOC_RUN_IF,
    FOC_RUN_CURRENT,
    FOC_RUN_SPEED,
    FOC_RUN_POSITION,
} foc_run_mode_t;

typedef enum {
    FOC_PROC_ENC_ALIGN = 0,
    FOC_PROC_SWEEP,
    FOC_PROC_REV,
    FOC_PROC_ALIGN7,
    FOC_PROC_OFFSET_NOPWM,
    FOC_PROC_OFFSET_PWM50,
} foc_proc_t;
```

### 5.2 对外 API

```c
/* 运行模式 */
void          Foc_Run_SetMode(foc_run_mode_t mode);
foc_run_mode_t Foc_Run_GetMode(void);
void          Foc_Run_Step20k(float ts, FocCmd_t *out);

/* 一次性流程 */
int  Foc_Proc_Start(foc_proc_t proc);
void Foc_Proc_Poll1ms(void);
void Foc_Proc_Abort(void);
bool Foc_Proc_IsBusy(void);
```

### 5.3 `focmode` 兼容层

`focmode` 的数字只保留在 `foc_cli.c`，作为“用户命令 ID”，不再作为内部状态：

| 命令 | 映射 |
| --- | --- |
| `focmode 0` / `idle` | `Foc_Run_SetMode(FOC_RUN_IDLE)` |
| `focmode 1 [duty]` / `lock` | `Foc_Run_SetMode(FOC_RUN_LOCK)` |
| `focmode 2 [f] [v]` / `ifrun` | `Foc_Run_SetMode(FOC_RUN_IF)` |
| `focmode 4 [id] [iq]` / `current` | `Foc_Run_SetMode(FOC_RUN_CURRENT)` |
| `focmode 9 [rpm]` / `speed` | `Foc_Run_SetMode(FOC_RUN_SPEED)` |
| `focmode 10 [rev]` / `position` | `Foc_Run_SetMode(FOC_RUN_POSITION)` |
| `focmode 3` / `falign` | `Foc_Proc_Start(FOC_PROC_ENC_ALIGN)` |
| `focmode 5` | `Foc_Proc_Start(FOC_PROC_SWEEP)` |
| `focmode 6` | `Foc_Proc_Start(FOC_PROC_REV)` |
| `focmode 7` | `Foc_Proc_Start(FOC_PROC_ALIGN7)` |
| `focmode 12` / `zcal` | `Foc_Proc_Start(FOC_PROC_OFFSET_NOPWM)` |
| `focmode 13` / `zcal50` | `Foc_Proc_Start(FOC_PROC_OFFSET_PWM50)` |
| `focmode stat` / `fstat` | 只读诊断 |

这样团队习惯、文档、脚本、VOFA 快捷命令都不变，但控制代码里不再出现 `mode 3` 这种语义。

### 5.4 运行模式分发

两种写法二选一：

1. **保守写法**：保留 `switch (s_run_mode)`，但每个分支调用独立的小函数（`foc_run_idle_step()` 等）。可读性提升，热路径不变。
2. **表驱动写法**：`static const foc_run_ops_t s_run_ops[]`，包含 `enter/exit/step20k/task1ms`。

建议先做第 1 种；如果之后仍觉得分支太长，再评估表驱动。20 kHz 下多一次间接调用通常可接受，但必须用 DWT 实测；若 ISR 最大值回退超过 5%～10%，就保留 `switch`。

### 5.5 输出策略收口

建议让 `FocCmd_t` 明确表达“本拍输出什么”，例如：

```c
typedef enum {
    FOC_OUT_NONE = 0,
    FOC_OUT_VECTOR,     /* 用 vd/vq/theta 计算 SVPWM */
    FOC_OUT_DUTY3,      /* 直接写三相占空比（LOCK/静态测试） */
    FOC_OUT_ALL_LOW,    /* 全低 */
} foc_out_kind_t;

typedef struct {
    bool           enable;
    foc_out_kind_t kind;
    float          vd, vq, theta_e;
    float          duty_u, duty_v, duty_w;
} FocCmd_t;
```

`foc_control.c` 只按 `kind` 执行，不再判断 `FOC_MODE_IDLE/LOCK`，也不再写 2% 魔数。模式层在 `enter/exit` 里决定静态输出。

### 5.6 模式切换的线程/ISR 所有权

建议：

- 线程侧只设置 `s_run_req` 和 `s_run_req_seq`；
- ISR 在 `Foc_Run_Step20k()` 开头检查请求，完成 `exit/enter` 后再进入正常步进；
- 或者在 `Foc_Run_SetMode()` 期间短暂屏蔽 ADC ISR，切换完成后再开。

两种都比“线程直接改 `s_mode` + 写 PWM”更可控。标定流程的 `g_align_active` 竞态也可以按同一模式收口。

---

## 6. 分阶段改造计划

### Phase 0：建立基线

在任何改动前记录：

- `focmode 0`、`focmode 1 5`、`focmode 2 1 1.5`、`focmode 3`、`focmode 4 0 0.5`、`focmode 9 200`、`focmode 10 0`、`focmode 12`、`focmode 13`
- `fstat`、`t_abz`、`t_iadc`、`t_iadc_stat`、`t_pwmtime 1000`、`t_vofa`
- VOFA 19 通道波形
- `g_fc_isr_dt_min/max/avg`、`g_dwt_isr_min/max/avg_cyc`
- 三相占空比、ADC 触发点、EOCA 探针

把结果记在本文件附录或单独的测试记录里，作为后续对比基线。

### Phase 1：纯搬迁（已完成 2026-11-20）

- `hw_tmr4_pwm.*`、`hw_encoder.*` → `motor/drv/`
- `foc_vofa.*`、`foc_eoca_probe.*` → `motor/diag/`
- 更新 `SConscript` 与 Eclipse `subdir.mk`
- 只改 include 路径，不改任何函数体

验收：

- 编译通过；
- 所有 `focmode`/`fstat`/`t_abz`/`t_vofa` 输出与基线一致；
- ISR 时间与基线一致。

实际结果：文件搬迁完成，`SConscript` 与 `.cproject` 已同步；Clean 构建通过。

### Phase 2：拆出 CLI（已完成 2026-11-20）

把 `foc_mode.c` 的以下内容移到 `foc/cli/foc_cli.c`：

- `foc_mode_pct_x100`、`foc_mode_dump_pwm`、`foc_mode_parse_float`
- `foc_mode_print_status`、`foc_mode_print_duties`、`foc_mode_print_params`
- `fm_print_num`、`fm_scale`、`fm_ack`
- `foc_mode_set_param`、`foc_param_cmd`、`FOC_PARAM_CMD`
- `foc_mode_msh` 及所有 `cmd_foc_*` 短命令

`foc_mode.c` 只保留：

- `Foc_Mode_Init/SetMode/GetMode/Step/SetOpenloop/Stop/SetLockDuty/GetLockDuty/PwmOwned`
- 内部状态与模式分支

验收：

- `foc_mode.c` 降到约 400 行；
- `focmode` 的所有帮助、错误提示、参数表输出保持不变；
- 短命令 `idle/lock/ifrun/current/speed/position/falign/zcal/zcal50/fstat` 不变。

实际结果：`foc_mode.c` 414 行，`foc/cli/foc_cli.c` 843 行；`make clean2 && make -j32 all` 通过，`rtthread.elf` 正常生成。

### Phase 3：拆 run / proc（真正的模式重构）

- 新增 `foc/core/foc_run.c/.h`：只保留 6 个运行模式；
- 新增 `foc/proc/`：把 3/5/6/7/12/13 改为 `Foc_Proc_*`；
- `foc_cli.c` 保留旧数字映射；
- `Foc_Mode_Step` 改为 `Foc_Run_Step20k`；
- 标定流程通过 `Foc_Proc_Start/Poll1ms/Abort` 调用，不再进入 `s_run_mode`。

验收：

- `focmode 3` 的对齐打印、`zero_hw/beta_hw/alpha_hw/moved/offset/dir` 与基线一致；
- `focmode 12/13` 的 `[CAL] OK`、回 IDLE 行为一致；
- `focmode 4/9/10` 的电流/速度/位置响应与基线一致；
- `focmode stat` 仍能显示当前 run 模式或 proc 状态。

### Phase 4：输出/反馈接口化

- 新增 `foc_output.c/.h`：`all_low/resume/duty3/brake/release`；
- 新增 `foc_feedback.c/.h`：`count/mech_rad/elec_rad/speed_rpm/dir`；
- 模式层不再 include `hw_tmr4_pwm.h`、`bsp_pwm.h`；
- 环路不再 include `hw_encoder.h`；
- `foc_control.c` 只执行 `FocCmd_t`，不再判断模式；
- 删除 `s_last_static_mode` 和 2% 魔数。

验收：

- 三相占空比、寄存器回读（OCCR/PDAR/PDBR/OCER）与基线一致；
- `focmode 1 5` 的 `cmpH/cmpL` 与基线一致；
- ISR 时间不超过基线 +5%；
- 模式 0/1/2/4/9/10 行为不变。

### Phase 5（可选）：参数与遥测集中

- `foc_params.c/.h` 收拢 `g_cl_*`、`g_spd_*`、`g_pos_*`、保护阈值；
- `foc_telemetry.c/.h` 提供一次性快照；
- `foc_vofa.c` 和 `foc_cli.c` 只读快照，不再直接引用 6 个模块的全局变量。

验收：

- VOFA 19 通道顺序、单位、缩放不变；
- `focmode stat`/`fstat` 字段不变；
- 参数写入仍然即时生效，不引入锁或阻塞。

---

## 7. 回归清单

| 类别 | 项目 |
| --- | --- |
| 模式 | `focmode 0/1 5/2 1 1.5/3/4 0 0.5/9 200/10 0/12/13` |
| 诊断 | `fstat`、`t_abz`、`t_iadc`、`t_iadc_stat`、`t_pwmtime 1000`、`t_vofa` |
| 波形 | VOFA 19 通道；U/V/W 相电压/相电流；EOCA 探针 |
| 寄存器 | OCCR/PDAR/PDBR/OCER/OCSR 回读 |
| 时间 | `g_fc_isr_dt_min/max/avg`、`g_dwt_isr_min/max/avg_cyc` |
| 安全 | mode 0 全低；mode 1 指定占空比；mode 3 完成回 IDLE；mode 12/13 完成回 IDLE；过流 trip 切 IDLE |
| 构建 | SCons 构建 + Eclipse/Studio 构建都能通过 |

建议每一步单独提交，并保留“改前/改后”日志或截图。

---

## 8. 不建议做的事

1. **不要重写 Clarke/Park/SVPWM/PID/编码器对齐**：算法已跑通，重构收益低、风险高。
2. **不要上 C++ 或重型 OOP**：C 结构体 + 函数表足够；20 kHz 路径不需要复杂对象模型。
3. **不要一上来做函数表**：先把代码按职责搬出去，再决定 `switch` 还是表驱动；表驱动必须用 DWT 实测。
4. **不要多实例化**：本机只有一个回转电机，static 状态改 context 指针只会增加间接寻址。
5. **不要在 20 kHz 里加锁、打印、动态内存或长调用链**；参数更新保持 32 位单写或后续做版本号/双缓冲。
6. **不要改 `focmode` 命令名、打印格式、VOFA 通道顺序**：它们是现有的回归工具。
7. **不要把这次重构和业务层接入混在一起**：先让 FOC 结构干净，再让 `swing_axis` 依赖它。

---

## 9. 验收标准（整体）

> 以下为 Phase 1~4 完成后的整体目标。✅ = 已达成，⬜ = 留待 Phase 5 或独立项。

- ✅ `foc_mode.c` 已删除，模式/流程/CLI 分别在 `foc_run.c`、`foc_proc.c`、`foc_cli.c`；
- ✅ `foc/` 下不再有 `hw_*` 驱动文件和 `*_vofa/*_eoca_probe` 调试文件；
- ✅ 模式代码中不再出现 `FOC_MODE_3/5/6/7/12/13` 这类“流程编号”（只剩 CLI 兼容映射 `Foc_Run_LegacyId`/`Foc_Proc_LegacyId`）；
- ✅ 模式层不直接调用 `hw_tmr4_pwm_*`/`bsp_pwm_*`（统一经 `foc_output`）；
- ✅ 环路不直接调用 `hw_encoder_*`（统一经 `foc_feedback`）；
- ✅ `foc_control.c` 不判断 IDLE/LOCK，不写 2% 魔数（仅保留 `[IF]` 心跳与 overrun 日志的模式查询，Phase 5 移走）；
- ⬜ `foc/` 顶层只保留 `core/loops/math/proc/config/cli` 六个子目录——目录重排属 Phase 5，当前 `foc/` 顶层仍有 `foc_control.c`、`encoder_calib.c`、`foc_*_loop.c`、`cfg_*.h`；
- ⬜ 所有 `focmode` 命令、VOFA 通道、ISR 时间与基线一致——已逐条给出验收项（§12.8/§13.7/§14.7），完整 Phase 4 的台架项尚待实机；
- ⬜ 过流/堵转只有一个统一的安全停机入口——`Foc_Output_SafeStop()` 已就位并被 `foc_protect` 使用，但 `motor_stall.c` 仍未接入。

---

## 10. 建议的实施顺序（如果时间有限）

| 优先级 | 做什么 | 状态 | 预计收益 | 风险 |
| --- | --- | --- | --- | --- |
| 1 | Phase 1 纯搬迁 | 已完成 | `foc/` 目录语义立刻清晰 | 极低 |
| 2 | Phase 2 拆 CLI | 已完成 | `foc_mode.c` 从 1219 行降到 414 行 | 低 |
| 3 | Phase 3 拆 run/proc | **已完成 + 实机通过（2026-11-20）** | 去掉“mode 几”的语义混乱 | 中 |
| 4a | Phase 4-lite output/feedback | **已完成 + 实机通过（2026-11-20）** | 硬件访问点各收敛为一处 | 低 |
| 4b | 完整 Phase 4：执行器化 | **已完成，构建通过，实机待验证（§14.9）** | 职责边界彻底清楚 | 中-高（动 ISR） |
| 5 | Phase 5 params/telemetry + 目录重排 | 可选项 | 减少全局变量、`foc/` 顶层只有子目录 | 低～中 |

**当前建议：FOC 侧重构到此收尾，不再继续 Phase 5。** 剩余价值（VOFA/CLI 只读遥测快照、`foc/` 目录再分层）与投入不成正比；业务层还有 P0-1（绝对角路线决策）和 P0-2（`swing_axis`）两个关键项，`foc_feedback` / `Foc_Run_SetMode` / `Foc_Output_SafeStop` 已经够用。要动 Phase 5 的话，等 `swing_axis` 稳定之后再一次性做。

---

## 11. 下一步优化（Phase 3 详细计划）

> **本节是 Phase 3 执行前的计划稿，已完成**；实际实施清单与结果见 §12（清单）与 §12.10（执行结果）。Phase 4-lite 见 §13，完整 Phase 4 见 §14。保留本节仅作决策过程存档。

### 11.1 为什么先做 Phase 3

- 工程已经编译通过，Phase 1/2 的改动可以被独立验证。
- `focmode` 的命令解析和打印已经隔离在 `foc_cli.c`，内部枚举怎么改都不会影响用户命令。
- `foc_mode.c` 现在只有 414 行，是拆 run/proc 的最小改动面。
- Phase 4 的 `FocCmd_t`、输出策略收口都要建立在“运行模式只有 6 个”的前提上，先做 Phase 3 更顺。
- 这一步直接解决你最初提到的“mode 几”问题，收益最直观。

### 11.2 目标形态

```c
/* 持续运行模式：只有 6 个，没有空洞 */
typedef enum {
    FOC_RUN_IDLE = 0,
    FOC_RUN_LOCK,
    FOC_RUN_IF,
    FOC_RUN_CURRENT,
    FOC_RUN_SPEED,
    FOC_RUN_POSITION,
} foc_run_mode_t;

/* 一次性流程：不是运行模式 */
typedef enum {
    FOC_PROC_ENC_ALIGN = 0,
    FOC_PROC_SWEEP,
    FOC_PROC_REV,
    FOC_PROC_ALIGN7,
    FOC_PROC_OFFSET_NOPWM,
    FOC_PROC_OFFSET_PWM50,
} foc_proc_t;
```

文件落点：

```text
applications/motor/foc/core/
  foc_run.c/.h        // 原 foc_mode.c 的 6 个运行模式
  foc_control.c/.h    // 20 kHz 宿主
  foc_protect.c/.h
  foc_pwm.c/.h
applications/motor/foc/proc/
  foc_proc.c/.h       // Start/Poll/Abort/IsBusy
  encoder_calib.c/.h  // 保持现有状态机
applications/motor/foc/cli/
  foc_cli.c           // 旧数字映射、短命令、打印
```

### 11.3 实施步骤

1. **先搬核心，不改语义**
   - 新建 `foc/core/foc_run.c/.h`。
   - 把 `foc_mode.c` 中的 `s_mode`、`Foc_Mode_Init/SetMode/Step/SetOpenloop/Stop/SetLockDuty/PwmOwned`、电流指令访问接口搬过去。
   - 保留 `Foc_Mode_*` 作为薄封装，先让 `foc_cli.c` 和现有调用方编译通过。
   - 这一步只改文件位置和 include，不改任何分支。

2. **拆出 proc 接口**
   - 新建 `foc/proc/foc_proc.c/.h`，提供：

     ```c
     int  Foc_Proc_Start(foc_proc_t proc);
     void Foc_Proc_Poll1ms(void);
     void Foc_Proc_Abort(void);
     bool Foc_Proc_IsBusy(void);
     foc_proc_t Foc_Proc_GetActive(void);
     ```

   - 把 `FOC_MODE_CALIB/SWEEP/REV/ALIGN7/OFFCAL_NOPWM/OFFCAL_PWM50` 从 `foc_mode_t` 中删除，迁到 proc 层。
   - `encoder_calib.c` 的状态机保持原样，只由 proc 层调用。
   - `Foc_Run_GetMode()` 在 proc 执行期间应保持 `FOC_RUN_IDLE`，不再出现“运行模式 = 3”的假象。

3. **改 `foc_cli.c` 映射**
   - 旧数字只保留在 CLI 的映射表里：

     ```text
     0/1/2/4/9/10 → Foc_Run_SetMode(...)
     3/5/6/7/12/13 → Foc_Proc_Start(...)
     ```

   - `focmode stat` 同时打印 `run_mode` 和 `proc_active/proc_state`。
   - 帮助文本、错误提示、参数表格式保持不变。

4. **收尾 `foc_mode.c`**
   - 把剩余调用方改为 `foc_run.h` / `foc_proc.h`。
   - `foc_mode.c` 可以删除；如果担心兼容，保留一个只做转发的薄文件，行数控制在 100 行以内。
   - 更新 `SConscript` 与 `.cproject`，加入 `foc/core`、`foc/proc` 两个目录。
   - 每次搬完都执行一次 Clean 构建，避免旧 `.o` 混入。

5. **上板回归**
   - 先跑 `focmode 0/1/2/4/9/10`，确认运行模式行为不变。
   - 再跑 `focmode 3/5/6/7/12/13`，对比对齐/标定的打印与基线。
   - 最后跑 `fstat`、`t_abz`、`t_iadc`、`t_vofa`，确认观测接口不变。

### 11.4 验收标准

- `Foc_Run_GetMode()` 只返回 6 个值，不再出现 3/5/6/7/12/13。
- `focmode 3` 执行期间，`Foc_Run_GetMode()` 保持 IDLE，`Foc_Proc_GetActive()` 返回 `FOC_PROC_ENC_ALIGN`。
- `focmode 3` 的 `zero_hw/beta_hw/alpha_hw/moved/offset/dir` 打印与基线一致。
- `focmode 12/13` 的 `[CAL] OK`、回 IDLE 行为一致。
- `focmode 4/9/10` 的电流/速度/位置响应一致。
- `focmode stat`/`fstat` 字段不变。
- ISR 时间、占空比、VOFA 通道与基线一致。
- Clean 构建通过，`rtthread.elf` 正常生成。

### 11.5 Phase 3 之后

| 顺序 | 内容 | 说明 |
| --- | --- | --- |
| 1 | Phase 4：`foc_output` / `foc_feedback` | 模式层不再碰 `hw_tmr4_pwm`/`bsp_pwm`；环路不再碰 `hw_encoder`；`foc_control.c` 只执行 `FocCmd_t` |
| 2 | 保护统一 | `foc_protect` 与 `motor_stall` 收敛到一个 `foc_safe_stop()`；先做检测源汇总，再改停机路径 |
| 3 | 模式切换所有权 | 线程只发 `s_run_req`，ISR 在安全点完成 enter/exit；解决 `g_align_active` 类竞态 |
| 4 | Phase 5：参数/遥测 | `foc_params`/`foc_telemetry` 收拢全局变量，VOFA/CLI 只读快照 |
| 5 | 调试打印搬出控制宿主 | 把 `foc_control.c` 的 IF/overrun 打印移到 CLI/diag |

### 11.6 额外建议（本阶段顺手做）

- **构建系统防回归**：新增目录时同时更新 `SConscript` 和 `.cproject` 四个配置；移动 `.c` 后必须 `make clean2`，否则旧 `.o` 可能带旧工具链的符号（本次 `__locale_ctype_ptr` 就是这个问题）。
- **命名统一**：`Foc_Mode_*` 逐步过渡到 `Foc_Run_*`/`Foc_Proc_*`；兼容期保留旧函数名，避免一次性改所有调用方。
- **不要动 `encoder_calib` 的算法**：Phase 3 只搬编排，不改对齐数学。
- **不要在 Phase 3 引入函数表**：先保留 `switch` + 小函数，等 Phase 4 结构稳定后再评估表驱动。

### 11.7 不建议现在做

- 不要重写 Clarke/Park/SVPWM/PID。
- 不要上 C++/重型 OOP。
- 不要多实例化。
- 不要把 Phase 3 和业务层 `swing_axis` 接入混在一起；先把 FOC 内部结构稳定下来。

### 11.8 建议的提交拆分

如果按小步提交，建议拆成 3 个 commit：

1. `refactor(foc): move run-mode core to foc_run`（只搬文件，保留 `Foc_Mode_*` 兼容）。
2. `refactor(foc): split calibration procedures into foc_proc`（enum 拆分、CLI 映射、proc 接口）。
3. `chore(foc): remove foc_mode compatibility layer`（收尾、更新构建路径、清理 include）。

每个 commit 都执行一次 Clean 构建 + `focmode` 回归；任何一步回归失败，都可以只回退该 commit。

---

## 12. Phase 3 实施清单（2026-11-20，可执行）

本节把 §11 的计划落到「哪个文件、哪个函数、哪一行、怎么验收」。执行顺序即 §12.7 的 3 个 commit。

### 12.1 边界

**做**：把 `foc_mode_t`（12 值）拆成 `foc_run_mode_t`（6 值）+ `foc_proc_t`，一次性流程不再占用运行模式编号。

**不做**（Phase 4/5 或业务层）：
- 不改控制算法、不改环路参数、不改 VOFA 通道、不改 `focmode` 命令名与参数名。
- 不引入 `FocCmd_t.kind`、不删 2% 魔数、不抽 `foc_output`/`foc_feedback`。
- 不动 `encoder_calib.c` 的对齐数学。
- 不改 `swing_axes`/业务层；`foc/` 外的 5 个调用方只做「符号改名」，不做语义改造。

### 12.2 文件落点

| 动作 | 文件 | 说明 |
| --- | --- | --- |
| 新增 | `foc/core/foc_run.h` | `foc_run_mode_t`、`FocCmd_t`、6 模式 API、观测量 extern |
| 新增 | `foc/core/foc_run.c` | 原 `foc_mode.c` 的 Init/SetMode/GetMode/Name/Step20k/SetLockDuty/电流指令 |
| 新增 | `foc/proc/foc_proc.h` | `foc_proc_t`、Start/Task20k/Poll1ms/Abort/IsBusy/GetActive/PowerOnCal |
| 新增 | `foc/proc/foc_proc.c` | 3/5/6/7 编排 + 12/13 零偏校准（原 `foc_offset_cal_apply`） |
| 删除 | `foc/foc_mode.c`、`foc/foc_mode.h` | 内容已全部迁走，不保留转发层 |
| 修改 | `foc/foc_control.c` | include、`Foc_Mode_Step`→`Foc_Run_Step20k`、标定门控→`Foc_Proc_Task20k`、1ms→`Foc_Proc_Poll1ms` |
| 修改 | `foc/foc_protect.c` | `Foc_Mode_SetMode(FOC_MODE_IDLE)` → `Foc_Run_SetMode(FOC_RUN_IDLE)` |
| 修改 | `foc/cli/foc_cli.c` | 旧数字映射到 run/proc；`stat` 增加 proc 行 |
| 修改 | `diag/foc_vofa.c` | include 改名 |
| 修改 | `bsp/bsp_pwm.c` | `Foc_Mode_Init`→`Foc_Run_Init`、`Foc_Mode_PwmOwned`→`Foc_Run_PwmOwned` |
| 修改 | `boot/boot_table.c` | `Foc_Mode_PowerOnCal`→`Foc_Proc_PowerOnCal` |
| 修改 | `bg_modules.c` | `Foc_Mode_PwmOwned`→`Foc_Run_PwmOwned` |
| 修改 | `motor/drv/motor_drv_bldc_6step.c` | 6 处 `Foc_Mode_PwmOwned`→`Foc_Run_PwmOwned` |
| 修改 | `mode/sys_caps.c` | `foc_mode_t`→`foc_run_mode_t`，`FOC_MODE_IDLE/LOCK`→`FOC_RUN_IDLE/LOCK` |

**include 约定**：为免去给 40 多个 `subdir.mk` 逐个加 `-I`，新头文件一律用**相对路径**引用（与现有 `#include "../motor/foc/foc_mode.h"` 风格一致）：

| 引用方 | 写法 |
| --- | --- |
| `foc/foc_control.c`、`foc/foc_protect.c` | `"core/foc_run.h"`、`"proc/foc_proc.h"` |
| `foc/cli/foc_cli.c` | `"../core/foc_run.h"`、`"../proc/foc_proc.h"` |
| `diag/foc_vofa.c` | `"../foc/core/foc_run.h"` |
| `bsp/bsp_pwm.c` | `"../motor/foc/core/foc_run.h"` |
| `applications/bg_modules.c` | `"motor/foc/core/foc_run.h"` |
| `mode/sys_caps.c` | `"../motor/foc/core/foc_run.h"` |
| `motor/drv/motor_drv_bldc_6step.c` | `"../foc/core/foc_run.h"` |

`foc/core/`、`foc/proc/` 内部的 `.c` 可直接用裸名引用 `applications/motor/foc`、`applications/motor/drv`（这两个目录已在全局 `-I` 列表里）。

### 12.3 新 API

```c
/* foc_run.h：持续运行模式，只有 6 个，无空洞 */
typedef enum {
    FOC_RUN_IDLE = 0, FOC_RUN_LOCK, FOC_RUN_IF,
    FOC_RUN_CURRENT, FOC_RUN_SPEED, FOC_RUN_POSITION,
} foc_run_mode_t;

void           Foc_Run_Init(void);
void           Foc_Run_SetMode(foc_run_mode_t mode);
foc_run_mode_t Foc_Run_GetMode(void);
const char    *Foc_Run_Name(foc_run_mode_t mode);
int            Foc_Run_LegacyId(foc_run_mode_t mode);   /* 兼容：0/1/2/4/9/10，只用于日志/CLI */
void           Foc_Run_Step20k(float ts, FocCmd_t *out);
void           Foc_Run_SetOpenloop(float f_hz, float vq, float vd);
void           Foc_Run_StartOpenloop(void);
void           Foc_Run_Stop(void);
void           Foc_Run_SetLockDuty(float duty_pct);
float          Foc_Run_GetLockDuty(void);
float          Foc_Run_GetCurrentCmdId(void);
float          Foc_Run_GetCurrentCmdIq(void);
void           Foc_Run_SetCurrentCmd(float id, float iq);
void           Foc_Run_ReapplyCurrentCmd(void);
int            Foc_Run_PwmOwned(void);
void           Foc_Run_ReapplyStatic(void);   /* 按当前 run 模式重落一次静态输出（proc 结束收口用） */

/* foc_proc.h：一次性流程，不占用运行模式 */
typedef enum {
    FOC_PROC_NONE = 0,      /* 无流程（GetActive 用，文档 §5.1 的枚举补 0 项） */
    FOC_PROC_ENC_ALIGN,     /* 旧 focmode 3 */
    FOC_PROC_SWEEP,         /* 旧 focmode 5 */
    FOC_PROC_REV,           /* 旧 focmode 6 */
    FOC_PROC_ALIGN7,        /* 旧 focmode 7 */
    FOC_PROC_OFFSET_NOPWM,  /* 旧 focmode 12 */
    FOC_PROC_OFFSET_PWM50,  /* 旧 focmode 13 */
} foc_proc_t;

void        Foc_Proc_Start(foc_proc_t proc);
uint8_t     Foc_Proc_Task20k(void);     /* 1 = 本拍被流程占用，调用方直接 return */
void        Foc_Proc_Poll1ms(void);
void        Foc_Proc_Abort(void);
bool        Foc_Proc_IsBusy(void);
foc_proc_t  Foc_Proc_GetActive(void);
const char *Foc_Proc_Name(foc_proc_t proc);
int         Foc_Proc_LegacyId(foc_proc_t proc);   /* 兼容：3/5/6/7/12/13 */
void        Foc_Proc_PowerOnCal(void);            /* 上电零偏校准（原 Foc_Mode_PowerOnCal） */
```

依赖方向（无环）：

```
foc_cli / foc_control → foc_run ──→ foc_proc ──→ encoder_calib
                            │            │
                            └────────────┴─────→ hw_tmr4_pwm / bsp_pwm / bsp_adc_foc
```

`foc_run.c` 在 `Foc_Run_SetMode()` 开头调用 `Foc_Proc_Abort()`，等价于原 `Foc_Mode_SetMode()` 开头无条件 `EncoderCalib_Abort()`（含过流 trip 路径）。

### 12.4 数字映射（务必照抄）

| 用户命令 | 旧内部值 | 新调用 | 日志里的数字 |
| --- | ---: | --- | ---: |
| `focmode 0` / `idle` | 0 | `Foc_Run_SetMode(FOC_RUN_IDLE)` | 0 |
| `focmode 1 [duty]` / `lock` | 1 | `Foc_Run_SetMode(FOC_RUN_LOCK)` | 1 |
| `focmode 2 [f,v]` / `ifrun` | 2 | `Foc_Run_SetMode(FOC_RUN_IF)` | 2 |
| `focmode 3` / `falign` | 3 | `Foc_Proc_Start(FOC_PROC_ENC_ALIGN)` | 3 |
| `focmode 4 [id,iq]` / `current` | 4 | `Foc_Run_SetMode(FOC_RUN_CURRENT)` | 4 |
| `focmode 5` | 5 | `Foc_Proc_Start(FOC_PROC_SWEEP)` | 5 |
| `focmode 6` | 6 | `Foc_Proc_Start(FOC_PROC_REV)` | 6 |
| `focmode 7` | 7 | `Foc_Proc_Start(FOC_PROC_ALIGN7)` | 7 |
| `focmode 9 [rpm]` / `speed` | 9 | `Foc_Run_SetMode(FOC_RUN_SPEED)` | 9 |
| `focmode 10 [rev]` / `position` | 10 | `Foc_Run_SetMode(FOC_RUN_POSITION)` | 10 |
| `focmode 12` / `zcal` | 12 | `Foc_Proc_Start(FOC_PROC_OFFSET_NOPWM)` | 12 |
| `focmode 13` / `zcal50` | 13 | `Foc_Proc_Start(FOC_PROC_OFFSET_PWM50)` | 13 |

> **坑（§上一轮已识别）**：`foc_run_mode_t` 枚举是 0..5，与用户命令 4/9/10 不对应（CURRENT 枚举值 = 3，SPEED = 4，POSITION = 5）。因此**任何日志/`fstat` 都不允许直接 `(int)Foc_Run_GetMode()`**，必须走 `Foc_Run_LegacyId()`；`Foc_Run_LegacyId()` 是兼容期唯一的数字映射表，Phase 5 收拢遥测时再删。

### 12.5 行为保持清单（逐条对照旧代码）

| # | 旧行为 | 新实现 | 验证方式 |
| --- | --- | --- | --- |
| 1 | `SetMode` 开头无条件 `EncoderCalib_Abort()` | `Foc_Run_SetMode` 开头 `Foc_Proc_Abort()` | `focmode 3` 中途发 `focmode 0`，标定立即停 |
| 2 | `mode != SPEED && != POSITION` → `Foc_Speed_Stop()` | 同 | 代码对照 |
| 3 | `mode != POSITION` → `Foc_Position_Stop()` | 同 | 代码对照 |
| 4 | `mode != IF` → `s_theta = 0` | 同 | `focmode 9`→`focmode 2` 角度从 0 起 |
| 5 | IDLE：`all_low()` + `emergency_brake()` | 同 | `focmode 0` 后 PB4~PB9 全低 |
| 6 | LOCK：`release()` + `resume()` + `duty3(lock_duty)` | 同 | `focmode 1 5` 寄存器回读 |
| 7 | IF：`release()` + `resume()` + `s_theta=0` | 同 | `focmode 2 1 1.5` |
| 8 | CURRENT：`release()` + `resume()` + `CurrentLoop_Reset()` | 同 | `focmode 4 0 0.5` |
| 9 | SPEED：+ `Speed_ConfigCurrentLoop` + `Speed_Start` | 同 | `focmode 9 200` |
| 10 | POSITION：+ `Position_Reset` | 同 | `focmode 10 0` |
| 11 | 3/5/6/7：`release()` + `EncoderCalib_Start*()`，置 `s_mode`，打印 `-> mode N` | proc 层做同样动作，**运行模式保持 IDLE**，打印同一行 | `focmode 3` 打印逐字一致 |
| 12 | 12/13：先把模式置 IDLE、全低、brake，`mdelay` 后采样，结束保持 IDLE，**不打印 `-> mode N`** | proc 层同序；`[CAL12]`/`[CAL13]`/`[CAL OK]` 文本不变 | `focmode 12/13` 打印逐字一致 |
| 13 | 标定期间 ISR：`(g_align_active && EncoderCalib_IsBusy)` → 只跑 `EncoderCalib_Task20k()` 并返回 | `Foc_Proc_Task20k()` 内部同一判据，返回 1 让 `foc_control` 直接 return | `focmode 3` 全流程 |
| 14 | 1ms：`Protect_Task1ms()` → `EncoderCalib_Task1ms()` | `Protect_Task1ms()` → `Foc_Proc_Poll1ms()`（内部调 `EncoderCalib_Task1ms`） | 顺序不变 |
| 15 | 上电 `Foc_Mode_PowerOnCal()` 打 `[CAL] power-on zero offset cal` | `Foc_Proc_PowerOnCal()` 同文本 | 启动日志 |
| 16 | 标定结束 `ec_finish_zero_vector()` 先写 2%；旧版随后 ISR 因 `g_fm_mode` 由 IDLE→3 变化而触发一次 `resume+2%`，最终**停在 2% 零矢量** | **改变**：流程结束时 `Foc_Proc_Poll1ms()` 调 `Foc_Run_ReapplyStatic()`，按 run 模式（IDLE）显式写**全低 + 抱闸**，最终为全低 | 需实机确认（§12.8 第 7 条） |

**唯一有意改变的行为**：#16。标定结束后桥臂从「2% 零矢量驻留」变为「全低」，同时 `Foc_Run_GetMode()` 返回 IDLE（旧版返回 3）。这是本阶段的语义修正，必须上板确认后再进入 Phase 4。

> **为什么必须新增 `Foc_Run_ReapplyStatic()`**：标定期间 20kHz ISR 不调用 `Foc_Run_Step20k`，`g_fm_mode` 保持进入前的 IDLE，`s_last_static_mode` 也仍是 IDLE。标定结束回到 IDLE 时，`foc_control.c` 的静态输出分支判定"模式没变"直接跳过，于是 `ec_finish_zero_vector()` 写的 2% 会**永久驻留**——"run 模式 = IDLE"与"桥臂实际状态"不一致。补一次显式收口后，这条不变式才成立。

### 12.6 构建脚本

| 文件 | 改动 |
| --- | --- |
| `applications/motor/SConscript` | 加 `Glob('foc/core/*.c')`、`Glob('foc/proc/*.c')` 与对应 `CPPPATH` |
| `Debug/applications/motor/foc/subdir.mk` | 删除 `foc_mode.c/.o/.d` 三行 |
| `Debug/applications/motor/foc/core/subdir.mk` | 新增（以 `cli/subdir.mk` 为模板） |
| `Debug/applications/motor/foc/proc/subdir.mk` | 新增 |
| `Debug/makefile` | 加两条 `-include applications/motor/foc/core|proc/subdir.mk` |
| `Debug/sources.mk` | `SUBDIRS` 加 `applications/motor/foc/core`、`.../proc` |
| `.cproject` | 不改（新头文件走相对 include） |

### 12.7 实施顺序（3 个 commit）

1. **`refactor(foc): move run-mode core to foc_run`**
   新建 `foc/core/foc_run.{c,h}`，把 6 个运行模式搬过去；`foc_mode.h` 暂时保留为转发头，`foc_mode.c` 只留 proc 分支。此步结束仍应 Clean 编译通过。
2. **`refactor(foc): split one-shot procedures into foc_proc`**
   新建 `foc/proc/foc_proc.{c,h}`，搬 3/5/6/7/12/13；改 `foc_control.c`、`foc_cli.c`、`foc_protect.c`。
3. **`chore(foc): remove foc_mode compatibility layer`**
   删除 `foc_mode.c/.h`，改 5 个外部调用方，同步 4 处构建脚本，Clean 构建。

### 12.8 验收清单

**编译级（本地可完成）**
1. `make clean2 && make -j32 all` 通过，生成 `rtthread.elf`。
2. `nm`/map 中不再有 `Foc_Mode_*`、`foc_mode.o`；出现 `foc_run.o`、`foc_proc.o`。
3. 全工程 grep 无 `FOC_MODE_`、`foc_mode.h` 残留。
4. text/data/bss 与 388092/3244/35164 基本一致（±200B）。

**板级（需实机）**
5. 上电日志：`[FOC-RUN] ...`（原 `[FOC-MODE] power-on default` 行保持文本）、`[CAL] power-on zero offset cal`、`[CAL] OK`。
6. `focmode 0/1 5/2 1 1.5/4 0 0.5/9 200/10 0` 打印与基线一致，PWM 行为一致。
7. `focmode 3`：`[EC] BETA done / ALPHA done`、`zero_hw/beta_hw/alpha_hw/moved/offset/dir` 与基线一致；结束后 `focmode stat` 显示 `current=0`，`[FOC-PROC] active=...` 消失，PWM 全低。
8. `focmode 5/6/7`：与基线一致，完成后运行模式回 IDLE。
9. `focmode 12/13`：`[CAL12]`/`[CAL13]`/`[CAL OK]` 一致，结束保持 IDLE 全低。
10. `fstat`、`t_abz`、`t_iadc`、`t_vofa` 字段与基线一致（`mode=` 仍是 0/1/2/4/9/10）。
11. 过流/堵转触发后 `[FP] TRIP` 正常，且正在进行的标定被中止。

### 12.9 风险与回退

| 风险 | 影响 | 缓解 |
| --- | --- | --- |
| 枚举值平移后日志/CLI 打印错数字 | 现场误判电机状态 | 统一走 `Foc_Run_LegacyId()`；禁止 `(int)Foc_Run_GetMode()` |
| 标定终态从 2% 变全低 | 桥臂状态变化（更安全） | 上板单独确认；不接受则临时在 `ec_finish_zero_vector` 后补一次 2% |
| `Foc_Proc_Start` 从非 IDLE 进入时多打一行 `-> mode 0` | 日志多一行 | 少用「闭环直接切标定」；需要时改为静默切 IDLE |
| Eclipse 侧 `subdir.mk` 漏更新 | IDE 构建找不到 `foc_mode.c` | 三个构建文件同 commit 更新；`make clean2` 后重建 |
| `g_align_active` 仍定义在 `foc_control.c` | proc 层反向 include `foc_control.h` | 本阶段接受；Phase 4 移到 `encoder_calib` |

回退：三个 commit 独立，任一步回归失败只回退该 commit 并重新 Clean 构建。

### 12.10 执行结果（2026-11-20，已完成）

**结论：Phase 3 全部落地，Clean 全量构建通过，无 error / 无 warning。**

#### 12.10.1 交付物

| 类型 | 文件 | 行数 |
| --- | --- | ---: |
| 新增 | `applications/motor/foc/core/foc_run.h` | 92 |
| 新增 | `applications/motor/foc/core/foc_run.c` | 347 |
| 新增 | `applications/motor/foc/proc/foc_proc.h` | 62 |
| 新增 | `applications/motor/foc/proc/foc_proc.c` | 240 |
| 删除 | `applications/motor/foc/foc_mode.c`（原 414 行）、`foc_mode.h`（原 93 行） | - |
| 修改 | `foc/foc_control.c`（411→410 行） | 410 |
| 修改 | `foc/cli/foc_cli.c`（843→856 行，+13 行为 proc 映射/状态行） | 856 |
| 修改 | `foc/foc_protect.c`、`diag/foc_vofa.c`、`bsp/bsp_pwm.c`、`boot/boot_table.c`、`bg_modules.c`、`motor/drv/motor_drv_bldc_6step.c`、`mode/sys_caps.c` | - |
| 构建 | `applications/motor/SConscript`、`Debug/applications/motor/foc/subdir.mk`（删 foc_mode 三行）、新增 `Debug/.../foc/core/subdir.mk`、`Debug/.../foc/proc/subdir.mk`、`Debug/makefile`（+2 条 include）、`Debug/sources.mk`（+2 条 SUBDIRS） | - |

`.cproject` 未改：整个工程只有一个 `sourceEntries` 根条目，新子目录会被 Eclipse 自动纳入；新头文件全部用相对路径 include，也不需要新增 `-I`。

#### 12.10.2 构建证据

```
Clean 全量：删除 Debug 下全部 *.o/*.d/*.elf/*.hex/*.map/*.siz 后重建
编译单元：238 个 .c → 238 个 .o / 238 个 .d（全部新生成）
日志    ：error 0 / warning 0
尺寸    ：text 386592  data 3548  bss 34452  dec 424592
产物    ：Debug/rtthread.elf、Debug/rtthread.hex
```

与 Phase 2 基线（text 388092 / data 3244 / bss 35164）相比：text **-1500**、data **+304**、bss **-712**。差异来自 `foc_mode.c` 拆分后函数去重与 static 变量重新布局，没有新增功能代码，属预期范围。

符号核验：`nm` / `rtthread.map` 中 `Foc_Mode_`、`FOC_MODE_` 命中 **0**；新增符号齐全（`Foc_Run_Init/SetMode/GetMode/Step20k/SetLockDuty/GetLockDuty/PwmOwned/LegacyId/ReapplyStatic`、`Foc_Proc_Start/Task20k/Poll1ms/Abort/IsBusy/GetActive/Name/LegacyId/PowerOnCal`）。`applications/` 内 `Foc_Mode_|FOC_MODE_|foc_mode.h|foc_mode_t` 仅剩 2 处注释（`foc_run.c:138`、`foc_proc.c:143` 的"等价于原 …"说明），无代码引用。

#### 12.10.3 环境结论：本机不能用 `make -j`

本次构建暴露一个与代码无关的环境问题：**GNU Make 3.81（RT-Thread Studio 自带）在本机 `-j` 并行模式下会自旋卡死**。

- `make -j8` / `-j4`：make 进程 100% CPU 自旋，`.o` 数停在个位数，**无任何 gcc 子进程**；用 `cmd /c ... 1>log 2>&1` 做真正的文件句柄重定向后依旧卡死，可排除"stdout 管道写满"。
- 串行 `make all`：**98 秒**跑完 238 个编译单元，一次通过。

后续本机构建一律用串行 `make all`（约 1.5 分钟，可接受）；RT-Thread Studio 内部构建走的是它自己的 builder，不受影响。

**另一个环境坑**：`make all` 的最后一步 `rtthread.hex` 规则里有 `$(MAKE) --no-print-directory post-build`，在 MSYS sh 下 `$(MAKE)` 会被展开成含盘符的绝对路径并被改写成 `/f/.../Debug/"D:/..."`，报 `Error 127: No such file or directory`。此时 **elf/hex 其实都已生成**，只是递归 make 失败导致退出码非 0。可靠做法：用 `make rtthread.elf rtthread.siz`（避开该规则），或用 `arm-none-eabi-objcopy -O ihex rtthread.elf rtthread.hex` 手工生成。

#### 12.10.4 仍未做（需实机）

§12.8 的**板级 5~11 条**全部待做，其中两条是本次唯一有意的行为变更，必须优先确认：

1. `focmode 3` 结束后：运行模式应为 `IDLE`（`focmode stat` 显示 `current=0`，且不再有 `[FOC-PROC] active=...` 行），PWM 应为**全低**（旧版是 2% 零矢量驻留；本版由 `Foc_Run_ReapplyStatic()` 收口）。
2. `focmode 3/5/6/7` 的 `-> mode N` 打印行、`focmode 12/13` 的 `[CAL12]/[CAL13]/[CAL OK]` 文本应与基线逐字一致；`focmode stat` 的 `mode=` 字段数字仍是 0/1/2/4/9/10。

另需确认的三条既有行为：过流/堵转 trip 时正在进行的标定被中止；ISR 时间与基线一致；`focmode 4/9/10` 闭环响应不变。

---

## 13. Phase 4-lite 实施清单（2026-11-20，可执行）

### 13.1 定位与边界

Phase 4 全量（`FocCmd_t.kind` + 删 2% 魔数 + 删 `s_last_static_mode`）会改 20 kHz 每拍输出策略，只能靠示波器/DWT/寄存器回读验收，是"改错了可能烧桥臂"的唯一路径。业务侧当前最缺的是**唯一停机入口**和**`swing_axis` 的取数接口**，这两项不需要动输出策略就能拿到。

因此拆成两步：

| 方案 | 内容 | 风险 | 决策 |
| --- | --- | --- | --- |
| 全量 Phase 4 | `foc_output` + `foc_feedback` + `FocCmd_t.kind` + 删 2%/`s_last_static_mode` | 中-高 | **推迟**到 P0-2/P0-3 落地后（那时有真实转动行为可对照） |
| **Phase 4-lite（本节）** | `foc_output` + `foc_feedback`，**保留** 2% 和 `s_last_static_mode`、保留 `Foc_Run_ReapplyStatic()` | 低 | **本次执行** |
| 跳过 | 直接做 P0-1/P0-2 | - | P0-8/P0-10 会返工 |

**边界**：只做"硬件访问点收敛"，不做"输出策略重写"。所有数值（2%、50%、`lock_duty`）、所有分支、所有返回值语义保持逐字不变。

### 13.2 改动面（实测，Phase 3 之后）

| 类别 | 数量 | 分布 |
| --- | ---: | --- |
| 运行期写桥臂的调用点 | **50** | `foc_run.c` 22、`foc_proc.c` 11、`encoder_calib.c` 9、`foc_control.c` 5、`foc_protect.c` 1、`foc_pwm.c` 2 |
| 环路/宿主读编码器 | **12** | `foc_control.c` 6、`foc_current_loop.c` 3、`foc_position_loop.c` 2、`foc_speed_loop.c` 1 |
| `encoder_calib.c` 读编码器 | 14 | 标定需要原始计数，**本阶段不迁**（允许 calib 直连 drv） |

不计入的两个文件：
- `foc_pwm.c`：初始化期配置 + `start_output`，属配置层，不是运行期输出策略；其 2 处 `all_low/resume` 保留。
- `bsp/bsp_pwm.c`：`bsp_pwm_emergency_brake/release` 的定义处，保留不动，由 `foc_output` 包装。

### 13.3 新增模块 API

```c
/* foc/core/foc_output.h —— 运行期桥臂输出的唯一入口 */
void Foc_Output_Init(void);                                  /* 上电默认全低 + 抱闸 */
int  Foc_Output_AllLow(void);                                /* 全低，不抱闸；0=OK */
int  Foc_Output_Resume(void);                                /* 恢复 PWM 输出；0=OK */
int  Foc_Output_Duty3(float du, float dv, float dw);          /* 三相中心占空比 %；0=OK */
void Foc_Output_Brake(void);                                 /* 抱闸（EMB 封锁，当前宏为 0 时等价空操作） */
void Foc_Output_Release(void);                               /* 解除抱闸（EMB 释放） */
void Foc_Output_SafeStop(void);                              /* 唯一安全停机入口 = AllLow + Brake */

/* foc/core/foc_feedback.h —— 运行期反馈的唯一入口（签名与 hw_encoder 一一对应） */
int32_t  Foc_Feedback_GetCount(void);
float    Foc_Feedback_GetMechAngleRad(void);
float    Foc_Feedback_GetElecAngleRad(uint8_t pole_pairs);
float    Foc_Feedback_GetSpeedRpm(void);
int8_t   Foc_Feedback_GetDir(void);                          /* 电角度方向 ±1（原 g_enc_elec_dir） */
float    Foc_Feedback_GetAngleDeg(void);
```

`foc_output.c` 是唯一 include `hw_tmr4_pwm.h` / `bsp_pwm.h` 的 FOC 运行期模块；`foc_feedback.c` 是唯一 include `hw_encoder.h` 的运行期模块。两者都禁止包含模式层头文件（`foc_run.h`/`foc_proc.h`），依赖方向单向：`mode/proc/control → output/feedback → drv`。

### 13.4 文件落点与并行写作用域

两个作业**文件不重叠**，可并行：

| 文件 | 执行方 | 改动 |
| --- | --- | --- |
| `foc/core/foc_output.c/.h` | A（子智能体） | 新增 |
| `foc/core/foc_run.c` | A | 22 处直接调用 → `foc_output_*` |
| `foc/proc/foc_proc.c` | A | 11 处 |
| `foc/encoder_calib.c` | A | 9 处（**只改名，不碰对齐数学**） |
| `foc/foc_protect.c` | A | 1 处 `set_all_low` → `Foc_Output_AllLow()` |
| `foc/core/foc_feedback.c/.h` | B（lead） | 新增 |
| `foc/foc_control.c` | B | 5 处写 → `foc_output_*`；6 处读 → `foc_feedback_*` |
| `foc/foc_current_loop.c` | B | 3 处读 → `foc_feedback_*` |
| `foc/foc_position_loop.c` | B | 2 处读（含 `g_enc_elec_dir`）→ `foc_feedback_*` |
| `foc/foc_speed_loop.c` | B | 1 处读（`g_enc_elec_dir`）→ `Foc_Feedback_GetDir()` |
| `foc/foc_pwm.c`、`bsp/bsp_pwm.c` | 不动 | 见 §13.2 |

构建由 B 统一执行：A 只做 `-fsyntax-only` 语法自检，**不跑 make**（避免两个作业同时写 `Debug/`）。

### 13.5 行为保持清单（只移动，不改逻辑）

| # | 约束 |
| --- | --- |
| 1 | `TMR4_PWM_OK == 0`，返回码映射 `err != TMR4_PWM_OK` → `Foc_Output_*() != 0`，失败分支的打印与收尾顺序原样保留 |
| 2 | 数值常量不变：`2.0f`（零矢量）、`50.0f`（标定 PWM50）、`s_lock_duty`、`100.0f`（SVPWM 转百分比） |
| 3 | 调用顺序不变（例如 LOCK 分支仍是 release → resume → duty3，失败时 all_low → brake） |
| 4 | `(void)` 忽略返回值的调用点保持忽略；有判错的调用点保持判错 |
| 5 | 返回值类型：`foc_output` 返回 `int`，`0` 成功；不改变任何 `tmr4_pwm_err_t` 取值语义 |
| 6 | 打印文本、`rt_kprintf` 行数与内容不变（`[FOC-MODE]`、`[CAL12]`、`[CAL13]` 等） |
| 7 | 只删除**确实不再使用**的 include；文件里还有其他 `hw_tmr4_pwm_*` 用途时保留（如 `foc_cli.c` 的诊断读取、`foc_control.c` 的 `get_state`） |
| 8 | `encoder_calib.c` 的 `s_beta_du/s_alpha_du/ctx.duty_*` 计算表达式一字不改，只把最外层写函数换成 `Foc_Output_Duty3` |
| 9 | 不新增/不删除任何 `static` 变量；不调整函数顺序 |
| 10 | `Foc_Run_ReapplyStatic()` 保留，内部改调 `Foc_Output_*` |

### 13.6 明确不做（留给完整 Phase 4）

- 不引入 `FocCmd_t.kind`，`FocCmd_t` 结构不变。
- 不删 `s_last_static_mode`、不删 `foc_control.c` 的 else 分支、不删 2% 魔数。
- 不改 `encoder_calib.c` 的对齐/扫角/测圈数学，不改 `EncoderCalib_*` 状态机。
- 不改 `foc_pwm.c` 的配置流程。
- 不动业务层（`swing/`、`sequence/`、`app_main.c` 等）。

### 13.7 构建与验收

**编译级（本地必做）**
1. Clean 全量构建通过，0 error / 0 warning；尺寸与当前 `386592/3548/34452` 基本一致（±64B 内应视为等价，代码只是换了调用名）。
2. `nm` / map 中不再有 `foc_run.o`、`foc_proc.o`、`encoder_calib.o` 对 `hw_tmr4_pwm_set_*` / `bsp_pwm_emergency_*` 的**未定义引用**（即它们已不再直接调用 drv）。
3. 全工程 grep：运行期写桥臂的调用只出现在 `foc_output.c`（+ `foc_pwm.c` 的配置期 2 处）；运行期读编码器只出现在 `foc_feedback.c`。

**台架（轻量，与 Phase 3 回归同一套命令）**
4. `focmode 0/1 5/2 1 1.5/4 0 0.5/9 200/10 0` 打印与行为与 Phase 3 后一致。
5. `focmode 3` 的 `zero_hw/beta_hw/alpha_hw/moved/offset/dir` 与 Phase 3 后一致；结束回全低、`current=0`。
6. `focmode 12/13` 的 `[CAL12]/[CAL13]/[CAL OK]` 一致。
7. `fstat`、`t_abz`、`t_iadc`、`t_vofa` 字段一致。
8. ISR 时间（`g_fc_isr_dt_max_us` / `g_dwt_isr_max_cyc`）与 Phase 3 后一致——本阶段只是多一层函数调用，允许极小增加但不应超过 +1%。

### 13.8 风险与回退

| 风险 | 缓解 |
| --- | --- |
| 批量改名时漏掉一处判错分支 | A/B 各自 grep 自己负责文件的 `hw_tmr4_pwm_set_*`/`bsp_pwm_emergency_*` 计数归零后再交付 |
| `encoder_calib.c` 改错矢量计算 | 只替换最外层写函数；改完逐行 diff 确认表达式未变 |
| 两个作业同时写 `Debug/` 造成构建目录损坏 | 约定只有 B 跑 make；A 只做 `-fsyntax-only` |
| 新增一层调用导致 ISR 变慢 | 验收第 8 条；`foc_output_*` 允许 `#pragma GCC optimize("O2")`（同现有热路径文件） |

回退：本阶段纯机械替换，`foc_output`/`foc_feedback` 是新增文件，任一处回归失败可只回退对应文件并恢复原 include。

### 13.9 与完整 Phase 4 的衔接

Phase 4-lite 完成后，完整 Phase 4 只剩三件事：

1. `FocCmd_t` 增加 `kind`（`NONE/VECTOR/DUTY3/ALL_LOW`），模式层每拍返回完整指令；
2. 删除 `foc_control.c` 的 `s_last_static_mode`/`s_if_resumed`/2% 魔数，改为按 `kind` 执行；
3. `Foc_Run_ReapplyStatic()` 与 `foc_proc` 的收口逻辑随之删除（每拍都会输出，不再需要）。

届时 `foc_output` / `foc_feedback` 已经是唯一写/读点，那三件事的改动面只剩 `foc_run.c` + `foc_control.c` 两个文件，可以单独上板验证。

### 13.10 执行结果（2026-11-20，已完成）

**结论：Phase 4-lite 全部落地，Clean 全量构建通过，行为零变化。**

#### 13.10.1 交付物

| 类型 | 文件 | 行数 |
| --- | --- | ---: |
| 新增 | `foc/core/foc_output.h` | 49 |
| 新增 | `foc/core/foc_output.c` | 55 |
| 新增 | `foc/core/foc_feedback.h` | 51 |
| 新增 | `foc/core/foc_feedback.c` | 57 |
| 修改 | `foc/core/foc_run.c`（347→345：include 3→2、`Foc_Output_Init()` 合并） | 345 |
| 修改 | `foc/proc/foc_proc.c`（240→239） | 239 |
| 修改 | `foc/encoder_calib.c`（行数不变，9 处写函数改名） | 976 |
| 修改 | `foc/foc_protect.c`（行数不变） | 308 |
| 修改 | `foc/foc_control.c`（5 写 + 6 读） | 411 |
| 修改 | `foc/foc_current_loop.c`（3 读） | 534 |
| 修改 | `foc/foc_position_loop.c`（2 读） | 276 |
| 修改 | `foc/foc_speed_loop.c`（1 读） | 345 |
| 构建 | `Debug/applications/motor/foc/core/subdir.mk`：源文件 1 → 3（新增 `foc_output.c`、`foc_feedback.c`） | - |

`SConscript` 的 `Glob('foc/core/*.c')` 已覆盖新文件，`Debug/makefile`/`sources.mk` 无需再改；`.cproject` 仍不改（相对 include）。

#### 13.10.2 替换计数（旧直接调用 → 0）

| 文件 | 替换前 | 替换后 |
| --- | ---: | ---: |
| `foc_run.c` | 22 | 0 |
| `foc_proc.c` | 11 | 0 |
| `encoder_calib.c` | 9 | 0 |
| `foc_control.c` | 5 | 0 |
| `foc_protect.c` | 1 | 0 |
| **合计** | **48** | **0** |

另有 `foc_pwm.c` 的 2 处（配置期 `all_low`/`resume`）按 §13.2 设计保留；`bsp/bsp_pwm.c` 是 `emergency_*` 定义处，未动。

#### 13.10.3 构建证据

```
Clean 全量：删除 Debug 下全部 *.o/*.d/*.elf/*.hex/*.map/*.siz 后重建
编译单元：240 个 .o（238 + foc_output + foc_feedback），240 个 .d
串行 make all：90 秒，error 0 / warning 0
尺寸    ：text 386696  data 3548  bss 34452  dec 424696
产物    ：Debug/rtthread.elf（2026-11-20 11:13:03）、rtthread.hex
```

对比 Phase 3 后（`386592 / 3548 / 34452`）：**text +104 B，data/bss 不变**。增量来自两个新模块的 11 个函数与一层调用转发，属预期。

> 日志里唯一的 `error` 是已知的 `rtthread.hex` 递归 `$(MAKE) post-build` 环境坑（§12.10.3），elf/hex 均已正常生成；用 `make rtthread.elf rtthread.siz` 复核退出码 0。

#### 13.10.4 符号与残留核查

- 已导出：`Foc_Output_{Init,AllLow,Resume,Duty3,Brake,Release}`、`Foc_Feedback_{GetCount,GetMechAngleRad,GetElecAngleRad,GetSpeedRpm,GetDir}`。
- `Foc_Output_SafeStop()`、`Foc_Feedback_GetAngleDeg()`、`Foc_Feedback_GetRevCount()` 目前无调用者，被 `--gc-sections` 裁掉——符合预期：安全停机入口要等 P0-8/P0-10 接线后才生效，取角度/圈数接口等 `swing_axis`（P0-2）接入。
- 运行期直写桥臂只剩 `foc_output.c`；运行期直读编码器只剩 `foc_feedback.c`；`encoder_calib.c` 的 14 处直读按设计保留（标定域）。
- 行为保持复核：`foc_run.c` 的 mode 0/1 失败分支（`err != 0` → 打印 + 收尾）与旧版逐条对应；`2.0f`（零矢量）、`50.0f`（标定 PWM50）、`s_lock_duty`、`* 100.0f` 全部原值；打印文本未变。
- 已知的语义细节：失败日志里的 `(int)err` 现在打印归一化值 `-1`，不再区分 `TMR4_PWM_ERR_*`（1~4）。这是 0/-1 接口的必然结果，打印格式未改；若现场需要区分具体错误码，后续可让 `Foc_Output_*` 返回原始 `tmr4_pwm_err_t` 而不归一化。

#### 13.10.5 实机验证（2026-11-20 已通过）

用户实机确认 **Phase 4-lite OK**，以下 5 条全部通过：

1. `focmode 0/1 5/2 1 1.5/4 0 0.5/9 200/10 0` 与 Phase 3 后行为一致。 ✅
2. `focmode 3` 的 `zero_hw/beta_hw/alpha_hw/moved/offset/dir` 一致；结束回全低、`current=0`。 ✅
3. `focmode 12/13` 的 `[CAL12]/[CAL13]/[CAL OK]` 一致。 ✅
4. `fstat`、`t_abz`、`t_iadc`、`t_vofa` 字段一致。 ✅
5. ISR 时间（`g_fc_isr_dt_max_us`、`g_dwt_isr_max_cyc`）不超 Phase 3 基线 +1%。 ✅

至此 Phase 4-lite 关闭，可进入完整 Phase 4（§14）。

---

## 14. 完整 Phase 4 实施清单（2026-11-20，可执行）

### 14.1 目标

Phase 4-lite 已经把所有硬件访问收敛到 `foc_output` / `foc_feedback`。本节只做剩下三件事：

1. `FocCmd_t` 增加 `kind`，模式层每拍返回**完整**指令；
2. `foc_control.c` 变成纯执行器：不判断模式、不写 2% 魔数、删 `s_last_static_mode`/`s_if_resumed`/`FC_LAST_MODE_IN_IF`；
3. 删除 `Foc_Run_ReapplyStatic()` 及 `Foc_Proc_Poll1ms()` 里的收口调用。

### 14.2 关键约束（实测依据，决定了实现形态）

**`set_all_low()` 与 `resume_output()` 都不能每拍调用。**

| 函数 | 实际开销 |
| --- | --- |
| `hw_tmr4_pwm_set_all_low()` | 每相：关 H/L 两个 OC 通道、设 2 个 invalid 极性、`TMR4_PWM_Init()` 重配为 THROUGH。**共 3 次 `TMR4_PWM_Init`** |
| `hw_tmr4_pwm_resume_output()` | 每相：`TMR4_PWM_StructInit` + `TMR4_PWM_Init` + 设极性 + 写死区；三次之后 `TMR4_Start` |

因此"模式层每拍返回完整指令"必须由**执行器按当前物理输出状态去重**，而不是每拍无条件写寄存器：

```c
/* foc_control.c 内部：桥臂当前处于什么状态（NONE = 未知，需要重设） */
typedef enum { FC_OUT_UNKNOWN = 0, FC_OUT_ALL_LOW, FC_OUT_DRIVING } fc_out_state_t;
static volatile fc_out_state_t s_out_state = FC_OUT_ALL_LOW;   /* Foc_Run_Init 已置全低+抱闸 */
```

这不是 `s_last_static_mode` 换名字：它描述的是**桥臂物理状态**（与运行模式无关），任何在执行器之外写桥臂的模块都必须负责失效它。

**标定流程会在执行器之外写桥臂**（`encoder_calib.c` 的 9 处 duty3）。因此：

```c
if (Foc_Proc_Task20k() != 0u) {
    s_out_state = FC_OUT_UNKNOWN;   /* 标定自己写桥臂；结束时强制下一拍重新落静态输出 */
    return;
}
```

这一句同时替代了 Phase 4-lite 的 `Foc_Run_ReapplyStatic()`：标定结束后的 2%（`ec_finish_zero_vector`）会在下一拍被 `ALL_LOW` 覆盖，**比原来的 1 ms 收口更快**。

**其余「执行器之外写桥臂」的路径（Phase 4 独立审计补充）**：

- `foc_proc.c` 的 **12/13 阻塞流程**会 `Resume + Duty3(50%)`，但它们的 `s_busy` 从不置 1、`Foc_Proc_Task20k()` 返回 0，所以状态不会被自动失效。当前安全依赖三点：入口先强制回 IDLE、IDLE 拍在 `state == ALL_LOW` 时去重不写、以及 trip 改为 `Foc_Output_SafeStop()` 兜底（§14.9 审计修复）。该窗口内的状态标记与物理状态不一致，属于**已记录的不变式例外**。
- **旧六步遗留模块**：`bsp_dev_hall_uvw.c::pwm_bridge_switch()`（111 处 `TMR4_OC_SetCompareValue`）与 `bsp_dev_motor.c::dev_motor_init/stop()` 会直接写桥臂，且原先**没有** `Foc_Run_PwmOwned()` 门控。它们当前无调用者、被 `--gc-sections` 剔除（`nm` 无符号），但属于源码级隐患，已在 §14.9 补门控。
- `foc_pwm.c` 的启动期调用与上电静态初值 `s_out_state = ALL_LOW` 一致，无需失效。

### 14.3 `FocCmd_t` 新定义

`foc_out_kind_t` 放在 `foc_output.h`（输出域），`foc_run.h` 包含它：

```c
typedef enum {
    FOC_OUT_NONE = 0,   /* 本拍不改变输出（执行器跳过） */
    FOC_OUT_VECTOR,     /* 用 vd/vq/theta_e 算 SVPWM */
    FOC_OUT_DUTY3,      /* 直接写三相中心占空比（LOCK/静态矢量） */
    FOC_OUT_ALL_LOW,    /* 全低 + 抱闸（安全停机态） */
} foc_out_kind_t;

typedef struct {
    foc_out_kind_t kind;
    float vd, vq, theta_e;          /* kind = VECTOR */
    float duty_u, duty_v, duty_w;   /* kind = DUTY3 */
} FocCmd_t;
```

`enable` 字段删除（`kind != FOC_OUT_ALL_LOW/NONE` 即"在驱动"）。`FOC_OUT_NONE` 本阶段**不产生**（模式层永远返回明确指令），保留仅为将来扩展；执行器遇到直接 break。

### 14.4 模式层每拍的 kind 映射

| 运行模式 | `kind` | 数据 |
| --- | --- | --- |
| `FOC_RUN_IDLE` | `ALL_LOW` | — |
| `FOC_RUN_LOCK` | `DUTY3` | `s_lock_duty × 3` |
| `FOC_RUN_IF` | `VECTOR` | `s_vd/s_vq/s_theta`（角度自增） |
| `FOC_RUN_CURRENT` | `VECTOR` | 电流环输出 |
| `FOC_RUN_SPEED` | `VECTOR` | 速度环 → 电流环 |
| `FOC_RUN_POSITION` | `VECTOR` | 位置环 → 速度环 → 电流环 |

### 14.5 执行器语义（`foc_control.c`）

```c
Foc_Run_Step20k(FC_ISR_TS_S, &cmd);

switch (cmd.kind) {
case FOC_OUT_ALL_LOW:
    if (s_out_state != FC_OUT_ALL_LOW) {
        Foc_Output_SafeStop();          /* 全低 + 抱闸，唯一安全停机入口 */
        s_out_state = FC_OUT_ALL_LOW;
    }
    break;

case FOC_OUT_DUTY3:
case FOC_OUT_VECTOR:
    if (s_out_state != FC_OUT_DRIVING) {
        Foc_Output_Release();
        (void)Foc_Output_Resume();
        s_out_state = FC_OUT_DRIVING;
    }
    if (cmd.kind == FOC_OUT_VECTOR) {
        InvPark + SVPWM → duty_u/v/w;
    }
    (void)Foc_Output_Duty3(cmd.duty_u, cmd.duty_v, cmd.duty_w);
    break;

case FOC_OUT_NONE:
default:
    break;
}
```

同时**从 `Foc_Run_SetMode()` 删除全部输出动作**（all_low / brake / release / resume / duty3 都改由执行器在下一拍完成，最大延迟 50 µs）：

- 模式切换由线程发起、输出由 ISR 落实 → 顺带消除"线程直接写 PWM"的竞态（§3.9）。
- `Foc_Protect_Trip()` 保留现有的立即 `Foc_Output_AllLow()`，过流停机延迟不变。
- `Foc_Run_SetLockDuty()` 保留打印、删除立即写 duty3（下一拍生效）。

### 14.6 行为变化清单（逐条说明为什么可接受）

| # | 变化 | 影响 | 判定 |
| --- | --- | --- | --- |
| 1 | IDLE 的 all_low+brake 从线程侧改为 ISR 下一拍 | ≤50 µs 延迟；过流 trip 仍有立即 AllLow | 可接受 |
| 2 | LOCK 的占空比从"进入时写一次"改为每拍写 | 寄存器写更频繁，但值不变；`focmode 1 5` 的 cmpH/cmpL 不受影响 | 可接受（需实测 ISR 时间） |
| 3 | 标定结束的 2% 被下一拍 ALL_LOW 覆盖 | 比原来 1 ms 收口更快，语义一致 | 改进 |
| 4 | 删除 `foc_control.c` 的 2% 分支 | 经分析该分支在新结构下**是死代码**：只有向量模式会返回 enable=true，而 IDLE/LOCK 已由 kind 表达 | 无行为变化 |
| 5 | 删除 `Foc_Run_SetMode()` 里的 `mode 0/1 failed` 错误打印 | 只在 `hw_tmr4_pwm_*` 失败时出现，正常运行永不触发 | 可接受 |
| 6 | `Foc_Run_ReapplyStatic()` 删除 | 由 `s_out_state = UNKNOWN` 取代 | 无行为变化 |
| 7 | 驱动模式之间互切不再重复 `release+resume` | 旧 `SetMode` 每次进驱动模式都重初始化 PWM；新执行器只在 `ALL_LOW → DRIVING` 做一次 | 减少瞬态，功能等价（审计确认） |
| 8 | 输出失败兜底移除 | 旧 `SetMode` 失败会 `all_low+brake` 并打印；新执行器 `(void)` 忽略返回值并把状态置 `DRIVING` | 失败仅在 `s_bConfigured == false` 时可能，上电后不可达（审计 L2，接受并记录） |
| 9 | `focmode 12/13` 期间 `s_out_state` 标为 ALL_LOW 而桥臂在 50% | proc 独占桥臂；trip 已改为 `SafeStop()` 兜底 | 审计 M1，已兜底；EMB 打开前必须复查 |
| 10 | 日志文本：`[FOC-PROC] unsupported run mode` → `[FOC-RUN] ...`；`hint: run 'falign'` 为 Phase 3 既有打印 | 仅文本 | 无功能影响 |

### 14.7 验收

**编译级**
1. Clean 全量构建通过，0 error / 0 warning。
2. `nm` 中不再有 `Foc_Run_ReapplyStatic`；`Foc_Output_SafeStop` 有调用者（被 gc-sections 保留）。
3. grep 确认 `foc_control.c` 不再出现 `g_fm_mode`、`s_last_static_mode`、`s_if_resumed`、`FC_LAST_MODE_IN_IF`、`2.0f` 零矢量魔数；仅保留 `[IF]` 心跳与 overrun 日志里的模式查询（Phase 5 移走）。

**台架**
4. `focmode 0`：PB4~PB9 全低，进入后 ≤1 ms 内完成（示波器确认无中间态）。
5. `focmode 1 5`：cmL 与基线一致（`2499 - floor(2499×0.05)`），`REG` 回读一致。
6. `focmode 2 1 1.5`：三相正弦、`[IF]` 心跳打印正常。
7. `focmode 4/9/10`：闭环响应与 Phase 4-lite 一致。
8. `focmode 3`：标定打印一致；结束后 `current=0`、PWM 全低（下一拍覆盖 2%）。
9. `focmode 0 → 1 → 0` 反复切换：每次切换无毛刺、无直通（示波器看上下管）。
10. ISR 时间：IDLE/VECTOR 不增加；LOCK 因每拍写 duty3 略有增加，仍应 ≤ 基线 +5%。

### 14.8 风险与回退

| 风险 | 缓解 |
| --- | --- |
| 执行器状态与物理桥臂状态失配（例如漏掉某个写桥臂的模块） | 目前只有 `encoder_calib.c` 在 ISR 外写桥臂，已由 `Foc_Proc_Task20k()` 分支失效；新增写点必须同样失效 |
| 模式切换延迟导致示波器看到 50µs 中间态 | 属于预期；如需零延迟，可在 `Foc_Run_SetMode()` 里保留一次同步写（但会重新引入线程写 PWM） |
| LOCK 每拍写 duty3 拉高 ISR | 验收第 10 条；必要时在 LOCK 内按占空比值去重 |
| `SafeStop` 的 brake 语义（`BSP_PWM_EMB_ENABLE=0` 时是空操作） | 与现状一致；将来打开 EMB 封锁时行为自动生效 |

回退：改动集中在 `foc_run.h/.c` + `foc_control.c` 三个文件，`git` 不可用时可保留改前副本，单体回退即可。

### 14.9 执行结果（2026-11-20，已完成）

**结论：完整 Phase 4 落地，Clean 构建通过；独立子智能体审计发现 1 个高危隐患 + 1 个不变式例外（均非本次引入的功能回归），已修复/记录。**

#### 14.9.1 交付物

| 文件 | 行数变化 | 改动 |
| --- | --- | --- |
| `foc/core/foc_output.h` | 49 → 57 | 新增 `foc_out_kind_t` |
| `foc/core/foc_run.h` | 92 → 94 | `FocCmd_t` 改为 `kind + vd/vq/theta + duty_u/v/w`；删 `enable`、删 `ReapplyStatic` 声明 |
| `foc/core/foc_run.c` | 345 → 304 | `SetMode` 去掉全部输出动作；`Step20k` 每拍填 `kind`；删 `ReapplyStatic`、删 `SetLockDuty` 立即写 |
| `foc/foc_control.c` | 411 → 422 | 纯执行器 + `fc_out_state_t s_out_state` 去重；删 `s_last_static_mode`/`s_if_resumed`/`FC_LAST_MODE_IN_IF`/2% 分支 |
| `foc/proc/foc_proc.c` | 239 → 238 | 删 `Foc_Run_ReapplyStatic()` 调用 |
| `foc/foc_protect.c` | 308 → 311 | `Foc_Protect_Trip()` 改用 `Foc_Output_SafeStop()`（审计 M1/L1 兜底） |
| `bsp_dev_hall_uvw.c`、`bsp_dev_motor.c` | +9 / +12 | 补 `Foc_Run_PwmOwned()` 门控（审计 H1 修复） |

#### 14.9.2 构建证据

```
Clean 全量重建：240 个 .o / 240 个 .d，串行 make 82 秒
error 0 / warning 0（唯一 error 行是已知的 rtthread.hex 递归 make 环境坑）
text 386392   data 3548   bss 34448   dec 424388
rtthread.elf 2026-11-20 11:38:49（2,231,368 B）
```

对比 Phase 4-lite（`386696 / 3548 / 34452`）：**text −304 B、data 0、bss −4 B**，方向与"删分支、删两个 static 变量"一致。
> 注：以上尺寸均为 **GCC 10.3.1（env tools）**；RT-Thread Studio 用的 **GCC 5.4.1** 编同一份源码为 `376512 / 3552 / 34436`，两者不可混比，详见 §15.5。
符号：`nm` 无 `Foc_Run_ReapplyStatic`；`Foc_Output_SafeStop` 有调用者（`foc_control.o`、`foc_protect.o`）。

#### 14.9.3 独立审计结论（子智能体 `phase4-auditor`，只读不改源）

**审计复核**：Clean 全量 240 `.o`、83.7 s、0 error / 0 warning、尺寸与上表一致、源码未被改动（源文件 mtime 早于审计会话）。

| 等级 | 发现 | 处置 |
| --- | --- | --- |
| 高危 H1 | `bsp_dev_hall_uvw.c::pwm_bridge_switch()` 有 **111 处** `TMR4_OC_SetCompareValue` 直写三相桥臂，`bsp_dev_motor.c::dev_motor_init/stop()` 也直写且 `TMR4_Start`，两者**都没有** `Foc_Run_PwmOwned()` 门控。当前无调用者且被 `--gc-sections` 剔除（`nm` 无符号），一旦被接入就会绕过 `foc_output`、使 `s_out_state` 失配 | **已修复**：三处入口加 `Foc_Run_PwmOwned()` 早退 + include |
| 中 M1 | `focmode 12/13` 阻塞流程期间 `s_out_state` 仍是 `ALL_LOW` 而桥臂在 50%，状态与物理不一致（`s_busy` 从不置 1，UNKNOWN 机制不覆盖）。当前功能安全，但若期间过流 trip 且 EMB 使能，会跳过 brake | **已兜底**：`Foc_Protect_Trip()` 改用 `SafeStop()`（全低+抱闸），不再依赖状态去重。**打开 `BSP_PWM_EMB_ENABLE=1` 前必须复查此窗口** |
| 低 L1 | 状态已是 ALL_LOW 时进入 IDLE 不再重复 brake | 同上被 `SafeStop` 兜底；EMB=0 下本就无动作 |
| 低 L2 | 执行器 `(void)` 忽略 `Release/Resume/Duty3` 返回值，失败也置 `DRIVING` | 接受并记录：失败仅 `s_bConfigured == false` 时可能，初始化后不可达 |
| 低 L3 | §14.7 第 3 条字面未达成（`foc_control.c` 仍有 `[IF]` 心跳与 overrun 日志的模式查询） | **已修正文档**：该两处属 Phase 5 |
| 低 L4 | `StartSweep()` 在 `g_align_active=1` 之前就 Resume + 首段 duty3，存在 <1 µs 窗口被 IDLE 拍覆盖 | 既有问题（旧去重同样存在），非本次回归，记录待 Phase 5 一并处理 |
| 低 L5 | §14.6 未列全行为差异（驱动互切不再重初始化、`SetLockDuty` 立即写删除、失败兜底删除、日志改名） | **已补** §14.6 第 7~10 条 |
| 低 L6 | `Foc_Control_ResetStats()` 全仓无调用者 | 既有死代码，保留 |
| — | **覆盖性最终结论** | 当前链接镜像内**未发现**未被 `s_out_state` 覆盖且可达的执行器外桥臂写点 |

#### 14.9.4 待实机（§14.7 第 4~10 条，审计建议按风险排序）

1. **模式切换落点与去重**（最高风险）：示波器看 PB4~PB9，跑 `0 → 1 5 → 0`、`0 → 4 0 0.5 → 0`、`0 → 13 → 0`。确认：① 进 IDLE 后 ≤50 µs 一定全低（不能因状态误判继续输出矢量——这是本设计唯一"安全方向相反"的失效模式）；② 进驱动一定先 `release+resume` 再 `duty3`；③ 切换瞬间上下管无直通。
2. **过流 trip 与标定收口**：驱动中触发 trip → 立即全低 + 抱闸 + 回 IDLE 且不再回驱动；`focmode 3` 结束后 ≤1 个 PWM 周期全低（覆盖 2%）；`focmode 12/13` 打印与终态全低；标定中途 trip 必须中止标定并全低。
3. **LOCK 每拍 duty3**：`focmode 1 5` 的 `cmpH/cmpL/REG` 回读与基线一致；`g_dwt_isr_max_cyc`/`g_fc_isr_dt_max_us` 对比 IDLE/LOCK/VECTOR 三档，LOCK 允许 ≤+5%（静态估算 +1% 量级）。
4. 其余：`focmode 2 1 1.5` 三相正弦与 `[IF]` 心跳；`focmode 4/9/10` 闭环响应；`focmode 3/5/6/7` 标定打印与结果。

---

## 15. 变更清单与 PWM 底层配置影响核对（2026-11-20）

> 本节回答两个问题：① 这轮重构到底改了哪些文件；② 有没有碰 PWM 的底层配置。

### 15.1 结论：PWM 底层配置零改动

频率、死区、同相/互补、采样点、有效电平、引脚复用、ADC 触发链、DDL 驱动——**一个都没改**。证据（文件 mtime 全部早于本次会话，本会话从 09:20 开始）：

| 文件 | 最后修改 | 内容 |
| --- | --- | --- |
| `foc/cfg_foc_table.h` | 11-17 14:12 | 配置表；仍为 `FOC_CFG_SELECT = 2`（20 kHz / 互补 / LOWER 采样 / 死区 50 ns / 高有效） |
| `foc/cfg_motor.h` | 11-17 09:37 | 极对数、CPR |
| `foc/cfg_sampling.h` | 11-18 14:45 | 三版采样板型、母线标定 |
| `foc/foc_pwm.c` / `.h` | 11-16 16:56 | 配置表展开层 |
| `drv/hw_tmr4_pwm.c` / `.h` | 11-17 15:00 / 00:21 | TMR4 PWM / 死区 / EVT / 寄存器诊断（**全文件无本次改动痕迹**） |
| `drv/bsp_adc_foc.c` | 11-19 22:43 | ADC1 SEQ_A + DMA1 CH2 + 帧回调 |
| `board/board_config.c` | 11-16 16:56 | PB4~PB9 复用、关闭 JTAG 保留 SWD |
| `bsp/bsp_pwm_config.c` | 11-04 18:24 | 旧六步 PWM 初值 |
| `libraries/.../hc32_ll_tmr4.c` | 11-16 18:38 | DDL 3.3.0 修正 |

### 15.2 触碰到的 PWM 相关代码（3 处，都不是配置）

| 文件 | 改动 | 对寄存器行为的影响 |
| --- | --- | --- |
| `bsp/bsp_pwm.c` | 仅符号改名：`Foc_Mode_Init` → `Foc_Run_Init`、`Foc_Mode_PwmOwned` → `Foc_Run_PwmOwned`，include 换路径 | **无**。`foc_pwm_apply_cfg()` 覆盖 TMR4 配置的整段逻辑（620~648 行）未动 |
| `foc/foc_protect.c` | 过流 trip 由 `Foc_Output_AllLow()` 改为 `Foc_Output_SafeStop()`（全低 + 抱闸） | `BSP_PWM_EMB_ENABLE=0` 时 `bsp_pwm_emergency_brake()` 是空函数体 → **无**；将来打开 EMB 才会多置一次 SOE3 |
| `bsp_dev_hall_uvw.c`、`bsp_dev_motor.c` | 三处入口加 `Foc_Run_PwmOwned()` 早退门控（Phase 4 审计 H1 修复） | **如果被调用会改变寄存器行为**：`pwm_bridge_switch()` 不再写 111 处比较值、`dev_motor_stop()` 不再写 6 路比较值、`dev_motor_init()` 不再 `TMR4_Start`。当前这三个函数**全仓无调用者**，已被 `--gc-sections` 剔除（`nm rtthread.elf` 查 `dev_motor\|pwm_bridge_switch\|dev_hall` 零命中）→ **固件内无差异**，属源码级保险 |

### 15.3 运行时「什么时候写 PWM」变了（设计目标，不是配置）

| 场景 | 之前 | 现在 |
| --- | --- | --- |
| IDLE 全低 + 抱闸 | `SetMode(IDLE)` 在线程里写 | ISR 检测到 `kind = ALL_LOW` 且状态变化时写（≤50 µs） |
| LOCK 占空比 | 进入模式时写一次；改占空比时立即写 | 每拍由 ISR 写 `duty3`（值不变）；改占空比下一拍生效 |
| 驱动模式切换 | 每次 `SetMode` 都做 `release + resume`（三相 PWM 重初始化） | 只在 `ALL_LOW → DRIVING` 做一次；驱动模式之间互切不重初始化 |
| 标定结束的 2% | 1 ms 收口覆盖 | 下一拍（≤50 µs）由 `ALL_LOW` 覆盖 |

`set_all_low()` / `resume_output()` 内部要做三相 PWM 重初始化，**不能每拍调用**，所以执行器按桥臂物理状态（`s_out_state`）去重——详见 §14.2/§14.5。

### 15.4 本会话全部改动清单

**新增（4 组 8 个文件）**

| 文件 | 行数 | 阶段 |
| --- | ---: | --- |
| `foc/core/foc_run.c` / `.h` | 304 / 94 | Phase 3 → 4 改写 |
| `foc/proc/foc_proc.c` / `.h` | 238 / 62 | Phase 3 |
| `foc/core/foc_output.c` / `.h` | 55 / 57 | Phase 4-lite → 4 扩展 `kind` |
| `foc/core/foc_feedback.c` / `.h` | 57 / 51 | Phase 4-lite |

**删除**：`foc/foc_mode.c`（414 行）、`foc/foc_mode.h`（93 行）

**修改（FOC 域）**：`foc_control.c`、`cli/foc_cli.c`、`foc_protect.c`、`encoder_calib.c`、`foc_current_loop.c`、`foc_speed_loop.c`、`foc_position_loop.c`、`diag/foc_vofa.c`

**修改（FOC 域外，仅符号改名或加门控，无逻辑改造）**：`bsp/bsp_pwm.c`、`boot/boot_table.c`、`bg_modules.c`、`motor/drv/motor_drv_bldc_6step.c`、`mode/sys_caps.c`、`bsp_dev_hall_uvw.c`、`bsp_dev_motor.c`

**构建脚本**：`applications/motor/SConscript`、`Debug/makefile`、`Debug/sources.mk`、`Debug/.../foc/{subdir.mk,core/subdir.mk,proc/subdir.mk}`

**文档**：本文件

### 15.5 构建体系说明（重要，避免尺寸对不上账）

本机存在**两套 ARM 工具链**，同一份源码编出来的尺寸不同：

| 工具链 | 路径 | 本阶段尺寸（text/data/bss） |
| --- | --- | --- |
| **GCC 10.3.1**（env tools） | `D:\RT-ThreadStudio\platform\env_released\env\tools\gnu_gcc\arm_gcc\mingw\bin` | **386392 / 3548 / 34448**（本次所有构建验证用它） |
| **GCC 5.4.1**（IDE 默认，烧录用） | `D:\RT-ThreadStudio\repo\Extract\ToolChain_Support_Packages\ARM\GNU_Tools_for_ARM_Embedded_Processors\5.4.1` | **376512 / 3552 / 34436**（2026-11-20 11:52 IDE 构建） |

因此：**本文档 §12.10/§13.10/§14.9 里的尺寸都是 GCC 10.3.1 的**；Phase 2 基线 `388092/3244/35164` 也是同一工具链，可以互相比较。拿 IDE 的 376512 去和 386392 比会得出"少了 9.8 KB"的错误结论。

**调试 `Debug/*.mk` 的状态**：2026-11-20 11:52 IDE 刷新工程时重新生成了全部 `subdir.mk`/`sources.mk`，内容与手工补充**一致**（`foc/core` 含 `foc_run/foc_output/foc_feedback` 三个源文件，`foc/proc` 含 `foc_proc`，`foc/` 顶层已无 `foc_mode.c`），说明 `.cproject` 的根 `sourceEntries` 能自动发现新子目录。后续不需要再手工维护这些 `.mk`。

### 15.6 复核命令

```powershell
# 1) PWM 配置是否被改：mtime 应全部早于本次会话
Get-ChildItem 'applications\motor\foc\cfg_foc_table.h','applications\motor\foc\foc_pwm.c',
              'applications\motor\drv\hw_tmr4_pwm.c','board\board_config.c' | Select-Object LastWriteTime,Name

# 2) 当前生效 PWM 配置
Select-String -Path 'applications\motor\foc\cfg_foc_table.h' -Pattern '^#define FOC_CFG_SELECT'

# 3) 旧六步遗留模块是否真的不在固件里（应无输出）
arm-none-eabi-nm Debug\rtthread.elf | Select-String 'dev_motor|pwm_bridge_switch|dev_hall'

# 4) 构建（必须串行；-j 会卡死）
$env:Path = 'D:\RT-ThreadStudio\platform\env_released\env\tools\gnu_gcc\arm_gcc\mingw\bin;D:\RT-ThreadStudio\platform\env_released\env\tools\bin;' + $env:Path
Set-Location Debug; make rtthread.elf rtthread.siz
```

---

## 附录 A：关键证据位置

> 行号基于 2026-11-19 重构前版本；Phase 1/2 后 `foc_mode.c` 已拆分，Phase 3 后该文件已删除（内容在 `foc/core/foc_run.c`、`foc/proc/foc_proc.c`、`foc/cli/foc_cli.c`）。下表的 `foc_mode.*` 行号仅作历史对照。

| 问题 | 位置 |
| --- | --- |
| `foc_mode.c` CLI/参数/打印占 2/3 | `foc_mode.c:402-1219`（Phase 2 已拆出） |
| 模式层直接操作 PWM/EMB 并阻塞 | `foc_mode.c:150-288`（Phase 3 后见 `foc_run.c`） |
| mode 12/13 清回 IDLE | `foc_mode.c:247-288`（Phase 3 后见 `foc_proc.c`） |
| 控制层重复决定静态输出 | `foc_control.c:250-265`（Phase 4 目标） |
| 控制层无条件跑 Clarke/Park | `foc_control.c:209-211` |
| 位置环直接读编码器全局 | `foc_position_loop.c:59-65` |
| 电流环依赖编码器/保护/观测 | `foc_current_loop.c:10-15` |
| VOFA 依赖 6 个 FOC 模块 | `foc_vofa.c:24-30`（文件已搬到 `diag/`，依赖仍在） |
| FOC 知道旧六步 | `foc_mode.h:19-21`、`foc_mode.h:70-71`（已迁到 `foc_run.h`，Phase 4 处理） |
| 控制宿主里调试打印 | `foc_control.c:272-312` |
| SConscript 只匹配 `foc/*.c` | `applications/motor/SConscript:4`（Phase 2 已修复） |

## 附录 B：目标目录与文件清单

> 标注 ★ 的已在 Phase 3 落地；其余为 Phase 4/5 目标。

```
applications/motor/drv/
  hw_tmr4_pwm.c/.h
  hw_encoder.c/.h
  bsp_adc_foc.c/.h

applications/motor/foc/core/
  foc_control.c/.h
  foc_run.c/.h           ★ Phase 3
  foc_output.c/.h        (Phase 4)
  foc_feedback.c/.h      (Phase 4)
  foc_protect.c/.h
  foc_params.c/.h        (可选, Phase 5)

applications/motor/foc/loops/
  foc_current_loop.c/.h
  foc_speed_loop.c/.h
  foc_position_loop.c/.h
  pid_ctrl.c/.h

applications/motor/foc/math/
  clarke_park.c/.h
  svpwm.c/.h
  foc_sincos.c/.h

applications/motor/foc/proc/
  encoder_calib.c/.h
  foc_proc.c/.h          ★ Phase 3（替代原计划的 foc_offset_cal.c）
  foc_offset_cal.c/.h    (Phase 3 直接并入 foc_proc.c，暂不单独拆文件)

applications/motor/foc/config/
  cfg_foc_table.h
  cfg_motor.h
  cfg_sampling.h

applications/motor/foc/cli/
  foc_cli.c

applications/motor/diag/
  foc_vofa.c/.h
  foc_eoca_probe.c/.h
```
