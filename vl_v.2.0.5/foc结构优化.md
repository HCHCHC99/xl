# FOC 结构优化方案

- 代码基线：`D:\xl\vl_v.2.0.5`（HC32F460 + RT-Thread）
- 范围：仅重构 `applications/motor/foc/` 的内部结构与文件边界，**不接入业务层、不改控制算法**
- 日期：2026-11-19
- 前置结论：当前 FOC 核心已可独立运行（msh 驱动），是重构成本最低的窗口期；一旦业务层通过 `swing_axis` 依赖 FOC 接口，再改代价会明显上升。

---

## 1. 结论

建议做，但按“**保行为、分阶段、可回退**”的方式做，不要一次性重写。

- 你的直觉是对的：`mode 几` 的写法不好，`foc/` 目录也存在职责越界。
- 问题不在编号本身，而在于**把“持续运行模式、一次性标定流程、调试命令”三类东西塞进了同一个枚举和同一个文件**。
- 最大收益、最低风险的两步是：
  1. 把驱动和调试文件从 `foc/` 搬出去；
  2. 把 `foc_mode.c` 里的 CLI、参数表、打印拆到 `foc_cli.c`。
- 模式枚举拆分（run mode / procedure）是真正的架构调整，建议放在第 3 步，并保留 `focmode` 命令兼容。

---

## 2. 现状数据

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

---

## 3. 现状问题

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

### Phase 1：纯搬迁（不改逻辑）

- `hw_tmr4_pwm.*`、`hw_encoder.*` → `motor/drv/`
- `foc_vofa.*`、`foc_eoca_probe.*` → `motor/diag/`
- 更新 `SConscript` 与 Eclipse `subdir.mk`
- 只改 include 路径，不改任何函数体

验收：

- 编译通过；
- 所有 `focmode`/`fstat`/`t_abz`/`t_vofa` 输出与基线一致；
- ISR 时间与基线一致。

### Phase 2：拆出 CLI（不改控制逻辑）

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

- `foc/` 顶层只保留 `core/loops/math/proc/config/cli` 六个子目录；
- `foc_mode.c` 不再存在，或降到 450 行以内；
- `foc/` 下不再有 `hw_*` 驱动文件和 `*_vofa/*_eoca_probe` 调试文件；
- 模式代码中不再出现 `FOC_MODE_3/5/6/7/12/13` 这类“流程编号”；
- 模式层不直接调用 `hw_tmr4_pwm_*`/`bsp_pwm_*`；
- 环路不直接调用 `hw_encoder_*`；
- `foc_control.c` 不判断 IDLE/LOCK，不写 2% 魔数；
- 所有 `focmode` 命令、VOFA 通道、ISR 时间与基线一致；
- 过流/堵转只有一个统一的安全停机入口。

---

## 10. 建议的实施顺序（如果时间有限）

| 优先级 | 做什么 | 预计收益 | 风险 |
| --- | --- | --- | --- |
| 1 | Phase 1 纯搬迁 | `foc/` 目录语义立刻清晰 | 极低 |
| 2 | Phase 2 拆 CLI | `foc_mode.c` 从 1219 行降到约 400 行 | 低 |
| 3 | Phase 3 拆 run/proc | 去掉“mode 几”的语义混乱 | 中 |
| 4 | Phase 4 output/feedback | 职责边界彻底清楚 | 中（动 ISR，需实测） |
| 5 | Phase 5 params/telemetry | 减少全局变量，调试更干净 | 低～中 |

**只做 1+2** 就能拿到大部分可读性收益；**做 1～4** 是完整的架构优化；Phase 5 可以后续再做。

---

## 附录 A：关键证据位置

| 问题 | 位置 |
| --- | --- |
| `foc_mode.c` CLI/参数/打印占 2/3 | `foc_mode.c:402-1219` |
| 模式层直接操作 PWM/EMB 并阻塞 | `foc_mode.c:150-288` |
| mode 12/13 清回 IDLE | `foc_mode.c:247-288` |
| 控制层重复决定静态输出 | `foc_control.c:250-265` |
| 控制层无条件跑 Clarke/Park | `foc_control.c:209-211` |
| 位置环直接读编码器全局 | `foc_position_loop.c:59-65` |
| 电流环依赖编码器/保护/观测 | `foc_current_loop.c:10-15` |
| VOFA 依赖 6 个 FOC 模块 | `foc_vofa.c:24-30` |
| FOC 知道旧六步 | `foc_mode.h:19-21`、`foc_mode.h:70-71` |
| 控制宿主里调试打印 | `foc_control.c:272-312` |
| SConscript 只匹配 `foc/*.c` | `applications/motor/SConscript:4` |

## 附录 B：目标目录与文件清单

```
applications/motor/drv/
  hw_tmr4_pwm.c/.h
  hw_encoder.c/.h
  bsp_adc_foc.c/.h

applications/motor/foc/core/
  foc_control.c/.h
  foc_run.c/.h
  foc_output.c/.h
  foc_feedback.c/.h
  foc_protect.c/.h
  foc_params.c/.h        (可选)

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
  foc_offset_cal.c/.h

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
