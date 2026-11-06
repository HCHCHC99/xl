# TMR4 配置相关改动清单（方案 A）

日期：2026-11-06
范围：`ws/hw/hw_tmr4_pwm.c`、`ws/hw/hw_tmr4_pwm.h`
目标：让 `ws/cfg/cfg_foc_table.h` 的“输出模式 × 采样臂”四种组合（互补/同向 × 高侧/低侧）只靠改 `FOC_CFG_SELECT` 就能生效，且波形相位、占空比语义、采样触发点全部显式确定。

---

## 0. 现状与实测依据

- PCLK1 = 100 MHz，20 kHz 三角波：periodTicks = 2500，CPSR（峰值寄存器）= 2499，1 tick = 10 ns，半周期 25 µs。
- 实测：命令 50%（OCCRL = 1250）时 PB5 正脉宽 24.74 µs（49.48%），从 PB5 上升沿到 EOCA 中断入口 26.2 µs。
- 推算：PB5 高电平窗口中点 = 峰值（t=12.5 µs），ADC 触发在峰值，采样臂 = 高侧；转换+中断延迟约 13.7 µs。
- 结论 1：当前互补模式实际是“高侧在峰值导通、低侧在谷底导通”，配置 2（表里写 LOWER）实际采的是高侧。
- 结论 2：比较值被当作“关断点”使用，高侧占空比 = (CPSR − X) / CPSR，非 50% 命令时输出反相（命令 30% → 实际 70%）。50% 时看不出来。
- 结论 3：死区实际只有配置值的一半。500 ns 配出 25 tick = 250 ns，实测缺口 0.26 µs 与之吻合。
- 结论 4：同向模式两路都在谷底附近导通，上下管重叠，不能用于半桥。

---

## 1. 统一的波形几何（本清单所有改动的前提）

两种输出模式都必须产生同一套几何：

- 高侧导通：计数在 `(Xh, CPSR]` 区间为高，窗口居中于峰值。
  高侧占空比 `Dh = (CPSR − Xh) / CPSR`
- 低侧导通：计数在 `[0, Xl)` 区间为高，窗口居中于谷底。
  低侧占空比 `Dl = Xl / CPSR`
- 死区：`Xh − Xl = 死区 tick 数`（每个切换边各留一次，共两次/周期）
- 命令占空比 = 高侧占空比：`Xh = CPSR × (1 − Dh)`，`Xl = Xh − deadTicks`

采样点语义（与 `cfg_foc_table.h` 注释一致）：

| 采样点 | 物理含义 | ADC 触发 |
|---|---|---|
| CFG_SAMPLE_UPPER | 高侧导通区中点 | SCCR = CPSR，匹配条件 = 计数上溢（峰值） |
| CFG_SAMPLE_LOWER | 低侧导通区中点 | SCCR = 0，匹配条件 = 计数下溢（谷底） |

触发点与输出模式（互补/同向）完全无关，这是“四种组合自由选”的关键。

---

## 2. 逐项改动

### 2.1 `hw_tmr4_pwm.h`

1. 增加采样点枚举（供 hw_tmr4_pwm.c 内部使用，避免直接依赖 cfg 宏）：
```c
typedef enum {
    TMR4_SAMPLE_HIGH_SIDE = 0,   /* 峰值触发，采上桥臂导通区 */
    TMR4_SAMPLE_LOW_SIDE  = 1,   /* 谷底触发，采下桥臂导通区 */
} tmr4_sample_point_t;
```
2. 增加编译期开关（默认从 cfg 宏取值，集中在 h 内）：
```c
#ifndef TMR4_SAMPLE_POINT
#define TMR4_SAMPLE_POINT   TMR4_SAMPLE_LOW_SIDE   /* 由 cfg_foc_table.h 覆盖 */
#endif
```
3. 注释里写死两个前提：ACTIVE_HIGH 必须为 true；命令占空比 = 高侧占空比。
4. 修正 `hw_tmr4_pwm_set_duty3_deadtime` 的注释（当前无调用者，可选保留）。

### 2.2 死区换算 `DeadTimeNsToTicks`

- 现状（L73-L82）：`ns × timerHz / 2e9`，结果只有一半。
- 改为：`ns × timerHz / 1e9`。
- 实测依据：500 ns → 25 tick → 250 ns，与 PB5 脉宽缺口一致。
- 副作用：表里 500 ns 会变成真正的 500 ns，死区占比翻倍，低速小电流畸变会略增，属预期。

### 2.3 比较值换算

新增一个明确的换算函数，替换所有“直接写 cmp”的路径（互补与同向都用它）：

```c
/* 占空比 -> 比较值（峰值导通，比较值是关断点，X = CPSR*(1-D)） */
static uint16_t HighDutyToCmpH(float dutyPct)
{
    int32_t cpsr = (int32_t)s_ctx.periodTicks - 1;
    int32_t x;

    if (dutyPct < 0.0f)   dutyPct = 0.0f;
    if (dutyPct > 100.0f) dutyPct = 100.0f;

    x = cpsr - (int32_t)((float)cpsr * dutyPct / 100.0f);
    if (x < 1)        x = 1;                 /* 避开 0% 边界 */
    if (x > cpsr - 1) x = cpsr - 1;          /* 避开 100% 边界 */
    return (uint16_t)x;
}
```

- 同向模式下两路都用这一路换算；下管比较值 = 上管比较值 + deadTicks。
- 所有内部 `s_occrH[ch] / s_occrL[ch]` 仍保存“寄存器里的原始比较值”。

### 2.4 `OCModeCompLow`（互补模式唯一使用的比较通道配置）

改成显式电平动作，禁止依赖 INVT 与上电初始电平。位值：
`TMR4_OC_HOLD=0`，`TMR4_OC_HIGH=1`，`TMR4_OC_LOW=2`，`TMR4_OC_INVT=3`。

```c
static void OCModeCompLow(CM_TMR4_TypeDef *TMR4x, uint32_t ch)
{
    un_tmr4_oc_ocmrl_t m;
    m.OCMRx = 0U;
    /* OCF 状态位：保持置位，便于状态观察 */
    m.OCMRx_f.OCFDCL  = TMR4_OC_OCF_SET;
    m.OCMRx_f.OCFPKL  = TMR4_OC_OCF_SET;
    m.OCMRx_f.OCFUCL  = TMR4_OC_OCF_SET;
    m.OCMRx_f.OCFZRL  = TMR4_OC_OCF_SET;
    /* 电平动作：谷底强制低、上计数匹配拉高、峰值强制高、下计数匹配拉低。
     * 上计数必须写 HIGH：写 HOLD 只剩“峰值→下降沿”半个窗口，占空比会减半 */
    m.OCMRx_f.OPDCL   = TMR4_OC_LOW;    /* 下计数匹配 -> LOW  */
    m.OCMRx_f.OPPKL   = TMR4_OC_LOW;    /* 峰值匹配   -> LOW  (占空比 0% 边界) */
    m.OCMRx_f.OPUCL   = TMR4_OC_HIGH;   /* 上计数匹配 -> HIGH (峰值前的上升沿) */
    m.OCMRx_f.OPZRL   = TMR4_OC_HIGH;   /* 谷底匹配   -> HIGH (占空比 100% 边界) */
    m.OCMRx_f.OPNPKL  = TMR4_OC_HIGH;   /* 峰值无匹配 -> HIGH */
    m.OCMRx_f.OPNZRL  = TMR4_OC_LOW;    /* 谷底无匹配 -> LOW  */
    /* EOP 与 OP 镜像、bit19~16=0：保证独立操作模式。
     * 互补模式虽未使能 H 通道，但 OCCR*h 可能为 0（谷底恒匹配），
     * 不镜像会让谷底走 EOP 分支（HOLD），波形被破坏。 */
    m.OCMRx_f.EOPNDCL = TMR4_OC_HOLD;
    m.OCMRx_f.EOPNUCL = TMR4_OC_HOLD;
    m.OCMRx_f.EOPDCL  = TMR4_OC_LOW;
    m.OCMRx_f.EOPPKL  = TMR4_OC_LOW;
    m.OCMRx_f.EOPUCL  = TMR4_OC_HIGH;
    m.OCMRx_f.EOPZRL  = TMR4_OC_HIGH;
    m.OCMRx_f.EOPNPKL = TMR4_OC_HIGH;
    m.OCMRx_f.EOPNZRL = TMR4_OC_LOW;
    TMR4_OC_SetLowChCompareMode(TMR4x, ch, m);
}
```

推导（X 在 0..CPSR 之间）：
谷底 LOW → 上计数匹配 HIGH → 峰值保持 HIGH → 下计数匹配 LOW → 谷底 LOW。
高侧导通窗口 `(X, CPSR]`，宽度 `2×(CPSR−X)`，即 `Dh = (CPSR−X)/CPSR`。
若上计数写 HOLD，高电平只剩“峰值→下降沿”的一半，实测就是 50% 命令只有约 24% 占空比。
边界：X=0 → 全程 HIGH（100%）；X=CPSR → 全程 LOW（0%）。

### 2.5 `OCModeSyncHigh`（同向模式上管）

```c
static void OCModeSyncHigh(CM_TMR4_TypeDef *TMR4x, uint32_t ch)
{
    un_tmr4_oc_ocmrh_t m;
    m.OCMRx = 0U;
    m.OCMRx_f.OCFDCH  = TMR4_OC_OCF_SET;
    m.OCMRx_f.OCFPKH  = TMR4_OC_OCF_SET;
    m.OCMRx_f.OCFUCH  = TMR4_OC_OCF_SET;
    m.OCMRx_f.OCFZRH  = TMR4_OC_OCF_SET;
    /* 同向：上下管同相，导通窗口 (X, CPSR]（居中峰值） */
    m.OCMRx_f.OPDCH   = TMR4_OC_LOW;    /* 下计数匹配 -> LOW  */
    m.OCMRx_f.OPPKH   = TMR4_OC_LOW;    /* 峰值匹配   -> LOW  */
    m.OCMRx_f.OPUCH   = TMR4_OC_HIGH;   /* 上计数匹配 -> HIGH */
    m.OCMRx_f.OPZRH   = TMR4_OC_HIGH;   /* 谷底匹配   -> HIGH */
    m.OCMRx_f.OPNPKH  = TMR4_OC_HIGH;   /* 峰值无匹配 -> HIGH */
    m.OCMRx_f.OPNZRH  = TMR4_OC_LOW;    /* 谷底无匹配 -> LOW  */
    TMR4_OC_SetHighChCompareMode(TMR4x, ch, m);
}
```

结果：上下管同相，导通窗口 `(X, CPSR]`，`D = (CPSR−X)/CPSR`，PB5 与 PB4 同高同低（死区为 0 时完全一致）；峰值处导通、谷底处关断。

### 2.6 `OCModeSyncLow`（同向模式下管）

保持“峰值导通”，与上管同相，并补齐边界强制项与独立模式镜像：

```c
static void OCModeSyncLow(CM_TMR4_TypeDef *TMR4x, uint32_t ch)
{
    un_tmr4_oc_ocmrl_t m;
    m.OCMRx = 0U;
    m.OCMRx_f.OCFDCL  = TMR4_OC_OCF_SET;
    m.OCMRx_f.OCFPKL  = TMR4_OC_OCF_SET;
    m.OCMRx_f.OCFUCL  = TMR4_OC_OCF_SET;
    m.OCMRx_f.OCFZRL  = TMR4_OC_OCF_SET;
    m.OCMRx_f.OPDCL   = TMR4_OC_HIGH;   /* 下计数匹配 -> HIGH */
    m.OCMRx_f.OPPKL   = TMR4_OC_LOW;    /* 峰值匹配   -> LOW  */
    m.OCMRx_f.OPUCL   = TMR4_OC_LOW;    /* 上计数匹配 -> LOW  */
    m.OCMRx_f.OPZRL   = TMR4_OC_HIGH;   /* 谷底匹配   -> HIGH */
    m.OCMRx_f.OPNPKL  = TMR4_OC_LOW;    /* 峰值无匹配 -> LOW  */
    m.OCMRx_f.OPNZRL  = TMR4_OC_HIGH;   /* 谷底无匹配 -> HIGH */
    /* 独立操作模式：bit19~16=0，bit31~20 与 bit15~4 镜像 */
    m.OCMRx_f.EOPNDCL = TMR4_OC_HOLD;
    m.OCMRx_f.EOPNUCL = TMR4_OC_HOLD;
    m.OCMRx_f.EOPDCL  = TMR4_OC_HIGH;
    m.OCMRx_f.EOPPKL  = TMR4_OC_LOW;
    m.OCMRx_f.EOPUCL  = TMR4_OC_LOW;
    m.OCMRx_f.EOPZRL  = TMR4_OC_HIGH;
    m.OCMRx_f.EOPNPKL = TMR4_OC_LOW;
    m.OCMRx_f.EOPNZRL = TMR4_OC_HIGH;
    TMR4_OC_SetLowChCompareMode(TMR4x, ch, m);
}
```

结果：低侧导通在 `[0, Xl)`，`Dl = Xl/CPSR`。

### 2.7 `ApplySyncDutyWithDeadTime`

旧公式（h = center + dt，l = center − dt）按新几何重写：

```c
static void ApplySyncDutyWithDeadTime(tmr4_pwm_channel_t ch,
                                      float dutyPct,
                                      uint16_t deadTicks)
{
    const pwm_pair_desc_t *pair = &s_pairs[ch];
    uint16_t cpsr = (uint16_t)(s_ctx.periodTicks - 1U);
    int32_t xh = (int32_t)HighDutyToCmpH(dutyPct);   /* 同向：上管峰值导通 */
    int32_t xl = xh + (int32_t)deadTicks;

    /* 死区为 0 时两路完全相同；非 0 时下管推迟 deadTicks 开通、提前 deadTicks 关断 */
    if (xl > (int32_t)cpsr - 1) xl = (int32_t)cpsr - 1;

    OccrHWrite((int)ch, pair->ocChH, (uint16_t)xh);
    OccrLWrite((int)ch, pair->ocChL, (uint16_t)xl);
}
```

- 上下管同相、同时上升（都居中峰值），下管推迟 `deadTicks` 开通、提前 `deadTicks` 关断。
- 死区为 0 时 `Xh = Xl`，PB5 与 PB4 完全一致。
- 同向模式下两管同时导通，仅适合台架波形验证或门极带互锁的驱动，不能直接带功率级。
- `TMR4_DEADTIME_TICKS_MAX` 仍为 0x3FF，钳位保留。

### 2.8 `ShadowApplyOC`（影子装载点随采样点）

装载点必须避开采样瞬间，否则采样叠加 reload 噪声：

```c
static void ShadowApplyOC(uint32_t ocCh)
{
    uint16_t cond;

    if (TMR4_SAMPLE_POINT == TMR4_SAMPLE_HIGH_SIDE) {
        cond = TMR4_OC_BUF_COND_VALLEY;   /* 峰值采样 -> 谷底装载 */
    } else {
        cond = TMR4_OC_BUF_COND_PEAK;     /* 谷底采样 -> 峰值装载 */
    }

    TMR4_OC_SetCompareBufCond(CM_TMR4_3, ocCh, TMR4_OC_BUF_CMP_VALUE,
        TMR4_SHADOW_OCCR_ENABLE ? cond : TMR4_OC_BUF_COND_IMMED);
    TMR4_OC_SetCompareBufCond(CM_TMR4_3, ocCh, TMR4_OC_BUF_CMP_MD,
        TMR4_SHADOW_OCMR_ENABLE ? cond : TMR4_OC_BUF_COND_IMMED);
}
```

同时把 `config()` 里 `stcTmr4OcInit.u16CompareValueBufCond` / `u16CompareModeBufCond` 的初值改成同一 cond，保持一致。

### 2.9 `hw_tmr4_pwm_config()` 两个分支

互补分支（L330-L350）：
- `TMR4_OC_Init` 的比较初值改为 `HighDutyToCmpH(50.0f)`（即 Xh，50% 时约 1250）。
- `OCModeCompLow` 换成 2.4 的新版本。
- `OccrLWrite(i, ocChL, HighDutyToCmpH(50.0f))` 替换现在的 `periodTicks/2`。
- 其余不变：只使能 OCL、DEAD_TMR、PDAR/PDBR 等于同值、极性 HOLD/HOLD。

同向分支（L351-L374）：
- 初始化 OCH/OCL 后调用 2.5 / 2.6 的新版本。
- 用 `ApplySyncDutyWithDeadTime(ch, 50.0f, deadTicks)` 代替现在的 `(periodTicks/2, deadTicks)`。
- `TMR4_OC_Init` 的初值同样用 `HighDutyToCmpH(50.0f)` 给 OCH 和 OCL。
- 其余（THROUGH、极性、两路都使能）不变。

### 2.10 占空比写入路径（三处必须一起改）

- `hw_tmr4_pwm_set_duty_float`（L632-L663）
  互补：`OccrLWrite(i, ocChL, HighDutyToCmpH(centerPct))`
  同向 `TMR4_MODE_HIGH_SIDE`：`OccrHWrite(i, ocChH, HighDutyToCmpH(centerPct))`
  同向 `TMR4_MODE_LOW_SIDE`：保持“下管常开”语义（比较值 0 = 100%），该分支不随 `centerPct` 更新
  同向 `TMR4_MODE_OFF`：`ApplySyncDutyWithDeadTime(ch, centerPct, deadTicks)`

- `hw_tmr4_pwm_set_duty3`（L665-L696）
  把三处 `CenterPctToCompare` 换成上述对应换算；互补仍只写 OCL。

- `hw_tmr4_pwm_set_duty3_deadtime`（L701-L738）
  输入本来就是“高侧占空比”，新几何下 `actual_duty_to_center` 返回 high 本身，可简化为直接调 `set_duty3(highU, highV, highW)`；当前无调用者，保留接口即可。

- `hw_tmr4_pwm_set_output_type`（L967-L1039）
  互补：`OccrLWrite` 用 `HighDutyToCmpH(d50)`；同向：`ApplySyncDutyWithDeadTime`。
- `hw_tmr4_pwm_set_channel_mode`（L893-L962）
  `HIGH_SIDE` 的 `OccrHWrite` 用 `HighDutyToCmpH`；`LOW_SIDE` 的 100% 初值写 0。
- `hw_tmr4_pwm_resume_output`（L430-L488）
  调用新的 OCMode 函数；比较值不动。

### 2.11 `hw_tmr4_pwm_config_adc_trigger`（关键：与输出模式解耦）

```c
if (TMR4_SAMPLE_POINT == TMR4_SAMPLE_HIGH_SIDE) {
    stcEvtInit.u16CompareValue = (uint16_t)(s_ctx.periodTicks - 1U);  /* CPSR */
    stcEvtInit.u16MatchCond    = TMR4_EVT_MATCH_CNT_PEAK;
} else {
    stcEvtInit.u16CompareValue = 0U;
    stcEvtInit.u16MatchCond    = TMR4_EVT_MATCH_CNT_VALLEY;
}
```

- 与当前 L1081-L1087 正好对调。
- 不再引用 `FOC_CFG_OUTPUT_TYPE`，四种组合自然成立。
- 增加一条启动日志，把“输出模式 / 采样臂 / SCCR / 匹配条件”打出来，避免以后再看错：
```c
MAIN_D("[HW_TMR4] sample=%s trig=%s (%u)",
       (TMR4_SAMPLE_POINT == TMR4_SAMPLE_HIGH_SIDE) ? "HIGH" : "LOW",
       (TMR4_SAMPLE_POINT == TMR4_SAMPLE_HIGH_SIDE) ? "PEAK" : "VALLEY",
       (unsigned)stcEvtInit.u16CompareValue);
```
- `enable` 参数目前未使用；如需真正关断，补 `TMR4_EVT_MatchCondCmd(..., DISABLE)`。

### 2.12 读数/换算函数

- `hw_tmr4_pwm_get_state`（L772-L799）
  互补：`highPct = 100×(CPSR − X)/periodTicks − deadPct/2`；`lowPct = 100 − highPct − deadPct`。
  同向：`highPct = 100×(CPSR − Xh)/periodTicks`；`lowPct = 100×(CPSR − Xl)/periodTicks`（引脚无死区延迟）。
- `hw_tmr4_pwm_center_to_actual`
  互补保持 `high = center − dtPct/2`、`low = 100 − high − dtPct`。
  同向改为 `high = center`、`low = center − dtPct`。
- `hw_tmr4_pwm_actual_duty_to_center`
  互补：`center = high + dtPct/2`；同向：`center = high`。

> 说明：现有函数里“同向 high=center+dt、low=center−dt”是旧几何的产物，必须改；否则对外语义两种模式不一致。

---

## 3. 位配置速查表

| 场景 | 寄存器 | OPU C/D | OPPK | OPZR | OPNPK | OPNZR | EOP* |
|---|---|---|---|---|---|---|---|
| 互补（OCMRL） | OCMRul/vl/wl | U=HIGH, D=LOW | LOW | HIGH | HIGH | LOW | 与 OP 镜像，bit19~16=0 |
| 同向上管（OCMRH） | OCMRuh/vh/wh | U=HIGH, D=LOW | LOW | HIGH | HIGH | LOW | 无 |
| 同向下管（OCMRL） | OCMRul/vl/wl | U=HIGH, D=LOW | LOW | HIGH | HIGH | LOW | 与 OP 镜像，bit19~16=0 |

（HOLD=0，HIGH=1，LOW=2，INVT=3；互补与同向下管的 EOP 必须镜像 OP，且 bit19~16 为 0，保证独立操作模式。）

---

## 4. 边界与钳位

1. `HighDutyToCmpH` 把 X 钳在 `[1, CPSR−1]`，规避 0%/100% 的极值匹配歧义。
2. 同向模式 `Xl = Xh + deadTicks`，必须保证 `Xl ≤ CPSR−1`；死区过大时低侧脉冲先被压缩到最短。
3. `TMR4_DEADTIME_TICKS_MAX = 0x3FF`（10 bit）保持不变；换算修正后 500 ns = 50 tick，仍在范围内。
4. ACTIVE_HIGH 必须 true。若改成 false，两路整体反相，高/低侧窗口互换，采样映射失效；如需支持，必须在 `OCMode*` 里同步翻转并把采样点映射一起换。
5. 互补模式只用 OCL，OCH 保持 `OCEH=0`；不要顺手使能 OCH，否则会进入链接操作模式，行为改变。

---

## 5. 验证清单（对应四种组合）

统一前置：20 kHz、死区按修正后的真实值（例如 500 ns = 50 tick）、`ACTIVE_HIGH=true`。

1. 占空比极性
   - mode 1（LOCK，固定 50%）：PB5 脉宽 ≈ 25 µs − 0.5 µs = 24.5 µs。
   - mode 2/4 取一个非 50% 工作点：PB5 脉宽 ≈ 命令占空比 × 50 µs − 0.5 µs。若变成 (1−命令)×50 µs，说明换算还有漏改。
2. 采样点
   - 在 `Foc_Control_EocaHandler` 第一行读 `TMR4_GetCountValue(CM_TMR4_3)` 存入全局：
     - 峰值触发：CNT ≈ 2499 − N（N = 转换+中断延迟 tick 数，500 tick ≈ 5 µs）。
     - 谷底触发：CNT ≈ 0 + N。
   - 配合 PB5/PB4 电平确认采样瞬间落在对应导通区。
3. 组合矩阵
   - 配置 0：SYNC + UPPER → 峰值触发；同向模式下两管同相、居中峰值，峰值处两管都导通，采样落在导通区。
   - 配置 1：SYNC + LOWER → 谷底触发；谷底处两管都关断，采样落在关断区（同向模式下“低侧采样”没有对应的独立导通区）。
   - 配置 2/4：COMP + LOWER → 谷底触发，PB4 为高时采样。
   - 配置 5：COMP + UPPER → 峰值触发，PB5 为高时采样。
4. 装载点
   - 峰值采样时确认 OCCR/OCMR 在谷底装载（示波器看占空比跳变发生在谷底附近），反之亦然。
5. 同向模式不直通
   - 示波器同时看 PB5/PB4：任何时刻只能一路为高，间隔 ≥ 死区。

---

## 6. 本次改动会带来的行为变化

1. 死区由“名义值的一半”变成真实值，500 ns 配置下死区占比翻倍（20 kHz、50 µs 周期下 1%）。
2. 非 50% 命令下的实际相电压方向会翻转回正确方向。若整机符号（电流采样方向、编码器 dir）此前是按“反相”标定的，改完后 mode 3 对齐、mode 4 的 iq 正负可能整体反向，需要重新跑一次对齐/校准，并复核 iq 反馈符号。
3. 同向模式保持“上下管同相”（PB4/PB5 同高同低），导通区居中峰值，死区为 0 时两路完全相同；非 0 时下管推迟/提前 deadTicks。注意：同向模式下上下管同时导通，只适合台架波形验证或门极带互锁的驱动，不能直接带功率级。同向模式下“低侧采样”没有独立的低侧导通区（谷底处两管都关断）。
4. 采样点变为与模式无关，`cfg_foc_table.h` 里 `UPPER/LOWER` 的注释语义第一次真正成立。

---

## 7. 明确不在本次改动范围内

1. ADC 顺序采样导致的占空比窗口约束：3 个电流通道约需 10 µs，高侧采样要求占空比 ≳ 25%，低侧采样要求 ≲ 75%；全调制范围要可靠，需要“按扇区/占空比自动切换采样点”或双 ADC 同步采样。
2. vbus 仍在同一序列里（motor_sampling.c），会拉长转换时间；建议移到序列 B 或降低采样时间（20 → 8~10 ADCLK），可把 13.7 µs 的触发→中断延迟压下来。
3. 电流环 PI 与 SVPWM 的符号复核（由第 6.2 条触发，属另外的验证项）。
