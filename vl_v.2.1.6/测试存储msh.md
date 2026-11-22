# 存储验证 MSH 指令手册

- 适用工程：`vl_v.2.0.5`
- 固件入口：`t_param`（[param_cmd.c](file:///d:/xl/vl_v.2.0.5/applications/param/param_cmd.c)）
- 存储实现：`applications/param/param_store.c` / `param_manager.c` / `param_port_efm.c`

所有打印只用整型/ASCII，不含中文与浮点（RTT 控制台限制）。

---

## 1. 存储布局

| 组 | 扇区 | 扇区地址（EFM 别名） | 单条记录 | 每扇区可存 | 擦除上限 | 换扇规则 |
| --- | ---: | --- | ---: | ---: | ---: | --- |
| motion | 56 | `0x00070000` | 116 B | 70 | 不限 | 单扇区，满了原地擦除 |
| angle_mech | 57 | `0x00072000` | 76 B | 107 | 不限 | 单扇区，满了原地擦除 |
| system | 58 | `0x00074000` | 128 B | 64 | 不限 | 单扇区，满了原地擦除 |
| fast | 59 | `0x00076000` | 44 B | 186 | 10000/扇区 | 先用满并原地擦除，擦除次数到 10000 后下一次写满才换扇 |
| fast | 60 | `0x00078000` | 44 B | 186 | 10000/扇区 | 同上 |
| fast | 61 | `0x0007A000` | 44 B | 186 | 10000/扇区 | 同上，顺序 61→60→59→61 |
| 预留 | 62 | `0x0007C000` | - | - | - | 不由本模块使用 |
| device | 63 | `0x0007E000` | 108 B | 75 | 不限 | 单扇区，满了原地擦除 |

> HC32F460 EFM 使用 `0x00000000` 起始的别名地址。调试器 Memory 窗口直接填 `0x00070000`，不要加成 `0x08070000`。

### 1.1 记录格式（所有组统一）

```
offset 0        head          uint32  记录 magic
offset 4        seq           uint32  序号，单调递增，上电取最大
offset 8        erase_count   uint32  写这条记录时当前扇区的擦除次数
offset 12       payload       ...     各组数据
offset size-8   checksum      uint32  CRC32 over [0, size-8)
offset size-4   tail          uint32  magic ^ 0xA5A5A5A5
```

| 组 | magic | 期望 tail |
| --- | --- | --- |
| motion | `0x5501A001` | `0xF0A405A4` |
| system | `0x5502A002` | `0xF0A705A7` |
| angle_mech | `0x5503A003` | `0xF0A605A6` |
| device | `0x5504A004` | `0xF0A105A1` |
| fast | `0x5505A005` | `0xF0A005A0` |

CRC32：多项式 `0xEDB88320`，初值 `0xFFFFFFFF`，结果异或 `0xFFFFFFFF`。

### 1.2 找最新一条记录

上电扫描时取 `seq` 最大的有效记录（head/tail/CRC 都通过）。

- 下一条写入地址 = 组内 `curr_addr`
- 最新一条记录起始地址 = `curr_addr - record_size`
- 也可用 `t_param info` 的 `left` 反推：
  `latest = sector_base + (slots_per_sector - left - 1) * record_size`

---

## 2. 指令总览

| 指令 | 作用 |
| --- | --- |
| `t_param` / `t_param dump` | 存储状态 + 各组当前值 |
| `t_param info` | 只看状态：扇区、`left=剩余/总槽位`、擦除次数、`seq`、`wr`、`src` |
| `t_param values` | 只看各组当前值（motion/system 带 `0xID`） |
| `t_param set <id\|name> <val> [save]` | 改配置 RAM；带 `save` 立即落盘 |
| `t_param save [all\|cfg\|am\|dev\|fast]` | 按业务打包落盘；默认 all = CFG+AM+DEV，不含 fast |
| `t_param write <group> [count]` | 对某区精确写入 `count` 次（默认 1），只消耗该区槽位 |
| `t_param erasecnt <group> <n>` | 把该组当前扇区擦除次数设为 `n`（只改 RAM） |
| `t_param erase [all\|slow\|fast] confirm` | 擦除后把当前 RAM 值作为新记录写回 |
| `t_param fill <group> [leave]` | 对该区当前扇区反复写入，直到只剩 `leave` 槽（默认 2） |

`group` 取值：`motion`、`system`、`angle_mech`（`am`）、`device`（`dev`）、`fast`。

前提：**电机必须停机**（IDLE/FAULT）。RUNNING / RAMPING / BRAKING 时所有写/擦指令都会拒绝。

【临时调试旁路】WORK 模式默认没有 `PARAM_WRITE`。为便于现场验证，只要 FOC 处于 `focmode 0`（IDLE）或 `focmode 1`（LOCK），所有申请 `PARAM_WRITE` 的命令都临时放行（`t_param`、`t_mech_set`、`t_cfg set/save` 等）：

```
[CAP] temporary bypass: PARAM_WRITE allowed in focmode 0 (mode=WORK, caps=0x0193)
```

其余 FOC 模式（2/3/4/5/6/7）仍按能力位拒绝。该行为由 `sys_caps.c` 的 `SYS_CAP_TEMP_ALLOW_PARAM_WRITE` 控制，正式版应改为 0。

---

## 3. 常用验证流程

### 3.1 上电自动打印

启动表 `param_dump` 在自检末尾自动执行一次：

```
[PARAM] ===== storage status =====
[PARAM] motion     rec=116B sec=56 cur=56 left=68/70 erase=1 seq=1 wr=1 src=FLASH
[PARAM] angle_mech rec=76B sec=57 cur=57 left=106/107 erase=1 seq=0 wr=0 src=DEFAULT
[PARAM] system     rec=128B sec=58 cur=58 left=63/64 erase=1 seq=1 wr=1 src=FLASH
[PARAM] fast       rec=44B sec=59~61 cur=61 left=185/186 erase=1 seq=0 wr=0 src=DEFAULT
[PARAM]   sec 59: left=186/186 (after erase)
[PARAM]   sec 60: left=186/186 (after erase)
[PARAM]   sec 61: left=185/186 *cur
[PARAM]   total_left=557 (3 sectors)
[PARAM] device     rec=108B sec=63 cur=63 left=74/75 erase=1 seq=1 wr=1 src=FLASH
[PARAM] ===== current values =====
...
[PARAM] ===== end =====
```

- `src=FLASH`：记录从 Flash 读出且 CRC 通过。
- `src=DEFAULT`：该组仍是本次上电写入的默认记录（还没有真实保存）。
- 关闭自动打印：`param_store.h` 里 `PARAM_BOOT_DUMP_ENABLE 0`。

### 3.2 某区只存一次

```
t_param info                 # 记录起始 left/seq
t_param write motion 1       # 只写 motion 一条
t_param info                 # motion left 减 1，seq 加 1
```

`write` 只打包并保存指定组，不碰其它区；`save motion` 会同时写 motion + system（CFG）。

### 3.3 某区写到只剩 2 次

```
t_param fill fast 2
# [PARAM] fill fast done: writes=184 left=2/186 cur=61 seq=184 rc=0
```

`leave` 默认 2；写满后再写 1~2 次即可观察原地擦除（默认寿命未到）。

### 3.4 写满后的原地擦除（快区）

```
t_param erase fast confirm
t_param fill fast 0          # 把当前扇区写满，left=0/186
t_param write fast 1         # 寿命未到：cur 不变，erase 加 1，left=185/186
```

### 3.5 按擦除寿命换扇（写满后才换）

```
t_param erasecnt fast 10000
# [PARAM] fast erase_count: 1 -> 10000
# [PARAM] 'fast' will rotate sector 61 -> 60 after CURRENT sector is full
t_param fill fast 0          # 写满当前扇区，此时还不换扇
t_param write fast 1         # 写满 + 寿命到限：cur=60, erase=1, left=185/186
```

`erasecnt` 只改 RAM，不写 Flash；重启后擦除次数从记录里的 `erase_count` 恢复。

### 3.6 慢区原地擦除

```
t_param fill motion 2
t_param write motion 1       # left=1/70
t_param write motion 1       # left=0/70
t_param write motion 1       # erase+1, left=69/70，整扇区被擦除重写
```

### 3.7 擦除

```
t_param erase confirm        # 慢区 56/57/58/63 + 快区 59/60/61
t_param erase slow confirm   # 只擦慢区
t_param erase fast confirm   # 只擦快区
```

语义：格式化后用**当前 RAM 值**重写，SN / 机械 active 行 / 当前配置保留；`seq=1`、`erase=1`、`left=满容量-1`。恢复默认请用 `t_cfg erase confirm` / `t_mech_def`。

### 3.8 改值并掉电验证

```
t_param set pwm_freq_hz 20000 save
t_param values               # 确认 0x0501 = 20000
# 断电重启
t_param values               # 仍为 20000，且 src=FLASH
```

---

## 4. 调试器里看什么

### 4.1 Flash 扇区

Memory 窗口输入扇区基址：

| 组 | 基址 |
| --- | --- |
| motion | `0x00070000` |
| angle_mech | `0x00072000` |
| system | `0x00074000` |
| fast 59/60/61 | `0x00076000` / `0x00078000` / `0x0007A000` |
| device | `0x0007E000` |

以 motion 为例，写入 2 条记录后（`left=68/70`）：

- 第 1 条：`0x70000 ~ 0x70073`（116B）
- 第 2 条：`0x70074 ~ 0x700E7`
- 下一条写入地址：`0x700E8`

单条记录检查点（以第 2 条为例）：

| 位置 | 期望 |
| --- | --- |
| `0x70074 + 0` | `0x5501A001`（motion head） |
| `0x70074 + 4` | `seq`，应比上一条大 |
| `0x70074 + 8` | 写入时当前扇区 `erase_count` |
| `0x70074 + 108` | CRC32（`size-8`） |
| `0x70074 + 112` | `0xF0A405A4`（tail，`size-4`） |

`curr_addr` 可由程序变量直接读（见 4.2），比手算可靠。

### 4.2 关键变量

以下变量都是 `param_store.c` 的文件内静态变量；在 Keil 调试时建议在 `param_store.c` 作用域内添加到 Watch，或直接看地址。

| 变量 | 含义 |
| --- | --- |
| `s_rt_motion` / `s_rt_system` / `s_rt_am` / `s_rt_dev` / `s_rt_fast` | 各组的 `param_runtime_t`：`curr_sec`、`curr_addr`、`last_seq`、`erase_count`、`write_count`、`defaults_written` |
| `s_motion` / `s_system` / `s_am` / `s_dev` / `s_fast` | 各组记录 RAM 镜像：`head`、`seq`、`erase_count`、payload、`checksum`、`tail` |
| `s_cfg_motion` / `s_cfg_system` / `s_cfg_am` / `s_cfg_dev` / `s_cfg_fast` | 引擎配置：`sec_start`、`sec_end`、`record_size`、`max_erase_life`、`magic` |
| `s_pending` | 异步保存请求位 |
| `g_sys_params` | 编码器零点/校准标志/举升零点（业务 RAM） |
| `g_mech.tbl` / `g_mech.cfg` | 机械 active 行与派生量 |
| `g_device_info` | 设备信息（业务 RAM） |
| `s_rt[]`（`sys_config.c` 内静态） | 全部 sys_config 运行项与 dirty 标志 |

快速判断“当前写入位置”：

```
curr_addr = s_rt_xxx.curr_addr
latest    = curr_addr - s_cfg_xxx.record_size
```

### 4.3 验证要点

1. **写完立即读**：`seq` 加 1、`left` 减 1；Memory 里 `curr_addr` 指针同步后移。
2. **有效记录**：head/tail 正确 + CRC 通过；任何一项不符，上电会被跳过。
3. **换扇**：只有当前扇区**写满**且 `erase_count >= 10000` 时，`s_rt_fast.curr_sec` 才改变；新扇区基址出现 `seq` 更大的记录。
4. **写满原地擦除**：擦除次数未到 10000 时，整扇区被擦除后仍在原扇区，只剩基址一条新记录，`erase_count` 加 1。
5. **掉电恢复**：重启后 `t_param info` 的 `seq` 与掉电前最后一条一致，`src=FLASH`。

---

## 5. 常见问题

| 现象 | 排查 |
| --- | --- |
| `program FAILED ... (ret=-1)` | EFM 擦写缺 `EFM_SetBusStatus(EFM_BUS_HOLD)`，中断取指与 Flash 操作冲突（PGMISMTCH）；已在 `param_port_efm.c` 修复 |
| 擦除时 HardFault（pc=0xfffffffe） | 首先确认固件镜像没有占用参数扇区：上电 `storage status` 会打印 `app code_end=0x...`，必须小于 `0x00070000`；擦/写低于 code_end 的地址会被拒绝并打印 `refuse erase/write ... overlaps code` |
| `app code_end` 大于 0x70000 | 固件已压到参数区，必须裁剪代码或重新划分参数区，不能继续擦写 |
| 重启后 `src=DEFAULT` | 没找到有效记录：检查 head/tail/CRC；首次烧录属正常 |
| `left` 不减少 | 电机在运行（被拒绝）；或写 Flash 失败，看 `[PARAM] program FAILED` |
| 一直不换扇 | 快区只有“写满 + `erase>=10000`”才换扇；寿命未到时写满只原地擦除。慢区本来就不换扇 |
| `t_param: command not found` | `param_cmd.c` 没加入编译（Keil 工程需手动添加） |
| 值改了但重启丢失 | `t_param set` 未带 `save`；或保存被延后，用 `t_param save` 手动落盘 |
| 换扇方向不对 | 快区顺序固定 61→60→59→61，冷启动扫描按 `seq` 最大取记录，与扇区号无关 |

---

## 6. 实机验证记录（2026-11-19）

以下为 2026-11-19 实机 RTT 日志核对结果，均已通过。

| # | 验证项 | 操作 | 实测结果 |
| ---: | --- | --- | --- |
| 1 | 上电自检自动打印 | 上电 | `param_dump` 正常输出 `storage status` + `current values`；`app code_end=0x0005C258` < `0x00070000` |
| 2 | 慢区一键擦除 | `t_param erase slow confirm` | `mask=0x7 OK`，各组 `seq=1`、`erase=1`、`left` 复位，无 HardFault |
| 3 | 快区一键擦除 | `t_param erase fast confirm` | `cur=61`、`seq=1`、`erase=1`、`left=185/186`，无 HardFault |
| 4 | 慢区正常写入 | `t_param write motion 1` | `seq`/`wr` 加 1，`left` 减 1 |
| 5 | 慢区写满原地擦除 | `t_param fill motion 0` → `t_param write motion 1` | `cur=56` 不变，`erase 1→2`，`left=69/70` |
| 6 | 快区写满原地擦除 | `t_param fill fast 0` → `t_param write fast 1` | `cur=61` 不变，`erase 1→2`，`left=185/186` |
| 7 | 寿命到限后换扇 | `t_param erasecnt fast 10000` → `fill fast 0` → `write fast 1` | 写满前不换扇；写入时 `cur 61→60`，`erase=1`，`left=185/186` |
| 8 | 改值并落盘 | `t_param set pwm_deadtime_ns 1500 save` | `values` 显示 1500，随后改回 1000 正常 |
| 9 | `t_mech_set` 持久化 | `t_mech_set out_max_x10 2100` → 重启 | 重启后 `out=[-1000,2100]x10`，未被候选表覆盖 |
| 10 | 掉电/复位恢复 | 重启 | `seq`/`cur`/`erase` 全部保持，`src=FLASH` |
| 11 | WORK 模式写权限旁路 | `focmode 0/1` 下执行写命令 | `[CAP] temporary bypass` 后放行，`t_param` / `t_mech_set` 均可写 |
| 12 | 快区完整回绕 | `erasecnt fast 10000` + `fill fast 0` + `write fast 1`，重复两次 | `cur` 走 60→59→61，`seq` 373→558→559→744→745 单调递增，新扇区 `erase=1` |
| 13 | 全擦 | `t_param erase confirm` | `mask=0xF OK`；5 组均 `seq=1 erase=1`，`left` 复位（motion 69/70、AM 106/107、system 63/64、fast 185/186、device 74/75）；`values` 中 `out=[-1000,2150]x10`、SN、设备信息均保留 |

已知行为：`t_param set <id> <val> save` 实际按 `CFG|AM|DEV`（mask=0x7）保存，因此 `system`、`angle_mech`、`device` 的 `seq` 会一起递增；如需只写 CFG 可后续调整。擦除后 `wr` 是运行期累计写次数，不随擦除清零。

**未通过项（测试方法无效，不算失败）**：在 WORK + `focmode 2` 下执行写命令，三条都被 `[CAP] deny PARAM_WRITE` 拦在能力位，未进入 `motor_is_busy()` 检查，因此“电机运行中拒绝写”仍未验证。正确做法见第 7 节第 2 条。

---

## 7. 待验证项

尚未在实机执行的用例，按优先级排列：

1. **坏记录跳过**：`t_param write system 3` 后，用调试器把最新一条的 `head`（`0x5502A002`）改成 0 或改坏 CRC，重启后 `system seq` 应回退 1、`src=FLASH`、`values` 正常。最新记录地址 = `0x00074000 + (64 - left - 1) * 128`。
2. **电机运行中拒绝写**：必须先在 MAINT 下测（WORK + `focmode 2` 只会被能力位拦截）：
   ```
   t_param info
   t_wmode maint
   focmode 2
   t_param write motion 1
   t_param erase fast confirm
   t_param fill motion 0
   t_param erasecnt fast 10000
   t_param set pwm_deadtime_ns 1500 save
   focmode 0
   t_param values
   t_param info
   t_wmode work
   ```
   期望：前五条打印 `motor is running, ... refused, stop first`；`set ... save` 先改 RAM 再拒绝保存；停机后 `seq`/`left` 与测试前一致；`values` 中 `pwm_deadtime_ns=1500` 只在 RAM（未落盘），最后 `t_param set pwm_deadtime_ns 1000 save` 恢复。
3. **非默认 CFG 值跨重启**：`t_param set pwm_deadtime_ns 1500 save` → 断电重启 → `t_param values` 应为 1500 且 `src=FLASH`，再改回 1000。
4. **真实掉电**（非复位）：`t_param write system 100` 期间直接断电，重启后 `seq` 等于最后一条完整写入，`src=FLASH`，无坏记录被采纳。
5. **10000 次真实擦写老化**：`erasecnt` 只是模拟计数，未做真实寿命验证。
6. **SPI 绝对编码器联动**：快区字段与接口已预留，驱动未实现，无法验证上电直接用绝对编码器算电角度。

已通过：快区回绕 60→59→61、`t_param erase confirm` 全擦（见第 6 节第 12、13 项）。
