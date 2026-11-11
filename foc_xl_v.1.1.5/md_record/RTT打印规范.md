# RTT 打印规范

本规范统一 ws 文件夹下所有模块的 RTT 调试打印写法。

---

## 1. 打印开关宏（标准写法）

每个模块在自己的 `.h` 头文件中定义 **0/1 赋值式** 开关宏（不要用 `#ifdef`），参照：
`HC32F460_DDL_Rev3.3.0/projects/ev_hc32f460_lqfp100_v2/ws/I.h#L36-42`

```c
/* Debug switch: 1 = RTT prints on, 0 = off */
#define DEBUG_I_WS   0
#if DEBUG_I_WS
    #define I_DEBUG(fmt, ...)    MAIN_D("[I] " fmt, ##__VA_ARGS__)
#else
    #define I_DEBUG(fmt, ...)    ((void)0)
#endif
```

要点：

| 项目 | 规则 |
|------|------|
| 开关形式 | `#define XXX_DBG 0`（0/1 赋值式，**禁止** `#ifdef XXX`） |
| 开关值 | 1 = 打印开，0 = 打印关（默认 0） |
| 宏命名 | `<模块>_<DBG>`，如 `FOC_IQPI_DBG`；电流模块沿用 `DEBUG_I_WS` |
| 打印封装 | `<模块>_DBG(fmt, ...)`，内部走 `MAIN_D`，前缀统一 `[模块名] ` |
| 定义位置 | 各模块自己的 `.h` 头文件，`.c` 只调用封装宏 |

---

## 2. 打印内容约束

1. **禁止打印浮点型**：SEGGER_RTT_printf 不支持 `%f`。
   - 小数改为「整数部分 + 小数部分」两个整型分别打印，或直接缩放为整型（mV / mA / mrad 等）。
   - 示例：`v = 1.2345V` → 打印 `v=%d.%03d`，`(int)v, (int)(v*1000)%1000`。
2. **禁止打印中文**：RTT 输出中文可能乱码，日志统一英文。
3. **禁止发送 +Inf 类特殊浮点**：JustFloat 协议中 0x7F8000F0（+Inf）会被 VOFA+ 误判为帧尾导致分帧错误。

---

## 3. 当前各模块开关宏一览

| 文件 | 开关宏 | 打印封装 |
|------|--------|----------|
| I.h | `DEBUG_I_WS` | `I_DEBUG` |
| cur_loop.h | `CUR_LOOP_DBG` | `CURLOOP_DBG` |
| dev_comm_runner.h | `COMM_RUNNER_DBG` | — |
| foc_calib.h | `FOC_CALIB_DBG` | — |
| foc_cal.h | `FOC_CAL_DBG` | `CAL_DBG`（[CAL] 前缀，打印在 foc_obs 事件段，ISR 内不打印） |
| foc_iq_pi.h | `FOC_IQPI_DBG` | `IQPI_DBG` |
| foc_obs.h | `FOC_OBS_DBG` | `OBS_DBG` |
| foc_zizeng.h | `FOC_ZIZENG_DBG` | — |
| hall_sensor_3ch.h | `HALL_SENSOR3_DBG` | — |
| foc_51_rs.h | `FOC_RS51_DBG` | `RS51_DBG`（[RS51] 前缀，打印在 foc_obs 事件段，ISR 内不打印） |
| foc_52_ldlq.h | `FOC_LDLQ52_DBG` | `LDLQ52_DBG`（[LDLQ52] 前缀，打印在 foc_obs 事件段，ISR 内不打印） |
| foc_53_flx.h | `FOC_FLX53_DBG` | `FLX53_DBG`（[FLX53] 前缀，打印在 foc_obs 事件段，ISR 内不打印） |
| Hardware.h（Adp 层，母线电压 PA4） | `DEBUG_VBUS` | `VBUS_DBG`（[VBUS] 前缀） |
| main.c（无独立 .h，定义在文件顶部） | `DEBUG_MAIN` | `MAIN_DBG` |

新增模块时按第 1 节模板添加，并同步更新本表。
