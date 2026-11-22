/**
 * @file    vehicle_control.h
 * @brief   整车状态控制对象 —— 继承基类的第 5 个业务对象
 *
 * 【2026-11-14 P2 / 设计基线 V3.0 §5】四态单机定稿
 *   OFF / ACC_ON / STARTED / FAULT
 *   依据：需求基线 §4.5（Q-A / Q-B / Q-C）、裁定-整车对象状态模型与E系列-V1.0.md
 *
 * 【相对旧 OFF/CRANKING/ON/STOPPING 的变更】
 *   ① 删除 CRANKING / STOPPING 过渡态 —— 去抖改由运行属性
 *      rpm_cond_ms / volt_cond_ms / fault_cond_ms 承担（平面状态机 + 显式子状态字段）
 *      【2026-11-15 需求基线 V1.9】Q-A 改为两条件**独立软件计时**，
 *      原 start_cond_ms 单一计时器作废（不再保留，避免双份真值）
 *   ② 删除 STOPPING / stop_pending / exit_rpm / exit_volt_dv / exit_ms（B-1 只进不退）
 *   ③ 新增 ACC_ON（门 1 由钥匙输入驱动）、FAULT（Q-B 故障降级，仅手动）
 *   ④ g_vehicle_cfg 改 RAM 运行副本，init 从配置表装载（R6、关闭 G-01 死配置）
 *   ⑤ 新增黑板只读接口、故障状态字、[TEMP-E6] engine_running
 *
 * 【继承】首字段 control_object_t base 即 obj_base_t（offsetof==0）；
 *         框架 obj_base_t 只有 1 个 sm 槽位 ⇒ 本对象只能挂 1 套状态机（实测 obj_base.h:85）。
 *
 * 【铁律 3 · 门禁边界】本对象只发布「车辆处于什么工况」，
 *   不发布「此刻能不能动作」；组合门禁归 app_main / sys_gate，不进本对象。
 */

#ifndef __VEHICLE_CONTROL_H
#define __VEHICLE_CONTROL_H

#include <stdint.h>
#include "../framework/core/control_object.h"
#include "../framework/manager/ctrl_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 轮询周期（须与 50ms tick 一致）
 * ============================================================ */
#define VEHICLE_POLL_MS              50U

/* ============================================================
 * 状态枚举（四态单机，用户 2026-11-14 定稿）
 * ============================================================ */
typedef enum {
    VEH_STATE_OFF = 0,          /* 未上电 / 未启车 */
    VEH_STATE_ACC_ON,           /* 钥匙上电（门 1 成立），尚未启车 */
    VEH_STATE_STARTED,          /* 已启车（门 2 成立）*/
    VEH_STATE_FAULT,            /* 故障降级：**仅手动**（Q-B ①）*/
    VEH_STATE_MAX
} vehicle_state_t;

/* ============================================================
 * 故障状态字 —— **只读汇聚快照**，不新建故障源（设计基线 §13A.2）
 * ============================================================ */
#define VEH_FAULT_CAN_SILENT      (1UL << 0)  /* Q-D1：>can_silent_ms 无任何帧触发解析线程查表 */
#define VEH_FAULT_VOLT_MISSING    (1UL << 1)  /* sys_voltage_is_ready()==0（Q-D3 解读 A）*/

/* ---- Q-D3 扩展点（2026-11-14 16:49「后续可能会添加其他条件，也能触发」）----
 * 当前按解读 A：Q-B 的两个条件以上述位做 **OR**（fault_word != 0 即计时）。
 * 预留 bit2~bit7。后续新增任何条件，只需两步，**不用改状态机/转移表/门禁/去抖**：
 *   ① 本处加一个  #define VEH_FAULT_XXX  (1UL << n)
 *   ② vehicle_poll() 汇聚段加一行  if (...) ctx->fault_word |= VEH_FAULT_XXX;
 * 判定链是「fault_word != 0 → fault_cond_ms 累计 → VEH_EVT_FAULT」，天然可扩展。
 * ⚠️ 刻意不设「有效位掩码」：掩码会让新增条件时多一个必须同步改的地方（漏改即静默失效）。
 * -------------------------------------------------------------------------- */

/* ============================================================
 * 判定参数（配置属性 → 运行副本，Q-84 重启生效）
 * ============================================================
 * Q-A（V1.9）：rpm > enter_rpm 连续 enter_ms **或** (volt_ok && dv > enter_volt_dv)
 *      连续 enter_ms —— **两条件各一个独立软件计时器，任一达标即启车**
 * Q-B：fault_word != 0 连续 fault_ms ⇒ 进 FAULT（两条件 OR，见故障位扩展点）
 *      CAN 静默门限 can_silent_ms（Q-D2，默认 500）
 * ============================================================ */
typedef struct {
    uint16_t enter_rpm;         /* 进入转速门限 (rpm)，默认 400 */
    uint16_t enter_volt_dv;     /* 进入电压门限 (0.1V)，默认 130 = 13.0V */
    uint16_t enter_ms;          /* 启车连续满足时长 (ms)，默认 500 */
    uint16_t fault_ms;          /* 故障连续满足时长 (ms)，默认 2000（SWR-N-002）*/
    uint16_t poll_ms;           /* 轮询周期 (ms)，须等于 tick 周期 */
    uint16_t can_silent_ms;     /* Q-D2：无任何报文触发查表的时长 (ms)，默认 500 */
} vehicle_cfg_t;

/** RAM 运行副本（R6）；ROM 默认值在 vehicle_trans_table.c，init 时被配置表覆盖 */
extern vehicle_cfg_t g_vehicle_cfg;

/* ============================================================
 * 业务上下文 = 运行属性
 * ============================================================ */
typedef struct {
    uint16_t rpm_cond_ms;       /* Q-A(V1.9) 转速条件独立软件计时；中断只清本计时器 */
    uint16_t volt_cond_ms;      /* Q-A(V1.9) 电压条件独立软件计时；中断只清本计时器 */
    uint16_t fault_cond_ms;     /* Q-B 故障信号连续满足时长；消失清零（2s/2s）*/
    uint16_t last_rpm;          /* 最近一次转速采样 */
    uint16_t last_dv;           /* 最近一次电压采样（0.1V）*/
    uint8_t  last_volt_ok;      /* 最近一次电压采集通道是否就绪 */
    uint8_t  engine_running;    /* [TEMP-E6] 只读观测快照；不参与迁移/门禁/fault_word */
    uint32_t fault_word;        /* VEH_FAULT_* 汇聚快照 */
    uint32_t started_count;     /* 累计进入 STARTED 次数 */
    uint32_t off_count;         /* 累计回到 OFF 次数（D-4 后不发 STOPPED，仅内部统计）*/
} vehicle_context_t;

/* ============================================================
 * 整车控制对象（与 swing/lift/mode/seq 同构）
 * ============================================================ */
typedef struct {
    control_object_t  base;         /* 基类（必须首位） */
    StateMachine_t    sm_instance;  /* 唯一状态机实例（静态分配） */
    vehicle_context_t ctx;          /* 业务上下文 */
} vehicle_control_t;

extern vehicle_control_t g_vehicle_obj;

/* ============================================================
 * 框架 / 对象方法
 * ============================================================ */
int vehicle_control_register(ctrl_manager_t *mgr);
vehicle_context_t *vehicle_control_get_context(void);
const char *vehicle_state_name(vehicle_state_t state);

/* ---- 黑板只读（单一真值源；推事件有 1 拍滞后，强一致读这里）---- */
vehicle_state_t vehicle_state_get(void);
uint32_t        vehicle_fault_word_get(void);
uint8_t         vehicle_engine_running_get(void);   /* [TEMP-E6] */
int             vehicle_cfg_load(void);             /* init 时从配置表装载到运行副本 */

/* ------------------------------------------------------------
 * 兼容 API：签名与原 service/vehicle_state.h 完全一致
 * ⇒ app_main.c / boot_table.c 调用点零改动
 * ------------------------------------------------------------ */
int     vehicle_state_init(void);
void    vehicle_poll(void);
uint8_t vehicle_is_started(void);
void    vehicle_force_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* __VEHICLE_CONTROL_H */
