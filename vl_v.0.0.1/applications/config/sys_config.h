/**
 * @file    sys_config.h
 * @brief   系统配置表 —— 可在线读写的运行参数（sys_config 分区）
 * @note    设计文档：docs/02-设计/方案-系统配置表sys_config-V1.0.md
 *
 * ============================================================
 * 【核心设计：三层取值 + 逐项回退】
 * ------------------------------------------------------------
 *   编译期默认值（硬编码，永远存在）
 *        ↓ 仅当 Flash 该项"有效"才覆盖
 *   Flash 读到的值
 *        ↓ 仅当运行时 set 才覆盖
 *   RAM 当前生效值
 *
 * 启动时先无条件加载全部默认值，再尝试读 Flash。
 * 读不到 / CRC 坏 / 某项类型不符 / 某项越界 ⇒ 该场景直接使用默认值。
 * 绝不允许坏数据进入业务，也绝不因配置丢失而停机。
 *
 * 【为什么不用裸结构体落盘】
 *   sys_param.h:36 写着"字段顺序与 sizeof 不得随意变更"，根因是
 *   storage 层用整块 magic+CRC 判有效性，结构一变整块作废
 *   （MARK-11 已踩过：16B→24B 导致现场标定数据全丢）。
 *   本模块改用「ID+type+value」自描述项数组，判定粒度下沉到"项"：
 *   新增参数只影响自己，老数据照常可用。
 *
 * 【与其它存储的分工】
 *   mech_param   齿数比/减速比   ← 唯一真值，本模块只镜像
 *   sys_param    编码器零点/举升零位 ← 唯一真值，本模块只镜像标志
 *   sys_config   运行阈值/开关     ← 本模块唯一真值
 * ============================================================
 */

#ifndef __SYS_CONFIG_H
#define __SYS_CONFIG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 1. 数据类型
 * ============================================================ */
typedef enum {
    CFG_T_U8  = 1,
    CFG_T_U16 = 2,
    CFG_T_I16 = 3,
    CFG_T_I32 = 4,
    CFG_T_U32 = 5
} cfg_type_t;

/* ============================================================
 * 2. flags 位定义
 * ============================================================ */
#define CFG_F_READONLY      0x01U   /* 只读，set 拒绝（镜像项用） */
#define CFG_F_REQ_RESTART   0x02U   /* 改后需重启生效 */
#define CFG_F_REQ_CALIB     0x04U   /* 修改前要求 is_calibrated=1 */
#define CFG_F_REQ_SAFE      0x08U   /* 修改前要求处于安全模式（非工作模式） */
#define CFG_F_ADVANCED      0x10U   /* 高级项，list 默认不显示 */

/* ============================================================
 * 3. 单条配置项（落盘形态，8 字节固定）
 * ============================================================ */
typedef struct {
    uint16_t id;        /* 参数 ID，一旦分配永不复用、永不改含义 */
    uint8_t  type;      /* cfg_type_t */
    uint8_t  flags;     /* 落盘时快照，便于脱离代码诊断 */
    union {
        uint8_t  u8;
        uint16_t u16;
        int16_t  i16;
        int32_t  i32;
        uint32_t u32;
    } v;
} cfg_item_t;

/* ============================================================
 * 4. 整表（落盘形态）
 * ============================================================
 * ============================================================
 * 128 项 × 8B = 1024B + 表头 8B = 1032B，加 storage 的 8B 块头 = 1040B。
 *
 * 【2026-11-10 从 96 扩到 128，为三环模块 4 实例让位】
 *   容量核算：
 *     扩前已用 50 项，余量 46
 *     三环模块 4 实例 × 13 项参数 = 52 项
 *     50 + 52 = 102 > 96 → 必须扩
 *   扩到 128 后：128 - 50 - 52 = 26 项余量，够后续两个模块。
 *
 *   上限校验（两条都必须满足）：
 *     ① storage.c:21 STORAGE_BLK_MAX_SIZE = 4096
 *        1040B ≪ 4096B ✓
 *     ② Flash 扇区 8192B（storage_config.h:19）
 *        1040B ≪ 8192B ✓（仍是单扇区内，不跨扇区）
 *
 *   ⚠ 扩容量的代价（已评估并接受）：
 *     · RAM 增加：(128-96) × (8B s_rt + 1B s_src + 1B s_dirty) = 400B
 *     · Flash 擦写时间：整扇区擦除 8192B，块大小从 784→1040 不改变擦除粒度
 *     · **向后兼容**：blob 头部有 version 与 count，
 *       旧固件读新 blob 会因 count > 96 被 :446 拒绝并回退默认值。
 *       即"降级刷旧固件会丢配置" —— 这是可接受的（不可能同时跑两版）。
 * ============================================================ */
#define CFG_MAX_ITEMS       128U

typedef struct {
    uint16_t   version;                 /* 表结构版本，仅诊断用 */
    uint16_t   count;                   /* 有效项数 */
    uint32_t   crc;                     /* 覆盖 version+count+item[0..count-1] */
    cfg_item_t item[CFG_MAX_ITEMS];
} sys_config_blob_t;

/* ============================================================
 * 5. 参数定义表（编译期常量，含默认值与范围）
 * ============================================================ */
typedef struct {
    uint16_t    id;
    const char *name;
    uint8_t     type;
    uint8_t     flags;
    int32_t     def_val;        /* 硬编码默认值 —— 永远可用的兜底 */
    int32_t     min_val;        /* 范围校验下界（来自代码，非 Flash） */
    int32_t     max_val;
    const char *unit;
} cfg_def_t;

/* ============================================================
 * 6. 取值来源（诊断用）
 * ============================================================ */
typedef enum {
    CFG_SRC_DEFAULT = 0,    /* 当前值来自编译期默认 */
    CFG_SRC_FLASH   = 1,    /* 来自 Flash，未被运行时修改 */
    CFG_SRC_RAM     = 2     /* 运行时 set 过（可能未保存） */
} cfg_src_t;

/* ============================================================
 * 7. 参数 ID 定义
 * ============================================================ */

/* --- 0x01xx 整车状态 --- */
#define CFG_ID_VEH_ENTER_RPM        0x0101U
#define CFG_ID_VEH_EXIT_RPM         0x0102U
#define CFG_ID_VEH_ENTER_VOLT_DV    0x0103U
#define CFG_ID_VEH_EXIT_VOLT_DV     0x0104U
#define CFG_ID_VEH_ENTER_MS         0x0105U
#define CFG_ID_VEH_EXIT_MS          0x0106U

/* --- 0x0107~0x0109 钥匙(ACC)去抖：E-3（2026-11-14）纳入配置表 ---
 * 默认 500/500/50；改后重启生效（Q-84）。
 * 依据：docs/02-设计/裁定-整车对象状态模型与E系列-V1.0.md §5.3
 * 注意：0x0107~0x010F 已实测空闲；0x0C00~0x0C3C 实际被 cl0~cl3 PID 占用，勿用。 */
#define CFG_ID_ACC_ENTER_MS         0x0107U   /* 钥匙上电确认去抖 */
#define CFG_ID_ACC_EXIT_MS          0x0108U   /* 钥匙下电确认去抖（E-3 的 500ms）*/
#define CFG_ID_ACC_POLL_MS          0x0109U   /* 钥匙轮询周期，须等于 50ms tick */

/* --- 0x010A 整车故障去抖：Q-B（2026-11-14）-----------------
 * 依据 SWR-N-002「传感器故障 2s 确认 / 2s 恢复」。
 * 0x010A~0x010F 实测空闲；0x010B~ 继续预留。 */
#define CFG_ID_VEH_FAULT_MS         0x010AU   /* 故障信号连续满足时长，默认 2000ms */

/* --- 0x010B CAN 总线静默门限：Q-D2（2026-11-14）---------------
 * 判定源：解析线程查表心跳（见 ut_can_scheduler.c can_is_bus_silent）。
 * 默认 500ms = 接收侧最慢周期帧(100ms) 的 5 倍余量。
 * 依据：docs/02-设计/方案-整车对象落地与全链路整理-V1.0.md §1.6 Q-D2 */
#define CFG_ID_VEH_CAN_SILENT_MS    0x010BU   /* 无任一报文触发查表的时长，默认 500ms */

/* --- 0x02xx 测速 --- */
#define CFG_ID_SPEED_WINDOW_TICKS   0x0201U
#define CFG_ID_SPEED_TICK_HZ        0x0202U
#define CFG_ID_SPEED_DT_MAX_RATIO   0x0203U

/* --- 0x03xx 看门狗 --- */
#define CFG_ID_WDT_CHECKIN_ENABLE   0x0301U
#define CFG_ID_WDT_TIMEOUT_MS       0x0302U
#define CFG_ID_WDT_IDLE_HOOK        0x0303U

/* --- 0x04xx 电流采样 --- */
#define CFG_ID_CUR_ZERO_BUS         0x0401U
#define CFG_ID_CUR_ZERO_W           0x0402U
#define CFG_ID_CUR_ZERO_V           0x0403U
#define CFG_ID_CUR_LSB_NV           0x0404U   /* 纳伏，805660 = 0.80566 mV */
#define CFG_ID_CUR_GAIN_X100        0x0405U

/* --- 0x05xx PWM / 电机 ---
 *
 * 【2026-11-04 · 0x0504/0x0505/0x0506 已废弃】
 * 这三项是 P1 阶段"先定义占位"留下的，存在三个问题：
 *   ① 默认值 0 —— 一旦接入即等于关闭过流保护，是安全隐患
 *   ② 单位 mA —— 与 motor_oc_cfg_t 的 _a10（0.1A）差 10 倍
 *   ③ 缺延时参数 —— 而堵转判据的核心恰恰是"阈值 + 持续时间"
 * 按 sys_config.c 顶部规则「不得修改已有 ID 的含义或范围」，
 * 此处保留 ID 不复用、不改含义，仅标注废弃；
 * 堵转保护改用 0x09xx 段（见下）。这三项从未被任何业务消费，
 * 也从未落盘，保留只是为了避免 ID 被日后误重新分配。 */
#define CFG_ID_PWM_FREQ_HZ          0x0501U
#define CFG_ID_PWM_DEADTIME_NS      0x0502U
#define CFG_ID_MOTOR_POLE_PAIRS     0x0503U
#define CFG_ID_MOTOR_OC_L1_MA       0x0504U   /* @deprecated 未接入，用 0x0905 */
#define CFG_ID_MOTOR_OC_L2_MA       0x0505U   /* @deprecated 未接入，用 0x0903 */
#define CFG_ID_MOTOR_OC_L3_MA       0x0506U   /* @deprecated 未接入，用 0x0901 */
#define CFG_ID_MOTOR_RAMP_MS        0x0507U
#define CFG_ID_FOC_MAX_MODULATION   0x0508U

/* --- 0x06xx 举升 --- */
#define CFG_ID_LIFT_UPPER_X10      0x0601U
#define CFG_ID_LIFT_LOWER_X10      0x0602U

/* --- 0x07xx 标定标志（镜像 sys_param，只读） --- */
#define CFG_ID_ENC_ZERO_VALID       0x0701U
#define CFG_ID_LIFT_CALIB_VALID    0x0702U
#define CFG_ID_CALIB_COUNT          0x0703U
#define CFG_ID_CALIB_LAST_TICK      0x0704U

/* --- 0x08xx 齿数镜像（镜像 mech_cfg，只读） --- */
#define CFG_ID_MECH_TEETH_Z0        0x0801U
#define CFG_ID_MECH_TEETH_Z1        0x0802U
#define CFG_ID_MECH_TEETH_Z2        0x0803U
#define CFG_ID_MECH_ACTIVE_IDX      0x0804U

/* --- 0x09xx 堵转保护（2026-11-04 新增） ---
 *
 * 判据：四条 OR，任一满足即触发堵转保护
 *   ① 电流 >= 55.0A 且持续 10ms
 *   ② 电流 >= 50.0A 且持续 100ms
 *   ③ 电流 >= 45.0A 且持续 500ms
 *   ④ 电机轴转速 < 36rpm 且持续 2.5s
 *
 * 触发后：停止输出 + 电磁锁 + FAULT + 键盘 BLOCKED，
 *         用户松开手柄后全自动清除。
 *
 * 【单位统一用 _a10（0.1A）】
 *   与 motor_oc_cfg_t / board_motor_cfg.h 完全一致，零换算，
 *   避免"两套单位"这类静默错误（0x0504 的教训）。
 *
 * 【为什么这些项不加 CFG_F_REQ_RESTART】
 *   堵转判据属于调参频繁的参数，需要在线改完立即生效。
 *   motor_stall 模块在每次 update 时读当前的 cfg 快照，
 *   t_cfg set 后由 sys_config_apply 触发 reload 即可。
 */
#define CFG_ID_STALL_OC3_A10        0x0901U   /* 55.0A 阈值 */
#define CFG_ID_STALL_OC3_MS         0x0902U   /* 55A 持续时长 */
#define CFG_ID_STALL_OC2_A10        0x0903U   /* 50.0A 阈值 */
#define CFG_ID_STALL_OC2_MS         0x0904U   /* 50A 持续时长 */
#define CFG_ID_STALL_OC1_A10        0x0905U   /* 45.0A 阈值 */
#define CFG_ID_STALL_OC1_MS         0x0906U   /* 45A 持续时长 */
#define CFG_ID_STALL_LOW_RPM        0x0907U   /* 低速阈值（电机轴） */
#define CFG_ID_STALL_LOW_MS         0x0908U   /* 低速持续时长 */
#define CFG_ID_STALL_EN_MIN_A10     0x0909U   /* 转速判据启用门控：最小电流 */
#define CFG_ID_MOTOR_RATED_A10      0x090AU   /* 额定电流（37A） */
#define CFG_ID_STALL_AUTO_CLEAR_MS  0x090BU   /* 松开后清除延时，0=立即 */

/* --- 0x0Axx 机械传动选择（2026-11-04 新增）---
 *
 * 【为什么要三个独立项，而不是一个"机型码"】
 *   卸粮筒传动链有三段**互相正交**的减速环节：
 *     ① 编码器齿轮组齿数比 —— 决定游标解算（41:42:43 / 55:56:57）
 *     ② 电机内部减速比     —— 电机本体减速箱（136.6 / 216）
 *     ③ 电机外部减速比     —— 机型齿比（65:13 / 75:14 / 53:11）
 *   三者无耦合：同一台电机可装不同机型，同一机型可配不同电机。
 *   若合成一个"机型码"，2×2×3=12 组合就要手工枚举 12 个码值，
 *   将来加一个齿型就变成 18 个，且无法表达"这个组合不存在"。
 *   拆成三项后，组合数由 mech_cfg 的候选表统一裁决，配置侧永远只有 3 个字段。
 *
 * 【为什么全部带 CFG_F_REQ_RESTART】
 *   任一项变化都会改变 i_total → 改变 cnt↔角度换算系数 → 旧零点作废。
 *   上电时 mech_cfg_store_init 读这三项合成候选 id，
 *   与 Flash 里已落盘的行比对：不同则褪零（C-08）并重写。
 *   运行中改这三项不会立即生效，避免"远端一条帧清掉现场标定"。
 *
 * 【取值语义与 mech_cfg.h 的 MECH_*_SEL_xxx 严格一一对应】
 *   两处若漂移，上电会选到错误的候选。故此处注释里写明映射，
 *   并在 mech_cfg_store.c 用 switch 显式转换而非强转下标。 */
#define CFG_ID_MECH_ENC_SEL         0x0A01U   /* 0=41:42:43  1=55:56:57 */
#define CFG_ID_MECH_IIN_SEL         0x0A02U   /* 0=136.6     1=216 */
#define CFG_ID_MECH_IOUT_SEL        0x0A03U   /* 0=65:13(13T) 1=75:14(14T) 2=53:11(11T) */

/* ============================================================
 * 【2026-11-09 说明：电流零点复用 0x04xx 段，不新增 0x0B 段】
 * ------------------------------------------------------------
 * 本工程早已在 0x04xx 段定义了完整的电流标定项：
 *   0x0401 cur_zero_bus   （默认 2048，范围 1500~2600）
 *   0x0402 cur_zero_w
 *   0x0403 cur_zero_v
 *   0x0404 cur_lsb_nv     （0.80566mV = 805660 nV）
 *   0x0405 cur_gain_x100  （26.4mV/A x100 = 2640）
 *
 * 零点校准功能直接消费这三个 0x04xx 零点项即可，
 * **不要另建 0x0Bxx 段** —— 那会造成两套零点并存，
 * 一旦两处不一致，排障时分不清哪个生效。
 *
 * 原计划的 0x0B01~0x0B04 已撤销。校准状态改为复用
 * 零点值本身表达：三路都等于默认 2048 即视为"未校准"。
 * ============================================================ */

/* ============================================================
 * 0x0B 段：机械传动概念与方向极性（2026-11-09 新增）
 * ------------------------------------------------------------
 * 【传动链三级定义（用户 2026-11-09 裁定，勿再混用）】
 *
 *   电机轴 ──(① i_in 电机内部减速比)──> 电机总成输出轴
 *          ──(② i_out 电机外部减速比)──> 卸粮筒
 *
 *   ① i_in  = 136.6 或 216      （CFG_ID_MECH_IIN_SEL）
 *   ② i_out = 65/13、75/14、53/11（CFG_ID_MECH_IOUT_SEL）
 *
 *   ⚠ 转速 12±2 r/min 指的是 **电机总成输出轴**，不是卸粮筒！
 *     卸粮筒转速 = (12±2) / i_out ≈ 2.24~2.49 r/min（13T 时约 2.40）。
 *     计算零位/限位检测带宽、停车滑行距离时**必须用卸粮筒转速**。
 *     2026-11-09 曾误用 12r/min 直接当筒速，导致结论全部偏差 5 倍。
 *
 *   ⚠ 编码器齿轮组（55:56:57）装在电机轴上，测得的是电机轴转角。
 *     角度换算已由 mech_cfg 的 i_total = i_in × i_out 一并覆盖，
 *     故 swing_angle 拿到的 current_position 已是卸粮筒角度。
 * ============================================================ */
#define CFG_ID_SWING_DIR_POLARITY   0x0B01U   /* 0=正逻辑 1=负逻辑 */

/* ---- 方向极性取值 ---- */
/* 正逻辑（0，默认，用户 2026-11-09 指定为"正逻辑"）：
 *   左转（SWING_LEFT / MOVE_LEFT）  → 角度减小
 *   右转（SWING_RIGHT / MOVE_RIGHT）→ 角度增大
 * 负逻辑（1）：
 *   左转 → 角度增大，右转 → 角度减小
 *
 * 【为什么要做成配置项】
 *   极性正确性取决于两处硬件，代码无法自证：
 *     ① 电机相序接线（MOTOR_DIR_CW 物理上是顺/逆时针）
 *     ② 编码器齿轮安装方向（encoder_raw 增大对应哪个转向）
 *   任一相反都会导致"左转角度反而增大"。
 *   把它显式化后，现场只需"试一下，反了就改 0/1"。
 *
 * 【本项只影响指令方向映射，不改变角度计算】
 *   角度始终 = encoder_raw - zero_offset，不做符号翻转。
 *   这样"当前角度"物理含义保持唯一，便于现场用示波器核对。
 * ============================================================ */

/* ============================================================
 * 8. API
 * ============================================================ */

/**
 * @brief 初始化：先加载默认值，再合并 Flash（逐项校验）
 * @return 0 成功（含"Flash 无效全部用默认"的情形）
 * @note  不阻断启动。任何 Flash 异常都退化为默认值。
 */
int sys_config_init(void);

/**
 * @brief 把 RAM 当前值落盘（不区分 dirty，整表写入）
 * @return 0 成功；<0 失败（失败时 dirty 保留，不假装成功）
 */
int sys_config_save(void);

/**
 * @brief 从 Flash 重新加载（丢弃未保存的 RAM 修改）
 */
int sys_config_load(void);

/**
 * @brief 恢复默认值
 * @param id  0 = 全部；其它 = 指定 ID
 * @return 0 成功
 */
int sys_config_restore_default(uint16_t id);

/**
 * @brief 擦除整块（等价全部回默认），需调用方二次确认
 */
int sys_config_erase(void);

/* ---- 读写 ---- */
int32_t sys_config_get_u32(uint16_t id, uint32_t *out);
int32_t sys_config_get_i32(uint16_t id, int32_t *out);

/**
 * @brief 设置一项（只改 RAM，标记 dirty，不落盘）
 * @return 0 成功；<0 失败（ID 不存在/只读/越界/flags 不满足）
 * @note  失败原因通过 rt_kprintf 打印具体 ID 与原因，不静默
 */
int32_t sys_config_set_i32(uint16_t id, int32_t val);

/* ---- 诊断 ---- */
uint8_t     sys_config_is_dirty(void);
uint32_t    sys_config_dirty_count(void);
cfg_src_t   sys_config_get_src(uint16_t id);
const cfg_def_t *sys_config_find_def(uint16_t id);
const cfg_def_t *sys_config_find_def_by_name(const char *name);
const char *sys_config_type_name(uint8_t type);
const char *sys_config_src_name(cfg_src_t s);
void        sys_config_dump_stats(void);

/* ---- 内部访问（供 sys_config_cmd.c 遍历，避免暴露静态数组）---- */
uint16_t          sys_config_item_count(void);
const cfg_item_t *sys_config_item_at(uint16_t idx);
uint8_t           sys_config_item_is_dirty(uint16_t idx);
int32_t           sys_config_set_raw(uint16_t id, int32_t val);

/**
 * @brief 刷新只读镜像项（标定标志/齿数比）的当前真值
 * @note  真值分别在 sys_param 与 mech_cfg，本表不存副本。
 *        dump / save 前调用，保证显示与实物一致。
 */
void sys_config_refresh_mirrors(void);

#ifdef __cplusplus
}
#endif

#endif /* __SYS_CONFIG_H */