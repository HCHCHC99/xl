/**
 * @file    protocol_defs.h
 * @brief   卸粮筒 CAN 协议数据定义 —— 唯一数据源
 *
 * 【设计原则】
 *   1. 一个报文 = 一个共用体（union），`raw[8]` 与 `bits.*` 叠加同一段内存。
 *   2. 位域顺序直接对应协议 bit 定义，解析/组包不再手写位移运算。
 *   3. 每个报文注明方向、周期，避免把发送帧误挂到接收路由。
 *
 * 【字节序】
 *   目标芯片 HC32F452 为小端（Little-Endian），多字节字段
 *   （如 0x18FF11D1 的 int16 电机旋转角度）按小端布局，
 *   与协议表一致，无需额外转换。
 *
 * 【位域约定】
 *   ARM GCC 对 uint8_t 型位域从 LSB 开始分配，且不跨存储单元。
 *   因此按字节分组书写即可精确对应 ByteN bitM。
 *
 * 【协议基准】
 *   CHK-CAN-PROTO-V1.0（docs/02-设计/协议一致性核查表-V1.0.md）
 *   主基准：水稻机无刷电机控制协议(1).xlsx
 */

#ifndef __PROTOCOL_DEFS_H
#define __PROTOCOL_DEFS_H

#include <stdint.h>
#include <stddef.h>

/* -------------------- 字节序与对齐控制 -------------------- */
#pragma pack(push, 1)

/* ============================================================
 * 基础 CAN 帧结构（供接收/发送模块共用）
 * ============================================================ */
typedef struct {
    uint32_t id;
    uint8_t  len;
    uint8_t  data[8];
} can_frame_t;

/* ============================================================
 * 编译期布局校验（不依赖 RT-Thread，保持本头文件零 OS 依赖）
 * ============================================================ */
#define PROTO_ASSERT_SIZE(type, name)  typedef char proto_assert_##name[(sizeof(type) == 8) ? 1 : -1]
#define PROTO_ASSERT_OFF(type, member, off, name) \
    typedef char proto_assert_##name[(offsetof(type, member) == (off)) ? 1 : -1]


/* ================================================================
 * 1. 0x18FF8380 — 8/10 键开关面板输入（每键 2 bit）
 * ================================================================
 * 方向：开关面板 → UTMotVCU（本机【接收】）
 * 周期：100ms
 * 编码：00 = 未按下；01 = 按下；10~11 = 预留
 * ================================================================ */
typedef union {
    uint8_t raw[8];
    struct {
        /* ----- Byte0 ----- */
        uint8_t clutch_combine  : 2;   /* bit0-1 卸粮离合结合 */
        uint8_t clutch_separate : 2;   /* bit2-3 卸粮离合分离 */
        uint8_t reserved0       : 4;   /* bit4-7 预留 */

        /* ----- Byte1 ----- */
        uint8_t key_up          : 2;   /* bit0-1 上 */
        uint8_t key_down        : 2;   /* bit2-3 下 */
        uint8_t key_left        : 2;   /* bit4-5 左 */
        uint8_t key_right       : 2;   /* bit6-7 右 */

        /* ----- Byte2 ----- */
        uint8_t onekey_home     : 2;   /* bit0-1 一键转入（灯帧「一键回位」） */
        uint8_t key_calib       : 2;   /* bit2-3 系统校准 */
        uint8_t onekey_mid      : 2;   /* bit4-5 一键转出（右中） */
        uint8_t onekey_rear     : 2;   /* bit6-7 一键转出（右后） */

        /* ----- Byte3 ----- */
        uint8_t remote_type;           /* bit0-7 遥控类型：00无/01 8键/02 8键带离合/03 10键 */

        /* ----- Byte4-7 ----- */
        uint8_t reserved[4];
    } bits;
} frame_0x18FF8380_t;
typedef frame_0x18FF8380_t panel_switch_t;

/** 2bit 按键值 → 0/1（01=按下，其余=未按下；保持原有判定条件不变） */
#define PANEL_KEY_PRESSED(v)        (((v) == 0x01U) ? 1U : 0U)


/* ================================================================
 * 2. 0x18FF11D1 — 控制器状态反馈
 * ================================================================
 * 方向：UTMotVCU（本机【发送】）→ 显示屏 HMI
 * 周期：100ms
 * 角度：swing_angle_raw 原始值，物理值 = raw - 300（范围 -300~300）
 *       lift_angle_raw  原始值，物理值 = raw -  90（范围  -90~90）
 * ================================================================ */
typedef union {
    uint8_t raw[8];
    struct {
        /* ----- Byte0：控制器总体状态 ----- */
        uint8_t power_voltage_sta : 2;  /* bit0-1 00正常/01欠压/10过压/11预留 */
        uint8_t calib_sta         : 2;  /* bit2-3 00无/01已校准/10未校准/11预留 */
        uint8_t comm_sta          : 1;  /* bit4   0通信正常/1通信故障 */
        uint8_t work_mode         : 3;  /* bit5-7 000待机/001手动/010一键展开/011一键回位 */

        /* ----- Byte1：电机与升降 ----- */
        uint8_t encoder_sta       : 1;  /* bit0   0正常/1故障 */
        uint8_t motor_sta         : 3;  /* bit1-3 000停止/001左转/010右转/011开路/100堵转 */
        uint8_t lift_sensor_sta   : 1;  /* bit4   0正常/1故障 */
        uint8_t lift_sta          : 3;  /* bit5-7 000停止/001上升/010下降 */

        /* ----- Byte2：电磁阀状态 ----- */
        uint8_t up_valve_sta      : 2;  /* bit0-1 00正常/01开路/10短路/11预留 */
        uint8_t down_valve_sta    : 2;  /* bit2-3 同上 */
        uint8_t reserved2         : 4;

        /* ----- Byte3：机型 -----
         * 依据 SRS 表4-1（截图为准）：
         *   bit0   机型设置状态  0=未设置 / 1=已设置
         *   bit1-2 机型代码反馈  0=13齿 / 1=14齿 / 2=11齿
         * 注：早期 Excel 版本写为 bit1 单 bit，仅能表达 13/14 齿，
         *     已按 SRS 更正为 2 bit（可表达 11 齿）。 */
        uint8_t teeth_set_sta     : 1;  /* bit0    */
        uint8_t teeth_code        : 2;  /* bit1-2  */
        uint8_t reserved3         : 5;  /* bit3-7  */

        /* ----- Byte4 ----- */
        uint8_t reserved4;

        /* ----- Byte5-6：电机旋转角度（int16 小端） ----- */
        int16_t swing_angle_raw;

        /* ----- Byte7：卸粮筒升降角度（原始值 0~180） ----- */
        uint8_t lift_angle_raw;
    } bits;
} frame_0x18FF11D1_t;
typedef frame_0x18FF11D1_t status_frame_t;

/* 角度换算常量 */
#define STATUS_ANGLE_SWING_OFFSET       (300)   /* 回转角偏移 */
#define STATUS_ANGLE_LIFT_OFFSET       (90)    /* 举升角偏移 */


/* ================================================================
 * 3. 0x18FFA4F0 — 开关面板灯状态
 * ================================================================
 * 方向：UTMotVCU（本机【发送】）→ 开关面板
 * 周期：100ms
 * 每键 4 bit，字段内位义：bit0=绿 / bit1=黄 / bit2=红 / bit3=闪烁
 *（协议以 1-based 描述 bit1~bit4，映射到字段 bit0~bit3）
 * ================================================================ */
typedef union {
    uint8_t raw[8];
    struct {
        uint8_t key_up_led         : 4;   /* Byte0 bit0-3 按键1（上） */
        uint8_t key_down_led       : 4;   /* Byte0 bit4-7 按键2（下） */

        uint8_t key_left_led       : 4;   /* Byte1       按键3（左） */
        uint8_t key_right_led      : 4;   /* Byte1       按键4（右） */

        uint8_t key_home_led       : 4;   /* Byte2       按键5（一键回位） */
        uint8_t key_calib_led      : 4;   /* Byte2       按键6（校准） */

        uint8_t key_mid_led        : 4;   /* Byte3       按键7（一键右中） */
        uint8_t key_rear_led       : 4;   /* Byte3       按键8（一键右后） */

        uint8_t key_clutch_com_led : 4;   /* Byte4       按键9（离合结合） */
        uint8_t key_clutch_sep_led : 4;   /* Byte4       按键10（离合分离） */

        uint8_t reserved5;                /* Byte5       预留 */

        uint8_t ind_motor_led      : 4;   /* Byte6       指示灯1（电机） */
        uint8_t ind_ctrl_led       : 4;   /* Byte6       指示灯2（控制器） */

        uint8_t ind_sensor_led     : 4;   /* Byte7 bit0-3 指示灯3（传感器） */
        uint8_t backlight          : 1;   /* Byte7 bit4   手柄按键背景灯 */
        uint8_t reserved7          : 3;
    } bits;
} frame_0x18FFA4F0_t;
typedef frame_0x18FFA4F0_t led_frame_t;

/* 单键灯 4bit 位义 */
#define LED_BIT_GREEN               (0x01U)
#define LED_BIT_YELLOW              (0x02U)
#define LED_BIT_RED                 (0x04U)
#define LED_BIT_BLINK               (0x08U)


/* ================================================================
 * 4. 0x18FF0E63 — 电磁阀控制器故障反馈
 * ================================================================
 * 方向：电磁控制盒 → UTMotVCU（本机【接收】）
 * 周期：100ms
 * ================================================================ */
typedef union {
    uint8_t raw[8];
    struct {
        uint8_t byte0;

        /* ----- Byte1：故障码 ----- */
        uint8_t up_fault    : 2;   /* bit0-1 00正常/01开路/10短路 */
        uint8_t down_fault  : 2;   /* bit2-3 同上 */
        uint8_t reserved1   : 4;

        uint8_t byte2;

        /* ----- Byte3：开关状态 ----- */
        uint8_t reserved3a  : 4;   /* bit0-3 预留 */
        uint8_t up_open     : 1;   /* bit4   1打开/0关闭 */
        uint8_t down_open   : 1;   /* bit5   1打开/0关闭 */
        uint8_t reserved3b  : 2;   /* bit6-7 预留 */

        uint8_t reserved[4];
    } bits;
} frame_0x18FF0E63_t;
typedef frame_0x18FF0E63_t solenoid_fb_t;


/* ================================================================
 * 5. 0x18FF1D60 — 卸粮筒升降状态控制
 * ================================================================
 * 方向：UTMotVCU（本机【发送】）→ 电磁控制盒
 * 周期：200ms
 * 编码：00 或 01 = 不控制；10 = 无效；11 = 有效
 * 停止发 0x00（两路均「不控制」）
 * ================================================================ */
typedef union {
    uint8_t raw[8];
    struct {
        uint8_t byte0;
        uint8_t byte1;
        uint8_t byte2;

        /* ----- Byte3 ----- */
        uint8_t up_sw       : 2;   /* bit0-1 上升开关 */
        uint8_t down_sw     : 2;   /* bit2-3 下降开关 */
        uint8_t reserved3   : 4;

        uint8_t reserved[4];
    } bits;
} frame_0x18FF1D60_t;
typedef frame_0x18FF1D60_t valve_ctrl_t;

/* Byte3 取值 */
#define VALVE_CODE_NONE             (0x00U)   /* 不控制 */
#define VALVE_CODE_VALID            (0x03U)   /* 有效（11） */


/* ================================================================
 * 6. 0x18FF7F18 — 机型设置
 * ================================================================
 * 方向：HMI → UTMotVCU（本机【接收】，事件型）
 * 编码：00 清除 / 01 13齿 / 10 14齿 / 11 11齿
 * ================================================================ */
typedef union {
    uint8_t raw[8];
    struct {
        uint8_t teeth_set : 2;   /* Byte0 bit0-1 */
        uint8_t reserved0 : 6;
        uint8_t reserved[7];
    } bits;
} frame_0x18FF7F18_t;
typedef frame_0x18FF7F18_t teeth_setting_t;


/* ================================================================
 * 7. 0x0CF00400 — 发动机转速（补充协议）
 * ================================================================
 * 方向：整车 ECU → UTMotVCU（本机【接收】）
 * 周期：10ms
 * Byte3-4 无符号 16 位（小端），精度 0.125 rpm/LSB
 * 换算：rpm = raw >> 3（1/8，整数运算无误差）
 * ================================================================ */
typedef union {
    uint8_t raw[8];
    struct {
        uint8_t  byte0;
        uint8_t  byte1;
        uint8_t  byte2;
        uint16_t engine_rpm_raw;   /* Byte3-4 小端 */
        uint8_t  byte5;
        uint8_t  byte6;
        uint8_t  byte7;
    } bits;
} frame_0x0CF00400_t;
typedef frame_0x0CF00400_t engine_rpm_t;

/** 转速原始值 → rpm（精度 0.125） */
#define ENGINE_RPM_OF(raw)          ((uint16_t)((raw) >> 3))


#pragma pack(pop)

/* ================================================================
 * 编译期布局校验
 * ================================================================ */
PROTO_ASSERT_SIZE(frame_0x18FF8380_t, 8380_size);
PROTO_ASSERT_SIZE(frame_0x18FF11D1_t, 11D1_size);
PROTO_ASSERT_SIZE(frame_0x18FFA4F0_t, A4F0_size);
PROTO_ASSERT_SIZE(frame_0x18FF0E63_t, 0E63_size);
PROTO_ASSERT_SIZE(frame_0x18FF1D60_t, 1D60_size);
PROTO_ASSERT_SIZE(frame_0x18FF7F18_t, 7F18_size);
PROTO_ASSERT_SIZE(frame_0x0CF00400_t, 0CF_size);

/* 关键字段偏移：确认未因位域排布差异而错位 */
PROTO_ASSERT_OFF(frame_0x18FF8380_t, bits.remote_type,       3, 8380_remote);
PROTO_ASSERT_OFF(frame_0x18FF11D1_t, bits.swing_angle_raw,   5, 11D1_angle_swing);
PROTO_ASSERT_OFF(frame_0x18FF11D1_t, bits.lift_angle_raw,    7, 11D1_angle_lift);
PROTO_ASSERT_OFF(frame_0x0CF00400_t, bits.engine_rpm_raw,    3, 0CF_rpm);


/* ================================================================
 * 调试辅助：协议 ID 与类型名映射
 * ================================================================ */
typedef struct {
    uint32_t    id;
    const char *type_name;
    const char *direction;
} proto_id_map_t;

#define PROTO_ENTRY(TYPE, ID, DIR) { ID, #TYPE, DIR }
static const proto_id_map_t g_proto_id_map[] = {
    PROTO_ENTRY(panel_switch_t,   0x18FF8380, "RX 面板->本机"),
    PROTO_ENTRY(status_frame_t,   0x18FF11D1, "TX 本机->HMI"),
    PROTO_ENTRY(led_frame_t,      0x18FFA4F0, "TX 本机->面板"),
    PROTO_ENTRY(solenoid_fb_t,    0x18FF0E63, "RX 电磁盒->本机"),
    PROTO_ENTRY(valve_ctrl_t,     0x18FF1D60, "TX 本机->电磁盒"),
    PROTO_ENTRY(teeth_setting_t,  0x18FF7F18, "RX HMI->本机"),
    PROTO_ENTRY(engine_rpm_t,     0x0CF00400, "RX 整车ECU->本机"),
    {0, NULL, NULL}
};

#endif /* __PROTOCOL_DEFS_H */
