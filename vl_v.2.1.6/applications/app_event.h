/**
 * @file    app_event.h
 * @brief   应用事件统一枚举定义
 * @note    所有模块事件在此集中定义，按模块分组
 *          事件值分配规则：每个模块占用 0x1000 区间
 *          - 系统事件:    0x0000 ~ 0x0FFF
 *          - 垂直控制:    0x1000 ~ 0x1FFF
 *          - 回转控制:    0x2000 ~ 0x2FFF
 *          - 校准控制:    0x3000 ~ 0x3FFF
 *          - CAN通信:     0x4000 ~ 0x4FFF
 *          - 离合控制:    0x5000 ~ 0x5FFF
 *          - 预留扩展:    0x6000 ~ 0xFFFF
 */

#ifndef __APP_EVENT_H
#define __APP_EVENT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 1. 事件基础值（BASE宏集中定义）
 *    每模块预留 4096 个事件 (0x1000)
 * ============================================================ */
#define SYS_EVENT_BASE      0x0000
#define LIFT_EVENT_BASE    0x1000
#define SWING_EVENT_BASE     0x2000
#define CALIB_EVENT_BASE    0x3000
#define CAN_EVENT_BASE      0x4000
#define CLUTCH_EVENT_BASE   0x5000
#define OTA_EVENT_BASE      0x6000
#define SEQ_EVENT_BASE      0x7000
#define MODE_EVENT_BASE     0x8000
#define VEHICLE_EVENT_BASE  0x9000      /* 整车状态对象（2026-11-12 对象化）*/

/* 公共事件类型（所有事件统一为 uint32_t） */
typedef uint32_t app_event_t;

/* ============================================================
 * 2. 系统事件 (0x0000 ~ 0x0FFF)
 * ============================================================ */
typedef enum {
    EVT_TICK_50MS = SYS_EVENT_BASE,      /* 0x0000 50ms 周期 Tick */
    EVT_FAULT_TRIGGERED,                 /* 0x0001 故障触发 */
    EVT_FAULT_CLEARED,                   /* 0x0002 故障清除 */
    EVT_SYSTEM_RESET,                    /* 0x0003 系统复位 */
    EVT_SYSTEM_STARTUP,                  /* 0x0004 系统启动完成 */
    SYS_EVT_TEETH_CLEARED,               /* 0x0005 机型设置被清除
                                          *        受 sys_param.teeth_clear_action 控制，
                                          *        需停止输出时由业务层响应 */
    SYS_EVT_VEHICLE_STARTED,             /* 0x0006 启车状态确认（未启车→已启车） */
    SYS_EVT_VEHICLE_STOPPED,             /* 0x0007 熄车状态确认（已启车→未启车） */
    SYS_EVT_ACC_ON,                      /* 0x0008 钥匙上电（PB0 低有效，500ms 去抖）
                                          *        【Q33 独立】与 VEHICLE_STARTED 是两个
                                          *        独立状态，互不参与对方判定 */
    SYS_EVT_ACC_OFF,                     /* 0x0009 钥匙下电（PB0 高，500ms 去抖） */
    SYS_EVT_UV_ENTER   = 0x000AU,        /* 0x000A 母线欠压进入（sys_voltage 状态机
                                          *        抖动确认后投递，业务层做急停+存档）*/
    SYS_EVT_UV_RECOVER = 0x000BU,        /* 0x000B 母线欠压恢复（迟滞回正常）*/
} sys_event_t;

/* ============================================================
 * 3. 举升角控制事件 (0x1000 ~ 0x1FFF)
 * ============================================================ */
typedef enum {
    /* --- 手动操作事件 --- */
    LIFT_EVT_UP_DOWN = LIFT_EVENT_BASE,  /* 上升按键按下 */
    LIFT_EVT_UP_UP,                      /* 上升按键松开 */
    LIFT_EVT_DOWN_DOWN,                  /* 下降按键按下 */
    LIFT_EVT_DOWN_UP,                    /* 下降按键松开 */

    /* --- 限位事件 --- */
    LIFT_EVT_LIMIT_REACHED,              /* 限位到达（通用） */
    LIFT_EVT_LIMIT_REACHED_UPPER,        /* 上限位到达 (+43°) */
    LIFT_EVT_LIMIT_REACHED_LOWER,        /* 下限位到达 (-30°) */
    LIFT_EVT_LIMIT_CLEARED,              /* 离开限位（回到中间位） */

    /* --- 超时事件 --- */
    LIFT_EVT_TIMEOUT_5S,                 /* 5秒延时到 */

    /* --- 故障事件 --- */
    LIFT_EVT_VALVE_FAULT,                /* 电磁阀故障 */
    LIFT_EVT_VALVE_FAULT_CLEAR,          /* 电磁阀故障清除 */
    LIFT_EVT_SENSOR_FAULT,               /* 角度传感器故障 */
    LIFT_EVT_SENSOR_FAULT_CLEAR,         /* 角度传感器故障清除 */

    /* 举升序列协同事件（补充到 vert_event_t） */
    LIFT_EVT_SEQ_UP,
    LIFT_EVT_SEQ_DOWN,
    LIFT_EVT_SEQ_STOP,
} vert_event_t;

/* ============================================================
 * 4. 回转控制事件 (0x2000 ~ 0x2FFF)
 * ============================================================ */
typedef enum {
    /* --- 手动操作事件 --- */
    SWING_EVT_LEFT_DOWN = SWING_EVENT_BASE,   /* 左转按键按下 */
    SWING_EVT_LEFT_UP,                       /* 左转按键松开 */
    SWING_EVT_RIGHT_DOWN,                    /* 右转按键按下 */
    SWING_EVT_RIGHT_UP,                      /* 右转按键松开 */

    /* --- 一键序列事件 --- */
    SWING_EVT_ONKEY_MIDDLE,                  /* 一键右中 */
    SWING_EVT_ONKEY_REAR,                    /* 一键右后 */
    SWING_EVT_ONKEY_HOME,                    /* 一键回位 */

    /* --- 序列内部事件 --- */
    SWING_EVT_SEQ_STEP_COMPLETE,             /* 序列步骤完成 */
    SWING_EVT_SEQ_TIMEOUT,                   /* 序列超时 */
    SWING_EVT_SEQ_ABORT,                     /* 序列中止 */

    /* --- 角度/限位事件 --- */
    SWING_EVT_ANGLE_ARRIVED,                 /* 到达目标角度 */
    SWING_EVT_LIMIT_LEFT_REACHED,            /* 左限位到达 (-100°) */
    SWING_EVT_LIMIT_RIGHT_REACHED,           /* 右限位到达 (+215°) */
    SWING_EVT_LIMIT_CLEARED,                 /* 离开限位 */
    SWING_EVT_ZERO_REACHED,                  /* 到达零位 */

    /* 故障 */
    SWING_EVT_ENCODER_FAULT,
    SWING_EVT_ENCODER_FAULT_CLEAR,

    /* 回转序列协同事件 */
    SWING_EVT_SEQ_MOVE_LEFT,
    SWING_EVT_SEQ_MOVE_RIGHT,
    SWING_EVT_SEQ_STOP,

    /* 刹车盘事件 */
    SWING_EVT_BRAKE_UNLOCKED_LEFT,     /* 解锁完成，切左转 */
    SWING_EVT_BRAKE_UNLOCKED_RIGHT,    /* 解锁完成，切右转 */
    SWING_EVT_BRAKE_LOCKED,            /* 锁止完成 */
    SWING_EVT_BRAKE_FAULT,             /* 刹车盘故障 */
} swing_event_t;

/* ============================================================
 * 5. 校准控制事件 (0x3000 ~ 0x3FFF)
 * ============================================================ */
typedef enum {
    /* --- 按键事件 --- */
    CALIB_EVT_CLICK = CALIB_EVENT_BASE,     /* 单次点击（校准按键按下） */
    CALIB_EVT_DOUBLE_CLICK,                 /* 双击（间隔 ≤ 1s） */
    CALIB_EVT_LONG_PRESS,                   /* 长按（> 2s） */
    CALIB_EVT_KEY_UP,                       /* 校准按键松开 */

    /* --- 校准流程事件 --- */
    CALIB_EVT_READY,                        /* 进入准备态（黄灯5s） */
    CALIB_EVT_RUNNING,                      /* 正在校准中 */
    CALIB_EVT_SUCCESS,                      /* 校准成功 */
    CALIB_EVT_FAIL,                         /* 校准失败 */
    CALIB_EVT_EXIT,                         /* 退出校准 */
} calib_event_t;

/* ============================================================
 * 6. 序列控制事件 (0x7000 ~ 0x7FFF)
 * ============================================================ */
typedef enum {
    SEQ_EVT_ONKEY_MIDDLE = SEQ_EVENT_BASE,
    SEQ_EVT_ONKEY_REAR,
    SEQ_EVT_ONKEY_HOME,
    SEQ_EVT_STEP_COMPLETE,
    SEQ_EVT_TIMEOUT,
    SEQ_EVT_ABORT,

    /* ---- Q-F3 路 A / 2026-11-14 E1：序列**对象级**状态机内部事件 ----
     * 与上面 0x7000~0x7005 的面板/编排事件分开，留 0x10 间隔避免误用。
     * 这些事件只经 control_object_process(&g_seq_obj.base, ...) **同步投递**，
     * 不走 post_event，也不依赖 app_main 的区间路由（I-04 不影响）。
     * 【引擎非周期调度】条件满足才投一次，引擎不自己查表。 */
    SEQ_OBJ_EVT_START  = SEQ_EVENT_BASE + 0x10U,   /* 0x7010 IDLE -> ACTIVE */
    SEQ_OBJ_EVT_FINISH = SEQ_EVENT_BASE + 0x11U,   /* 0x7011 ACTIVE -> IDLE（完成/中止）*/
    SEQ_OBJ_EVT_ABORT  = SEQ_EVENT_BASE + 0x12U,   /* 0x7012 ACTIVE -> IDLE（按键/故障中断）*/
} seq_event_t;

/* ============================================================
 * 7.模式相关事件 (0x8000 ~ 0x8FFF)
 * ============================================================ */
typedef enum {
    EVT_MODE_CHANGED = MODE_EVENT_BASE,     /* 模式变更（广播） */
    MODE_EVT_ENTER_FACTORY,                 /* 进入工厂模式 */
    MODE_EVT_ENTER_TEST,                    /* 进入测试模式 */
    MODE_EVT_ENTER_WORK,                    /* 进入工作模式 */
    MODE_EVT_TEST_TIMEOUT,                  /* 测试模式超时 */
    MODE_EVT_CONFIG_CHANGED,                /* 模式配置变更 */
} mode_event_t;

/* ============================================================
 * 8. CAN通信事件 (0x4000 ~ 0x4FFF)
 * ============================================================
 * 区间划分：
 *   0x4000 ~ 0x400F  通信层事件（收/发/总线错误）
 *   0x4010 ~ 0x401F  0x18FF11D1 状态帧字段变更（本机发送数据源）
 *   0x4020 ~ 0x402F  0x18FFA4F0 灯帧字段变更（本机发送数据源）
 *
 * 说明：事件只表达「哪个字段变了」，字段的新值由各模块维护的
 *       状态对象提供，status_tx / led_tx 收到事件后刷新本地缓存共用体。
 * ============================================================ */
typedef enum {
    /* --- 通信层 --- */
    CAN_EVT_RX_FRAME = CAN_EVENT_BASE,      /* 0x4000 收到CAN帧 */
    CAN_EVT_TX_COMPLETE,                    /* 0x4001 发送完成 */
    CAN_EVT_BUS_ERROR,                      /* 0x4002 CAN总线错误 */
    CAN_EVT_OVERFLOW,                       /* 0x4003 接收溢出 */
    CAN_EVT_TIMEOUT,                        /* 0x4004 通信超时 */

    /* --- 0x18FF11D1 状态帧数据源（0x4010~0x401F） --- */
    CAN_EVT_STA_POWER     = CAN_EVENT_BASE + 0x10,  /* 0x4010 电源电压状态 */
    CAN_EVT_STA_CALIB     = CAN_EVENT_BASE + 0x11,  /* 0x4011 控制器校准状态 */
    CAN_EVT_STA_COMM      = CAN_EVENT_BASE + 0x12,  /* 0x4012 控制器通信状态 */
    CAN_EVT_STA_MODE      = CAN_EVENT_BASE + 0x13,  /* 0x4013 控制器工作模式 */
    CAN_EVT_STA_MOTOR     = CAN_EVENT_BASE + 0x14,  /* 0x4014 电机编码器/工作状态 */
    CAN_EVT_STA_LIFT_SEN  = CAN_EVENT_BASE + 0x15,  /* 0x4015 升降传感器状态 */
    CAN_EVT_STA_LIFT      = CAN_EVENT_BASE + 0x16,  /* 0x4016 卸粮筒升降状态 */
    CAN_EVT_STA_VALVE     = CAN_EVENT_BASE + 0x17,  /* 0x4017 电磁阀故障状态 */
    CAN_EVT_STA_TEETH     = CAN_EVENT_BASE + 0x18,  /* 0x4018 机型设置状态 */
    CAN_EVT_STA_ANGLE     = CAN_EVENT_BASE + 0x19,  /* 0x4019 回转/举升角度刷新 */

    /* --- 0x18FFA4F0 灯帧数据源（0x4020~0x402F） --- */
    CAN_EVT_LED_KEY       = CAN_EVENT_BASE + 0x20,  /* 0x4020 按键灯 */
    CAN_EVT_LED_IND       = CAN_EVENT_BASE + 0x21,  /* 0x4021 指示灯 */
    CAN_EVT_LED_BACKLIGHT = CAN_EVENT_BASE + 0x22,  /* 0x4022 背景灯 */
} can_event_t;

/* ============================================================
 * 9. 离合控制事件 (0x5000 ~ 0x5FFF)
 * ============================================================ */
typedef enum {
    CLUTCH_EVT_COMBINE = CLUTCH_EVENT_BASE,     /* 结合指令 */
    CLUTCH_EVT_SEPARATE,                        /* 分离指令 */
    CLUTCH_EVT_COMBINE_COMPLETE,                /* 结合完成 */
    CLUTCH_EVT_SEPARATE_COMPLETE,               /* 分离完成 */
    CLUTCH_EVT_FAULT,                           /* 离合故障 */
    CLUTCH_EVT_FAULT_CLEAR,                     /* 离合故障清除 */
} clutch_event_t;

/* ============================================================
 * 10. OTA 事件 (0x6000 ~ 0x6FFF)
 * ============================================================ */
typedef enum {
    OTA_EVT_START = OTA_EVENT_BASE,
    OTA_EVT_PROGRESS,
    OTA_EVT_COMPLETE,
    OTA_EVT_FAILED,
    OTA_EVT_ROLLBACK,
} ota_event_t;

/* ============================================================
 * 12. 整车状态事件 (0x9000 ~ 0x9FFF)
 *     【2026-11-12】整车状态对象化（PLAN-VEH-OBJ-V1.0）
 *
 *     说明：这些是**状态机内部驱动事件**，与 SYS_EVT_VEHICLE_STARTED/STOPPED
 *     不同 —— 后者是对外广播的"结果通知"，本组是内部"过程推进"。
 *     Event_t 为 uint32_t 不能带参数，故"条件是否满足/计时到否"由
 *     vehicle_poll() 在上下文里算好后，选择发 POLL 还是 CONFIRM。
 * ============================================================ */
typedef enum {
    VEH_EVT_POLL = VEHICLE_EVENT_BASE,   /* 0x9000 条件成立，进入过渡态（开始计时）*/
    VEH_EVT_CONFIRM,                     /* 0x9001 去抖计时到，确认跳变 */
    VEH_EVT_BACK,                        /* 0x9002 条件消失，从过渡态回退到稳定态 */
    VEH_EVT_FORCE_RESET,                 /* 0x9003 强制复位到 OFF（调试）*/

    /* ---- 2026-11-14 P2：四态单机新增（Q-A / Q-B / Q-C）----
     * 旧 POLL/CONFIRM/BACK 三事件中的 POLL/BACK 在四态模型下不再使用，
     * 保留枚举值仅为不改动 0x9000~0x9003 的既有编号；CONFIRM 复用为「启车确认」。
     * 注意：VEH_EVT_* 是状态机内部**同步**驱动事件，禁止改成 post_event
     *       （0x9000 段在 app_main 无路由分支，异步投递会被静默丢弃，V4.0 §8.3）。 */
    VEH_EVT_ACC_ON,                      /* 0x9004 钥匙上电确认 → OFF→ACC_ON */
    VEH_EVT_ACC_OFF,                     /* 0x9005 钥匙下电确认 → 任意态→OFF（B-1 唯一清除途径）*/
    VEH_EVT_FAULT,                       /* 0x9006 故障 2s 确认 → 任意工作态→FAULT（Q-B）*/
} vehicle_event_t;

/* ============================================================
 * 13. 事件投递接口
 * ============================================================ */

/**
 * @brief 投递事件到消息队列
 * @param event 事件值（app_event_t 类型，实际为 uint32_t）
 * @note  线程安全，可在中断或线程上下文调用
 */
void post_event(app_event_t event);

#ifdef __cplusplus
}
#endif

#endif /* __APP_EVENT_H */
