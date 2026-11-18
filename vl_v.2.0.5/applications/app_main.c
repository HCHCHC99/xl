/**
 * @file    app_main.c
 * @brief   前台主线程：事件驱动，处理核心业务
 */

#include <rtthread.h>
#include "app_event.h"
#include "ipc_center.h"
#include "framework/state_engine/state_engine.h"

#include "lift/lift_angle.h"
#include "lift/lift_control.h"
#include "lift/lift_actions.h"
#include "lift/lift_sensor.h"

#include "swing/swing_angle.h"
#include "swing/swing_control.h"

#include "swing/swing_actions.h"
#include "mode/mode_control.h"
#include "mode/mode_actions.h"

/* CAN 状态上报发送帧缓存（事件驱动数据源） */
#include "CAN/status_tx.h"
#include "CAN/led_tx.h"
#include "CAN/ut_can_handlers.h"

/* 启车状态判定 */
#include "service/vehicle_state.h"

/* 钥匙上电状态（PB0，Q33 独立于启车判定） */
#include "service/key_state.h"

/* 序列引擎（需 seq_control_set_vehicle_started） */
#include "sequence/seq_control.h"

#include "framework/state_engine/sm_assert.h"

/* 2026-11-15 C2：描述表路由 + 能力门禁（LEG-3 / G-07 / Q-C4） */
#include "framework/core/obj_desc.h"
#include "framework/manager/ctrl_manager.h"
#include "mode/sys_caps.h"

/* 全局管理器实例：沿用工程惯例在各使用点 extern（见 boot_table.c / lift_init.c） */
extern ctrl_manager_t g_ctrl_mgr;

/* ============================================================
 * 启车门禁判定
 * ============================================================
 * 依据 SRS：
 *   REQ003/004/005/006（手动左/右/上/下）→ 前置条件仅为「车辆上电」
 *   REQ007/008/009（一键右中/右后/回位）→ 前置条件为「车辆处于启车状态」
 *   REQ011/012（离合结合/分离）        → 前置条件为「车辆处于启车状态」
 *
 * 因此只能按事件枚举精确拦截，不能按区间：
 *   SWING_EVT_ONKEY_* 与 SWING_EVT_LEFT/RIGHT_* 同在 0x2000 区间。
 * ============================================================ */
static uint8_t evt_needs_vehicle_started(app_event_t e)
{
    switch (e) {
    case SWING_EVT_ONKEY_MIDDLE:
    case SWING_EVT_ONKEY_REAR:
    case SWING_EVT_ONKEY_HOME:
        return 1U;

    case SEQ_EVT_ONKEY_MIDDLE:
    case SEQ_EVT_ONKEY_REAR:
    case SEQ_EVT_ONKEY_HOME:
        return 1U;

    case CLUTCH_EVT_COMBINE:
    case CLUTCH_EVT_SEPARATE:
        return 1U;

    default:
        return 0U;      /* 手动上下/左右：只需上电，放行 */
    }
}

/* ============================================================
 * 门 1 门禁判定（Q-C / D-P1，2026-11-14 随 P2 补）
 * ============================================================
 * 依据：需求基线 §3.3 REQ-PWR-03「acc_is_on()==0 时所有动作指令（含手动四向）被拒绝」
 *       + §4.5 Q-C「去抖窗口内动作放行，确认下电后禁动作」。
 *
 * 【Q-C 怎么成立】钥匙下电去抖由 key_state 模块负责：500ms 窗口内
 *   acc_is_on() 仍返回 1 ⇒ 本门禁不拦 ⇒ 动作放行（TEMP-E5 临时偏差）；
 *   去抖确认后才返回 0 ⇒ 拦下动作指令。无需在门禁里另做计时。
 *
 * ⚠️ 安全铁律：**限制启动，绝不能限制停止**。
 *    本函数只列「按下 / 触发」类事件；松开、限位、故障、序列内部推进
 *    一律返回 0 放行，否则钥匙一掉电，正在动的筒会停不下来。
 * ============================================================ */
static uint8_t evt_is_motion_start(app_event_t e)
{
    switch (e) {
    /* 手动四向：只拦「按下」，松开必须放行 */
    case LIFT_EVT_UP_DOWN:
    case LIFT_EVT_DOWN_DOWN:
    case SWING_EVT_LEFT_DOWN:
    case SWING_EVT_RIGHT_DOWN:

    /* 一键（SWING 与 SEQ 两套枚举） */
    case SWING_EVT_ONKEY_MIDDLE:
    case SWING_EVT_ONKEY_REAR:
    case SWING_EVT_ONKEY_HOME:
    case SEQ_EVT_ONKEY_MIDDLE:
    case SEQ_EVT_ONKEY_REAR:
    case SEQ_EVT_ONKEY_HOME:

    /* 离合 */
    case CLUTCH_EVT_COMBINE:
    case CLUTCH_EVT_SEPARATE:
        return 1U;

    default:
        return 0U;   /* 松开 / 限位 / 故障 / 序列内部推进 / 系统事件：放行 */
    }
}

/* 外部举升角对象 */
extern lift_angle_t g_lift_angle;
//extern StateMachine_t g_lift_sm;

/**
 * @brief 前台主线程入口
 */
void app_main_thread_entry(void *param)
{
    app_event_t event;

    rt_kprintf("[app_main] Started, waiting for events...\n\n");

    while (1) {
        /* 阻塞等待事件（无事件时挂起） */
        rt_mq_recv(g_app_event_mq, &event, sizeof(app_event_t), RT_WAITING_FOREVER);

        /* ----- 50ms Tick 处理 ----- */
        if (event == EVT_TICK_50MS) {
            /* 更新举升角对象：读取电压 → 计算角度 → 检测位置变化 → 投递事件 */
            lift_angle_update(&g_lift_angle);

            /* 更新 回转角对象：spi获取3颗编码器信息 → 计算角度 → 检测位置变化 → 投递事件 */
            swing_angle_update(&g_swing_angle);

            /* 2. 刹车到位上报
                                       *    必须显式调用：StateFuncTable_t 只有 state/enter/exit 三个字段
                                       *    （state_engine.h:46-50），没有 tick 回调入口 */
            swing_act_tick_check(&g_swing_obj.ctx);
            lift_act_tick_check(&g_lift_obj.ctx);
            /* 3. 模式超时检测（同样无法挂进函数表） */
            mode_tick_check(&g_mode_obj.ctx);

            /* 4. 角度已刷新 → 通知 0x18FF11D1 状态帧更新缓存 */
            post_event(CAN_EVT_STA_ANGLE);

            /* 5. 角度传感器故障 → 举升控制上下文
             *    感知层不依赖业务层，由调度层在此搬运 */
            if (g_lift_angle.sensor != RT_NULL) {
                lift_control_update_sensor_fault(
                    lift_sensor_get_fault(g_lift_angle.sensor));
            }

            /* 6. 启车状态判定：50ms 轮询 + 迟滞 */
            vehicle_poll();

            /* 7. 钥匙上电状态：PB0 低有效 + 500ms 去抖
             *    【Q33 独立】与启车判定无耦合，各自独立维护状态 */
            acc_poll();

            /* 阀故障 500ms 去抖 */
//            lift_control_valve_fb_poll();

//            if (++s_tick_div >= 4U) {             /* 200ms 状态帧 */
//                s_tick_div = 0U;
//                valve_tx_apply();
//            }
            continue;   /* tick 不投给状态机：自转移在 state_engine.c:106 直接 return，投了也是空转 */


            /* 序列引擎周期更新 */
//            extern void seq_control_tick_update(void);   /* 或直接调用 */
//            seq_control_update();   /* 需将函数暴露 */
            /* 可在此添加其他 50ms 周期任务 */
        }

        /* ----- 系统事件：启车 / 熄车状态跳变 ----- */
        if (event == SYS_EVT_VEHICLE_STARTED) {
            /* 【F4 修复 2026-11-12】原代码只 post_event 未同步序列引擎，
             * 导致 seq_control.c:43 的 ctx->vehicle_started 恒 0
             * ⇒ 一键序列即使被唤醒也被拒（门禁读 vehicle_is_started() 另一个变量）。 */
            seq_control_set_vehicle_started(1U);
            rt_kprintf("[app_main] vehicle STARTED -> output enabled\n");
            post_event(CAN_EVT_LED_IND);        /* 通知灯帧刷新 */
            continue;
        }
        if (event == SYS_EVT_VEHICLE_STOPPED) {
            /* 【2026-11-14 P2 / D-4】整车对象已不再发此事件（REQ-PWR-06）。
             * 本分支保留仅为兼容，正常路径的序列清除已移到 SYS_EVT_ACC_OFF。 */
            seq_control_set_vehicle_started(0U);    /* 【F4 修复 2026-11-12】同步熄车 */
            rt_kprintf("[app_main] vehicle STOPPED -> output disabled\n");
            post_event(CAN_EVT_LED_IND);
            continue;
        }

        /* ----- 系统事件：钥匙上电 / 下电（Q33 独立状态） -----
         * 【2026-11-14 P2 / Q-C】下电确认后：① 序列侧清启车副本
         * ② 门 1 门禁统一在下方 evt_is_motion_start() 拦截动作指令。
         * 【D-4 / REQ-PWR-06】整车对象不再发 SYS_EVT_VEHICLE_STOPPED，
         * 因此原来由该事件承担的 seq_control_set_vehicle_started(0)
         * 必须移到这里，否则钥匙下电后序列引擎仍误以为已启车。 */
        if (event == SYS_EVT_ACC_ON) {
            rt_kprintf("[app_main] ignition key ON\n");
            post_event(CAN_EVT_LED_IND);
            continue;
        }
        if (event == SYS_EVT_ACC_OFF) {
            rt_kprintf("[app_main] ignition key OFF -> gate1 closed, seq cleared\n");
            seq_control_set_vehicle_started(0U);   /* D-4：替代 STOPPED 事件的清除职责 */
            post_event(CAN_EVT_LED_IND);
            continue;
        }

        /* ----- 系统事件：机型设置被清除（Q4 可配置行为） ----- */
        if (event == SYS_EVT_TEETH_CLEARED) {
            /* 由 sys_param.teeth_clear_action 触发（TEETH_CLEAR_STOP）
             * TODO(Q4)：按业务需要停止输出
             *   post_event(SWING_EVT_SEQ_ABORT);
             *   post_event(LIFT_EVT_SEQ_STOP);
             *   或调用对应模块 stop 接口 */
            rt_kprintf("[app_main] teeth cleared -> stop output\n");
            continue;
        }

        /* ----- CAN 状态上报事件（0x4010 ~ 0x402F）-----
         * 只刷新本机发送帧的本地缓存，实际发帧由 can_tx 周期调度。
         * 区间与 app_event.h can_event_t 定义保持一致。
         */
        if (event >= (CAN_EVENT_BASE + 0x10) && event <= (CAN_EVENT_BASE + 0x2F)) {
            status_tx_on_event(event);
            led_tx_on_event(event);
            continue;
        }

        /* ----- 门 1 门禁（Q-C / D-P1）：钥匙下电确认后禁止一切动作指令 -----
         * 放在启车门禁之前；停止类事件已在 evt_is_motion_start() 中排除。
         * FAULT 态下车门 1 仍成立 ⇒ 手动四向放行、一键由下方门 2 拒绝
         * ⇒ 自然实现 Q-B ①「仅手动」。 */
        if ((acc_is_on() == 0U) && (evt_is_motion_start(event) != 0U)) {
            rt_kprintf("[app_main] drop evt 0x%X: ACC off (gate 1)\n",
                       (unsigned)event);
            continue;
        }

        /* ----- 启车门禁：一键/离合需启车状态，手动四向放行（SRS） ----- */
        if ((vehicle_is_started() == 0U) &&
            (evt_needs_vehicle_started(event) != 0U)) {
            rt_kprintf("[app_main] drop evt 0x%X: vehicle not started\n",
                       (unsigned)event);
            continue;
        }

        /* ----- 事件路由（2026-11-15 C2 / LEG-3 / G-07）-----
         * 【Q-C4 按建议代裁定】先查描述表；命中后做能力门禁，再经
         * control_object_process() 专用投递接口（E-2）同步触发引擎一次。
         * 【引擎非周期调度 E-1】只有匹配到事件才查表/投递，不在此轮询。
         * 【零回归】未命中描述表的事件仍走下方原有区间分支：mode（②类）
         * 保持原路径；0x3000/0x5000/0x6000/0x7000 四段仍静默丢弃
         * （I-04，属 Q-C7 范围外，本批不改）。
         * 【门禁粒度】required_caps 是对象级粗粒度（MANUAL）；一键/校准
         * 等事件级能力仍由动作层 sys_mode_allows_* 判定，待 S2 落地后细化。 */
        {
            const ctrl_obj_desc_t *desc = obj_desc_find_by_event(event);

            if (desc != NULL) {
                control_object_t *target =
                    ctrl_manager_find_by_type(&g_ctrl_mgr,
                                              (ctrl_obj_type_t)desc->type);

                if ((sys_caps_get() & desc->required_caps) != desc->required_caps) {
                    rt_kprintf("[app_main] drop evt 0x%X: caps=0x%X need=0x%X\n",
                               (unsigned)event, (unsigned)sys_caps_get(),
                               (unsigned)desc->required_caps);
                } else if (target == NULL) {
                    rt_kprintf("[app_main] drop evt 0x%X: type=%u not registered\n",
                               (unsigned)event, (unsigned)desc->type);
                } else {
                    /* E-2：唯一入口，基类校验 base.sm / is_initialized 后投一次 */
                    control_object_process(target, event);
                }
                continue;   /* 描述表已命中，不再走下面的旧区间分支 */
            }
        }

        /* ----- 回退：未命中描述表时的原有区间路由（保持既有行为） -----
         * 【Q-F6 / 2026-11-14 E1 铁律 2 整改】原来 3 处直接调
         * StateMachine_SendEvent(&xxx.sm_instance, ...)，绕过 control_object
         * 聚合根。现统一改走**专用投递接口** control_object_process()
         * （= obj_base_process，obj_base.c:106），由基类校验
         * base.sm / is_initialized 后再同步触发引擎一次。 */
        if (event >= LIFT_EVENT_BASE && event < SWING_EVENT_BASE) {
            /* 举升 0x1000 ~ 0x1FFF */
            control_object_process(&g_lift_obj.base, event);
        } else if (event >= SWING_EVENT_BASE && event < CALIB_EVENT_BASE) {
            /* 回转 0x2000 ~ 0x2FFF（原先遗漏，导致左右手动完全不动） */
            control_object_process(&g_swing_obj.base, event);
        } else if (event >= MODE_EVENT_BASE && event <= (MODE_EVENT_BASE + 0x0FF)) {
            /* 【P0 修正 2026-10-27】模式事件 0x8000 ~ 0x80FF 原先无人接收。
             * mode_init() → mode_control_set_mode() → control_object_emit()
             *   实际是投递到 g_app_event_mq（ctrl_executor.c:24-33 single_emit），
             * 但 app_main 只路由 0x1000/0x2000 两个区间，模式事件被静默丢弃。
             * 现转发给模式状态机，使 MODE_EVT_ENTER_* 能真正驱动迁移。 */
            control_object_process(&g_mode_obj.base, event);
        }
        /* 校准 0x3000：待校准模块（独立）接入后转发 */

    }
}
