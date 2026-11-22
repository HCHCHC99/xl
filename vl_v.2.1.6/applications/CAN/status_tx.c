/**
 * @file    status_tx.c
 * @brief   0x18FF11D1 控制器状态反馈帧组包实现
 * @note
 *   本机发送帧，方向：UTMotVCU → HMI，周期 100ms。
 *   所有字段通过 CAN_EVT_STA_* 事件刷新，pack 只做 memcpy，
 *   保证发送线程上下文不访问其他模块内部对象。
 */

#include "status_tx.h"
#include "protocol_defs.h"
#include "hw_objects.h"
#include "can_tx_cfg.h"          /* CAN_TX_EN_STATUS 模块级开关 */
#include "../sys_param/sys_param.h"
#include "../service/sys_voltage.h"   /* sys_voltage_is_uv()（§10.5-E）*/

#include <string.h>
#include <rtthread.h>

/* ============================================================
 * 本地缓存（事件写入，pack 读出）
 * ============================================================ */
static frame_0x18FF11D1_t s_tx;

/* ============================================================
 * 默认值：零位角度 + 全部正常
 * ============================================================ */
static void status_tx_reset_default(void)
{
    memset(s_tx.raw, 0, sizeof(s_tx.raw));

    /* 角度写零位（物理 0°），而非 0 原始值（那样是 -300° / -90°） */
    s_tx.bits.swing_angle_raw = (int16_t)STATUS_ANGLE_SWING_OFFSET;
    s_tx.bits.lift_angle_raw  = (uint8_t)STATUS_ANGLE_LIFT_OFFSET;

    s_tx.bits.power_voltage_sta = STA_POWER_NORMAL;
    s_tx.bits.calib_sta         = STA_CALIB_NONE;
    s_tx.bits.comm_sta          = 0U;
    s_tx.bits.work_mode         = STA_MODE_STANDBY;

    s_tx.bits.encoder_sta       = 0U;
    s_tx.bits.motor_sta         = STA_MOTOR_STOP;
    s_tx.bits.lift_sensor_sta   = 0U;
    s_tx.bits.lift_sta          = STA_LIFT_STOP;

    s_tx.bits.up_valve_sta      = STA_VALVE_NORMAL;
    s_tx.bits.down_valve_sta    = STA_VALVE_NORMAL;

    s_tx.bits.teeth_set_sta     = 0U;
    s_tx.bits.teeth_code        = 0U;
}

/* ============================================================
 * 事件入口
 * ============================================================ */
void status_tx_on_event(app_event_t event)
{
    switch (event) {

    /* ----- Byte0 ----- */
    case CAN_EVT_STA_POWER:
        /* 【§10.5-E】母线欠压状态由 sys_voltage 状态机维护（SYS_EVT_UV_* 事件
         * 触发本事件刷新）。00=正常 / 01=欠压；02=过压预留。 */
        s_tx.bits.power_voltage_sta = (sys_voltage_is_uv() != 0U)
                                      ? STA_POWER_UNDER : STA_POWER_NORMAL;
        break;

    case CAN_EVT_STA_CALIB:
        /* 【P0 修正 2026-10-27】未校准必须上报 UNDONE(10)，不能是 NONE(00)。
         * REQ010 §①：手柄按 calib_sta 决定控制器指示灯 —— 已校准绿常亮、
         * 未校准黄常亮。上报 00「无」时手柄无法区分，黄灯永不亮，校准入口断链。 */
        s_tx.bits.calib_sta = (g_sys_params.is_calibrated != 0U)
                              ? STA_CALIB_DONE : STA_CALIB_UNDONE;
        break;

    case CAN_EVT_STA_COMM:
        /* TODO(N-1)：CAN 通信超时监测（CAN_EVT_TIMEOUT）就绪后填充 */
        s_tx.bits.comm_sta = 0U;
        break;

    case CAN_EVT_STA_MODE:
        /* TODO(N-1)：mode 模块 getter 就绪后替换 */
        s_tx.bits.work_mode = STA_MODE_STANDBY;
        break;

    /* ----- Byte1 ----- */
    case CAN_EVT_STA_MOTOR:
        /* TODO(N-1)：motor 编码器故障 + swing 状态机 getter 就绪后替换 */
        s_tx.bits.encoder_sta = 0U;
        s_tx.bits.motor_sta   = STA_MOTOR_STOP;
        break;

    case CAN_EVT_STA_LIFT_SEN:
        /* TODO(N-1)：lift_sensor 故障标志就绪后替换 */
        s_tx.bits.lift_sensor_sta = 0U;
        break;

    case CAN_EVT_STA_LIFT:
        /* TODO(N-1)：lift 状态机 getter 就绪后替换 */
        s_tx.bits.lift_sta = STA_LIFT_STOP;
        break;

    /* ----- Byte2 ----- */
    case CAN_EVT_STA_VALVE:
        s_tx.bits.up_valve_sta   = g_solenoid.up_fault;
        s_tx.bits.down_valve_sta = g_solenoid.down_fault;
        break;

    /* ----- Byte3 ----- */
    case CAN_EVT_STA_TEETH:
        s_tx.bits.teeth_set_sta = (g_sys_params.teeth_type != TEETH_UNSET) ? 1U : 0U;
        /* Byte3 bit1-2 机型代码反馈（2 bit，依 SRS 表4-1）：
         *   0 = 13齿 / 1 = 14齿 / 2 = 11齿
         * 未设置时按 0（13齿）占位，由 bit0 区分"未设置"。 */
        switch (g_sys_params.teeth_type) {
        case TEETH_14: s_tx.bits.teeth_code = 1U; break;
        case TEETH_11: s_tx.bits.teeth_code = 2U; break;
        case TEETH_13:
        default:       s_tx.bits.teeth_code = 0U; break;
        }
        break;

    /* ----- Byte5-7：角度 ----- */
    case CAN_EVT_STA_ANGLE:
        /* TODO(N-1)：角度 getter 就绪后改为真实值
         *   int32_t ha = swing_angle_get_value();   // 0.1°
         *   int32_t pa = lift_angle_get_value();  // 0.1°
         *   s_tx.bits.swing_angle_raw = (int16_t)(ha / 10 + STATUS_ANGLE_SWING_OFFSET);
         *   s_tx.bits.lift_angle_raw  = (uint8_t)(pa / 10 + STATUS_ANGLE_LIFT_OFFSET);
         */
        break;

    default:
        break;
    }
}

/* ============================================================
 * 组包（can_tx 发送线程调用）
 * ============================================================ */
void status_tx_pack(can_tx_frame_t *frame)
{
    if (frame == RT_NULL) {
        return;
    }
    memcpy(frame->data, s_tx.raw, 8);
}

/* ============================================================
 * 注册
 * ============================================================ */
static const can_tx_entry_t s_status_entry = {
    .name      = "status",
    .id        = STATUS_FRAME_ID,
    .dlc       = 8U,
    .period_ms = STATUS_FRAME_PERIOD_MS,
    .phase_ms  = STATUS_FRAME_PHASE_MS,
    .trig      = CAN_TX_TRIG_PERIODIC,
    .pack      = status_tx_pack,
};

int status_tx_init(void)
{
    /* 【can_tx_cfg.h】模块级开关：0 = 完全不注册（不占 slot、不调 pack）。
     * 返回 0 而非错误：boot 表把本步标为 BOOT_FATAL，返回非 0 会中断启动。
     * 「关闭」是正常状态，不是故障。 */
#if !CAN_TX_EN_STATUS
    rt_kprintf("[CAN-TX] status frame DISABLED by can_tx_cfg.h\n");
    return 0;
#else
    status_tx_reset_default();

    /* 【P0 修正 2026-10-27】上电即建立静态字段的真实值。
     * 原实现只有 reset_default()，而 CAN_EVT_STA_CALIB / STA_POWER / STA_COMM /
     * STA_MODE 四个事件全工程零投递 ⇒ 这四个字段永远停在 reset 的初值。
     * 其中 calib_sta 初值 STA_CALIB_NONE(00)，手柄据此判断"无校准状态"，
     * REQ010 §① 的「未校准→控制器指示灯黄常亮」永不成立，校准入口断链。
     * 这里直接按真实数据源同步一次；运行期变化由对应模块 post 事件刷新。 */
    status_tx_on_event(CAN_EVT_STA_CALIB);
    status_tx_on_event(CAN_EVT_STA_POWER);
    status_tx_on_event(CAN_EVT_STA_COMM);
    status_tx_on_event(CAN_EVT_STA_MODE);

    return can_tx_register(&s_status_entry);
#endif
}
