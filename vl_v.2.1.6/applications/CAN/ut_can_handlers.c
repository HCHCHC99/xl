/**
 * @file    ut_can_handlers.c
 * @brief   CAN 接收回调函数实现
 * @note    解析一律使用 protocol_defs.h 的共用体位域，
 *          禁止手写位移（避免协议与代码脱节）。
 */

#include "ut_can_config.h"
#include "ut_can_handlers.h"
#include "protocol_defs.h"
#include "app_event.h"
#include "hw_objects.h"
#include "../storage/storage.h"
#include "framework/state_engine/sm_assert.h"
#include <string.h>
#include "../sys_param/sys_param.h"
#include "../swing/swing_control.h"
#include "../lift/lift_control.h"
#include "../sequence/seq_control.h"
#include "../keyboard/keyboard_sm.h"
#include "../motor/motor.h"
#include "../motor/motor_config.h"
#include "ut_can_send.h"
#include <finsh.h>          /* MSH_CMD_EXPORT_ALIAS（t_canrx） */

/* ----- 外部事件投递接口（在 app_event.c 中实现） ----- */
extern void post_event(app_event_t evt);

/* ============================================================
 * 接收调试开关（M-6）
 * ============================================================
 * 编译期 CAN_RX_DEBUG：1 = 保留打印代码；0 = 整段裁掉（省 Flash）。
 * 运行期 t_canrx：默认 **关**，需要看真实总线时 `t_canrx 1`。
 *
 * 【为什么默认关】接收周期 10~100ms，全打印会刷屏，把 msh 交互冲掉
 * （2026-11-20 实测：t_sim 注入 50ms 转速帧 → 20 行/秒）。
 *
 * 【模拟注入帧永不打印】t_sim 的帧走 can_sim_inject()，置 s_rx_sim 标记，
 * 打印宏直接跳过 —— 否则 t_sim key hold 之类会持续刷屏。
 * ============================================================ */
#define CAN_RX_DEBUG        1

static volatile uint8_t s_rx_dump_en = 0U;   /* 运行期开关，默认关 */
static volatile uint8_t s_rx_sim     = 0U;   /* 1 = 当前正在分发模拟注入帧 */

#if CAN_RX_DEBUG
static rt_uint8_t can_dlc_to_bytes(rt_uint32_t dlc)
{
    static const rt_uint8_t dlc_len[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};

    return dlc_len[dlc & 0x0FU];
}

/* 打印整帧原始字节 */
static void can_printf(const can_frame_t *rxmsg)
{
    rt_uint8_t data_len;
    rt_uint8_t i;

    data_len = can_dlc_to_bytes(rxmsg->len);
    rt_kprintf("ID:%X Data:", rxmsg->id);
    for (i = 0; i < data_len; i++) {
        rt_kprintf("%02X ", rxmsg->data[i]);
    }
    rt_kprintf("\n");
}
#define CAN_RX_DUMP(f)      do { if ((s_rx_dump_en != 0U) && (s_rx_sim == 0U)) { can_printf(f); } } while (0)
#define CAN_RX_LOG(...)     do { if ((s_rx_dump_en != 0U) && (s_rx_sim == 0U)) { rt_kprintf(__VA_ARGS__); } } while (0)
#else
#define CAN_RX_DUMP(f)      ((void)0)
#define CAN_RX_LOG(...)     ((void)0)
#endif /* CAN_RX_DEBUG */

/* ============================================================
 * 发动机转速有效性判定
 * ============================================================ */
#define ENGINE_RPM_TIMEOUT_MS   100U    /* 报文周期 10ms 的 10 倍冗余 */

/* ============================================================
 * 面板按键处理
 * ============================================================
 * 按键解析、组合键判定、互斥仲裁、超时释放、指令生成
 * 已全部移交 keyboard 模块（applications/keyboard/）。
 * 本文件只负责把原始报文转交过去。
 * ============================================================ */


/* ============================================================
 * 回调函数：处理 0x18FF8380（8/10键开关面板）
 * ============================================================
 * 协议：每键 2 bit，00=未按下 / 01=按下 / 10~11=预留
 * 位分配见 protocol_defs.h frame_0x18FF8380_t
 * 判定条件与整改前一致：归一化为 0/1 后做边沿比对
 * ============================================================ */
static void hdl_panel_switch(const can_frame_t *frame)
{
    SM_ASSERT_PTR(frame);
    SM_ASSERT_MSG(frame->len >= 4, "Panel switch frame too short");

//    CAN_RX_DUMP(frame);

    /* 按键解析 / 组合键判定 / 互斥仲裁 / 超时释放 / 指令生成 / 事件投递
     * 全部由 keyboard 模块负责，本处只做原始报文转交。 */
    kb_on_can_frame(frame->data, frame->len);
}

/* ============================================================
 * 回调函数：处理 0x18FF0E63（电磁阀反馈）
 * ============================================================
 * 协议：Byte1 bit0-1 上升阀故障 / bit2-3 下降阀故障
 *       Byte3 bit4   上升阀开关 / bit5   下降阀开关
 * ============================================================ */
static void hdl_solenoid_feedback(const can_frame_t *frame)
{
    const solenoid_fb_t *p;

    SM_ASSERT_PTR(frame);
    SM_ASSERT_MSG(frame->len >= 4, "Solenoid feedback frame length too short");

    p = (const solenoid_fb_t *)frame->data;

    uint8_t up_fb      = p->bits.up_open;
    uint8_t down_fb    = p->bits.down_open;
    uint8_t up_fault   = p->bits.up_fault;
    uint8_t down_fault = p->bits.down_fault;

    g_solenoid.valve_feedback = (uint8_t)((up_fb << 0) | (down_fb << 1));
    g_solenoid.up_fault       = up_fault;
    g_solenoid.down_fault     = down_fault;

    /* 同步到举升控制上下文：
     *   故障码（非 0 即故障）→ 触发 FAULT_VALVE 状态，立即关阀
     *   开关反馈           → 供 0x18FF11D1 Byte2 / 诊断使用 */
    lift_control_update_valve_fault((up_fault   != 0U) ? 1U : 0U,
                                     (down_fault != 0U) ? 1U : 0U);
    lift_control_update_valve_fb(up_fb, down_fb);

    /* 通知 0x18FF11D1 Byte2 电磁阀状态更新 */
    post_event(CAN_EVT_STA_VALVE);

    /* ---- 调试信息 ---- */
    CAN_RX_DUMP(frame);
    CAN_RX_LOG("[CAN]0E63 UpFb%d DnFb%d UpErr%d DnErr%d\n",
               up_fb, down_fb, up_fault, down_fault);

    /* 更新垂直控制故障状态（待 lift 侧接口就绪后启用）
     * lift_control_update_valve_fault(up_fault != 0, down_fault != 0);
     * lift_control_update_valve_fb(up_fb, down_fb);
     */
}

extern int sys_param_save(void);
/* ============================================================
 * 回调函数：处理 0x18FF7F18（机型设置）
 * ============================================================
 * 协议：Byte0 bit0-1  00 清除 / 01 13齿 / 10 14齿 / 11 11齿
 *
 * 已确认的处理策略：
 *   Q1 收到即覆盖（不做"仅 EEP 为空才接受"的判断）
 *   Q2 可能周期重发 → 值去重（sys_param_set_teeth + storage 双层）
 *   Q3 写入后从 Flash 回读核对，核对通过才算应答；
 *      无写入动作时维持上次读取/初始化参数，照常应答
 *   Q4 收到清除指令后，按 sys_param.teeth_clear_action 决定是否停止输出
 *   Q5 11 齿反馈：按 SRS 表4-1，11D1 Byte3 bit1-2 为 2bit，
 *      13齿→0 / 14齿→1 / 11齿→2
 * ============================================================ */
static void hdl_teeth_setting(const can_frame_t *frame)
{
    const teeth_setting_t *p;
    uint8_t type;
    uint8_t target;

    SM_ASSERT_PTR(frame);
    SM_ASSERT_MSG(frame->len >= 1, "Teeth setting frame length too short");

    p    = (const teeth_setting_t *)frame->data;
    type = p->bits.teeth_set;

    CAN_RX_DUMP(frame);
    CAN_RX_LOG("[CAN]7F18 teeth_set=%u\n", type);

    /* ---- 协议值 → 内部枚举 ---- */
    switch (type) {
    case 0:  target = TEETH_UNSET; break;
    case 1:  target = TEETH_13;    break;
    case 2:  target = TEETH_14;    break;
    case 3:  target = TEETH_11;    break;
    default: return;                     /* 2bit 字段不可能越界 */
    }

    /* ---- Q1 + Q2：收到即覆盖，但值未变化时不产生写入动作 ---- */
    if (g_sys_params.teeth_type != target) {

        g_sys_params.teeth_type = target;

        /* ---- Q3：写入 → 回读核对 ---- */
        if (sys_param_save() == RT_EOK) {
            if (sys_param_load() == RT_EOK) {
                if (g_sys_params.teeth_type == target) {
                    CAN_RX_LOG("[CAN]7F18 write+verify OK -> %u\n", target);
                } else {
                    /* Flash 内容与预期不符：以回读值为准，视为写入失败 */
                    CAN_RX_LOG("[CAN]7F18 verify MISMATCH, keep %u\n",
                               g_sys_params.teeth_type);
                }
            } else {
                CAN_RX_LOG("[CAN]7F18 reload FAILED, keep %u\n",
                           g_sys_params.teeth_type);
            }
        } else {
            CAN_RX_LOG("[CAN]7F18 save FAILED, keep %u\n",
                       g_sys_params.teeth_type);
            (void)sys_param_load();      /* 回滚到上次落盘值 */
        }
    } else {
        /* 无写入动作：维持上次读取/初始化参数，直接应答 */
        CAN_RX_LOG("[CAN]7F18 unchanged (%u), keep\n", target);
    }

    /* ---- Q4：清除机型设置后的行为按配置决定 ---- */
    if ((target == TEETH_UNSET) &&
        (g_sys_params.teeth_clear_action == TEETH_CLEAR_STOP)) {
        post_event(SYS_EVT_TEETH_CLEARED);
    }

    /* ---- 应答：刷新 0x18FF11D1 Byte3（bit0 设置状态 / bit1 机型代码）---- */
    post_event(CAN_EVT_STA_TEETH);
}

/* ============================================================
 * 回调函数：处理 0x0CF00400（发动机转速，补充协议）
 * ============================================================
 * 方向：整车 ECU → 本机
 * Byte3-4 无符号 16 位（小端），精度 0.125 rpm/LSB
 * 换算：rpm = raw >> 3
 *
 * 备注：0x18FF11D1 为本机【发送】帧，不再挂接收路由（PR-07）；
 *       其角度字段由本机传感器提供，不依赖 CAN 接收。
 * ============================================================ */
static uint16_t  s_engine_rpm      = 0;
static rt_tick_t s_engine_rpm_tick = 0;
static uint8_t   s_engine_rpm_seen = 0U;

static void hdl_engine_rpm(const can_frame_t *frame)
{
    const engine_rpm_t *p;
    uint16_t rpm;

    SM_ASSERT_PTR(frame);
    SM_ASSERT_MSG(frame->len >= 5, "Engine rpm frame too short");

    p   = (const engine_rpm_t *)frame->data;
    rpm = ENGINE_RPM_OF(p->bits.engine_rpm_raw);

    s_engine_rpm      = rpm;
    s_engine_rpm_tick = rt_tick_get();
    s_engine_rpm_seen = 1U;

    /* ---- 调试信息 ---- */
    CAN_RX_DUMP(frame);
    CAN_RX_LOG("[CAN]0CF00400 rpm_raw=%u rpm=%u\n",
               (unsigned)p->bits.engine_rpm_raw, (unsigned)rpm);
}

/**
 * @brief 读取最近一次发动机转速（带超时失效）
 * @return rpm；0 表示「未收到过」或「已超过 100ms 未更新」
 * @note  报文周期 10ms，超时取 100ms（10 倍冗余）。
 *        整车 CAN 断线后转速归零，避免启车判定被残留值误导。
 */
uint16_t can_get_engine_rpm(void)
{
    if (s_engine_rpm_seen == 0U) {
        return 0U;
    }
    if ((rt_tick_get() - s_engine_rpm_tick) >
        rt_tick_from_millisecond(ENGINE_RPM_TIMEOUT_MS)) {
        return 0U;
    }
    return s_engine_rpm;
}

/* 【Q-B / 2026-11-14】发动机转速帧超时只读镜像。
 * ⚠️ 2026-11-14 16:49 起**不再作为 Q-B 的故障判据**：用户 Q-D1 裁定改用
 *    ut_can_scheduler.c 的 can_is_bus_silent()（整条总线 / 解析线程查表心跳）。
 * 本函数保留供诊断与后续条件扩展（Q-D3「后续可能会添加其他条件」）复用，不删除（R-05）。 */
uint8_t can_is_engine_rpm_timeout(void)
{
    if (s_engine_rpm_seen == 0U) {
        return 0U;
    }
    if ((rt_tick_get() - s_engine_rpm_tick) >
        rt_tick_from_millisecond(ENGINE_RPM_TIMEOUT_MS)) {
        return 1U;
    }
    return 0U;
}


/* ============================================================
 * 路由配置表单
 * ============================================================
 * 只登记【接收】方向的报文。
 * 本机发送帧（0x18FF11D1 / 0x18FFA4F0 / 0x18FF1D60）不得在此出现。
 * ============================================================ */
const can_route_entry_t can_route_table[] = {
    {0x18FF8380, 0x1FFFFFFF, hdl_panel_switch},        /* 开关面板 → 本机 */
    {0x18FF0E63, 0x1FFFFFFF, hdl_solenoid_feedback},   /* 电磁控制盒 → 本机 */
    {0x18FF7F18, 0x1FFFFFFF, hdl_teeth_setting},       /* HMI → 本机 */
    {0x0CF00400, 0x1FFFFFFF, hdl_engine_rpm},          /* 整车 ECU → 本机 */
};
const uint32_t can_route_table_size = sizeof(can_route_table) / sizeof(can_route_entry_t);

/* ============================================================
 * 台架注入（任务清单 §11）
 * ------------------------------------------------------------
 * 不经过硬件滤波器 / ringbuffer / 解析线程，直接按同一张路由表分发，
 * 因此回调内部逻辑与真实收帧完全一致（含互斥仲裁、去重、事件投递）。
 * ============================================================ */
#if (INPUT_SIM_ENABLE && CAN_SIM_ENABLE)
void can_sim_inject(uint32_t id, const uint8_t *data, uint8_t len)
{
    can_frame_t f;
    uint32_t i;

    if ((data == RT_NULL) || (len == 0U) || (len > 8U)) {
        return;
    }

    memset(&f, 0, sizeof(f));
    f.id  = id;
    f.len = len;
    memcpy(f.data, data, len);

    s_rx_sim = 1U;          /* 注入帧不参与 CAN_RX_DUMP/LOG（防刷屏） */
    for (i = 0U; i < can_route_table_size; i++) {
        if ((f.id & can_route_table[i].mask) ==
            (can_route_table[i].id & can_route_table[i].mask)) {
            if (can_route_table[i].callback != RT_NULL) {
                can_route_table[i].callback(&f);
            }
            break;
        }
    }
    s_rx_sim = 0U;
}
#endif /* INPUT_SIM_ENABLE && CAN_SIM_ENABLE */

/* ============================================================
 * t_canrx — 接收帧打印运行期开关（默认关）
 * ============================================================
 * t_canrx        → 查看当前状态
 * t_canrx 1      → 打开（看真实总线的原始字节 + 解析结果）
 * t_canrx 0      → 关闭
 * 模拟注入帧（t_sim）无论开关如何都不打印。
 * ============================================================ */
static void t_canrx(int argc, char **argv)
{
    if (argc < 2) {
        rt_kprintf("[CANRX] rx dump = %u (t_canrx 1 to enable)\n",
                   (unsigned)s_rx_dump_en);
        return;
    }

    if ((argv[1][0] == '1') && (argv[1][1] == '\0')) {
        s_rx_dump_en = 1U;
        rt_kprintf("[CANRX] rx dump ON (real bus only)\n");
    } else if ((argv[1][0] == '0') && (argv[1][1] == '\0')) {
        s_rx_dump_en = 0U;
        rt_kprintf("[CANRX] rx dump OFF\n");
    } else {
        rt_kprintf("[CANRX] usage: t_canrx 0|1\n");
    }
}

MSH_CMD_EXPORT_ALIAS(t_canrx, t_canrx, CAN rx frame dump on/off);
