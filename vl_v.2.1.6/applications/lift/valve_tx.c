/**
 * @file    valve_tx.c
 * @brief   电磁阀控制帧组包实现
 * @note    报文方向：UTMotVCU（本机）→ 电磁控制盒，周期 200ms
 *          Byte3 bit0-1 = 上升阀, bit2-3 = 下降阀
 *          编码：00/01 = 不控制，10 = 无效（禁用），11 = 有效
 *          本实现只输出 0x00 与 0x03（字段值），绝不输出 0x02
 *          停止时两路均「不控制」→ Byte3 = 0x00
 */
#include "valve_tx.h"
#include "../CAN/can_tx_cfg.h"   /* CAN_TX_EN_VALVE 模块级开关 */
#include "../CAN/protocol_defs.h"
#include "hw_objects.h"
#include "framework/state_engine/sm_assert.h"
#include <string.h>
#include <rtthread.h>

#define VALVE_PERIOD_MS     200U
#define VALVE_PHASE_MS      0U

/* ================= 组包 ================= */
void valve_tx_pack(can_tx_frame_t *frame)
{
    valve_ctrl_t f;
    uint8_t vo;

    /* 【2026-10-27 · D-58】原为 SM_ASSERT_PTR(frame)，改为直接返回。
     * 本函数在 can_tx 线程每 200ms 执行一次，属热路径；
     * 而 frame 由 can_tx_poll 以 &f 传入，恒非 NULL。
     * 保留断言只会让正常路径为"不可能发生的分支"预留
     * rt_kprintf 的约 600 B 栈。 */
    if (frame == RT_NULL) {
        return;
    }

    memset(f.raw, 0, sizeof(f.raw));
    vo = g_solenoid.valve_output;

    /* 字段值直接取协议码：3 = 二进制的 11（有效），0 = 00（不控制） */
    f.bits.up_sw   = (vo & 0x01U) ? VALVE_CODE_VALID : VALVE_CODE_NONE;
    f.bits.down_sw = (vo & 0x02U) ? VALVE_CODE_VALID : VALVE_CODE_NONE;

    memcpy(frame->data, f.raw, 8);
}

/* ================= 注册项 ================= */
static const can_tx_entry_t s_valve_entry = {
    .name      = "valve_ctrl",
    .id        = VALVE_FRAME_ID,
    .dlc       = 8U,
    .period_ms = VALVE_PERIOD_MS,
    .phase_ms  = VALVE_PHASE_MS,
    .trig      = CAN_TX_TRIG_ONCHANGE,
    .pack      = valve_tx_pack,
};

/* ================= 变更即发 ================= */
void valve_tx_notify_change(void)
{
    can_tx_force(VALVE_FRAME_ID);
}

/* ================= 初始化 ================= */
int valve_tx_init(void)
{
#if !CAN_TX_EN_VALVE
    rt_kprintf("[CAN-TX] valve frame DISABLED by can_tx_cfg.h\n");
    return 0;
#else
    return can_tx_register(&s_valve_entry);
#endif
}
