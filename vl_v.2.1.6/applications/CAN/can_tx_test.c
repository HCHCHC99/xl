/**
 * @file    can_tx_test.c
 * @brief   状态帧 / 灯帧的 RTT 观测命令（P8 / 2026-11-23）
 *
 * ============================================================
 * 为什么单独做这两个命令
 * ============================================================
 * 这两条帧都是"本机发出去给别人看"的：状态帧给 HMI，灯帧给遥控器面板。
 * 台架上两头都没有，`t_cantx` 只能看到每个 slot 的 ok/err 计数，
 * 看不到帧里到底是什么 —— 想验"编码器故障该报红闪"就必须接 CAN 仪。
 *
 * t_sta / t_led 直接打印发送缓存 + 逐字段解码 + 判定依据，
 * 与"帧有没有真的发出去"无关（can_tx_cfg.h 关掉发送后照样能看）。
 *
 * ============================================================
 * 用法
 * ============================================================
 *   t_sta           打印一次状态帧
 *   t_sta 5         连打 5 次，间隔 500ms（看变化用）
 *   t_led           打印一次灯帧
 *   t_led 5         同上
 *
 * 只读命令，不要求 DIAG_WRITE（与 t_cantx 一致）。
 */

#include <rtthread.h>
#include <finsh.h>
#include <stdlib.h>
#include <string.h>

#include "status_tx.h"
#include "led_tx.h"
#include "ut_can_scheduler.h"
#include "../config/sys_config.h"

#define CAN_TX_TEST_MAX_REPEAT   10
#define CAN_TX_TEST_INTERVAL_MS  500

static int parse_repeat(int argc, char **argv)
{
    int n = 1;

    if (argc >= 2) {
        n = atoi(argv[1]);
    }
    if (n < 1) {
        n = 1;
    }
    if (n > CAN_TX_TEST_MAX_REPEAT) {
        n = CAN_TX_TEST_MAX_REPEAT;
    }
    return n;
}

/* ------------------------------------------------------------
 * t_sta [n] —— 打印 0x18FF11D1 状态帧
 * ------------------------------------------------------------ */
static void cmd_t_sta(int argc, char **argv)
{
    int n = parse_repeat(argc, argv);
    int i;

    for (i = 0; i < n; i++) {
        if (n > 1) {
            rt_kprintf("[t_sta] %d/%d\n", i + 1, n);
        }
        status_tx_dump();
        if ((n > 1) && (i + 1 < n)) {
            rt_thread_mdelay(CAN_TX_TEST_INTERVAL_MS);
        }
    }
}
MSH_CMD_EXPORT_ALIAS(cmd_t_sta, t_sta, dump status frame 0x18FF11D1);

/* ------------------------------------------------------------
 * t_led [n] —— 打印 0x18FFA4F0 灯帧
 * ------------------------------------------------------------ */
static void cmd_t_led(int argc, char **argv)
{
    int n = parse_repeat(argc, argv);
    int i;

    for (i = 0; i < n; i++) {
        if (n > 1) {
            rt_kprintf("[t_led] %d/%d\n", i + 1, n);
        }
        led_tx_dump();
        if ((n > 1) && (i + 1 < n)) {
            rt_thread_mdelay(CAN_TX_TEST_INTERVAL_MS);
        }
    }
}
MSH_CMD_EXPORT_ALIAS(cmd_t_led, t_led, dump led frame 0x18FFA4F0);

/* ------------------------------------------------------------
 * t_comm —— CAN 通信静默/门限诊断（P10）
 * ------------------------------------------------------------
 * t_comm          打印 age / thr / state
 * t_comm test     无总线时验证 300 ms 边界：250 ms 不报、350 ms 报
 * t_comm age <ms> 台架专用：把"最近一帧时刻"拨到 ms 之前
 * ------------------------------------------------------------ */
static void comm_print_line(void)
{
    int32_t  thr = CFG_DEF_CAN_FAULT_MS;
    uint32_t age = can_last_frame_age_ms();
    uint8_t  st;

    (void)sys_config_get_i32(CFG_ID_CAN_FAULT_MS, &thr);
    st = can_is_bus_silent((uint32_t)thr);

    if (age == 0xFFFFFFFFU) {
        rt_kprintf("[COMM] age=never thr=%ldms state=%u(%s)\n",
                   (long)thr, (unsigned)st,
                   (st != 0U) ? "FAULT" : "ok");
    } else {
        rt_kprintf("[COMM] age=%lums thr=%ldms state=%u(%s)\n",
                   (unsigned long)age, (long)thr, (unsigned)st,
                   (st != 0U) ? "FAULT" : "ok");
    }
}

#if (INPUT_SIM_ENABLE && CAN_SILENT_SIM_ENABLE)
static void comm_boundary_test(void)
{
    int32_t  thr = CFG_DEF_CAN_FAULT_MS;
    uint32_t age;
    uint8_t  st;

    (void)sys_config_get_i32(CFG_ID_CAN_FAULT_MS, &thr);

    /* 关掉 cansilent 强制，让判定走真实 age */
    input_sim_clear(INPUT_SIM_CANSILENT);

    can_scheduler_set_last_frame_age_ms(250U);
    age = can_last_frame_age_ms();
    st  = can_is_bus_silent((uint32_t)thr);
    rt_kprintf("[COMM] test age=%lums thr=%ldms -> %u(%s) expect ok\n",
               (unsigned long)age, (long)thr, (unsigned)st,
               (st != 0U) ? "FAULT" : "ok");

    can_scheduler_set_last_frame_age_ms(350U);
    age = can_last_frame_age_ms();
    st  = can_is_bus_silent((uint32_t)thr);
    rt_kprintf("[COMM] test age=%lums thr=%ldms -> %u(%s) expect FAULT\n",
               (unsigned long)age, (long)thr, (unsigned)st,
               (st != 0U) ? "FAULT" : "ok");

    can_scheduler_set_last_frame_age_ms(0U);
    comm_print_line();
}
#endif

static void cmd_t_comm(int argc, char **argv)
{
    if ((argc >= 2) && (strcmp(argv[1], "test") == 0)) {
#if (INPUT_SIM_ENABLE && CAN_SILENT_SIM_ENABLE)
        comm_boundary_test();
#else
        rt_kprintf("[COMM] test disabled (INPUT_SIM_ENABLE/CAN_SILENT_SIM_ENABLE=0)\n");
#endif
        return;
    }

    if ((argc >= 3) && (strcmp(argv[1], "age") == 0)) {
#if (INPUT_SIM_ENABLE && CAN_SILENT_SIM_ENABLE)
        int32_t age_ms = atoi(argv[2]);

        if (age_ms < 0) {
            age_ms = 0;
        }
        can_scheduler_set_last_frame_age_ms((uint32_t)age_ms);
        comm_print_line();
#else
        rt_kprintf("[COMM] age set disabled (INPUT_SIM_ENABLE/CAN_SILENT_SIM_ENABLE=0)\n");
#endif
        return;
    }

    comm_print_line();
}
MSH_CMD_EXPORT_ALIAS(cmd_t_comm, t_comm, dump CAN comm silence age/threshold);
