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

#include "status_tx.h"
#include "led_tx.h"

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
