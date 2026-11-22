/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-05-23     supperthomas init
 * 2022-10-07     supperthomas add the support of RT_USING_SERIAL_V2
 *
 */
#include "rtdevice.h"
#include "../RTT/SEGGER_RTT.h"

static struct rt_serial_device _serial_jlink_rtt;

/* ============================================================
 * 【2026-10-27 · P4】RTT 控制台诊断计数器
 *
 * 为什么需要：控制台曾出现"敲了命令没反应，等一会儿才好"。这条链路上
 * 有四个可能断点，肉眼完全看不出来是哪一个：
 *   ① 主机写入下行缓冲时被 NO_BLOCK_SKIP 整段丢弃（缓冲太小）
 *   ② 设备没 open，segger_rtt_check 整次跳过
 *   ③ idle 钩子没被调度到（系统不空闲），键取不出来
 *   ④ 取出来了但 serial rx fifo 溢出
 *
 * ①~③ 都在本文件内，用下面几个计数器即可闭环定位；
 * ④ 由 rt_hw_serial_isr 的 _serial_check_buffer_size() 打印（只打一次）。
 *
 * 用法：串口正常时先 `t_con` 取一次基线，复现卡顿时再取一次，
 * 两个快照做差即可 —— keys_taken 不动而 check_calls 在涨，
 * 说明 idle 在跑但缓冲里没东西（= 主机侧写失败，①）；两个都不动，
 * 说明 idle 没被调度（③）。
 * ============================================================ */

typedef struct {
    rt_uint32_t check_calls;    /* segger_rtt_check 被调用次数（≈idle 钩子调度次数） */
    rt_uint32_t keys_taken;     /* 累计取出的字节数 */
    rt_uint32_t skip_not_open;  /* 因设备未 open 而整次跳过的次数 */
    rt_uint32_t pending_peak;   /* 下行缓冲"待处理字节数"的历史峰值 */
    rt_uint32_t pending_now;    /* 当前待处理字节数（最近一次采样值） */
} rtt_con_stat_t;

static rtt_con_stat_t s_rtt_stat;

/* 【2026-11-18】RTT 输入轮询定时器
 * 原实现只在 idle 钩子里取键；电机闭环时 20kHz ISR 占用接近满载，
 * idle 长时间得不到调度，下行命令永远没人取，表现为 msh 输入无效。
 * 这里改用 10ms 软件定时器轮询，定时器线程优先级高于 idle，
 * 只要还有一点 CPU 余量就能把输入取出来。 */
static struct rt_timer s_rtt_rx_poll_timer;
static volatile rt_uint8_t s_rtt_rx_gate;   /* 检测到下行数据 = 1 */

static void segger_rtt_check(void);

static void rtt_rx_poll_timeout(void *parameter)
{
    (void)parameter;

    /* 轻量检测：只判断缓冲里有没有数据，避免每 10ms 都进一遍串口 ISR */
    if (SEGGER_RTT_HASDATA(RTT_DEFAULT_BUFFER_INDEX) != 0u) {
        s_rtt_rx_gate = 1u;
    }

    /* 门控消费：真正取键、喂给 tshell 的动作在这里做 */
    if (s_rtt_rx_gate != 0u) {
        s_rtt_rx_gate = 0u;
        segger_rtt_check();
    }
}

/* 下行缓冲有效容量的边界值（SizeOfBuffer − 1）。
 * pending 达到这个值就说明缓冲已满，下一次主机写入必然被整段丢弃。 */
static rt_uint32_t _rtt_down_capacity(void)
{
    rt_uint32_t sz = _SEGGER_RTT.aDown[RTT_DEFAULT_BUFFER_INDEX].SizeOfBuffer;
    return (sz > 0U) ? (sz - 1U) : 0U;
}

typedef struct
{
    struct rt_serial_device *serial;
} drv_jlink_rtt_cfg_t;

static drv_jlink_rtt_cfg_t m_jlink_rtt_cfg =
{
    .serial = &_serial_jlink_rtt,
};


static rt_err_t _rtt_cfg(struct rt_serial_device *serial, struct serial_configure *cfg)
{
    return RT_EOK;
}

static rt_err_t _rtt_ctrl(struct rt_serial_device *serial, int cmd, void *arg)
{
    return RT_EOK;
}
static int _rtt_putc(struct rt_serial_device *serial, char c)
{
    SEGGER_RTT_PutChar(RTT_DEFAULT_BUFFER_INDEX, c);
    return RT_EOK;
}

static int _rtt_getc(struct rt_serial_device *serial)
{
    int ch;

    ch = SEGGER_RTT_GetKey();

    /* 【P4】计数。注意：V1 与 V2 两条路径的取键位置不同但都会走到这里
     * （V2 在 segger_rtt_check 里先 GetKey 再 isr，那样不经过本函数，
     *  所以 V2 的统计见 segger_rtt_check 分支），此处仅覆盖 V1。
     * 本工程实际生效的是 RT_USING_SERIAL_V1。 */
    if (ch >= 0) {
        s_rtt_stat.keys_taken++;
    }

    return ch;
}
#ifdef RT_USING_SERIAL_V2
///< RT-Thread Kernel version
#if defined(RT_VERSION_CHECK) && (RTTHREAD_VERSION >= RT_VERSION_CHECK(5, 0, 2))
static rt_ssize_t mcu_transmit(struct rt_serial_device     *serial,
                                rt_uint8_t           *buf,
                                rt_size_t             size,
                                rt_uint32_t           tx_flag)
#else   /* legacy version macros (<5.0.2) */
static rt_size_t mcu_transmit(struct rt_serial_device     *serial,
                                rt_uint8_t           *buf,
                                rt_size_t             size,
                                rt_uint32_t           tx_flag)
#endif
{
    RT_ASSERT(serial != RT_NULL);
    RT_ASSERT(buf != RT_NULL);

    SEGGER_RTT_Write(RTT_DEFAULT_BUFFER_INDEX, buf,size);
    rt_hw_serial_isr(serial, RT_SERIAL_EVENT_TX_DMADONE);

    return size;
}
#endif

static struct rt_uart_ops _jlink_rtt_ops =
{
    _rtt_cfg,
    _rtt_ctrl,
    _rtt_putc,
    _rtt_getc,
#ifdef RT_USING_SERIAL_V2
    .transmit = mcu_transmit
#endif
};

static void segger_rtt_check_body(void)
{
    rt_uint32_t pending;

    /* 【P4】采样待处理字节数（主机已写入、设备尚未取走的量）。
     * 这个值长期贴近容量上限，就是"缓冲太小 / idle 取键太慢"的直接证据。 */
    pending = SEGGER_RTT_HASDATA(RTT_DEFAULT_BUFFER_INDEX);
    s_rtt_stat.pending_now = pending;
    if (pending > s_rtt_stat.pending_peak) {
        s_rtt_stat.pending_peak = pending;
    }

    s_rtt_stat.check_calls++;

    if (!(_serial_jlink_rtt.parent.open_flag & RT_DEVICE_OFLAG_OPEN)) {
        s_rtt_stat.skip_not_open++;
        return;
    }

#ifdef RT_USING_SERIAL_V2
    struct rt_serial_rx_fifo *rx_fifo;
    rx_fifo = (struct rt_serial_rx_fifo *)_serial_jlink_rtt.serial_rx;
    RT_ASSERT(rx_fifo != RT_NULL);
    while (SEGGER_RTT_HasKey() && (_serial_jlink_rtt.parent.open_flag & RT_DEVICE_OFLAG_OPEN))
    {
        rt_ringbuffer_putchar(&(rx_fifo->rb), SEGGER_RTT_GetKey());
        s_rtt_stat.keys_taken++;                /* 【P4】V2 路径在此取键 */
        rt_hw_serial_isr(&_serial_jlink_rtt, RT_SERIAL_EVENT_RX_IND);
    }
#else  //RT_USING_SERIAL_V1
    /* V1 路径：rt_hw_serial_isr 内部会循环调 ops->getc（即 _rtt_getc），
     * 把 fifo 一次抽干，故 keys_taken 在 _rtt_getc 里累加。 */
    while (SEGGER_RTT_HasKey() && (_serial_jlink_rtt.parent.open_flag & RT_DEVICE_OFLAG_OPEN))
    {
        rt_hw_serial_isr(&_serial_jlink_rtt, RT_SERIAL_EVENT_RX_IND);
    }
#endif
}

/* 防重入：idle 钩子可能被定时器线程打断，两边不能同时操作串口 FIFO */
static volatile rt_uint8_t s_rtt_in_check;

static void segger_rtt_check(void)
{
    if (s_rtt_in_check != 0u) {
        return;
    }
    s_rtt_in_check = 1u;
    segger_rtt_check_body();
    s_rtt_in_check = 0u;
}


int rt_hw_jlink_rtt_init(void)
{
    SEGGER_RTT_Init();

    /* 恢复 idle 钩子：这是调度器启动前/后的兜底输入通路。
     * 定时器轮询必须放到 INIT_APP（调度器和 timer 系统初始化之后），
     * 否则 rt_system_timer_init() 会把提前启动的定时器从链表里丢掉。 */
    rt_thread_idle_sethook(segger_rtt_check);

    SEGGER_RTT_SetTerminal(RTT_DEFAULT_TERMINAL_INDEX);

    SEGGER_RTT_ConfigUpBuffer(RTT_DEFAULT_BUFFER_INDEX, "RTTUP", NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_SKIP);

    SEGGER_RTT_ConfigDownBuffer(RTT_DEFAULT_BUFFER_INDEX, "RTTDOWN", NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_SKIP);

    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    _serial_jlink_rtt.ops = &_jlink_rtt_ops;
    _serial_jlink_rtt.config = config;
    m_jlink_rtt_cfg.serial = &_serial_jlink_rtt;
    rt_hw_serial_register(&_serial_jlink_rtt, "jlinkRtt", \
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX,  &m_jlink_rtt_cfg);

    SEGGER_RTT_printf(0,"\r\n SEGGER_RTT ADDRESS:%p \r\n",&_SEGGER_RTT);

    return 0;
}

int rt_hw_jlink_console_init(void)
{
    rt_hw_jlink_rtt_init();
    rt_console_set_device("jlinkRtt");
    return 0;
}
#ifdef RT_USING_SERIAL_V2
INIT_APP_EXPORT(rt_hw_jlink_console_init);
//if use RT_USING_SERIAL_V2 you can only use console after schedule, the root log missing
#endif

// if use RT_USING_SERIAL_V1  you can add the code before schedule
#ifdef RT_USING_SERIAL_V1
INIT_BOARD_EXPORT(rt_hw_jlink_console_init);
#endif

/* 【2026-11-18】10ms 输入轮询：检测到下行数据就置门控并在本循环消费。
 * 必须放在 INIT_APP：rt_hw_board_init() 早于 rt_system_timer_init()，
 * 在 INIT_BOARD 里启动的定时器会被后者从链表里丢掉，永远不触发。 */
static int rtt_rx_poll_init(void)
{
    rt_timer_init(&s_rtt_rx_poll_timer,
                  "rttpoll",
                  rtt_rx_poll_timeout,
                  RT_NULL,
                  rt_tick_from_millisecond(10),
                  RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    rt_timer_start(&s_rtt_rx_poll_timer);
    return 0;
}
INIT_APP_EXPORT(rtt_rx_poll_init);

/* ============================================================
 * 【2026-10-27 · P4】t_con —— RTT 控制台链路诊断
 *
 * 判定表（两次快照做差，d 为增量）：
 *   d.check_calls ≈ 0        → idle 钩子没被调度（系统长期不空闲 / 钩子被覆盖）
 *   d.check_calls > 0
 *     d.keys_taken ≈ 0       → idle 在跑但缓冲空 = 主机侧写入被丢弃（缓冲太小/超长行）
 *     d.keys_taken > 0 但无响应 → 断点在 serial rx fifo 或 finsh（看 rx_ovf 提示）
 *   pending_peak == capacity → 缓冲曾经写满，NO_BLOCK_SKIP 必然丢弃过整段数据
 * ============================================================ */
static void t_con(int argc, char **argv)
{
    rt_uint32_t cap = _rtt_down_capacity();
    rt_uint32_t up_sz;
    rt_device_t dev;

    (void)argc; (void)argv;

    dev = rt_device_find("jlinkRtt");
    up_sz = _SEGGER_RTT.aUp[RTT_DEFAULT_BUFFER_INDEX].SizeOfBuffer;

    rt_kprintf("\n===== RTT CONSOLE DIAG =====\n");
    rt_kprintf("device      : jlinkRtt (%s)\n",
               (dev != RT_NULL) ? "registered" : "NOT FOUND");
    rt_kprintf("open_flag   : 0x%04x %s\n",
               (unsigned)_serial_jlink_rtt.parent.open_flag,
               (_serial_jlink_rtt.parent.open_flag & RT_DEVICE_OFLAG_OPEN) ? "(OPEN)" : "(CLOSED)");
    rt_kprintf("buf up/down : %u / %u B   (down usable %u B)\n",
               (unsigned)up_sz,
               (unsigned)_SEGGER_RTT.aDown[RTT_DEFAULT_BUFFER_INDEX].SizeOfBuffer,
               (unsigned)cap);
    rt_kprintf("--------------------------------------------------\n");
    rt_kprintf("check_calls     : %u   (idle hook invocations)\n",
               (unsigned)s_rtt_stat.check_calls);
    rt_kprintf("keys_taken      : %u   (bytes pulled out of RTT)\n",
               (unsigned)s_rtt_stat.keys_taken);
    rt_kprintf("skip_not_open   : %u\n", (unsigned)s_rtt_stat.skip_not_open);
    rt_kprintf("pending now/peak: %u / %u B\n",
               (unsigned)s_rtt_stat.pending_now,
               (unsigned)s_rtt_stat.pending_peak);

    /* 逐条给结论，避免操作者自己算 */
    rt_kprintf("--------------------------------------------------\n");
    if (dev == RT_NULL) {
        rt_kprintf("VERDICT: FAIL - console device missing\n");
    } else if (!(_serial_jlink_rtt.parent.open_flag & RT_DEVICE_OFLAG_OPEN)) {
        rt_kprintf("VERDICT: FAIL - device not opened, keys will never be read\n");
    } else if (s_rtt_stat.check_calls == 0U) {
        rt_kprintf("VERDICT: FAIL - idle hook never ran (system never idle?)\n");
    } else if (s_rtt_stat.pending_peak >= cap && cap > 0U) {
        rt_kprintf("VERDICT: WARN - down buffer hit full (%u/%u), SKIP mode drops whole lines\n",
                   (unsigned)s_rtt_stat.pending_peak, (unsigned)cap);
        rt_kprintf("        -> increase BUFFER_SIZE_DOWN or send shorter commands\n");
    } else if (s_rtt_stat.keys_taken == 0U) {
        rt_kprintf("VERDICT: IDLE - hook runs but no key ever arrived; host not writing\n");
    } else {
        rt_kprintf("VERDICT: OK - console link is alive\n");
    }
    rt_kprintf("tip: take this snapshot twice and diff, see code comment for table\n");
    rt_kprintf("============================\n\n");
}
MSH_CMD_EXPORT(t_con, show RTT console link diagnostics);


