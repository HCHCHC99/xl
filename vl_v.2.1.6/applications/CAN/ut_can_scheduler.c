/**
 * @file    ut_can_scheduler.c
 * @brief   CAN 接收调度器（环形缓冲区 + 解析线程）
 */

#include "ut_can_config.h"
#include "ut_can_scheduler.h"
#include "app_event.h"
#include "framework/state_engine/sm_assert.h"
#include "ipc_center.h"
#include <rtdevice.h>
#include <string.h>
#include "ut_can_send.h"

/* ----- 环形缓冲区 ----- */
#define CAN_RB_SIZE   (sizeof(can_frame_t) * 32)
static uint8_t can_rb_pool[CAN_RB_SIZE];

static struct rt_ringbuffer can_rb;
static rt_sem_t can_rx_sem;

/* 【Q-D1 / 2026-11-14】CAN 通信心跳：解析线程每取一帧更新一次。
 * volatile：解析线程写、app_main 主线程读（vehicle_poll），跨线程可见性。
 * 32 位对齐读写在 Cortex-M4 上原子；两处均为线程上下文，无需关中断。 */
static volatile uint32_t s_any_frame_tick = 0U;

/* 使用 ipc_center 中的信号量 */
extern rt_sem_t g_can_rx_sem;

/* ----- 外部路由表（在 can_handlers.c 中定义） ----- */
extern const can_route_entry_t can_route_table[];
extern const uint32_t can_route_table_size;
extern can_device_entry_t g_devices[CAN_CHANNEL_MAX];

/* ============================================================
 * 初始化
 * ============================================================ */
void can_scheduler_init(void)
{
    rt_ringbuffer_init(&can_rb, can_rb_pool, CAN_RB_SIZE);
    /* g_can_rx_sem 已在 ipc_center 中创建 */
}

/* ============================================================
 * 中断回调（由底层CAN驱动调用，极速入队）
 * ============================================================ */
void can_scheduler_push_frame(uint32_t id, uint8_t *data, uint8_t len)
{
    can_frame_t frame;

    /* 【2026-10-27 · D-58】原为 SM_ASSERT_PTR/SM_ASSERT_RANGE，改为直接返回。
     *
     * 本函数的调用上下文很关键：can_rx_call() 是 rt_device_set_rx_indicate
     * 注册的回调，由 CAN 接收中断路径调用。在中断里触发断言会走到
     * rt_assert_handler → rt_kprintf → rt_device_write，即在 ISR 内做
     * 阻塞式设备写 —— 比栈溢出更严重的问题。
     *
     * 参数由驱动层 msg.data/msg.len 直接传入，len 已被驱动截断到 0~8，
     * 正常不可达；异常时静默丢弃一帧，绝不进 ISR 打印。 */
    if (data == RT_NULL) {
        return;
    }
    if (len > 8U) {
        len = 8U;
    }

    frame.id = id;
    frame.len = len;
    memcpy(frame.data, data, frame.len);

    if (frame.len < 8U) {
        memset(&frame.data[frame.len], 0, 8U - frame.len);
    }

    if (rt_ringbuffer_put(&can_rb, (rt_uint8_t*)&frame, sizeof(can_frame_t)) == sizeof(can_frame_t)) {
        rt_sem_release(g_can_rx_sem);
    }
    /* 缓冲区满：静默丢弃 */
}

/* 接收数据回调函数 */
static rt_err_t can_rx_call(rt_device_t dev, rt_size_t size)
{
    struct rt_can_msg msg; uint32_t n = 0;
    if (rt_device_read(g_devices[0].dev, 0, &msg, sizeof(msg)) == sizeof(msg)) {
        if (!msg.rtr){
            can_scheduler_push_frame(msg.id, msg.data, msg.len);
        }
    }

    /* CAN 接收到数据后产生中断，调用此回调函数，然后发送接收信号量 */
    rt_sem_release(g_can_rx_sem);

//    rt_kprintf("[CAN] can_rx_call...\n\n");

    return RT_EOK;
}


/* ============================================================
 * 解析线程（表驱动查表 + 回调）
 * ============================================================ */
static void can_parser_thread_entry(void *param)
{
    can_frame_t frame;

    /* 设置接收回调函数 */
    rt_device_set_rx_indicate(g_devices[0].dev, can_rx_call);

    while (1) {
        rt_sem_take(g_can_rx_sem, RT_WAITING_FOREVER);
        while (rt_ringbuffer_get(&can_rb, (rt_uint8_t*)&frame, sizeof(can_frame_t)) == sizeof(can_frame_t)) {
            /* ---- Q-D1：任何一帧触发解析线程查表，即认定 CAN 通信有效 ----
             * 放在查表 for 之前 ⇒ 不要求命中 can_route_table（4 条登记 ID）；
             * 等价于「通过硬件滤波器全收的任何非 RTR 数据帧」。
             * 注意：本函数在线程上下文，不是 ISR，故不受 D-58 约束。 */
            s_any_frame_tick = (uint32_t)rt_tick_get();

            /* 查表分发 */
            for (uint32_t i = 0; i < can_route_table_size; i++) {
                if ((frame.id & can_route_table[i].mask) == (can_route_table[i].id & can_route_table[i].mask)) {
                    if (can_route_table[i].callback) {
                        can_route_table[i].callback(&frame);
                    }
                    break;
                }
            }
        }
    }
}

/* ============================================================
 * CAN 总线静默判定（Q-D1）
 * ============================================================ */
uint8_t can_is_bus_silent(uint32_t timeout_ms)
{
    uint32_t last = s_any_frame_tick;   /* volatile 单次快照 */

#if (INPUT_SIM_ENABLE && CAN_SILENT_SIM_ENABLE)
    /* 台架模拟：t_sim cansilent 1 强制判静默（测 comm_sta / 整车 2s FAULT） */
    if (input_sim_active(INPUT_SIM_CANSILENT) != 0U) {
        return (uint8_t)((input_sim_value(INPUT_SIM_CANSILENT) != 0) ? 1U : 0U);
    }
#endif

    /* 从未有任何帧触发过解析线程查表 ⇒ 按新定义直接判静默 */
    if (last == 0U) {
        return 1U;
    }

    /* 无符号减法，tick 回绕时仍正确 */
    if ((uint32_t)(rt_tick_get() - last) > rt_tick_from_millisecond(timeout_ms)) {
        return 1U;
    }
    return 0U;
}

/* ============================================================
 * 启动调度器
 * ============================================================ */
void can_scheduler_start(void)
{
    /* 【2026-10-27 · D-59】原 2048，按实测下调。
     * 实测：正常 264 B / 含打印 424 B，+240 B 开销 → 峰值约 664 B。
     * 取 1024：留 1.5 倍余量。
     * 注意：本线程通过 can_route_table[] 的函数指针分发报文，
     *       静态图看不到回调内部；当前回调均为轻量 handler。 */
    rt_thread_t tid = rt_thread_create("can_parser",
                                        can_parser_thread_entry,
                                        RT_NULL,
                                        1024,
                                        5,
                                        10);

    SM_ASSERT_PTR(tid);

    rt_thread_startup(tid);

    rt_kprintf("[CAN] Parser thread created (Prio 5).\n\n");
}
