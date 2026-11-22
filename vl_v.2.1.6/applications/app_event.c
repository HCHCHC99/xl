/**
 * @file    app_event.c
 * @brief   事件投递实现
 */

#include "app_event.h"
#include "ipc_center.h"
#include "framework/state_engine/sm_assert.h"

/**
 * @brief 投递事件到前台主队列
 * @note  【2026-10-27 · D-58】原为 SM_ASSERT_PTR(g_app_event_mq)。
 *
 *        本函数是全工程最热的路径之一（57 个调用点），keyboard 线程
 *        每次按键扫描、CAN 解析线程每帧分发、各业务模块每次状态变化
 *        都要经过它。而 SM_ASSERT_PTR 在失败时会展开：
 *          rt_assert_handler(32) → rt_kprintf(32) → rt_vsnprintf(80)
 *            → print_number(72) ...  ≈ 600 B
 *        即每个调用者的栈都要为这条"永不发生的分支"预留 600 B。
 *
 *        g_app_event_mq 在 boot 第 2 步 ipc_center_init 就已创建，
 *        早于所有可能调用 post_event 的线程启动，NULL 不可达。
 *        故降级为直接返回：异常时丢一个事件（可接受），
 *        而不是把整机卡死在 rt_assert_handler 的 while(1) 里。
 */
void post_event(app_event_t event)
{
    if (g_app_event_mq == RT_NULL) {
        return;
    }
    rt_mq_send(g_app_event_mq, &event, sizeof(app_event_t));
}
