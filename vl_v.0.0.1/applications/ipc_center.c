/**
 * @file    ipc_center.c
 * @brief   IPC 中心 - 集中创建所有 IPC 对象
 */

#include "ipc_center.h"
#include "framework/state_engine/sm_assert.h"

/* ============================================================
 * 句柄定义
 * ============================================================ */
rt_mq_t g_app_event_mq = RT_NULL;
rt_sem_t g_can_rx_sem = RT_NULL;
rt_mutex_t g_state_machine_lock = RT_NULL;
rt_mutex_t g_can_send_lock = RT_NULL;

/* ============================================================
 * 初始化
 * ============================================================ */
int ipc_center_init(void)
{
    /* 1. 创建主事件消息队列（容量 32 条，每条 4 字节） */
    g_app_event_mq = rt_mq_create("evt_mq", sizeof(uint32_t), 32, RT_IPC_FLAG_PRIO);
    SM_ASSERT_PTR(g_app_event_mq);

    /* 2. 创建 CAN 接收信号量（初始值 0） */
    g_can_rx_sem = rt_sem_create("can_rx", 0, RT_IPC_FLAG_PRIO);
    SM_ASSERT_PTR(g_can_rx_sem);

    /* 3. 创建状态机互斥锁 */
    g_state_machine_lock = rt_mutex_create("sm_lock", RT_IPC_FLAG_PRIO);
    SM_ASSERT_PTR(g_state_machine_lock);

    /* 4. 创建 CAN 发送互斥锁 */
    g_can_send_lock = rt_mutex_create("can_tx", RT_IPC_FLAG_PRIO);
    SM_ASSERT_PTR(g_can_send_lock);

    rt_kprintf("[Main] IPC center initialized.\n\n");

    return 0;
}
//INIT_COMPONENT_EXPORT(ipc_center_init);
