#include "EventBus.h"
#include "main.h"      // 引用 __nop()
#include "rtt_log.h"   // 提供 MAIN_D 宏
#include <string.h>    // 提供 memset

// --- 配置区域 ---
#define MAX_SUBSCRIBERS_PER_TOPIC  4   // 每个主题最多支持4个订阅者

// ========== 二维数组存储（替代单链表） ==========
// 每个槽位存储：回调函数指针 + 优先级
typedef struct {
    EventCallback callback;     // 回调函数指针
    uint8_t       priority;     // 优先级（0=最高）
} SubscriberSlot_t;

// s_subscribers[topic][slot] - 每个主题下固定4个回调槽位
static SubscriberSlot_t s_subscribers[TOPIC_MAX][MAX_SUBSCRIBERS_PER_TOPIC];
// s_sub_count[topic] - 每个主题当前已注册的有效回调数
static uint8_t s_sub_count[TOPIC_MAX];

// ========== 门控机制 ==========
static volatile bool s_event_bus_enabled = false;   // 全局使能标志
static volatile uint32_t s_pending_events = 0;      // 位图：记录使能前有哪些主题被发布过

// ========== Keil Watch 调试结构体（全部 volatile，可在 Watch 窗口直接观察） ==========
// active_topic 使用 Topic_t 枚举类型，Keil Watch 中会直接显示枚举名（如 TOPIC_POWER）
typedef struct {
    volatile uint32_t pending_events;                    // 挂起事件位图
    volatile bool     enabled;                           // 使能标志
    volatile uint8_t  sub_counts[TOPIC_MAX];             // 各主题当前订阅数
    volatile void*    active_callback;                   // 当前正在执行的回调地址
    volatile Topic_t  active_topic;                      // 当前正在发布的话题（枚举类型，Watch 显示枚举名）
    volatile uint8_t  active_slot;                       // 当前正在执行的回调槽位
    volatile uint8_t  active_priority;                   // 当前正在执行的回调优先级
} EventBusDebug_t;

static volatile EventBusDebug_t s_eb_debug = {0};

/**
 * @brief 获取主题名称（安全版本）
 */
static const char* EventBus_GetTopicName(Topic_t topic) {
    if (topic < TOPIC_MAX) {
        return g_topic_names[topic];
    }
    return "UNKNOWN";
}

/**
 * @brief 初始化平台
 */
void EventBus_Init(void) {
    // 清零二维数组
    memset((void*)s_subscribers, 0, sizeof(s_subscribers));

    // 清零各主题计数
    for (int i = 0; i < TOPIC_MAX; i++) {
        s_sub_count[i] = 0;
    }

    // 清零门控标志
    s_event_bus_enabled = false;
    s_pending_events = 0;

    // 清零调试结构体
    memset((void*)&s_eb_debug, 0, sizeof(s_eb_debug));

    EVENT_BUS_DEBUG_PRINT("EventBus initialized, max subscribers per topic=%d\r\n",
                          MAX_SUBSCRIBERS_PER_TOPIC);
}

/**
 * @brief 订阅话题（带优先级）
 * @param topic 关注的话题
 * @param callback 触发时的回调函数
 * @param priority 优先级（0=最高，数值越大优先级越低）
 * @return 是否订阅成功
 *
 * @note 优先级排序规则：
 *       - 数值越小优先级越高（0 > 1 > 2 > ...）
 *       - 同等优先级下，先订阅的先执行（插入到同优先级末尾）
 * @note 重复订阅检测：同一 topic + callback 组合只允许注册一次
 */
bool EventBus_Subscribe(Topic_t topic, EventCallback callback, uint8_t priority) {
    if (topic >= TOPIC_MAX || callback == (void*)0) {
        EVENT_BUS_DEBUG_PRINT("Subscribe failed: topic=%d (%s), callback=%p\r\n",
                              topic, EventBus_GetTopicName(topic), callback);
        return false;
    }

    // --- 重复订阅检测 ---
    for (uint8_t i = 0; i < MAX_SUBSCRIBERS_PER_TOPIC; i++) {
        if (s_subscribers[topic][i].callback == callback) {
            EVENT_BUS_DEBUG_PRINT("Subscribe failed: duplicate! topic=%s, callback=%p\r\n",
                                  EventBus_GetTopicName(topic), callback);
            return false;
        }
    }

    // --- 检查是否还有空槽位 ---
    if (s_sub_count[topic] >= MAX_SUBSCRIBERS_PER_TOPIC) {
        EVENT_BUS_DEBUG_PRINT("Subscribe failed: topic=%s full! max=%d\r\n",
                              EventBus_GetTopicName(topic), MAX_SUBSCRIBERS_PER_TOPIC);
        return false;
    }

    // --- 按优先级插入排序 ---
    // 找到第一个优先级 <= 新优先级的槽位（即优先级数值 >= 新优先级的槽位）
    // 然后将该槽位及之后的所有有效槽位后移，腾出位置插入
    uint8_t insert_pos = s_sub_count[topic];  // 默认插入末尾

    for (uint8_t i = 0; i < s_sub_count[topic]; i++) {
        // 优先级数值越小优先级越高
        // 新插入的优先级 > 当前槽位的优先级（即新插入的优先级更低）
        // 或者同等优先级时，新插入的排在后面（先订阅的先执行）
        if (s_subscribers[topic][i].priority > priority) {
            // 当前槽位的优先级数值更大（优先级更低），新插入的应该插在这里
            insert_pos = i;
            break;
        }
    }

    // 如果 insert_pos < s_sub_count[topic]，需要将后面的槽位后移
    if (insert_pos < s_sub_count[topic]) {
        // 从后往前移动，避免覆盖
        for (uint8_t i = s_sub_count[topic]; i > insert_pos; i--) {
            s_subscribers[topic][i].callback = s_subscribers[topic][i - 1].callback;
            s_subscribers[topic][i].priority = s_subscribers[topic][i - 1].priority;
        }
    }

    // 插入新订阅
    s_subscribers[topic][insert_pos].callback = callback;
    s_subscribers[topic][insert_pos].priority = priority;
    s_sub_count[topic]++;

    // 同步调试结构体
    s_eb_debug.sub_counts[topic] = s_sub_count[topic];

    EVENT_BUS_VERBOSE_PRINT("Subscribed: topic=%s, callback=%p, priority=%d, slot=%d, total=%d\r\n",
                            EventBus_GetTopicName(topic), callback, priority, insert_pos, s_sub_count[topic]);
    return true;
}

/**
 * @brief 内部发布函数（无条件执行）
 */
static void EventBus_DoPublish(Topic_t topic, void* payload) {
    uint8_t callback_count = 0;

    // 更新调试信息（active_topic 为 Topic_t 枚举类型，Keil Watch 显示枚举名）
    s_eb_debug.active_topic = topic;

    for (uint8_t i = 0; i < s_sub_count[topic]; i++) {
        if (s_subscribers[topic][i].callback != (void*)0) {
            callback_count++;

            // 更新调试信息：当前执行的回调、槽位和优先级
            s_eb_debug.active_callback = (void*)s_subscribers[topic][i].callback;
            s_eb_debug.active_slot = i;
            s_eb_debug.active_priority = s_subscribers[topic][i].priority;

            EVENT_BUS_VERBOSE_PRINT("Calling callback %p for topic=%s (slot=%d, priority=%d)\r\n",
                                    s_subscribers[topic][i].callback, EventBus_GetTopicName(topic),
                                    i, s_subscribers[topic][i].priority);

            // 执行回调
            s_subscribers[topic][i].callback(payload);

            __nop(); // Debug: 可在 Keil Watch 中观察 s_eb_debug.active_callback
        }
    }

    // 清除调试信息
    s_eb_debug.active_callback = (void*)0;
    s_eb_debug.active_slot = 0;
    s_eb_debug.active_priority = 0;

    if (callback_count > 0) {
        EVENT_BUS_VERBOSE_PRINT("Published topic=%s, called %d callback(s)\r\n",
                                EventBus_GetTopicName(topic), callback_count);
    }
}

/**
 * @brief 发布话题（支持嵌套发布，支持门控机制）
 * @param topic 话题类型
 * @param payload 数据载体指针
 *
 * @note 门控行为：
 *       在 EventBus_Enable() 调用之前，Publish 仅记录事件标志位，
 *       不执行任何回调。使能后，之前被记录的挂起事件会立即执行一次，
 *       之后所有 Publish 正常执行回调。
 */
void EventBus_Publish(Topic_t topic, void* payload) {
    if (topic >= TOPIC_MAX) {
        EVENT_BUS_DEBUG_PRINT("Publish failed: invalid topic=%d\r\n", topic);
        return;
    }

    // ---- 门控检查 ----
    if (!s_event_bus_enabled) {
        // 总线尚未使能：仅记录事件标志，不执行回调
        s_pending_events |= (1UL << topic);
        s_eb_debug.pending_events = s_pending_events;  // 同步调试结构体
        EVENT_BUS_VERBOSE_PRINT("Publish deferred: topic=%s (bus not enabled yet)\r\n",
                                EventBus_GetTopicName(topic));
        return;
    }

    // 执行发布
    EventBus_DoPublish(topic, payload);
}

/**
 * @brief 启用事件总线
 *
 * 调用此函数后：
 * 1. 将所有使能前挂起的事件按位图顺序依次执行一次回调
 * 2. 之后所有 Publish 正常执行
 *
 * @note 此函数应在所有模块完成 Subscribe 后调用一次。
 *       典型用法：
 *       @code
 *       EventBus_Init();
 *       // ... 各模块 Subscribe ...
 *       EventBus_Enable();  // 放行所有挂起事件
 *       @endcode
 */
void EventBus_Enable(void) {
    if (s_event_bus_enabled) {
        EVENT_BUS_DEBUG_PRINT("EventBus already enabled, ignore\r\n");
        return;
    }

    EVENT_BUS_DEBUG_PRINT("EventBus enabling... pending_events=0x%08lX\r\n",
                          (unsigned long)s_pending_events);

    // 1. 先置标志，使后续 Publish 正常执行
    s_event_bus_enabled = true;
    s_eb_debug.enabled = true;  // 同步调试结构体

    // 2. 遍历所有主题，处理挂起事件
    //    注意：处理过程中可能产生新的嵌套 Publish，但此时已使能，会正常执行
    for (Topic_t t = (Topic_t)0; t < TOPIC_MAX; t++) {
        if (s_pending_events & (1UL << t)) {
            EVENT_BUS_DEBUG_PRINT("EventBus replay pending event: topic=%s\r\n",
                                  EventBus_GetTopicName(t));
            EventBus_DoPublish(t, (void*)0);  // 挂起事件 payload 为 NULL
            __nop(); // Debug: 可在 Keil Watch 中观察 s_pending_events 位清除情况
        }
    }

    // 3. 清除挂起标志
    s_pending_events = 0;
    s_eb_debug.pending_events = 0;  // 同步调试结构体

    EVENT_BUS_DEBUG_PRINT("EventBus enabled\r\n");

    __nop(); // Debug: 可在 Keil Watch 中观察 s_eb_debug
}

/**
 * @brief 查询事件总线是否已启用
 * @return true 已启用，false 未启用（初始化阶段）
 */
bool EventBus_IsEnabled(void) {
    return s_event_bus_enabled;
}
