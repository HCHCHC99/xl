#include "msg_queue.h"
#include "TickTimer.h"
#include <string.h>

/* 获取优先级字符串 */
static const char* GetPriorityStr(msg_prio_t priority)
{
    switch(priority) {
        case MSG_PRIO_URGENT: return "URGENT";
        case MSG_PRIO_HIGH:   return "HIGH";
        case MSG_PRIO_NORMAL: return "NORMAL";
        case MSG_PRIO_LOW:    return "LOW";
        default:              return "UNKNOWN";
    }
}

void MsgQueue_Init(msg_queue_t *queue, msg_t *buffer, uint16_t size, queue_config_t *config, const char *name)
{
    if (queue == NULL || buffer == NULL) return;

    queue->buffer = buffer;
    queue->size = size;
    queue->head = 0;
    queue->tail = 0;
    queue->count = 0;

    if (name != NULL && strlen(name) < sizeof(queue->name)) {
        strcpy((char*)queue->name, name);
    } else {
        strcpy((char*)queue->name, "unknown");
    }

    if (config != NULL) {
        queue->config = *config;
    } else {
        queue->config.max_size = size;
        queue->config.overwrite = false;
        queue->config.priority_enabled = false;
        queue->config.timeout_ms = 0;
    }

    memset(&queue->stats, 0, sizeof(queue_stats_t));
    Lock_Init(&queue->lock, LOCK_TYPE_MUTEX, queue->name);

    QUEUE_INIT_PRINT("Queue '%s' initialized, size=%d, overwrite=%d, priority=%d",
                queue->name, size, queue->config.overwrite, queue->config.priority_enabled);
}

/* 内部入队函数 */
static bool EnqueueInternal(msg_queue_t *queue, msg_t *msg)
{
    uint16_t pos;

    if (queue->count >= queue->size) {
        queue->stats.overflow_count++;

        QUEUE_SEND_PRINT("Queue '%s' FULL! count=%d/%d, overflow_count=%d",
                    queue->name, queue->count, queue->size, queue->stats.overflow_count);

        if (!queue->config.overwrite) {
            QUEUE_SEND_PRINT("Queue '%s' DROPPED message (overwrite disabled)", queue->name);
            return false;
        }

        QUEUE_SEND_PRINT("Queue '%s' OVERWRITE oldest message", queue->name);
        queue->head = (queue->head + 1) % queue->size;
        queue->count--;
    }

    /* 找到插入位置（优先级模式） */
    if (queue->config.priority_enabled && queue->count > 0) {
        uint16_t i;
        bool inserted = false;

        for (i = 0; i < queue->count; i++) {
            pos = (queue->tail - 1 - i + queue->size) % queue->size;
            if (queue->buffer[pos].priority <= msg->priority) {
                memmove(&queue->buffer[(pos + 2) % queue->size],
                       &queue->buffer[(pos + 1) % queue->size],
                       (queue->count - i) * sizeof(msg_t));
                queue->tail = (queue->tail + 1) % queue->size;
                pos = (pos + 1) % queue->size;
                inserted = true;
                QUEUE_SEND_PRINT("Queue '%s' priority insert at pos=%d", queue->name, pos);
                break;
            }
        }

        if (!inserted) {
            pos = queue->tail;
            queue->tail = (queue->tail + 1) % queue->size;
        }
    } else {
        pos = queue->tail;
        queue->tail = (queue->tail + 1) % queue->size;
    }

    memcpy(&queue->buffer[pos], msg, sizeof(msg_t));
    queue->buffer[pos].timestamp = tickTimer_GetCount();
    queue->count++;

    QUEUE_SEND_PRINT("Queue '%s' ENQUEUE - pos=%d, type=%d, len=%d, prio=%s, count=%d/%d",
                queue->name, pos, msg->type, msg->len,
                GetPriorityStr(msg->priority), queue->count, queue->size);

    return true;
}

bool MsgQueue_Send(msg_queue_t *queue, msg_t *msg, bool blocking, const char *caller)
{
    uint64_t startTick = 0;
    bool result = false;

    if (queue == NULL || msg == NULL) {
        QUEUE_SEND_PRINT("Send failed - queue or msg is NULL (caller='%s')", caller ? caller : "unknown");
        return false;
    }

    QUEUE_SEND_PRINT("Send request - queue='%s', caller='%s', type=%d, len=%d, prio=%s, blocking=%d",
                queue->name, caller ? caller : "unknown", msg->type, msg->len,
                GetPriorityStr(msg->priority), blocking);

    if (blocking) {
        startTick = tickTimer_GetCount();
    }

    while (1) {
        Lock_Lock(&queue->lock, 100, "MsgQueue_Send");

        result = EnqueueInternal(queue, msg);

        Lock_Unlock(&queue->lock, "MsgQueue_Send");

        if (result) {
            queue->stats.total_enqueued++;
            if (queue->count > queue->stats.max_usage) {
                queue->stats.max_usage = queue->count;
                QUEUE_SEND_PRINT("Queue '%s' new max usage=%d", queue->name, queue->stats.max_usage);
            }
            QUEUE_SEND_PRINT("Send SUCCESS - queue='%s', caller='%s', total_enqueued=%d",
                        queue->name, caller ? caller : "unknown", queue->stats.total_enqueued);
            return true;
        }

        if (!blocking) {
            queue->stats.total_dropped++;
            QUEUE_SEND_PRINT("Send FAILED (non-blocking) - queue='%s', caller='%s', total_dropped=%d",
                        queue->name, caller ? caller : "unknown", queue->stats.total_dropped);
            return false;
        }

        if (queue->config.timeout_ms > 0) {
            if ((tickTimer_GetCount() - startTick) >= queue->config.timeout_ms) {
                QUEUE_SEND_PRINT("Send TIMEOUT - queue='%s', caller='%s', timeout=%dms",
                            queue->name, caller ? caller : "unknown", queue->config.timeout_ms);
                return false;
            }
        }

        for (volatile int i = 0; i < 100; i++);
    }
}

bool MsgQueue_SendUrgent(msg_queue_t *queue, msg_t *msg, const char *caller)
{
    bool result;
    bool prio_enabled;

    if (queue == NULL || msg == NULL) return false;

    QUEUE_SEND_PRINT("Urgent send request - queue='%s', caller='%s', type=%d, len=%d",
                queue->name, caller ? caller : "unknown", msg->type, msg->len);

    Lock_Lock(&queue->lock, 100, "MsgQueue_SendUrgent");

    prio_enabled = queue->config.priority_enabled;
    queue->config.priority_enabled = true;
    msg->priority = MSG_PRIO_URGENT;

    result = EnqueueInternal(queue, msg);

    queue->config.priority_enabled = prio_enabled;

    Lock_Unlock(&queue->lock, "MsgQueue_SendUrgent");

    if (result) {
        queue->stats.total_enqueued++;
        QUEUE_SEND_PRINT("Urgent send SUCCESS - queue='%s', caller='%s'", queue->name, caller ? caller : "unknown");
    } else {
        queue->stats.total_dropped++;
        QUEUE_SEND_PRINT("Urgent send FAILED (queue full) - queue='%s', caller='%s'",
                    queue->name, caller ? caller : "unknown");
    }

    return result;
}

bool MsgQueue_Receive(msg_queue_t *queue, msg_t *msg, uint32_t timeoutMs, const char *caller)
{
    uint64_t startTick;

    if (queue == NULL || msg == NULL) return false;

    QUEUE_RECV_PRINT("Receive request - queue='%s', caller='%s', timeout=%dms, current_count=%d",
                queue->name, caller ? caller : "unknown", timeoutMs, queue->count);

    startTick = tickTimer_GetCount();

    while (1) {
        Lock_Lock(&queue->lock, 100, "MsgQueue_Receive");

        if (queue->count > 0) {
            memcpy(msg, &queue->buffer[queue->head], sizeof(msg_t));
            uint16_t old_head = queue->head;
            queue->head = (queue->head + 1) % queue->size;
            queue->count--;
            Lock_Unlock(&queue->lock, "MsgQueue_Receive");

            queue->stats.total_dequeued++;

            QUEUE_RECV_PRINT("Receive SUCCESS - queue='%s', caller='%s', pos=%d, type=%d, len=%d, prio=%s, remaining=%d, total_dequeued=%d",
                        queue->name, caller ? caller : "unknown", old_head, msg->type, msg->len,
                        GetPriorityStr(msg->priority), queue->count, queue->stats.total_dequeued);
            return true;
        }

        Lock_Unlock(&queue->lock, "MsgQueue_Receive");

        if (timeoutMs == 0) {
            QUEUE_RECV_PRINT("Receive FAILED (empty, non-blocking) - queue='%s', caller='%s'",
                        queue->name, caller ? caller : "unknown");
            return false;
        }

        if ((tickTimer_GetCount() - startTick) >= timeoutMs) {
            QUEUE_RECV_PRINT("Receive TIMEOUT - queue='%s', caller='%s', timeout=%dms",
                        queue->name, caller ? caller : "unknown", timeoutMs);
            return false;
        }

        for (volatile int i = 0; i < 100; i++);
    }
}

bool MsgQueue_Peek(msg_queue_t *queue, msg_t *msg)
{
    if (queue == NULL || msg == NULL) return false;

    Lock_Lock(&queue->lock, 100, "MsgQueue_Peek");

    if (queue->count == 0) {
        Lock_Unlock(&queue->lock, "MsgQueue_Peek");
        QUEUE_RECV_PRINT("Peek FAILED (empty) - queue='%s'", queue->name);
        return false;
    }

    memcpy(msg, &queue->buffer[queue->head], sizeof(msg_t));

    Lock_Unlock(&queue->lock, "MsgQueue_Peek");

    QUEUE_RECV_PRINT("Peek SUCCESS - queue='%s', type=%d, len=%d", queue->name, msg->type, msg->len);
    return true;
}

uint16_t MsgQueue_GetCount(msg_queue_t *queue)
{
    uint16_t count;

    if (queue == NULL) return 0;

    Lock_Lock(&queue->lock, 100, "MsgQueue_GetCount");
    count = queue->count;
    Lock_Unlock(&queue->lock, "MsgQueue_GetCount");

    return count;
}

void MsgQueue_GetStats(msg_queue_t *queue, queue_stats_t *stats)
{
    if (queue == NULL || stats == NULL) return;

    Lock_Lock(&queue->lock, 100, "MsgQueue_GetStats");
    memcpy(stats, &queue->stats, sizeof(queue_stats_t));
    Lock_Unlock(&queue->lock, "MsgQueue_GetStats");

    QUEUE_STATS_PRINT("Queue '%s' stats - enqueued=%d, dequeued=%d, dropped=%d, max_usage=%d, overflow=%d",
                queue->name, stats->total_enqueued, stats->total_dequeued,
                stats->total_dropped, stats->max_usage, stats->overflow_count);
}

void MsgQueue_Clear(msg_queue_t *queue)
{
    if (queue == NULL) return;

    QUEUE_SEND_PRINT("Queue '%s' CLEAR - clearing %d messages", queue->name, queue->count);

    Lock_Lock(&queue->lock, 100, "MsgQueue_Clear");
    queue->head = 0;
    queue->tail = 0;
    queue->count = 0;
    Lock_Unlock(&queue->lock, "MsgQueue_Clear");
}

uint16_t MsgQueue_DeleteByType(msg_queue_t *queue, uint16_t type)
{
    uint16_t deleted = 0;
    uint16_t i, new_tail;
    msg_t temp_buffer[256];

    if (queue == NULL) return 0;

    QUEUE_SEND_PRINT("Delete by type - queue='%s', type=%d", queue->name, type);

    Lock_Lock(&queue->lock, 100, "MsgQueue_DeleteByType");

    new_tail = 0;
    for (i = 0; i < queue->count; i++) {
        uint16_t idx = (queue->head + i) % queue->size;
        if (queue->buffer[idx].type != type) {
            memcpy(&temp_buffer[new_tail++], &queue->buffer[idx], sizeof(msg_t));
        } else {
            deleted++;
        }
    }

    for (i = 0; i < new_tail; i++) {
        memcpy(&queue->buffer[i], &temp_buffer[i], sizeof(msg_t));
    }

    queue->head = 0;
    queue->tail = new_tail;
    queue->count = new_tail;

    Lock_Unlock(&queue->lock, "MsgQueue_DeleteByType");

    QUEUE_SEND_PRINT("Deleted %d messages of type %d from queue '%s', remaining=%d",
                deleted, type, queue->name, queue->count);
    return deleted;
}

bool MsgQueue_WaitNotEmpty(msg_queue_t *queue, uint32_t timeoutMs)
{
    uint64_t startTick;

    if (queue == NULL) return false;

    QUEUE_RECV_PRINT("Wait not empty - queue='%s', timeout=%dms", queue->name, timeoutMs);

    startTick = tickTimer_GetCount();

    while (MsgQueue_GetCount(queue) == 0) {
        if (timeoutMs > 0 && (tickTimer_GetCount() - startTick) >= timeoutMs) {
            QUEUE_RECV_PRINT("Wait not empty TIMEOUT - queue='%s'", queue->name);
            return false;
        }
        for (volatile int i = 0; i < 100; i++);
    }

    QUEUE_RECV_PRINT("Wait not empty SUCCESS - queue='%s' now has %d messages",
                queue->name, MsgQueue_GetCount(queue));
    return true;
}
