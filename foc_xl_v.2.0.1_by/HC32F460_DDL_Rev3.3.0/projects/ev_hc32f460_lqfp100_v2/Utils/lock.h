#ifndef __LOCK_H__
#define __LOCK_H__

#include "stdint.h"
#include "stdbool.h"
#include "rtt_manager.h"

/*=============================================================================
 * 调试宏定义（旧名称兼容）
 * 开关在 rtt_manager.h 中统一管理：LOCK_xxx_PRINT
 *============================================================================*/
#ifdef LOCK_INIT_PRINT
    #define LOCK_INIT_PRINT(fmt, ...)    MAIN_D("[LOCK_INIT] " fmt, ##__VA_ARGS__)
#else
    #define LOCK_INIT_PRINT(fmt, ...)    ((void)0)
#endif

#ifdef LOCK_LOCK_PRINT
    #define LOCK_LOCK_PRINT(fmt, ...)    MAIN_D("[LOCK_LOCK] " fmt, ##__VA_ARGS__)
#else
    #define LOCK_LOCK_PRINT(fmt, ...)    ((void)0)
#endif

#ifdef LOCK_TRY_PRINT
    #define LOCK_TRY_PRINT(fmt, ...)     MAIN_D("[LOCK_TRY] " fmt, ##__VA_ARGS__)
#else
    #define LOCK_TRY_PRINT(fmt, ...)     ((void)0)
#endif

#ifdef LOCK_UNLOCK_PRINT
    #define LOCK_UNLOCK_PRINT(fmt, ...)  MAIN_D("[LOCK_UNLOCK] " fmt, ##__VA_ARGS__)
#else
    #define LOCK_UNLOCK_PRINT(fmt, ...)  ((void)0)
#endif

/* 锁类型 */
typedef enum {
    LOCK_TYPE_MUTEX = 0,    /* 互斥锁 */
    LOCK_TYPE_SPIN = 1,     /* 自旋锁 */
    LOCK_TYPE_RECURSIVE = 2 /* 递归锁 */
} lock_type_t;

/* 锁状态 */
typedef enum {
    LOCK_FREE = 0,
    LOCK_BUSY = 1
} lock_state_t;

/* 锁结构体 */
typedef struct {
    volatile lock_state_t state;
    volatile uint32_t owner_task;
    volatile uint32_t lock_count;
    volatile uint32_t wait_count;
    lock_type_t type;
    uint32_t lock_time;      /* 记录加锁时的时间戳（调试用） */
} mutex_t;

/* 初始化锁 */
void Lock_Init(mutex_t *lock, lock_type_t type, const char *name);

/* 尝试获取锁（非阻塞） */
bool Lock_TryLock(mutex_t *lock, const char *caller);

/* 获取锁（阻塞，带超时） */
bool Lock_Lock(mutex_t *lock, uint32_t timeoutMs, const char *caller);

/* 释放锁 */
void Lock_Unlock(mutex_t *lock, const char *caller);

/* 获取锁状态 */
bool Lock_IsLocked(mutex_t *lock);

/* 获取等待者数量 */
uint32_t Lock_GetWaitCount(mutex_t *lock);

#endif
