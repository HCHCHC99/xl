/**
 * @file    sm_assert.h
 * @brief   独立断言公共模块
 * @note    所有需要断言功能的模块统一包含此文件
 *
 * 断言行为通过编译宏控制：
 *   - SM_USE_RT_ASSERT: 使用 RT-Thread 断言
 *   - SM_USE_STD_ASSERT: 使用标准断言
 *   - 两者都为 0: 禁用断言
 *
 * 用户可在编译选项中或通过 sm_assert_config.h 自定义
 */

#ifndef __SM_ASSERT_H
#define __SM_ASSERT_H

#include <stdint.h>

/* ============================================================
 * 1. 断言配置注入点
 * ============================================================ */
#ifdef SM_ASSERT_CONFIG_FILE
    #include SM_ASSERT_CONFIG_FILE
#else
    /* 否则包含默认配置文件 */
    #include "sm_assert_config.h"
#endif


/* ============================================================
 * 2. 断言宏定义
 * ============================================================ */

#ifndef SM_ASSERT

    #if SM_USE_RT_ASSERT
        /* 使用 RT-Thread 断言 */
        #include <rtthread.h>

        /* 检测 RT_ASSERT_MSG 是否可用 */
        #ifdef RT_ASSERT_MSG
            #define SM_ASSERT(expr)           RT_ASSERT(expr)
            #define SM_ASSERT_MSG(expr, msg)  RT_ASSERT_MSG(expr, msg)
        #else
            /* 【2026-10-27 修复 · D-61】RT-Thread 4.1.1 没有 RT_ASSERT_MSG，
             * 原实现回退到标准库 assert()，带来两个严重问题：
             *
             * ① 栈开销远超预期。newlib 的 assert → __assert_func →
             *    fiprintf → _vfiprintf_r(236 B) → __smakebuf_r(88 B)
             *    → _malloc_r → rt_malloc → rt_mutex_release ... 
             *    实测该链 692 B，比 rt_kprintf 链（约 200 B）贵 3.4 倍，
             *    而且中途会 malloc（在中断上下文或小栈线程里极危险）。
             *
             * ② 与工程预期不符。配置写的是 SM_USE_RT_ASSERT=1，作者
             *    本意是走 RT-Thread 的断言机制，实际却用了 std assert。
             *
             * 现改为直接调用 rt_assert_handler：语义一致（打印后停机），
             * 但只走 rt_kprintf 链，且不 malloc。
             *
             * 注：RT_ASSERT(expr) 是存在的，但它不带自定义消息，
             *     且同样走 rt_assert_handler，故统一用后者以便带 msg。 */
            #define SM_ASSERT(expr) \
                do { \
                    if (!(expr)) { \
                        rt_assert_handler(#expr, __func__, __LINE__); \
                    } \
                } while (0)

            #define SM_ASSERT_MSG(expr, msg) \
                do { \
                    if (!(expr)) { \
                        rt_assert_handler((msg), __func__, __LINE__); \
                    } \
                } while (0)
        #endif

    #elif SM_USE_STD_ASSERT
        /* 使用标准断言 */
        #include <assert.h>
        #define SM_ASSERT(expr)           assert(expr)
        #define SM_ASSERT_MSG(expr, msg)  assert((expr) && (msg))

    #else
        /* 禁用断言（生产环境） */
        #define SM_ASSERT(expr)           ((void)0)
        #define SM_ASSERT_MSG(expr, msg)  ((void)0)

    #endif

#endif /* SM_ASSERT */


/* ============================================================
 * 3. 通用辅助断言宏（所有模块统一使用）
 * ============================================================ */

#ifndef SM_ASSERT_PTR
    #define SM_ASSERT_PTR(ptr)       SM_ASSERT_MSG((ptr) != NULL, "NULL pointer")
#endif

#ifndef SM_ASSERT_STATE
    #define SM_ASSERT_STATE(state, max) \
        SM_ASSERT_MSG((state) < (max), "State out of range")
#endif

#ifndef SM_ASSERT_TABLE
    #define SM_ASSERT_TABLE(table, size) \
        SM_ASSERT_MSG((table) != NULL && (size) > 0, "Table invalid")
#endif

#ifndef SM_ASSERT_INITIALIZED
    #define SM_ASSERT_INITIALIZED(flag) \
        SM_ASSERT_MSG((flag), "Module not initialized")
#endif

#ifndef SM_ASSERT_RANGE
    #define SM_ASSERT_RANGE(val, min, max) \
        SM_ASSERT_MSG((val) >= (min) && (val) <= (max), "Value out of range")
#endif

#ifndef SM_ASSERT_TRUE
    #define SM_ASSERT_TRUE(expr)     SM_ASSERT_MSG((expr), "Expression false")
#endif

#ifndef SM_ASSERT_FALSE
    #define SM_ASSERT_FALSE(expr)    SM_ASSERT_MSG(!(expr), "Expression true")
#endif

#ifndef SM_ASSERT_EQ
    #define SM_ASSERT_EQ(a, b)       SM_ASSERT_MSG((a) == (b), "Values not equal")
#endif

#ifndef SM_ASSERT_NEQ
    #define SM_ASSERT_NEQ(a, b)      SM_ASSERT_MSG((a) != (b), "Values equal")
#endif


/* ============================================================
 * 4. 断言状态查询
 *    用于调试，不产生额外代码
 * ============================================================ */

#ifndef SM_ASSERT_ENABLED
    #if SM_USE_RT_ASSERT || SM_USE_STD_ASSERT
        #define SM_ASSERT_ENABLED    1
    #else
        #define SM_ASSERT_ENABLED    0
    #endif
#endif

#endif /* __SM_ASSERT_H */
