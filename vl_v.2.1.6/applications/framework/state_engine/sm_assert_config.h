/**
 * @file    sm_assert_config.h
 * @brief   用户自定义断言配置
 * @note    在编译选项中添加 -DSM_ASSERT_CONFIG_FILE=\"sm_assert_config.h\"
 *          即可启用此配置，覆盖默认断言行为
 */

#ifndef __SM_ASSERT_CONFIG_H
#define __SM_ASSERT_CONFIG_H

/* ============================================================
 * 选择断言模式
 * ============================================================ */

/* 方式1: 使用 RT-Thread 断言 */
#define SM_USE_RT_ASSERT   1
#define SM_USE_STD_ASSERT  0

/* 方式2: 使用标准断言 */
// #define SM_USE_RT_ASSERT   0
// #define SM_USE_STD_ASSERT  1

/* 方式3: 禁用断言 */
// #define SM_USE_RT_ASSERT   0
// #define SM_USE_STD_ASSERT  0


/* ============================================================
 * 可选：完全自定义断言宏
 * 如果定义了 SM_ASSERT 和 SM_ASSERT_MSG，上述配置将被忽略
 * ============================================================ */

// #include <rtthread.h>
// #define SM_ASSERT(expr) \
//     do { \
//         if (!(expr)) { \
//             rt_kprintf("ASSERT: %s %s:%d\n", #expr, __FILE__, __LINE__); \
//             NVIC_SystemReset(); \
//         } \
//     } while(0)
//
// #define SM_ASSERT_MSG(expr, msg) \
//     do { \
//         if (!(expr)) { \
//             rt_kprintf("ASSERT: %s (%s) %s:%d\n", #expr, msg, __FILE__, __LINE__); \
//             NVIC_SystemReset(); \
//         } \
//     } while(0)

#endif /* __SM_ASSERT_CONFIG_H */
