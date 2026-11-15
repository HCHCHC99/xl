#ifndef _HC32F46X_FLASH_H_
#define _HC32F46X_FLASH_H_

#include "hc32_ll_efm.h"

// ========== 调试输出宏定义 ==========
// 在Keil中定义 FLASH_DEBUG_ENABLE 来开启调试输出
// 方法：Project -> Options for Target -> C/C++ -> Define 中添加 FLASH_DEBUG_ENABLE
#ifdef FLASH_DEBUG_ENABLE
    #define FLASH_DEBUG(fmt, ...)    MAIN_D(fmt, ##__VA_ARGS__)
#else
    #define FLASH_DEBUG(fmt, ...)
#endif

// 定义测试地址（确保在Flash可用区域内，避开程序区域）
#define TEST_FLASH_ADDR 0x00020000

// FLASH操作状态
typedef enum
{
    HC32FLASH_OK = 0,           // 操作完成
    HC32FLASH_BUSY = 1,         // 忙
    HC32FLASH_COLERR = 2,       // 读写访问错误
    HC32FLASH_PGMISMTCH = 3,    // 单编程回读错误
    HC32FLASH_PGAERR = 4,       // 编程对齐错误
    HC32FLASH_WPRERR = 5,       // 写保护错误
    HC32FLASH_PEWERR = 6,       // 在擦写不许可模式下擦写FLASH
} HC32FLASH_STATUS;

// 函数声明
HC32FLASH_STATUS HC32FLASH_GetStatus(void);
HC32FLASH_STATUS HC32FLASH_EraseSector(uint32_t u32Addr);
HC32FLASH_STATUS HC32FLASH_WritedWord_NoCheck(uint32_t u32Addr, uint32_t data);
HC32FLASH_STATUS HC32FLASH_WritedWord_Check(uint32_t u32Addr, uint32_t data);
uint32_t HC32FLASH_ReaddWord(uint32_t u32Addr);
void flash_test_example(void);

#endif
