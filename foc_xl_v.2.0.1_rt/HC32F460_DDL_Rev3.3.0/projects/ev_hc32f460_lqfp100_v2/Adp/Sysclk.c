/**
 *******************************************************************************
 * @file  Sysclk.c
 * @brief 系统滴答定时器驱动实现
 * @note  明确区分"时钟源使能"和"系统时钟源选择"
 *******************************************************************************
 */

#include "Sysclk.h"
#include "hc32_ll_utility.h"
#include "hc32_ll.h"

/*==============================================================================
 * 静态函数声明 - 全部条件编译，避免未引用警告
 *============================================================================*/

static void SysTick_NVIC_Config(void);
static int32_t SysTick_Clock_Enable(void);

#if (SYSTICK_ENABLE_PLL == 1UL)
static int32_t SysTick_PLL_Init(void);
#endif

#if (SYSTICK_ENABLE_XTAL == 1UL)
static int32_t SysTick_XTAL_Init(void);
#endif

#if (SYSTICK_ENABLE_XTAL32 == 1UL)
static int32_t SysTick_XTAL32_Init(void);
#endif

#if (SYSTICK_ENABLE_HRC == 1UL)
static int32_t SysTick_HRC_Init(void);
#endif

#if (SYSTICK_ENABLE_MRC == 1UL)
static int32_t SysTick_MRC_Init(void);
#endif

#if (SYSTICK_ENABLE_LRC == 1UL)
static int32_t SysTick_LRC_Init(void);
#endif

/*==============================================================================
 * 函数实现
 *============================================================================*/

void SysTick_Handler(void)
{}

/**
 * @brief SysTick定时器初始化(唯一对外接口)
 * @param 无
 * @retval int32_t 0:成功, -1:失败
 */
int32_t Systick_config(void)
{
    int32_t i32Ret;
    uint32_t u32Reload;
    stc_clock_freq_t stcClkFreq;

    /* 1. 解除写保护 */
    LL_PERIPH_WE(LL_PERIPH_ALL);

    /* 2. 使能各时钟源(仅使能, 不选择) */
    i32Ret = SysTick_Clock_Enable();
    if (i32Ret != LL_OK) {
        return -1;
    }

    /* 3. 配置总线时钟分频 */
    CLK_SetClockDiv(CLK_BUS_CLK_ALL,
                    SYSTICK_HCLK_DIV |
                    SYSTICK_EXCLK_DIV |
                    SYSTICK_PCLK0_DIV |
                    SYSTICK_PCLK1_DIV |
                    SYSTICK_PCLK2_DIV |
                    SYSTICK_PCLK3_DIV |
                    SYSTICK_PCLK4_DIV);

    /* 4. 等待PLL稳定（如果使能了PLL） */
#if (SYSTICK_ENABLE_PLL == 1UL)
    /* 等待MPLL ready */
    while (SET != CLK_GetStableStatus(CLK_STB_FLAG_PLL)) {
        ;
    }
#endif

    /* 5. 配置SRAM等待周期（高频需要） */
    SRAM_SetWaitCycle(SRAM_SRAMH, SRAM_WAIT_CYCLE0, SRAM_WAIT_CYCLE0);
    SRAM_SetWaitCycle((SRAM_SRAM12 | SRAM_SRAM3 | SRAM_SRAMR), SRAM_WAIT_CYCLE1, SRAM_WAIT_CYCLE1);

    /* 6. 配置Flash等待周期（200MHz需要5个等待周期） */
    (void)EFM_SetWaitCycle(EFM_WAIT_CYCLE5);

    /* 7. 配置GPIO读取等待周期 */
    GPIO_SetReadWaitCycle(GPIO_RD_WAIT3);

    /* 8. 切换驱动能力（高频需要） */
    (void)PWC_HighSpeedToHighPerformance();

    /* 9. 设置系统时钟源 */
    CLK_SetSysClockSrc(SYSTICK_SYSCLK_SRC);

    /* 10. 获取当前系统时钟频率 */
    (void)CLK_GetClockFreq(&stcClkFreq);
    SystemCoreClock = stcClkFreq.u32SysclkFreq;

    /* 11. 配置缓存 */
    EFM_CacheRamReset(ENABLE);
    EFM_CacheRamReset(DISABLE);
    EFM_CacheCmd(ENABLE);

    /* 12. 配置SysTick定时器 */
    /* 注意：SystemCoreClock = 200,000,000 Hz */
    /* u32Reload = 200,000,000 / 1000 - 1 = 199,999 */
    u32Reload = (SystemCoreClock / 1000UL) * SYSTICK_INT_INTERVAL_MS - 1UL;

    /* 检查重装载值是否超过24位最大值 */
    if (u32Reload > SysTick_LOAD_RELOAD_Msk) {
        CLOCK_DEBUG("Error: Reload value %lu exceeds 24-bit max (0x%lX)",
                    u32Reload, SysTick_LOAD_RELOAD_Msk);
        return -1;
    }

    CLOCK_DEBUG("SysTick reload value: %lu (expected 199999 for 200MHz)", u32Reload);

    SysTick->LOAD = u32Reload;
    SysTick->VAL = 0UL;
    SysTick->CTRL = (SYSTICK_CLK_SRC << SysTick_CTRL_CLKSOURCE_Pos) |
                    SysTick_CTRL_TICKINT_Msk |
                    SysTick_CTRL_ENABLE_Msk;

    /* 13. 配置中断优先级（低优先级，让其他中断优先） */
    SysTick_NVIC_Config();

    /* 14. 施加写保护 */
    LL_PERIPH_WP(LL_PERIPH_EFM  | LL_PERIPH_SRAM);

    /* 15. 打印配置结果 */
    Print_All_Clock_Freq();

    return 0;
}

/**
 * @brief 时钟源使能(只使能, 不选择)
 * @param 无
 * @retval int32_t 0:成功, 其他:失败
 */
static int32_t SysTick_Clock_Enable(void)
{
    int32_t i32Ret = LL_OK;

    /* 1. 使能HRC */
    #if (SYSTICK_ENABLE_HRC == 1UL)
    i32Ret = SysTick_HRC_Init();
    if (i32Ret != LL_OK) return i32Ret;
    #endif

    /* 2. 使能MRC */
    #if (SYSTICK_ENABLE_MRC == 1UL)
    i32Ret = SysTick_MRC_Init();
    if (i32Ret != LL_OK) return i32Ret;
    #endif

    /* 3. 使能LRC */
    #if (SYSTICK_ENABLE_LRC == 1UL)
    i32Ret = SysTick_LRC_Init();
    if (i32Ret != LL_OK) return i32Ret;
    #endif

    /* 4. 使能XTAL */
    #if (SYSTICK_ENABLE_XTAL == 1UL)
    i32Ret = SysTick_XTAL_Init();
    if (i32Ret != LL_OK) return i32Ret;
    #endif

    /* 5. 使能XTAL32 */
    #if (SYSTICK_ENABLE_XTAL32 == 1UL)
    i32Ret = SysTick_XTAL32_Init();
    if (i32Ret != LL_OK) return i32Ret;
    #endif

    /* 6. 使能PLL */
    #if (SYSTICK_ENABLE_PLL == 1UL)
    i32Ret = SysTick_PLL_Init();
    if (i32Ret != LL_OK) return i32Ret;
    #endif

    return LL_OK;
}

/**
 * @brief PLL初始化
 * @param 无
 * @retval int32_t 0:成功, 其他:失败
 */
#if (SYSTICK_ENABLE_PLL == 1UL)
static int32_t SysTick_PLL_Init(void)
{
    int32_t i32Ret;
    stc_clock_pll_init_t stcPLLInit;

    (void)CLK_PLLStructInit(&stcPLLInit);

    stcPLLInit.PLLCFGR = 0UL;
    stcPLLInit.PLLCFGR_f.PLLSRC = SYSTICK_PLL_SRC;
    stcPLLInit.PLLCFGR_f.PLLM = SYSTICK_PLL_M - 1UL;
    stcPLLInit.PLLCFGR_f.PLLN = SYSTICK_PLL_N - 1UL;
    stcPLLInit.PLLCFGR_f.PLLP = SYSTICK_PLL_P - 1UL;
    stcPLLInit.PLLCFGR_f.PLLQ = SYSTICK_PLL_Q - 1UL;
    stcPLLInit.PLLCFGR_f.PLLR = SYSTICK_PLL_R - 1UL;
    stcPLLInit.u8PLLState = CLK_PLL_ON;

    i32Ret = CLK_PLLInit(&stcPLLInit);
    return i32Ret;
}
#endif

/**
 * @brief 外部晶振初始化
 * @param 无
 * @retval int32_t 0:成功, 其他:失败
 */
#if (SYSTICK_ENABLE_XTAL == 1UL)
static int32_t SysTick_XTAL_Init(void)
{
    stc_clock_xtal_init_t stcXtalInit;

    (void)CLK_XtalStructInit(&stcXtalInit);

    stcXtalInit.u8State = CLK_XTAL_ON;
    stcXtalInit.u8Mode = CLK_XTAL_MD_OSC;
    stcXtalInit.u8Drv = CLK_XTAL_DRV_ULOW;
    stcXtalInit.u8StableTime = CLK_XTAL_STB_2MS;

    return CLK_XtalInit(&stcXtalInit);
}
#endif

/**
 * @brief 32.768kHz外部晶振初始化
 * @param 无
 * @retval int32_t 0:成功, 其他:失败
 */
#if (SYSTICK_ENABLE_XTAL32 == 1UL)
static int32_t SysTick_XTAL32_Init(void)
{
    stc_clock_xtal32_init_t stcXtal32Init;

    (void)CLK_Xtal32StructInit(&stcXtal32Init);

    stcXtal32Init.u8State = CLK_XTAL32_ON;
    stcXtal32Init.u8Drv = CLK_XTAL32_DRV_MID;
    stcXtal32Init.u8Filter = CLK_XTAL32_FILTER_ALL_MD;

    return CLK_Xtal32Init(&stcXtal32Init);
}
#endif

/**
 * @brief HRC初始化
 * @param 无
 * @retval int32_t 0:成功, 其他:失败
 */
#if (SYSTICK_ENABLE_HRC == 1UL)
static int32_t SysTick_HRC_Init(void)
{
    return CLK_HrcCmd(ENABLE);
}
#endif

/**
 * @brief MRC初始化
 * @param 无
 * @retval int32_t 0:成功, 其他:失败
 */
#if (SYSTICK_ENABLE_MRC == 1UL)
static int32_t SysTick_MRC_Init(void)
{
    return CLK_MrcCmd(ENABLE);
}
#endif

/**
 * @brief LRC初始化
 * @param 无
 * @retval int32_t 0:成功, 其他:失败
 */
#if (SYSTICK_ENABLE_LRC == 1UL)
static int32_t SysTick_LRC_Init(void)
{
    return CLK_LrcCmd(ENABLE);
}
#endif

/**
 * @brief SysTick中断优先级配置
 * @param 无
 * @retval 无
 */
static void SysTick_NVIC_Config(void)
{
    NVIC_SetPriority(SysTick_IRQn, 15U);
}

void Print_All_Clock_Freq(void)
{
    stc_clock_freq_t stcClkFreq;
    int32_t i32Ret;

    /* 1. 调用库函数获取所有频率 */
    i32Ret = CLK_GetClockFreq(&stcClkFreq);

    /* 2. 检查是否调用成功 */
    if (i32Ret != LL_OK) {
        CLOCK_DEBUG("CLK_GetClockFreq failed! Error code: %d", i32Ret);
        return;
    }

    /* 3. 打印所有时钟频率 */
    CLOCK_OUT("========== System Clock Frequencies ==========");
    CLOCK_OUT("System Clock (SYSCLK) : %u Hz", stcClkFreq.u32SysclkFreq);
    CLOCK_OUT("HCLK (AHB Clock)      : %u Hz", stcClkFreq.u32HclkFreq);
    CLOCK_OUT("PCLK0 (APB0 Clock)    : %u Hz", stcClkFreq.u32Pclk0Freq);
    CLOCK_OUT("PCLK1 (APB1 Clock)    : %u Hz", stcClkFreq.u32Pclk1Freq);
    CLOCK_OUT("PCLK2 (APB2 Clock)    : %u Hz", stcClkFreq.u32Pclk2Freq);
    CLOCK_OUT("PCLK3 (APB3 Clock)    : %u Hz", stcClkFreq.u32Pclk3Freq);
    CLOCK_OUT("PCLK4 (APB4 Clock)    : %u Hz", stcClkFreq.u32Pclk4Freq);
    CLOCK_OUT("External Clock (EXCLK): %u Hz", stcClkFreq.u32ExclkFreq);
    CLOCK_OUT("==============================================");

    /* 可选：如果需要更详细的调试信息，可以用CLOCK_DEBUG打印 */
    CLOCK_DEBUG("Clock frequency acquisition completed successfully");
}
/*******************************************************************************
 * EOF
 ******************************************************************************/
