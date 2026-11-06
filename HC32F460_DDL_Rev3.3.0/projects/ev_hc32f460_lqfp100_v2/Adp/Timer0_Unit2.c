/**
 *******************************************************************************
 * @file  Timer0_Unit2.c
 * @brief Timer0单元2的周期中断驱动 (包含通道A和通道B)
 *
 * 分频说明：本单元使用 PCLK1/64 分频（与 Unit1 的 DIV1 不同）。
 *   1ms 周期在 64 分频下计数裕量充足，降低计数频率可减少抖动；
 *   Unit1 因需微秒级时间戳才用 DIV1。
 *******************************************************************************
 */

#include "Timer0_Unit2.h"
#include "TickTimer.h"
#include "hw_board_pins.h"
#include "foc_protect.h"    /* 1ms 过流保护判断 */
#include "encoder_calib.h"
/*==============================================================================
 * 静态变量
 *============================================================================*/

static uint16_t s_saved_count[2] = {0, 0};
static uint32_t s_target_period[2] = {1000, 1000};

/* 1ms 任务累计次数，Watch 观察用；正常每秒 +1000 */
volatile uint32_t g_tmr0_2_1ms_cnt = 0u;

/*---------------------------------------------------------------------------
 * 1ms 用户任务（挂法 A：跑在 CHB 分支，和 CHA 的 tickTimer_Update 互不干扰）
 *---------------------------------------------------------------------------*/
static void My_1ms_Task(void)
{
    g_tmr0_2_1ms_cnt++;
    /* 过流保护集中判断（快/慢双窗口 + 恢复） */
    Foc_Protect_Task1ms();
    /* TODO: 按键扫描 / 慢速上报 ... */
    /* 编码器电角度对齐非阻塞状态机（1ms 推进） */
    EncoderCalib_Task1ms();
}
/*==============================================================================
 * 静态函数
 *============================================================================*/

static void TMR0_Unit2_IRQHandler(void);
static uint32_t TMR0_GetChannelConfig(TMR0_Channel2_t ch, uint32_t *int_src, uint32_t *int_flag, uint32_t *int_type);

/*==============================================================================
 * 宏定义
 *============================================================================*/

#define USEC_TO_COUNT(us, clockFreqInHz) (uint16_t)(((uint64_t)(us) * (clockFreqInHz)) / 1000000U)

/*==============================================================================
 * 函数实现
 *============================================================================*/

static uint32_t TMR0_GetChannelConfig(TMR0_Channel2_t ch, uint32_t *int_src, uint32_t *int_flag, uint32_t *int_type)
{
    if (ch == TMR0_CHANNEL_A_2) {
        *int_src = INT_SRC_TMR0_2_CMP_A;
        *int_flag = TMR0_FLAG_CMP_A;
        *int_type = TMR0_INT_CMP_A;
        return TMR0_CH_A;
    } else {
        *int_src = INT_SRC_TMR0_2_CMP_B;
        *int_flag = TMR0_FLAG_CMP_B;
        *int_type = TMR0_INT_CMP_B;
        return TMR0_CH_B;
    }
}

int32_t TMR0_Unit2_Init(TMR0_Channel2_t ch, uint32_t period_us, ReCount2_t cmd)
{
    stc_tmr0_init_t stcTmr0Init;
    stc_irq_signin_config_t stcIrqSignConfig;
    stc_clock_freq_t stcClkFreq;
    uint32_t u32Pclk1, u32Compare;
    uint32_t timer_ch, int_src, int_flag, int_type;
    static uint8_t s_irq_registered = 0;  /* 静态变量记录是否已注册 */
    uint8_t ch_idx = (ch == TMR0_CHANNEL_A_2) ? 0 : 1;

    s_target_period[ch_idx] = period_us;
    timer_ch = TMR0_GetChannelConfig(ch, &int_src, &int_flag, &int_type);

    FCG_Fcg2PeriphClockCmd(TMR0_UNIT2_CLK, ENABLE);

    CLK_GetClockFreq(&stcClkFreq);
    u32Pclk1 = stcClkFreq.u32Pclk1Freq;

    u32Compare = USEC_TO_COUNT(period_us, (u32Pclk1 / 64UL));
    u32Compare = u32Compare - 1UL;

    (void)TMR0_StructInit(&stcTmr0Init);
    stcTmr0Init.u32ClockSrc = TMR0_CLK_SRC_INTERN;
    stcTmr0Init.u32ClockDiv = TMR0_CLK_DIV64;
    stcTmr0Init.u32Func = TMR0_FUNC_CMP;
    stcTmr0Init.u16CompareValue = (uint16_t)u32Compare;

    if (LL_OK != TMR0_Init(TMR0_UNIT2, timer_ch, &stcTmr0Init)) {
        return -1;
    }

    TMR0_SetCountValue(TMR0_UNIT2, timer_ch, s_saved_count[ch_idx]);
    TMR0_ClearStatus(TMR0_UNIT2, int_flag);
    TMR0_IntCmd(TMR0_UNIT2, int_type, ENABLE);

    /* 只在第一次初始化时注册中断 */
    if (!s_irq_registered) {
        stcIrqSignConfig.enIntSrc = int_src;  /* 注意：这里需要选择其中一个中断源 */
        stcIrqSignConfig.enIRQn = TMR0_UNIT2_IRQn;
        stcIrqSignConfig.pfnCallback = &TMR0_Unit2_IRQHandler;

        if (LL_OK != INTC_IrqSignIn(&stcIrqSignConfig)) {
            return -1;
        }

        NVIC_ClearPendingIRQ(TMR0_UNIT2_IRQn);
        NVIC_SetPriority(TMR0_UNIT2_IRQn, DDL_IRQ_PRIO_04);
        NVIC_EnableIRQ(TMR0_UNIT2_IRQn);

        s_irq_registered = 1;
    }

    TMR0_Start(TMR0_UNIT2, timer_ch);

    if (cmd == TICK_RESET_2) {
        tickTimer_Init();
    }

    return 0;
}

int32_t TMR0_Unit2_DeInit(TMR0_Channel2_t ch)
{
    uint32_t timer_ch, int_src, int_flag, int_type;
    uint8_t ch_idx = (ch == TMR0_CHANNEL_A_2) ? 0 : 1;

    timer_ch = TMR0_GetChannelConfig(ch, &int_src, &int_flag, &int_type);

    s_saved_count[ch_idx] = TMR0_GetCountValue(TMR0_UNIT2, timer_ch);

    TMR0_Stop(TMR0_UNIT2, timer_ch);
    TMR0_IntCmd(TMR0_UNIT2, int_type, DISABLE);
    TMR0_ClearStatus(TMR0_UNIT2, int_flag);

    (void)TMR0_DeInit(TMR0_UNIT2);

    if (ch == TMR0_CHANNEL_B_2) {
        NVIC_DisableIRQ(TMR0_UNIT2_IRQn);
        NVIC_ClearPendingIRQ(TMR0_UNIT2_IRQn);
    }

    return 0;
}

int32_t TMR0_Unit2_Reconfig(TMR0_Channel2_t ch)
{
    stc_clock_freq_t stcClkFreq;
    uint32_t u32Pclk1, u32Compare;
    uint32_t timer_ch, int_src, int_flag, int_type;
    uint8_t ch_idx = (ch == TMR0_CHANNEL_A_2) ? 0 : 1;

    timer_ch = TMR0_GetChannelConfig(ch, &int_src, &int_flag, &int_type);

    TMR0_Stop(TMR0_UNIT2, timer_ch);

    CLK_GetClockFreq(&stcClkFreq);
    u32Pclk1 = stcClkFreq.u32Pclk1Freq;

    u32Compare = USEC_TO_COUNT(s_target_period[ch_idx], (u32Pclk1 / 64UL));
    u32Compare = u32Compare - 1UL;

    TMR0_SetCompareValue(TMR0_UNIT2, timer_ch, (uint16_t)u32Compare);
    TMR0_SetCountValue(TMR0_UNIT2, timer_ch, s_saved_count[ch_idx]);
    TMR0_Start(TMR0_UNIT2, timer_ch);

    return 0;
}

void TMR0_Unit2_Start(TMR0_Channel2_t ch)
{
    uint32_t timer_ch = (ch == TMR0_CHANNEL_A_2) ? TMR0_CH_A : TMR0_CH_B;
    TMR0_Start(TMR0_UNIT2, timer_ch);
}

void TMR0_Unit2_Stop(TMR0_Channel2_t ch)
{
    uint32_t timer_ch = (ch == TMR0_CHANNEL_A_2) ? TMR0_CH_A : TMR0_CH_B;
    TMR0_Stop(TMR0_UNIT2, timer_ch);
}

uint16_t TMR0_Unit2_GetCount(TMR0_Channel2_t ch)
{
    uint32_t timer_ch = (ch == TMR0_CHANNEL_A_2) ? TMR0_CH_A : TMR0_CH_B;
    return TMR0_GetCountValue(TMR0_UNIT2, timer_ch);
}

static void TMR0_Unit2_IRQHandler(void)
{
    if (TMR0_GetStatus(TMR0_UNIT2, TMR0_FLAG_CMP_A) == SET) {
        TMR0_ClearStatus(TMR0_UNIT2, TMR0_FLAG_CMP_A);
        tickTimer_Update();  /* 假设通道A用于系统心跳 */

    }

    if (TMR0_GetStatus(TMR0_UNIT2, TMR0_FLAG_CMP_B) == SET) {
        TMR0_ClearStatus(TMR0_UNIT2, TMR0_FLAG_CMP_B);
        /* 通道B的用户代码可以在这里添加 */
        My_1ms_Task();          /* ← 1ms 任务 */
    }
}

/*******************************************************************************
 * 中断服务函数
 ******************************************************************************/

void TMR0_2_IRQHandler(void)
{
    TMR0_Unit2_IRQHandler();
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
