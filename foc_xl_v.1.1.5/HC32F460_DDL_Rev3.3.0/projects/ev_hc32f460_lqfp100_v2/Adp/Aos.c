/**
 *******************************************************************************
 * @file  Aos.c
 * @brief AOS (阿波罗事件路由系统) 适配层
 *******************************************************************************
 */

#include "Aos.h"
#include "rtt_log.h"
/*==============================================================================
 * 静态函数
 *============================================================================*/

/*==============================================================================
 * 函数实现
 *============================================================================*/

/**
 * @brief  AOS 初始化
 */
void AOS_Init(void)
{
    /* 使能 AOS 时钟 */
    FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_AOS, ENABLE);

    /* 第一级触发：TMR0的CMP_B事件触发ADC1序列A */
    AOS_Connect(AOS_ADC1_0, EVT_SRC_TMR0_1_CMP_B);

    /* 第二级触发：ADC1转换完成事件触发DMA */
    /* 注意：使用正确的DMA目标宏 */
    AOS_Connect(AOS_DMA1_1, EVT_SRC_ADC1_EOCA);  // 对应DMA通道1 -> ADC CH4
    AOS_Connect(AOS_DMA1_2, EVT_SRC_ADC1_EOCA);  // 对应DMA通道2 -> ADC CH5

    /* 打印配置信息 */
    // MAIN_D("AOS configured: TMR0->ADC1, ADC1->DMA1_CH1/CH2\r\n");
}

/**
 * @brief  AOS 反初始化
 */
/**
 * @brief  AOS 初始�?(BEMF 专用): TMR4_3 UDF -> ADC1, ADC1 EOCA -> DMA1 CH0/1/2/3
 * @note   TMR4_3 UDF (underflow) = PWM 周期中点 (三角波 peak)
 *         此时所有开关管远离切换沿，适合 ADC 采样反电动势
 */
void AOS_InitForBemf(void)
{
    /* 使能 AOS 时钟 */
    FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_AOS, ENABLE);

    /* TMR4_3 SCMP0 (PWM周期中点) -> ADC1 序列A 硬件触发 */
    AOS_Connect(AOS_ADC1_0, EVT_SRC_TMR4_3_SCMP0);

    /* ADC1 序列A 转换完成 (EOCA) -> DMA1 CH0/1/2/3 (并行搬运 4 通道数据) */
    AOS_Connect(AOS_DMA1_0, EVT_SRC_ADC1_EOCA);   /* DMA1 CH0 -> ADC1 DR0 (M_BEMF/中性点) */
    AOS_Connect(AOS_DMA1_1, EVT_SRC_ADC1_EOCA);   /* DMA1 CH1 -> ADC1 DR1 (U_BEMF) */
    AOS_Connect(AOS_DMA1_2, EVT_SRC_ADC1_EOCA);   /* DMA1 CH2 -> ADC1 DR2 (V_BEMF) */
    AOS_Connect(AOS_DMA1_3, EVT_SRC_ADC1_EOCA);   /* DMA1 CH3 -> ADC1 DR3 (W_BEMF) */

    MAIN_D("[AOS] BEMF routing: TMR4_3_SCMP0 -> ADC1_SEQA, EOCA -> DMA1_CH0/1/2/3\r\n");
}

/**
 * @brief  AOS routing for current sensing (I module)
 *         TMR4_3 SCMP2 (PWM peak) -> ADC1 SEQ_B trigger
 * @note   Uses AOS_ADC1_1 (TRGSEL1), independent from BEMF which uses AOS_ADC1_0 (TRGSEL0).
 */
void AOS_InitForCurrent(void)
{
    /* Enable AOS clock */
    FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_AOS, ENABLE);

    /* TMR4_3 SCMP2 (PWM peak) -> ADC1 trigger input 1 (SEQ_B) */
    AOS_Connect(AOS_ADC1_1, EVT_SRC_TMR4_3_SCMP2);

    MAIN_D("[AOS] Current routing: TMR4_3_SCMP2 -> AOS_ADC1_1 (ADC1_TRGSEL1)\r\n");
}

void AOS_DeInit(void)
{
    /* 禁用 AOS 时钟 */
    FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_AOS, DISABLE);
}

/**
 * @brief  AOS 使能（AOS 模块本身没有全局使能位，此函数保留为接口）
 */
void AOS_Enable(void)
{
    FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_AOS, ENABLE);
}

/**
 * @brief  AOS 禁用
 */
void AOS_Disable(void)
{
    FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_AOS, DISABLE);
}

/**
 * @brief  连接事件源到目标
 * @param  u32Target   目标 @ref AOS_Target_Select
 * @param  enSource    事件源 @ref en_event_src_t
 */
void AOS_Connect(uint32_t u32Target, en_event_src_t enSource)
{
    /* 先确保 AOS 时钟已使能 */
    AOS_Enable();

    /* 设置触发事件源 */
    AOS_SetTriggerEventSrc(u32Target, enSource);
}

/**
 * @brief  断开事件源与目标的连接
 * @param  u32Target   目标 @ref AOS_Target_Select
 */
void AOS_Disconnect(uint32_t u32Target)
{
    /* 将事件源设置为 0（无触发） */
    AOS_SetTriggerEventSrc(u32Target, (en_event_src_t)0);
}

/**
 * @brief  使能通用触发
 * @param  u32Target          目标 @ref AOS_Target_Select
 * @param  u32CommonTrigger   通用触发ID @ref AOS_COMM_TRIG1/2
 */
void AOS_EnableCommonTrigger(uint32_t u32Target, uint32_t u32CommonTrigger)
{
    AOS_CommonTriggerCmd(u32Target, u32CommonTrigger, ENABLE);
}

/**
 * @brief  禁用通用触发
 * @param  u32Target          目标 @ref AOS_Target_Select
 * @param  u32CommonTrigger   通用触发ID @ref AOS_COMM_TRIG1/2
 */
void AOS_DisableCommonTrigger(uint32_t u32Target, uint32_t u32CommonTrigger)
{
    AOS_CommonTriggerCmd(u32Target, u32CommonTrigger, DISABLE);
}

/**
 * @brief  软件触发
 */
void AOS_SoftwareTrigger(void)
{
    AOS_SW_Trigger();
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
