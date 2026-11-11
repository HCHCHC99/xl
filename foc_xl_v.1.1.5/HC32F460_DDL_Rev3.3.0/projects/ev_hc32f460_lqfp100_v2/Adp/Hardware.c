#include "Hardware.h"
#include "Adc.h"
#include "Aos.h"
#include "Dma.h"
#include "rtt_log.h"

/**
 * ============================================================================
 * @brief  示例：使用 PA6 (ADC1_CH6) 进行 ADC 采集
 *
 * 本示例展示如何结合 Adp 层的三个模块完成 ADC 采集：
 *   1. Adc.c/.h   - ADC 适配层：管理 ADC 实例、配置、启动/停止
 *   2. Aos.c/.h   - AOS 事件路由：连接 TMR0→ADC1→DMA 的事件链
 *   3. Dma.c/.h   - DMA 适配层：管理 DMA 通道、缓冲区、传输完成中断
 *
 * 数据流：
 *   TMR0 (定时触发) → AOS → ADC1 (采样) → AOS → DMA1 (搬运到内存)
 *
 * 两种模式可选（通过 ADC_USE_DMA_MODE 宏切换）：
 *   - 中断模式：ADC 转换完成触发中断，在中断中读取数据
 *   - DMA 模式：ADC 转换完成触发 DMA，自动将数据搬运到缓冲区
 * ============================================================================
 */

/* 选择 PA6 的采集模式：1=DMA模式，0=中断模式 */
#define ADC_USE_DMA_MODE   1

/* PA6 对应的 ADC 通道号：ADC1_CH6 */
#define PA6_ADC_CHANNEL    ADC_CH6
#define PA6_ADC_PORT       GPIO_PORT_A
#define PA6_ADC_PIN        GPIO_PIN_06

/* PA6 的 ADC 实例 ID（由 Adc_Create 返回）*/
static uint8_t s_u8Pa6AdcId = 0xFF;

/* PA6 的 DMA 通道号（DMA 模式使用）*/
#define PA6_DMA_CHANNEL    1   /* 使用 DMA1 通道 1 */
#define PA6_DMA_BUFFER_SIZE 8  /* 缓冲区大小 */

/* ============================================================================
 * 步骤 1：定义 PA6 中断模式回调函数
 * ============================================================================
 * 当中断模式使能时，每次 ADC 转换完成都会调用此回调。
 * 注意：回调在中断上下文中执行，应尽量简短。
 * 在 Keil Debug 时，可在此处打断点，Watch 窗口中观察 u16Voltage 的值。
 * ============================================================================ */
static void Pa6_AdcCallback_Interrupt(uint16_t u16AdcValue)
{
    /* 将 ADC 原始值 (12bit, 0~4095) 转换为电压值 (mV) */
    uint16_t u16Voltage = (uint16_t)(((uint32_t)u16AdcValue * 3300UL) / 4095UL);

    /* 在 Keil Watch 窗口中可观察：u16AdcValue, u16Voltage */
    __nop(); // Debug: 在此处打断点查看 PA6 中断模式 ADC 值

    /* 注意：MAIN_D 在中断中调用可能影响实时性，调试时可取消注释 */
    // MAIN_D("PA6[INT] Raw=%u, %umV\r\n", u16AdcValue, u16Voltage);
}

/* ============================================================================
 * 步骤 2：定义 DMA 传输完成回调函数
 * ============================================================================
 * 当 DMA 完成一次块传输后调用。
 * 注意：回调在 DMA 中断上下文中执行，应尽量简短。
 * 在 Keil Debug 时，可在此处打断点，观察 DMA 缓冲区中的数据。
 * ============================================================================ */
static void Pa6_DmaCallback(void)
{
    /* 此回调由 Dma 层在 DMA 传输完成中断中调用 */
    /* 实际数据读取在 main loop 中通过 Adc_GetLatestValue() 完成 */
    __nop(); // Debug: 在此处打断点，确认 DMA 传输完成
}

/* ============================================================================
 * 步骤 3：初始化 PA6 ADC 采集（在 Hardware_Init 中调用）
 * ============================================================================
 * 此函数展示完整的初始化流程：
 *   1. 创建 ADC 实例（Adc_Create）
 *   2. 初始化 ADC 和 DMA（Adc_Init 内部自动完成）
 *   3. 启动 ADC 转换（Adc_Start）
 *
 * 注意：AOS 初始化（AOS_Init）必须在调用此函数之前完成，
 * 因为 AOS_Init 中配置了 TMR0→ADC1→DMA 的事件路由。
 * ============================================================================ */
static void Pa6_Adc_Init(void)
{
    stc_adc_config_t stcAdcConfig;

    /* ---- 填充 ADC 配置 ---- */
    stcAdcConfig.u8Channel = PA6_ADC_CHANNEL;          /* ADC1_CH6 */
    stcAdcConfig.stcPin.u8Port = PA6_ADC_PORT;         /* GPIO_PORT_A */
    stcAdcConfig.stcPin.u8Pin = PA6_ADC_PIN;           /* GPIO_PIN_06 */

#if (ADC_USE_DMA_MODE == 1)
    /* ===== DMA 模式 ===== */
    stcAdcConfig.enMode = ADC_MODE_DMA;
    stcAdcConfig.pfnCallback = NULL;                    /* DMA 模式不需要 ADC 回调 */
    stcAdcConfig.stcDmaConfig.u16BufferSize = PA6_DMA_BUFFER_SIZE;
    stcAdcConfig.stcDmaConfig.u8DmaChannel = PA6_DMA_CHANNEL;

    MAIN_D("[PA6_ADC] Initializing PA6 (CH6) in DMA mode, DMA1 CH%d, buffer=%d\r\n",
           PA6_DMA_CHANNEL, PA6_DMA_BUFFER_SIZE);
#else
    /* ===== 中断模式 ===== */
    stcAdcConfig.enMode = ADC_MODE_INTERRUPT;
    stcAdcConfig.pfnCallback = Pa6_AdcCallback_Interrupt;

    MAIN_D("[PA6_ADC] Initializing PA6 (CH6) in Interrupt mode\r\n");
#endif

    /* ---- 创建 ADC 实例 ---- */
    s_u8Pa6AdcId = Adc_Create(&stcAdcConfig);
    if (s_u8Pa6AdcId == 0xFF) {
        MAIN_D("[PA6_ADC] ERROR: Failed to create ADC instance!\r\n");
        return;
    }

    /* ---- 初始化 ADC（内部会自动初始化 DMA）---- */
    Adc_Init();

    /* ---- 启动 ADC 转换（启动 TMR0 定时触发）---- */
    Adc_Start();

    MAIN_D("[PA6_ADC] PA6 ADC initialized successfully! ADC_ID=%d\r\n", s_u8Pa6AdcId);
}

/* ============================================================================
 * 步骤 4：在 main loop 中轮询读取 PA6 ADC 数据
 * ============================================================================
 * 此函数应在 main loop 中周期性调用，用于获取最新的 ADC 值。
 * 在 Keil Debug 时，可在此处打断点，Watch 窗口中观察各变量的值。
 * ============================================================================ */
void Pa6_Adc_Process(void)
{
    static uint32_t s_u32LastPrintMs = 0;
    uint32_t u32Now = tickTimer_GetCount();

    /* 每 500ms 打印一次 PA6 ADC 值 */
    if ((u32Now - s_u32LastPrintMs) >= 500) {
        s_u32LastPrintMs = u32Now;

        if (s_u8Pa6AdcId != 0xFF) {
            /* 获取最新 ADC 原始值 */
            uint16_t u16Raw = Adc_GetLatestValue(s_u8Pa6AdcId);

            /* 转换为电压 (mV) */
            uint16_t u16Voltage = (uint16_t)(((uint32_t)u16Raw * 3300UL) / 4095UL);

#if (ADC_USE_DMA_MODE == 1)
            /* DMA 模式：还可以获取平均值 */
            uint16_t u16Avg = Adc_GetAverageValue(s_u8Pa6AdcId);
            uint16_t u16AvgMv = (uint16_t)(((uint32_t)u16Avg * 3300UL) / 4095UL);
            uint32_t u32Samples = Adc_GetSampleCount(s_u8Pa6AdcId);

            MAIN_D("PA6 DMA: Raw=%4u, %3umV, Avg=%4u, %3umV, Cnt=%lu\r\n",
                   u16Raw, u16Voltage, u16Avg, u16AvgMv, u32Samples);
#else

            /* 中断模式 */
            MAIN_D("PA6 INT: Raw=%4u, %3umV\r\n", u16Raw, u16Voltage);
#endif

            /* 在 Keil Watch 窗口中可观察：u16Raw, u16Voltage */
            __nop(); // Debug: 在此处打断点查看 PA6 ADC 值
        }
    }
}

/* ============================================================================
 * 步骤 5：AOS 事件路由配置说明
 * ============================================================================
 * AOS_Init() 在 Hardware_Init() 中被调用，它配置了以下事件路由：
 *
 *   第一级：TMR0_1 CMP_B 事件 → ADC1 序列 A 触发
 *     - AOS_Connect(AOS_ADC1_0, EVT_SRC_TMR0_1_CMP_B);
 *     - 作用：TMR0 定时器每溢出一次，触发 ADC1 启动一次序列 A 转换
 *     - 定时周期由 Adc.c 中的 Timer0_Config(1000) 决定（默认 1ms）
 *
 *   第二级：ADC1 EOCA 事件 → DMA1 通道 1/2 触发
 *     - AOS_Connect(AOS_DMA1_1, EVT_SRC_ADC1_EOCA);  // 对应 DMA1 CH1
 *     - AOS_Connect(AOS_DMA1_2, EVT_SRC_ADC1_EOCA);  // 对应 DMA1 CH2
 *     - 作用：ADC1 转换完成后，自动触发 DMA 将数据搬运到内存缓冲区
 *
 * 注意：如果使用中断模式（ADC_USE_DMA_MODE=0），第二级 AOS 路由不需要配置，
 * 因为数据在 ADC 中断中直接读取。但当前 AOS_Init() 中已配置了 DMA 路由，
 * 不影响中断模式的使用（DMA 通道未使能时不会响应）。
 * ============================================================================ */

/* ============================================================================
 * 步骤 6：DMA 适配层使用说明
 * ============================================================================
 * 当使用 DMA 模式时，Adc_Init() 内部会自动完成以下操作：
 *
 *   1. 为每个 DMA 模式的 ADC 实例调用 Dma_Create() 创建 DMA 实例
 *      - 配置源地址为 ADC 数据寄存器 (ADC1->DR6)
 *      - 配置目标地址为 DMA 内部自动分配的缓冲区
 *      - 配置传输方向：外设→内存
 *      - 配置传输模式：循环传输（重复模式）
 *      - 配置数据宽度：16bit（ADC 为 12bit）
 *      - 配置块大小：由 ADC_DMA_BUFFER_SIZE 决定（默认 8）
 *
 *   2. 调用 Dma_Init() 初始化所有 DMA 通道
 *      - 分配 DMA 缓冲区（malloc）
 *      - 配置 DMA 硬件寄存器
 *      - 注册 DMA 中断处理函数
 *
 *   3. 调用 Dma_StartAll() 启动所有 DMA 通道
 *      - 使能 DMA 通道，等待 ADC 转换完成触发
 *
 * 数据流完整路径：
 *   TMR0 (1ms定时)
 *     → AOS 路由 → ADC1 启动序列A转换
 *     → ADC1 采样 PA6 (CH6)
 *     → ADC1 转换完成 (EOCA)
 *     → AOS 路由 → DMA1 CH1 启动传输
 *     → DMA1 将 ADC1->DR6 的数据搬运到内存缓冲区
 *     → DMA 传输完成中断 → 更新数据标志
 *     → main loop 中 Adc_GetLatestValue() 读取最新值
 * ============================================================================ */

/* ============================================================================
 * 原有的回调函数保留（兼容性）
 * ============================================================================ */
void MyAdcCallback_PA1(uint16_t u16AdcValue)
{
    uint16_t u16Voltage = (u16AdcValue * 3300) / 4096;
    __nop(); // Debug: PA1 ADC 回调
}

void MyAdcCallback_PA2(uint16_t u16AdcValue)
{
    uint16_t u16Voltage = (u16AdcValue * 3300) / 4096;
    __nop(); // Debug: PA2 ADC 回调
}

void MyAdcCallback_PA6(uint16_t u16AdcValue)
{
    uint16_t u16Voltage = (u16AdcValue * 3300) / 4096;
    __nop(); // Debug: PA6 ADC 回调（旧版，新版使用 Pa6_AdcCallback_Interrupt）
}

/* ============================================================================
 * 母线电压检测（PA4 / ADC1_CH4）—— 软件触发单次转换
 * ============================================================================
 * 硬件分压与设计说明见 Hardware.h 的 VBUS 段。
 * 此处刻意不走 Adp/Adc.c 的框架：Adc_Init() 会把 CM_TMR0_1 的 CH_B 重配为 1ms，
 * 而那正是 Hardware_Init 里配好的 500us 系统节拍。母线电压变化极慢，主循环里
 * 每 VBUS_UPDATE_MS 采一次足够，无需 TMR0 / AOS / DMA。
 * ============================================================================ */

volatile uint16_t g_vbus_raw = 0U;      /* PA4 原始 ADC 码值（0~4095） */
volatile uint16_t g_vbus_mv = 0U;       /* 最新母线电压 (mV) */
volatile float    g_vbus_v  = 0.0F;     /* 最新母线电压 (V) */

static uint8_t s_vbus_ready = 0U;       /* 1 = 已完成初始化 */
static uint8_t s_vbus_first = 1U;       /* 1 = 首拍直通，不做 EMA */

/**
 * @brief  Set PA4 to analog mode (ADC pin must be analog)
 */
static void Vbus_SetPinAnalogMode(void)
{
    stc_gpio_init_t stcGpioInit;

    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinAttr = PIN_ATTR_ANALOG;
    LL_PERIPH_WE(LL_PERIPH_GPIO);
    (void)GPIO_Init(VBUS_ADC_PORT, VBUS_ADC_PIN, &stcGpioInit);
    LL_PERIPH_WP(LL_PERIPH_GPIO);
}

/**
 * @brief  Initialize the Vbus ADC (call once in Hardware_Init)
 */
void Vbus_Adc_Init(void)
{
    stc_adc_init_t stcAdcInit;

    /* 1. Enable ADC1 peripheral clock */
    FCG_Fcg3PeriphClockCmd(VBUS_ADC_PERIPH_CLK, ENABLE);

    /* 2. Sequence A single-shot; software start (TRGENA stays disabled so the
     *    TMR0_1_CMP_B -> AOS_ADC1_0 hardware trigger cannot disturb it) */
    (void)ADC_StructInit(&stcAdcInit);
    stcAdcInit.u16ScanMode   = ADC_MD_SEQA_SINGLESHOT;
    stcAdcInit.u16Resolution = ADC_RESOLUTION_12BIT;
    stcAdcInit.u16DataAlign  = ADC_DATAALIGN_RIGHT;
    (void)ADC_Init(VBUS_ADC_UNIT, &stcAdcInit);

    /* 3. Analog pin + enable channel + sample window */
    Vbus_SetPinAnalogMode();
    ADC_ChCmd(VBUS_ADC_UNIT, ADC_SEQ_A, VBUS_ADC_CHANNEL, ENABLE);
    ADC_SetSampleTime(VBUS_ADC_UNIT, VBUS_ADC_CHANNEL, VBUS_ADC_SAMPLE_TIME);

    s_vbus_ready = 1U;
    VBUS_DBG("init: ADC1 CH%d, div %d/%d kOhm, sample %d ADCLK",
             (int)VBUS_ADC_CHANNEL,
             (int)(VBUS_DIV_R_HIGH_OHM / 1000.0F),
             (int)(VBUS_DIV_R_LOW_OHM / 1000.0F),
             (int)VBUS_ADC_SAMPLE_TIME);
}

/**
 * @brief  Poll one conversion and refresh g_vbus_mv / g_vbus_v.
 *         Call periodically in the main loop.
 */
void Vbus_Adc_Process(void)
{
    static uint32_t s_u32LastMs = 0U;
    uint32_t u32Now;
    uint32_t u32Wait;
    uint16_t u16Raw;
    float    fVadc;
    float    fVbus;

    if (s_vbus_ready == 0U) {
        return;
    }

    u32Now = tickTimer_GetCount();
    if ((u32Now - s_u32LastMs) < VBUS_UPDATE_MS) {
        return;
    }
    s_u32LastMs = u32Now;

    /* Software start, then poll EOCA (bounded, never block forever) */
    ADC_ClearStatus(VBUS_ADC_UNIT, ADC_FLAG_EOCA);
    if (ADC_Start(VBUS_ADC_UNIT) != LL_OK) {
        return;
    }
    for (u32Wait = 0U; u32Wait < VBUS_ADC_TIMEOUT; u32Wait++) {
        if (ADC_GetStatus(VBUS_ADC_UNIT, ADC_FLAG_EOCA) == SET) {
            break;
        }
    }
    if (ADC_GetStatus(VBUS_ADC_UNIT, ADC_FLAG_EOCA) != SET) {
        VBUS_DBG("read timeout");
        return;
    }

    u16Raw = ADC_GetValue(VBUS_ADC_UNIT, VBUS_ADC_CHANNEL);
    ADC_ClearStatus(VBUS_ADC_UNIT, ADC_FLAG_EOCA);
    g_vbus_raw = u16Raw;                /* 原始码值（未滤波），供 Watch / RTT 观察 */

    /* Vadc -> Vbus: multiply back the divider ratio (R_HIGH + R_LOW) / R_LOW */
    fVadc = ((float)u16Raw * VBUS_ADC_VREF) / VBUS_ADC_FULL_SCALE;
    fVbus = fVadc * ((VBUS_DIV_R_HIGH_OHM + VBUS_DIV_R_LOW_OHM) / VBUS_DIV_R_LOW_OHM);

    /* First sample passes through, then EMA (display/readout only, no control use) */
    if (s_vbus_first != 0U) {
        s_vbus_first = 0U;
        g_vbus_v = fVbus;
    } else {
        g_vbus_v += VBUS_FILT_ALPHA * (fVbus - g_vbus_v);
    }
    g_vbus_mv = (uint16_t)((g_vbus_v * 1000.0F) + 0.5F);
}

/* ============================================================================
 * 主硬件初始化函数
 * ============================================================================ */
void Hardware_Init(void)
{
    /* 1. 系统时钟初始化（必须最先执行） */
    if (Systick_config() != 0) {
        /* 配置失败：时钟频率过高或PLL/晶振初始化失败 */
        while (1);
    }
    TMR0_Unit2_Init(TMR0_CHANNEL_A_2, 1000UL, TICK_RESET_2);
    TMR0_Unit2_Init(TMR0_CHANNEL_B_2, 5000UL, TICK_RESET_2);
    TMR0_Unit1_Init(TMR0_CHANNEL_B_1, 500UL, TICK_RESET_1);
    TMR0_Unit1_Init(TMR0_CHANNEL_A_1, 1000UL, TICK_RESET_1);

    tickTimer_DelayMs(1000);
    // Output_GPIO_Init(GPIO_LED_PORT, GPIO_LED_PIN, GPIO_INIT_HIGH);

    /* 2. AOS 事件路由初始化
     *    配置 TMR0→ADC1→DMA 的事件链
     *    必须在 ADC 初始化之前调用 */
    AOS_Init();

    /* 3. GPIO 初始化 */
    Output_GPIO_Init(GPIO_LED_PORT, GPIO_LED_PIN, GPIO_INIT_HIGH);

    // Output_GPIO_Init(PHU_PORT, PHU_PIN, GPIO_INIT_LOW);
    // Output_GPIO_Init(PHV_PORT, PHV_PIN, GPIO_INIT_LOW);

    // TMR0_Unit2_Reconfig(TMR0_CHANNEL_A_2);
    // TMR0_Unit2_Reconfig(TMR0_CHANNEL_A_1);
    // TMR0_Unit2_Reconfig(TMR0_CHANNEL_B_2);
    // TMR0_Unit2_Reconfig(TMR0_CHANNEL_B_1);
    // TMR0_Unit2_Reconfig(TMR0_CHANNEL_B_2);
    // /* 5. PA6 ADC 采集初始化
    //  *    展示 Adc + Aos + Dma 三层配合使用的完整流程
    //  *
    //  *    数据流：
    //  *      TMR0 (1ms定时)
    //  *        → AOS 路由 → ADC1 启动序列A转换
    //  *        → ADC1 采样 PA6 (CH6)
    //  *        → ADC1 转换完成 (EOCA)
    //  *        → AOS 路由 → DMA1 CH1 启动传输
    //  *        → DMA1 将数据搬运到内存缓冲区
    //  *        → DMA 传输完成中断 → 更新数据标志
    //  *        → main loop 中 Pa6_Adc_Process() 读取并打印
    //  *
    //  *    切换模式：修改 ADC_USE_DMA_MODE 宏
    //  *      = 1: DMA 模式（自动搬运，CPU 负载低）
    //  *      = 0: 中断模式（每次转换触发中断） */
    /* 4. 母线电压检测初始化（PA4 / ADC1_CH4，软件触发单次转换）
     *    不走 Adp/Adc.c 框架：其 Adc_Init() 会把 500us 系统节拍改掉
     *    采样动作在 main loop 里由 Vbus_Adc_Process() 周期触发 */
    Vbus_Adc_Init();

    // Pa6_Adc_Init();

    // MAIN_D("[Hardware] Hardware init complete! PA6 ADC running in %s mode\r\n",
    //        (ADC_USE_DMA_MODE ? "DMA" : "Interrupt"));

}
