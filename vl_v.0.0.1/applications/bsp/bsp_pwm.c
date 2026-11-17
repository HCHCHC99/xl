/**
 * @file    bsp_pwm.c
 * @brief   三相 PWM BSP 层实现（HC32F452 TIM4_3）
 * @note    使用 TIM4 高级定时器实现三相互补 PWM
 *                                  支持死区时间、中心对齐、紧急刹车
 */

#include "bsp_pwm.h"
#include "bsp_hw_init.h"
#include "framework/state_engine/sm_assert.h"
#include <string.h>
#include <rtthread.h>

/* ============================================================
 * HC32F452 底层驱动
 * ============================================================ */
#include "hc32_ll.h"
#include "hc32_ll_tmr4.h"
#include "hc32_ll_gpio.h"
#include "hc32_ll_fcg.h"
#include "hc32_ll_interrupts.h"     /* INTC_IrqSignIn：TMR4 时基中断（2026-11-02） */
#include "board_config.h"           /* BSP_TMR4_3_ISR_IRQ_NUM */
#include "../motor/foc/foc_pwm.h"   /* FOC PWM 配置表入口（2026-11-16 方案 B） */
#include "../motor/foc/foc_mode.h"  /* FOC 模式层：上电默认 mode 0 */

/* 【2026-11-17】ADC 采样已改为 TMR4 EVT0 硬件触发：
 *   触发点由 cfg_foc_table.h 的 FOC_CFG_SAMPLE_POINT 决定，
 *   由 hw_tmr4_pwm_config_adc_trigger() 配置 EVT0，
 *   AOS_ADC1_0 → ADC1 SEQ_A，底层在 bsp_adc_foc.c。
 *   本文件的谷值 ISR 只维护 20kHz 时基，不再软件启动 ADC。 */

/* ============================================================
 * 编译期裁剪
 * ============================================================ */
#define BSP_PWM_USE_EMB_HW_TRIG     0   /* 0: 只用软件触发；               1: 使能硬件触发 */
#define BSP_PWM_USE_BRAKE_LOW       0   /* 0: 刹车后高阻；                   1: 刹车后低电平 */
#define BSP_PWM_USE_DUTY_CACHE      0   /* 0：ISR 里立即写寄存器 ；1: 三相同步缓存更新 */

/* EMB 功率级封锁总开关（2026-11-17）
 *   0 = 不启用：SOE3 不置位，原封锁代码保留但不执行
 *   1 = 启用：恢复原行为（上电/IDLE/故障时封锁功率级）
 * 置 0 后安全停机只依赖 hw_tmr4_pwm_set_all_low()（六路 OC 关闭）。 */
#ifndef BSP_PWM_EMB_ENABLE
#define BSP_PWM_EMB_ENABLE          0
#endif

/* ============================================================
 * 硬件常量
 * ============================================================ */
#define BSP_PWM_TIM4_UNIT           (CM_TMR4_3)
/* PCLK1 = HCLK / 2 = 200MHz / 2 = 100MHz（board.c: PLL 8M×50÷2=200MHz，CLK_PCLK1_DIV2） */
#define BSP_PWM_TIM4_CLK_HZ         (100000000UL)

/* EMB Group3（TIM4_3 对应） */
#define BSP_PWM_EMB_CTL3            (*(volatile uint32_t *)(0x40017C60UL))
#define BSP_PWM_EMB_PWMLV3          (*(volatile uint32_t *)(0x40017C64UL))
#define BSP_PWM_EMB_SOE3            (*(volatile uint32_t *)(0x40017C68UL))
#define BSP_PWM_EMB_STATCLR3        (*(volatile uint32_t *)(0x40017C70UL))
#define BSP_PWM_EMB_INTEN3          (*(volatile uint32_t *)(0x40017C74UL))
#define BSP_PWM_TMR4_ECER_U3        (*(volatile uint32_t *)(0x40055410UL))
#define BSP_PWM_TMR4_ECSR_U3        (*(volatile uint32_t *)(0x40024CF0UL))
#define BSP_PWM_TMR4_3_CCSR         (*(volatile uint16_t *)(0x40024C48UL))

/* ============================================================
 * 引脚定义
 * ============================================================
 * 六路 PWM 引脚（PB4~PB9）已在 board/board_config.h 统一定义与配置：
 *   PWM_UH / PWM_UL / PWM_VH / PWM_VL / PWM_WH / PWM_WL
 * 归一化到 board_config.c 的 rt_hw_board_pwm_init()，
 * 避免同一引脚在两处重复配置（GPIO_Init 会整段覆盖 PCR）。
 * ============================================================ */

/* ============================================================
 * 【2026-11-02 新增】FOC 微小时基（TMR4_3 周期中断）
 * ------------------------------------------------------------
 * 提供与 PWM 严格同相的 20 kHz 周期计数，供 FOC 使用。
 *
 * 【为什么计数器用 volatile + 32bit】
 *   中断里写、任务里读，必须 volatile 防编译器优化。
 *   32bit 在 Cortex-M4 上是原子读写，单写单读无需临界区。
 *   回绕周期 = 2^32 / 20000 Hz ≈ 59.6 小时，实际不会撞上。
 *
 * 【为什么不用 rt_tick_get() 做时基】
 *   rt_tick 分辨率 1ms，无法满足 FOC 所需的 50 μs。
 *   而且它由 SysTick 产生，与 PWM 无相位关系。
 * ============================================================ */
static volatile uint32_t s_pwm_cycle_cnt;      /* PWM 周期计数，50us/个 */
static volatile uint32_t s_pwm_peak_cnt;       /* 峰值事件计数（诊断用）*/
static volatile uint32_t s_pwm_spurious_cnt;   /* 无标志却进中断的次数（诊断用）*/
static volatile uint8_t  s_pwm_tb_enabled;

/**
 * @brief TMR4_3 周期中断（三角波谷值，20 kHz）
 * @note  本步只做时基计数，不涉及电流采样（方案 A 范围）。
 *        后续 FOC 实现时，电角度外推与电流环计算放这里。
 */
static void bsp_pwm_tmr4_isr(void)
{
    uint16_t ccsr = BSP_PWM_TMR4_3_CCSR;
    uint8_t  is_valley = (uint8_t)((ccsr & TMR4_CCSR_IRQZF) ? 1U : 0U);
    uint8_t  is_peak   = (uint8_t)((ccsr & TMR4_CCSR_IRQPF) ? 1U : 0U);

    /* 先清两个标志（清完再累加，避免退出前又被置位导致重入）*/
    TMR4_ClearStatus(BSP_PWM_TIM4_UNIT,
                     TMR4_FLAG_CNT_VALLEY | TMR4_FLAG_CNT_PEAK);

    /* 【2026-11-03】按标志判定事件类型，而不是"进了中断就 +1"。
     *
     * 原因：OVF(358) 与 UDF(359) 都映射到了本 ISR（定位阶段的双保险），
     * 三角波下峰值/谷值各会产生一次请求，若"进中断就 +1"会变成 40 kHz。
     * 用标志判定后：
     *   - 谷值事件（IRQZF）才累加 20 kHz 主时基      ← FOC 用这个
     *   - 峰值事件（IRQPF）只计 s_peak_cnt，供诊断区分
     * 这样无论哪个源生效，主计数都严格是 20 kHz。 */
    if (is_valley)
    {
        s_pwm_cycle_cnt++;

        /* ADC 采样由 TMR4 EVT0 硬件触发，这里只维护时基计数。
         * 好处：采样点与 PWM 严格同相，不受 ISR 延迟影响。 */
    }
    if (is_peak)
    {
        s_pwm_peak_cnt++;
    }

    /* 两个标志都没置位还进了中断：说明是误触发或标志被别处清了。
     * 计数但不归类，便于诊断发现配置错误。 */
    if ((is_valley == 0U) && (is_peak == 0U))
    {
        s_pwm_spurious_cnt++;
    }
}

/**
 * @brief 取 PWM 周期计数（FOC 时基）
 * @retval 自 PWM 启动以来经过的 PWM 周期数，每 50 μs 加 1
 * @note   回绕周期约 59.6 小时。做时间差时用无符号减法：
 *           uint32_t dcyc = now - prev;   // 自动处理回绕
 */
uint32_t bsp_pwm_get_cycle_cnt(void)
{
    return s_pwm_cycle_cnt;
}

/**
 * @brief 周期计数换算成微秒
 * @param cycles 周期数（可为任意 uint32，差值语义）
 */
uint32_t bsp_pwm_cycle_to_us(uint32_t cycles)
{
    uint32_t freq = foc_pwm_get_frequency();

    /* 频率由 cfg_foc_table.h 决定，不能再写死 20kHz。
     * 例：20kHz → 50μs/周期；10kHz → 100μs/周期。 */
    if (freq == 0U) {
        freq = 20000U;
    }

    return cycles * (1000000U / freq);
}

/**
 * @brief 时基寄存器直读快照（诊断用）
 * @param out [0] cycle_cnt   软件周期计数
 *            [1] CNTR        硬件计数器当前值（0..period，三角波）
 *            [2] CCSR        控制/状态寄存器原始值
 *            [3] STOP 位     bit6，0=在跑 1=停
 *            [4] IRQZEN 位   bit13，谷值中断使能
 *            [5] IRQZF 位    bit14，谷值标志（会自清，抓不到属正常）
 *            [6] MODE 位     bit5，1=三角波
 *            [7] SOE3        EMB 输出封锁位，1=已封锁（安全）
 * @note  直读寄存器而非走 DDL 接口，是为了绕过 DDL 的位域封装，
 *        把"原始值"原样呈现——诊断要的是真值，不是解释后的值。
 */
void bsp_pwm_tb_snapshot(uint32_t out[8])
{
    uint16_t ccsr;

    if (out == RT_NULL) { return; }

    ccsr = BSP_PWM_TMR4_3_CCSR;

    out[0] = s_pwm_cycle_cnt;
    out[1] = (uint32_t)TMR4_GetCountValue(BSP_PWM_TIM4_UNIT);
    out[2] = (uint32_t)ccsr;
    out[3] = (uint32_t)((ccsr & TMR4_CCSR_STOP)    ? 1U : 0U);
    out[4] = (uint32_t)((ccsr & TMR4_CCSR_IRQZEN)  ? 1U : 0U);
    out[5] = (uint32_t)((ccsr & TMR4_CCSR_IRQZF)   ? 1U : 0U);
    out[6] = (uint32_t)((ccsr & TMR4_CCSR_MODE)    ? 1U : 0U);
    out[7] = (uint32_t)(BSP_PWM_EMB_SOE3 & 0x01UL);
}

/**
 * @brief 读 TMR4 硬件计数器当前值
 * @note  不依赖中断，纯粹读寄存器。诊断用。
 */
uint16_t bsp_pwm_tb_get_cntr(void)
{
    return TMR4_GetCountValue(BSP_PWM_TIM4_UNIT);
}

/**
 * @brief 中断链路完整快照（诊断用，2026-11-03 新增）
 *
 * 【为什么要这个】
 * 反复出现"CNTR 在跑、IRQZEN 使能、IRQZF 置位，但 cycle_cnt 恒 0"。
 * 已有的 snapshot 只看了 TMR4 外设侧，没看 INTC 映射与 NVIC 使能，
 * 导致只能靠猜。本接口把断链路上每一环都读出来，一次定位。
 *
 * @param out [0] INTC  SEL98 值（应为 359=INT_SRC_TMR4_3_UDF；复位值 0x1FF）
 *            [1] INTC  IER 寄存器（外设侧总使能，bit 对应向量号所在位）
 *            [2] NVIC  ISER 对应字（Cortex 侧使能）
 *            [3] NVIC  ISPR 对应字（挂起标志，1=有待处理中断）
 *            [4] NVIC  IPR  对应字节（优先级，低 4bit 有效）
 *            [5] TMR4  CCSR 原始值
 *            [6] TMR4  CNTR 值
 *            [7] 期望的 int_src（359=UDF）
 *
 * 【判读表】
 *   SEL98 != 358        → INTC 映射没写进去（IrqSignIn 失败或地址算错）
 *   NVIC_ISER bit == 0  → NVIC 没使能
 *   NVIC_ISPR bit == 1  → 中断已挂起但没进（优先级被 --- 屏蔽？）
 *   IER bit == 0        → HC32 外设侧总开关没开（第二级使能）
 *   全对但仍不进       → 查 TMR4 ISR 源（ISR 寄存器 vs CCSR 标志）
 */
void bsp_pwm_tb_irq_snapshot(uint32_t out[8])
{
    uint32_t irq = (uint32_t)BSP_TMR4_3_ISR_IRQ_NUM;

    if (out == RT_NULL) { return; }

    /* INTC 侧：SEL 是独立成员（SEL0/SEL1/...），不是数组，
     * 用 &SEL0 + 4*n 计算地址。 */
    out[0] = *(volatile uint32_t *)((uint32_t)&CM_INTC->SEL0 + (4U * irq));
    out[1] = (uint32_t)CM_INTC->IER;

    /* Cortex 侧：NVIC 寄存器（只取本向量对应位）*/
    out[2] = (uint32_t)((NVIC->ISER[irq >> 5U] >> (irq & 0x1FU)) & 1U);
    out[3] = (uint32_t)((NVIC->ISPR[irq >> 5U] >> (irq & 0x1FU)) & 1U);
    out[4] = (uint32_t)NVIC->IP[irq];       /* 8bit，低 4bit 是优先级 */

    out[5] = (uint32_t)BSP_PWM_TMR4_3_CCSR;
    out[6] = (uint32_t)TMR4_GetCountValue(BSP_PWM_TIM4_UNIT);
    out[7] = (uint32_t)INT_SRC_TMR4_3_UDF;
}

/**
 * @brief 事件计数快照（诊断用）
 * @param out [0] 谷值计数（主时基，应 20kHz）
 *            [1] 峰值计数（诊断）
 *            [2] 无标志误进中断次数（应恒 0）
 */
void bsp_pwm_tb_event_cnt(uint32_t out[3])
{
    if (out == RT_NULL) { return; }

    out[0] = s_pwm_cycle_cnt;
    out[1] = s_pwm_peak_cnt;
    out[2] = s_pwm_spurious_cnt;
}

/**
 * @brief 微时基是否已使能（诊断用）
 */
uint8_t bsp_pwm_tb_is_enabled(void)
{
    return s_pwm_tb_enabled;
}

/* ============================================================
 * 私有数据
 * ============================================================ */
bsp_pwm_priv_t g_bsp_pwm_priv = {0};

/* ============================================================
 * 内联：EMB 软件刹车 / 恢复（ISR 安全）
 * ============================================================ */
static inline void bsp_pwm_emb_stop(void)
{
#if BSP_PWM_EMB_ENABLE
    BSP_PWM_EMB_SOE3 |= 1UL;
#else
    /* 用户裁定：不启用 EMB 功率级封锁，原代码保留 */
#endif
}

static inline void bsp_pwm_emb_release(void)
{
    BSP_PWM_EMB_SOE3 &= ~1UL;
}

static uint16_t bsp_pwm_duty_to_cmp(uint32_t duty)
{
    uint32_t period = g_bsp_pwm_priv.period_counts;
    uint32_t max    = g_bsp_pwm_priv.max_duty;

    if (duty > max) duty = max;
    return (uint16_t)(period - (period * duty) / max);
}

/* ============================================================
 * 内部：GPIO 引脚配置
 * ============================================================
 * 已上移至 board/board_config.c 的 rt_hw_board_pwm_init()。
 * 本文件不再直接配置引脚，仅负责 TIM4 / EMB 寄存器。
 * ============================================================ */


/* ============================================================
 * EMB 初始化（配置表方式）
 * ============================================================ */
static void bsp_pwm_emb_init(void)
{
    FCG_Fcg2PeriphClockCmd(FCG2_PERIPH_EMB, ENABLE);

    BSP_PWM_EMB_CTL3   = 0x00000000UL;
    BSP_PWM_EMB_INTEN3 = 0x00000000UL;

#if BSP_PWM_USE_BRAKE_LOW
    BSP_PWM_TMR4_ECER_U3 = 0x00000002UL;
#else
    BSP_PWM_TMR4_ECER_U3 = 0x00000001UL;
#endif
    BSP_PWM_TMR4_ECSR_U3 = 0x00000000UL;

    /* 默认不刹车 */
    BSP_PWM_EMB_SOE3 = 0;
}

/* ============================================================
 * TIM4 三相互补 PWM 初始化
 * ============================================================ */
static void bsp_pwm_tim4_init(uint32_t freq_hz, uint32_t deadtime_ns)
{
    stc_tmr4_init_t stcTmr4Init;
    stc_tmr4_oc_init_t stcTmr4OcInit;
    stc_tmr4_pwm_init_t stcTmr4PwmInit;
    un_tmr4_oc_ocmrl_t unOcmrl;
    uint32_t period;
    uint16_t dt_counts;
    uint32_t i;

    /* 1. 周期计数：三角波中心对齐 */
    period = BSP_PWM_TIM4_CLK_HZ / (2UL * freq_hz);
    g_bsp_pwm_priv.period_counts = period;

    /* 2. 死区计数：死区时钟不分频 */
    dt_counts = (uint16_t)((deadtime_ns * (BSP_PWM_TIM4_CLK_HZ / 1000000UL)) / 1000UL);

    /* 3. 计数器初始化 */
    (void)TMR4_StructInit(&stcTmr4Init);
    stcTmr4Init.u16ClockDiv    = TMR4_CLK_DIV1;
    stcTmr4Init.u16PeriodValue = (uint16_t)period;
    (void)TMR4_Init(BSP_PWM_TIM4_UNIT, &stcTmr4Init);

    /* 4. 三角波模式：直接写 CCSR.MODE=1 */
    BSP_PWM_TMR4_3_CCSR |= (uint16_t)0x0020U;   /* MODE = 1 */

    /* 5. 三相 OC + PWM 循环初始化 */
    for (i = 0; i < 3U; i++) {
        uint32_t oc_ch;
        uint32_t pwm_ch;

        if (i == 0U)      { oc_ch = TMR4_OC_CH_UL; pwm_ch = TMR4_PWM_CH_U; }
        else if (i == 1U) { oc_ch = TMR4_OC_CH_VL; pwm_ch = TMR4_PWM_CH_V; }
        else              { oc_ch = TMR4_OC_CH_WL; pwm_ch = TMR4_PWM_CH_W; }

        /* 5.1 OC 初始化：初始 0% 占空比 */
        (void)TMR4_OC_StructInit(&stcTmr4OcInit);
        stcTmr4OcInit.u16CompareValue = (uint16_t)period;   /* 0% */
        (void)TMR4_OC_Init(BSP_PWM_TIM4_UNIT, oc_ch, &stcTmr4OcInit);

        /* 5.2 比较模式：与官方例程一致，OPDCL/OPPKL/OPUCL/OPZRL = INVT */
        unOcmrl.OCMRx_f.OCFDCL  = TMR4_OC_OCF_SET;
        unOcmrl.OCMRx_f.OCFPKL  = TMR4_OC_OCF_SET;
        unOcmrl.OCMRx_f.OCFUCL  = TMR4_OC_OCF_SET;
        unOcmrl.OCMRx_f.OCFZRL  = TMR4_OC_OCF_SET;
        unOcmrl.OCMRx_f.OPDCL   = TMR4_OC_INVT;
        unOcmrl.OCMRx_f.OPPKL   = TMR4_OC_INVT;
        unOcmrl.OCMRx_f.OPUCL   = TMR4_OC_INVT;
        unOcmrl.OCMRx_f.OPZRL   = TMR4_OC_INVT;
        unOcmrl.OCMRx_f.OPNPKL  = TMR4_OC_HOLD;
        unOcmrl.OCMRx_f.OPNZRL  = TMR4_OC_HOLD;
        unOcmrl.OCMRx_f.EOPNDCL = TMR4_OC_HOLD;
        unOcmrl.OCMRx_f.EOPNUCL = TMR4_OC_HOLD;
        unOcmrl.OCMRx_f.EOPDCL  = TMR4_OC_INVT;
        unOcmrl.OCMRx_f.EOPPKL  = TMR4_OC_INVT;
        unOcmrl.OCMRx_f.EOPUCL  = TMR4_OC_INVT;
        unOcmrl.OCMRx_f.EOPZRL  = TMR4_OC_INVT;
        unOcmrl.OCMRx_f.EOPNPKL = TMR4_OC_HOLD;
        unOcmrl.OCMRx_f.EOPNZRL = TMR4_OC_HOLD;
        TMR4_OC_SetLowChCompareMode(BSP_PWM_TIM4_UNIT, oc_ch, unOcmrl);
        TMR4_OC_Cmd(BSP_PWM_TIM4_UNIT, oc_ch, ENABLE);

        /* 5.3 PWM 死区定时器模式 */
        (void)TMR4_PWM_StructInit(&stcTmr4PwmInit);
        stcTmr4PwmInit.u16Mode     = TMR4_PWM_MD_DEAD_TMR;
        stcTmr4PwmInit.u16ClockDiv = TMR4_PWM_CLK_DIV1;
        (void)TMR4_PWM_Init(BSP_PWM_TIM4_UNIT, pwm_ch, &stcTmr4PwmInit);

        /* 5.4 死区时间 */
        TMR4_PWM_SetDeadTimeValue(BSP_PWM_TIM4_UNIT, pwm_ch, TMR4_PWM_PDAR_IDX, dt_counts);
        TMR4_PWM_SetDeadTimeValue(BSP_PWM_TIM4_UNIT, pwm_ch, TMR4_PWM_PDBR_IDX, dt_counts);
    }

    rt_kprintf("[BSP-PWM] TIM4_3 legacy cfg: freq=%uHz, period=%u, dt=%uns(%u cnt)"
               " (will be overridden by cfg_foc_table)\r\n",
               (unsigned)freq_hz, (unsigned)period,
               (unsigned)deadtime_ns, (unsigned)dt_counts);

    /* ============================================================
     * 【2026-11-02 新增】TMR4_3 周期中断 —— FOC 微小时基（方案 A）
     * ------------------------------------------------------------
     * 只做一件事：每 50 μs 累加一个周期计数，为 FOC 提供
     * 与 PWM 严格同相的时间基准。ADC 采样方式本步不动。
     *
     * 【为什么用 VALLEY 而不是 PEAK】
     *   谷值（CNT=0）时刻三相下桥关断、上桥导通，是 FOC 高边
     *   电流采样的标准窗口。后续把此中断接 ADC 硬件触发时可直接复用。
     *
     * 【频率核查】
     *   三角波 0→2500→0，完整周期 5000 计数 @100MHz = 50 μs
     *   只使能 VALLEY ⇒ 每 50 μs 一次 = 20 kHz ✓
     *   若误使能 PEAK+VALLEY 会变 40 kHz，导致时基快一倍。
     *
     * 【清标志顺序】
     *   必须先 TMR4_ClearStatus 再累加，否则标志未清会重复进中断。
     * ============================================================ */
    {
        stc_irq_signin_config_t stcIrqSignin;
        int32_t i32Ret;

        /* 只使能谷值中断（不要 PEAK，否则 40kHz）
         * VALLEY 对应 CCSR.IRQZEN(bit13) */
        TMR4_IntCmd(BSP_PWM_TIM4_UNIT, TMR4_INT_CNT_VALLEY, ENABLE);

        /* 【2026-11-03 修复】中断源必须是 UDF，不是 OVF
         * ------------------------------------------------------------
         * 现象（连续三轮）：CNTR 在跑、IRQZEN=1、IRQZF=1、cycle_cnt 恒 0。
         *
         * 根因：TMR4 的 CCSR.IRQZEN/IRQZF 是**外设内部**的计数比较中断，
         * 它对外发出的中断请求要经 INTC 的 int_src 编号转发。
         * 而 int_src 的 GOVF/GUDF 与三角波事件是**一一对应**的：
         *
         *   CNT: 0 ──上数──> 2500 ──下数──> 0
         *        到达 2500 转向  => GOVF (OverFlow)  <-> PEAK  (IRQPF)
         *        到达 0    转向  => GUDF (UnderFlow) <-> VALLEY(IRQZF)
         *
         * 原来写 OVF(358) 却使能 VALLEY：
         *   - VALLEY 请求已使能，但它的出口是 GUDF(359)，没被映射
         *   - OVF(358) 虽映射到 INT098，但其请求受 IRQPEN(bit8)=0 门控
         *     （实测 CCSR=0x6220 bit8=0），从不 assert
         *   两头都不通 ⇒ ISR 永不进，而 IRQZF 因无人清除而一直保持 1。
         *
         * 这解释了这个"三合一"现象，且与实测寄存器值完全吻合。
         *
         * 为什么选 UDF 而不是改用 PEAK 配 OVF：
         *   谷值(CNT=0)是 FOC 高边电流采样的标准窗口，必须保留 VALLEY。
         *   359/32=11，合法向量仍是 INT098..INT103，向量号无需再改。
         * ============================================================ */
        stcIrqSignin.enIntSrc    = INT_SRC_TMR4_3_UDF;      /* 359，谷值/下溢 */
        stcIrqSignin.enIRQn      = BSP_TMR4_3_ISR_IRQ_NUM;
        stcIrqSignin.pfnCallback = &bsp_pwm_tmr4_isr;

        /* 【2026-11-02 教训】这里原来写的是 (void)INTC_IrqSignIn(...)。
         * 它内部有硬件分组范围校验：(src/32)*6+32 .. +37。
         * TMR4_3_OVF=358 → 只允许 INT098..INT103；当时选 INT060
         * 被拒绝，返回 LL_ERR_INVD_PARAM，但返回值被丢弃 ⇒
         * INTSEL 未写、回调未登记，中断永远进不来，且毫无提示。
         * 现在必须检查：失败就打印错误码，绝不让映射失败无声无息。 */
        i32Ret = INTC_IrqSignIn(&stcIrqSignin);
        if (i32Ret != LL_OK)
        {
            rt_kprintf("[BSP-PWM] FATAL: IrqSignIn failed (%d)\n", (int)i32Ret);
            rt_kprintf("          src=%u vec=INT%03d out of allowed window\n",
                       (unsigned)INT_SRC_TMR4_3_UDF,
                       (int)BSP_TMR4_3_ISR_IRQ_NUM);
            rt_kprintf("          rule: vec must be in [(src/32)*6+32, +37]\n");
            rt_kprintf("          => for src=%u use INT%03u..INT%03u\n",
                       (unsigned)INT_SRC_TMR4_3_UDF,
                       (unsigned)((INT_SRC_TMR4_3_UDF / 32U) * 6U + 32U),
                       (unsigned)((INT_SRC_TMR4_3_UDF / 32U) * 6U + 37U));
            s_pwm_tb_enabled = 0U;
            return;                 /* 不使能 NVIC，避免空指针分发 */
        }

        NVIC_ClearPendingIRQ(stcIrqSignin.enIRQn);
        NVIC_SetPriority(stcIrqSignin.enIRQn, BSP_TMR4_3_ISR_IRQ_PRIO);
        NVIC_EnableIRQ(stcIrqSignin.enIRQn);

        /* 【2026-11-03 双保险】再把 OVF(358) 也映射到同一个向量。
         *
         * 理由：三角波下 GOVF/GUDF 两个信号都可能被硬件置起，
         * 但具体哪一个对应 CCSR 的 VALLEY 标志，手册描述不清晰
         * （TL 的 figure 与 DDL 注释有出入）。
         * INTC 允许同一个向量绑定多个 int_src 吗？——不允许，
         * 但允许同一个 int_src 绑多个向量；反过来一个向量只能有一个 int_src。
         * 所以这里改为：把 OVF 也挂到**另一个**向量 INT099，
         * 回调仍是本 ISR。哪个先来都能计数，另一个不会误伤。
         *
         * 成本：多占一个向量号（INT099），无其他副作用。
         * 定位清楚后再删掉多余的那条。 */
        {
            stc_irq_signin_config_t stcAlt = stcIrqSignin;

            stcAlt.enIntSrc    = INT_SRC_TMR4_3_OVF;        /* 358 */
            stcAlt.enIRQn      = INT099_IRQn;               /* 与 098 同组，合法 */
            stcAlt.pfnCallback = &bsp_pwm_tmr4_isr;

            if (INTC_IrqSignIn(&stcAlt) == LL_OK)
            {
                NVIC_ClearPendingIRQ(stcAlt.enIRQn);
                NVIC_SetPriority(stcAlt.enIRQn, BSP_TMR4_3_ISR_IRQ_PRIO);
                NVIC_EnableIRQ(stcAlt.enIRQn);
            }
        }

        s_pwm_tb_enabled = 1U;
    }

    /* ============================================================
     * 【2026-11-02 修复】时基计数器必须常驻运行
     * ------------------------------------------------------------
     * 缺陷现象：t_pwmtime 全程 0，cycle_cnt 恒 0，中断永不进。
     *
     * 根因：bsp_pwm_tim4_init() 只配置寄存器、**从不启动计数器**。
     *       原设计把启动寄托在 bsp_pwm_enable_impl()（set_output>0
     *       时由 bldc_6step 调用），形成死锁：
     *         电机要转 → 需要角度 → 角度更新靠 1ms 定时器
     *         而 FOC 时基要等电机转才启动 → 永不自洽。
     *       而且 dev_motor_init() 这个唯一的另一处启动点，
     *       其调用方 bsp_mod_bldc_ctrl.c 整个文件已 // 注释。
     *
     * 修法：初始化阶段就把计数器跑起来。
     * ============================================================ */
    bsp_pwm_emb_stop();
    TMR4_Start(BSP_PWM_TIM4_UNIT);

    rt_kprintf("[BSP-PWM] timebase ISR enabled: 20kHz (valley), vec=INT%03d\n",
               (int)BSP_TMR4_3_ISR_IRQ_NUM);
    rt_kprintf("[BSP-PWM] TMR4 counter STARTED (emb locked, cycle_cnt begins)\n");
}


///* ============================================================
// * 占空比内部写入：duty(0~max_duty) → 比较值
// * ============================================================ */
//static inline uint16_t bsp_pwm_duty_to_cmp(uint32_t duty)
//{
//    uint32_t period = g_bsp_pwm_priv.period_counts;
//    uint32_t max    = g_bsp_pwm_priv.max_duty;
//
//    if (duty > max) duty = max;
//
//    /* OCCRxl = period 时 0%，OCCRxl = 0 时 100% */
//    return (uint16_t)(period - (period * duty) / max);
//}
//
//static void bsp_pwm_write_duty(uint32_t idx, uint32_t duty)
//{
//    uint16_t cmp = bsp_pwm_duty_to_cmp(duty);
//
//    switch (idx) {
//    case 0: TMR4_OC_SetCompareValue(BSP_PWM_TIM4_UNIT, TMR4_OC_CH_UL, cmp); break;
//    case 1: TMR4_OC_SetCompareValue(BSP_PWM_TIM4_UNIT, TMR4_OC_CH_VL, cmp); break;
//    case 2: TMR4_OC_SetCompareValue(BSP_PWM_TIM4_UNIT, TMR4_OC_CH_WL, cmp); break;
//    default: break;
//    }
//}

/* ============================================================
 * 硬件初始化（由 bsp_hw_init 表调用）
 * ============================================================
 * 只做寄存器级配置：TIM4 时钟 → 三相互补 PWM → EMB
 * 六路引脚的复用功能由 board_config.c 的 rt_hw_board_pwm_init() 完成，
 * 必须在本函数之前执行（见 g_hw_init_steps 顺序）。
 * ============================================================ */
int bsp_pwm_hw_init(void *arg)
{
    const hw_pwm_cfg_t *cfg = (const hw_pwm_cfg_t *)arg;

    if (cfg == RT_NULL) {
        rt_kprintf("[BSP-PWM] hw_init: cfg is NULL\n");
        return -1;
    }

    /* 幂等：重复调用只做空转 */
    if (g_bsp_pwm_priv.initialized) {
        return 0;
    }

    /* 记录硬件真实配置，供 init_impl 与 cfg 比对 */
    g_bsp_pwm_priv.max_duty = cfg->max_duty;
    g_bsp_pwm_priv.braked   = 0;

    LL_PERIPH_WE(LL_PERIPH_FCG);
    FCG_Fcg2PeriphClockCmd(FCG2_PERIPH_TMR4_3, ENABLE);
    LL_PERIPH_WP(LL_PERIPH_FCG);

    /* 【2026-11-02 顺序修正】emb_init 必须在 tim4_init **之前**
     *
     * 缺陷：原来顺序是 tim4_init() 然后 emb_init()。
     * 而 tim4_init() 末尾为了让时基常驻会调用 emb_stop()
     * （把 SOE3 置 1 封锁功率级），紧接着的 emb_init() 又把
     * SOE3 清零（其末尾是 `BSP_PWM_EMB_SOE3 = 0;`）——
     * 顺序反了，封锁被覆盖。
     *
     * 实测证据：t_pwmtime 的 SOE3 显示 0，说明功率级**没有**被封锁。
     * 虽然此刻占空比是 0%（compare=period，六路都不导通），
     * 未观察到实际输出，但这属于"靠巧合安全"，不能留。
     *
     * 正确顺序：先初始化 EMB 寄存器，再由 tim4_init 末尾统一置封锁。
     * tim4_init 不依赖 EMB 已初始化（它只写 TMR4 的 CCSR/OC/死区），
     * 所以交换顺序无副作用。 */
    bsp_pwm_emb_init();
    bsp_pwm_tim4_init(cfg->freq_hz, cfg->deadtime_ns);

    /* ============================================================
     * 【2026-11-16 · 方案 B】TMR4 PWM 最终配置以 cfg_foc_table.h 为准
     * ------------------------------------------------------------
     * bsp_pwm_tim4_init() 保留原六步方波路径（时基 ISR / NVIC 路由 /
     * EMB 封锁），随后由 FOC 配置表覆盖 PWM 几何与死区：
     *   20kHz、互补、下桥臂采样、死区 50ns（FOC_CFG_SELECT = 2）。
     *
     * 覆盖顺序必须是「旧初始化 → FOC 配置」：
     *   hw_tmr4_pwm_config() 内部 TMR4_Init 会写 CCSR 的 CLEAR/STOP，
     *   把谷值中断使能一并清掉，所以配置后要重新使能 VALLEY 再启动。
     *
     * 六步方波代码不删除，只是六路 OC 在 set_all_low() 里被关闭，
     * 即使上层误调 hw_pwm_set_duty() 也不会输出到引脚。
     * ============================================================ */
    bsp_pwm_emb_stop();

    if (foc_pwm_apply_cfg() != 0) {
        rt_kprintf("[BSP-PWM] FOC PWM apply failed\r\n");
        return -1;
    }

    TMR4_IntCmd(BSP_PWM_TIM4_UNIT, TMR4_INT_CNT_VALLEY, ENABLE);
    TMR4_Start(BSP_PWM_TIM4_UNIT);

    /* 六步 ops 的占空比换算依赖该值，保持与 FOC 配置表一致 */
    g_bsp_pwm_priv.period_counts = (uint32_t)foc_pwm_get_period_ticks();

    rt_kprintf("[BSP-PWM] TMR4_3 overridden by cfg_foc_table: %lu Hz, dt=%u ns\r\n",
               (unsigned long)foc_pwm_get_frequency(),
               (unsigned)foc_pwm_get_deadtime_ns());

    g_bsp_pwm_priv.initialized = 1;

    /* 上电默认进入 FOC mode 0（IDLE：六路全低 + EMB 封锁）。
     * 后续通过 msh 命令 `focmode 0/1` 切换。 */
    Foc_Mode_Init();

    return 0;
}

/* ============================================================
 * Ops 实现
 * ============================================================ */
static rt_err_t bsp_pwm_init_impl(hw_pwm_t *self)
{
    SM_ASSERT_PTR(self);

    /* 硬件由 bsp_hw_init 表统一初始化，此处只校验不改寄存器 */
    if (!g_bsp_pwm_priv.initialized) {
        rt_kprintf("[BSP-PWM] %s: hardware not initialised\n", self->cfg->name);
        return -RT_ERROR;
    }

    /* 对象侧 cfg 与硬件实际配置不一致时告警（不失败，以硬件为准） */
    if (self->cfg->max_duty != g_bsp_pwm_priv.max_duty) {
        rt_kprintf("[BSP-PWM] warn: %s max_duty cfg=%u hw=%u\n",
                   self->cfg->name,
                   (unsigned)self->cfg->max_duty,
                   (unsigned)g_bsp_pwm_priv.max_duty);
    }

    return RT_EOK;
}

static rt_err_t bsp_pwm_deinit_impl(hw_pwm_t *self)
{
    SM_ASSERT_PTR(self);
    (void)self;
    if (!g_bsp_pwm_priv.initialized) return RT_EOK;

    bsp_pwm_emb_stop();          /* 先刹车 */
    TMR4_Stop(BSP_PWM_TIM4_UNIT);
    g_bsp_pwm_priv.initialized = 0;
    rt_kprintf("[BSP-PWM] Deinitialized\n");
    return RT_EOK;
}

static rt_err_t bsp_pwm_set_duty_impl(hw_pwm_t *self, uint8_t phase, uint32_t duty)
{
    SM_ASSERT_PTR(self);
    uint16_t cmp;

    (void)self;

    /* FOC 模式已接管 TMR4_3：旧六步路径（bg_sched → motor_update →
     * openloop_update → bldc_6step_set_output）每 5ms 会把 OCCR 写回 0%，
     * 覆盖 focmode 设置的占空比。这里直接忽略旧路径的写入。 */
    if (Foc_Mode_PwmOwned()) {
        return RT_EOK;
    }

    if (!g_bsp_pwm_priv.initialized) return -RT_ERROR;

    cmp = bsp_pwm_duty_to_cmp(duty);

#if BSP_PWM_USE_DUTY_CACHE
    /* 只写缓存，由 flush 统一写寄存器，保证三相同步 */
    g_bsp_pwm_priv.duty_cache[idx] = (duty > self->max_duty) ? self->max_duty : duty;
#else
    switch (phase) {
    case HW_PWM_PHASE_U:
        TMR4_OC_SetCompareValue(BSP_PWM_TIM4_UNIT, TMR4_OC_CH_UL, cmp);
        break;
    case HW_PWM_PHASE_V:
        TMR4_OC_SetCompareValue(BSP_PWM_TIM4_UNIT, TMR4_OC_CH_VL, cmp);
        break;
    case HW_PWM_PHASE_W:
        TMR4_OC_SetCompareValue(BSP_PWM_TIM4_UNIT, TMR4_OC_CH_WL, cmp);
        break;
    default:
        return -RT_EINVAL;
    }
#endif
    return RT_EOK;
}

/* 提供一个 flush，用于三相同步更新 */
void bsp_pwm_flush(void)
{
#if BSP_PWM_USE_DUTY_CACHE
    uint32_t i;
    for (i = 0; i < 3U; i++) {
        bsp_pwm_write_duty(i, g_bsp_pwm_priv.duty_cache[i]);
    }
#endif
}

static rt_err_t bsp_pwm_enable_impl(hw_pwm_t *self)
{
    SM_ASSERT_PTR(self);
    (void)self;
    if (!g_bsp_pwm_priv.initialized) return -RT_ERROR;

    bsp_pwm_emb_release();
    g_bsp_pwm_priv.braked = 0;
    TMR4_Start(BSP_PWM_TIM4_UNIT);
    return RT_EOK;
}

static rt_err_t bsp_pwm_disable_impl(hw_pwm_t *self)
{
    SM_ASSERT_PTR(self);
    (void)self;
    if (!g_bsp_pwm_priv.initialized) return -RT_ERROR;

    /* 【2026-11-02 修复】只封锁功率级，**不停计数器**
     *
     * 缺陷：本函数原来会 TMR4_Stop()。但它的调用者是
     *   bldc_6step_coast()（滑行），而滑行是**正常操作**，
     *   刹车、故障停机、正常减速都会走到这里。
     *   一旦 Stop，20 kHz 时基中断立刻断掉、cycle_cnt 冻住，
     *   而滑行后电机仍在惯性转动——此时角度增量全靠 1ms
     *   定时器读 Z0 维持，时基却已失效，FOC 电角度外推全废。
     *
     * 计数器与输出是两件事：
     *   EMB SOE3=1  → 六路输出强制安全电平，功率级不动作（这才是"disable"）
     *   TMR4 计数   → 时基心跳，应常驻
     *
     * 只有 deinit（真正卸载外设）才允许 TMR4_Stop。 */
    bsp_pwm_emb_stop();
    return RT_EOK;
}

static rt_err_t bsp_pwm_brake_impl(hw_pwm_t *self)
{
    SM_ASSERT_PTR(self);
    (void)self;
    if (!g_bsp_pwm_priv.initialized) return -RT_ERROR;

    bsp_pwm_emb_stop();
    g_bsp_pwm_priv.braked = 1;
    return RT_EOK;
}

static rt_err_t bsp_pwm_brake_release_impl(hw_pwm_t *self)
{
    SM_ASSERT_PTR(self);
    (void)self;
    if (!g_bsp_pwm_priv.initialized) return -RT_ERROR;

    bsp_pwm_emb_release();
    g_bsp_pwm_priv.braked = 0;
    return RT_EOK;
}

/* ============================================================
 * Ops 表
 * ============================================================ */
const hw_pwm_ops_t g_bsp_pwm_ops = {
    .init          = bsp_pwm_init_impl,
    .deinit        = bsp_pwm_deinit_impl,
    .set_duty      = bsp_pwm_set_duty_impl,
    .enable        = bsp_pwm_enable_impl,
    .disable       = bsp_pwm_disable_impl,
    .brake         = bsp_pwm_brake_impl,
    .brake_release = bsp_pwm_brake_release_impl,
};

///* ============================================================
// * 全局实例注册
// * ============================================================ */
//void bsp_pwm_register(void)
//{
//    g_swing_pwm.name     = "pwm_3phase";
//    g_swing_pwm.ops      = &g_bsp_pwm_ops;
//    g_swing_pwm.priv     = &g_bsp_pwm_priv;
//    g_swing_pwm.max_duty = MOTOR_PWM_MAX_DUTY;
//    g_swing_pwm.is_ready = 0;
//
//    rt_kprintf("[BSP-PWM] Registered\n");
//}

/* ============================================================
 * ISR 安全接口：直接操作寄存器，不走 DDL，不做断言
 * ============================================================ */
void bsp_pwm_emergency_brake(void)
{
#if BSP_PWM_EMB_ENABLE
    BSP_PWM_EMB_SOE3 |= 1UL;
#else
    /* 用户裁定：不启用 EMB 功率级封锁；停机由 hw_tmr4_pwm_set_all_low() 保证 */
#endif
}

void bsp_pwm_emergency_release(void)
{
    BSP_PWM_EMB_SOE3 &= ~1UL;
}
