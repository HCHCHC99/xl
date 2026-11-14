#include "hw_tmr4_pwm.h"
#include "hc32_ll_tmr4.h"
#include "hc32_ll_gpio.h"
#include "hc32_ll_fcg.h"
#include "hc32_ll_clk.h"
#include "hc32_ll_utility.h"
#include "hw_board_pins.h"      /* 引脚宏：本文件不配 GPIO，但需要 pwmCh 对应关系 */
#include "rtt_log.h"
#include <stdbool.h>
#include <stddef.h>
#include "cfg_foc_table.h"

/*=============================================================================
 * 采样臂 / 影子装载点（编译期，由 cfg_foc_table.h 决定）
 *
 *   FOC_CFG_SAMPLE_POINT == CFG_SAMPLE_UPPER : 采峰值（高侧导通区）
 *   FOC_CFG_SAMPLE_POINT == CFG_SAMPLE_LOWER : 采谷底（低侧导通区）
 *
 *   装载点取采样点的“另一个极值”，避免占空比更新与 ADC 采样撞在同一时刻。
 *   注意：CFG_SAMPLE_* 是枚举常量，不能用 #if 判断，只能用运行期表达式。
 *=============================================================================*/
#define TMR4_SAMPLE_IS_HIGH_SIDE   ((FOC_CFG_SAMPLE_POINT) == CFG_SAMPLE_UPPER)
#define TMR4_OC_LOAD_COND()        ((FOC_CFG_SAMPLE_POINT) == CFG_SAMPLE_UPPER \
                                    ? TMR4_OC_BUF_COND_VALLEY : TMR4_OC_BUF_COND_PEAK)

/*=============================================================================
 * Single-instance state (hard-coded to TMR4_3)
 *=============================================================================*/
typedef struct {
    uint32_t ocChH;
    uint32_t ocChL;
    uint32_t pwmCh;
} pwm_pair_desc_t;

static const pwm_pair_desc_t s_pairs[3] = {
    { TMR4_OC_CH_UH, TMR4_OC_CH_UL, TMR4_CHANNEL_U },
    { TMR4_OC_CH_VH, TMR4_OC_CH_VL, TMR4_CHANNEL_V },
    { TMR4_OC_CH_WH, TMR4_OC_CH_WL, TMR4_CHANNEL_W },
};

static bool                 s_bConfigured  = false;
static bool                 s_bRunning     = false;
static tmr4_pwm_context_t   s_ctx          = { 0U, 0U, 0U, 0U, 0.0f };
static tmr4_output_type_t   s_outputType[3] = {
    TMR4_OUTPUT_SYNC, TMR4_OUTPUT_SYNC, TMR4_OUTPUT_SYNC,
};
static tmr4_channel_mode_t  s_chOpMode[3] = {
    TMR4_MODE_OFF, TMR4_MODE_OFF, TMR4_MODE_OFF,
};
static uint16_t s_occrH[3]  = { 0U, 0U, 0U };
static uint16_t s_occrL[3]  = { 0U, 0U, 0U };
static float    s_centerPct[3] = { 0.0f, 0.0f, 0.0f };
static bool     s_bActiveHigh = true;

/*=============================================================================
 * Internal helpers
 *=============================================================================*/
static bool ChannelIsValid(tmr4_pwm_channel_t ch)
{
    return (uint32_t)ch <= (uint32_t)TMR4_CHANNEL_W;
}

static void ShadowApplyOC(uint32_t ocCh)
{
    uint16_t cond  = TMR4_OC_LOAD_COND();
    uint16_t immed = TMR4_OC_BUF_COND_IMMED;

    TMR4_OC_SetCompareBufCond(CM_TMR4_3, ocCh, TMR4_OC_BUF_CMP_VALUE,
        TMR4_SHADOW_OCCR_ENABLE ? cond : immed);
    TMR4_OC_SetCompareBufCond(CM_TMR4_3, ocCh, TMR4_OC_BUF_CMP_MD,
        TMR4_SHADOW_OCMR_ENABLE ? cond : immed);
}

static void OccrHWrite(int idx, uint32_t ocCh, uint16_t value)
{
    s_occrH[idx] = value;
    TMR4_OC_SetCompareValue(CM_TMR4_3, ocCh, value);
}

static void OccrLWrite(int idx, uint32_t ocCh, uint16_t value)
{
    s_occrL[idx] = value;
    TMR4_OC_SetCompareValue(CM_TMR4_3, ocCh, value);
}

static uint16_t DeadTimeNsToTicks(uint16_t deadTimeNs, uint32_t timerHz)
{
    uint16_t ticks;

    /* 死区计数器与 PWM 计数器同源（PCLK1，DIV1），1 tick = 1 个时钟周期。
     * 实测：500 ns @100 MHz 应得 50 tick = 500 ns；旧代码多除了 2，只剩 250 ns。 */
    ticks = (uint16_t)((uint64_t)deadTimeNs * timerHz / 1000000000ULL);
    if (ticks > TMR4_DEADTIME_TICKS_MAX) {
        ticks = TMR4_DEADTIME_TICKS_MAX;
    }
    return ticks;
}

static void WriteDeadTimeForChannel(const pwm_pair_desc_t *pair, uint16_t ticks)
{
    TMR4_PWM_SetDeadTimeValue(CM_TMR4_3, pair->pwmCh, TMR4_PWM_PDAR_IDX, ticks);
    TMR4_PWM_SetDeadTimeValue(CM_TMR4_3, pair->pwmCh, TMR4_PWM_PDBR_IDX, ticks);
}

static void RefreshContext(void)
{
    uint32_t timerHz = CLK_GetBusClockFreq(CLK_BUS_PCLK1);
    float    fPeriod = (float)s_ctx.periodTicks;

    s_ctx.deadTicks   = DeadTimeNsToTicks(s_ctx.deadTimeNs, timerHz);
    s_ctx.deadTimePct = (fPeriod > 0.0f)
                      ? ((float)s_ctx.deadTicks * 100.0f / fPeriod) : 0.0f;
}

static float CompareToPct(uint16_t ticks)
{
    float fPeriod = (float)s_ctx.periodTicks;
    return (fPeriod > 0.0f) ? ((float)ticks * 100.0f / fPeriod) : 0.0f;
}

/*=============================================================================
 * 占空比 -> 比较值（新几何）
 *
 *   高侧（互补 / 同向都用这一路）：导通窗口 (X, CPSR]
 *     Dh = (CPSR - X)/CPSR  =>  X = CPSR*(1-Dh)
 *
 *   钳位到 [1, CPSR-1]，避开 0%/100% 的极值匹配歧义区
 *=============================================================================*/
static uint16_t HighDutyToCmpH(float dutyPct)
{
    int32_t cpsr = (int32_t)s_ctx.periodTicks - 1;
    int32_t x;

    if (dutyPct < 0.0f)   dutyPct = 0.0f;
    if (dutyPct > 100.0f) dutyPct = 100.0f;

    x = cpsr - (int32_t)((float)cpsr * dutyPct / 100.0f);

    if (x < 1)        x = 1;
    if (x > cpsr - 1) x = cpsr - 1;
    return (uint16_t)x;
}

static void ApplySyncDutyWithDeadTime(tmr4_pwm_channel_t ch,
                                      float dutyPct,
                                      uint16_t deadTicks)
{
    const pwm_pair_desc_t *pair = &s_pairs[ch];
    uint16_t cpsr = (uint16_t)(s_ctx.periodTicks - 1U);
    int32_t xh = (int32_t)HighDutyToCmpH(dutyPct);   /* 同向：上管峰值导通 */
    int32_t xl = xh + (int32_t)deadTicks;

    /* 死区为 0 时两路完全相同；非 0 时下管推迟 deadTicks 开通、提前 deadTicks 关断 */
    if (xl > (int32_t)cpsr - 1) xl = (int32_t)cpsr - 1;

    OccrHWrite((int)ch, pair->ocChH, (uint16_t)xh);
    OccrLWrite((int)ch, pair->ocChL, (uint16_t)xl);
}

static uint16_t ComplementaryPolarity(tmr4_pwm_channel_t ch, bool activeHigh)
{
    (void)ch;   /* U/V/W 三相统一，ch 不再参与判断 */

    if (!activeHigh) {
        return TMR4_PWM_OXH_INVT_OXL_INVT;
    }
    /* 三相统一：上下管都 HOLD，与 U 相一致 */
    return TMR4_PWM_OXH_HOLD_OXL_HOLD;
}

/*=============================================================================
 * OC compare mode helpers
 *=============================================================================*/
static void OCModeSyncHigh(CM_TMR4_TypeDef *TMR4x, uint32_t ch)
{
    un_tmr4_oc_ocmrh_t m;
    m.OCMRx = 0U;
    m.OCMRx_f.OCFDCH  = TMR4_OC_OCF_SET;
    m.OCMRx_f.OCFPKH  = TMR4_OC_OCF_SET;
    m.OCMRx_f.OCFUCH  = TMR4_OC_OCF_SET;
    m.OCMRx_f.OCFZRH  = TMR4_OC_OCF_SET;
    /* 同向模式：上下管同相，导通窗口 (X, CPSR]（居中峰值）
     * 谷底拉低、上计数匹配拉高、峰值保持高、下计数匹配拉低 */
    m.OCMRx_f.OPDCH   = TMR4_OC_LOW;
    m.OCMRx_f.OPPKH   = TMR4_OC_LOW;
    m.OCMRx_f.OPUCH   = TMR4_OC_HIGH;
    m.OCMRx_f.OPZRH   = TMR4_OC_HIGH;
    m.OCMRx_f.OPNPKH  = TMR4_OC_HIGH;
    m.OCMRx_f.OPNZRH  = TMR4_OC_LOW;
    TMR4_OC_SetHighChCompareMode(TMR4x, ch, m);
}

static void OCModeSyncLow(CM_TMR4_TypeDef *TMR4x, uint32_t ch)
{
    un_tmr4_oc_ocmrl_t m;
    m.OCMRx = 0U;
    m.OCMRx_f.OCFDCL  = TMR4_OC_OCF_SET;
    m.OCMRx_f.OCFPKL  = TMR4_OC_OCF_SET;
    m.OCMRx_f.OCFUCL  = TMR4_OC_OCF_SET;
    m.OCMRx_f.OCFZRL  = TMR4_OC_OCF_SET;
    /* 同向：上下管同相，导通窗口 (X, CPSR]（居中峰值） */
    m.OCMRx_f.OPDCL   = TMR4_OC_LOW;
    m.OCMRx_f.OPPKL   = TMR4_OC_LOW;
    m.OCMRx_f.OPUCL   = TMR4_OC_HIGH;
    m.OCMRx_f.OPZRL   = TMR4_OC_HIGH;
    m.OCMRx_f.OPNPKL  = TMR4_OC_HIGH;
    m.OCMRx_f.OPNZRL  = TMR4_OC_LOW;
    /* 独立操作模式：bit19~16=0，bit31~20 与 bit15~4 镜像同一组动作 */
    m.OCMRx_f.EOPNDCL = TMR4_OC_HOLD;
    m.OCMRx_f.EOPNUCL = TMR4_OC_HOLD;
    m.OCMRx_f.EOPDCL  = TMR4_OC_LOW;
    m.OCMRx_f.EOPPKL  = TMR4_OC_LOW;
    m.OCMRx_f.EOPUCL  = TMR4_OC_HIGH;
    m.OCMRx_f.EOPZRL  = TMR4_OC_HIGH;
    m.OCMRx_f.EOPNPKL = TMR4_OC_HIGH;
    m.OCMRx_f.EOPNZRL = TMR4_OC_LOW;
    TMR4_OC_SetLowChCompareMode(TMR4x, ch, m);
}

static void OCModeCompLow(CM_TMR4_TypeDef *TMR4x, uint32_t ch)
{
    un_tmr4_oc_ocmrl_t m;
    m.OCMRx = 0U;
    m.OCMRx_f.OCFDCL  = TMR4_OC_OCF_SET;
    m.OCMRx_f.OCFPKL  = TMR4_OC_OCF_SET;
    m.OCMRx_f.OCFUCL  = TMR4_OC_OCF_SET;
    m.OCMRx_f.OCFZRL  = TMR4_OC_OCF_SET;
    /* in_op 在 (Xh, CPSR] 为高，窗口关于峰值对称：
     * 谷底拉低 -> 上计数匹配拉高（峰值前的上升沿）-> 峰值保持高 -> 下计数匹配拉低
     * 若上计数用 HOLD，就只有“峰值到下降沿”这半个窗口，占空比会只有一半 */
    m.OCMRx_f.OPDCL   = TMR4_OC_LOW;
    m.OCMRx_f.OPPKL   = TMR4_OC_LOW;    /* 占空比 0% 边界（Xh=峰值） */
    m.OCMRx_f.OPUCL   = TMR4_OC_HIGH;
    m.OCMRx_f.OPZRL   = TMR4_OC_HIGH;   /* 占空比 100% 边界（Xh=0） */
    m.OCMRx_f.OPNPKL  = TMR4_OC_HIGH;   /* 峰值无匹配 -> 高（常规情况） */
    m.OCMRx_f.OPNZRL  = TMR4_OC_LOW;    /* 谷底无匹配 -> 低（常规情况） */
    /* EOP 与 OP 镜像、bit19~16=0：保证独立操作模式。
     * 互补模式虽然没有使能 H 通道，但 OCCR*h 仍可能是 0（谷底恒匹配），
     * 若不镜像，谷底会走 EOP 分支而不是 OPNZRL，波形会被破坏。 */
    m.OCMRx_f.EOPNDCL = TMR4_OC_HOLD;
    m.OCMRx_f.EOPNUCL = TMR4_OC_HOLD;
    m.OCMRx_f.EOPDCL  = TMR4_OC_LOW;
    m.OCMRx_f.EOPPKL  = TMR4_OC_LOW;
    m.OCMRx_f.EOPUCL  = TMR4_OC_HIGH;
    m.OCMRx_f.EOPZRL  = TMR4_OC_HIGH;
    m.OCMRx_f.EOPNPKL = TMR4_OC_HIGH;
    m.OCMRx_f.EOPNZRL = TMR4_OC_LOW;
    TMR4_OC_SetLowChCompareMode(TMR4x, ch, m);
}

/*=============================================================================
 * Numeric conversion (center <-> actual H/L)
 *=============================================================================*/
void hw_tmr4_pwm_center_to_actual(tmr4_pwm_channel_t ch,
                                  float centerPct,
                                  float *outHighPct,
                                  float *outLowPct)
{
    float dtPct;

    if (outHighPct == NULL || outLowPct == NULL) return;

    if (!s_bConfigured || !ChannelIsValid(ch)) {
        *outHighPct = centerPct;
        *outLowPct  = centerPct;
        return;
    }

    dtPct = s_ctx.deadTimePct;

    if (s_outputType[ch] == TMR4_OUTPUT_COMPLEMENTARY) {
        /* 互补：高侧引脚上升沿被 PDBR 延迟半个死区，低侧同理 */
        *outHighPct = centerPct - dtPct * 0.5f;
        *outLowPct  = 100.0f - (*outHighPct) - dtPct;
    } else {
        /* 同向：上下管同相，都是峰值导通；下管窄 deadTicks */
        *outHighPct = centerPct;
        *outLowPct  = centerPct - dtPct;
    }
}

tmr4_pwm_err_t hw_tmr4_pwm_actual_duty_to_center(tmr4_pwm_channel_t ch,
                                                 float highPct,
                                                 float lowPct,
                                                 float *outCenterPct)
{
    float dtPct;

    if (outCenterPct == NULL)              return TMR4_PWM_ERR_BAD_PARAM;
    if (!s_bConfigured)                    return TMR4_PWM_ERR_NOT_INIT;
    if (!ChannelIsValid(ch))               return TMR4_PWM_ERR_BAD_PARAM;

    dtPct = s_ctx.deadTimePct;

    if (s_outputType[ch] == TMR4_OUTPUT_COMPLEMENTARY) {
        *outCenterPct = highPct + dtPct * 0.5f;
    } else {
        *outCenterPct = highPct;
    }
    (void)lowPct;
    return TMR4_PWM_OK;
}

/*=============================================================================
 * Config
 *=============================================================================*/
tmr4_pwm_err_t hw_tmr4_pwm_config(const tmr4_pwm_config_t *cfg)
{
    stc_tmr4_init_t     stcTmr4Init;
    stc_tmr4_oc_init_t  stcTmr4OcInit;
    stc_tmr4_pwm_init_t stcTmr4PwmInit;
    uint32_t timerHz;
    uint16_t deadTicks;
    uint16_t periodTicks;
    int i;

    if (cfg == NULL)      return TMR4_PWM_ERR_BAD_PARAM;
    if (cfg->freqHz == 0) return TMR4_PWM_ERR_FREQ;

    timerHz = CLK_GetBusClockFreq(CLK_BUS_PCLK1);
    periodTicks = (uint16_t)(timerHz / (cfg->freqHz * 2U));
    if (periodTicks < 2U) return TMR4_PWM_ERR_FREQ;

    s_outputType[0] = cfg->outputTypeU;
    s_outputType[1] = cfg->outputTypeV;
    s_outputType[2] = cfg->outputTypeW;
    s_bActiveHigh   = cfg->activeHigh;

    s_ctx.periodTicks = periodTicks;
    s_ctx.freqHz      = cfg->freqHz;
    s_ctx.deadTimeNs  = cfg->deadTimeNs;
    RefreshContext();

    MAIN_D("[HW_TMR4] clk=%lu Hz period=%u [U:%s V:%s W:%s]",
           (unsigned long)timerHz, s_ctx.periodTicks,
           (cfg->outputTypeU == TMR4_OUTPUT_COMPLEMENTARY) ? "COMP" : "SYNC",
           (cfg->outputTypeV == TMR4_OUTPUT_COMPLEMENTARY) ? "COMP" : "SYNC",
           (cfg->outputTypeW == TMR4_OUTPUT_COMPLEMENTARY) ? "COMP" : "SYNC");

    FCG_Fcg2PeriphClockCmd(FCG2_PERIPH_TMR4_3, ENABLE);

    TMR4_StructInit(&stcTmr4Init);
    stcTmr4Init.u16ClockDiv    = TMR4_CLK_DIV1;
    stcTmr4Init.u16CountMode   = TMR4_MD_TRIANGLE;
    stcTmr4Init.u16PeriodValue = s_ctx.periodTicks - 1U;
    TMR4_Init(CM_TMR4_3, &stcTmr4Init);
    TMR4_PeriodBufCmd(CM_TMR4_3, TMR4_SHADOW_CPSR_ENABLE ? ENABLE : DISABLE);

    deadTicks = s_ctx.deadTicks;

    TMR4_OC_StructInit(&stcTmr4OcInit);

    for (i = 0; i < 3; i++) {
        const pwm_pair_desc_t *pair = &s_pairs[i];
        uint16_t polarity;

        TMR4_PWM_StructInit(&stcTmr4PwmInit);
        stcTmr4PwmInit.u16ClockDiv = TMR4_PWM_CLK_DIV1;

        if (s_outputType[i] == TMR4_OUTPUT_COMPLEMENTARY) {
            stcTmr4OcInit.u16CompareValue        = HighDutyToCmpH(50.0f);
            stcTmr4OcInit.u16OcInvalidPolarity   = TMR4_OC_INVD_LOW;
            stcTmr4OcInit.u16CompareValueBufCond = TMR4_OC_LOAD_COND();
            stcTmr4OcInit.u16CompareModeBufCond  = TMR4_OC_LOAD_COND();
            stcTmr4OcInit.u16BufLinkTransObject  = TMR4_OC_BUF_CMP_VALUE | TMR4_OC_BUF_CMP_MD;
            TMR4_OC_Init(CM_TMR4_3, pair->ocChL, &stcTmr4OcInit);

            OCModeCompLow(CM_TMR4_3, pair->ocChL);
            ShadowApplyOC(pair->ocChL);
            TMR4_OC_Cmd(CM_TMR4_3, pair->ocChL, ENABLE);

            s_occrH[i] = 0U;
            OccrLWrite(i, pair->ocChL, HighDutyToCmpH(50.0f));

            stcTmr4PwmInit.u16Mode = TMR4_PWM_MD_DEAD_TMR;
            polarity = ComplementaryPolarity((tmr4_pwm_channel_t)i, s_bActiveHigh);

            TMR4_PWM_Init(CM_TMR4_3, pair->pwmCh, &stcTmr4PwmInit);
            TMR4_PWM_SetPolarity(CM_TMR4_3, pair->pwmCh, polarity);
            WriteDeadTimeForChannel(pair, deadTicks);
        } else {
            stcTmr4OcInit.u16CompareValue        = HighDutyToCmpH(50.0f);
            stcTmr4OcInit.u16CompareValueBufCond = TMR4_OC_LOAD_COND();
            stcTmr4OcInit.u16CompareModeBufCond  = TMR4_OC_LOAD_COND();
            stcTmr4OcInit.u16BufLinkTransObject  = TMR4_OC_BUF_CMP_VALUE | TMR4_OC_BUF_CMP_MD;

            TMR4_OC_Init(CM_TMR4_3, pair->ocChH, &stcTmr4OcInit);
            TMR4_OC_Init(CM_TMR4_3, pair->ocChL, &stcTmr4OcInit);
            OCModeSyncHigh(CM_TMR4_3, pair->ocChH);
            OCModeSyncLow (CM_TMR4_3, pair->ocChL);
            TMR4_OC_Cmd(CM_TMR4_3, pair->ocChH, ENABLE);
            TMR4_OC_Cmd(CM_TMR4_3, pair->ocChL, ENABLE);
            ShadowApplyOC(pair->ocChH);
            ShadowApplyOC(pair->ocChL);

            ApplySyncDutyWithDeadTime((tmr4_pwm_channel_t)i, 50.0f, deadTicks);

            stcTmr4PwmInit.u16Mode = TMR4_PWM_MD_THROUGH;
            polarity = s_bActiveHigh ? TMR4_PWM_OXH_HOLD_OXL_HOLD
                                     : TMR4_PWM_OXH_INVT_OXL_INVT;

            TMR4_PWM_Init(CM_TMR4_3, pair->pwmCh, &stcTmr4PwmInit);
            TMR4_PWM_SetPolarity(CM_TMR4_3, pair->pwmCh, polarity);
        }

        s_centerPct[i] = 50.0f;
    }

    s_bConfigured = true;
    s_bRunning    = false;

    (void)hw_tmr4_pwm_config_adc_trigger(true);

    MAIN_D("[HW_TMR4] configured: period=%u polar=%s dt=%u ns (%u ticks), sample=%s, load=%s, ADC trig=%s",
           s_ctx.periodTicks, cfg->activeHigh ? "HIGH" : "LOW",
           s_ctx.deadTimeNs, s_ctx.deadTicks,
           TMR4_SAMPLE_IS_HIGH_SIDE ? "HIGH(peak)" : "LOW(valley)",
           TMR4_SAMPLE_IS_HIGH_SIDE ? "VALLEY" : "PEAK",
           TMR4_SAMPLE_IS_HIGH_SIDE ? "PEAK" : "VALLEY");

    return TMR4_PWM_OK;
}

/*=============================================================================
 * Start / Stop / Resume / AllLow / SafeState
 *=============================================================================*/
tmr4_pwm_err_t hw_tmr4_pwm_start_output(void)
{
    if (!s_bConfigured) return TMR4_PWM_ERR_NOT_INIT;
    TMR4_Start(CM_TMR4_3);
    s_bRunning = true;
    return TMR4_PWM_OK;
}

tmr4_pwm_err_t hw_tmr4_pwm_stop_output(void)
{
    int i;
    if (!s_bConfigured) return TMR4_PWM_ERR_NOT_INIT;

    TMR4_Stop(CM_TMR4_3);
    s_bRunning = false;

    for (i = 0; i < 3; i++) {
        const pwm_pair_desc_t *pair = &s_pairs[i];
        if (s_outputType[i] == TMR4_OUTPUT_SYNC) {
            TMR4_OC_Cmd(CM_TMR4_3, pair->ocChH, DISABLE);
        }
        TMR4_OC_Cmd(CM_TMR4_3, pair->ocChL, DISABLE);
    }
    return TMR4_PWM_OK;
}

/*=============================================================================
 * Resume：从 stop / all_low 恢复
 *
 *   - 按 s_outputType 重新初始化 PWM 模块（互补 / 直通）
 *   - 重新使能 OC 通道
 *   - 重新启动 TMR4 计数
 *   - 不改变比较值 / 死区配置
 *
 *   用途：mode 0（全低）切回 mode 1（50%）或 mode 2（IF）时调用
 *=============================================================================*/
tmr4_pwm_err_t hw_tmr4_pwm_resume_output(void)
{
    int i;
    stc_tmr4_pwm_init_t stcPwmInit;
    uint16_t polarity;

    if (!s_bConfigured) return TMR4_PWM_ERR_NOT_INIT;

    for (i = 0; i < 3; i++) {
        const pwm_pair_desc_t *pair = &s_pairs[i];

        TMR4_PWM_StructInit(&stcPwmInit);
        stcPwmInit.u16ClockDiv = TMR4_PWM_CLK_DIV1;

        if (s_outputType[i] == TMR4_OUTPUT_COMPLEMENTARY) {
            /* ---- 互补模式：和 config 的 COMPL 分支完全一致 ---- */
            stcPwmInit.u16Mode = TMR4_PWM_MD_DEAD_TMR;

            /* 恢复 OC compare mode（set_all_low 没动它，但为对称起见重设） */
            OCModeCompLow(CM_TMR4_3, pair->ocChL);

            /* 恢复 invalid polarity（set_all_low 改成了 LOW，config 里是 LOW，
             * 这里保持一致即可；若 config 用别的值，按 config 填） */
            TMR4_OC_SetOcInvalidPolarity(CM_TMR4_3, pair->ocChL, TMR4_OC_INVD_LOW);
            TMR4_OC_SetOcInvalidPolarity(CM_TMR4_3, pair->ocChH, TMR4_OC_INVD_LOW);

            /* 上管由硬件互补生成，不单独使能 H */
            TMR4_OC_Cmd(CM_TMR4_3, pair->ocChL, ENABLE);

            polarity = ComplementaryPolarity((tmr4_pwm_channel_t)i, s_bActiveHigh);
        } else {
            /* ---- 同步模式：和 config 的 SYNC 分支完全一致 ---- */
            stcPwmInit.u16Mode = TMR4_PWM_MD_THROUGH;

            OCModeSyncHigh(CM_TMR4_3, pair->ocChH);
            OCModeSyncLow (CM_TMR4_3, pair->ocChL);

            TMR4_OC_SetOcInvalidPolarity(CM_TMR4_3, pair->ocChH, TMR4_OC_INVD_LOW);
            TMR4_OC_SetOcInvalidPolarity(CM_TMR4_3, pair->ocChL, TMR4_OC_INVD_LOW);

            TMR4_OC_Cmd(CM_TMR4_3, pair->ocChH, ENABLE);
            TMR4_OC_Cmd(CM_TMR4_3, pair->ocChL, ENABLE);

            polarity = s_bActiveHigh ? TMR4_PWM_OXH_HOLD_OXL_HOLD
                                     : TMR4_PWM_OXH_INVT_OXL_INVT;
        }

        TMR4_PWM_Init(CM_TMR4_3, pair->pwmCh, &stcPwmInit);
        TMR4_PWM_SetPolarity(CM_TMR4_3, pair->pwmCh, polarity);

        if (s_outputType[i] == TMR4_OUTPUT_COMPLEMENTARY) {
            WriteDeadTimeForChannel(pair, s_ctx.deadTicks);
        }
    }

    TMR4_Start(CM_TMR4_3);
    s_bRunning = true;
    return TMR4_PWM_OK;
}

/*=============================================================================
 * SetAllLow：六路全低（mode 0 安全停机）
 *
 *   为什么不能只用 stop_output：
 *     stop_output 只停计数 + 关 OC，引脚电平由 OC invalid polarity 决定
 *     互补模式下上管由硬件互补生成，低侧 invalid=LOW 会导致上管为高
 *
 *   为什么不能 Stop TMR4：
 *     TMR4 是 EOCA 的时基源。Stop 后没有 EVT0 → ADC 不触发 → EOCA 不来
 *     → ISR 停 → 切不回 mode 1/2。所以本函数**不 Stop TMR4**，
 *     只关 OC + 设 invalid LOW + 改 THROUGH/HOLD，让引脚全低，
 *     但计数仍在跑，EVT0/ADC/EOCA 链保持
 *
 *   切回 mode 1/2 时必须调 hw_tmr4_pwm_resume_output() 恢复 PWM 模式
 *=============================================================================*/
tmr4_pwm_err_t hw_tmr4_pwm_set_all_low(void)
{
    int i;
    stc_tmr4_pwm_init_t stcPwmInit;

    if (!s_bConfigured) return TMR4_PWM_ERR_NOT_INIT;

    /* ★ 不 Stop TMR4：保持 EVT0/ADC/EOCA 链不断 */
    /* TMR4_Stop(CM_TMR4_3);  ← 删除 */
    /* s_bRunning = false;    ← 删除 */

    for (i = 0; i < 3; i++) {
        const pwm_pair_desc_t *pair = &s_pairs[i];

        TMR4_OC_Cmd(CM_TMR4_3, pair->ocChH, DISABLE);
        TMR4_OC_Cmd(CM_TMR4_3, pair->ocChL, DISABLE);

        TMR4_OC_SetOcInvalidPolarity(CM_TMR4_3, pair->ocChH, TMR4_OC_INVD_LOW);
        TMR4_OC_SetOcInvalidPolarity(CM_TMR4_3, pair->ocChL, TMR4_OC_INVD_LOW);

        TMR4_PWM_StructInit(&stcPwmInit);
        stcPwmInit.u16Mode     = TMR4_PWM_MD_THROUGH;
        stcPwmInit.u16ClockDiv = TMR4_PWM_CLK_DIV1;
        stcPwmInit.u16Polarity = TMR4_PWM_OXH_HOLD_OXL_HOLD;
        TMR4_PWM_Init(CM_TMR4_3, pair->pwmCh, &stcPwmInit);
    }
    return TMR4_PWM_OK;
}

tmr4_pwm_err_t hw_tmr4_pwm_set_safe_state(tmr4_safe_state_t state)
{
    int i;

    if (!s_bConfigured) return TMR4_PWM_ERR_NOT_INIT;

    switch (state) {
    case TMR4_SAFE_HIZ:
        for (i = 0; i < 3; i++) {
            const pwm_pair_desc_t *pair = &s_pairs[i];
            TMR4_OC_Cmd(CM_TMR4_3, pair->ocChH, DISABLE);
            TMR4_OC_Cmd(CM_TMR4_3, pair->ocChL, DISABLE);
        }
        break;

    case TMR4_SAFE_ALL_OFF:
        for (i = 0; i < 3; i++) {
            const pwm_pair_desc_t *pair = &s_pairs[i];
            TMR4_OC_Cmd(CM_TMR4_3, pair->ocChH, DISABLE);
            TMR4_OC_Cmd(CM_TMR4_3, pair->ocChL, DISABLE);
            TMR4_OC_SetOcInvalidPolarity(CM_TMR4_3, pair->ocChH, TMR4_OC_INVD_LOW);
            TMR4_OC_SetOcInvalidPolarity(CM_TMR4_3, pair->ocChL, TMR4_OC_INVD_LOW);
        }
        break;

    default:
        return TMR4_PWM_ERR_BAD_PARAM;
    }
    return TMR4_PWM_OK;
}

/*=============================================================================
 * Frequency / dead-time
 *=============================================================================*/
tmr4_pwm_err_t hw_tmr4_pwm_set_frequency(uint32_t freqHz)
{
    uint32_t timerHz;
    uint16_t periodTicks;
    uint16_t deadTicks;
    int i;

    if (!s_bConfigured) return TMR4_PWM_ERR_NOT_INIT;
    if (freqHz == 0U)   return TMR4_PWM_ERR_FREQ;

    timerHz     = CLK_GetBusClockFreq(CLK_BUS_PCLK1);
    periodTicks = (uint16_t)(timerHz / (freqHz * 2U));
    if (periodTicks < 2U) return TMR4_PWM_ERR_FREQ;

    s_ctx.periodTicks = periodTicks;
    s_ctx.freqHz      = freqHz;
    RefreshContext();
    deadTicks = s_ctx.deadTicks;

    TMR4_SetPeriodValue(CM_TMR4_3, s_ctx.periodTicks - 1U);

    for (i = 0; i < 3; i++) {
        if (s_outputType[i] == TMR4_OUTPUT_COMPLEMENTARY) {
            WriteDeadTimeForChannel(&s_pairs[i], deadTicks);
        }
    }

    (void)hw_tmr4_pwm_config_adc_trigger(true);

    return TMR4_PWM_OK;
}

tmr4_pwm_err_t hw_tmr4_pwm_set_deadtime(uint16_t deadTimeNs)
{
    int i;

    if (!s_bConfigured) return TMR4_PWM_ERR_NOT_INIT;

    s_ctx.deadTimeNs = deadTimeNs;
    RefreshContext();

    for (i = 0; i < 3; i++) {
        if (s_outputType[i] == TMR4_OUTPUT_COMPLEMENTARY) {
            WriteDeadTimeForChannel(&s_pairs[i], s_ctx.deadTicks);
        }
    }
    return TMR4_PWM_OK;
}

/*=============================================================================
 * Duty (center semantics)
 *=============================================================================*/
tmr4_pwm_err_t hw_tmr4_pwm_set_duty(tmr4_pwm_channel_t ch, uint16_t dutyU16)
{
    float centerPct;

    if (!s_bConfigured)      return TMR4_PWM_ERR_NOT_INIT;
    if (!ChannelIsValid(ch)) return TMR4_PWM_ERR_BAD_PARAM;
    if (dutyU16 > TMR4_PWM_DUTY_MAX) dutyU16 = TMR4_PWM_DUTY_MAX;

    centerPct = (float)dutyU16 * 100.0f / (float)TMR4_PWM_DUTY_MAX;
    return hw_tmr4_pwm_set_duty_float(ch, centerPct);
}

tmr4_pwm_err_t hw_tmr4_pwm_set_duty_float(tmr4_pwm_channel_t ch, float centerPct)
{
    const pwm_pair_desc_t *pair;
    uint16_t deadTicks;

    if (!s_bConfigured)      return TMR4_PWM_ERR_NOT_INIT;
    if (!ChannelIsValid(ch)) return TMR4_PWM_ERR_BAD_PARAM;

    deadTicks = s_ctx.deadTicks;
    pair      = &s_pairs[ch];
    s_centerPct[ch] = centerPct;

    if (s_outputType[ch] == TMR4_OUTPUT_COMPLEMENTARY) {
        OccrLWrite((int)ch, pair->ocChL, HighDutyToCmpH(centerPct));
        return TMR4_PWM_OK;
    }

    switch (s_chOpMode[ch]) {
    case TMR4_MODE_HIGH_SIDE:
        OccrHWrite((int)ch, pair->ocChH, HighDutyToCmpH(centerPct));
        break;
    case TMR4_MODE_LOW_SIDE:
        break;
    case TMR4_MODE_OFF:
    default:
        ApplySyncDutyWithDeadTime(ch, centerPct, deadTicks);
        break;
    }
    return TMR4_PWM_OK;
}

tmr4_pwm_err_t hw_tmr4_pwm_set_duty3(float duPct, float dvPct, float dwPct)
{
    float    duty[3];
    uint16_t deadTicks;
    int i;

    if (!s_bConfigured) return TMR4_PWM_ERR_NOT_INIT;

    duty[0] = duPct;
    duty[1] = dvPct;
    duty[2] = dwPct;
    deadTicks = s_ctx.deadTicks;

    for (i = 0; i < 3; i++) {
        const pwm_pair_desc_t *pair = &s_pairs[i];
        s_centerPct[i] = duty[i];

        if (s_outputType[i] == TMR4_OUTPUT_COMPLEMENTARY) {
            OccrLWrite(i, pair->ocChL, HighDutyToCmpH(duty[i]));
        } else {
            if (s_chOpMode[i] == TMR4_MODE_HIGH_SIDE) {
                OccrHWrite(i, pair->ocChH, HighDutyToCmpH(duty[i]));
            } else if (s_chOpMode[i] == TMR4_MODE_LOW_SIDE) {
                /* L fixed 100%, no update */
            } else {
                ApplySyncDutyWithDeadTime((tmr4_pwm_channel_t)i, duty[i], deadTicks);
            }
        }
    }
    return TMR4_PWM_OK;
}

/*=============================================================================
 * 定点占空比（Q15 偏移）
 *   入参 = dutyPct - 50%，Q15：16384 = 50% 周期，范围 ±16384
 *   互补模式：cmp = CPSR - CPSR/2 - (off*CPSR)/2^15，纯整数
 *   其他模式：回落到浮点 set_duty3（仅非主流配置会走到）
 *=============================================================================*/
tmr4_pwm_err_t hw_tmr4_pwm_set_duty_q15(int16_t du_off, int16_t dv_off, int16_t dw_off)
{
    int16_t off[3];
    int32_t cpsr;
    bool    allComp = true;
    int     i;

    if (!s_bConfigured) return TMR4_PWM_ERR_NOT_INIT;

    off[0] = du_off;
    off[1] = dv_off;
    off[2] = dw_off;

    for (i = 0; i < 3; i++) {
        s_centerPct[i] = 50.0f + (float)off[i] * (100.0f / 32768.0f);
        if (s_outputType[i] != TMR4_OUTPUT_COMPLEMENTARY) {
            allComp = false;
        }
    }

    if (!allComp) {
        return hw_tmr4_pwm_set_duty3(s_centerPct[0], s_centerPct[1], s_centerPct[2]);
    }

    cpsr = (int32_t)s_ctx.periodTicks - 1;
    for (i = 0; i < 3; i++) {
        int32_t x = cpsr - (cpsr >> 1) - (((int32_t)off[i] * cpsr) >> 15);

        if (x < 1)        x = 1;
        if (x > cpsr - 1) x = cpsr - 1;
        OccrLWrite(i, s_pairs[i].ocChL, (uint16_t)x);
    }
    return TMR4_PWM_OK;
}

/*=============================================================================
 * Dead-time-aware duty
 *=============================================================================*/
tmr4_pwm_err_t hw_tmr4_pwm_set_duty3_deadtime(float highU, float highV, float highW)
{
    float    high[3];
    float    low[3];
    float    center[3];
    float    dtPct;
    tmr4_pwm_err_t err;
    int i;

    if (!s_bConfigured) return TMR4_PWM_ERR_NOT_INIT;

    high[0] = highU;
    high[1] = highV;
    high[2] = highW;

    for (i = 0; i < 3; i++) {
        if (high[i] < 0.0f)   high[i] = 0.0f;
        if (high[i] > 100.0f) high[i] = 100.0f;
    }

    dtPct = s_ctx.deadTimePct;

    for (i = 0; i < 3; i++) {
        /* 两种模式的低侧导通占比都是 1 - 高侧 - 死区 */
        low[i] = 100.0f - high[i] - dtPct;
        if (low[i] < 0.0f)   low[i] = 0.0f;
        if (low[i] > 100.0f) low[i] = 100.0f;

        err = hw_tmr4_pwm_actual_duty_to_center((tmr4_pwm_channel_t)i,
                                                high[i], low[i], &center[i]);
        if (err != TMR4_PWM_OK) return err;
    }

    return hw_tmr4_pwm_set_duty3(center[0], center[1], center[2]);
}

/*=============================================================================
 * Read-back
 *=============================================================================*/
void hw_tmr4_pwm_get_context(tmr4_pwm_context_t *out)
{
    if (out == NULL) return;
    *out = s_ctx;
}

uint16_t hw_tmr4_pwm_get_period_ticks(void)   { return s_ctx.periodTicks; }
uint16_t hw_tmr4_pwm_get_dead_ticks(void)     { return s_ctx.deadTicks;   }
uint32_t hw_tmr4_pwm_get_frequency(void)      { return s_ctx.freqHz;      }
uint16_t hw_tmr4_pwm_get_deadtime(void)       { return s_ctx.deadTimeNs;  }

uint16_t hw_tmr4_pwm_get_occr(tmr4_pwm_channel_t ch, bool highSide)
{
    if (!ChannelIsValid(ch)) return 0U;
    return highSide ? s_occrH[ch] : s_occrL[ch];
}

uint16_t hw_tmr4_pwm_get_counter(void)
{
    return TMR4_GetCountValue(CM_TMR4_3);
}

void hw_tmr4_pwm_get_duty3(float *duPct, float *dvPct, float *dwPct)
{
    if (duPct) *duPct = s_centerPct[0];
    if (dvPct) *dvPct = s_centerPct[1];
    if (dwPct) *dwPct = s_centerPct[2];
}

void hw_tmr4_pwm_get_state(tmr4_pwm_channel_t ch, tmr4_pwm_state_t *out)
{
    float occrHPct, occrLPct;

    if (out == NULL) return;

    if (!s_bConfigured || !ChannelIsValid(ch)) {
        out->centerPct = 0.0f;
        out->highPct   = 0.0f;
        out->lowPct    = 0.0f;
        out->deadPct   = 0.0f;
        return;
    }

    occrHPct = CompareToPct(s_occrH[ch]);
    occrLPct = CompareToPct(s_occrL[ch]);

    out->centerPct = s_centerPct[ch];
    out->deadPct   = s_ctx.deadTimePct;

    if (s_outputType[ch] == TMR4_OUTPUT_COMPLEMENTARY) {
        /* 高侧导通窗口 (Xh, CPSR]：占比 = 100 - Xh/周期，引脚再损失半个死区 */
        out->highPct = 100.0f - occrLPct - s_ctx.deadTimePct * 0.5f;
        out->lowPct  = 100.0f - out->highPct - s_ctx.deadTimePct;
    } else {
        /* 同向：上下管同相，导通窗口 (X, CPSR]，占比 = 100 - X/周期 */
        out->highPct = 100.0f - occrHPct;
        out->lowPct  = 100.0f - occrLPct;
    }
}

/*=============================================================================
 * Channel / pin control
 *=============================================================================*/
tmr4_pwm_err_t hw_tmr4_pwm_channel_cmd(tmr4_pwm_channel_t ch, bool enable)
{
    const pwm_pair_desc_t *pair;

    if (!s_bConfigured)      return TMR4_PWM_ERR_NOT_INIT;
    if (!ChannelIsValid(ch)) return TMR4_PWM_ERR_BAD_PARAM;

    pair = &s_pairs[ch];

    if (s_outputType[ch] == TMR4_OUTPUT_COMPLEMENTARY) {
        TMR4_OC_Cmd(CM_TMR4_3, pair->ocChL, enable ? ENABLE : DISABLE);
        return TMR4_PWM_OK;
    }

    switch (s_chOpMode[ch]) {
    case TMR4_MODE_HIGH_SIDE:
        TMR4_OC_Cmd(CM_TMR4_3, pair->ocChH, enable ? ENABLE : DISABLE);
        break;
    case TMR4_MODE_LOW_SIDE:
        TMR4_OC_Cmd(CM_TMR4_3, pair->ocChL, enable ? ENABLE : DISABLE);
        break;
    default:
        TMR4_OC_Cmd(CM_TMR4_3, pair->ocChH, enable ? ENABLE : DISABLE);
        TMR4_OC_Cmd(CM_TMR4_3, pair->ocChL, enable ? ENABLE : DISABLE);
        break;
    }
    return TMR4_PWM_OK;
}

tmr4_pwm_err_t hw_tmr4_pwm_pin_cmd(tmr4_pwm_channel_t ch, bool highSide, bool enable)
{
    const pwm_pair_desc_t *pair;
    uint32_t ocCh;

    if (!s_bConfigured)      return TMR4_PWM_ERR_NOT_INIT;
    if (!ChannelIsValid(ch)) return TMR4_PWM_ERR_BAD_PARAM;

    if (s_outputType[ch] == TMR4_OUTPUT_COMPLEMENTARY && highSide) {
        return TMR4_PWM_ERR_STATE;
    }
    pair = &s_pairs[ch];
    ocCh = highSide ? pair->ocChH : pair->ocChL;
    TMR4_OC_Cmd(CM_TMR4_3, ocCh, enable ? ENABLE : DISABLE);
    return TMR4_PWM_OK;
}

tmr4_pwm_err_t hw_tmr4_pwm_pin_set_duty(tmr4_pwm_channel_t ch, bool highSide, uint16_t dutyU16)
{
    const pwm_pair_desc_t *pair;
    uint32_t ocCh;
    uint16_t cmpTicks;
    float    pct;

    if (!s_bConfigured)      return TMR4_PWM_ERR_NOT_INIT;
    if (!ChannelIsValid(ch)) return TMR4_PWM_ERR_BAD_PARAM;
    if (s_outputType[ch] == TMR4_OUTPUT_COMPLEMENTARY && highSide) {
        return TMR4_PWM_ERR_STATE;
    }
    if (dutyU16 > TMR4_PWM_DUTY_MAX) dutyU16 = TMR4_PWM_DUTY_MAX;

    pct      = (float)dutyU16 * 100.0f / (float)TMR4_PWM_DUTY_MAX;
    cmpTicks = HighDutyToCmpH(pct);   /* 同向模式：上下管都是峰值导通，映射相同 */
    pair     = &s_pairs[ch];
    ocCh     = highSide ? pair->ocChH : pair->ocChL;

    if (highSide) OccrHWrite((int)ch, ocCh, cmpTicks);
    else          OccrLWrite((int)ch, ocCh, cmpTicks);
    return TMR4_PWM_OK;
}

tmr4_pwm_err_t hw_tmr4_pwm_pin_set_invalid_level(tmr4_pwm_channel_t ch,
                                                 bool highSide, bool levelHigh)
{
    const pwm_pair_desc_t *pair;
    uint32_t ocCh;

    if (!s_bConfigured)      return TMR4_PWM_ERR_NOT_INIT;
    if (!ChannelIsValid(ch)) return TMR4_PWM_ERR_BAD_PARAM;

    pair = &s_pairs[ch];
    ocCh = highSide ? pair->ocChH : pair->ocChL;
    TMR4_OC_SetOcInvalidPolarity(CM_TMR4_3, ocCh,
        levelHigh ? TMR4_OC_INVD_HIGH : TMR4_OC_INVD_LOW);
    return TMR4_PWM_OK;
}

/*=============================================================================
 * SetChannelMode
 *=============================================================================*/
tmr4_pwm_err_t hw_tmr4_pwm_set_channel_mode(tmr4_pwm_channel_t ch,
                                            tmr4_channel_mode_t mode, float centerPct)
{
    const pwm_pair_desc_t *pair;
    stc_tmr4_pwm_init_t stcPwmInit;
    stc_tmr4_oc_init_t  stcOcInit;

    if (!s_bConfigured)      return TMR4_PWM_ERR_NOT_INIT;
    if (!ChannelIsValid(ch)) return TMR4_PWM_ERR_BAD_PARAM;

    pair     = &s_pairs[ch];
    s_chOpMode[ch]   = mode;
    s_centerPct[ch]  = centerPct;

    switch (mode) {
    case TMR4_MODE_OFF:
        TMR4_OC_Cmd(CM_TMR4_3, pair->ocChH, DISABLE);
        TMR4_OC_Cmd(CM_TMR4_3, pair->ocChL, DISABLE);
        TMR4_OC_SetOcInvalidPolarity(CM_TMR4_3, pair->ocChH, TMR4_OC_INVD_LOW);
        TMR4_OC_SetOcInvalidPolarity(CM_TMR4_3, pair->ocChL, TMR4_OC_INVD_LOW);
        TMR4_PWM_StructInit(&stcPwmInit);
        stcPwmInit.u16Mode     = TMR4_PWM_MD_THROUGH;
        stcPwmInit.u16ClockDiv = TMR4_PWM_CLK_DIV1;
        stcPwmInit.u16Polarity = TMR4_PWM_OXH_HOLD_OXL_HOLD;
        TMR4_PWM_Init(CM_TMR4_3, pair->pwmCh, &stcPwmInit);
        break;

    case TMR4_MODE_HIGH_SIDE:
        TMR4_OC_StructInit(&stcOcInit);
        stcOcInit.u16CompareValue        = HighDutyToCmpH(50.0f);
        stcOcInit.u16CompareValueBufCond = TMR4_OC_LOAD_COND();
        TMR4_OC_Init(CM_TMR4_3, pair->ocChH, &stcOcInit);
        OCModeSyncHigh(CM_TMR4_3, pair->ocChH);
        TMR4_OC_Cmd(CM_TMR4_3, pair->ocChH, ENABLE);
        OccrHWrite((int)ch, pair->ocChH, HighDutyToCmpH(centerPct));
        ShadowApplyOC(pair->ocChH);
        TMR4_OC_Cmd(CM_TMR4_3, pair->ocChL, DISABLE);
        TMR4_OC_SetOcInvalidPolarity(CM_TMR4_3, pair->ocChL, TMR4_OC_INVD_LOW);
        TMR4_PWM_StructInit(&stcPwmInit);
        stcPwmInit.u16Mode     = TMR4_PWM_MD_THROUGH;
        stcPwmInit.u16ClockDiv = TMR4_PWM_CLK_DIV1;
        stcPwmInit.u16Polarity = TMR4_PWM_OXH_HOLD_OXL_HOLD;
        TMR4_PWM_Init(CM_TMR4_3, pair->pwmCh, &stcPwmInit);
        break;

    case TMR4_MODE_LOW_SIDE:
        TMR4_OC_Cmd(CM_TMR4_3, pair->ocChH, DISABLE);
        TMR4_OC_SetOcInvalidPolarity(CM_TMR4_3, pair->ocChH, TMR4_OC_INVD_LOW);
        TMR4_OC_StructInit(&stcOcInit);
        stcOcInit.u16CompareValue        = 0U;   /* 低侧常开：X=0 -> 100% */
        stcOcInit.u16CompareValueBufCond = TMR4_OC_LOAD_COND();
        TMR4_OC_Init(CM_TMR4_3, pair->ocChL, &stcOcInit);
        OCModeSyncLow(CM_TMR4_3, pair->ocChL);
        TMR4_OC_Cmd(CM_TMR4_3, pair->ocChL, ENABLE);
        OccrLWrite((int)ch, pair->ocChL, 0U);
        ShadowApplyOC(pair->ocChL);
        TMR4_PWM_StructInit(&stcPwmInit);
        stcPwmInit.u16Mode     = TMR4_PWM_MD_THROUGH;
        stcPwmInit.u16ClockDiv = TMR4_PWM_CLK_DIV1;
        stcPwmInit.u16Polarity = TMR4_PWM_OXH_HOLD_OXL_HOLD;
        TMR4_PWM_Init(CM_TMR4_3, pair->pwmCh, &stcPwmInit);
        break;

    default:
        return TMR4_PWM_ERR_BAD_PARAM;
    }
    return TMR4_PWM_OK;
}

/*=============================================================================
 * SetOutputType
 *=============================================================================*/
tmr4_pwm_err_t hw_tmr4_pwm_set_output_type(tmr4_pwm_channel_t ch, tmr4_output_type_t type)
{
    const pwm_pair_desc_t *pair;
    stc_tmr4_oc_init_t  stcOcInit;
    stc_tmr4_pwm_init_t stcPwmInit;
    uint16_t deadTicks;
    uint16_t polarity;
    float    d50;

    if (!s_bConfigured)      return TMR4_PWM_ERR_NOT_INIT;
    if (!ChannelIsValid(ch)) return TMR4_PWM_ERR_BAD_PARAM;
    if (type != TMR4_OUTPUT_COMPLEMENTARY && type != TMR4_OUTPUT_SYNC) {
        return TMR4_PWM_ERR_BAD_PARAM;
    }
    if (s_outputType[ch] == type) return TMR4_PWM_OK;

    pair = &s_pairs[ch];

    s_chOpMode[ch]   = TMR4_MODE_OFF;
    s_outputType[ch] = type;

    deadTicks = s_ctx.deadTicks;

    TMR4_OC_StructInit(&stcOcInit);
    stcOcInit.u16CompareValueBufCond = TMR4_OC_LOAD_COND();

    d50 = s_centerPct[ch];
    if (d50 < 0.0f)   d50 = 0.0f;
    if (d50 > 100.0f) d50 = 100.0f;

    if (type == TMR4_OUTPUT_COMPLEMENTARY) {
        stcOcInit.u16CompareValue = HighDutyToCmpH(50.0f);
        TMR4_OC_Cmd(CM_TMR4_3, pair->ocChH, DISABLE);
        TMR4_OC_SetOcInvalidPolarity(CM_TMR4_3, pair->ocChH, TMR4_OC_INVD_LOW);
        s_occrH[ch] = 0U;

        TMR4_OC_Init(CM_TMR4_3, pair->ocChL, &stcOcInit);
        OCModeCompLow(CM_TMR4_3, pair->ocChL);
        ShadowApplyOC(pair->ocChL);
        OccrLWrite((int)ch, pair->ocChL, HighDutyToCmpH(d50));

        stcPwmInit.u16Mode     = TMR4_PWM_MD_DEAD_TMR;
        stcPwmInit.u16ClockDiv = TMR4_PWM_CLK_DIV1;
        TMR4_PWM_Init(CM_TMR4_3, pair->pwmCh, &stcPwmInit);
        polarity = ComplementaryPolarity(ch, s_bActiveHigh);
        TMR4_PWM_SetPolarity(CM_TMR4_3, pair->pwmCh, polarity);
        WriteDeadTimeForChannel(pair, deadTicks);

        TMR4_OC_Cmd(CM_TMR4_3, pair->ocChL, ENABLE);
    } else {
        stcOcInit.u16CompareValue = HighDutyToCmpH(50.0f);
        TMR4_OC_Cmd(CM_TMR4_3, pair->ocChL, DISABLE);

        TMR4_OC_Init(CM_TMR4_3, pair->ocChH, &stcOcInit);
        TMR4_OC_Init(CM_TMR4_3, pair->ocChL, &stcOcInit);
        OCModeSyncHigh(CM_TMR4_3, pair->ocChH);
        OCModeSyncLow (CM_TMR4_3, pair->ocChL);
        ShadowApplyOC(pair->ocChH);
        ShadowApplyOC(pair->ocChL);
        ApplySyncDutyWithDeadTime(ch, d50, deadTicks);

        stcPwmInit.u16Mode     = TMR4_PWM_MD_THROUGH;
        stcPwmInit.u16ClockDiv = TMR4_PWM_CLK_DIV1;
        TMR4_PWM_Init(CM_TMR4_3, pair->pwmCh, &stcPwmInit);
        polarity = s_bActiveHigh ? TMR4_PWM_OXH_HOLD_OXL_HOLD
                                 : TMR4_PWM_OXH_INVT_OXL_INVT;
        TMR4_PWM_SetPolarity(CM_TMR4_3, pair->pwmCh, polarity);

        TMR4_OC_Cmd(CM_TMR4_3, pair->ocChH, ENABLE);
        TMR4_OC_Cmd(CM_TMR4_3, pair->ocChL, ENABLE);
    }

    return TMR4_PWM_OK;
}

tmr4_output_type_t hw_tmr4_pwm_get_output_type(tmr4_pwm_channel_t ch)
{
    if (!ChannelIsValid(ch)) return TMR4_OUTPUT_SYNC;
    return s_outputType[ch];
}

/*=============================================================================
 * SetFocMode
 *=============================================================================*/
tmr4_pwm_err_t hw_tmr4_pwm_set_foc_mode(uint16_t deadTimeNs)
{
    tmr4_pwm_err_t err;
    int i;

    if (!s_bConfigured)     return TMR4_PWM_ERR_NOT_INIT;
    if (s_ctx.freqHz == 0U) return TMR4_PWM_ERR_STATE;

    if (deadTimeNs != 0U) {
        err = hw_tmr4_pwm_set_deadtime(deadTimeNs);
        if (err != TMR4_PWM_OK) return err;
    }

    for (i = 0; i < 3; i++) {
        err = hw_tmr4_pwm_set_output_type((tmr4_pwm_channel_t)i,
                                          TMR4_OUTPUT_COMPLEMENTARY);
        if (err != TMR4_PWM_OK) return err;
    }
    return TMR4_PWM_OK;
}

tmr4_pwm_err_t hw_tmr4_pwm_config_adc_trigger(bool enable)
{
    stc_tmr4_evt_init_t stcEvtInit;

    if (!s_bConfigured) return TMR4_PWM_ERR_NOT_INIT;

    TMR4_EVT_StructInit(&stcEvtInit);
    stcEvtInit.u16Mode        = TMR4_EVT_MD_CMP;
    stcEvtInit.u16OutputEvent = TMR4_EVT_OUTPUT_EVT0;

    /* 触发点只与采样臂有关，与输出模式（互补/同向）无关 */
    if (TMR4_SAMPLE_IS_HIGH_SIDE) {
        stcEvtInit.u16CompareValue = (uint16_t)(s_ctx.periodTicks - 1U);
        stcEvtInit.u16MatchCond    = TMR4_EVT_MATCH_CNT_PEAK;
    } else {
        stcEvtInit.u16CompareValue = 0U;
        stcEvtInit.u16MatchCond    = TMR4_EVT_MATCH_CNT_VALLEY;
    }

    (void)TMR4_EVT_Init(CM_TMR4_3, TMR4_EVT_CH_UL, &stcEvtInit);
    TMR4_EVT_MatchCondCmd(CM_TMR4_3, TMR4_EVT_CH_UL, stcEvtInit.u16MatchCond,
                          enable ? ENABLE : DISABLE);
    return TMR4_PWM_OK;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
