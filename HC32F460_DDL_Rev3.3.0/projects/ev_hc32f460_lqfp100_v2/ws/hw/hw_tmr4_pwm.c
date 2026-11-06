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
    uint16_t cond  = TMR4_OC_BUF_COND_PEAK;
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

    ticks = (uint16_t)((uint64_t)deadTimeNs * timerHz / 2000000000ULL);
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

static uint16_t CenterPctToCompare(float centerPct)
{
    int32_t cmp;
    int32_t hi;

    if (centerPct < 0.0f)   centerPct = 0.0f;
    if (centerPct > 100.0f) centerPct = 100.0f;

    hi  = (int32_t)s_ctx.periodTicks - 1;
    cmp = (int32_t)((float)s_ctx.periodTicks * centerPct / 100.0f);

    if (cmp < 0)  cmp = 0;
    if (cmp > hi) cmp = hi;
    return (uint16_t)cmp;
}

static float CompareToPct(uint16_t ticks)
{
    float fPeriod = (float)s_ctx.periodTicks;
    return (fPeriod > 0.0f) ? ((float)ticks * 100.0f / fPeriod) : 0.0f;
}

static void ApplySyncDutyWithDeadTime(tmr4_pwm_channel_t ch,
                                      uint16_t centerTicks,
                                      uint16_t deadTicks)
{
    const pwm_pair_desc_t *pair = &s_pairs[ch];
    int32_t h, l;
    int32_t lo = 0;
    int32_t hi = (int32_t)s_ctx.periodTicks - 1;

    h = (int32_t)centerTicks + (int32_t)deadTicks;
    l = (int32_t)centerTicks - (int32_t)deadTicks;

    if (h > hi) { int32_t s = h - hi; h -= s; l -= s; }
    if (l < lo) { int32_t s = lo - l; l += s; h += s; }
    if (h > hi) h = hi;
    if (l < lo) l = lo;

    OccrHWrite((int)ch, pair->ocChH, (uint16_t)h);
    OccrLWrite((int)ch, pair->ocChL, (uint16_t)l);
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
    m.OCMRx_f.OCFDCH = TMR4_OC_OCF_SET;
    m.OCMRx_f.OCFPKH = TMR4_OC_OCF_SET;
    m.OCMRx_f.OCFUCH = TMR4_OC_OCF_SET;
    m.OCMRx_f.OCFZRH = TMR4_OC_OCF_SET;
    m.OCMRx_f.OPDCH  = TMR4_OC_HIGH;
    m.OCMRx_f.OPPKH  = TMR4_OC_HIGH;
    m.OCMRx_f.OPUCH  = TMR4_OC_LOW;
    m.OCMRx_f.OPZRH  = TMR4_OC_HOLD;
    m.OCMRx_f.OPNPKH = TMR4_OC_LOW;
    m.OCMRx_f.OPNZRH = TMR4_OC_HOLD;
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
    m.OCMRx_f.OPDCL   = TMR4_OC_HIGH;
    m.OCMRx_f.OPPKL   = TMR4_OC_HIGH;
    m.OCMRx_f.OPUCL   = TMR4_OC_LOW;
    m.OCMRx_f.OPZRL   = TMR4_OC_HOLD;
    m.OCMRx_f.OPNPKL  = TMR4_OC_LOW;
    m.OCMRx_f.OPNZRL  = TMR4_OC_HOLD;
    m.OCMRx_f.EOPNDCL = TMR4_OC_HOLD;
    m.OCMRx_f.EOPNUCL = TMR4_OC_HOLD;
    m.OCMRx_f.EOPDCL  = TMR4_OC_HIGH;
    m.OCMRx_f.EOPPKL  = TMR4_OC_HIGH;
    m.OCMRx_f.EOPUCL  = TMR4_OC_LOW;
    m.OCMRx_f.EOPZRL  = TMR4_OC_HOLD;
    m.OCMRx_f.EOPNPKL = TMR4_OC_LOW;
    m.OCMRx_f.EOPNZRL = TMR4_OC_HOLD;
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
    m.OCMRx_f.OPDCL   = TMR4_OC_INVT;
    m.OCMRx_f.OPPKL   = TMR4_OC_INVT;
    m.OCMRx_f.OPUCL   = TMR4_OC_INVT;
    m.OCMRx_f.OPZRL   = TMR4_OC_INVT;
    m.OCMRx_f.OPNPKL  = TMR4_OC_HOLD;
    m.OCMRx_f.OPNZRL  = TMR4_OC_HOLD;
    m.OCMRx_f.EOPNDCL = TMR4_OC_HOLD;
    m.OCMRx_f.EOPNUCL = TMR4_OC_HOLD;
    m.OCMRx_f.EOPDCL  = TMR4_OC_INVT;
    m.OCMRx_f.EOPPKL  = TMR4_OC_INVT;
    m.OCMRx_f.EOPUCL  = TMR4_OC_INVT;
    m.OCMRx_f.EOPZRL  = TMR4_OC_INVT;
    m.OCMRx_f.EOPNPKL = TMR4_OC_HOLD;
    m.OCMRx_f.EOPNZRL = TMR4_OC_HOLD;
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
        *outHighPct = centerPct - dtPct * 0.5f;
        *outLowPct  = 100.0f - (*outHighPct) - dtPct;
    } else {
        *outHighPct = centerPct + dtPct;
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
        *outCenterPct = (highPct + lowPct) * 0.5f;
    }
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
            stcTmr4OcInit.u16CompareValue        = s_ctx.periodTicks / 2U;
            stcTmr4OcInit.u16OcInvalidPolarity   = TMR4_OC_INVD_LOW;
            stcTmr4OcInit.u16CompareValueBufCond = TMR4_OC_BUF_COND_PEAK;
            stcTmr4OcInit.u16CompareModeBufCond  = TMR4_OC_BUF_COND_PEAK;
            stcTmr4OcInit.u16BufLinkTransObject  = TMR4_OC_BUF_CMP_VALUE | TMR4_OC_BUF_CMP_MD;
            TMR4_OC_Init(CM_TMR4_3, pair->ocChL, &stcTmr4OcInit);

            OCModeCompLow(CM_TMR4_3, pair->ocChL);
            ShadowApplyOC(pair->ocChL);
            TMR4_OC_Cmd(CM_TMR4_3, pair->ocChL, ENABLE);

            s_occrH[i] = 0U;
            OccrLWrite(i, pair->ocChL, s_ctx.periodTicks / 2U);

            stcTmr4PwmInit.u16Mode = TMR4_PWM_MD_DEAD_TMR;
            polarity = ComplementaryPolarity((tmr4_pwm_channel_t)i, s_bActiveHigh);

            TMR4_PWM_Init(CM_TMR4_3, pair->pwmCh, &stcTmr4PwmInit);
            TMR4_PWM_SetPolarity(CM_TMR4_3, pair->pwmCh, polarity);
            WriteDeadTimeForChannel(pair, deadTicks);
        } else {
            stcTmr4OcInit.u16CompareValue        = stcTmr4Init.u16PeriodValue / 2U;
            stcTmr4OcInit.u16CompareValueBufCond = TMR4_OC_BUF_COND_PEAK;

            TMR4_OC_Init(CM_TMR4_3, pair->ocChH, &stcTmr4OcInit);
            TMR4_OC_Init(CM_TMR4_3, pair->ocChL, &stcTmr4OcInit);
            OCModeSyncHigh(CM_TMR4_3, pair->ocChH);
            OCModeSyncLow (CM_TMR4_3, pair->ocChL);
            TMR4_OC_Cmd(CM_TMR4_3, pair->ocChH, ENABLE);
            TMR4_OC_Cmd(CM_TMR4_3, pair->ocChL, ENABLE);
            ShadowApplyOC(pair->ocChH);
            ShadowApplyOC(pair->ocChL);

            ApplySyncDutyWithDeadTime((tmr4_pwm_channel_t)i,
                                      (uint16_t)(s_ctx.periodTicks / 2U),
                                      deadTicks);

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

    MAIN_D("[HW_TMR4] configured: period=%u polar=%s dt=%u ns (%u ticks), ADC trig=ON",
           s_ctx.periodTicks, cfg->activeHigh ? "HIGH" : "LOW",
           s_ctx.deadTimeNs, s_ctx.deadTicks);

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
    uint16_t cmpTicks;
    uint16_t deadTicks;

    if (!s_bConfigured)      return TMR4_PWM_ERR_NOT_INIT;
    if (!ChannelIsValid(ch)) return TMR4_PWM_ERR_BAD_PARAM;

    cmpTicks  = CenterPctToCompare(centerPct);
    deadTicks = s_ctx.deadTicks;
    pair      = &s_pairs[ch];
    s_centerPct[ch] = centerPct;

    if (s_outputType[ch] == TMR4_OUTPUT_COMPLEMENTARY) {
        OccrLWrite((int)ch, pair->ocChL, cmpTicks);
        return TMR4_PWM_OK;
    }

    switch (s_chOpMode[ch]) {
    case TMR4_MODE_HIGH_SIDE:
        OccrHWrite((int)ch, pair->ocChH, cmpTicks);
        break;
    case TMR4_MODE_LOW_SIDE:
        break;
    case TMR4_MODE_OFF:
    default:
        ApplySyncDutyWithDeadTime(ch, cmpTicks, deadTicks);
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
        uint16_t cmpTicks = CenterPctToCompare(duty[i]);
        s_centerPct[i] = duty[i];

        if (s_outputType[i] == TMR4_OUTPUT_COMPLEMENTARY) {
            OccrLWrite(i, pair->ocChL, cmpTicks);
        } else {
            if (s_chOpMode[i] == TMR4_MODE_HIGH_SIDE) {
                OccrHWrite(i, pair->ocChH, cmpTicks);
            } else if (s_chOpMode[i] == TMR4_MODE_LOW_SIDE) {
                /* L fixed 100%, no update */
            } else {
                ApplySyncDutyWithDeadTime((tmr4_pwm_channel_t)i, cmpTicks, deadTicks);
            }
        }
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
        if (s_outputType[i] == TMR4_OUTPUT_COMPLEMENTARY) {
            low[i] = 100.0f - high[i] - dtPct;
        } else {
            low[i] = high[i] - 2.0f * dtPct;
        }
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
        out->highPct = occrLPct - s_ctx.deadTimePct * 0.5f;
        out->lowPct  = 100.0f - out->highPct - s_ctx.deadTimePct;
    } else {
        out->highPct = occrHPct;
        out->lowPct  = occrLPct;
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
    cmpTicks = CenterPctToCompare(pct);
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
    uint16_t cmpTicks;

    if (!s_bConfigured)      return TMR4_PWM_ERR_NOT_INIT;
    if (!ChannelIsValid(ch)) return TMR4_PWM_ERR_BAD_PARAM;

    cmpTicks = CenterPctToCompare(centerPct);
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
        stcOcInit.u16CompareValue        = s_ctx.periodTicks / 2U;
        stcOcInit.u16CompareValueBufCond = TMR4_OC_BUF_COND_PEAK;
        TMR4_OC_Init(CM_TMR4_3, pair->ocChH, &stcOcInit);
        OCModeSyncHigh(CM_TMR4_3, pair->ocChH);
        TMR4_OC_Cmd(CM_TMR4_3, pair->ocChH, ENABLE);
        OccrHWrite((int)ch, pair->ocChH, cmpTicks);
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
        stcOcInit.u16CompareValue        = s_ctx.periodTicks / 2U;
        stcOcInit.u16CompareValueBufCond = TMR4_OC_BUF_COND_PEAK;
        TMR4_OC_Init(CM_TMR4_3, pair->ocChL, &stcOcInit);
        OCModeSyncLow(CM_TMR4_3, pair->ocChL);
        TMR4_OC_Cmd(CM_TMR4_3, pair->ocChL, ENABLE);
        OccrLWrite((int)ch, pair->ocChL, s_ctx.periodTicks - 1U);
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
    stcOcInit.u16CompareValue        = s_ctx.periodTicks / 2U;
    stcOcInit.u16CompareValueBufCond = TMR4_OC_BUF_COND_PEAK;

    d50 = s_centerPct[ch];
    if (d50 < 0.0f)   d50 = 0.0f;
    if (d50 > 100.0f) d50 = 100.0f;

    if (type == TMR4_OUTPUT_COMPLEMENTARY) {
        TMR4_OC_Cmd(CM_TMR4_3, pair->ocChH, DISABLE);
        TMR4_OC_SetOcInvalidPolarity(CM_TMR4_3, pair->ocChH, TMR4_OC_INVD_LOW);
        s_occrH[ch] = 0U;

        TMR4_OC_Init(CM_TMR4_3, pair->ocChL, &stcOcInit);
        OCModeCompLow(CM_TMR4_3, pair->ocChL);
        ShadowApplyOC(pair->ocChL);
        OccrLWrite((int)ch, pair->ocChL, CenterPctToCompare(d50));

        stcPwmInit.u16Mode     = TMR4_PWM_MD_DEAD_TMR;
        stcPwmInit.u16ClockDiv = TMR4_PWM_CLK_DIV1;
        TMR4_PWM_Init(CM_TMR4_3, pair->pwmCh, &stcPwmInit);
        polarity = ComplementaryPolarity(ch, s_bActiveHigh);
        TMR4_PWM_SetPolarity(CM_TMR4_3, pair->pwmCh, polarity);
        WriteDeadTimeForChannel(pair, deadTicks);

        TMR4_OC_Cmd(CM_TMR4_3, pair->ocChL, ENABLE);
    } else {
        TMR4_OC_Cmd(CM_TMR4_3, pair->ocChL, DISABLE);

        TMR4_OC_Init(CM_TMR4_3, pair->ocChH, &stcOcInit);
        TMR4_OC_Init(CM_TMR4_3, pair->ocChL, &stcOcInit);
        OCModeSyncHigh(CM_TMR4_3, pair->ocChH);
        OCModeSyncLow (CM_TMR4_3, pair->ocChL);
        ShadowApplyOC(pair->ocChH);
        ShadowApplyOC(pair->ocChL);
        ApplySyncDutyWithDeadTime(ch, CenterPctToCompare(d50), deadTicks);

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

		if (FOC_CFG_SAMPLE_POINT == CFG_SAMPLE_UPPER) {
				stcEvtInit.u16CompareValue = 0U;
				stcEvtInit.u16MatchCond    = TMR4_EVT_MATCH_CNT_VALLEY;
		} else {
				stcEvtInit.u16CompareValue = (uint16_t)(s_ctx.periodTicks - 1U);
				stcEvtInit.u16MatchCond    = TMR4_EVT_MATCH_CNT_PEAK;
		}

    (void)TMR4_EVT_Init(CM_TMR4_3, TMR4_EVT_CH_UL, &stcEvtInit);
    (void)enable;
    return TMR4_PWM_OK;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
