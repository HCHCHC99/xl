#include "Pwm.h"
#include "hc32_ll_clk.h"
#include "hc32_ll_fcg.h"
#include "hc32_ll_gpio.h"
#include "hc32_ll_tmra.h"
#include <stdbool.h>
#include "TickTimer.h"

/* 常量定义 */
#define PWM_MAX_CHANNELS            (8U)        // 最大通道数
#define PWM_DUTY_MAX                (100U)      // 最大占空比
#define PWM_DUTY_SCALE              (1000U)     // 占空比计算缩放因子
#define PWM_FREQ_MIN                (1U)        // 最小频率
#define PWM_RAMP_STEP_SIZE          (1U)        // 缓启动步进大小(%)
#define PWM_RAMP_MIN_TIME_MS        (10U)       // 最小缓启动时间(ms)

/* 静态函数声明 */
static bool pwmCalcPeriodValue(uint32_t freqHz, uint8_t countMode, uint32_t *period);
static uint32_t pwmCalcCompareValue(uint32_t period, uint16_t dutyPercent);
static void pwmWriteDuty(pwm_t *pwm, uint16_t dutyPercent);
static pwm_error_t pwmStartRamp(pwm_t *pwm, uint16_t targetDuty,
                                 uint16_t rampTimeMs, bool useCurrent);

/**
 * 计算PWM周期值
 */
static bool pwmCalcPeriodValue(uint32_t freqHz, uint8_t countMode, uint32_t *period)
{
    stc_clock_freq_t clkFreq;
    uint32_t timerClk;
    uint64_t temp;

    if ((0U == freqHz) || (NULL == period)) {
        return false;
    }

    if (CLK_GetClockFreq(&clkFreq) != 0) {
        return false;
    }

    timerClk = clkFreq.u32Pclk1Freq;
    temp = (uint64_t)timerClk;

    if (countMode == TMRA_MD_TRIANGLE) {   // 三角波模式频率减半
        temp /= 2U;
    }

    *period = (uint32_t)(temp / freqHz);

    if (*period <= 1U) {
        *period = 1U;
    } else {
        *period -= 1U;
    }

    return true;
}

/**
 * 计算比较值(带四舍五入)
 */
static uint32_t pwmCalcCompareValue(uint32_t period, uint16_t dutyPercent)
{
    uint64_t temp;

    temp = (uint64_t)period * dutyPercent * PWM_DUTY_SCALE / 100U;
    return (uint32_t)((temp + (PWM_DUTY_SCALE / 2U)) / PWM_DUTY_SCALE);
}

/**
 * 直接写入硬件占空比
 */
static void pwmWriteDuty(pwm_t *pwm, uint16_t dutyPercent)
{
    if ((NULL == pwm) || (NULL == pwm->tmr)) {
        return;
    }

    uint32_t compare = pwmCalcCompareValue(pwm->periodValue, dutyPercent);
    TMRA_SetCompareValue(pwm->tmr, pwm->channel, compare);
    pwm->dutyPercent = dutyPercent;
}

/**
 * 启动缓启动内部实现
 */
static pwm_error_t pwmStartRamp(pwm_t *pwm, uint16_t targetDuty,
                                 uint16_t rampTimeMs, bool useCurrent)
{
    uint16_t startDuty;
    uint16_t dutyDiff;

    if (NULL == pwm) {
        return PWM_ERR_NULL_PTR;
    }

    if ((NULL == pwm->tmr) || (!pwm->initialized)) {
        return PWM_ERR_NOT_INIT;
    }

    if (targetDuty > PWM_DUTY_MAX) {
        return PWM_ERR_INVALID_PARAM;
    }

    if (rampTimeMs < PWM_RAMP_MIN_TIME_MS) {
        rampTimeMs = PWM_RAMP_MIN_TIME_MS;
    }

    pwm->state = PWM_STATE_IDLE;  // 停止当前缓启动

    // 计算起始占空比
    if (useCurrent) {
        startDuty = pwm->dutyPercent;
    } else {
        startDuty = (targetDuty > pwm->dutyPercent) ? 0U : PWM_DUTY_MAX;
    }

    // 计算差值
    dutyDiff = (targetDuty > startDuty) ?
                (targetDuty - startDuty) : (startDuty - targetDuty);

    // 计算步数
    pwm->ramp.totalSteps = dutyDiff / PWM_RAMP_STEP_SIZE;
    if (pwm->ramp.totalSteps < 1U) {
        pwm->ramp.totalSteps = 1U;
    }

    // 计算步进延时
    pwm->ramp.stepDelayMs = rampTimeMs / pwm->ramp.totalSteps;
    if (pwm->ramp.stepDelayMs < PWM_RAMP_MIN_TIME_MS) {
        pwm->ramp.stepDelayMs = PWM_RAMP_MIN_TIME_MS;
        pwm->ramp.totalSteps = rampTimeMs / pwm->ramp.stepDelayMs;
        if (pwm->ramp.totalSteps < 1U) {
            pwm->ramp.totalSteps = 1U;
        }
    }

    // 初始化缓启动参数
    pwm->ramp.startDuty = startDuty;
    pwm->ramp.targetDuty = targetDuty;
    pwm->ramp.currentDuty = startDuty;
    pwm->ramp.currentStep = 0U;
    pwm->ramp.lastUpdateTick = tickTimer_GetCount();
    pwm->state = PWM_STATE_RAMPING;

    pwmWriteDuty(pwm, startDuty);  // 设置初始占空比

    return PWM_OK;
}

/*---------------------------------------------------------------------------
 * 全局函数实现
 *---------------------------------------------------------------------------*/

/**
 * 初始化PWM
 */
pwm_t PWM_Init(CM_TMRA_TypeDef *tmr, uint32_t periphClk, uint32_t channel,
               uint8_t port, uint16_t pin, uint16_t pinFunc,
               uint8_t countMode, uint8_t countDir,
               uint32_t freqHz, uint16_t dutyPercent,
               pwm_active_t active)
{
    pwm_t pwm = {0};
    stc_tmra_init_t tmraInit;
    stc_tmra_pwm_init_t pwmInit;
    stc_gpio_init_t gpioInit;
    uint32_t period;

    if ((NULL == tmr) || (dutyPercent > PWM_DUTY_MAX) || (freqHz < PWM_FREQ_MIN)) {
        return pwm;
    }

    if (!pwmCalcPeriodValue(freqHz, countMode, &period)) {
        return pwm;
    }

    FCG_Fcg2PeriphClockCmd(periphClk, ENABLE);  // 使能时钟

    // 初始化TimerA
    (void)TMRA_StructInit(&tmraInit);
    tmraInit.u8CountSrc = TMRA_CNT_SRC_SW;
    tmraInit.sw_count.u8CountMode = countMode;
    tmraInit.sw_count.u8CountDir = countDir;
    tmraInit.sw_count.u8ClockDiv = TMRA_CLK_DIV1;
    tmraInit.u32PeriodValue = period;
    (void)TMRA_Init(tmr, &tmraInit);

    // 初始化GPIO
    (void)GPIO_StructInit(&gpioInit);
    gpioInit.u16PinDir = PIN_DIR_OUT;
    gpioInit.u16PinDrv = PIN_HIGH_DRV;
    gpioInit.u16PinAttr = PIN_ATTR_DIGITAL;
    GPIO_Init(port, pin, &gpioInit);
    GPIO_SetFunc(port, pin, pinFunc);

    // 配置PWM
    (void)TMRA_PWM_StructInit(&pwmInit);
    pwmInit.u32CompareValue = pwmCalcCompareValue(period, dutyPercent);

    if (active == PWM_ACTIVE_HIGH) {   // 高有效配置
        pwmInit.u16StartPolarity = TMRA_PWM_LOW;
        pwmInit.u16StopPolarity = TMRA_PWM_LOW;
        pwmInit.u16CompareMatchPolarity = TMRA_PWM_HIGH;
        pwmInit.u16PeriodMatchPolarity = TMRA_PWM_LOW;
    } else {                           // 低有效配置
        pwmInit.u16StartPolarity = TMRA_PWM_HIGH;
        pwmInit.u16StopPolarity = TMRA_PWM_HIGH;
        pwmInit.u16CompareMatchPolarity = TMRA_PWM_LOW;
        pwmInit.u16PeriodMatchPolarity = TMRA_PWM_HIGH;
    }

    (void)TMRA_PWM_Init(tmr, channel, &pwmInit);
    TMRA_PWM_OutputCmd(tmr, channel, ENABLE);

    if (countMode == TMRA_MD_TRIANGLE) {
        TMRA_SetCountValue(tmr, 1UL);
    }

    // 填充实例
    pwm.tmr = tmr;
    pwm.periphClk = periphClk;
    pwm.channel = channel;
    pwm.port = port;
    pwm.pin = pin;
    pwm.pinFunc = pinFunc;
    pwm.countMode = countMode;
    pwm.countDir = countDir;
    pwm.freqHz = freqHz;
    pwm.dutyPercent = dutyPercent;
    pwm.activeHigh = (active == PWM_ACTIVE_HIGH);
    pwm.periodValue = period;
    pwm.initialized = true;
    pwm.state = PWM_STATE_IDLE;

    return pwm;
}

/**
 * 启动PWM定时器
 */
pwm_error_t PWM_Start(pwm_t *pwm)
{
    if (NULL == pwm) {
        return PWM_ERR_NULL_PTR;
    }

    if ((NULL == pwm->tmr) || (!pwm->initialized)) {
        return PWM_ERR_NOT_INIT;
    }

    TMRA_Start(pwm->tmr);
    return PWM_OK;
}

/**
 * 停止PWM定时器
 */
pwm_error_t PWM_Stop(pwm_t *pwm)
{
    if (NULL == pwm) {
        return PWM_ERR_NULL_PTR;
    }

    if ((NULL == pwm->tmr) || (!pwm->initialized)) {
        return PWM_ERR_NOT_INIT;
    }

    TMRA_Stop(pwm->tmr);
    return PWM_OK;
}

/**
 * 控制PWM输出使能
 */
pwm_error_t PWM_OutputCmd(pwm_t *pwm, pwm_output_state_t state)
{
    if (NULL == pwm) {
        return PWM_ERR_NULL_PTR;
    }

    if ((NULL == pwm->tmr) || (!pwm->initialized)) {
        return PWM_ERR_NOT_INIT;
    }

    TMRA_PWM_OutputCmd(pwm->tmr, pwm->channel, (en_functional_state_t)state);
    return PWM_OK;
}

/**
 * 设置PWM占空比(立即生效)
 */
pwm_error_t PWM_SetDuty(pwm_t *pwm, uint16_t dutyPercent)
{
    if (NULL == pwm) {
        return PWM_ERR_NULL_PTR;
    }

    if ((NULL == pwm->tmr) || (!pwm->initialized)) {
        return PWM_ERR_NOT_INIT;
    }

    if (dutyPercent > PWM_DUTY_MAX) {
        return PWM_ERR_INVALID_PARAM;
    }

    pwm->state = PWM_STATE_IDLE;  // 停止缓启动
    pwmWriteDuty(pwm, dutyPercent);

    return PWM_OK;
}

/**
 * 设置PWM频率(保持各通道占空比不变)
 */
pwm_error_t PWM_SetFrequency(pwm_t *pwm, uint32_t freqHz)
{
    uint32_t newPeriod;
    uint32_t compare;
    uint32_t tempCompare;
    uint32_t ch;
    uint16_t dutyPercent[PWM_MAX_CHANNELS] = {0};
    uint32_t channelMask = 0;

    if (NULL == pwm) {
        return PWM_ERR_NULL_PTR;
    }

    if ((NULL == pwm->tmr) || (!pwm->initialized)) {
        return PWM_ERR_NOT_INIT;
    }

    if (freqHz < PWM_FREQ_MIN) {
        return PWM_ERR_INVALID_PARAM;
    }

    // 保存所有通道的占空比
    for (ch = TMRA_CH1; ch <= TMRA_CH8; ch++) {
        tempCompare = TMRA_GetCompareValue(pwm->tmr, ch);
        if (tempCompare > 0U) {
            uint64_t temp = (uint64_t)tempCompare * PWM_DUTY_MAX * PWM_DUTY_SCALE;
            dutyPercent[ch] = (uint16_t)((temp + (pwm->periodValue * PWM_DUTY_SCALE / 2U))
                                         / (pwm->periodValue * PWM_DUTY_SCALE));
            channelMask |= (1UL << ch);
        }
    }

    if (!pwmCalcPeriodValue(freqHz, pwm->countMode, &newPeriod)) {
        return PWM_ERR_CLK_FAIL;
    }

    TMRA_SetPeriodValue(pwm->tmr, newPeriod);  // 更新周期

    // 更新所有通道的比较值
    for (ch = TMRA_CH1; ch <= TMRA_CH8; ch++) {
        if (channelMask & (1UL << ch)) {
            compare = pwmCalcCompareValue(newPeriod, dutyPercent[ch]);
            TMRA_SetCompareValue(pwm->tmr, ch, compare);
        }
    }

    pwm->periodValue = newPeriod;
    pwm->freqHz = freqHz;

    return PWM_OK;
}

/**
 * 获取周期值
 */
uint32_t PWM_GetPeriodValue(const pwm_t *pwm)
{
    if ((NULL != pwm) && (pwm->initialized)) {
        return pwm->periodValue;
    }
    return 0U;
}

/**
 * 获取当前频率
 */
uint32_t PWM_GetFreq(const pwm_t *pwm)
{
    if ((NULL != pwm) && (pwm->initialized)) {
        return pwm->freqHz;
    }
    return 0U;
}

/**
 * 获取当前占空比
 */
uint16_t PWM_GetDuty(const pwm_t *pwm)
{
    if ((NULL != pwm) && (pwm->initialized)) {
        return pwm->dutyPercent;
    }
    return 0U;
}

/**
 * 获取有效电平极性
 */
bool PWM_GetPolarity(const pwm_t *pwm)
{
    if ((NULL != pwm) && (pwm->initialized)) {
        return pwm->activeHigh;
    }
    return true;
}

/**
 * 获取当前状态
 */
pwm_state_t PWM_GetState(const pwm_t *pwm)
{
    if ((NULL != pwm) && (pwm->initialized)) {
        return pwm->state;
    }
    return PWM_STATE_IDLE;
}

/**
 * 从当前占空比开始缓启动
 */
pwm_error_t PWM_StartRampFromCurrent(pwm_t *pwm, uint16_t targetDuty, uint16_t rampTimeMs)
{
    return pwmStartRamp(pwm, targetDuty, rampTimeMs, true);
}

/**
 * 从0%或100%开始缓启动
 */
pwm_error_t PWM_StartRamp(pwm_t *pwm, uint16_t targetDuty, uint16_t rampTimeMs)
{
    return pwmStartRamp(pwm, targetDuty, rampTimeMs, false);
}

/**
 * 停止缓启动
 */
pwm_error_t PWM_StopRamp(pwm_t *pwm)
{
    if (NULL == pwm) {
        return PWM_ERR_NULL_PTR;
    }

    if ((NULL == pwm->tmr) || (!pwm->initialized)) {
        return PWM_ERR_NOT_INIT;
    }

    pwm->state = PWM_STATE_IDLE;
    return PWM_OK;
}

/**
 * 更新PWM状态(需在主循环中周期性调用)
 */
pwm_error_t PWM_Update(pwm_t *pwm)
{
    uint64_t currentTick;
    uint64_t elapsed;
    uint32_t newDuty;

    if (NULL == pwm) {
        return PWM_ERR_NULL_PTR;
    }

    if ((NULL == pwm->tmr) || (!pwm->initialized)) {
        return PWM_ERR_NOT_INIT;
    }

    if (pwm->state != PWM_STATE_RAMPING) {  // 只在缓启动状态处理
        return PWM_OK;
    }

    currentTick = tickTimer_GetCount();
    elapsed = currentTick - pwm->ramp.lastUpdateTick;

    if (elapsed >= pwm->ramp.stepDelayMs) {  // 到达步进时间
        pwm->ramp.currentStep++;

        if (pwm->ramp.currentStep >= pwm->ramp.totalSteps) {
            pwm->ramp.currentDuty = pwm->ramp.targetDuty;  // 到达终点
            pwm->state = PWM_STATE_COMPLETE;
        } else {
            // 线性插值计算新占空比
            uint32_t step = pwm->ramp.currentStep;
            uint32_t total = pwm->ramp.totalSteps;
            uint32_t start = pwm->ramp.startDuty;
            uint32_t target = pwm->ramp.targetDuty;

            if (target > start) {
                newDuty = start + ((target - start) * step) / total;
            } else {
                newDuty = start - ((start - target) * step) / total;
            }

            pwm->ramp.currentDuty = (uint16_t)newDuty;
        }

        pwmWriteDuty(pwm, pwm->ramp.currentDuty);  // 更新硬件
        pwm->ramp.lastUpdateTick = currentTick;

        if (pwm->state == PWM_STATE_COMPLETE) {
            pwm->state = PWM_STATE_IDLE;
        }
    }

    return PWM_OK;
}
/*=============================================================================
 * 从指定起始占空比开始缓启动
 *============================================================================*/
pwm_error_t PWM_StartRamp_TargetFromStart(pwm_t *pwm, uint16_t startDuty, uint16_t targetDuty, uint16_t rampTimeMs)
{
    uint16_t dutyDiff;

    if (NULL == pwm) {
        return PWM_ERR_NULL_PTR;
    }

    if ((NULL == pwm->tmr) || (!pwm->initialized)) {
        return PWM_ERR_NOT_INIT;
    }

    if (targetDuty > PWM_DUTY_MAX) {
        return PWM_ERR_INVALID_PARAM;
    }

    if (startDuty > PWM_DUTY_MAX) {
        return PWM_ERR_INVALID_PARAM;
    }

    if (rampTimeMs < PWM_RAMP_MIN_TIME_MS) {
        rampTimeMs = PWM_RAMP_MIN_TIME_MS;
    }

    pwm->state = PWM_STATE_IDLE;  // 停止当前缓启动

    // 计算差值
    dutyDiff = (targetDuty > startDuty) ?
                (targetDuty - startDuty) : (startDuty - targetDuty);

    // 计算步数
    pwm->ramp.totalSteps = dutyDiff / PWM_RAMP_STEP_SIZE;
    if (pwm->ramp.totalSteps < 1U) {
        pwm->ramp.totalSteps = 1U;
    }

    // 计算步进延时
    pwm->ramp.stepDelayMs = rampTimeMs / pwm->ramp.totalSteps;
    if (pwm->ramp.stepDelayMs < PWM_RAMP_MIN_TIME_MS) {
        pwm->ramp.stepDelayMs = PWM_RAMP_MIN_TIME_MS;
        pwm->ramp.totalSteps = rampTimeMs / pwm->ramp.stepDelayMs;
        if (pwm->ramp.totalSteps < 1U) {
            pwm->ramp.totalSteps = 1U;
        }
    }

    // 初始化缓启动参数
    pwm->ramp.startDuty = startDuty;
    pwm->ramp.targetDuty = targetDuty;
    pwm->ramp.currentDuty = startDuty;
    pwm->ramp.currentStep = 0U;
    pwm->ramp.lastUpdateTick = tickTimer_GetCount();
    pwm->state = PWM_STATE_RAMPING;

    // 立即设置起始占空比
    pwmWriteDuty(pwm, startDuty);

    return PWM_OK;
}
