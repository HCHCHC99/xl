/*=============================================================================
 * foc_protect.c  —  过流保护层实现（两层窗口 + 恢复状态机，无堵转）
 *
 * 数据流：
 *   20kHz EOCA: Foc_Control_Step() -> Foc_Protect_PushIsr(iu,iv,iw)
 *               -> 写 s_fast_buf / s_slow_buf，头指针自增，不判断
 *   1ms  中断: My_1ms_Task() -> Foc_Protect_Task1ms()
 *               -> 遍历缓冲区 -> 快/慢判断 -> 触发/恢复
 *
 * 恢复策略：
 *   TRIP   : 电流 < i_recover_a 持续 recover_delay_ms -> 自动 NORMAL
 *   LATCHED: 若 FP_LATCH_AUTO_RECOVER_MS > 0，
 *            电流 < i_recover_a 持续该 ms 数 -> 自动 NORMAL（保留 trip_cnt）
 *            否则必须手动 Foc_Protect_ClearFault()
 *
 * 本版新增诊断打印：
 *   [FP][STATE-DBG] 每 500ms 打一次当前状态/故障码/快慢层值
 *   [FP][LATCH-DBG] LATCHED 下每 500ms 打一次三相电流与恢复计数
 *=============================================================================*/

#include "foc_protect.h"
#include "hw_tmr4_pwm.h"
#include "foc_mode.h"
#include "Timer0_Unit2.h"     /* tickTimer_GetCount 所在（TickTimer.h 被它包含） */
#include "rtt_log.h"

#include <math.h>
#include <string.h>
#include <stddef.h>

/* main.c 的全局：Trip 时置 0，防止主循环把模式覆盖回 IF */
extern volatile int32_t g_mode_cmd;

/* foc_control.c 的电流镜像：恢复任务读当前值（不读缓冲区） */
extern volatile float g_fc_iu;
extern volatile float g_fc_iv;
extern volatile float g_fc_iw;

/*=============================================================================
 * 环形缓冲区（单生产者 20kHz / 单消费者 1ms，无锁）
 *=============================================================================*/
static volatile float    s_fast_buf[FP_FAST_LEN];
static volatile uint32_t s_fast_head = 0u;
static volatile float    s_slow_buf[FP_SLOW_LEN];
static volatile uint32_t s_slow_head = 0u;

/*=============================================================================
 * 内部状态
 *=============================================================================*/
static FocProtectCfg_t s_cfg;
static volatile uint8_t s_enabled   = 0u;
static uint32_t         s_init_ms   = 0u;

static uint16_t s_slow_confirm_cnt = 0u;   /* 慢层连续确认计数（1ms 单位） */
static uint16_t s_recover_cnt      = 0u;   /* TRIP 恢复计时（1ms 单位） */

#if (FP_LATCH_AUTO_RECOVER_MS > 0u)
static uint16_t s_latched_recover_cnt = 0u; /* LATCHED 自动恢复计时（1ms 单位） */
#endif

/*=============================================================================
 * Watch 观测量
 *=============================================================================*/
volatile fp_state_t g_fp_state    = FP_STATE_NORMAL;
volatile uint32_t   g_fp_fault    = FOC_FAULT_NONE;
volatile float      g_fp_fast_max = 0.0f;
volatile float      g_fp_slow_avg = 0.0f;
volatile uint32_t   g_fp_trip_cnt = 0u;

/*=============================================================================
 * 内部函数声明
 *=============================================================================*/
static void Foc_Protect_Trip(uint32_t fault);
static void Foc_Protect_RecoverTask(void);
static void Foc_Protect_LoadCfgFromMacros(FocProtectCfg_t *cfg);

/*=============================================================================
 * 默认配置（从宏填）
 *=============================================================================*/
static void Foc_Protect_LoadCfgFromMacros(FocProtectCfg_t *cfg)
{
    if (cfg == NULL) return;

    cfg->i_fast_trip_a    = FP_I_FAST_TRIP_A;
    cfg->i_slow_trip_a    = FP_I_SLOW_TRIP_A;
    cfg->i_recover_a      = FP_I_RECOVER_A;
    cfg->slow_confirm_ms  = FP_SLOW_CONFIRM_MS;
    cfg->recover_delay_ms = FP_RECOVER_DELAY_MS;
    cfg->latch_after_trips= (uint8_t)FP_LATCH_AFTER_TRIPS;
}

void Foc_Protect_GetDefaultCfg(FocProtectCfg_t *cfg)
{
    Foc_Protect_LoadCfgFromMacros(cfg);
}

/*=============================================================================
 * 初始化
 *=============================================================================*/
void Foc_Protect_Init(const FocProtectCfg_t *cfg)
{
    if (cfg != NULL) {
        s_cfg = *cfg;
    } else {
        Foc_Protect_LoadCfgFromMacros(&s_cfg);
    }

    memset((void *)s_fast_buf, 0, sizeof(s_fast_buf));
    memset((void *)s_slow_buf, 0, sizeof(s_slow_buf));
    s_fast_head = 0u;
    s_slow_head = 0u;

    s_slow_confirm_cnt = 0u;
    s_recover_cnt      = 0u;
#if (FP_LATCH_AUTO_RECOVER_MS > 0u)
    s_latched_recover_cnt = 0u;
#endif

    g_fp_state    = FP_STATE_NORMAL;
    g_fp_fault    = FOC_FAULT_NONE;
    g_fp_fast_max = 0.0f;
    g_fp_slow_avg = 0.0f;
    g_fp_trip_cnt = 0u;

    s_init_ms = tickTimer_GetCount();
    s_enabled = 0u;   /* 启动屏蔽 FP_STARTUP_BLANK_MS 后自动使能 */

    MAIN_D("[FP] init: fast_x100=%ldA slow_x100=%ldA rec_x100=%ldA "
           "confirm=%ums latch=%u auto_rec=%ums",
           (long)(s_cfg.i_fast_trip_a * 100.0f),
           (long)(s_cfg.i_slow_trip_a * 100.0f),
           (long)(s_cfg.i_recover_a   * 100.0f),
           (unsigned)s_cfg.slow_confirm_ms,
           (unsigned)s_cfg.latch_after_trips,
           (unsigned)FP_LATCH_AUTO_RECOVER_MS);
}

void Foc_Protect_Enable(void)  { s_enabled = 1u; }
void Foc_Protect_Disable(void) { s_enabled = 0u; }

/*=============================================================================
 * 20kHz EOCA：只搬运
 *=============================================================================*/
void Foc_Protect_PushIsr(float iu, float iv, float iw)
{
    float a, b, c, imax;

    a = fabsf(iu);
    b = fabsf(iv);
    c = fabsf(iw);
    imax = a;
    if (b > imax) imax = b;
    if (c > imax) imax = c;

    s_fast_buf[s_fast_head & (FP_FAST_LEN - 1u)] = imax;
    s_fast_head++;

    s_slow_buf[s_slow_head & (FP_SLOW_LEN - 1u)] = imax;
    s_slow_head++;
}

/*=============================================================================
 * 触发动作：复用 IDLE 全低机制
 *=============================================================================*/
static void Foc_Protect_Trip(uint32_t fault)
{
    /* 1. 六路全低（不 Stop TMR4，保持 EOCA 链） */
    (void)hw_tmr4_pwm_set_all_low();

    /* 2. 切 IDLE 模式（让 20kHz 控制链走 IDLE 分支） */
    Foc_Mode_SetMode(FOC_MODE_IDLE);

    /* 3. 同步 main.c 的模式指令，防止主循环下一圈覆盖回 IF */
    g_mode_cmd = 0;

    /* 4. 记录故障与触发次数 */
    g_fp_fault |= fault;
    g_fp_trip_cnt++;

    /* 5. 状态机：累计触发达到 latch 次数则锁存，否则 TRIP 等恢复 */
    if (g_fp_trip_cnt >= (uint32_t)s_cfg.latch_after_trips) {
        g_fp_state = FP_STATE_LATCHED;
    } else {
        g_fp_state = FP_STATE_TRIP;
    }

    /* 6. 清确认/恢复计数 */
    s_slow_confirm_cnt = 0u;
    s_recover_cnt      = 0u;
#if (FP_LATCH_AUTO_RECOVER_MS > 0u)
    s_latched_recover_cnt = 0u;
#endif

    MAIN_E("[FP] TRIP fault=0x%08lX cnt=%lu state=%d "
           "iu_x100=%ld iv_x100=%ld iw_x100=%ld",
           (unsigned long)g_fp_fault, (unsigned long)g_fp_trip_cnt,
           (int)g_fp_state,
           (long)(g_fc_iu * 100.0f),
           (long)(g_fc_iv * 100.0f),
           (long)(g_fc_iw * 100.0f));
}

/*=============================================================================
 * 恢复任务：只在 TRIP 状态工作
 *   读当前三相电流最大值（不读缓冲区），回落到 i_recover_a 以下开始计时，
 *   计时达到 recover_delay_ms 后回 NORMAL，清故障码
 *   注意：trip_cnt 不清（累计整个运行期），只有 ClearFault 才清零
 *=============================================================================*/
static void Foc_Protect_RecoverTask(void)
{
    float a, b, c, imax;

    if (g_fp_state != FP_STATE_TRIP) {
        return;
    }

    a = fabsf(g_fc_iu);
    b = fabsf(g_fc_iv);
    c = fabsf(g_fc_iw);
    imax = a;
    if (b > imax) imax = b;
    if (c > imax) imax = c;

    if (imax < s_cfg.i_recover_a) {
        if (++s_recover_cnt >= s_cfg.recover_delay_ms) {
            s_recover_cnt = 0u;
            g_fp_fault    = FOC_FAULT_NONE;
            g_fp_state    = FP_STATE_NORMAL;
            MAIN_D("[FP] recover -> NORMAL (trip_cnt=%lu kept)",
                   (unsigned long)g_fp_trip_cnt);
        }
    } else {
        s_recover_cnt = 0u;
    }
}

/*---------------------------------------------------------------------------
 * LATCHED 自动恢复任务（带诊断打印）
 *   条件：电流持续 < i_recover_a 达到 FP_LATCH_AUTO_RECOVER_MS
 *   动作：清 fault、回 NORMAL；trip_cnt 保留（便于诊断"曾经锁存过"）
 *         如需连 trip_cnt 一起清，打开 g_fp_trip_cnt = 0u;
 *   诊断：每 500ms 打一次三相电流与恢复计数，用于定位"卡在 LATCHED"的原因
 *---------------------------------------------------------------------------*/
#if (FP_LATCH_AUTO_RECOVER_MS > 0u)
static void Foc_Protect_LatchedRecoverTask(void)
{
    float a, b, c, imax;
    static uint32_t s_dbg_last_ms = 0;

    a = fabsf(g_fc_iu);
    b = fabsf(g_fc_iv);
    c = fabsf(g_fc_iw);
    imax = a;
    if (b > imax) imax = b;
    if (c > imax) imax = c;

    /* ---- 诊断打印：每 500ms 一次 ---- */
    {
        uint32_t now = tickTimer_GetCount();
        if ((now - s_dbg_last_ms) >= 500u) {
            s_dbg_last_ms = now;
            MAIN_D("[FP][LATCH-DBG] iu_x100=%ld iv_x100=%ld iw_x100=%ld "
                   "imax_x100=%ld rec_thr_x100=%ld rec_cnt=%u/%u",
                   (long)(g_fc_iu * 100.0f),
                   (long)(g_fc_iv * 100.0f),
                   (long)(g_fc_iw * 100.0f),
                   (long)(imax * 100.0f),
                   (long)(s_cfg.i_recover_a * 100.0f),
                   (unsigned)s_latched_recover_cnt,
                   (unsigned)FP_LATCH_AUTO_RECOVER_MS);
        }
    }

    if (imax < s_cfg.i_recover_a) {
        if (++s_latched_recover_cnt >= FP_LATCH_AUTO_RECOVER_MS) {
            s_latched_recover_cnt = 0u;
            s_slow_confirm_cnt    = 0u;
            s_recover_cnt         = 0u;
            g_fp_fault            = FOC_FAULT_NONE;
            g_fp_state            = FP_STATE_NORMAL;
            /* g_fp_trip_cnt = 0u; */   /* 需要连次数一起清就打开这行 */
            MAIN_D("[FP] LATCHED auto recover -> NORMAL (trip_cnt=%lu kept)",
                   (unsigned long)g_fp_trip_cnt);
        }
    } else {
        s_latched_recover_cnt = 0u;
    }
}
#endif /* FP_LATCH_AUTO_RECOVER_MS > 0u */

/*=============================================================================
 * 1ms 集中判断
 *=============================================================================*/
void Foc_Protect_Task1ms(void)
{
    uint32_t i, idx;
    float    fast_max = 0.0f;
    float    slow_sum = 0.0f;
    float    slow_avg;

    /* ---- 启动屏蔽 ---- */
    if (!s_enabled) {
        if ((tickTimer_GetCount() - s_init_ms) >= FP_STARTUP_BLANK_MS) {
            s_enabled = 1u;
        } else {
            return;
        }
    }

    /* ---- 诊断：每 500ms 打一次状态 ---- */
    {
        static uint32_t s_dbg_state_ms = 0;
        uint32_t now = tickTimer_GetCount();
        if ((now - s_dbg_state_ms) >= 500u) {
            s_dbg_state_ms = now;
            MAIN_D("[FP][STATE-DBG] state=%d fault=0x%08lX trip_cnt=%lu "
                   "fast_max_x100=%ld slow_avg_x100=%ld en=%u",
                   (int)g_fp_state,
                   (unsigned long)g_fp_fault,
                   (unsigned long)g_fp_trip_cnt,
                   (long)(g_fp_fast_max * 100.0f),
                   (long)(g_fp_slow_avg * 100.0f),
                   (unsigned)s_enabled);
        }
    }

    /* ---- LATCHED：可选自动恢复，否则只等手动复位 ---- */
    if (g_fp_state == FP_STATE_LATCHED) {
#if (FP_LATCH_AUTO_RECOVER_MS > 0u)
        Foc_Protect_LatchedRecoverTask();
#endif
        return;
    }

    /* ---- 1. 快层：最近 FP_FAST_LEN 点峰值 ---- */
    for (i = 0u; i < FP_FAST_LEN; i++) {
        idx = (s_fast_head - 1u - i) & (FP_FAST_LEN - 1u);
        if (s_fast_buf[idx] > fast_max) {
            fast_max = s_fast_buf[idx];
        }
    }
    g_fp_fast_max = fast_max;

    if (g_fp_state == FP_STATE_NORMAL) {
        if (fast_max > s_cfg.i_fast_trip_a) {
            Foc_Protect_Trip(FOC_FAULT_OC_FAST);
            return;
        }
    }

    /* ---- 2. 慢层：最近 FP_SLOW_LEN 点平均 ---- */
    for (i = 0u; i < FP_SLOW_LEN; i++) {
        idx = (s_slow_head - 1u - i) & (FP_SLOW_LEN - 1u);
        slow_sum += s_slow_buf[idx];
    }
    slow_avg = slow_sum / (float)FP_SLOW_LEN;
    g_fp_slow_avg = slow_avg;

    if (g_fp_state == FP_STATE_NORMAL) {
        if (slow_avg > s_cfg.i_slow_trip_a) {
            if (++s_slow_confirm_cnt >= s_cfg.slow_confirm_ms) {
                s_slow_confirm_cnt = 0u;
                Foc_Protect_Trip(FOC_FAULT_OC_SLOW);
                return;
            }
        } else {
            s_slow_confirm_cnt = 0u;
        }
    }

    /* ---- 3. 恢复任务 ---- */
    Foc_Protect_RecoverTask();
}

/*=============================================================================
 * 手动复位
 *=============================================================================*/
void Foc_Protect_ClearFault(void)
{
    s_slow_confirm_cnt = 0u;
    s_recover_cnt      = 0u;
#if (FP_LATCH_AUTO_RECOVER_MS > 0u)
    s_latched_recover_cnt = 0u;
#endif

    g_fp_fault    = FOC_FAULT_NONE;
    g_fp_trip_cnt = 0u;
    g_fp_state    = FP_STATE_NORMAL;

    MAIN_D("[FP] manual clear -> NORMAL");
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
