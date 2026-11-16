/**
 * @file    foc_mode.c
 * @brief   FOC 模式层实现（IDLE / LOCK）
 * @note    IDLE：六路全低 + EMB 封锁。
 *          LOCK：解除 EMB，恢复 PWM，输出 2% 零矢量。
 *          2% 取自参考工程 foc_control.c 的静态输出分支
 *          （hw_tmr4_pwm_set_duty3(2.0f, 2.0f, 2.0f)）。
 */

#include "foc_mode.h"
#include "cfg_foc_table.h"   /* 配置表宏：LOCK 诊断打印用 */
#include "hw_tmr4_pwm.h"
#include "../../bsp/bsp_pwm.h"

#include <rtthread.h>
#include <finsh.h>

#define FOC_LOCK_DUTY_PCT   (2.0f)

static volatile foc_mode_t s_mode = FOC_MODE_IDLE;
static volatile float      s_lock_duty = FOC_LOCK_DUTY_PCT;

void Foc_Mode_Init(void)
{
    s_mode = FOC_MODE_IDLE;

    /* 上电默认 mode 0：六路全低 + EMB 封锁。
     * 此时 TMR4 计数与 20kHz 时基仍在运行，只是 OC 关闭。 */
    (void)hw_tmr4_pwm_set_all_low();
    bsp_pwm_emergency_brake();

    rt_kprintf("[FOC-MODE] power-on default: mode 0 (IDLE, all low)\r\n");
}

void Foc_Mode_SetMode(foc_mode_t mode)
{
    tmr4_pwm_err_t err;

    switch (mode) {
    case FOC_MODE_IDLE:
        err = hw_tmr4_pwm_set_all_low();
        if (err != TMR4_PWM_OK) {
            rt_kprintf("[FOC-MODE] mode 0 failed: %d\r\n", (int)err);
            return;
        }
        bsp_pwm_emergency_brake();
        break;

    case FOC_MODE_LOCK:
        /* 先解除 EMB，再恢复 PWM 输出，最后给 2% 零矢量 */
        bsp_pwm_emergency_release();

        err = hw_tmr4_pwm_resume_output();
        if (err != TMR4_PWM_OK) {
            bsp_pwm_emergency_brake();
            rt_kprintf("[FOC-MODE] mode 1 resume failed: %d\r\n", (int)err);
            return;
        }

        err = hw_tmr4_pwm_set_duty3(s_lock_duty, s_lock_duty, s_lock_duty);
        if (err != TMR4_PWM_OK) {
            (void)hw_tmr4_pwm_set_all_low();
            bsp_pwm_emergency_brake();
            rt_kprintf("[FOC-MODE] mode 1 duty failed: %d\r\n", (int)err);
            return;
        }
        break;

    default:
        rt_kprintf("[FOC-MODE] unsupported mode %d (only 0/1)\r\n", (int)mode);
        return;
    }

    s_mode = mode;
    {
        int d10 = (int)(s_lock_duty * 10.0f + 0.5f);

        rt_kprintf("[FOC-MODE] -> mode %d (%s), duty=%d.%d%%\r\n", (int)mode,
                   (mode == FOC_MODE_IDLE) ? "IDLE, all low"
                                           : "LOCK, zero vector",
                   d10 / 10, d10 % 10);
    }
}

foc_mode_t Foc_Mode_GetMode(void)
{
    return s_mode;
}

void Foc_Mode_SetLockDuty(float duty_pct)
{
    if (duty_pct < 0.0f)   duty_pct = 0.0f;
    if (duty_pct > 100.0f) duty_pct = 100.0f;

    s_lock_duty = duty_pct;

    /* 已在 LOCK 时立即生效，不必重新切模式 */
    if (s_mode == FOC_MODE_LOCK) {
        int d10 = (int)(duty_pct * 10.0f + 0.5f);

        (void)hw_tmr4_pwm_set_duty3(duty_pct, duty_pct, duty_pct);
        rt_kprintf("[FOC-MODE] LOCK duty -> %d.%d%%\r\n", d10 / 10, d10 % 10);
    }
}

float Foc_Mode_GetLockDuty(void)
{
    return s_lock_duty;
}

int Foc_Mode_PwmOwned(void)
{
#if FOC_DISABLE_LEGACY_6STEP
    return 1;
#else
    return (s_mode != FOC_MODE_IDLE) ? 1 : 0;
#endif
}

/* 百分比转 x100 整数，避免依赖 printf 浮点格式 */
static int foc_mode_pct_x100(float pct)
{
    return (int)(pct * 100.0f + ((pct >= 0.0f) ? 0.5f : -0.5f));
}

/*=============================================================================
 * LOCK 模式 PWM 诊断打印
 *   - 预期占空比：命令值（UVW 相同）
 *   - 实际占空比：驱动按死区/钳位换算后的引脚高/低侧占比
 *   - 采样触发点：EVT 通道 UL 的 SCCR/SCSR 实际寄存器值
 *=============================================================================*/
static void foc_mode_dump_pwm(float expect_pct)
{
    static const char phase_name[3] = { 'U', 'V', 'W' };
    tmr4_pwm_state_t        st;
    tmr4_adc_trigger_info_t trig;
    int e = foc_mode_pct_x100(expect_pct);
    int i;

    /* RT-Thread 单次 rt_kprintf 只有 128 B 缓冲（RT_CONSOLEBUF_SIZE），
     * 超过 126 字符会被截断并写入 NUL，导致后续显示异常。
     * 配置 5 的单行描述约 127 字符，正好超限，所以拆成两行。 */
    rt_kprintf("[FOC-MODE] cfg[%d] %s\r\n",
               (int)FOC_CFG_SELECT, FOC_CFG_NAME);

    rt_kprintf("[FOC-MODE] freq=%lu Hz, output=%s, sample=%s, "
               "deadtime=%u ns, active=%s\r\n",
               (unsigned long)FOC_CFG_PWM_FREQ,
               (FOC_CFG_OUTPUT_TYPE == TMR4_OUTPUT_COMPLEMENTARY)
                   ? "COMPLEMENTARY" : "SYNC",
               (FOC_CFG_SAMPLE_POINT == CFG_SAMPLE_UPPER)
                   ? "UPPER(peak)" : "LOWER(valley)",
               (unsigned)FOC_CFG_DEADTIME_NS,
               (FOC_CFG_ACTIVE_HIGH) ? "HIGH" : "LOW");

    rt_kprintf("[FOC-MODE] expect U/V/W = %d.%02d / %d.%02d / %d.%02d %%\r\n",
               e / 100, e % 100, e / 100, e % 100, e / 100, e % 100);
    rt_thread_mdelay(10);

    for (i = 0; i < 3; i++) {
        int c, h, l, d;

        hw_tmr4_pwm_get_state((tmr4_pwm_channel_t)i, &st);

        c = foc_mode_pct_x100(st.centerPct);
        h = foc_mode_pct_x100(st.highPct);
        l = foc_mode_pct_x100(st.lowPct);
        d = foc_mode_pct_x100(st.deadPct);

        /* 0%/100% 边界处死区会让模型值略为负数，引脚实际为 0 */
        if (h < 0) { h = 0; }
        if (l < 0) { l = 0; }
        if (d < 0) { d = 0; }

        rt_kprintf("[FOC-MODE] actual  %c: center=%d.%02d high=%d.%02d low=%d.%02d"
                   " dead=%d.%02d %% (cmpH=%u cmpL=%u)\r\n",
                   phase_name[i],
                   c / 100, c % 100, h / 100, h % 100,
                   l / 100, l % 100, d / 100, d % 100,
                   (unsigned)hw_tmr4_pwm_get_occr((tmr4_pwm_channel_t)i, true),
                   (unsigned)hw_tmr4_pwm_get_occr((tmr4_pwm_channel_t)i, false));
    }

    /* 寄存器回读：确认 OCCR 与死区 PDAR/PDBR 的真实值。
     * 影子装载使能时，OCCR 要等一个装载点后才更新，前面已有 10ms 等待。 */
    for (i = 0; i < 3; i++) {
        tmr4_pwm_regs_t regs;

        hw_tmr4_pwm_get_regs((tmr4_pwm_channel_t)i, &regs);

        if (FOC_CFG_OUTPUT_TYPE == TMR4_OUTPUT_COMPLEMENTARY) {
            rt_kprintf("[FOC-MODE] REG %c: OCCRH=%u OCCRL=%u PDAR=%u PDBR=%u "
                       "OCER=0x%04X OCSR=0x%04X\r\n",
                       phase_name[i],
                       (unsigned)regs.occrH, (unsigned)regs.occrL,
                       (unsigned)regs.pdar,  (unsigned)regs.pdbr,
                       (unsigned)regs.ocer,  (unsigned)regs.ocsr);
        } else {
            rt_kprintf("[FOC-MODE] REG %c: OCCRH=%u OCCRL=%u "
                       "(SYNC: PDAR/PDBR reg=%u/%u) OCER=0x%04X OCSR=0x%04X\r\n",
                       phase_name[i],
                       (unsigned)regs.occrH, (unsigned)regs.occrL,
                       (unsigned)regs.pdar,  (unsigned)regs.pdbr,
                       (unsigned)regs.ocer,  (unsigned)regs.ocsr);
        }
    }

    /* 互补/同相时序说明：CNT 为 TMR4 三角波计数器（0..period-1） */
    if (FOC_CFG_OUTPUT_TYPE == TMR4_OUTPUT_COMPLEMENTARY) {
        rt_kprintf("[FOC-MODE] COMP: LS ON around valley (CNT 0..cmpL), "
                   "HS ON around peak (CNT cmpL..%u)\r\n",
                   (unsigned)(hw_tmr4_pwm_get_period_ticks() - 1U));
        rt_kprintf("[FOC-MODE] COMP: both up/down slopes, "
                   "each edge loses %u dead ticks\r\n",
                   (unsigned)hw_tmr4_pwm_get_dead_ticks());
    } else {
        rt_kprintf("[FOC-MODE] SYNC timing: HS/LS ON together around peak "
                   "(CNT cmp..%u, both slopes), dead offset=%u ticks\r\n",
                   (unsigned)(hw_tmr4_pwm_get_period_ticks() - 1U),
                   (unsigned)hw_tmr4_pwm_get_dead_ticks());
    }
    rt_thread_mdelay(10);

    hw_tmr4_pwm_get_adc_trigger_info(&trig);
    {
        const char *match = "?";
        uint32_t us_x100 = 0U;

        if (trig.timerHz != 0U) {
            /* tick -> us：compareValue * 1e6 / timerHz，再放大 100 倍保留小数 */
            us_x100 = (uint32_t)(((uint64_t)trig.compareValue * 100000000ULL) / trig.timerHz);
        }
        if ((trig.matchCond & TMR4_EVT_MATCH_CNT_VALLEY) != 0U) {
            match = "VALLEY";
        } else if ((trig.matchCond & TMR4_EVT_MATCH_CNT_PEAK) != 0U) {
            match = "PEAK";
        }

        rt_kprintf("[FOC-MODE] ADC trig: %s SCCR=%u (%u.%02u us after valley), "
                   "match=%s EVT0/SCM0, period=%u, CNT_now=%u\r\n",
                   trig.highSide ? "UPPER" : "LOWER",
                   (unsigned)trig.compareValue,
                   (unsigned)(us_x100 / 100U), (unsigned)(us_x100 % 100U),
                   match,
                   (unsigned)trig.periodTicks,
                   (unsigned)trig.cntNow);

        /* EOCA 中断入口时的 CNT，用来估算 ADC 转换完成相对触发点的延迟。
         * UPPER：触发在峰值，之后 CNT 下数，delta = SCCR - CNT；
         * LOWER：触发在谷底 0，之后 CNT 上数，delta = CNT。 */
        if (trig.timerHz != 0U) {
            uint16_t eoca_cnt = foc_eoca_probe_get_cnt();
            uint32_t delta_ticks;

            if (trig.highSide) {
                delta_ticks = (eoca_cnt <= trig.compareValue)
                            ? (uint32_t)(trig.compareValue - eoca_cnt) : 0U;
            } else {
                delta_ticks = (uint32_t)eoca_cnt;
            }

            {
                uint32_t eoca_us_x100 =
                    (uint32_t)(((uint64_t)delta_ticks * 100000000ULL) / trig.timerHz);

                rt_kprintf("[FOC-MODE] EOCA: CNT=%u, %u.%02u us after trigger\r\n",
                           (unsigned)eoca_cnt,
                           (unsigned)(eoca_us_x100 / 100U),
                           (unsigned)(eoca_us_x100 % 100U));
            }
        }
    }
    rt_thread_mdelay(10);
    rt_kprintf("[FOC-MODE] dump end\r\n");
}

/* 解析 "52.6" 这类无符号百分数，不依赖 atof */
static int foc_mode_parse_duty(const char *s, float *out)
{
    float v    = 0.0f;
    float frac = 0.1f;
    int   any  = 0;

    if ((s == RT_NULL) || (out == RT_NULL)) return -1;

    while ((*s >= '0') && (*s <= '9')) {
        v = v * 10.0f + (float)(*s - '0');
        s++;
        any = 1;
    }

    if (*s == '.') {
        s++;
        while ((*s >= '0') && (*s <= '9')) {
            v += (float)(*s - '0') * frac;
            frac *= 0.1f;
            s++;
            any = 1;
        }
    }

    if ((*s != '\0') || (any == 0)) return -1;
    if ((v < 0.0f) || (v > 100.0f)) return -1;

    *out = v;
    return 0;
}

/*=============================================================================
 * msh 命令
 *   focmode        打印当前模式
 *   focmode 0      IDLE：六路全低 + EMB 封锁
 *   focmode 1      LOCK：解除 EMB，输出默认 2% 零矢量
 *   focmode 1 52.6 LOCK：三路上桥 52.6% 占空比
 *=============================================================================*/
static int foc_mode_msh(int argc, char **argv)
{
    if (argc == 1) {
        int d10 = (int)(Foc_Mode_GetLockDuty() * 10.0f + 0.5f);

        rt_kprintf("[FOC-MODE] current mode = %d (%s), lock duty = %d.%d%%\r\n",
                   (int)Foc_Mode_GetMode(),
                   (Foc_Mode_GetMode() == FOC_MODE_IDLE) ? "IDLE, all low"
                                                         : "LOCK, zero vector",
                   d10 / 10, d10 % 10);
        rt_kprintf("usage: focmode <0|1> [duty%%]\r\n");
        return 0;
    }

    if ((argv[1][0] == '0') && (argv[1][1] == '\0')) {
        Foc_Mode_SetMode(FOC_MODE_IDLE);
        return 0;
    }

    if ((argv[1][0] == '1') && (argv[1][1] == '\0')) {
        float duty = FOC_LOCK_DUTY_PCT;      /* 不带参数默认 2% */

        if (argc >= 3) {
            if (foc_mode_parse_duty(argv[2], &duty) != 0) {
                rt_kprintf("invalid duty: %s (expect 0..100)\r\n", argv[2]);
                return -1;
            }
        }

        Foc_Mode_SetLockDuty(duty);
        Foc_Mode_SetMode(FOC_MODE_LOCK);

        /* 仅在真正进入 LOCK 后打印，避免 resume/duty 失败时给出误导信息 */
        if (Foc_Mode_GetMode() == FOC_MODE_LOCK) {
            foc_mode_dump_pwm(duty);
        }
        return 0;
    }

    rt_kprintf("usage: focmode <0|1> [duty%%]\r\n"
               "  focmode 0        IDLE, all low\r\n"
               "  focmode 1        LOCK, default 2%%\r\n"
               "  focmode 1 52.6   LOCK, all three high-side = 52.6%%\r\n");
    return -1;
}

MSH_CMD_EXPORT_ALIAS(foc_mode_msh, focmode, set FOC mode 0=IDLE 1=LOCK);
