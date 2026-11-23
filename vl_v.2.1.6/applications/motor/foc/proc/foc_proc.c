/**
 *******************************************************************************
 * @file  foc_proc.c
 * @brief FOC 一次性流程层实现
 *
 *        4 个标定流程（3/5/6/7）只做编排，状态机仍在 encoder_calib.c；
 *        2 个零偏校准（12/13）是阻塞流程，流程内自己完成采样。
 *
 *        与原 foc_mode.c 的差异：
 *          - 流程不再写入运行模式，运行模式始终由 foc_run 层维护；
 *          - 12/13 原先把 s_mode 强制改回 IDLE 的写法随之取消，
 *            由 Foc_Proc_Start() 统一"先收回到 IDLE"。
 *******************************************************************************
 */

#include "foc_proc.h"
#include "../core/foc_run.h"
#include "../encoder_calib.h"
#include "../foc_control.h"          /* g_align_active */

/* 欠压门禁（任务清单 §10.5-C / 业务.md §13.6-A）*/
#include "../../../service/sys_voltage.h"

#include "../core/foc_output.h"
#include "bsp_adc_foc.h"

#include <rtthread.h>

#include <stddef.h>

/*=============================================================================
 * 内部状态
 *=============================================================================*/
static volatile foc_proc_t s_active = FOC_PROC_NONE;
static volatile uint8_t    s_busy   = 0u;

const char *Foc_Proc_Name(foc_proc_t p)
{
    switch (p) {
    case FOC_PROC_NONE:         return "NONE";
    case FOC_PROC_ENC_ALIGN:    return "CALIB, encoder align";
    case FOC_PROC_ONE_POINT:    return "ALIGN1P, one-point alpha";
    case FOC_PROC_SWEEP:        return "SWEEP, angle sweep";
    case FOC_PROC_REV:          return "REV, one-rev count";
    case FOC_PROC_ALIGN7:       return "ALIGN7, ramp align";
    case FOC_PROC_OFFSET_NOPWM: return "OFFCAL, no PWM";
    case FOC_PROC_OFFSET_PWM50: return "OFFCAL, PWM 50%";
    default:                    return "UNKNOWN";
    }
}

int Foc_Proc_LegacyId(foc_proc_t p)
{
    switch (p) {
    case FOC_PROC_NONE:         return 0;
    case FOC_PROC_ENC_ALIGN:    return 3;
    case FOC_PROC_ONE_POINT:    return 3;      /* 同属编码器对齐，日志沿用 3 */
    case FOC_PROC_SWEEP:        return 5;
    case FOC_PROC_REV:          return 6;
    case FOC_PROC_ALIGN7:       return 7;
    case FOC_PROC_OFFSET_NOPWM: return 12;
    case FOC_PROC_OFFSET_PWM50: return 13;
    default:                    return -1;
    }
}

/*=============================================================================
 * 电流零偏校准（原 foc_mode.c 的 foc_offset_cal_apply）
 *   - 阻塞采集，20kHz ISR 被动累加，直到 2048 样本或超时
 *   - 结果只写 RAM（foc_adc_apply_offset_raw），不写 Flash
 *=============================================================================*/
static uint8_t foc_offset_cal_apply(uint32_t timeout_ms)
{
    uint16_t z[3];
    uint32_t t0;
    uint32_t n;

    foc_adc_cal_begin();
    t0 = rt_tick_get();

    while (foc_adc_cal_get_sample_count() < 2048U) {
        if ((rt_tick_get() - t0) >= rt_tick_from_millisecond(timeout_ms)) {
            break;
        }
        rt_thread_mdelay(1);
    }

    n = foc_adc_cal_get_sample_count();
    foc_adc_cal_stop();

    if (foc_adc_cal_get_result(z) == 0U) {
        rt_kprintf("[CAL] FAIL: too few samples (n=%lu, timeout=%lums)\r\n",
                   (unsigned long)n, (unsigned long)timeout_ms);
        return 0U;
    }

    foc_adc_apply_offset_raw(z);

    rt_kprintf("[CAL] OK: N=%lu bus=%u w=%u v=%u (iu=-iv-iw)\r\n",
               (unsigned long)n,
               (unsigned)z[0], (unsigned)z[1], (unsigned)z[2]);
    return 1U;
}

/* 旧 focmode 12：无 PWM 零偏校准 */
static void foc_proc_offset_nopwm(void)
{
    (void)Foc_Output_AllLow();
    Foc_Output_Brake();
    rt_kprintf("[CAL12] no-PWM offset calibration\r\n");
    rt_thread_mdelay(5);          /* 等 ISR 完成 IDLE 收尾 */
    (void)foc_offset_cal_apply(300U);
}

/* 旧 focmode 13：PWM 50% 零矢量下的零偏校准 */
static void foc_proc_offset_pwm50(void)
{
    (void)Foc_Output_AllLow();
    Foc_Output_Release();
    (void)Foc_Output_Resume();
    rt_thread_mdelay(2);          /* 等 ISR 处理完 IDLE 的一次性全低写入 */
    (void)Foc_Output_Duty3(50.0f, 50.0f, 50.0f);
    rt_kprintf("[CAL13] PWM 50%% zero-vector offset calibration\r\n");
    rt_thread_mdelay(5);          /* 输出稳定后再采集 */
    (void)foc_offset_cal_apply(300U);
    (void)Foc_Output_AllLow();
}

void Foc_Proc_PowerOnCal(void)
{
    rt_kprintf("[CAL] power-on zero offset cal (no PWM)...\r\n");

    /* 上电阶段本来就是 IDLE 全低；这里只做采样与零点更新 */
    (void)foc_offset_cal_apply(300U);
}

/*=============================================================================
 * 流程编排
 *=============================================================================*/
void Foc_Proc_Start(foc_proc_t proc)
{
    if (proc == FOC_PROC_NONE) {
        return;
    }

    /* 【§10.5-C 欠压门禁】欠压期间拒绝需要转矩的一次性流程 3/5/6/7
     * （编码器对齐 / 扫角 / 测圈 / 斜坡对齐）；12/13 为零偏校准，行为不变。 */
    if ((sys_voltage_is_uv() != 0U) &&
        ((proc == FOC_PROC_ENC_ALIGN) || (proc == FOC_PROC_ONE_POINT) ||
         (proc == FOC_PROC_SWEEP) ||
         (proc == FOC_PROC_REV) || (proc == FOC_PROC_ALIGN7))) {
        rt_kprintf("[FOC-PROC] refuse proc %d: undervoltage\n",
                   Foc_Proc_LegacyId(proc));
        return;
    }

    /* 一次性流程只在 IDLE 下启动：先把运行模式收回 IDLE，
     * 等价于原 Foc_Mode_SetMode() 里"停环 + 关输出"的前置处理。
     * 已在 IDLE 时不重复切换，保证 focmode 3/5/6/7 的日志与基线一致。 */
    if (Foc_Run_GetMode() != FOC_RUN_IDLE) {
        Foc_Run_SetMode(FOC_RUN_IDLE);
    }

    EncoderCalib_Abort();

    s_active = FOC_PROC_NONE;
    s_busy   = 0u;

    switch (proc) {
    case FOC_PROC_ENC_ALIGN:
        Foc_Output_Release();
        EncoderCalib_StartAlign();
        break;

    case FOC_PROC_ONE_POINT:
        Foc_Output_Release();
        EncoderCalib_StartOnePoint();
        break;

    case FOC_PROC_SWEEP:
        Foc_Output_Release();
        EncoderCalib_StartSweep();
        break;

    case FOC_PROC_REV:
        Foc_Output_Release();
        EncoderCalib_StartRevMeasure();
        break;

    case FOC_PROC_ALIGN7:
        Foc_Output_Release();
        EncoderCalib_StartAlign7();
        break;

    case FOC_PROC_OFFSET_NOPWM:
        /* 阻塞流程：进入即结束，运行模式保持 IDLE。
         * 原实现不打印 "-> mode 12"，这里保持一致。 */
        foc_proc_offset_nopwm();
        return;

    case FOC_PROC_OFFSET_PWM50:
        foc_proc_offset_pwm50();
        return;

    default:
        rt_kprintf("[FOC-PROC] unsupported proc %d\r\n", (int)proc);
        return;
    }

    s_active = proc;
    s_busy   = 1u;

    rt_kprintf("[FOC-MODE] -> mode %d (%s)\r\n",
               Foc_Proc_LegacyId(proc), Foc_Proc_Name(proc));
}

uint8_t Foc_Proc_Task20k(void)
{
    /* 与原 foc_control.c 的判据完全一致：标志与状态机同时成立才接管。
     * 启动瞬间"标志=1 但状态机尚未置忙"的窗口里不能接管，
     * 否则会与 Start*() 的写入竞争。 */
    if ((g_align_active == 0u) || (EncoderCalib_IsBusy() == 0u)) {
        return 0u;
    }

    EncoderCalib_Task20k();
    return 1u;
}

void Foc_Proc_Poll1ms(void)
{
    EncoderCalib_Task1ms();

    if ((s_busy != 0u) && (EncoderCalib_IsBusy() == 0u)) {
        s_busy   = 0u;
        s_active = FOC_PROC_NONE;

        /* 流程结束不需要在这里收口：标定期间 foc_control.c 的 ISR 会把输出状态
         * 置为 UNKNOWN，流程结束后第一拍就会按运行模式的 kind 重落静态输出，
         * 覆盖 ec_finish_zero_vector() 留下的 2%。 */
    }
}

void Foc_Proc_Abort(void)
{
    EncoderCalib_Abort();
    s_active = FOC_PROC_NONE;
    s_busy   = 0u;
}

bool Foc_Proc_IsBusy(void)
{
    return (EncoderCalib_IsBusy() != 0u) ? true : false;
}

foc_proc_t Foc_Proc_GetActive(void)
{
    return s_active;
}