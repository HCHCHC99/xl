/**
 *******************************************************************************
 * @file  foc_run.c
 * @brief FOC 运行模式层实现（IDLE / LOCK / IF / CURRENT / SPEED / POSITION）
 *
 *        本文件只负责「进入后保持」的持续运行模式。
 *        一次性流程（编码器对齐/扫角/测圈/斜坡对齐/零偏校准）在 foc_proc.c，
 *        通过 Foc_Proc_Start/Abort 编排，不再写入本层的 s_run_mode。
 *
 *        切模式时先 Foc_Proc_Abort()：避免残留的 g_align_active 让 20kHz ISR
 *        一直跳过 Foc_Run_Step20k（原 g_align_active 竞态，见移植过程 10.9）。
 *******************************************************************************
 */

#include "foc_run.h"
#include "../proc/foc_proc.h"

/* 欠压门禁（任务清单 §10.5-C / 业务.md §13.6-A）*/
#include "../../../service/sys_voltage.h"

#include "foc_current_loop.h"
#include "foc_speed_loop.h"
#include "foc_position_loop.h"
#include "foc_output.h"
#include "encoder_calib.h"     /* g_motor_is_aligned：对齐提示只在未对齐时打印 */

#include <rtthread.h>

#include <stddef.h>

/* 20kHz ISR 热路径：Debug 构建是 -O0，闭环时单拍容易超时。只对本文件开 O2。 */
#pragma GCC optimize("O2")

#define FR_TWO_PI          6.28318530f
#define FOC_LOCK_DUTY_PCT  (2.0f)

/*=============================================================================
 * 内部状态
 *=============================================================================*/
static volatile foc_run_mode_t s_run_mode   = FOC_RUN_IDLE;
static volatile float          s_theta      = 0.0f;
static volatile float          s_freq       = 0.0f;
static volatile float          s_vq         = 0.0f;
static volatile float          s_vd         = 0.0f;
static volatile float          s_lock_duty  = FOC_LOCK_DUTY_PCT;

/* mode 4 的 id/iq 指令缓存：支持运行中单独修改其中一个 */
static volatile float          s_cl_id_cmd  = 0.0f;
static volatile float          s_cl_iq_cmd  = 0.5f;

volatile foc_run_mode_t g_fm_mode     = FOC_RUN_IDLE;
volatile float          g_fm_theta_e  = 0.0f;
volatile float          g_fm_freq_hz  = 0.0f;
volatile float          g_fm_vq       = 0.0f;
volatile float          g_fm_vd       = 0.0f;
volatile uint32_t       g_fm_step_cnt = 0;

const char *Foc_Run_Name(foc_run_mode_t m)
{
    switch (m) {
    case FOC_RUN_IDLE:     return "IDLE, all low";
    case FOC_RUN_LOCK:     return "LOCK, zero vector";
    case FOC_RUN_IF:       return "IF, open-loop drag";
    case FOC_RUN_CURRENT:  return "CURRENT, closed loop";
    case FOC_RUN_SPEED:    return "SPEED, speed loop";
    case FOC_RUN_POSITION: return "POSITION, position loop";
    default:               return "UNKNOWN";
    }
}

/* 兼容期唯一的数字映射：CLI 命令编号与内部枚举不再绑定。
 * 日志与 fstat 里的 mode=N 必须走这里，禁止直接 (int)Foc_Run_GetMode()。 */
int Foc_Run_LegacyId(foc_run_mode_t m)
{
    switch (m) {
    case FOC_RUN_IDLE:     return 0;
    case FOC_RUN_LOCK:     return 1;
    case FOC_RUN_IF:       return 2;
    case FOC_RUN_CURRENT:  return 4;
    case FOC_RUN_SPEED:    return 9;
    case FOC_RUN_POSITION: return 10;
    default:               return -1;
    }
}

/*=============================================================================
 * 初始化
 *=============================================================================*/
void Foc_Run_Init(void)
{
    s_run_mode = FOC_RUN_IDLE;
    s_theta    = 0.0f;
    s_freq     = 0.0f;
    s_vq       = 0.0f;
    s_vd       = 0.0f;
    g_fm_mode  = FOC_RUN_IDLE;

    Foc_Output_Init();

    rt_kprintf("[FOC-MODE] power-on default: mode 0 (IDLE, all low)\r\n");
}

foc_run_mode_t Foc_Run_GetMode(void)
{
    return s_run_mode;
}

/* mode 4 电流指令缓存访问：供 foc_cli.c 使用，避免 CLI 直接碰静态变量 */
float Foc_Run_GetCurrentCmdId(void)
{
    return s_cl_id_cmd;
}

float Foc_Run_GetCurrentCmdIq(void)
{
    return s_cl_iq_cmd;
}

void Foc_Run_SetCurrentCmd(float id, float iq)
{
    s_cl_id_cmd = id;
    s_cl_iq_cmd = iq;
    Foc_CurrentLoop_SetRef(id, iq);
}

void Foc_Run_ReapplyCurrentCmd(void)
{
    Foc_CurrentLoop_SetRef(s_cl_id_cmd, s_cl_iq_cmd);
}

/*=============================================================================
 * 模式切换
 *   只接受 6 个运行模式；一次性流程请调用 Foc_Proc_Start()。
 *=============================================================================*/
void Foc_Run_SetMode(foc_run_mode_t mode)
{
    /* 【§10.5-C 欠压门禁】欠压期间拒绝转动类模式 IF/CURRENT/SPEED/POSITION；
     * 放行 IDLE(LOCK 也属静态输出)。保持 void 签名：打印一行后直接拒绝，
     * 业务层与 msh 都会被同一入口拦住，不需要调用者各自判断。 */
    if ((sys_voltage_is_uv() != 0U) &&
        ((mode == FOC_RUN_IF) || (mode == FOC_RUN_CURRENT) ||
         (mode == FOC_RUN_SPEED) || (mode == FOC_RUN_POSITION))) {
        rt_kprintf("[FOC-RUN] refuse mode %d: undervoltage\n",
                   Foc_Run_LegacyId(mode));
        return;
    }

    /* 任何模式切换先结束上一次一次性流程。
     * 等价于原 Foc_Mode_SetMode() 开头的 EncoderCalib_Abort()：
     * 否则 g_align_active 若残留为 1，20kHz ISR 会一直跳过 Foc_Run_Step20k，
     * 表现为 IF/CURRENT/SPEED/POSITION 都不刷新占空比（只有 LOCK 能写）。 */
    Foc_Proc_Abort();

    /* ---- 离开旧模式的收尾 ---- */
    if (mode != FOC_RUN_SPEED && mode != FOC_RUN_POSITION) {
        Foc_Speed_Stop();
    }
    if (mode != FOC_RUN_POSITION) {
        Foc_Position_Stop();
    }
    if (mode != FOC_RUN_IF) {
        s_theta = 0.0f;
    }

    /* ---- 进入新模式 ----
     * 本层不再直接写桥臂：输出由 Foc_Run_Step20k() 每拍下发 kind，
     * 执行器（foc_control.c）按"桥臂当前状态"去重后落寄存器，
     * 最大延迟一个 PWM 周期（50us）。过流 trip 的立即停机仍在 foc_protect.c。 */
    switch (mode) {
    case FOC_RUN_IDLE:
    case FOC_RUN_LOCK:
    case FOC_RUN_IF:
        break;

    case FOC_RUN_CURRENT:
        Foc_CurrentLoop_Reset();
        break;

    case FOC_RUN_SPEED:
        Foc_CurrentLoop_Reset();
        Foc_Speed_ConfigCurrentLoop();
        Foc_Speed_Start();
        break;

    case FOC_RUN_POSITION:
        Foc_CurrentLoop_Reset();
        Foc_Speed_ConfigCurrentLoop();
        Foc_Speed_Start();
        Foc_Position_Reset();
        break;

    default:
        rt_kprintf("[FOC-RUN] unsupported run mode %d\r\n", (int)mode);
        return;
    }

    s_run_mode = mode;

    rt_kprintf("[FOC-MODE] -> mode %d (%s)\r\n",
               Foc_Run_LegacyId(mode), Foc_Run_Name(mode));

    if ((mode != FOC_RUN_IDLE) && (mode != FOC_RUN_LOCK) && (mode != FOC_RUN_IF) &&
        (g_motor_is_aligned == 0U)) {
        rt_kprintf("[FOC-MODE] hint: run 'falign' (or 'focmode 3') first if not aligned\r\n");
    }
}

void Foc_Run_SetOpenloop(float freq_hz, float vq_volts, float vd_volts)
{
    s_freq = freq_hz;
    s_vq   = vq_volts;
    s_vd   = vd_volts;
}

void Foc_Run_StartOpenloop(void)
{
    /* 与参考工程 Enter_IF_Mode() 一致：
     * 先复位角度，再走完整模式切换（解除 EMB / 恢复 PWM / 清理流程与旧环） */
    Foc_Run_SetMode(FOC_RUN_IF);
}

void Foc_Run_Stop(void)
{
    Foc_Run_SetMode(FOC_RUN_IDLE);
}

/*=============================================================================
 * 每拍
 *=============================================================================*/
void Foc_Run_Step20k(float ts, FocCmd_t *out)
{
    if (out == NULL) return;

    switch (s_run_mode) {
    case FOC_RUN_IF:
        s_theta += FR_TWO_PI * s_freq * ts;
        if (s_theta >= FR_TWO_PI) s_theta -= FR_TWO_PI;
        if (s_theta < 0.0f)       s_theta += FR_TWO_PI;

        out->kind    = FOC_OUT_VECTOR;
        out->vd      = s_vd;
        out->vq      = s_vq;
        out->theta_e = s_theta;
        break;

    case FOC_RUN_CURRENT:
        Foc_CurrentLoop_Step(&out->vd, &out->vq, &out->theta_e);
        out->kind = FOC_OUT_VECTOR;
        break;

    case FOC_RUN_SPEED:
        Foc_Speed_Task20k();
        Foc_CurrentLoop_Step(&out->vd, &out->vq, &out->theta_e);
        out->kind = FOC_OUT_VECTOR;
        break;

    case FOC_RUN_POSITION:
        Foc_Position_Task20k();
        Foc_Speed_Task20k();
        Foc_CurrentLoop_Step(&out->vd, &out->vq, &out->theta_e);
        out->kind = FOC_OUT_VECTOR;
        break;

    case FOC_RUN_LOCK:
        out->kind   = FOC_OUT_DUTY3;
        out->duty_u = s_lock_duty;
        out->duty_v = s_lock_duty;
        out->duty_w = s_lock_duty;
        out->vd = out->vq = 0.0f;
        out->theta_e = 0.0f;
        break;

    case FOC_RUN_IDLE:
    default:
        out->kind = FOC_OUT_ALL_LOW;
        out->vd = out->vq = 0.0f;
        out->theta_e = 0.0f;
        break;
    }

    g_fm_mode     = s_run_mode;
    g_fm_theta_e  = out->theta_e;
    g_fm_freq_hz  = s_freq;
    g_fm_vq       = out->vq;
    g_fm_vd       = out->vd;
    g_fm_step_cnt++;
}

/*=============================================================================
 * LOCK 占空比 / PWM 归属
 *=============================================================================*/
void Foc_Run_SetLockDuty(float duty_pct)
{
    if (duty_pct < 0.0f)   duty_pct = 0.0f;
    if (duty_pct > 100.0f) duty_pct = 100.0f;

    s_lock_duty = duty_pct;

    /* 立即写 duty3 的动作已移交给执行器（每拍按 kind=DUTY3 下发），
     * 这里只保留确认打印，避免线程侧与 ISR 同时写桥臂。 */
    if (s_run_mode == FOC_RUN_LOCK) {
        int d10 = (int)(duty_pct * 10.0f + 0.5f);

        rt_kprintf("[FOC-MODE] LOCK duty -> %d.%d%%\r\n", d10 / 10, d10 % 10);
    }
}

float Foc_Run_GetLockDuty(void)
{
    return s_lock_duty;
}

int Foc_Run_PwmOwned(void)
{
#if FOC_DISABLE_LEGACY_6STEP
    return 1;
#else
    return (s_run_mode != FOC_RUN_IDLE) ? 1 : 0;
#endif
}