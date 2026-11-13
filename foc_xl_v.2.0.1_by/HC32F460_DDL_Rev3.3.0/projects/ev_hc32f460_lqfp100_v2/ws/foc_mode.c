/*=============================================================================
 * foc_mode.c  —  L6 模式层实现（多模式）
 *
 * 模式语义：
 *   IDLE      : 全低，安全停机
 *   LOCK      : 2% 零矢量，锁死
 *   IF        : q 轴加压 + 角度自增，开环拖动（enable=true）
 *   CALIB     : 输出全低，校准动作由主循环阻塞执行（enable=false）
 *   CURRENT   : 有感闭环电流环，PID 产出 vd/vq + 编码器角度（enable=true）
 *   SPEED     : 速度外环 + 电流环串级（enable=true）
 *   POSITION  : 位置外环（连续 P + 阻尼，2ms）-> 速度环 -> 电流环
 *   PARAM_IDENT : 离线参数辨识状态机（enable=true），结果只写 RAM
 *=============================================================================*/

#include "foc_mode.h"
#include "foc_current_loop.h"   /* FOC_MODE_CURRENT / FOC_MODE_SPEED 分支用 */
#include "foc_speed_loop.h"
#include "foc_position_loop.h"
#include "param_ident.h"
#include <stddef.h>

#define FM_TWO_PI  6.28318530f

/* 全部 volatile：主循环写，ISR 读，必须加，否则编译器可能缓存旧值 */
static volatile foc_mode_t s_mode  = FOC_MODE_IDLE;
static volatile float      s_theta = 0.0f;
static volatile float      s_freq  = 0.0f;
static volatile float      s_vq    = 0.0f;
static volatile float      s_vd    = 0.0f;

volatile foc_mode_t g_fm_mode     = FOC_MODE_IDLE;
volatile float      g_fm_theta_e  = 0.0f;
volatile float      g_fm_freq_hz  = 0.0f;
volatile float      g_fm_vq       = 0.0f;
volatile float      g_fm_vd       = 0.0f;
volatile uint32_t   g_fm_step_cnt = 0;

void Foc_Mode_Init(void)
{
    s_mode  = FOC_MODE_IDLE;
    s_theta = 0.0f;
    s_freq  = 0.0f;
    s_vq    = 0.0f;
    s_vd    = 0.0f;
    g_fm_mode = FOC_MODE_IDLE;
}

foc_mode_t Foc_Mode_GetMode(void) { return s_mode; }

void Foc_Mode_SetMode(foc_mode_t m)
{
    foc_mode_t old = s_mode;

    s_mode = m;

    /* 离开参数辨识模式：记录中断原因（故障保护或人为切模式） */
    if ((old == FOC_MODE_PARAM_IDENT) && (m != FOC_MODE_PARAM_IDENT)) {
        ParamIdent_Abort();
    }

    if (m != FOC_MODE_IF) {
        s_theta = 0.0f;   /* 离开 IF 时清角度 */
    }

    /* 离开速度模式时先清速度环状态与 iq_ref */
    if (m != FOC_MODE_SPEED) {
        Foc_Speed_Stop();
    }

    /* 进入闭环模式时复位 PID，防止从 IF 切来时的旧积分残留冲击 */
    if ((m == FOC_MODE_CURRENT) || (m == FOC_MODE_SPEED) ||
        (m == FOC_MODE_POSITION)) {
        Foc_CurrentLoop_Reset();
    }

    /* 位置模式：进入时以当前位置为目标保持；离开时清位置环输出 */
    if (m == FOC_MODE_POSITION) {
        Foc_Position_Reset();
    } else {
        Foc_Position_Stop();
    }
}

void Foc_Mode_SetOpenloop(float freq_hz, float vq_volts, float vd_volts)
{
    s_freq = freq_hz;
    s_vq   = vq_volts;
    s_vd   = vd_volts;
}

void Foc_Mode_StartOpenloop(void)
{
    s_theta = 0.0f;
    s_mode  = FOC_MODE_IF;
}

void Foc_Mode_Stop(void)
{
    s_mode = FOC_MODE_IDLE;
}

void Foc_Mode_Step(float ts, FocCmd_t *out)
{
    if (out == NULL) return;

    switch (s_mode) {
    case FOC_MODE_IF:
        /* 角度积分 */
        s_theta += FM_TWO_PI * s_freq * ts;
        if (s_theta >= FM_TWO_PI) s_theta -= FM_TWO_PI;
        if (s_theta < 0.0f)       s_theta += FM_TWO_PI;

        out->vd      = s_vd;
        out->vq      = s_vq;
        out->theta_e = s_theta;
        out->enable  = true;
        break;

    case FOC_MODE_CURRENT:
        /* 有感闭环：PID 产出 vd/vq，编码器角度 */
        Foc_CurrentLoop_Step(&out->vd, &out->vq, &out->theta_e);
        out->enable = true;
        break;

    case FOC_MODE_SPEED:
        /* 速度外环（5ms 抽稀）先算 iq_ref，再跑电流环 */
        Foc_Speed_Task20k();
        Foc_CurrentLoop_Step(&out->vd, &out->vq, &out->theta_e);
        out->enable = true;
        break;

    case FOC_MODE_POSITION:
        /* 位置环（2ms）-> 速度环（1ms）-> 电流环，全程闭环无刹车段 */
        Foc_Position_Task20k();
        Foc_Speed_Task20k();
        Foc_CurrentLoop_Step(&out->vd, &out->vq, &out->theta_e);
        out->enable = true;
        break;

    case FOC_MODE_PARAM_IDENT:
        /* 离线参数辨识：模块自带 d/q PI，结束时会自行切回 IDLE */
        ParamIdent_Step20kHz(&out->vd, &out->vq, &out->theta_e, &out->enable);
        break;

    case FOC_MODE_CALIB:  /* 校准模式：输出全低，动作由主循环接管 */
    case FOC_MODE_LOCK:
    case FOC_MODE_IDLE:
    default:
        out->vd = out->vq = 0.0f;
        out->theta_e = 0.0f;
        out->enable  = false;
        break;
    }

    g_fm_mode     = s_mode;
    g_fm_theta_e  = out->theta_e;
    g_fm_freq_hz  = s_freq;
    g_fm_vq       = out->vq;
    g_fm_vd       = out->vd;
    g_fm_step_cnt++;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
