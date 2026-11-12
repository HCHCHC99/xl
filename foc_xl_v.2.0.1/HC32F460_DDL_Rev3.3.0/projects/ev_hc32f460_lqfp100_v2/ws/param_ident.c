/*=============================================================================
 * param_ident.c  —  L9 离线参数辨识实现（Rs / Ld / Lq，mode 11）
 *
 * 数据流（20kHz ISR）：
 *   Foc_Mode_Step (L6)
 *     -> ParamIdent_Step20kHz()
 *          -> 自有两路 pid_ctrl（d/q），反馈取 g_fc_id / g_fc_iq
 *          -> 按状态机做电流闭环 / 电压直接注入
 *          -> 输出 vd/vq/theta_e，L5 走 InvPark -> SVPWM -> SetDuty
 *
 * 与参考工程的差异：
 *   - 参考 10kHz 开关、20kHz 控制，电压延迟固定 2 级；
 *     本工程 20kHz 开关、20kHz 控制，谷底采样/峰值影子装载，采用方案A：
 *     激励极性每 2 拍翻转，只在极性一致的第二拍取样。
 *   - 参考电压是 12V 归一化值；本工程 vd/vq 直接是 V，拟合时按母线换算。
 *=============================================================================*/

#include "param_ident.h"
#include "pid_ctrl.h"
#include "hw_encoder.h"
#include "cfg_motor.h"
#include "foc_control.h"
#include "foc_mode.h"
#include "foc_protect.h"
#include "rtt_log.h"

#include <math.h>
#include <stddef.h>

/*---------------------------------------------------------------------------
 * 可调参数实例（初值来自 .h 的宏）
 *---------------------------------------------------------------------------*/
volatile float    g_pi_cur_kp       = PI_CUR_KP_INIT;
volatile float    g_pi_cur_ki       = PI_CUR_KI_INIT;
volatile float    g_pi_cur_vmax_v   = PI_CUR_VMAX_INIT;
volatile float    g_pi_cur_iterm_v  = PI_CUR_ITERM_INIT;
volatile float    g_pi_vbus_v       = PI_VBUS_INIT;
volatile float    g_pi_ld_bias_a    = PI_LD_BIAS_CURRENT_INIT;
volatile float    g_pi_lq_bias_a    = PI_LQ_BIAS_CURRENT_INIT;
volatile float    g_pi_ld_excite_v  = PI_LD_EXCITE_V_INIT;
volatile float    g_pi_lq_excite_v  = PI_LQ_EXCITE_V_INIT;
volatile uint16_t g_pi_pol_hold     = PI_POL_HOLD_INIT;

/*---------------------------------------------------------------------------
 * 结果与观测量
 *---------------------------------------------------------------------------*/
volatile PARAM_IDENT_RESULT g_param_ident = {0};

volatile uint32_t g_pi_tick_cnt    = 0u;
volatile uint8_t  g_pi_rs_index    = 0u;
volatile uint8_t  g_pi_pol         = 0u;
volatile float    g_pi_id_ref      = 0.0f;
volatile float    g_pi_iq_ref      = 0.0f;
volatile float    g_pi_center_v    = 0.0f;
volatile float    g_pi_vlast_cmd   = 0.0f;
volatile float    g_pi_axis_current= 0.0f;
volatile float    g_pi_axis_slope  = 0.0f;
volatile float    g_pi_start_theta = 0.0f;
volatile float    g_pi_end_theta   = 0.0f;
volatile int32_t  g_pi_start_cnt   = 0;
volatile int32_t  g_pi_end_cnt     = 0;

/*---------------------------------------------------------------------------
 * 编译期常量
 *---------------------------------------------------------------------------*/
#define PI_CAL_MIN_DEN         1.0e-12f
#define PI_CAL_FLT_MAX         3.402823e38f
#define PI_POL_HOLD_MIN        2u
#define PI_POL_HOLD_MAX        10u

/*---------------------------------------------------------------------------
 * 内部运行数据
 *---------------------------------------------------------------------------*/
typedef struct {
    uint32_t state_count;
    uint8_t  rs_index;
    uint8_t  axis_polarity;
    uint16_t pol_cnt;
    bool     axis_prev_valid;
    bool     skip_next_sample;
    float    id_ref;
    float    iq_ref;
    float    id_sum;
    float    ud_sum;
    float    axis_center_sum;
    float    axis_center_cmd;
    float    axis_last_current;
    float    axis_vlast_cmd;
    float    axis_sum_x;
    float    axis_sum_y;
    float    axis_sum_xx;
    float    axis_sum_xy;
} PI_RUNTIME;

static PI_RUNTIME s_rt;

static struct pid_dev  s_pid_d;
static struct pid_priv s_priv_d;
static struct pid_dev  s_pid_q;
static struct pid_priv s_priv_q;

static bool  s_ready = false;
static float s_vd    = 0.0f;
static float s_vq    = 0.0f;

static const float s_rs_current[PI_RS_POINT_NUM] = {
    PI_RS_CURRENT_1,
    PI_RS_CURRENT_2,
    PI_RS_CURRENT_3
};

/*---------------------------------------------------------------------------
 * 数学工具
 *---------------------------------------------------------------------------*/
static bool pi_is_finite_positive(float v)
{
    return (v == v) && (v > 0.0f) && (v < PI_CAL_FLT_MAX);
}

static bool pi_is_finite(float v)
{
    return (v == v) && (v < PI_CAL_FLT_MAX) && (v > -PI_CAL_FLT_MAX);
}

/* 下发电压指令（V）-> 实际施加电压（V） */
static float pi_actual_voltage(float cmd)
{
    float vbus = g_pi_vbus_v;

    if (vbus < 1.0f) {
        vbus = CFG_SAMPLING_SIM_VBUS_V;
    }
    return cmd * (vbus / CFG_SAMPLING_SIM_VBUS_V);
}

/* 线性斜坡：每拍靠近 target，步长不超 step */
static bool pi_ramp(float *ref, float target, float step)
{
    if (*ref < (target - step)) {
        *ref += step;
        return false;
    }
    if (*ref > (target + step)) {
        *ref -= step;
        return false;
    }
    *ref = target;
    return true;
}

/* 被激励轴电流 */
static float pi_axis_current(bool axis_is_q)
{
    return axis_is_q ? g_fc_iq : g_fc_id;
}

/*---------------------------------------------------------------------------
 * 电流环参数下发
 *---------------------------------------------------------------------------*/
static void pi_apply_params(void)
{
    float kp   = g_pi_cur_kp;
    float ki   = g_pi_cur_ki;
    float vmax = g_pi_cur_vmax_v;
    float iterm= g_pi_cur_iterm_v;

    if (!s_ready) {
        return;
    }

    if (kp < 0.0f)    kp = 0.0f;
    if (ki < 0.0f)    ki = 0.0f;
    if (vmax < 1.0f)  vmax = PI_CUR_VMAX_INIT;
    if (iterm < 0.0f) iterm = 0.0f;

    (void)s_pid_d.ops->set_param(&s_priv_d, kp, ki, 0.0f);
    (void)s_pid_q.ops->set_param(&s_priv_q, kp, ki, 0.0f);

    (void)s_pid_d.ops->set_limit(&s_priv_d, -vmax, vmax, 0.0f);
    (void)s_pid_q.ops->set_limit(&s_priv_q, -vmax, vmax, 0.0f);

    (void)s_pid_d.ops->set_iterm_limit(&s_priv_d, iterm);
    (void)s_pid_q.ops->set_iterm_limit(&s_priv_q, iterm);

    (void)s_pid_d.ops->set_aw_mode(&s_priv_d, PID_AW_CLAMP);
    (void)s_pid_q.ops->set_aw_mode(&s_priv_q, PID_AW_CLAMP);

    (void)s_pid_d.ops->set_diff_filter(&s_priv_d, 1.0f);
    (void)s_pid_q.ops->set_diff_filter(&s_priv_q, 1.0f);

    (void)s_pid_d.ops->set_feedforward(&s_priv_d, 0.0f, 0.0f, false, false);
    (void)s_pid_q.ops->set_feedforward(&s_priv_q, 0.0f, 0.0f, false, false);
}

/*---------------------------------------------------------------------------
 * 输出：d/q 电流闭环
 *---------------------------------------------------------------------------*/
static void pi_apply_current(float id_ref, float iq_ref)
{
    float out_d;
    float out_q;

    out_d = s_pid_d.ops->calc(&s_priv_d, g_fc_id, id_ref, 0.0f, 0.0f, PI_DT_S);
    out_q = s_pid_q.ops->calc(&s_priv_q, g_fc_iq, iq_ref, 0.0f, 0.0f, PI_DT_S);

    s_vd = out_d;
    s_vq = out_q;

    g_pi_id_ref = id_ref;
    g_pi_iq_ref = iq_ref;
}

/*---------------------------------------------------------------------------
 * 输出：一个轴直接注入电压，另一个轴维持零电流闭环
 *---------------------------------------------------------------------------*/
static void pi_apply_voltage(bool axis_is_q, float axis_cmd)
{
    float out_d;
    float out_q;
    float vlim = g_pi_cur_vmax_v;

    if (vlim < 1.0f) {
        vlim = PI_CUR_VMAX_INIT;
    }
    if (axis_cmd > vlim)  axis_cmd = vlim;
    if (axis_cmd < -vlim) axis_cmd = -vlim;

    if (axis_is_q) {
        out_q = axis_cmd;
        out_d = s_pid_d.ops->calc(&s_priv_d, g_fc_id, 0.0f, 0.0f, 0.0f, PI_DT_S);
    } else {
        out_d = axis_cmd;
        out_q = s_pid_q.ops->calc(&s_priv_q, g_fc_iq, 0.0f, 0.0f, 0.0f, PI_DT_S);
    }

    s_vd = out_d;
    s_vq = out_q;

    g_pi_id_ref = 0.0f;
    g_pi_iq_ref = 0.0f;
}

/*---------------------------------------------------------------------------
 * 清零电感拟合累计量
 *---------------------------------------------------------------------------*/
static void pi_reset_axis_samples(float initial_current)
{
    s_rt.axis_polarity    = 0u;
    s_rt.pol_cnt          = 0u;
    s_rt.axis_prev_valid  = false;
    s_rt.skip_next_sample = true;
    s_rt.axis_last_current= initial_current;
    s_rt.axis_vlast_cmd   = 0.0f;
    s_rt.axis_sum_x       = 0.0f;
    s_rt.axis_sum_y       = 0.0f;
    s_rt.axis_sum_xx      = 0.0f;
    s_rt.axis_sum_xy      = 0.0f;

    g_param_ident.sample_count = 0u;
    g_pi_pol          = 0u;
    g_pi_vlast_cmd    = 0.0f;
    g_pi_axis_current = initial_current;
    g_pi_axis_slope   = 0.0f;
}

/*---------------------------------------------------------------------------
 * 双极性电压激励 + 最小二乘样本累计（方案A）
 *
 * 极性每 hold 拍翻转。跨极性切换的那个区间前后半段电压不同，不能用于
 * 拟合；用 skip_next_sample 跳过该区间，只在同一极性块内部的区间取样，
 * 此时配对区间内实际电压恒为 axis_vlast_cmd。
 *---------------------------------------------------------------------------*/
static bool pi_run_excitation(bool axis_is_q,
                              float center_cmd,
                              float excite_v,
                              uint16_t target_count)
{
    float axis_current;
    float slope;
    float mid;
    float model_v;
    float axis_cmd;
    uint16_t hold;
    bool is_block_start;

    axis_current = pi_axis_current(axis_is_q);
    g_pi_axis_current = axis_current;
    g_pi_vlast_cmd    = s_rt.axis_vlast_cmd;

    hold = g_pi_pol_hold;
    if (hold < PI_POL_HOLD_MIN) {
        hold = PI_POL_HOLD_MIN;
    }
    if (hold > PI_POL_HOLD_MAX) {
        hold = PI_POL_HOLD_MAX;
    }

    if (s_rt.axis_prev_valid && !s_rt.skip_next_sample) {
        slope = (axis_current - s_rt.axis_last_current) / PI_DT_S;
        mid   = 0.5f * (axis_current + s_rt.axis_last_current);
        model_v = pi_actual_voltage(s_rt.axis_vlast_cmd)
                - g_param_ident.result_Rs * mid;

        s_rt.axis_sum_x  += slope;
        s_rt.axis_sum_y  += model_v;
        s_rt.axis_sum_xx += slope * slope;
        s_rt.axis_sum_xy += slope * model_v;
        g_param_ident.sample_count++;
        g_pi_axis_slope = slope;
    }

    if (g_param_ident.sample_count >= target_count) {
        return true;
    }

    axis_cmd = center_cmd;
    if (s_rt.axis_polarity == 0u) {
        axis_cmd += excite_v;
    } else {
        axis_cmd -= excite_v;
    }

    is_block_start = (s_rt.pol_cnt == 0u);

    pi_apply_voltage(axis_is_q, axis_cmd);

    s_rt.axis_vlast_cmd    = axis_cmd;
    s_rt.axis_last_current = axis_current;
    s_rt.axis_prev_valid   = true;
    s_rt.skip_next_sample  = is_block_start;
    g_pi_vlast_cmd         = axis_cmd;

    s_rt.pol_cnt++;
    if (s_rt.pol_cnt >= hold) {
        s_rt.pol_cnt = 0u;
        s_rt.axis_polarity ^= 1u;
    }
    g_pi_pol = s_rt.axis_polarity;

    return false;
}

/*---------------------------------------------------------------------------
 * 最小二乘：电感与截距
 *---------------------------------------------------------------------------*/
static bool pi_calc_inductance(float *inductance, float *voltage_offset)
{
    float n;
    float den;

    if (g_param_ident.sample_count < 2u) {
        return false;
    }

    n = (float)g_param_ident.sample_count;
    den = n * s_rt.axis_sum_xx - s_rt.axis_sum_x * s_rt.axis_sum_x;
    if (fabsf(den) <= PI_CAL_MIN_DEN) {
        return false;
    }

    *inductance = (n * s_rt.axis_sum_xy - s_rt.axis_sum_x * s_rt.axis_sum_y) / den;
    *voltage_offset = (s_rt.axis_sum_y - (*inductance) * s_rt.axis_sum_x) / n;

    return pi_is_finite_positive(*inductance) && pi_is_finite(*voltage_offset);
}

/*---------------------------------------------------------------------------
 * 结束：清输出、记录结果、回到 IDLE
 *   注意：本函数在 20kHz 中断里调用，不做任何打印
 *---------------------------------------------------------------------------*/
static void pi_finish(bool success, uint8_t error)
{
    s_vd = 0.0f;
    s_vq = 0.0f;

    g_pi_end_theta = hw_encoder_get_elec_angle_rad(CFG_MOTOR_POLE_PAIRS);
    g_pi_end_cnt   = hw_encoder_get_count();

    g_param_ident.state = PI_STATE_FINISH;
    if (success) {
        g_param_ident.progress = 100u;
    }
    g_param_ident.error  = error;
    g_param_ident.status = success ? PI_STATUS_SUCCESS : PI_STATUS_FAILED;

    Foc_Mode_SetMode(FOC_MODE_IDLE);
}

/*=============================================================================
 * 初始化 / 复位
 *=============================================================================*/
void ParamIdent_Init(void)
{
    if (s_ready) {
        return;
    }

    if (pid_dev_init(&s_pid_d, &s_priv_d, PID_MODE_POSITION) != 0) {
        MAIN_E("[PI] pid_dev_init d failed");
        return;
    }
    if (pid_dev_init(&s_pid_q, &s_priv_q, PID_MODE_POSITION) != 0) {
        MAIN_E("[PI] pid_dev_init q failed");
        return;
    }

    s_ready = true;

    pi_apply_params();
    ParamIdent_Reset();

    MAIN_D("[PI] param ident ready: kp_x10000=%ld ki_x100=%ld vmax_x100=%ld "
           "iterm_x100=%ld vbus_x100=%ld pol_hold=%u",
           (long)(g_pi_cur_kp * 10000.0f),
           (long)(g_pi_cur_ki * 100.0f),
           (long)(g_pi_cur_vmax_v * 100.0f),
           (long)(g_pi_cur_iterm_v * 100.0f),
           (long)(g_pi_vbus_v * 100.0f),
           (unsigned)g_pi_pol_hold);
}

void ParamIdent_Reset(void)
{
    uint8_t i;

    g_param_ident.status   = PI_STATUS_IDLE;
    g_param_ident.state    = PI_STATE_IDLE;
    g_param_ident.progress = 0u;
    g_param_ident.error    = PI_ERROR_NONE;
    g_param_ident.result_Rs = 0.0f;
    g_param_ident.result_Ld = 0.0f;
    g_param_ident.result_Lq = 0.0f;
    g_param_ident.voltage_offset    = 0.0f;
    g_param_ident.ld_voltage_offset = 0.0f;
    g_param_ident.lq_voltage_offset = 0.0f;
    g_param_ident.sample_count = 0u;

    for (i = 0u; i < PI_RS_POINT_NUM; i++) {
        g_param_ident.id_avg[i] = 0.0f;
        g_param_ident.ud_avg[i] = 0.0f;
    }

    s_rt.state_count      = 0u;
    s_rt.rs_index         = 0u;
    s_rt.axis_polarity    = 0u;
    s_rt.pol_cnt          = 0u;
    s_rt.axis_prev_valid  = false;
    s_rt.skip_next_sample = true;
    s_rt.id_ref           = 0.0f;
    s_rt.iq_ref           = 0.0f;
    s_rt.id_sum           = 0.0f;
    s_rt.ud_sum           = 0.0f;
    s_rt.axis_center_sum  = 0.0f;
    s_rt.axis_center_cmd  = 0.0f;
    s_rt.axis_last_current= 0.0f;
    s_rt.axis_vlast_cmd   = 0.0f;
    s_rt.axis_sum_x       = 0.0f;
    s_rt.axis_sum_y       = 0.0f;
    s_rt.axis_sum_xx      = 0.0f;
    s_rt.axis_sum_xy      = 0.0f;

    g_pi_tick_cnt     = 0u;
    g_pi_rs_index     = 0u;
    g_pi_pol          = 0u;
    g_pi_id_ref       = 0.0f;
    g_pi_iq_ref       = 0.0f;
    g_pi_center_v     = 0.0f;
    g_pi_vlast_cmd    = 0.0f;
    g_pi_axis_current = 0.0f;
    g_pi_axis_slope   = 0.0f;
    g_pi_start_theta  = 0.0f;
    g_pi_end_theta    = 0.0f;
    g_pi_start_cnt    = 0;
    g_pi_end_cnt      = 0;

    s_vd = 0.0f;
    s_vq = 0.0f;

    if (s_ready) {
        s_pid_d.ops->reset(&s_priv_d);
        s_pid_q.ops->reset(&s_priv_q);
    }
}

/*=============================================================================
 * 启动 / 中止
 *=============================================================================*/
bool ParamIdent_Start(void)
{
    if (!s_ready) {
        return false;
    }

    if (ParamIdent_IsRunning()) {
        return true;
    }

    if (Foc_Mode_GetMode() != FOC_MODE_IDLE) {
        ParamIdent_Reset();
        g_param_ident.status = PI_STATUS_FAILED;
        g_param_ident.error  = PI_ERROR_NOT_STOP;
        return false;
    }

    if (g_fp_state != FP_STATE_NORMAL) {
        ParamIdent_Reset();
        g_param_ident.status = PI_STATUS_FAILED;
        g_param_ident.error  = PI_ERROR_FAULT_ACTIVE;
        return false;
    }

    ParamIdent_Reset();
    pi_apply_params();

    g_pi_start_theta = hw_encoder_get_elec_angle_rad(CFG_MOTOR_POLE_PAIRS);
    g_pi_start_cnt   = hw_encoder_get_count();

    g_param_ident.status = PI_STATUS_RUNNING;
    g_param_ident.state  = PI_STATE_INIT;

    return true;
}

void ParamIdent_Abort(void)
{
    if (g_param_ident.status != PI_STATUS_RUNNING) {
        return;
    }

    g_pi_end_theta = hw_encoder_get_elec_angle_rad(CFG_MOTOR_POLE_PAIRS);
    g_pi_end_cnt   = hw_encoder_get_count();

    s_vd = 0.0f;
    s_vq = 0.0f;

    g_param_ident.state  = PI_STATE_FINISH;
    g_param_ident.status = PI_STATUS_ABORTED;
    if (g_fp_state != FP_STATE_NORMAL) {
        g_param_ident.error = PI_ERROR_ABORTED_BY_FAULT;
    } else {
        g_param_ident.error = PI_ERROR_INTERRUPTED;
    }
}

bool ParamIdent_IsRunning(void)
{
    return (g_param_ident.status == PI_STATUS_RUNNING);
}

/*=============================================================================
 * 20kHz 状态机
 *=============================================================================*/
void ParamIdent_Step20kHz(float *vd, float *vq, float *theta_e, bool *enable)
{
    float th;
    float target;
    float sum_x;
    float sum_y;
    float sum_xx;
    float sum_xy;
    float den;
    float inductance;
    float voltage_offset;
    bool  id_ready;
    bool  iq_ready;
    uint8_t i;

    if ((vd == NULL) || (vq == NULL) || (theta_e == NULL) || (enable == NULL)) {
        return;
    }

    if (!s_ready) {
        *vd = 0.0f;
        *vq = 0.0f;
        *theta_e = 0.0f;
        *enable = false;
        return;
    }

    th = hw_encoder_get_elec_angle_rad(CFG_MOTOR_POLE_PAIRS);
    g_pi_tick_cnt++;

    switch (g_param_ident.state) {

    /* ---- 初始化：零电压 50%，等待 PWM/反馈稳定 ---- */
    case PI_STATE_INIT:
        s_vd = 0.0f;
        s_vq = 0.0f;

        s_rt.state_count++;
        g_param_ident.progress = 5u;

        if (s_rt.state_count >= PI_INIT_COUNT) {
            s_rt.state_count = 0u;
            g_param_ident.state = PI_STATE_RS_RAMP;
        }
        break;

    /* ---- Rs：d 轴电流给定爬升 ---- */
    case PI_STATE_RS_RAMP:
        target = s_rs_current[s_rt.rs_index];
        (void)pi_ramp(&s_rt.id_ref, target, PI_CURRENT_RAMP_STEP);
        pi_apply_current(s_rt.id_ref, 0.0f);

        g_param_ident.progress = (uint8_t)(10u + s_rt.rs_index * 15u);

        if (s_rt.id_ref == target) {
            s_rt.state_count = 0u;
            g_param_ident.state = PI_STATE_RS_SETTLE;
        }
        break;

    /* ---- Rs：保持 100ms ---- */
    case PI_STATE_RS_SETTLE:
        target = s_rs_current[s_rt.rs_index];
        pi_apply_current(target, 0.0f);
        s_rt.state_count++;
        if (s_rt.state_count >= PI_RS_SETTLE_COUNT) {
            s_rt.state_count = 0u;
            s_rt.id_sum = 0.0f;
            s_rt.ud_sum = 0.0f;
            g_param_ident.sample_count = 0u;
            g_param_ident.state = PI_STATE_RS_SAMPLE;
        }
        break;

    /* ---- Rs：采样 50ms ---- */
    case PI_STATE_RS_SAMPLE:
        target = s_rs_current[s_rt.rs_index];
        pi_apply_current(target, 0.0f);

        s_rt.id_sum += g_fc_id;
        s_rt.ud_sum += pi_actual_voltage(s_vd);
        s_rt.state_count++;
        g_param_ident.sample_count = (uint16_t)s_rt.state_count;

        if (s_rt.state_count >= PI_RS_SAMPLE_COUNT) {
            g_param_ident.id_avg[s_rt.rs_index] =
                s_rt.id_sum / (float)PI_RS_SAMPLE_COUNT;
            g_param_ident.ud_avg[s_rt.rs_index] =
                s_rt.ud_sum / (float)PI_RS_SAMPLE_COUNT;

            s_rt.rs_index++;
            g_pi_rs_index = s_rt.rs_index;
            s_rt.state_count = 0u;
            if (s_rt.rs_index < PI_RS_POINT_NUM) {
                g_param_ident.state = PI_STATE_RS_RAMP;
            } else {
                g_param_ident.state = PI_STATE_RS_CALC;
            }
        }
        break;

    /* ---- Rs：三点带截距最小二乘 ---- */
    case PI_STATE_RS_CALC:
        pi_apply_current(s_rt.id_ref, 0.0f);

        sum_x = 0.0f;
        sum_y = 0.0f;
        sum_xx = 0.0f;
        sum_xy = 0.0f;
        for (i = 0u; i < PI_RS_POINT_NUM; i++) {
            sum_x  += g_param_ident.id_avg[i];
            sum_y  += g_param_ident.ud_avg[i];
            sum_xx += g_param_ident.id_avg[i] * g_param_ident.id_avg[i];
            sum_xy += g_param_ident.id_avg[i] * g_param_ident.ud_avg[i];
        }

        den = (float)PI_RS_POINT_NUM * sum_xx - sum_x * sum_x;
        if (fabsf(den) <= PI_CAL_MIN_DEN) {
            pi_finish(false, PI_ERROR_CALCULATION);
            break;
        }

        g_param_ident.result_Rs =
            ((float)PI_RS_POINT_NUM * sum_xy - sum_x * sum_y) / den;
        g_param_ident.voltage_offset =
            (sum_y - g_param_ident.result_Rs * sum_x) / (float)PI_RS_POINT_NUM;

        if (!pi_is_finite_positive(g_param_ident.result_Rs) ||
            !pi_is_finite(g_param_ident.voltage_offset)) {
            pi_finish(false, PI_ERROR_CALCULATION);
            break;
        }

        s_rt.state_count = 0u;
        s_rt.axis_center_sum = 0.0f;
        g_param_ident.sample_count = 0u;
        g_param_ident.progress = 65u;
        g_param_ident.state = PI_STATE_LD_PREPARE;
        break;

    /* ---- Ld 准备：0.3A 偏置，平均 d 轴中心电压 ---- */
    case PI_STATE_LD_PREPARE:
        id_ready = pi_ramp(&s_rt.id_ref, g_pi_ld_bias_a, PI_CURRENT_RAMP_STEP);
        iq_ready = pi_ramp(&s_rt.iq_ref, 0.0f, PI_CURRENT_RAMP_STEP);

        if (id_ready && iq_ready) {
            s_rt.state_count++;
        } else {
            s_rt.state_count = 0u;
            s_rt.axis_center_sum = 0.0f;
        }

        pi_apply_current(s_rt.id_ref, s_rt.iq_ref);

        if (s_rt.state_count > (PI_RS_SETTLE_COUNT - PI_CENTER_SAMPLE_COUNT)) {
            s_rt.axis_center_sum += s_vd;
        }
        g_param_ident.progress = 70u;

        if (s_rt.state_count >= PI_RS_SETTLE_COUNT) {
            s_rt.axis_center_cmd =
                s_rt.axis_center_sum / (float)PI_CENTER_SAMPLE_COUNT;
            g_pi_center_v = pi_actual_voltage(s_rt.axis_center_cmd);

            pi_reset_axis_samples(g_fc_id);
            g_param_ident.state = PI_STATE_LD_EXCITE;
        }
        break;

    /* ---- Ld 激励：d 轴 ±0.4V，q 轴零电流闭环 ---- */
    case PI_STATE_LD_EXCITE:
        if (pi_run_excitation(false, s_rt.axis_center_cmd,
                              g_pi_ld_excite_v, PI_LD_SAMPLE_COUNT)) {
            g_param_ident.progress = 82u;
            g_param_ident.state = PI_STATE_LD_CALC;
        } else {
            g_param_ident.progress = (uint8_t)(70u +
                ((uint32_t)g_param_ident.sample_count * 12u /
                 PI_LD_SAMPLE_COUNT));
        }
        break;

    /* ---- Ld 计算 ---- */
    case PI_STATE_LD_CALC:
        pi_apply_current(s_rt.id_ref, s_rt.iq_ref);

        if (!pi_calc_inductance(&inductance, &voltage_offset)) {
            pi_finish(false, PI_ERROR_CALCULATION);
            break;
        }
        g_param_ident.result_Ld = inductance;
        g_param_ident.ld_voltage_offset = voltage_offset;

        s_rt.state_count = 0u;
        s_rt.axis_center_sum = 0.0f;
        g_param_ident.sample_count = 0u;
        g_param_ident.progress = 83u;
        g_param_ident.state = PI_STATE_LQ_PREPARE;
        break;

    /* ---- Lq 准备：Id/Iq 回零，平均 q 轴中心电压 ---- */
    case PI_STATE_LQ_PREPARE:
        id_ready = pi_ramp(&s_rt.id_ref, 0.0f, PI_CURRENT_RAMP_STEP);
        iq_ready = pi_ramp(&s_rt.iq_ref, g_pi_lq_bias_a, PI_CURRENT_RAMP_STEP);

        if (id_ready && iq_ready) {
            s_rt.state_count++;
        } else {
            s_rt.state_count = 0u;
            s_rt.axis_center_sum = 0.0f;
        }

        pi_apply_current(s_rt.id_ref, s_rt.iq_ref);

        if (s_rt.state_count > (PI_RS_SETTLE_COUNT - PI_CENTER_SAMPLE_COUNT)) {
            s_rt.axis_center_sum += s_vq;
        }
        g_param_ident.progress = 85u;

        if (s_rt.state_count >= PI_RS_SETTLE_COUNT) {
            s_rt.axis_center_cmd =
                s_rt.axis_center_sum / (float)PI_CENTER_SAMPLE_COUNT;
            g_pi_center_v = pi_actual_voltage(s_rt.axis_center_cmd);

            pi_reset_axis_samples(g_fc_iq);
            g_param_ident.state = PI_STATE_LQ_EXCITE;
        }
        break;

    /* ---- Lq 激励：q 轴 ±0.4V，d 轴零电流闭环 ---- */
    case PI_STATE_LQ_EXCITE:
        if (pi_run_excitation(true, s_rt.axis_center_cmd,
                              g_pi_lq_excite_v, PI_LQ_SAMPLE_COUNT)) {
            g_param_ident.progress = 98u;
            g_param_ident.state = PI_STATE_LQ_CALC;
        } else {
            g_param_ident.progress = (uint8_t)(85u +
                ((uint32_t)g_param_ident.sample_count * 13u /
                 PI_LQ_SAMPLE_COUNT));
        }
        break;

    /* ---- Lq 计算，成功后结束 ---- */
    case PI_STATE_LQ_CALC:
        pi_apply_current(s_rt.id_ref, s_rt.iq_ref);

        if (!pi_calc_inductance(&inductance, &voltage_offset)) {
            pi_finish(false, PI_ERROR_CALCULATION);
            break;
        }
        g_param_ident.result_Lq = inductance;
        g_param_ident.lq_voltage_offset = voltage_offset;
        pi_finish(true, PI_ERROR_NONE);
        break;

    case PI_STATE_FINISH:
    case PI_STATE_IDLE:
    default:
        /* 未经 Start 直接切到本模式时，按计算失败结束 */
        if (g_param_ident.state != PI_STATE_FINISH) {
            pi_finish(false, PI_ERROR_CALCULATION);
        }
        break;
    }

    /* 结束/失败后强制零输出，避免静态分支输出 50% */
    if ((g_param_ident.status != PI_STATUS_RUNNING) ||
        (g_param_ident.state == PI_STATE_FINISH)) {
        s_vd = 0.0f;
        s_vq = 0.0f;
        *enable = false;
    } else {
        *enable = true;
    }

    *vd = s_vd;
    *vq = s_vq;
    *theta_e = th;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
