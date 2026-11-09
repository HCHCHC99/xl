/*=============================================================================
 * foc_position_loop.c  —  L8 位置外环实现
 *
 * 调度：Foc_Mode_Step 的 FOC_MODE_POSITION 分支，每拍调用 Foc_Position_Task20k，
 *       内部每 200 拍（10ms）执行一次完整的位置环计算，然后交给速度环。
 *
 * 位置基准：
 *   多圈机械位置（圈）= g_enc_count × g_enc_elec_dir / CFG_ENCODER_CPR - 零点偏移。
 *   写 g_pos_zero_req=1 时把当前位置设为 0 并把目标清零。
 *
 * 激活门控：
 *   电流环预热 + 在线零偏校准完成前（Foc_CurrentLoop_IsActive()=false），
 *   位置环不积分、速度指令恒为 0。
 *=============================================================================*/

#include "foc_position_loop.h"
#include "pid_ctrl.h"
#include "foc_current_loop.h"
#include "foc_speed_loop.h"   /* g_spd_target_rpm */
#include "hw_encoder.h"       /* hw_encoder_get_count / g_enc_elec_dir */
#include "cfg_motor.h"        /* CFG_ENCODER_CPR */
#include "rtt_log.h"

/*---------------------------------------------------------------------------
 * 编译期常量
 *---------------------------------------------------------------------------*/
#define POS_ISR_HZ          (20000.0f)
#define POS_DT_S            ((float)POS_CTRL_DIV / POS_ISR_HZ)     /* 0.01s */

/*---------------------------------------------------------------------------
 * 可调参数变量
 *---------------------------------------------------------------------------*/
volatile float   g_pos_target_rev        = 0.0f;
volatile float   g_pos_kp                = POS_KP_INIT;
volatile float   g_pos_ki                = POS_KI_INIT;
volatile float   g_pos_kd                = POS_KD_INIT;
volatile float   g_pos_speed_limit_rpm   = POS_SPEED_LIMIT_RPM_INIT;
volatile float   g_pos_iterm_max_rpm     = POS_ITERM_MAX_RPM_INIT;
volatile float   g_pos_err_max_rev       = POS_ERR_MAX_REV_INIT;
volatile float   g_pos_deadband_rev      = POS_DEADBAND_REV_INIT;
volatile float   g_pos_creep_rpm         = POS_CREEP_RPM_INIT;
volatile float   g_pos_stall_rpm         = POS_STALL_RPM_INIT;
volatile uint8_t g_pos_creep_enable      = POS_CREEP_EN_INIT;
volatile uint8_t g_pos_brake_enable      = POS_BRAKE_EN_INIT;
volatile float   g_pos_brake_speed_rpm   = POS_BRAKE_SPEED_RPM_INIT;
volatile float   g_pos_brake_hyst_rev    = POS_BRAKE_HYST_REV_INIT;
volatile float   g_pos_limit_rev         = POS_LIMIT_REV_INIT;
volatile float   g_pos_back_gain         = POS_BACK_GAIN_INIT;
volatile uint8_t g_pos_zero_req          = 0u;

/*---------------------------------------------------------------------------
 * 观测量
 *---------------------------------------------------------------------------*/
volatile uint8_t g_pos_active   = 0u;
volatile uint8_t g_pos_brake    = 0u;
volatile float   g_pos_fb_rev   = 0.0f;
volatile float   g_pos_err_rev  = 0.0f;
volatile float   g_pos_cmd_rpm  = 0.0f;

/*---------------------------------------------------------------------------
 * 内部状态
 *---------------------------------------------------------------------------*/
static struct pid_dev  s_pid_pos;
static struct pid_priv s_priv_pos;

static bool     s_ready          = false;
static uint16_t s_div_cnt        = 0u;
static bool     s_brake          = false;
static float    s_pos_offset_rev = 0.0f;
static float    s_fb_rev         = 0.0f;
static float    s_err_rev        = 0.0f;

/* 当前位置（圈），正方向与速度环一致 */
static float pos_read_rev(void)
{
    float raw = (float)hw_encoder_get_count() * (float)g_enc_elec_dir
              / (float)CFG_ENCODER_CPR;

    return raw - s_pos_offset_rev;
}

/*=============================================================================
 * 初始化
 *=============================================================================*/
void Foc_Position_Init(void)
{
    if (s_ready) {
        return;   /* 幂等 */
    }

    if (pid_dev_init(&s_pid_pos, &s_priv_pos, PID_MODE_POSITION) != 0) {
        MAIN_E("[POS] pid_dev_init failed");
        return;
    }

    (void)s_pid_pos.ops->set_param(&s_priv_pos, POS_KP_INIT, POS_KI_INIT, POS_KD_INIT);
    (void)s_pid_pos.ops->set_limit(&s_priv_pos,
                                   -POS_SPEED_LIMIT_RPM_INIT,
                                    POS_SPEED_LIMIT_RPM_INIT,
                                    POS_BACK_GAIN_INIT);
    (void)s_pid_pos.ops->set_iterm_limit(&s_priv_pos, POS_ITERM_MAX_RPM_INIT);

    s_div_cnt        = 0u;
    s_pos_offset_rev = 0.0f;
    s_fb_rev         = 0.0f;
    s_err_rev        = 0.0f;

    g_pos_active  = 0u;
    g_pos_brake   = 0u;
    g_pos_fb_rev  = 0.0f;
    g_pos_err_rev = 0.0f;
    g_pos_cmd_rpm = 0.0f;

    s_brake = false;

    s_ready = true;

    MAIN_D("[POS] position loop ready: div=%u dt_us=%ld kp_x10=%ld ki_x10=%ld "
           "kd_x10=%ld spd_lim=%ld iterm=%ld err_max_x1000=%ld db_x1000=%ld limit_rev=%ld",
           (unsigned)POS_CTRL_DIV,
           (long)(POS_DT_S * 1e6f),
           (long)(g_pos_kp * 10.0f),
           (long)(g_pos_ki * 10.0f),
           (long)(g_pos_kd * 10.0f),
           (long)g_pos_speed_limit_rpm,
           (long)g_pos_iterm_max_rpm,
           (long)(g_pos_err_max_rev * 1000.0f),
           (long)(g_pos_deadband_rev * 1000.0f),
           (long)g_pos_limit_rev);
}

/*=============================================================================
 * 进入 / 离开位置模式
 *=============================================================================*/
void Foc_Position_Reset(void)
{
    if (!s_ready) {
        return;
    }

    s_pid_pos.ops->reset(&s_priv_pos);

    s_div_cnt = 0u;
    s_fb_rev  = pos_read_rev();
    s_err_rev = 0.0f;

    g_pos_fb_rev  = s_fb_rev;
    g_pos_err_rev = 0.0f;
    g_pos_cmd_rpm = 0.0f;
    g_pos_active  = 0u;
    g_pos_brake   = 0u;
    s_brake       = false;

    /* 进入时以当前位置为目标：先原地保持，再由 Watch 写新目标 */
    g_pos_target_rev = s_fb_rev;
}

void Foc_Position_Stop(void)
{
    if (!s_ready) {
        return;
    }

    s_pid_pos.ops->reset(&s_priv_pos);

    s_div_cnt = 0u;
    s_err_rev = 0.0f;

    g_pos_err_rev = 0.0f;
    g_pos_cmd_rpm = 0.0f;
    g_pos_active  = 0u;
    g_pos_brake   = 0u;
    s_brake       = false;
}

/*=============================================================================
 * 每拍任务（内部 10ms 抽稀）
 *=============================================================================*/
void Foc_Position_Task20k(void)
{
    float pos;
    float target;
    float err;
    float cmd;
    float lim;
    float iterm;
    float bg;
    float db;
    float err_max;
    float spd_lim;
    float creep;
    float stall;
    float spd_abs;
    float err_raw;
    float bspd;
    float hyst;
    uint8_t brake_en;

    if (!s_ready) {
        return;
    }

    s_div_cnt++;
    if (s_div_cnt < POS_CTRL_DIV) {
        return;
    }
    s_div_cnt = 0u;

    /* ---- 零点请求：当前位置设为 0，目标清零，复位 PI ---- */
    if (g_pos_zero_req != 0u) {
        g_pos_zero_req   = 0u;
        s_pos_offset_rev = (float)hw_encoder_get_count() * (float)g_enc_elec_dir
                         / (float)CFG_ENCODER_CPR;
        g_pos_target_rev = 0.0f;
        s_pid_pos.ops->reset(&s_priv_pos);
    }

    pos = pos_read_rev();
    s_fb_rev = pos;
    g_pos_fb_rev = pos;

    /* ---- 电流环未 active：不积分、速度指令归零 ---- */
    if (!Foc_CurrentLoop_IsActive()) {
        s_err_rev     = 0.0f;
        g_pos_err_rev = 0.0f;
        g_pos_cmd_rpm = 0.0f;
        g_pos_active  = 0u;
        g_spd_target_rpm = 0.0f;
        return;
    }
    g_pos_active = 1u;

    /* ---- 参数在线同步（Watch 改动下一拍生效）---- */
    lim = g_pos_limit_rev;
    if (lim < 0.0f) {
        lim = 0.0f;
    }
    iterm = g_pos_iterm_max_rpm;
    if (iterm < 0.0f) {
        iterm = 0.0f;
    }
    bg = g_pos_back_gain;
    if (bg < 0.0f) {
        bg = 0.0f;
    }
    if (bg > 1.0f) {
        bg = 1.0f;
    }
    err_max = g_pos_err_max_rev;
    if (err_max < 0.0f) {
        err_max = 0.0f;
    }
    db = g_pos_deadband_rev;
    if (db < 0.0f) {
        db = 0.0f;
    }
    spd_lim = g_pos_speed_limit_rpm;
    if (spd_lim < 1.0f) {
        spd_lim = 1.0f;
    }

    (void)s_pid_pos.ops->set_param(&s_priv_pos, g_pos_kp, g_pos_ki, g_pos_kd);
    (void)s_pid_pos.ops->set_limit(&s_priv_pos, -spd_lim, spd_lim, bg);
    (void)s_pid_pos.ops->set_iterm_limit(&s_priv_pos, iterm);

    /* ---- 目标软行程限位 ---- */
    target = g_pos_target_rev;
    if (target >  lim) target =  lim;
    if (target < -lim) target = -lim;

    /* ---- 到位刹车状态机 ---- */
    err_raw  = target - pos;
    brake_en = (g_pos_brake_enable != 0u) ? 1u : 0u;
    bspd     = g_pos_brake_speed_rpm;
    if (bspd < 0.0f) {
        bspd = 0.0f;
    }
    hyst = g_pos_brake_hyst_rev;
    if (hyst < 0.0f) {
        hyst = 0.0f;
    }
    spd_abs = g_spd_filt_rpm;
    if (spd_abs < 0.0f) {
        spd_abs = -spd_abs;
    }

    if (s_brake) {
        /* 误差超出（死区 + 滞环）或刹车被关闭：退出刹车，清速度环 */
        if ((brake_en == 0u) ||
            (err_raw >  (db + hyst)) ||
            (err_raw < -(db + hyst))) {
            s_brake     = false;
            g_pos_brake = 0u;
            Foc_Speed_Stop();
            s_pid_pos.ops->reset(&s_priv_pos);
        }
    } else {
        /* 误差进入死区且实际速度足够低：进入刹车 */
        if ((brake_en != 0u) &&
            (err_raw <  db) && (err_raw > -db) &&
            (spd_abs < bspd)) {
            s_brake     = true;
            g_pos_brake = 1u;
            s_pid_pos.ops->reset(&s_priv_pos);
        }
    }

    if (s_brake) {
        s_err_rev        = 0.0f;
        g_pos_err_rev    = 0.0f;
        g_pos_cmd_rpm    = 0.0f;
        g_spd_target_rpm = 0.0f;
        return;
    }

    /* ---- 误差限幅 + 死区 ---- */
    err = err_raw;
    if (err >  err_max) err =  err_max;
    if (err < -err_max) err = -err_max;
    if ((err < db) && (err > -db)) {
        err = 0.0f;
    }

    /* ---- 位置 PI：把限幅后的误差作为给定 ---- */
    cmd = s_pid_pos.ops->calc(&s_priv_pos, pos, pos + err, 0.0f, 0.0f, POS_DT_S);

    /* ---- 爬行下限：误差超出死区且实际速度接近 0 时，强制最低速度 ---- */
    if ((g_pos_creep_enable != 0u) && (err != 0.0f)) {
        creep = g_pos_creep_rpm;
        stall = g_pos_stall_rpm;
        if (creep < 0.0f) {
            creep = 0.0f;
        }
        if (stall < 0.0f) {
            stall = 0.0f;
        }

        if ((creep > 0.0f) && (spd_abs < stall)) {
            if ((cmd < creep) && (cmd > -creep)) {
                cmd = (err > 0.0f) ? creep : -creep;
            }
        }
    }

    s_err_rev     = err;
    g_pos_err_rev = err;
    g_pos_cmd_rpm = cmd;

    /* ---- 下发速度指令，交给速度环 ---- */
    g_spd_target_rpm = cmd;
}

bool Foc_Position_IsBrake(void)
{
    return (s_ready && s_brake);
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
