/**
 * @file    app_test.c
 * @brief   硬件手动测试命令集
 * @note    每个命令只测一项，互不干扰，方便单独验证
 *
 * 命令清单：
 *   t_brake_unlock         解锁刹车
 *   t_brake_lock           锁定刹车
 *   t_brake_state          查询刹车状态
 *   t_pwm <ch> <duty>      设置单通道 PWM 占空比
 *   t_pwm_all <duty>       设置 3 路 PWM 相同占空比
 *   t_pwm_off              关闭所有 PWM
 *   t_hall                 读一次霍尔状态
 *   t_hall_loop <ms>       循环打印霍尔状态（用于手动转动观察）
 *   t_hall_reset           清零霍尔计数
 *   t_motor_status         打印电机状态
 *   t_motor_start <rpm>    开环启动电机
 *   t_motor_stop           停止电机
 *   t_motor_step <n>       强制换相到第 n 步（n=0..5）
 */

#include <rtthread.h>
#include <finsh.h>
#include <stdlib.h>
#include <string.h>

#include "mode/sys_caps.h"          /* 能力位门禁（2026-10-27） */
#include "hw/hw_brake.h"
#include "hw/hw_hall.h"
#include "hw/hw_pwm.h"
#include "motor/motor.h"
#include "swing/swing_angle.h"      /* t_swing：角度/限位/解算有效性 */
#include "swing/swing_control.h"    /* t_swing：回转状态机状态      */

/* ============================================================
 * 全局对象引用（由 app_hw.c / motor_init.c 定义）
 * ============================================================ */
extern hw_pwm_t   g_swing_pwm;
extern hw_brake_t g_swing_brake;
extern hw_hall_t  g_swing_hall;
extern motor_t    g_swing_motor;

/* ============================================================
 * 工具函数
 * ============================================================ */
static const char *brake_state_str(hw_brake_state_t s)
{
    switch (s) {
    case BRAKE_STATE_LOCKED:    return "LOCKED";
    case BRAKE_STATE_UNLOCKING: return "UNLOCKING";
    case BRAKE_STATE_UNLOCKED:  return "UNLOCKED";
    case BRAKE_STATE_LOCKING:   return "LOCKING";
    case BRAKE_STATE_FAULT:     return "FAULT";
    default:                    return "?";
    }
}

static const char *hall_state_str(uint8_t s)
{
    switch (s) {
    case 0: return "000 invalid";
    case 1: return "001";
    case 2: return "010";
    case 3: return "011";
    case 4: return "100";
    case 5: return "101";
    case 6: return "110";
    case 7: return "111 invalid";
    default: return "?";
    }
}

/* ============================================================
 * 刹车测试
 * ============================================================ */
static void cmd_brake_unlock(int argc, char **argv)
{
    (void)argc; (void)argv;
    /* 危险：强制解锁刹车，机构可能因重力/惯性滑行 */
    if (sys_cap_require(SYS_CAP_DIAG_WRITE, "t_brake_unlock") == 0U) { return; }
    rt_err_t ret = hw_brake_unlock(&g_swing_brake);
    rt_kprintf("[t_brake] unlock ret=%d\n", ret);
}

static void cmd_brake_lock(int argc, char **argv)
{
    (void)argc; (void)argv;
    /* 危险：绕过状态机强制锁止 */
    if (sys_cap_require(SYS_CAP_DIAG_WRITE, "t_brake_lock") == 0U) { return; }
    rt_err_t ret = hw_brake_lock(&g_swing_brake);
    rt_kprintf("[t_brake] lock ret=%d\n", ret);
}

static void cmd_brake_state(int argc, char **argv)
{
    (void)argc; (void)argv;
    hw_brake_state_t s = hw_brake_get_state(&g_swing_brake);
    rt_kprintf("[t_brake] state = %d (%s)\n", s, brake_state_str(s));
}

/* ============================================================
 * t_swing —— 回转角度 / 限位 / 解算 / 状态机 / 刹车 一次打印
 * ------------------------------------------------------------
 * 台架用：配合 t_sim swing <x10> 验证限位、零位、过零与刹车时序。
 * 角度单位 0.1°，与业务一致。
 * ============================================================ */
static const char *swing_pos_str(swing_position_t p)
{
    switch (p) {
    case SWING_POS_LEFT_LIMIT:  return "LEFT_LIMIT (-100deg)";
    case SWING_POS_RIGHT_LIMIT: return "RIGHT_LIMIT (+215deg)";
    case SWING_POS_ZERO:        return "ZERO (0deg)";
    case SWING_POS_MIDDLE:      return "MIDDLE";
    default:                    return "UNKNOWN";
    }
}

static const char *swing_state_str(swing_state_t s)
{
    switch (s) {
    case H_STATE_IDLE:          return "IDLE";
    case H_STATE_UNLOCKING:     return "UNLOCKING";
    case H_STATE_LEFT_RUNNING:  return "LEFT_RUNNING";
    case H_STATE_RIGHT_RUNNING: return "RIGHT_RUNNING";
    case H_STATE_LOCKING:       return "LOCKING";
    case H_STATE_FAULT:         return "FAULT";
    default:                    return "?";
    }
}

static void cmd_swing_state(int argc, char **argv)
{
    swing_angle_t *a;
    int32_t x10;
    int32_t ax;

    (void)argc; (void)argv;

    a   = swing_angle_get();
    x10 = swing_angle_get_value(a);
    ax  = (x10 < 0) ? -x10 : x10;

    rt_kprintf("\n======== SWING CHECK ========\n");
    rt_kprintf("angle   : %s%d.%d deg  (x10=%d)\n",
               (x10 < 0) ? "-" : "", (int)(ax / 10), (int)(ax % 10), (int)x10);
    rt_kprintf("position: %s\n", swing_pos_str(a->position));
    rt_kprintf("flags   : zero=%u left=%u right=%u mid=%u\n",
               (unsigned)a->is_at_zero, (unsigned)a->is_at_left_limit,
               (unsigned)a->is_at_right_limit, (unsigned)a->is_at_middle);
    rt_kprintf("zero    : %d (encoder_zero_offset)\n", (int)a->encoder_zero_offset);
    rt_kprintf("safe    : %u   solve_valid=%u\n",
               (unsigned)swing_angle_is_safe(a),
               (unsigned)swing_angle_is_solve_valid());
    rt_kprintf("state   : %s\n", swing_state_str(swing_control_get_state()));
    rt_kprintf("brake   : %s\n", brake_state_str(hw_brake_get_state(&g_swing_brake)));
    rt_kprintf("dir_pol : %u\n", (unsigned)g_swing_dir_polarity);
    rt_kprintf("=============================\n\n");
}

MSH_CMD_EXPORT_ALIAS(cmd_swing_state, t_swing, show swing angle/limit/state/brake);

/* ============================================================
 * PWM 测试
 * ============================================================ */
/* ★ 请按 hw_pwm.h 实际接口调整以下 3 个函数名 */
extern rt_err_t hw_pwm_set_duty(hw_pwm_t *self, uint8_t ch, uint32_t duty);
extern rt_err_t hw_pwm_enable(hw_pwm_t *self);
extern rt_err_t hw_pwm_disable(hw_pwm_t *self);
/* 如果接口不同，比如 hw_pwm_set_duty(self, duty_a, duty_b, duty_c)，
 * 请在这里改。 */

static void cmd_pwm(int argc, char **argv)
{
    if (argc < 3) {
        rt_kprintf("usage: t_pwm <ch 0..2> <duty 0..3000>\n");
        return;
    }
    /* 危险：直接驱动功率管，绕过所有保护 */
    if (sys_cap_require(SYS_CAP_DIAG_WRITE, "t_pwm") == 0U) { return; }

    uint8_t  ch   = (uint8_t)atoi(argv[1]);
    uint32_t duty = (uint32_t)atoi(argv[2]);

    if (ch > 2) {
        rt_kprintf("ch must be 0..2\n");
        return;
    }

    rt_err_t ret = hw_pwm_set_duty(&g_swing_pwm, ch, duty);
    rt_kprintf("[t_pwm] ch=%d duty=%u ret=%d\n", ch, duty, ret);
}

static void cmd_pwm_all(int argc, char **argv)
{
    if (argc < 2) {
        rt_kprintf("usage: t_pwm_all <duty 0..3000>\n");
        return;
    }
    if (sys_cap_require(SYS_CAP_DIAG_WRITE, "t_pwm_all") == 0U) { return; }

    uint32_t duty = (uint32_t)atoi(argv[1]);
    for (uint8_t ch = 0; ch < 3; ch++) {
        hw_pwm_set_duty(&g_swing_pwm, ch, duty);
    }
    rt_kprintf("[t_pwm] all ch duty=%u\n", duty);
}

static void cmd_pwm_off(int argc, char **argv)
{
    (void)argc; (void)argv;
    for (uint8_t ch = 0; ch < 3; ch++) {
        hw_pwm_set_duty(&g_swing_pwm, ch, 0);
    }
    rt_kprintf("[t_pwm] all off\n");
}

/* ============================================================
 * 霍尔测试
 * ============================================================ */
static void cmd_hall(int argc, char **argv)
{
    (void)argc; (void)argv;
    uint8_t  s     = hw_hall_read_state(&g_swing_hall);
    uint32_t count = hw_hall_get_count(&g_swing_hall);
    uint32_t tick  = hw_hall_get_last_tick(&g_swing_hall);

    rt_kprintf("[t_hall] state=%d (%s) count=%u last_tick=%u\n",
               s, hall_state_str(s), count, tick);
}

static void cmd_hall_loop(int argc, char **argv)
{
    uint32_t period_ms = (argc > 1) ? (uint32_t)atoi(argv[1]) : 200;
    uint32_t last_count = hw_hall_get_count(&g_swing_hall);

    rt_kprintf("[t_hall] looping every %u ms, press any key to stop\n",
               period_ms);
    rt_kprintf("[t_hall] (rotate motor shaft by hand to observe changes)\n");

    /* 前 30 秒自动退出，或按键退出（这里简化处理） */
    for (int i = 0; i < 150; i++) {
        uint8_t  s     = hw_hall_read_state(&g_swing_hall);
        uint32_t count = hw_hall_get_count(&g_swing_hall);
        rt_kprintf("[t_hall] state=%d (%s) count=%u  %s\n",
                   s, hall_state_str(s), count,
                   (count != last_count) ? "CHANGED" : "");
        last_count = count;
        rt_thread_mdelay(period_ms);
    }
    rt_kprintf("[t_hall] loop done\n");
}

static void cmd_hall_reset(int argc, char **argv)
{
    (void)argc; (void)argv;
    /* 危险：清零霍尔计数会破坏转速闭环的基准，运行中清零可能导致
     * 速度环判定突变。归入诊断写权限。 */
    if (sys_cap_require(SYS_CAP_DIAG_WRITE, "t_hall_reset") == 0U) { return; }
    /* 部分实现里 count 是 volatile，直接写 0 */
    extern hw_hall_t g_swing_hall;
    g_swing_hall.capture_count = 0;
    rt_kprintf("[t_hall] count reset to 0\n");
}

/* ============================================================
 * 电机测试
 * ============================================================ */
/* ★ 以下函数名请按 motor.h / motor_drv_bldc_6step.h 实际接口调整 */
extern rt_err_t motor_start(motor_t *motor, uint8_t direction);
extern rt_err_t motor_stop(motor_t *motor);
extern rt_err_t motor_force_step(motor_t *m, uint8_t step);

static void cmd_motor_status(int argc, char **argv)
{
    (void)argc; (void)argv;
    rt_kprintf("[t_motor] motor obj @ %p\n", &g_swing_motor);
    rt_kprintf("[t_motor] hall_state = %d (%s)\n",
               g_swing_hall.last_state,
               hall_state_str(g_swing_hall.last_state));
//    rt_kprintf("[t_motor] pwm_duty   = %u\n", g_swing_motor.pwm_duty);
    /* 按 motor_t 实际字段添加 */
}

static void cmd_motor_start_cw(int argc, char **argv)
{
    (void)argc; (void)argv;
    /* 危险：开环启动，绕过门禁/限位/校准，直接驱动功率级 */
    if (sys_cap_require(SYS_CAP_DIAG_WRITE, "t_motor_start_cw") == 0U) { return; }

    /* 先松刹车 */
    hw_brake_unlock(&g_swing_brake);
    rt_thread_mdelay(200);

    rt_err_t ret = motor_start(&g_swing_motor, MOTOR_DIR_CW);
    rt_kprintf("[t_motor] start ret=%d\n", ret);
}

static void cmd_motor_start_ccw(int argc, char **argv)
{
    (void)argc; (void)argv;
    if (sys_cap_require(SYS_CAP_DIAG_WRITE, "t_motor_start_ccw") == 0U) { return; }

    /* 先松刹车 */
    hw_brake_unlock(&g_swing_brake);
    rt_thread_mdelay(200);

    rt_err_t ret = motor_start(&g_swing_motor, MOTOR_DIR_CCW);
    rt_kprintf("[t_motor] start ret=%d\n", ret);
}

static void cmd_motor_stop(int argc, char **argv)
{
    (void)argc; (void)argv;
    motor_stop(&g_swing_motor);
    rt_thread_mdelay(100);
    hw_brake_lock(&g_swing_brake);
    rt_kprintf("[t_motor] stopped, brake locked\n");
}

static void cmd_motor_step(int argc, char **argv)
{
    if (argc < 2) {
        rt_kprintf("usage: t_motor_step <0..5>\n");
        return;
    }
    /* 危险：强制换相，可造成直通/堵转 */
    if (sys_cap_require(SYS_CAP_DIAG_WRITE, "t_motor_step") == 0U) { return; }

    uint8_t step = (uint8_t)atoi(argv[1]);
    if (step > 5) {
        rt_kprintf("step must be 0..5\n");
        return;
    }
    rt_err_t ret = motor_stop(&g_swing_motor);
    rt_kprintf("[t_motor] force_step %d ret=%d\n", step, ret);
}

// msh测试函数示例
static void cmd_can_tx(void)
{
    /* 【2026-10-27】用独立能力位 CAN_TX_TEST，而不是 DIAG_WRITE：
     * 这条命令往 CAN 总线注入伪造帧，整机联调时可能被上位机误认为
     * 真实状态上报。工作模式必须禁掉，维护/工厂模式才放开。 */
    if (sys_cap_require(SYS_CAP_CAN_TX_TEST, "t_can_tx") == 0U) { return; }

    uint8_t data[8] = {0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88};

    extern int can_send(uint32_t id, uint8_t *data, uint8_t len);
    int ret = can_send(0x18FF0001, data, 8);
    rt_kprintf("can send ret = %d\n", ret);
}

/* ============================================================
 * 命令导出
 * ============================================================ */
MSH_CMD_EXPORT_ALIAS(cmd_brake_unlock,     t_brake_unlock, unlock swing brake);
MSH_CMD_EXPORT_ALIAS(cmd_brake_lock,       t_brake_lock,   lock swing brake);
MSH_CMD_EXPORT_ALIAS(cmd_brake_state,      t_brake_state,  show brake state);

MSH_CMD_EXPORT_ALIAS(cmd_pwm,              t_pwm,          set one pwm ch duty);
MSH_CMD_EXPORT_ALIAS(cmd_pwm_all,          t_pwm_all,      set all pwm duty);
MSH_CMD_EXPORT_ALIAS(cmd_pwm_off,          t_pwm_off,      turn off all pwm);

MSH_CMD_EXPORT_ALIAS(cmd_hall,             t_hall,         read hall once);
MSH_CMD_EXPORT_ALIAS(cmd_hall_loop,        t_hall_loop,    loop read hall);
MSH_CMD_EXPORT_ALIAS(cmd_hall_reset,       t_hall_reset,   reset hall counter);

MSH_CMD_EXPORT_ALIAS(cmd_motor_status,     t_motor_status, show motor status);
MSH_CMD_EXPORT_ALIAS(cmd_motor_start_cw,   t_motor_start_cw,  start motor openloop);
MSH_CMD_EXPORT_ALIAS(cmd_motor_start_ccw,  t_motor_start_ccw,  start motor openloop);
MSH_CMD_EXPORT_ALIAS(cmd_motor_stop,       t_motor_stop,   stop motor);
MSH_CMD_EXPORT_ALIAS(cmd_motor_step,       t_motor_step,   force one commutation step);

MSH_CMD_EXPORT_ALIAS(cmd_can_tx,           t_can_tx,       test can send);
