/**
 * @file    foc_mode.c
 * @brief   FOC 模式层实现（IDLE / LOCK）
 * @note    IDLE：六路全低 + EMB 封锁。
 *          LOCK：解除 EMB，恢复 PWM，输出 2% 零矢量。
 *          2% 取自参考工程 foc_control.c 的静态输出分支
 *          （hw_tmr4_pwm_set_duty3(2.0f, 2.0f, 2.0f)）。
 */

#include "foc_mode.h"
#include "hw_tmr4_pwm.h"
#include "../../bsp/bsp_pwm.h"

#include <rtthread.h>
#include <finsh.h>

#define FOC_LOCK_DUTY_PCT   (2.0f)

static volatile foc_mode_t s_mode = FOC_MODE_IDLE;

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

        err = hw_tmr4_pwm_set_duty3(FOC_LOCK_DUTY_PCT,
                                    FOC_LOCK_DUTY_PCT,
                                    FOC_LOCK_DUTY_PCT);
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
    rt_kprintf("[FOC-MODE] -> mode %d (%s)\r\n", (int)mode,
               (mode == FOC_MODE_IDLE) ? "IDLE, all low"
                                       : "LOCK, 2% zero vector");
}

foc_mode_t Foc_Mode_GetMode(void)
{
    return s_mode;
}

/*=============================================================================
 * msh 命令
 *   focmode        打印当前模式
 *   focmode 0      IDLE：六路全低 + EMB 封锁
 *   focmode 1      LOCK：解除 EMB，输出 2% 零矢量
 *=============================================================================*/
static int foc_mode_msh(int argc, char **argv)
{
    if (argc == 1) {
        rt_kprintf("[FOC-MODE] current mode = %d (%s)\r\n",
                   (int)Foc_Mode_GetMode(),
                   (Foc_Mode_GetMode() == FOC_MODE_IDLE) ? "IDLE, all low"
                                                         : "LOCK, 2% zero vector");
        rt_kprintf("usage: focmode <0|1>\r\n");
        return 0;
    }

    if ((argc == 2) &&
        (argv[1][0] >= '0') && (argv[1][0] <= '1') &&
        (argv[1][1] == '\0')) {
        Foc_Mode_SetMode((foc_mode_t)(argv[1][0] - '0'));
        return 0;
    }

    rt_kprintf("usage: focmode <0|1>   0=IDLE all low, 1=LOCK 2%% zero vector\r\n");
    return -1;
}

MSH_CMD_EXPORT_ALIAS(foc_mode_msh, focmode, set FOC mode 0=IDLE 1=LOCK);
