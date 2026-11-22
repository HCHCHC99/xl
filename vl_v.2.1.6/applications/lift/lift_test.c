/**
 * @file    lift_test.c
 * @brief   举升角传感器自检命令（MSH）
 * @note    全链路验证：ADC 硬件 → 电压换算 → 角度 → 位置判定
 *          仅用于台架调试，不参与生产逻辑
 */
#include <rtthread.h>
#include <finsh.h>
#include <stdlib.h>

#include "lift_adc.h"        /* lift_adc_read_raw / lift_adc_read_mv */
#include "lift_angle.h"      /* lift_angle_get_value / _get_position   */
#include "lift_control.h"    /* g_lift_obj（故障标志）                  */
#include "../hw_objects.h"   /* g_solenoid.valve_output（阀输出观测）   */
#include "../mode/sys_caps.h"         /* 能力位门禁（2026-10-27）        */
#include "../sys_param/sys_param.h"   /* ← 新增：sys_param_t / sys_param_get / _save */

/* ============================================================
 * 位置名
 * ============================================================ */
static const char *lift_pos_str(lift_position_t pos)
{
    switch (pos) {
    case LIFT_POS_UPPER_LIMIT: return "UPPER (+43.0deg)";
    case LIFT_POS_MIDDLE:      return "MIDDLE";
    case LIFT_POS_LOWER_LIMIT: return "LOWER (-30.0deg)";
    default:                    return "UNKNOWN";
    }
}

/* 控制对象状态名（t_lift 观测阀输出与限位拦截时用） */
static const char *lift_state_str(lift_state_t st)
{
    switch (st) {
    case P_STATE_IDLE:         return "IDLE";
    case P_STATE_UP_RUNNING:   return "UP_RUNNING";
    case P_STATE_DOWN_RUNNING: return "DOWN_RUNNING";
    case P_STATE_LIMIT_HOLDING:return "LIMIT_HOLDING";
    case P_STATE_FAULT_VALVE:  return "FAULT_VALVE";
    case P_STATE_FAULT_SENSOR: return "FAULT_SENSOR";
    default:                   return "?";
    }
}

/* ============================================================
 * t_lift —— 单次打印全链路状态
 * ============================================================ */
static void t_lift(int argc, char **argv)
{
    uint16_t raw;
    uint16_t mv;
    int32_t  angle;
    int32_t  a_abs;

    (void)argc; (void)argv;

    raw   = lift_adc_read_raw();
    mv    = lift_adc_read_mv(RT_NULL);
    angle = lift_angle_get_value(&g_lift_angle);
    a_abs = (angle < 0) ? -angle : angle;

    rt_kprintf("\n======== LIFT SENSOR CHECK ========\n");
    rt_kprintf("device  : %s  ch=%u\n",
               LIFT_ADC_DEV_NAME, (unsigned)LIFT_ADC_CHANNEL);
    rt_kprintf("raw     : %4u  (0x%03X, full=%u)\n",
               (unsigned)raw, (unsigned)raw, (unsigned)LIFT_ADC_RAW_MAX);
    rt_kprintf("voltage : %4u mV  (vref=%umV, div=%u:%u, over=%u)\n",
               (unsigned)mv, (unsigned)LIFT_ADC_VREF_MV,
               (unsigned)LIFT_ADC_DIV_NUM, (unsigned)LIFT_ADC_DIV_DEN,
               (unsigned)LIFT_ADC_OVERSAMPLE);
    rt_kprintf("angle   : %s%d.%d deg\n",
               (angle < 0) ? "-" : "", (int)(a_abs / 10), (int)(a_abs % 10));
    rt_kprintf("position: %s\n",
               lift_pos_str(lift_angle_get_position(&g_lift_angle)));
    rt_kprintf("state   : %s\n", lift_state_str(lift_control_get_state()));
    rt_kprintf("valve   : out=0x%02X  (bit0=up bit1=down)\n",
               (unsigned)g_solenoid.valve_output);
    rt_kprintf("fault   : sensor=%u valve_up=%u valve_down=%u\n",
               (unsigned)g_lift_obj.ctx.sensor_fault,
               (unsigned)g_lift_obj.ctx.valve_fault_up,
               (unsigned)g_lift_obj.ctx.valve_fault_down);
    /* 上两行是 LiftControl 的布尔镜像；下面这行是 0x18FF0E63 的原始故障码，
     * 只有它能区分"开路(1)/短路(2)"（状态帧 11D1 Byte2 也取这个）。 */
    rt_kprintf("fault_raw: up=%u dn=%u  (0=ok 1=open 2=short)\n",
               (unsigned)g_solenoid.up_fault,
               (unsigned)g_solenoid.down_fault);
    rt_kprintf("====================================\n\n");
}


/* ============================================================
 * t_lift_loop —— 周期打印，便于手动转动观察
 * 用法：t_lift_loop [次数]   默认 20 次，间隔 200ms
 * ============================================================ */
static void t_lift_loop(int argc, char **argv)
{
    int count = 20;
    int32_t a, aa;

    if (argc >= 2) {
        count = atoi(argv[1]);
        if (count <= 0) count = 20;
    }

    rt_kprintf("\n[Lift] loop x%d, 200ms interval\n", count);
    for (int i = 0; i < count; i++) {
        a  = lift_angle_get_value(&g_lift_angle);
        aa = (a < 0) ? -a : a;
        rt_kprintf("[%3d] raw=%4u  mv=%4u  angle=%s%d.%d\n",
                   i,
                   (unsigned)lift_adc_read_raw(),
                   (unsigned)lift_adc_read_mv(RT_NULL),
                   (a < 0) ? "-" : "", (int)(aa / 10), (int)(aa % 10));
        rt_thread_mdelay(200);
    }
    rt_kprintf("[Lift] loop done\n\n");
}

/* ============================================================
 * t_lift_calib —— 以当前电压为零位（把当前角度设为 0°）
 * ============================================================ */
static void t_lift_calib(int argc, char **argv)
{
    sys_param_t *p = sys_param_get();
    uint16_t mv;

    (void)argc; (void)argv;

    /* 【2026-10-27】校准写零位 = 改整机角度基准，工作模式必须禁掉。
     * 注意：这里是「旁路校准」通道，不消耗上电周期配额（配额属于
     * calib 模块的正式流程）；但能力位仍需通过。 */
    if (sys_cap_require(SYS_CAP_CALIB, "t_lift_calib") == 0U) { return; }

    mv = lift_adc_read_mv(RT_NULL);

    if (mv < LIFT_ADC_SIG_MIN_MV || mv > LIFT_ADC_SIG_MAX_MV) {
        rt_kprintf("[Calib] REJECT: %umV out of [%u, %u]mV\n",
                   (unsigned)mv,
                   (unsigned)LIFT_ADC_SIG_MIN_MV,
                   (unsigned)LIFT_ADC_SIG_MAX_MV);
        return;
    }

    p->lift_zero_voltage_mv = mv;
    p->is_calibrated         = 1U;
    sys_param_save();

    lift_angle_calibrate(&g_lift_angle, (int32_t)mv);

    rt_kprintf("[Calib] OK: zero=%umV, angle should be 0.0\n", (unsigned)mv);
}


/* ============================================================
 * t_lift_calib_clr —— 清除校准，回到 2203mV 默认
 * ============================================================ */
static void t_lift_calib_clr(int argc, char **argv)
{
    sys_param_t *p = sys_param_get();
    (void)argc; (void)argv;

    /* 清校准同样是基准变更，工作模式禁掉 */
    if (sys_cap_require(SYS_CAP_CALIB, "t_lift_calib_clr") == 0U) { return; }

    p->lift_zero_voltage_mv = LIFT_ADC_DEFAULT_ZERO_MV;
    p->is_calibrated         = 0U;
    sys_param_save();

    lift_angle_calibrate(&g_lift_angle, LIFT_ADC_DEFAULT_ZERO_MV);
    rt_kprintf("[Calib] cleared, back to default %umV\n",
               (unsigned)LIFT_ADC_DEFAULT_ZERO_MV);
}
MSH_CMD_EXPORT_ALIAS(t_lift_calib_clr, t_lift_calib_clr, clear calibration);
MSH_CMD_EXPORT_ALIAS(t_lift,           t_lift,           check lift sensor chain);
MSH_CMD_EXPORT_ALIAS(t_lift_loop,      t_lift_loop,      loop check lift sensor);
MSH_CMD_EXPORT_ALIAS(t_lift_calib,     t_lift_calib,     set current voltage as 0deg);
