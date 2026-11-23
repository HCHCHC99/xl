/**
 * @file    lift_test.c
 * @brief   举升角传感器自检命令（MSH）
 * @note    全链路验证：ADC 硬件 → 电压换算 → 角度 → 位置判定
 *          仅用于台架调试，不参与生产逻辑
 */
#include <rtthread.h>
#include <finsh.h>
#include <stdlib.h>
#include <string.h>

#include "lift_adc.h"        /* lift_adc_read_raw / lift_adc_read_mv */
#include "lift_angle.h"      /* lift_angle_get_value / _get_position   */
#include "lift_sensor.h"     /* P5：阈值与 2 s 去抖状态                */
#include "lift_control.h"    /* g_lift_obj（故障标志）                  */
#include "../hw_objects.h"   /* g_solenoid.valve_output（阀输出观测）   */
#include "../mode/sys_caps.h"         /* 能力位门禁（2026-10-27）        */
#include "../sys_param/sys_param.h"   /* ← 新增：sys_param_t / sys_param_get / _save */
#include "../service/input_sim.h"     /* t_valve_fb test：注入 0x18FF0E63 */

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
    /* 【P5 / B-6】传感器 2 s 去抖的中间态：latched=已确认故障，
     * timing=正在计时（进入或恢复），elapsed=已持续时长。
     * 故障时角度会冻结在上次可信值，angle_valid=0 才代表"这个角度不能用"。 */
    /* 【P6 / B-7】限位保持剩余时间（只在 LIMIT_HOLDING 有值） */
    {
        int32_t rem = lift_control_hold_remaining_ms();

        if (rem >= 0) {
            rt_kprintf("hold    : remaining=%ldms\n", (long)rem);
        }
    }
    if (g_lift_angle.sensor != RT_NULL) {
        rt_kprintf("sens    : latched=%u timing=%u elapsed=%lums angle_valid=%u\n",
                   (unsigned)g_lift_angle.sensor->fault_latched,
                   (unsigned)g_lift_angle.sensor->fault_timing,
                   (unsigned long)g_lift_angle.sensor->fault_elapsed_ms,
                   (unsigned)((lift_angle_get_position(&g_lift_angle) != LIFT_POS_UNKNOWN) ? 1U : 0U));
    }
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
/* ============================================================
 * t_valve_fb —— 阀故障 500 ms 去抖（P4 / B-5）
 * ------------------------------------------------------------
 * 背景：0x18FF0E63 的故障码是"当前状态"而非"事件"。原实现一帧非 0 就进
 *       FAULT_VALVE：台架敲一次 t_sim valve 1 0 即锁死，真机上偶发误码
 *       会让正在上升的卸粮筒瞬间失电。本命令用于观测去抖过程、切换新旧
 *       行为做对比，并自动跑一次时序自检。
 *
 * 用法：
 *   t_valve_fb                 列出用法与当前配置
 *   t_valve_fb on|off          去抖开 / 关（off = 旧行为，一帧即报，用于对比）
 *   t_valve_fb ms <n>          确认门限 0..5000（纯 RAM，重启回 500）
 *   t_valve_fb stat            两行状态：配置 + 两路原始码/计时/已确认
 *   t_valve_fb test            自动时序自检（约 2 s，阻塞；会短暂进 FAULT_VALVE）
 * ============================================================ */

#if defined(INPUT_SIM_ENABLE) && (INPUT_SIM_ENABLE == 1)
#define VFB_SIM_OK   1
#else
#define VFB_SIM_OK   0
#endif

static void vfb_usage(void)
{
    lift_vfb_state_t st;

    lift_control_valve_fb_get(&st);
    rt_kprintf("[VFB] usage: t_valve_fb on|off | ms <0-5000> | stat | test\n");
    rt_kprintf("[VFB] now: deb=%u ms=%u  (sim=%s)\n",
               (unsigned)st.debounce_en, (unsigned)st.confirm_ms,
               (VFB_SIM_OK != 0) ? "on" : "off");
}

/* 注入 0x18FF0E63：up/dn 为原始故障码 0=正常 1=开路 2=短路 */
static void vfb_inject(uint8_t up, uint8_t dn)
{
#if (VFB_SIM_OK == 1)
    uint32_t packed = (uint32_t)(up & 0x3U) | ((uint32_t)(dn & 0x3U) << 2);

    (void)input_sim_set(INPUT_SIM_VALVE, (int32_t)packed);
#else
    (void)up;
    (void)dn;
#endif
}

static uint32_t vfb_elapsed_ms(uint32_t t0)
{
    return ((uint32_t)(rt_tick_get() - t0) * 1000U) / RT_TICK_PER_SECOND;
}

static void vfb_stat(void)
{
    lift_vfb_state_t st;

    lift_control_valve_fb_get(&st);
    rt_kprintf("[VFB] deb=%u ms=%u  state=%s  out=0x%02X\n",
               (unsigned)st.debounce_en, (unsigned)st.confirm_ms,
               lift_state_str(lift_control_get_state()),
               (unsigned)g_solenoid.valve_output);
    rt_kprintf("[VFB] up raw=%u lat=%u tim=%u %lums | dn raw=%u lat=%u tim=%u %lums\n",
               (unsigned)st.ch[0].raw, (unsigned)st.ch[0].latched,
               (unsigned)st.ch[0].timing, (unsigned long)st.ch[0].elapsed_ms,
               (unsigned)st.ch[1].raw, (unsigned)st.ch[1].latched,
               (unsigned)st.ch[1].timing, (unsigned long)st.ch[1].elapsed_ms);
}

static void vfb_test(void)
{
    lift_vfb_state_t st;
    uint32_t t0, dt, limit;
    uint8_t  ok = 1U;

#if (VFB_SIM_OK != 0)
    lift_control_valve_fb_get(&st);
    limit = (uint32_t)st.confirm_ms + 1000U;

    rt_kprintf("[VFB] test start: deb=%u confirm=%ums (tolerance +50ms)\n",
               (unsigned)st.debounce_en, (unsigned)st.confirm_ms);
    if (st.debounce_en == 0U) {
        rt_kprintf("[VFB] WARN: debounce is OFF, 400ms case will fail by design\n");
    }

    /* --- 0. 清故障，等确认清除 --- */
    vfb_inject(0U, 0U);
    rt_thread_mdelay(300);

    /* --- 1. 400 ms 内不得置位 --- */
    t0 = rt_tick_get();
    vfb_inject(1U, 0U);
    while (1) {
        dt = vfb_elapsed_ms(t0);
        lift_control_valve_fb_get(&st);
        if (st.ch[0].latched != 0U) {
            rt_kprintf("[VFB] FAIL: latched at %lums, earlier than 400ms\n",
                       (unsigned long)dt);
            ok = 0U;
            break;
        }
        if (dt >= 400U) {
            /* 打印墙钟时间 dt（从注入算起），不是 st.elapsed_ms：
             * elapsed 由 50 ms poll 刷新，会比 dt 最多少一拍，容易误读成"计时偏小"。 */
            rt_kprintf("[VFB] 400ms: not latched  OK (dt=%lums, debounce=%lums)\n",
                       (unsigned long)dt, (unsigned long)st.ch[0].elapsed_ms);
            break;
        }
        rt_thread_mdelay(10);
    }

    /* --- 2. 等置位，打印实测确认时间 --- */
    while (ok != 0U) {
        dt = vfb_elapsed_ms(t0);
        lift_control_valve_fb_get(&st);
        if (st.ch[0].latched != 0U) {
            if ((dt >= 400U) && (dt <= (uint32_t)st.confirm_ms + 60U)) {
                rt_kprintf("[VFB] latched at %lums  OK (expect %u~%ums)\n",
                           (unsigned long)dt, (unsigned)st.confirm_ms,
                           (unsigned)st.confirm_ms + 50U);
            } else {
                rt_kprintf("[VFB] FAIL: latched at %lums, out of window\n",
                           (unsigned long)dt);
                ok = 0U;
            }
            break;
        }
        if (dt >= limit) {
            rt_kprintf("[VFB] FAIL: no latch within %lums\n", (unsigned long)limit);
            ok = 0U;
            break;
        }
        rt_thread_mdelay(10);
    }

    /* --- 3. 清除：故障码消失应立即恢复 --- */
    if (ok != 0U) {
        t0 = rt_tick_get();
        vfb_inject(0U, 0U);
        while (1) {
            dt = vfb_elapsed_ms(t0);
            lift_control_valve_fb_get(&st);
            if (st.ch[0].latched == 0U) {
                rt_kprintf("[VFB] cleared at %lums  OK\n", (unsigned long)dt);
                break;
            }
            if (dt >= 1000U) {
                rt_kprintf("[VFB] FAIL: not cleared within 1000ms\n");
                ok = 0U;
                break;
            }
            rt_thread_mdelay(10);
        }
    }

    vfb_inject(0U, 0U);     /* 收尾：不留故障 */
    rt_kprintf("[VFB] test %s | state=%s out=0x%02X\n",
               (ok != 0U) ? "PASS" : "FAIL",
               lift_state_str(lift_control_get_state()),
               (unsigned)g_solenoid.valve_output);
#else
    (void)st; (void)t0; (void)dt; (void)limit; (void)ok;
    rt_kprintf("[VFB] test skipped: input_sim disabled (INPUT_SIM_ENABLE=0)\n");
#endif
}

static void t_valve_fb(int argc, char **argv)
{
    if (argc < 2) {
        vfb_usage();
        return;
    }

    if (strcmp(argv[1], "stat") == 0) {
        if (sys_cap_require(SYS_CAP_DIAG_READ, "t_valve_fb stat") == 0U) { return; }
        vfb_stat();
        return;
    }

    if (strcmp(argv[1], "test") == 0) {
        if (sys_cap_require(SYS_CAP_DIAG_WRITE, "t_valve_fb test") == 0U) { return; }
        vfb_test();
        return;
    }

    if (strcmp(argv[1], "on") == 0) {
        lift_vfb_state_t st;
        if (sys_cap_require(SYS_CAP_DIAG_WRITE, "t_valve_fb on") == 0U) { return; }
        lift_control_valve_fb_get(&st);
        lift_control_valve_fb_set(1U, st.confirm_ms);
        rt_kprintf("[VFB] debounce -> ON (%ums confirm)\n", (unsigned)st.confirm_ms);
        return;
    }

    if (strcmp(argv[1], "off") == 0) {
        lift_vfb_state_t st;
        if (sys_cap_require(SYS_CAP_DIAG_WRITE, "t_valve_fb off") == 0U) { return; }
        lift_control_valve_fb_get(&st);
        lift_control_valve_fb_set(0U, st.confirm_ms);
        rt_kprintf("[VFB] debounce -> OFF (legacy: single frame faults)\n");
        return;
    }

    if ((strcmp(argv[1], "ms") == 0) && (argc >= 3)) {
        lift_vfb_state_t st;
        int32_t v = atoi(argv[2]);

        if (sys_cap_require(SYS_CAP_DIAG_WRITE, "t_valve_fb ms") == 0U) { return; }
        if ((v < 0) || (v > 5000)) {
            rt_kprintf("[VFB] usage: t_valve_fb ms <0-5000>\n");
            return;
        }
        lift_control_valve_fb_get(&st);
        lift_control_valve_fb_set(st.debounce_en, (uint16_t)v);
        rt_kprintf("[VFB] confirm_ms -> %ld (RAM only)\n", (long)v);
        return;
    }

    vfb_usage();
}
MSH_CMD_EXPORT_ALIAS(t_valve_fb, t_valve_fb, valve fault 500ms debounce: on|off|ms|stat|test);

/* ============================================================
 * t_lift_sens —— 举升传感器 2 s 确认/恢复（P5 / B-6）
 * ------------------------------------------------------------
 * 用例 TC-FLT-001~004 要求：4800 / 4400 / 250 / 520 mV 各持续 2.5 s，
 * 故障位在 2 s 后置位 / 恢复。用 1 s 步进的 t_seq 量不准，本命令用
 * 10 ms 采样直接测出确认耗时。
 *
 * 用法：
 *   t_lift_sens          打印阈值与当前状态
 *   t_lift_sens test     自动跑四段（约 9 s，阻塞）
 * ============================================================ */
static uint32_t lifs_ms(uint32_t t0)
{
    return ((uint32_t)(rt_tick_get() - t0) * 1000U) / RT_TICK_PER_SECOND;
}

/* 等 fault_latched 变成 want。
 * early_ms > 0 时，若在 early_ms 之前就变成 want，返回 -2（提前误动）。
 * 返回 0 成功（*elapsed=实测耗时） / -1 超时 / -2 提前 / -3 传感器不可用 */
static int lifs_wait(uint8_t want, uint32_t early_ms, uint32_t cap_ms, uint32_t *elapsed)
{
    lift_sensor_t *s = g_lift_angle.sensor;
    uint32_t t0;
    uint32_t dt;

    if (s == RT_NULL) {
        return -3;
    }
    t0 = rt_tick_get();

    for (;;) {
        dt = lifs_ms(t0);
        if (s->fault_latched == want) {
            if ((early_ms != 0U) && (dt < early_ms)) {
                *elapsed = dt;
                return -2;
            }
            *elapsed = dt;
            return 0;
        }
        if (dt >= cap_ms) {
            *elapsed = dt;
            return -1;
        }
        rt_thread_mdelay(10);
    }
}

static void lifs_usage(void)
{
    lift_sensor_t *s = g_lift_angle.sensor;

    rt_kprintf("[LSENS] usage: t_lift_sens | test\n");
    if ((s == RT_NULL) || (s->type != LIFT_SENSOR_TYPE_VOLTAGE)) {
        rt_kprintf("[LSENS] sensor not voltage type\n");
        return;
    }
    {
        lift_sensor_voltage_cfg_t *cfg = (lift_sensor_voltage_cfg_t *)s->data;

        rt_kprintf("[LSENS] thresh: hi %u/%u lo %u/%u confirm=%ums recover=%ums\n",
                   (unsigned)cfg->fault_high_mv, (unsigned)cfg->fault_high_exit_mv,
                   (unsigned)cfg->fault_low_mv,  (unsigned)cfg->fault_low_exit_mv,
                   (unsigned)cfg->confirm_ms,    (unsigned)cfg->recover_ms);
        rt_kprintf("[LSENS] now: mv=%umV latched=%u timing=%u elapsed=%lums\n",
                   (unsigned)cfg->last_voltage_mv,
                   (unsigned)s->fault_latched, (unsigned)s->fault_timing,
                   (unsigned long)s->fault_elapsed_ms);
    }
}

static void lifs_test(void)
{
    lift_sensor_t *s = g_lift_angle.sensor;
    lift_sensor_voltage_cfg_t *cfg;
    uint32_t conf, rec, cap, dt;
    uint8_t  ok = 1U;
    int      r;

    if ((s == RT_NULL) || (s->type != LIFT_SENSOR_TYPE_VOLTAGE)) {
        rt_kprintf("[LSENS] test skipped: no voltage sensor\n");
        return;
    }
    cfg  = (lift_sensor_voltage_cfg_t *)s->data;
    conf = cfg->confirm_ms;
    rec  = cfg->recover_ms;
    cap  = conf + 1500U;            /* 门限 + 1.5 s 余量 */

    rt_kprintf("[LSENS] test start: confirm=%lums recover=%lums (early gate 1500ms)\n",
               (unsigned long)conf, (unsigned long)rec);

    /* 归零，确保从"正常"起步 */
    (void)input_sim_set(INPUT_SIM_LIFT, 2203);
    rt_thread_mdelay(300);

    /* --- 1) 高限进入 --- */
    (void)input_sim_set(INPUT_SIM_LIFT, cfg->fault_high_mv);
    r = lifs_wait(1U, 1500U, cap, &dt);
    if (r == 0) {
        if ((dt >= conf) && (dt <= conf + 70U)) {
            rt_kprintf("[LSENS] hi enter at %lums  OK (expect %lu~%lums)\n",
                       (unsigned long)dt, (unsigned long)conf, (unsigned long)(conf + 50U));
        } else {
            rt_kprintf("[LSENS] FAIL: hi enter at %lums, out of window\n", (unsigned long)dt);
            ok = 0U;
        }
    } else if (r == -2) {
        rt_kprintf("[LSENS] FAIL: hi latched at %lums, earlier than 1500ms\n", (unsigned long)dt);
        ok = 0U;
    } else {
        rt_kprintf("[LSENS] FAIL: hi enter timeout\n");
        ok = 0U;
    }

    /* --- 2) 高限恢复 --- */
    if (ok != 0U) {
        (void)input_sim_set(INPUT_SIM_LIFT, cfg->fault_high_exit_mv);
        r = lifs_wait(0U, 1500U, cap, &dt);
        if (r == 0) {
            if ((dt >= rec) && (dt <= rec + 70U)) {
                rt_kprintf("[LSENS] hi exit  at %lums  OK (expect %lu~%lums)\n",
                           (unsigned long)dt, (unsigned long)rec, (unsigned long)(rec + 50U));
            } else {
                rt_kprintf("[LSENS] FAIL: hi exit at %lums, out of window\n", (unsigned long)dt);
                ok = 0U;
            }
        } else {
            rt_kprintf("[LSENS] FAIL: hi exit %s\n", (r == -2) ? "too early" : "timeout");
            ok = 0U;
        }
    }

    /* --- 3) 低限进入 --- */
    if (ok != 0U) {
        (void)input_sim_set(INPUT_SIM_LIFT, cfg->fault_low_mv);
        r = lifs_wait(1U, 1500U, cap, &dt);
        if (r == 0) {
            rt_kprintf("[LSENS] lo enter at %lums  %s\n", (unsigned long)dt,
                       ((dt >= conf) && (dt <= conf + 70U)) ? "OK" : "FAIL(out of window)");
            if ((dt < conf) || (dt > conf + 70U)) { ok = 0U; }
        } else {
            rt_kprintf("[LSENS] FAIL: lo enter %s\n", (r == -2) ? "too early" : "timeout");
            ok = 0U;
        }
    }

    /* --- 4) 低限恢复 --- */
    if (ok != 0U) {
        (void)input_sim_set(INPUT_SIM_LIFT, cfg->fault_low_exit_mv);
        r = lifs_wait(0U, 1500U, cap, &dt);
        if (r == 0) {
            rt_kprintf("[LSENS] lo exit  at %lums  %s\n", (unsigned long)dt,
                       ((dt >= rec) && (dt <= rec + 70U)) ? "OK" : "FAIL(out of window)");
            if ((dt < rec) || (dt > rec + 70U)) { ok = 0U; }
        } else {
            rt_kprintf("[LSENS] FAIL: lo exit %s\n", (r == -2) ? "too early" : "timeout");
            ok = 0U;
        }
    }

    /* 收尾：回正常电压 */
    (void)input_sim_set(INPUT_SIM_LIFT, 2203);
    rt_thread_mdelay(100);
    rt_kprintf("[LSENS] test %s | state=%s angle_valid=%u latched=%u\n",
               (ok != 0U) ? "PASS" : "FAIL",
               lift_state_str(lift_control_get_state()),
               (unsigned)((lift_angle_get_position(&g_lift_angle) != LIFT_POS_UNKNOWN) ? 1U : 0U),
               (unsigned)s->fault_latched);
}

static void t_lift_sens(int argc, char **argv)
{
    if (argc < 2) {
        lifs_usage();
        return;
    }
    if (strcmp(argv[1], "test") == 0) {
        if (sys_cap_require(SYS_CAP_DIAG_WRITE, "t_lift_sens test") == 0U) { return; }
        lifs_test();
        return;
    }
    lifs_usage();
}
MSH_CMD_EXPORT_ALIAS(t_lift_sens, t_lift_sens, lift sensor 2s confirm/recover: |test);

MSH_CMD_EXPORT_ALIAS(t_lift_calib_clr, t_lift_calib_clr, clear calibration);
MSH_CMD_EXPORT_ALIAS(t_lift,           t_lift,           check lift sensor chain);
MSH_CMD_EXPORT_ALIAS(t_lift_loop,      t_lift_loop,      loop check lift sensor);
MSH_CMD_EXPORT_ALIAS(t_lift_calib,     t_lift_calib,     set current voltage as 0deg);
