/**
 * @file    sys_config.c
 * @brief   系统配置表实现 —— 默认值优先、逐项校验、不做静默失败
 * @note    设计文档：docs/02-设计/方案-系统配置表sys_config-V1.0.md
 */

#include "sys_config.h"

#include "../param/param_store.h"
#include "../sys_param/sys_param.h"
#include "../angle/mech_cfg_store.h"

#include <string.h>
#include <rtthread.h>

/* ============================================================
 * 1. 参数定义表 —— 默认值硬编码于此，是永久兜底
 * ------------------------------------------------------------
 * 【必须遵守】新增参数只能"追加"，不得修改已有 ID 的含义或范围。
 *            改含义会让现场已存值变成"合法但错误"的数据，
 *            这种错误范围校验抓不到，比越界更危险。
 * ============================================================ */
static const cfg_def_t s_cfg_def[] = {
    /* ---- 0x01xx 整车状态（原 vehicle_state.c 的 const 表） ---- */
    { CFG_ID_VEH_ENTER_RPM,     "veh_enter_rpm",     CFG_T_U16, 0,
      400,   0, 3000,  "rpm" },
    { CFG_ID_VEH_EXIT_RPM,      "veh_exit_rpm",      CFG_T_U16, 0,
      350,   0, 3000,  "rpm" },
    { CFG_ID_VEH_ENTER_VOLT_DV, "veh_enter_volt_dv", CFG_T_U16, 0,
      130, 100,  200,  "0.1V" },
    { CFG_ID_VEH_EXIT_VOLT_DV,  "veh_exit_volt_dv",  CFG_T_U16, 0,
      125, 100,  200,  "0.1V" },
    { CFG_ID_VEH_ENTER_MS,      "veh_enter_ms",      CFG_T_U16, 0,
      500,   0, 5000,  "ms" },
    { CFG_ID_VEH_EXIT_MS,       "veh_exit_ms",       CFG_T_U16, 0,
      1000,  0, 10000, "ms" },

    /* ---- 0x02xx 测速 ----
     * speed_window_ticks 是热路径常量，P1 只做加载与展示，
     * P3 才讨论是否开放在线修改（见设计文档 R2）。 */
    { CFG_ID_SPEED_WINDOW_TICKS, "speed_window_ticks", CFG_T_U16,
      CFG_F_REQ_RESTART | CFG_F_ADVANCED,
      100,  20, 1000, "50us" },
    { CFG_ID_SPEED_TICK_HZ,      "speed_tick_hz",      CFG_T_U32,
      CFG_F_READONLY,
      20000, 1000, 50000, "Hz" },
    { CFG_ID_SPEED_DT_MAX_RATIO, "speed_dt_max_ratio", CFG_T_U8,
      CFG_F_ADVANCED,
      3,    1, 10, "x window" },

    /* ---- 0x03xx 看门狗 ----
     * 现状 s_checkin_enable 在 RAM，t_wdt_on 改了掉电即失。
     * 接入本表后才能真正持久化。 */
    { CFG_ID_WDT_CHECKIN_ENABLE, "wdt_checkin_enable", CFG_T_U8,
      0,
      0, 0, 1, "" },
    { CFG_ID_WDT_TIMEOUT_MS,     "wdt_timeout_ms",     CFG_T_U16,
      0,
      1000, 100, 10000, "ms" },
    { CFG_ID_WDT_IDLE_HOOK,      "wdt_idle_hook",      CFG_T_U8,
      CFG_F_REQ_RESTART,
      1, 0, 1, "" },

    /* ---- 0x04xx 电流采样 ----
     * Q7 裁定：不考虑节约空间，用纳伏保证精度。
     * 默认值刻意等于现有宏（FOC_CUR_ZERO_RAW=2048、0.80566mV、26.4mV/A），
     * 保证接入后行为与接入前完全一致（R3 风险对策）。 */
    { CFG_ID_CUR_ZERO_BUS,   "cur_zero_bus",   CFG_T_U16, 0,
      2048, 1500, 2600, "raw" },
    { CFG_ID_CUR_ZERO_W,     "cur_zero_w",     CFG_T_U16, 0,
      2048, 1500, 2600, "raw" },
    { CFG_ID_CUR_ZERO_V,     "cur_zero_v",     CFG_T_U16, 0,
      2048, 1500, 2600, "raw" },
    /* 0.80566 mV = 805660 nV。用 U32 存纳伏，精度无损 */
    { CFG_ID_CUR_LSB_NV,     "cur_lsb_nv",     CFG_T_U32, 0,
      805660, 100000, 5000000, "nV" },
    /* 26.4 mV/A x100 = 2640 */
    { CFG_ID_CUR_GAIN_X100,  "cur_gain_x100",  CFG_T_U16, 0,
      2640, 1000, 5000, "0.01mV/A" },

    /* ---- 0x05xx PWM / 电机 ----
     * P1 只定义不接入；接入时须评估过流/斜坡属安全相关（R3）。 */
    { CFG_ID_PWM_FREQ_HZ,      "pwm_freq_hz",      CFG_T_U32,
      CFG_F_REQ_RESTART | CFG_F_ADVANCED,
      20000, 5000, 40000, "Hz" },
    { CFG_ID_PWM_DEADTIME_NS,  "pwm_deadtime_ns",  CFG_T_U16,
      CFG_F_REQ_RESTART | CFG_F_ADVANCED,
      1000, 200, 5000, "ns" },
    { CFG_ID_MOTOR_POLE_PAIRS, "motor_pole_pairs", CFG_T_U8,
      CFG_F_REQ_RESTART,
      4, 1, 16, "pairs" },
    { CFG_ID_MOTOR_OC_L1_MA,   "motor_oc_l1_ma",   CFG_T_U16,
      CFG_F_ADVANCED,
      0, 0, 60000, "mA" },
    { CFG_ID_MOTOR_OC_L2_MA,   "motor_oc_l2_ma",   CFG_T_U16,
      CFG_F_ADVANCED,
      0, 0, 60000, "mA" },
    { CFG_ID_MOTOR_OC_L3_MA,   "motor_oc_l3_ma",   CFG_T_U16,
      CFG_F_ADVANCED,
      0, 0, 60000, "mA" },
    { CFG_ID_MOTOR_RAMP_MS,    "motor_ramp_ms",    CFG_T_U16,
      CFG_F_ADVANCED,
      0, 0, 10000, "ms" },
    /* 【2026-11-04 新增】SVPWM 最大调制度（x1000）
     * 用途：约束零矢量 111 段宽度，保证三路 ADC 有足够窗口采集
     *      （实测三路转换 4μs，111 段宽 = 50μs × min(du,dv,dw)）。
     * 默认 1000（100%）= 不限制，由用户实测整定后收紧。 */
    { CFG_ID_FOC_MAX_MODULATION, "foc_max_modul_x1000", CFG_T_U16,
      CFG_F_ADVANCED,
      1000, 500, 1000, "x1000" },

    /* ---- 0x06xx 举升 ----
     * 真值仍在 lift_angle.c 内部；此处先定义占位，
     * 接入时再决定是否替换硬编码。P1 不改业务。
     *
     * 【2026-11-04 修正 0x0602 默认值 -430 改为 -300】
     * 原默认值 -430 与实际代码不符，三处一致为 -30.0 度：
     *   lift_angle.c:89   lower_limit_x10 = -300   (-30.0 deg)
     *   lift_angle.h:27   LIFT_POS_LOWER_LIMIT    (-30 deg +/-2)
     *   lift_angle.h:69   int32_t lower_limit_x10  (-300, -30.0 deg)
     * 只有本表写成 -430，疑似把上限 430 抄错。
     *
     * 为什么必须改：一旦接入 lift 模块用本表覆盖，
     * 下限位会从 -30 度放宽到 -43 度，机械臂可能撞限位。
     * 当前无业务消费该项，改默认值不影响运行，但能消除隐患。 */
    { CFG_ID_LIFT_UPPER_X10, "lift_upper_x10", CFG_T_I16, 0,
      430, -900, 900, "0.1deg" },
    { CFG_ID_LIFT_LOWER_X10, "lift_lower_x10", CFG_T_I16, 0,
      -300, -900, 900, "0.1deg" },

    /* ---- 0x07xx 标定标志（镜像，只读）----
     * 真值在 sys_param，本表只提供统一查询入口，绝不存副本。
     * 改标定必须走 t_enc_cal / t_lift_calib（Q8 裁定）。 */
    { CFG_ID_ENC_ZERO_VALID,    "enc_zero_valid",    CFG_T_U8,
      CFG_F_READONLY, 0, 0, 1, "" },
    { CFG_ID_LIFT_CALIB_VALID, "lift_calib_valid", CFG_T_U8,
      CFG_F_READONLY, 0, 0, 1, "" },
    { CFG_ID_CALIB_COUNT,       "calib_count",       CFG_T_U16,
      CFG_F_READONLY, 0, 0, 65535, "times" },
    { CFG_ID_CALIB_LAST_TICK,   "calib_last_tick",   CFG_T_U32,
      CFG_F_READONLY, 0, 0, 2147483647, "tick" },

    /* ---- 0x08xx 齿数镜像（镜像 mech_cfg，只读）----
     * 真值在 mech_param，经 t_mech_cand 修改。
     * 【红线】本表不存副本，绝不作为角度解算的输入源。 */
    { CFG_ID_MECH_TEETH_Z0,   "mech_teeth_z0",   CFG_T_U16,
      CFG_F_READONLY, 55, 0, 65535, "teeth" },
    { CFG_ID_MECH_TEETH_Z1,   "mech_teeth_z1",   CFG_T_U16,
      CFG_F_READONLY, 56, 0, 65535, "teeth" },
    { CFG_ID_MECH_TEETH_Z2,   "mech_teeth_z2",   CFG_T_U16,
      CFG_F_READONLY, 57, 0, 65535, "teeth" },
    { CFG_ID_MECH_ACTIVE_IDX, "mech_active_idx", CFG_T_U8,
      CFG_F_READONLY, 0, 0, 255, "" },

    /* ---- 0x09xx 堵转保护（2026-11-04 新增） ----
     *
     * 【判据】四条 OR，任一满足即触发：
     *   ① 电流 >= 55.0A 持续 10ms
     *   ② 电流 >= 50.0A 持续 100ms
     *   ③ 电流 >= 45.0A 持续 500ms
     *   ④ 电机轴转速 < 36rpm 持续 2.5s
     *
     * 【默认值与被替换的宏完全一致】
     *   board_motor_cfg.h:91-98
     *     MOTOR_RATED_CURRENT_A10 370 / OC1 450+500ms / OC2 500+100ms / OC3 550+10ms
     *   接入后行为与接入前一致（R3 风险对策）。
     *
     * 【不加 CFG_F_REQ_RESTART】
     *   堵转判据需在线调参立即生效，motor_stall 每次 update 读快照。
     *
     * 【范围校验的意义】
     *   下限防止误设成 0（等于关闭保护）；
     *   上限 620（62A）略低于量程极限 62.5A，留出饱和检测余量。 */
    { CFG_ID_STALL_OC3_A10,      "stall_oc3_a10",      CFG_T_U16,
      CFG_F_ADVANCED, 550, 100, 620, "0.1A" },
    { CFG_ID_STALL_OC3_MS,       "stall_oc3_ms",       CFG_T_U16,
      CFG_F_ADVANCED, 10,  1, 1000, "ms" },
    { CFG_ID_STALL_OC2_A10,      "stall_oc2_a10",      CFG_T_U16,
      CFG_F_ADVANCED, 500, 100, 620, "0.1A" },
    { CFG_ID_STALL_OC2_MS,       "stall_oc2_ms",       CFG_T_U16,
      CFG_F_ADVANCED, 100, 1, 5000, "ms" },
    { CFG_ID_STALL_OC1_A10,      "stall_oc1_a10",      CFG_T_U16,
      CFG_F_ADVANCED, 450, 100, 620, "0.1A" },
    { CFG_ID_STALL_OC1_MS,       "stall_oc1_ms",       CFG_T_U16,
      CFG_F_ADVANCED, 500, 1, 10000, "ms" },
    { CFG_ID_STALL_LOW_RPM,      "stall_low_rpm",      CFG_T_U16,
      CFG_F_ADVANCED, 36,  5, 500, "rpm" },
    { CFG_ID_STALL_LOW_MS,       "stall_low_ms",       CFG_T_U16,
      CFG_F_ADVANCED, 2500, 100, 30000, "ms" },
    /* 转速判据的启用门控：电流必须大于此值才判低速，
     * 防止"空载 + 低速"被误判成堵转。 */
    { CFG_ID_STALL_EN_MIN_A10,   "stall_en_min_a10",   CFG_T_U16,
      CFG_F_ADVANCED, 50,  0, 620, "0.1A" },
    { CFG_ID_MOTOR_RATED_A10,    "motor_rated_a10",    CFG_T_U16,
      0, 370, 50, 620, "0.1A" },
    /* 松开手柄后清除故障的延时。0 = 立即清除（用户裁定全自动）。 */
    { CFG_ID_STALL_AUTO_CLEAR_MS, "stall_auto_clear_ms", CFG_T_U16,
      CFG_F_ADVANCED, 0, 0, 5000, "ms" },

    /* ---- 0x0Axx 机械传动选择（2026-11-04 新增）----
     *
     * 三项正交，映射见 sys_config.h 的注释。默认值 = 现场实测通过的组合：
     *   55:56:57  +  216  +  65:13
     * 该组合 i_total = 216×65/13 = 1080，与旧值 i_out=5/1 数值完全一致，
     * ⇒ 接入本表后行为与接入前一位不差（R3 风险对策）。
     *
     * 【范围校验的意义】
     *   上限卡在各自 SEL_COUNT-1，防止 Flash 里出现 3/2/4 这类
     *   越界选择子。越界会在上电时被判为无效项并回退默认（逐项回退），
     *   而不是让 mech_cfg 选到一个不存在的候选。 */
    { CFG_ID_MECH_ENC_SEL,  "mech_enc_sel",  CFG_T_U8,
      CFG_F_REQ_RESTART, 1, 0, 1, "0=41:42:43 1=55:56:57" },
    { CFG_ID_MECH_IIN_SEL,  "mech_iin_sel",  CFG_T_U8,
      CFG_F_REQ_RESTART, 1, 0, 1, "0=136.6 1=216" },
    { CFG_ID_MECH_IOUT_SEL, "mech_iout_sel", CFG_T_U8,
      CFG_F_REQ_RESTART, 0, 0, 2, "0=13T 1=14T 2=11T" },

    /* 【2026-11-09】电流零点校准未新增配置项 ——
     * 复用 0x0401~0x0403 的 cur_zero_bus/w/v（见该段注释）。
     * 避免两套零点并存导致"哪个生效"分不清。 */

    /* ---- 回转方向极性（0x0B 段，2026-11-09 新增）----
     *
     * 0 = 正逻辑（默认）：左转→角度减小，右转→角度增大
     * 1 = 负逻辑        ：左转→角度增大，右转→角度减小
     *
     * flags = CFG_F_REQ_RESTART：极性翻转会让正在转动的电机突然反向，
     *   必须重启生效，不能热改。
     *
     * 【现场判定方法】
     *   ① 上电看 [SWING] dir polarity 打印
     *   ② 手动按住右转，观察角度
     *      → 角度增大：保持 0
     *      → 角度减小：改为 1 后重启
     *      → 角度不动：角度链路故障，与极性无关（见 MARK-13） */
    { CFG_ID_SWING_DIR_POLARITY, "swing_dir_polarity", CFG_T_U8,
      CFG_F_REQ_RESTART, 0, 0, 1, "0=pos L-,R+ 1=neg L+,R-" },

    /* ============================================================
     * 0x0C 段：三环控制模块（2026-11-10 新增）
     * ------------------------------------------------------------
     * ID = 0x0C[实例0~3][参数0~12]，4 实例 × 13 项 = 52 项
     * 编码规则与参数含义见 ctrl_loop_cfg.h
     *
     * 【为什么 flags 多用 CFG_F_ADVANCED】
     *   PID 系数属"整定量"，不该出现在 t_cfg list 默认列表里，
     *   避免现场误改导致控制环失稳。要改时用
     *   t_cfg list all 或直接 cl set 命令。
     *
     * 【为什么实例 0 的 en 用 CFG_F_REQ_RESTART】
     *   总开关翻转会让控制环立刻接管/放手输出，
     *   运行中切换会造成输出跳变，必须重启生效。
     * ============================================================ */
    /* ---- 实例 0 ---- */
    { 0x0C00U, "cl0_en", CFG_T_U8,
      CFG_F_REQ_RESTART, 0, 0, 1, "0=off 1=on" },
    { 0x0C01U, "cl0_pos_en", CFG_T_U8,
      CFG_F_ADVANCED, 0, 0, 1, "0=off 1=on" },
    { 0x0C02U, "cl0_spd_en", CFG_T_U8,
      CFG_F_ADVANCED, 0, 0, 1, "0=off 1=on" },
    { 0x0C03U, "cl0_cur_en", CFG_T_U8,
      CFG_F_ADVANCED, 0, 0, 1, "0=off 1=on" },
    { 0x0C04U, "cl0_pos_period", CFG_T_U16,
      CFG_F_ADVANCED, 5, 1, 100, "ms" },
    { 0x0C05U, "cl0_spd_period", CFG_T_U16,
      CFG_F_ADVANCED, 1, 1, 100, "ms" },
    { 0x0C06U, "cl0_pos_kp", CFG_T_I32,
      CFG_F_ADVANCED, 0, 0, 100000, "x100" },
    { 0x0C07U, "cl0_pos_ki", CFG_T_I32,
      CFG_F_ADVANCED, 0, 0, 100000, "x100" },
    { 0x0C08U, "cl0_pos_kd", CFG_T_I32,
      CFG_F_ADVANCED, 0, 0, 100000, "x100" },
    { 0x0C09U, "cl0_spd_kp", CFG_T_I32,
      CFG_F_ADVANCED, 0, 0, 100000, "x100" },
    { 0x0C0AU, "cl0_spd_ki", CFG_T_I32,
      CFG_F_ADVANCED, 0, 0, 100000, "x100" },
    { 0x0C0BU, "cl0_spd_kd", CFG_T_I32,
      CFG_F_ADVANCED, 0, 0, 100000, "x100" },
    { 0x0C0CU, "cl0_cur_kp", CFG_T_I32,
      CFG_F_ADVANCED, 0, 0, 100000, "x100" },
    /* ---- 实例 1 ---- */
    { 0x0C10U, "cl1_en", CFG_T_U8,
      CFG_F_REQ_RESTART, 0, 0, 1, "0=off 1=on" },
    { 0x0C11U, "cl1_pos_en", CFG_T_U8,
      CFG_F_ADVANCED, 0, 0, 1, "0=off 1=on" },
    { 0x0C12U, "cl1_spd_en", CFG_T_U8,
      CFG_F_ADVANCED, 0, 0, 1, "0=off 1=on" },
    { 0x0C13U, "cl1_cur_en", CFG_T_U8,
      CFG_F_ADVANCED, 0, 0, 1, "0=off 1=on" },
    { 0x0C14U, "cl1_pos_period", CFG_T_U16,
      CFG_F_ADVANCED, 5, 1, 100, "ms" },
    { 0x0C15U, "cl1_spd_period", CFG_T_U16,
      CFG_F_ADVANCED, 1, 1, 100, "ms" },
    { 0x0C16U, "cl1_pos_kp", CFG_T_I32,
      CFG_F_ADVANCED, 0, 0, 100000, "x100" },
    { 0x0C17U, "cl1_pos_ki", CFG_T_I32,
      CFG_F_ADVANCED, 0, 0, 100000, "x100" },
    { 0x0C18U, "cl1_pos_kd", CFG_T_I32,
      CFG_F_ADVANCED, 0, 0, 100000, "x100" },
    { 0x0C19U, "cl1_spd_kp", CFG_T_I32,
      CFG_F_ADVANCED, 0, 0, 100000, "x100" },
    { 0x0C1AU, "cl1_spd_ki", CFG_T_I32,
      CFG_F_ADVANCED, 0, 0, 100000, "x100" },
    { 0x0C1BU, "cl1_spd_kd", CFG_T_I32,
      CFG_F_ADVANCED, 0, 0, 100000, "x100" },
    { 0x0C1CU, "cl1_cur_kp", CFG_T_I32,
      CFG_F_ADVANCED, 0, 0, 100000, "x100" },
    /* ---- 实例 2 ---- */
    { 0x0C20U, "cl2_en", CFG_T_U8,
      CFG_F_REQ_RESTART, 0, 0, 1, "0=off 1=on" },
    { 0x0C21U, "cl2_pos_en", CFG_T_U8,
      CFG_F_ADVANCED, 0, 0, 1, "0=off 1=on" },
    { 0x0C22U, "cl2_spd_en", CFG_T_U8,
      CFG_F_ADVANCED, 0, 0, 1, "0=off 1=on" },
    { 0x0C23U, "cl2_cur_en", CFG_T_U8,
      CFG_F_ADVANCED, 0, 0, 1, "0=off 1=on" },
    { 0x0C24U, "cl2_pos_period", CFG_T_U16,
      CFG_F_ADVANCED, 5, 1, 100, "ms" },
    { 0x0C25U, "cl2_spd_period", CFG_T_U16,
      CFG_F_ADVANCED, 1, 1, 100, "ms" },
    { 0x0C26U, "cl2_pos_kp", CFG_T_I32,
      CFG_F_ADVANCED, 0, 0, 100000, "x100" },
    { 0x0C27U, "cl2_pos_ki", CFG_T_I32,
      CFG_F_ADVANCED, 0, 0, 100000, "x100" },
    { 0x0C28U, "cl2_pos_kd", CFG_T_I32,
      CFG_F_ADVANCED, 0, 0, 100000, "x100" },
    { 0x0C29U, "cl2_spd_kp", CFG_T_I32,
      CFG_F_ADVANCED, 0, 0, 100000, "x100" },
    { 0x0C2AU, "cl2_spd_ki", CFG_T_I32,
      CFG_F_ADVANCED, 0, 0, 100000, "x100" },
    { 0x0C2BU, "cl2_spd_kd", CFG_T_I32,
      CFG_F_ADVANCED, 0, 0, 100000, "x100" },
    { 0x0C2CU, "cl2_cur_kp", CFG_T_I32,
      CFG_F_ADVANCED, 0, 0, 100000, "x100" },
    /* ---- 实例 3 ---- */
    { 0x0C30U, "cl3_en", CFG_T_U8,
      CFG_F_REQ_RESTART, 0, 0, 1, "0=off 1=on" },
    { 0x0C31U, "cl3_pos_en", CFG_T_U8,
      CFG_F_ADVANCED, 0, 0, 1, "0=off 1=on" },
    { 0x0C32U, "cl3_spd_en", CFG_T_U8,
      CFG_F_ADVANCED, 0, 0, 1, "0=off 1=on" },
    { 0x0C33U, "cl3_cur_en", CFG_T_U8,
      CFG_F_ADVANCED, 0, 0, 1, "0=off 1=on" },
    { 0x0C34U, "cl3_pos_period", CFG_T_U16,
      CFG_F_ADVANCED, 5, 1, 100, "ms" },
    { 0x0C35U, "cl3_spd_period", CFG_T_U16,
      CFG_F_ADVANCED, 1, 1, 100, "ms" },
    { 0x0C36U, "cl3_pos_kp", CFG_T_I32,
      CFG_F_ADVANCED, 0, 0, 100000, "x100" },
    { 0x0C37U, "cl3_pos_ki", CFG_T_I32,
      CFG_F_ADVANCED, 0, 0, 100000, "x100" },
    { 0x0C38U, "cl3_pos_kd", CFG_T_I32,
      CFG_F_ADVANCED, 0, 0, 100000, "x100" },
    { 0x0C39U, "cl3_spd_kp", CFG_T_I32,
      CFG_F_ADVANCED, 0, 0, 100000, "x100" },
    { 0x0C3AU, "cl3_spd_ki", CFG_T_I32,
      CFG_F_ADVANCED, 0, 0, 100000, "x100" },
    { 0x0C3BU, "cl3_spd_kd", CFG_T_I32,
      CFG_F_ADVANCED, 0, 0, 100000, "x100" },
    { 0x0C3CU, "cl3_cur_kp", CFG_T_I32,
      CFG_F_ADVANCED, 0, 0, 100000, "x100" },
    /* ---- E-3（2026-11-14）钥匙去抖参数纳入配置表 ----------------------
     * 依据：docs/02-设计/裁定-整车对象状态模型与E系列-V1.0.md §5.3
     * 追加在表尾（本文件顶部规则：新增参数只能追加，不得改已有 ID 含义）。
     * find_def_index() 按 ID 线性查找，物理位置无关。
     * CFG_F_REQ_RESTART：Q-84「改后重启生效」，避免去抖计时中途变判据。
     * ✅ D-1 已于 2026-11-14 15:34 裁定（Q-A：rpm>400 或 V>13.0V，连续 500ms），
     *    见 docs/01-基线/需求基线-BASE-REQ.md §4.5；本表 enter_volt_dv=130 已符合。 */
    { CFG_ID_ACC_ENTER_MS, "acc_enter_ms", CFG_T_U16,
      CFG_F_REQ_RESTART, 500, 100, 5000, "ms" },
    { CFG_ID_ACC_EXIT_MS,  "acc_exit_ms",  CFG_T_U16,
      CFG_F_REQ_RESTART, 500, 100, 5000, "ms" },
    { CFG_ID_ACC_POLL_MS,  "acc_poll_ms",  CFG_T_U16,
      CFG_F_REQ_RESTART,  50,  50,   50, "ms" },

    /* ---- Q-B（2026-11-14）整车 FAULT 去抖 -------------------------
     * 依据：需求基线 §4.5 Q-B / SWR-N-002（2s 确认、2s 恢复）。
     * 追加在表尾，不改既有 ID 含义；改后重启生效（Q-84）。 */
    { CFG_ID_VEH_FAULT_MS,  "veh_fault_ms",  CFG_T_U16,
      CFG_F_REQ_RESTART, 2000, 100, 10000, "ms" },

    /* ---- Q-D2（2026-11-14 16:49 裁定 500ms）-----------------------
     * 接收侧最慢周期帧 100ms ⇒ 取 5 倍余量；沿用旧 100ms 会周期性误进 FAULT。
     * 追加在表尾，不改既有 ID 含义；改后重启生效（Q-84）。 */
    { CFG_ID_VEH_CAN_SILENT_MS, "veh_can_silent_ms", CFG_T_U16,
      CFG_F_REQ_RESTART, 500, 100, 10000, "ms" },
};

#define CFG_DEF_COUNT   (sizeof(s_cfg_def) / sizeof(s_cfg_def[0]))

/* ============================================================
 * 2. 运行时状态
 * ============================================================ */
static cfg_item_t        s_rt[CFG_MAX_ITEMS]; /* RAM 当前生效值 */
static cfg_src_t         s_src[CFG_MAX_ITEMS];/* 每项来源 */
static uint8_t           s_dirty[CFG_MAX_ITEMS];
static uint8_t           s_inited = 0U;
static uint16_t          s_rt_count = 0U;     /* s_rt 实际项数 */
static uint32_t          s_write_count = 0U;  /* 诊断：落盘次数 */

/* ============================================================
 * 3. 工具
 * ============================================================ */
static const cfg_def_t *find_def_by_index(uint16_t idx)
{
    return (idx < CFG_DEF_COUNT) ? &s_cfg_def[idx] : RT_NULL;
}

/** 按 ID 查定义表下标，未找到返回 -1 */
static int32_t find_def_index(uint16_t id)
{
    uint16_t i;

    for (i = 0U; i < CFG_DEF_COUNT; i++) {
        if (s_cfg_def[i].id == id) {
            return (int32_t)i;
        }
    }
    return -1;
}

/** 取某项的 int32 表示（按 type 解引用） */
static int32_t item_to_i32(const cfg_item_t *it)
{
    switch (it->type) {
        case CFG_T_U8:  return (int32_t)it->v.u8;
        case CFG_T_U16: return (int32_t)it->v.u16;
        case CFG_T_I16: return (int32_t)it->v.i16;
        case CFG_T_I32: return it->v.i32;
        case CFG_T_U32: return (int32_t)it->v.u32;   /* 范围校验用，见下 */
        default:        return 0;
    }
}

/** 把 int32 写入项（按 type 截断） */
static void i32_to_item(cfg_item_t *it, int32_t val)
{
    switch (it->type) {
        case CFG_T_U8:  it->v.u8  = (uint8_t)(uint32_t)val;  break;
        case CFG_T_U16: it->v.u16 = (uint16_t)(uint32_t)val; break;
        case CFG_T_I16: it->v.i16 = (int16_t)val;            break;
        case CFG_T_I32: it->v.i32 = val;                     break;
        case CFG_T_U32: it->v.u32 = (uint32_t)val;           break;
        default: break;
    }
}

/**
 * @brief 范围校验
 * @note  U32 类型的值可能 > INT32_MAX，用无符号比较，避免把合法大值判成负数越界。
 */
static uint8_t value_in_range(const cfg_item_t *it, const cfg_def_t *d)
{
    int32_t v;

    if (it->type == CFG_T_U32) {
        uint32_t uv = it->v.u32;

        /* 默认/上下界都是 int32 常量；若上界为正，按无符号比 */
        if (d->min_val >= 0 && d->max_val >= 0) {
            return (uv >= (uint32_t)d->min_val && uv <= (uint32_t)d->max_val) ? 1U : 0U;
        }
        return 0U;   /* U32 配负边界属于定义表错误，判无效 */
    }

    v = item_to_i32(it);
    return (v >= d->min_val && v <= d->max_val) ? 1U : 0U;
}

/**
 * @brief 加载全部默认值到 RAM
 */
static void load_all_defaults(void)
{
    uint16_t i;

    s_rt_count = (uint16_t)CFG_DEF_COUNT;

    for (i = 0U; i < s_rt_count; i++) {
        s_rt[i].id    = s_cfg_def[i].id;
        s_rt[i].type  = s_cfg_def[i].type;
        s_rt[i].flags = s_cfg_def[i].flags;
        i32_to_item(&s_rt[i], s_cfg_def[i].def_val);

        s_src[i]   = CFG_SRC_DEFAULT;
        s_dirty[i] = 0U;
    }
}

/* ============================================================
 * 4. 初始化
 * ============================================================ */
int sys_config_init(void)
{
    uint16_t i;

    /* ① 先无条件加载默认值：默认值永远是兜底 */
    load_all_defaults();

    /* ② 新存储：从 MOTION/SYSTEM 记录恢复关键项，逐项做范围校验。
     * s_inited 必须先置位，set_raw 才有生效路径。 */
    s_inited = 1U;

    if (param_store_cfg_load() == 0) {
        rt_kprintf("[CFG] key params loaded from flash\n");
    } else {
        rt_kprintf("[CFG] no valid key record, using defaults\n");
        if (param_store_cfg_was_default() != 0) {
            (void)param_store_save_now(PARAM_REQ_CFG);
            rt_kprintf("[CFG] defaults written to flash\n");
        }
    }

    /* 只读镜像项始终以真值为准 */
    sys_config_refresh_mirrors();

    for (i = 0U; i < s_rt_count; i++) {
        s_dirty[i] = 0U;
    }

    rt_kprintf("[CFG] loaded: %u runtime items\n", (unsigned)s_rt_count);
    return 0;
}

/* ============================================================
 * 5. 保存 / 加载
 * ============================================================ */

int sys_config_save(void)
{
    uint16_t i;

    if (s_inited == 0U) { return -1; }

    /* 同步只读镜像项的当前真值 */
    sys_config_refresh_mirrors();

    /* 交给存储层异步落盘：运行中会延后到停机窗口 */
    if (param_store_cfg_save() != 0) {
        rt_kprintf("[CFG] SAVE request failed, data still in RAM\n");
        return -1;
    }

    s_write_count++;
    for (i = 0U; i < s_rt_count; i++) {
        s_dirty[i] = 0U;
        if (s_src[i] == CFG_SRC_RAM) { s_src[i] = CFG_SRC_FLASH; }
    }

    rt_kprintf("[CFG] SAVED %u key items (queued)\n", (unsigned)s_rt_count);
    return 0;
}

int sys_config_load(void)
{
    if (s_inited == 0U) { return -1; }

    /* 重新走一遍完整初始化流程：默认值 + Flash 合并。
     * 这样"未保存的 RAM 修改"自然被丢弃，无需单独清理。 */
    s_inited = 0U;
    return sys_config_init();
}

/* ============================================================
 * 6. 恢复默认 / 擦除
 * ============================================================ */
int sys_config_restore_default(uint16_t id)
{
    uint16_t i;

    if (s_inited == 0U) { return -1; }

    if (id == 0U) {
        load_all_defaults();
        rt_kprintf("[CFG] all items restored to DEFAULT (RAM only)\n");
        rt_kprintf("[CFG] run 't_cfg save' to persist\n");
        return 0;
    }

    for (i = 0U; i < s_rt_count; i++) {
        if (s_rt[i].id == id) {
            const cfg_def_t *d = find_def_by_index(i);

            if (d == RT_NULL) { return -1; }
            if ((d->flags & CFG_F_READONLY) != 0U) {
                rt_kprintf("[CFG] id=0x%04X is READONLY, cannot restore\n",
                           (unsigned)id);
                return -1;
            }

            i32_to_item(&s_rt[i], d->def_val);
            s_src[i]     = CFG_SRC_DEFAULT;
            s_dirty[i]   = 1U;

            rt_kprintf("[CFG] id=0x%04X (%s) -> default %d\n",
                       (unsigned)id, d->name, (int)d->def_val);
            return 0;
        }
    }

    rt_kprintf("[CFG] id=0x%04X not found\n", (unsigned)id);
    return -1;
}

int sys_config_erase(void)
{
    if (s_inited == 0U) { return -1; }

    rt_kprintf("[CFG] erasing storage...\n");
    load_all_defaults();

    if (param_store_save_now(PARAM_REQ_CFG) != 0) {
        rt_kprintf("[CFG] erase failed, defaults are in RAM only\n");
        return -1;
    }

    rt_kprintf("[CFG] erased, all defaults written back\n");
    return 0;
}

/* ============================================================
 * 7. 读写
 * ============================================================ */
int32_t sys_config_get_u32(uint16_t id, uint32_t *out)
{
    int32_t idx;

    if (s_inited == 0U || out == RT_NULL) { return -1; }

    idx = find_def_index(id);
    if (idx < 0) { return -1; }

    *out = (s_rt[idx].type == CFG_T_U32) ? s_rt[idx].v.u32
                                         : (uint32_t)item_to_i32(&s_rt[idx]);
    return 0;
}

int32_t sys_config_get_i32(uint16_t id, int32_t *out)
{
    int32_t idx;

    if (s_inited == 0U || out == RT_NULL) { return -1; }

    idx = find_def_index(id);
    if (idx < 0) { return -1; }

    *out = item_to_i32(&s_rt[idx]);
    return 0;
}

int32_t sys_config_set_i32(uint16_t id, int32_t val)
{
    int32_t idx;
    const cfg_def_t *d;

    if (s_inited == 0U) { return -1; }

    idx = find_def_index(id);
    if (idx < 0) {
        rt_kprintf("[CFG] set: id=0x%04X not found\n", (unsigned)id);
        return -1;
    }

    d = &s_cfg_def[idx];

    if ((d->flags & CFG_F_READONLY) != 0U) {
        rt_kprintf("[CFG] set: id=0x%04X (%s) is READONLY\n",
                   (unsigned)id, d->name);
        return -1;
    }

    /* 用临时项做范围校验，避免"先写坏再发现越界" */
    {
        cfg_item_t tmp = s_rt[idx];

        i32_to_item(&tmp, val);
        if (value_in_range(&tmp, d) == 0U) {
            rt_kprintf("[CFG] set: id=0x%04X (%s) value %d out of [%d, %d]\n",
                       (unsigned)id, d->name, (int)val,
                       (int)d->min_val, (int)d->max_val);
            return -1;
        }
    }

    i32_to_item(&s_rt[idx], val);
    s_src[idx]   = CFG_SRC_RAM;
    s_dirty[idx] = 1U;

    if ((d->flags & CFG_F_REQ_RESTART) != 0U) {
        rt_kprintf("[CFG] id=0x%04X (%s) = %d  (takes effect after RESTART)\n",
                   (unsigned)id, d->name, (int)val);
    } else {
        rt_kprintf("[CFG] id=0x%04X (%s) = %d\n",
                   (unsigned)id, d->name, (int)val);
    }
    rt_kprintf("[CFG] not persisted yet, run 't_cfg save'\n");
    return 0;
}

/* ============================================================
 * 8. 镜像项刷新（只读项的真值在别处）
 * ------------------------------------------------------------
 * 设计原则（设计文档 §5 0x07xx/0x08xx）：
 *   真值只有一处，本模块只做"统一查询入口"，绝不存副本。
 *   这里在 dump/save 前把真值抓过来，保证显示与实物一致。
 * ============================================================ */
void sys_config_refresh_mirrors(void)
{
    uint16_t zero[3];
    int32_t idx;

    /* ---- 0x07xx 标定标志：真值在 sys_param ---- */
    idx = find_def_index(CFG_ID_ENC_ZERO_VALID);
    if (idx >= 0) {
        i32_to_item(&s_rt[idx], (int32_t)(sys_param_get_enc_zero(zero) != 0U ? 1 : 0));
    }

    idx = find_def_index(CFG_ID_LIFT_CALIB_VALID);
    if (idx >= 0) {
        const sys_param_t *p = sys_param_get();
        i32_to_item(&s_rt[idx], (p != RT_NULL) ? (int32_t)p->is_calibrated : 0);
    }

    /* ---- 0x08xx 齿数：真值在 mech_cfg ---- */
    if (g_mech.is_loaded != 0U) {
        idx = find_def_index(CFG_ID_MECH_TEETH_Z0);
        if (idx >= 0) { i32_to_item(&s_rt[idx], (int32_t)g_mech.cfg.enc_z01_num); }

        idx = find_def_index(CFG_ID_MECH_TEETH_Z1);
        if (idx >= 0) { i32_to_item(&s_rt[idx], (int32_t)g_mech.cfg.enc_z01_den); }

        idx = find_def_index(CFG_ID_MECH_TEETH_Z2);
        if (idx >= 0) { i32_to_item(&s_rt[idx], (int32_t)g_mech.cfg.enc_z02_den); }

        idx = find_def_index(CFG_ID_MECH_ACTIVE_IDX);
        if (idx >= 0) { i32_to_item(&s_rt[idx], (int32_t)g_mech.active_idx); }
    }
}

/* ============================================================
 * 9. 诊断
 * ============================================================ */
uint8_t sys_config_is_dirty(void)
{
    return (sys_config_dirty_count() > 0U) ? 1U : 0U;
}

uint32_t sys_config_dirty_count(void)
{
    uint16_t i;
    uint32_t n = 0U;

    for (i = 0U; i < s_rt_count; i++) {
        if (s_dirty[i] != 0U) { n++; }
    }
    return n;
}

cfg_src_t sys_config_get_src(uint16_t id)
{
    int32_t idx = find_def_index(id);

    return (idx >= 0) ? s_src[idx] : CFG_SRC_DEFAULT;
}

const cfg_def_t *sys_config_find_def(uint16_t id)
{
    int32_t idx = find_def_index(id);

    return (idx >= 0) ? &s_cfg_def[idx] : RT_NULL;
}

const cfg_def_t *sys_config_find_def_by_name(const char *name)
{
    uint16_t i;

    if (name == RT_NULL) { return RT_NULL; }

    for (i = 0U; i < CFG_DEF_COUNT; i++) {
        if (rt_strcmp(s_cfg_def[i].name, name) == 0) {
            return &s_cfg_def[i];
        }
    }
    return RT_NULL;
}

const char *sys_config_type_name(uint8_t type)
{
    switch (type) {
        case CFG_T_U8:  return "U8";
        case CFG_T_U16: return "U16";
        case CFG_T_I16: return "I16";
        case CFG_T_I32: return "I32";
        case CFG_T_U32: return "U32";
        default:        return "?";
    }
}

const char *sys_config_src_name(cfg_src_t s)
{
    switch (s) {
        case CFG_SRC_DEFAULT: return "DEF";
        case CFG_SRC_FLASH:   return "FLASH";
        case CFG_SRC_RAM:     return "RAM";
        default:              return "?";
    }
}

void sys_config_dump_stats(void)
{
    if (s_inited == 0U) {
        rt_kprintf("[CFG] not initialized\n");
        return;
    }

    rt_kprintf("[CFG] items=%u (def table=%u)  dirty=%u\n",
               (unsigned)s_rt_count, (unsigned)CFG_DEF_COUNT,
               (unsigned)sys_config_dirty_count());
    rt_kprintf("[CFG] writes=%u  persist=motion+system records\n",
               (unsigned)s_write_count);
}

/* ============================================================
 * 10. 内部访问（供 cmd 模块遍历，避免暴露静态数组）
 * ============================================================ */
uint16_t sys_config_item_count(void)
{
    return s_rt_count;
}

const cfg_item_t *sys_config_item_at(uint16_t idx)
{
    return (idx < s_rt_count) ? &s_rt[idx] : RT_NULL;
}

uint8_t sys_config_item_is_dirty(uint16_t idx)
{
    return (idx < s_rt_count) ? s_dirty[idx] : 0U;
}

/* 供 cmd 模块直接写只读镜像的原始值（仅用于 refresh） */
int32_t sys_config_set_raw(uint16_t id, int32_t val)
{
    int32_t idx = find_def_index(id);

    if (idx < 0) { return -1; }
    i32_to_item(&s_rt[idx], val);
    return 0;
}