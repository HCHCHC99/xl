/**
 * @file    input_sim.c
 * @brief   业务输入台架模拟层实现（任务清单 §11）
 *
 * 【设计要点】
 *   1. 只覆盖"读输入"的位置：业务门禁、状态机、限位、故障判定全部走真实代码，
 *      所以用模拟输入跑出来的行为与真实硬件一致。
 *   2. 周期性输入（面板 0x18FF8380 / 转速 0x0CF00400）用 50ms 软定时器重发：
 *      面板侧 kb_on_can_frame() 有 300ms 超时释放，转速侧有 100ms 超时归零，
 *      单帧注入会被误判为"松手/停机"。
 *   3. 一次性输入（阀反馈 0x18FF0E63 / 机型 0x18FF7F18）只在设置时注入一帧，
 *      避免周期写 Flash（机型帧会触发 sys_param_save）。
 *   4. 注入走 ut_can_handlers.c 的真实路由表与回调，不改任何解析逻辑。
 *   5. 打印只用 ASCII/整数（RTT 控制台）。
 */

#include "input_sim.h"

#include <rtthread.h>
#include <finsh.h>
#include <msh.h>                    /* msh_exec：t_seq 逐条执行脚本 */
#include <string.h>

#include "../CAN/protocol_defs.h"
#include "../CAN/ut_can_handlers.h"
#include "../keyboard/keyboard_defs.h"
#include "../vehicle/vehicle_control.h"   /* t_bench：整车状态      */
#include "../lift/lift_adc.h"             /* t_sim status：LIFT_ADC_SIM_ENABLE */
#include "../swing/swing_angle.h"         /* t_sim status：SWING_ANGLE_SIM_ENABLE */
#include "key_state.h"                    /* t_bench：acc_is_on()   */
#include "sys_voltage.h"                  /* t_bench：母线读数      */

/* 主开关：INPUT_SIM_ENABLE=0 时整层不产生任何代码（t_sim 命令随之消失，
 * 各钩子也已被各自的 #if 排除，不会留未定义引用）。 */
#if INPUT_SIM_ENABLE

/* ============================================================
 * 参数
 * ============================================================ */
#define SIM_TICK_MS          50U      /* 重发周期：面板 100ms、转速 10ms 都满足 */
#define SIM_PULSE_TICKS      6U       /* 6 x 50ms = 300ms 短按 */

/* CAN 注入是否可用：CAN_SIM_ENABLE=0 时注入函数退化为空操作，
 * 保证"只关某一类输入"也能编译（各头文件开关互相独立）。 */
#if (INPUT_SIM_ENABLE && CAN_SIM_ENABLE)
#define SIM_CAN_ENABLED      1
#else
#define SIM_CAN_ENABLED      0
#endif

/* ============================================================
 * 状态（单字读写，跨线程安全）
 * ============================================================ */
static volatile int32_t s_value[INPUT_SIM_SRC_COUNT];
static volatile uint8_t s_active[INPUT_SIM_SRC_COUNT];

static struct rt_timer s_timer;
static uint8_t  s_timer_ready = 0U;
static uint8_t  s_panel_type  = 0U;     /* 注入帧 Byte3：0无/8 8键/10 10键 */
static uint8_t  s_pulse_left  = 0U;     /* 短按剩余 tick，0 = 非短按 */

static const char *const s_names[INPUT_SIM_SRC_COUNT] = {
    "acc",      /* 0 */
    "bus",      /* 1 */
    "swing",    /* 2 */
    "lift",     /* 3 */
    "key",      /* 4 */
    "rpm",      /* 5 */
    "valve",    /* 6 */
    "teeth",    /* 7 */
    "cansilent" /* 8 */
};

/* ============================================================
 * 注入：0x18FF8380 面板帧
 * ============================================================ */
static void sim_inject_keys(uint16_t mask)
{
#if SIM_CAN_ENABLED
    panel_switch_t f;

    memset(&f, 0, sizeof(f));

    f.bits.clutch_combine  = ((mask & KB_CLUTCH_COM)  != 0U) ? 1U : 0U;
    f.bits.clutch_separate = ((mask & KB_CLUTCH_SEP)  != 0U) ? 1U : 0U;

    f.bits.key_up     = ((mask & KB_UP)        != 0U) ? 1U : 0U;
    f.bits.key_down   = ((mask & KB_DOWN)      != 0U) ? 1U : 0U;
    f.bits.key_left   = ((mask & KB_LEFT)      != 0U) ? 1U : 0U;
    f.bits.key_right  = ((mask & KB_RIGHT)     != 0U) ? 1U : 0U;

    f.bits.onekey_home = ((mask & KB_ONKEY_HOME) != 0U) ? 1U : 0U;
    f.bits.key_calib   = ((mask & KB_CALIB)      != 0U) ? 1U : 0U;
    f.bits.onekey_mid  = ((mask & KB_ONKEY_MID)  != 0U) ? 1U : 0U;
    f.bits.onekey_rear = ((mask & KB_ONKEY_REAR) != 0U) ? 1U : 0U;

    f.bits.remote_type = s_panel_type;

    can_sim_inject(0x18FF8380UL, f.raw, 8U);
#else
    (void)mask;
#endif
}

/* ============================================================
 * 注入：0x0CF00400 发动机转速帧
 * ============================================================ */
static void sim_inject_rpm(uint16_t rpm)
{
#if SIM_CAN_ENABLED
    engine_rpm_t f;
    uint32_t raw;

    raw = (uint32_t)rpm * 8U;                  /* 0.125 rpm/bit */
    if (raw > 0xFFFFU) {
        raw = 0xFFFFU;
    }

    memset(&f, 0, sizeof(f));
    f.bits.engine_rpm_raw = (uint16_t)raw;

    can_sim_inject(0x0CF00400UL, f.raw, 8U);
#else
    (void)rpm;
#endif
}

/* ============================================================
 * 注入：0x18FF0E63 阀反馈帧（一次性）
 * ============================================================ */
static void sim_inject_valve(uint8_t packed)
{
#if SIM_CAN_ENABLED
    solenoid_fb_t f;

    memset(&f, 0, sizeof(f));

    f.bits.up_fault   = (uint8_t)(packed & 0x03U);
    f.bits.down_fault = (uint8_t)((packed >> 2) & 0x03U);
    f.bits.up_open    = (uint8_t)((packed >> 4) & 0x01U);
    f.bits.down_open  = (uint8_t)((packed >> 5) & 0x01U);

    can_sim_inject(0x18FF0E63UL, f.raw, 8U);
#else
    (void)packed;
#endif
}

/* ============================================================
 * 注入：0x18FF7F18 机型帧（一次性；会触发一次 Flash 保存）
 * ============================================================ */
static void sim_inject_teeth(uint8_t v)
{
#if SIM_CAN_ENABLED
    teeth_setting_t f;

    memset(&f, 0, sizeof(f));
    f.bits.teeth_set = (uint8_t)(v & 0x03U);

    can_sim_inject(0x18FF7F18UL, f.raw, 8U);
#else
    (void)v;
#endif
}

/* ============================================================
 * 周期重发
 * ============================================================ */
static void sim_timer_cb(void *param)
{
    (void)param;

    /* 短按计时：到点自动松开，并补发一帧"全松开"产生释放沿 */
    if (s_pulse_left != 0U) {
        s_pulse_left--;
        if (s_pulse_left == 0U) {
            s_active[INPUT_SIM_KEY] = 0U;
            s_value[INPUT_SIM_KEY]  = 0;
            sim_inject_keys(0U);
        }
    }

    if (s_active[INPUT_SIM_KEY] != 0U) {
        sim_inject_keys((uint16_t)s_value[INPUT_SIM_KEY]);
    }

    if (s_active[INPUT_SIM_RPM] != 0U) {
        int32_t v = s_value[INPUT_SIM_RPM];

        if (v < 0) {
            v = 0;
        } else if (v > 65535) {
            v = 65535;
        }
        sim_inject_rpm((uint16_t)v);
    }
}

void input_sim_init(void)
{
    if (s_timer_ready != 0U) {
        return;
    }

    rt_timer_init(&s_timer,
                  "sim_in",
                  sim_timer_cb,
                  RT_NULL,
                  rt_tick_from_millisecond((rt_int32_t)SIM_TICK_MS),
                  RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    (void)rt_timer_start(&s_timer);

    s_timer_ready = 1U;
}

/* ============================================================
 * 查询 / 设置
 * ============================================================ */
uint8_t input_sim_active(input_sim_src_t src)
{
    if ((uint32_t)src >= (uint32_t)INPUT_SIM_SRC_COUNT) {
        return 0U;
    }
    return s_active[src];
}

int32_t input_sim_value(input_sim_src_t src)
{
    if ((uint32_t)src >= (uint32_t)INPUT_SIM_SRC_COUNT) {
        return 0;
    }
    return s_value[src];
}

int input_sim_set(input_sim_src_t src, int32_t value)
{
    if ((uint32_t)src >= (uint32_t)INPUT_SIM_SRC_COUNT) {
        return -1;
    }

    input_sim_init();

    s_value[src] = value;

    switch (src) {
    case INPUT_SIM_KEY:
        s_pulse_left = 0U;
        s_active[src] = 1U;
        sim_inject_keys((uint16_t)value);
        break;

    case INPUT_SIM_RPM: {
        int32_t v = value;

        if (v < 0) {
            v = 0;
        } else if (v > 65535) {
            v = 65535;
        }
        s_value[src]  = v;
        s_active[src] = 1U;
        sim_inject_rpm((uint16_t)v);
        break;
    }

    case INPUT_SIM_VALVE:
        s_active[src] = 1U;
        sim_inject_valve((uint8_t)value);
        break;

    case INPUT_SIM_TEETH:
        s_active[src] = 1U;
        sim_inject_teeth((uint8_t)value);
        break;

    default:
        s_active[src] = 1U;
        break;
    }

    return 0;
}

void input_sim_clear(input_sim_src_t src)
{
    if ((uint32_t)src >= (uint32_t)INPUT_SIM_SRC_COUNT) {
        return;
    }

    s_active[src] = 0U;
    s_value[src]  = 0;

    if (src == INPUT_SIM_KEY) {
        s_pulse_left = 0U;
        sim_inject_keys(0U);        /* 补一帧全松开，产生释放沿 */
    }
}

void input_sim_clear_all(void)
{
    input_sim_src_t i;

    for (i = (input_sim_src_t)0; i < INPUT_SIM_SRC_COUNT;
         i = (input_sim_src_t)(i + 1)) {
        input_sim_clear(i);
    }
}

int input_sim_name_to_src(const char *name, input_sim_src_t *out)
{
    uint32_t i;

    if ((name == RT_NULL) || (out == RT_NULL)) {
        return -1;
    }

    for (i = 0U; i < (uint32_t)INPUT_SIM_SRC_COUNT; i++) {
        if (strcmp(name, s_names[i]) == 0) {
            *out = (input_sim_src_t)i;
            return 0;
        }
    }
    return -1;
}

/* ============================================================
 * MSH 命令
 * ============================================================ */
/* 有符号十进制解析（不接受除数字与前置 '-' 以外的字符） */
static int sim_parse_i32(const char *s, int32_t *out)
{
    int32_t v = 0;
    uint8_t neg = 0U;
    uint8_t digits = 0U;

    if ((s == RT_NULL) || (out == RT_NULL)) {
        return -1;
    }

    if (*s == '-') {
        neg = 1U;
        s++;
    }

    while ((*s >= '0') && (*s <= '9')) {
        v = (v * 10) + (int32_t)(*s - '0');
        s++;
        digits++;
    }

    if ((digits == 0U) || (*s != '\0')) {
        return -1;
    }

    *out = (neg != 0U) ? -v : v;
    return 0;
}

static const struct {
    const char *name;
    uint16_t    bit;
} s_key_names[] = {
    { "up",    KB_UP },
    { "down",  KB_DOWN },
    { "left",  KB_LEFT },
    { "right", KB_RIGHT },
    { "mid",   KB_ONKEY_MID },
    { "rear",  KB_ONKEY_REAR },
    { "home",  KB_ONKEY_HOME },
    { "calib", KB_CALIB },
    { "ccomb", KB_CLUTCH_COM },
    { "csep",  KB_CLUTCH_SEP },
};

#define KEY_NAME_COUNT  ((uint32_t)(sizeof(s_key_names) / sizeof(s_key_names[0])))

static int sim_key_bit(const char *name, uint16_t *bit)
{
    uint32_t i;

    for (i = 0U; i < KEY_NAME_COUNT; i++) {
        if (strcmp(name, s_key_names[i].name) == 0) {
            *bit = s_key_names[i].bit;
            return 0;
        }
    }
    return -1;
}

static void sim_dump(void)
{
    uint32_t i;

    rt_kprintf("[sim] master=%d tick=%dms panel=%u\n",
               (int)INPUT_SIM_ENABLE, (int)SIM_TICK_MS, (unsigned)s_panel_type);
    rt_kprintf("[sim] name       act value\n");

    for (i = 0U; i < (uint32_t)INPUT_SIM_SRC_COUNT; i++) {
        rt_kprintf("[sim] %-10s %u %d\n",
                   s_names[i],
                   (unsigned)s_active[i],
                   (int)s_value[i]);
    }

    rt_kprintf("[sim] acc: 0=key ON(low) 1=key OFF(high)\n");
    rt_kprintf("[sim] swing: 0.1deg(raw)  lift: mV  bus: mV  rpm: rpm\n");
    rt_kprintf("[sim] valve: bit0-1 up_fault/bit2-3 dn_fault/bit4-5 open\n");
    rt_kprintf("[sim] key names: up down left right mid rear home calib ccomb csep\n");
}

/* ============================================================
 * t_sim status / 上电 [SIM] boot —— 三态显示（2026-11-23 P1）
 * ------------------------------------------------------------
 * REAL(rt)  = 走真实输入
 * SIM(值)   = 被模拟值覆盖
 * OFF(cmp)  = 该输入的编译期开关为 0，本固件无法模拟
 * 只读打印，不改变任何覆盖状态。
 * ============================================================ */
static void sim_state_one(const char *name, uint8_t enabled, input_sim_src_t src)
{
    if (enabled == 0U) {
        rt_kprintf(" %s=OFF(cmp)", name);
    } else if (s_active[src] != 0U) {
        rt_kprintf(" %s=SIM(%d)", name, (int)s_value[src]);
    } else {
        rt_kprintf(" %s=REAL(rt)", name);
    }
}

static void sim_status_print(uint8_t boot)
{
    rt_kprintf("[SIM] %slayer=%s",
               (boot != 0U) ? "boot " : "",
               (INPUT_SIM_ENABLE != 0) ? "ON" : "OFF");

    sim_state_one("acc",   ACC_SIM_ENABLE,        INPUT_SIM_ACC);
    sim_state_one("bus",   SYS_VOLT_SIM_ENABLE,   INPUT_SIM_BUS);
    sim_state_one("lift",  LIFT_ADC_SIM_ENABLE,   INPUT_SIM_LIFT);
    sim_state_one("swing", SWING_ANGLE_SIM_ENABLE, INPUT_SIM_SWING);
    sim_state_one("key",   CAN_SIM_ENABLE,        INPUT_SIM_KEY);

    rt_kprintf("\n");
}

void input_sim_status(void)
{
    sim_status_print(0U);
}

void input_sim_boot_report(void)
{
    sim_status_print(1U);
}

/* ============================================================
 * t_bench —— 台架前置一键（原 M0 四步，每步间隔 1 s）
 * ------------------------------------------------------------
 * 把以下四步合成一条命令，自动按 1 s 间隔执行并给出结论：
 *   1) t_sim cansilent 0    伪造"通信正常"
 *   2) t_veh_clr            整车对象回 OFF
 *   3) t_sim bus <mv>       母线电压（默认 13200，>13.0V）
 *   4) t_sim acc on         钥匙上电（500ms 去抖）
 * 最后等 1.2 s（够 ACC 500ms + 启车 500ms）再汇总。
 *
 * 用法：t_bench [bus_mv]     （不给参数 = 13200 mV）
 * 别名：t_pre
 * ============================================================ */
static void t_bench(int argc, char **argv)
{
    int32_t  bus_mv = 13200;
    uint32_t fault;
    uint8_t  started;

    if (argc >= 2) {
        if (sim_parse_i32(argv[1], &bus_mv) != 0) {
            rt_kprintf("[bench] usage: t_bench [bus_mv]\n");
            return;
        }
    }
    if ((bus_mv < 6000) || (bus_mv > 25000)) {
        bus_mv = 13200;
    }

    rt_kprintf("[bench] preamble: 4 steps, 1s apart\n");

    /* 【P3 / 2026-11-23】先把模拟按键清零，必须在 ACC 上电之前。
     *
     * 原因：若上一轮留下 `t_sim key xxx hold`（尤其 mid+rear+home 这种组合），
     * 50ms 重发会让 keyboard 的 mask 一直非 0；ACC 上电进入 STARTED 时
     * kb_request_rebaseline() 见 mask != 0 就把键盘锁进 KSM_BLOCKED，
     * 而 BLOCKED 只在 mask == 0 时解除 ⇒ 本轮所有手动键被静默吞掉。
     * 表现是 m11 的"手动打断"假失败（无 [Seq] Interrupted、无 [Lift] 动作行），
     * 极易误判成 P3 逻辑缺陷。这里注入一次 mask=0 即可解锁。 */
    rt_kprintf("[bench] 0/5 release sim keys (avoid rebaseline BLOCKED)\n");
    (void)input_sim_set(INPUT_SIM_KEY, 0);
    rt_thread_mdelay(200);

    rt_kprintf("[bench] 1/4 cansilent -> 0 (fake comm OK)\n");
    (void)input_sim_set(INPUT_SIM_CANSILENT, 0);
    rt_thread_mdelay(1000);

    rt_kprintf("[bench] 2/4 vehicle reset -> OFF\n");
    vehicle_force_reset();
    rt_thread_mdelay(1000);

    rt_kprintf("[bench] 3/4 bus -> %ld mV\n", (long)bus_mv);
    (void)input_sim_set(INPUT_SIM_BUS, bus_mv);
    rt_thread_mdelay(1000);

    rt_kprintf("[bench] 4/4 acc -> ON (500ms debounce)\n");
    (void)input_sim_set(INPUT_SIM_ACC, 0);
    rt_thread_mdelay(1200);

    fault   = vehicle_fault_word_get();
    started = vehicle_is_started();

    rt_kprintf("[bench] state=%u started=%u acc=%u fault=0x%lX volt=%lumV\n",
               (unsigned)vehicle_state_get(), (unsigned)started,
               (unsigned)acc_is_on(), (unsigned long)fault,
               (unsigned long)sys_voltage_read_bus_mv());

    if ((started != 0U) && (fault == 0UL)) {
        rt_kprintf("[bench] OK: STARTED, ready for test\n");
    } else {
        rt_kprintf("[bench] NOT READY: see fault_word / bus / acc above\n");
    }
}

MSH_CMD_EXPORT_ALIAS(t_bench, t_bench, bench preamble: cansilent/reset/bus/acc);
MSH_CMD_EXPORT_ALIAS(t_bench, t_pre,   alias of t_bench);

/* ============================================================
 * t_seq —— 测试脚本：按固定间隔（默认 1 s）自动逐条执行
 * ------------------------------------------------------------
 * 把每个模块的用例固化成脚本，免去手打。
 *   t_seq            → 列出可用脚本
 *   t_seq m4         → 跑 m4，步间隔 1 s
 *   t_seq m4 500     → 步间隔 500 ms
 *   t_seq m18 3 4    → 依次跑 m18、m3、m4，步间隔 1 s
 *   t_seq m18 3 4 500→ 依次跑，步间隔 500 ms
 * 纯数字参数按 m<数字> 解析；最后一个 ≥100 的数字视为步间隔。
 * 每步先打印 `[seq] (i/n) <命令>`，再执行，再等待。
 * 用 msh_exec() 执行原文，等于把命令逐条敲一遍（打印/行为完全一致）。
 * ============================================================ */
typedef struct {
    const char *name;
    const char *desc;
    const char *const *steps;      /* 以 RT_NULL 结尾 */
} test_seq_t;

#define SEQ_BATCH_MAX   8U

/* M3 举升：角度链 / 手动阀输出 / 上限拦截
 * 【P5 / B-6 起】传感器阈值与 2 s 确认整体移到 m8：
 *   2 s 确认与 1 s 步长不兼容（旧脚本"注入后下一拍必报"的断言已不成立）。 */
static const char *const s_seq_m3[] = {
    "t_sim lift 2203", "t_lift",
    "t_sim lift 3768", "t_lift",
    "t_sim lift 1111", "t_lift",
    "t_sim lift 2203",
    "t_sim key up hold",    "t_lift",
    "t_sim key up release", "t_lift",
    "t_sim key down hold",  "t_lift",
    "t_sim key down release",
    "t_sim lift 3768", "t_sim key up hold", "t_lift", "t_sim key none",
    RT_NULL
};

/* M8 举升传感器阈值与 2 s 确认/恢复（P5 / B-6）
 * 1 s 步长只能验证"1 s 未报 / 3 s 已报 / 恢复同理"，
 * 精确确认时间由最后一步 t_lift_sens test（10 ms 采样）给出。 */
static const char *const s_seq_m8[] = {
    "t_lift_sens",           /* 阈值 + 当前状态 */
    "t_sim lift 2203",
    "t_lift",                /* 基线：正常 */
    "t_sim lift 4800",       /* 高限越界，开始计时 */
    "t_lift",                /* 1 s：latched=0 timing=1，不得报 */
    "t_lift",                /* 2 s：边界 */
    "t_lift",                /* 3 s：FAULT_SENSOR latched=1 */
    "t_sim key up hold",     /* 故障期手动（TC-ANG-013） */
    "t_lift",                /* 期望 UP_RUNNING / out=0x01 */
    "t_sim key none",
    "t_lift",                /* 松手后应由 tick_check 退回 FAULT_SENSOR */
    "t_sim lift 4400",       /* 进入恢复计时 */
    "t_lift",                /* 1 s：仍故障 */
    "t_lift",                /* 2 s：边界 */
    "t_lift",                /* 3 s：IDLE 恢复 */
    "t_sim lift 250",        /* 低限越界 */
    "t_lift",
    "t_lift",
    "t_lift",                /* 3 s：FAULT_SENSOR */
    "t_sim lift 520",        /* 低限恢复 */
    "t_lift",
    "t_lift",
    "t_lift",                /* 3 s：IDLE */
    "t_lift_sens test",      /* 精确时序（约 9 s） */
    "t_sim lift 2203",
    RT_NULL
};

/* M4 回转：角度映射 / 刹车时序 / 限位拦截 */
static const char *const s_seq_m4[] = {
    "t_sim swing 0",    "t_swing",
    "t_sim swing 2150", "t_swing",
    "t_sim swing -1000","t_swing",
    "t_sim swing 0",
    "t_sim key left hold", "t_swing", "t_swing",
    "t_sim key left release", "t_swing",
    "t_sim swing -1000", "t_sim key left hold", "t_swing", "t_sim key none",
    "t_sim swing 2150",  "t_sim key right hold", "t_swing", "t_sim key none",
    RT_NULL
};

/* M6 阀故障：正常 / 开路 / 短路 / 下降阀 / 恢复 / 故障期禁手动 */
static const char *const s_seq_m6[] = {
    "t_sim valve 0 0 0 0", "t_lift",
    "t_sim valve 1 0",     "t_lift",
    "t_sim valve 2 0",     "t_lift",
    "t_sim valve 0 2",     "t_lift",
    "t_sim valve 0 0",     "t_lift",
    "t_sim valve 1 0", "t_sim key up hold", "t_lift", "t_sim key none",
    "t_sim valve 0 0", "t_lift",
    RT_NULL
};

/* M7 机型设置：13/14/11 齿切换 / 同值去重 / 清除 / 机械表回归 */
static const char *const s_seq_m7[] = {
    "t_param values",
    "t_sim teeth 1", "t_param values",
    "t_sim teeth 2", "t_param values",
    "t_sim teeth 3", "t_param values",
    "t_sim teeth 3", "t_param values",      /* 同值重复：应去重，不写 Flash */
    "t_param info",
    "t_sim teeth 0", "t_param values",
    "t_param info",
    "t_mech_check",
    RT_NULL
};

/* M9 欠压：进入 / 门禁拒绝 / 恢复 / 存档（不涉及编码器对齐，valid 会是 0x00） */
static const char *const s_seq_m9[] = {
    "t_param info",         /* 记下 fast 的 seq / left 基线 */
    "focmode 1 5",          /* LOCK，制造"有输出"状态，便于观察急停 */
    "t_sim bus 10800",      /* ≥30ms → UV enter + 急停 + 写快区 */
    "t_param values",       /* ★ 看 fast 段：off / dir / valid / snap_seq */
    "focmode 2 1 1.5",      /* 欠压期间应 refuse mode 2 */
    "focmode 9 500",        /* 应 refuse mode 9 */
    "focmode 3",            /* 欠压下应 refuse proc 3（被拒，不会真的做对齐） */
    "focmode 0",
    "fstat",
    "t_sim bus 12300",      /* ≥500ms → UV recover */
    "focmode 2 1 1.5",      /* 恢复后应能进入 */
    "focmode 0",
    "t_param info",         /* fast seq +1、left −1 */
    "t_param values",       /* ★ 再次确认 fast 段值 */
    RT_NULL
};

/* M9a 存档字段（需要编码器对齐作为前置，供批次 3/4 用；平时不必跑） */
static const char *const s_seq_m9a[] = {
    "focmode 3",            /* 两点式对齐：让快照 valid 置 bit0 */
    "fstat",                /* 期望 aligned=1 */
    "t_param info",         /* fast 基线 */
    "focmode 1 5",
    "t_sim bus 10800",      /* UV enter + 存档 */
    "t_param values",       /* ★ 期望 off=对齐 offset、dir=±1、valid=0x01 */
    "t_param info",         /* seq +1、left −1 */
    "t_sim bus 12300",
    "t_param values",       /* 存档值未被改写 */
    RT_NULL
};

/* M10 通信故障（P10）：300 ms comm_sta/灯 → 500 ms 静默 + 2 s 整车 FAULT → 恢复 */
static const char *const s_seq_m10[] = {
    "t_veh",
    "t_param info",         /* 基线：证明通信故障不产生 Flash 写 */
    "t_sim cansilent 0",    /* 强制通信正常 */
    "t_sta",                /* comm=0(ok) */
    "t_led",                /* ctrl 绿/黄（按校准状态） */
    "t_comm test",          /* 250ms→ok / 350ms→FAULT，验证 300ms 门限 */
    "t_sim cansilent 0",    /* t_comm test 会清掉强制，这里恢复 */
    "t_sta",                /* comm=0 */
    "t_sim cansilent 1",    /* 强制静默 */
    "t_veh",                /* +1s：整车尚未 FAULT（500ms 静默 + 2s 确认） */
    "t_sta",                /* +2s：comm=1(FAULT)，证明 300ms 通道独立 */
    "t_led",                /* ctrl 红闪 <- comm fault */
    "t_veh",                /* +4s：整车 FAULT */
    "t_sim cansilent 0",
    "t_sta",                /* comm=0，恢复 */
    "t_led",                /* ctrl 恢复 */
    "t_veh_clr",
    "t_veh",                /* 回 OFF，条件仍满足则自回 ACC_ON/STARTED */
    "t_param info",         /* 与基线一致：未写 Flash */
    RT_NULL
};

/* M5 一键触发链（P2）：事件 → 描述表 → 序列 → 引擎步骤日志 → 完成 */
static const char *const s_seq_m5[] = {
    "t_seq_auto 0",
    "t_sim swing -500",     /* 起点 -50°（t_sim 会强制解算有效） */
    "t_sim lift 2203",      /* 举升 0° */
    "t_arb clear",
    "seq_status",           /* 基线：state=0 step=0 One-key=NONE */
    "t_sim key mid pulse",  /* 短按一键右中：SEQ 事件 → 描述表 → 序列对象 */
    "seq_status",           /* 期望 state=1(RUNNING) step=0 One-key=RIGHT_MIDDLE */
    "t_arb dump",           /* 期望 grant+1，last evt=0x7000 cls=ONEKEY mod=SEQ */
    "t_arb log",            /* 期望一条 0x7000 GRANT del=1 */
    "t_seq_auto 1",         /* 台架：角度类完成条件直接判满足 */
    "seq_status",           /* 之后每 50ms 推进一步 */
    "seq_status",
    "seq_status",           /* 期望 state=3(DONE) One-key=NONE */
    "t_seq_auto 0",
    "t_sim key mid pulse",  /* DONE 后再按：应重新启动（不是"同键忽略"） */
    "seq_status",           /* 期望 state=1 step=0 One-key=RIGHT_MIDDLE */
    "t_sim key none",
    RT_NULL
};

/* M11 一键仲裁（P3）：同键重复 / 异键中断 / 三键同按 / 超右 / 超左 / 手动打断 */
static const char *const s_seq_m11[] = {
    "t_seq_auto 0",
    "t_sim lift 2203",       /* 举升 0°：让 step 0 不会秒完成 */
    "t_sim swing -500",      /* 筒 -50°：让 step 1 不会秒完成 */
    "t_seq_case 1",          /* 同键重复 → 期望 -1 ignored */
    "seq_status",
    "seq_abort",
    "t_seq_case 2",          /* 异键中断 → 期望 interrupts 并重启 */
    "seq_status",
    "seq_abort",
    "t_seq_case 3",          /* 三键同按 → 期望 -4 multi-key */
    "seq_status",
    "t_sim swing 2180",      /* 超右 +218° */
    "t_seq_case 4",          /* rear 期望 -3 limit；mid 期望 0 OK */
    "t_seq_run rear",        /* 仲裁先于打断：mid 在跑时发 rear，期望 -3 且 mid 不受影响 */
    "seq_status",            /* 期望仍 state=1 / One-key=RIGHT_MIDDLE / limit=2 */
    "seq_abort",
    "t_sim swing -1030",     /* 超左 −103° */
    "t_seq_case 5",          /* 三种一键均期望 0 OK */
    "seq_status",
    "seq_abort",
    "t_sim swing -500",
    "t_seq_run mid",         /* 启动一条用来验证手动打断 */
    "t_sim key up hold",     /* 手动键 → 下一个 tick 判中断 */
    "t_sim key none",
    "seq_status",            /* 期望 state=4 / Abort reason=INTERRUPT */
    RT_NULL
};

/* M14 阀故障 500 ms 去抖（P4 / B-5） */
static const char *const s_seq_m14[] = {
    "t_valve_fb on",         /* 去抖开启（默认 500ms） */
    "t_valve_fb ms 500",
    "t_sim valve 0 0",       /* 清故障取基线 */
    "t_lift",                /* 期望 IDLE / out=0x00 / fault 全 0 */
    "t_sim valve 1 0",       /* 上升阀开路 → 开始计时 */
    "t_valve_fb stat",       /* 1s 后：raw=1 lat=1，state=FAULT_VALVE */
    "t_lift",                /* 期望 out=0x00、valve_up=1、fault_raw up=1 */
    "t_sim valve 2 0",       /* 开路→短路：已确认，不应重复投事件 */
    "t_valve_fb stat",       /* 期望 raw=2 仍 lat=1，仍 FAULT_VALVE */
    "t_sim valve 0 0",
    "t_lift",                /* 期望自动恢复 IDLE / out=0x00 */
    "t_seq_auto 0",
    "t_sim swing -500",
    "t_seq_run mid",         /* 起一条一键，准备验证"故障中止一键" */
    "seq_status",            /* 期望 state=1 / RIGHT_MIDDLE */
    "t_sim valve 2 0",       /* 一键中注入短路 → 500ms 后确认 → 序列中止 */
    "seq_status",            /* 期望 state=4 / Abort reason=INTERRUPT */
    "t_lift",                /* 期望 FAULT_VALVE / out=0x00 */
    "t_sim valve 0 0",
    "t_valve_fb off",        /* 切旧行为做对比 */
    "t_sim valve 1 0",
    "t_lift",                /* 期望"一帧即报"：立即 FAULT_VALVE */
    "t_sim valve 0 0",
    "t_valve_fb on",
    "t_valve_fb test",       /* 时序自检：400ms 不报 / 500~560ms 报 / 立即清除（约 2s） */
    RT_NULL
};

/* M15 限位后 5 s 保持（P6 / B-7）
 * 判据：到上限后阀继续输出 5 s（t_lift 的 hold remaining 递减），6 s 时 IDLE/out=0；
 *       参数改 2000 后行为随之变；保持期间反向按键可立即脱离。 */
static const char *const s_seq_m15[] = {
    "t_lift_sens",
    "t_sim lift 2203",
    "t_sim key up hold",
    "t_lift",                /* 期望 UP_RUNNING / out=0x01 */
    "t_sim lift 3768",       /* 到上限（42.9 deg） */
    "t_lift",                /* 1 s：LIMIT_HOLDING / out=0x01 / remaining≈4xxx */
    "t_lift",                /* 2 s */
    "t_lift",                /* 3 s */
    "t_lift",                /* 4 s */
    "t_lift",                /* 5 s 边界 */
    "t_lift",                /* 6 s：期望回 IDLE / out=0x00 */
    "t_sim key none",
    "t_cfg set lift_hold_ms 2000",
    "t_sim lift 2203",
    "t_sim key up hold",
    "t_sim lift 3768",
    "t_lift",                /* 期望 LIMIT_HOLDING / remaining≈1xxx（参数生效） */
    "t_sim key up release",
    "t_sim key down hold",   /* 反向按键应立即可脱离 */
    "t_lift",                /* 期望 DOWN_RUNNING / out=0x02 */
    "t_sim key none",
    "t_cfg set lift_hold_ms 5000",
    "t_sim lift 2203",
    RT_NULL
};

/* M16 手动过零停车（P6 / B-8，TC-MAN-004）
 * 判据：到 0° 停车 + 锁刹车；保持按住不继续；松开重按才继续；
 *       反向（+侧转回 0）同样停；swing_zero_stop=0 时可直穿。 */
static const char *const s_seq_m16[] = {
    "t_swing",               /* zero_stop=1 */
    "t_sim swing -500",      /* -50 deg */
    "t_sim key left hold",
    "t_swing",               /* 期望 LEFT_RUNNING */
    "t_sim swing -100",      /* -10 deg，仍在左侧 */
    "t_sim swing 0",         /* 进入 0 deg → 应停车 */
    "t_swing",               /* 期望 IDLE（键仍按着） */
    "t_swing",               /* 1 s 后仍 IDLE：不得自行继续 */
    "t_sim key none",
    "t_sim key left hold",   /* 松开重按 → 应继续转左 */
    "t_swing",               /* 期望 LEFT_RUNNING */
    "t_sim swing -800",
    "t_sim key none",
    "t_sim swing 800",       /* +80 deg，右侧 */
    "t_sim key right hold",
    "t_swing",               /* 期望 RIGHT_RUNNING */
    "t_sim swing 0",         /* 从右侧回 0 → 应停车 */
    "t_swing",               /* 期望 IDLE */
    "t_sim key none",
    "t_cfg set swing_zero_stop 0",
    "t_sim swing -500",
    "t_sim key left hold",
    "t_swing",               /* LEFT_RUNNING */
    "t_sim swing 0",         /* 对照：关掉开关后过零不停 */
    "t_swing",               /* 期望仍 LEFT_RUNNING */
    "t_sim key none",
    "t_cfg set swing_zero_stop 1",
    "t_sim swing 0",
    RT_NULL
};

/* M13 一键回位：已在零位只下降 + 下降 2 s 保持（P7 / B-4）
 * 覆盖 TC-ONE-008/010/011。
 *
 * 判据：
 *   A 段（回转在 0°±2°）——只走 2 步，不升不转：
 *     [Seq] home: swing at 0.0 deg (within +-2 deg), descend only
 *     Enter step 0: Descend to zero → DOWN_RUNNING / out=0x02
 *   到 0° 后（keep_lift）：
 *     +1 s 仍 out=0x02（阀没被关）→ +3 s 变 out=0x00 / IDLE（2 s 到点失电）
 *   B 段（回转 −60°）——完整五步：
 *     Raise to +25 → Move swing to zero → Raise to +34 → Descend to zero → Hold
 *     第 4 步（Descend）结束时同样保持 2 s：+1 s 通电、+3 s 失电
 *
 * 角度注入值（zero=2203 mV，0.2747 °/mV）：
 *   2203 → 0.0°   3113 → +25.0°（LIFT_GE 250 刚好满足）   3441 → +34.0° */
static const char *const s_seq_m13[] = {
    "t_seq_auto 0",
    "t_sim key none",

    /* ---- A 段：回转已在零位（TC-ONE-011）---- */
    "t_sim lift 3768",       /* 举升 42.9 deg，模拟从上方归位 */
    "t_sim swing 0",         /* 回转 0 deg，落在 +-2 deg 内 */
    "t_seq_run home",        /* 期望 descend only + Enter step 0: Descend to zero */
    "seq_status",            /* 期望 step=0（列表只有 2 步） */
    "t_lift",                /* 期望 DOWN_RUNNING / out=0x02 */
    "t_sim lift 2203",       /* 到 0 deg -> 步 0 完成，keep_lift 生效 */
    "t_lift",                /* +1 s：仍 out=0x02  <== 关键：到 0° 后仍在输出 */
    "seq_status",            /* 2 s 边界，不判（step=1 或已 DONE） */
    "t_lift",                /* +3 s：IDLE / out=0x00  <== 关键：2 s 后失电 */
    "seq_status",            /* 期望 DONE / Complete=1 */

    /* ---- B 段：回转不在零位，走完整五步（TC-ONE-008/010）---- */
    "t_sim swing -600",      /* -60 deg */
    "t_sim lift 2203",
    "t_seq_run home",        /* 不应打印 descend only；Enter step 0: Raise to +25 */
    "seq_status",            /* 期望 step=0 Raise to +25 */
    "t_sim lift 3113",       /* +25.0 deg -> 步 0 完成 */
    "seq_status",            /* 期望 step=1 Move swing to zero */
    "t_sim swing 0",         /* 回转到位 -> 步 1 完成 */
    "seq_status",            /* 期望 step=2 Raise to +34 */
    "t_sim lift 3441",       /* +34.0 deg -> 步 2 完成 */
    "seq_status",            /* 期望 step=3 Descend to zero */
    "t_lift",                /* 期望 DOWN_RUNNING / out=0x02 */
    "t_sim lift 2203",       /* 到 0 deg -> 步 3 完成，keep_lift 生效 */
    "t_lift",                /* +1 s：仍 out=0x02  <== 关键 */
    "seq_status",            /* 2 s 边界，不判 */
    "t_lift",                /* +3 s：IDLE / out=0x00  <== 关键 */
    "seq_status",            /* 期望 DONE / Complete=1 */
    RT_NULL
};

/* M17 状态帧 + 灯效（P8）
 * ------------------------------------------------------------
 * 覆盖：TC-ONE-017（work_mode）、TC-LED-001~010 的主要分支、
 *       状态帧 8 个字段的真实值、D-1 之外的灯色优先级。
 *
 * 为什么不用 CAN 仪：t_sta / t_led 打印的是发送缓存 + 解码，
 * 与是否真的发帧无关。CAN_TX_EN_*=0 时输出不变（只把 send: 标 OFF）。
 *
 * 分段与判据：
 *   A 基线      —— 角度 0/0 与真实举升角、mode=000、motor=000、全绿
 *   B 手动      —— mode=001、motor=001、lift=001；左右/升降键黄
 *   C 阀故障    —— 开路红闪 / 短路红常亮；状态帧 up_valve=01→10
 *   D 一键      —— mode=010、mid 键绿闪；abort 后回绿
 *   E 传感器    —— 2 s 后 sensor 位=1、传感器灯红；恢复后回绿
 *   F 编码器    —— enc 位=1；左右键与电机灯红闪（t_encfault 注入）
 *   G 通信静默  —— comm 位=1；控制器灯红闪（3 s > 2 s，整车会进 FAULT）
 *   H 欠压      —— power 位=01；恢复后回 00
 *   收尾        —— t_veh_clr 复位整车，避免后续脚本被门禁拦
 *
 * 本脚本 1 s 步长，2 s 类判据只能看"已/未确认"，精确时序见 m8。
 * 前置：t_bench（已起车、cansilent 0、13.2 V）。 */
static const char *const s_seq_m17[] = {
    "t_seq_auto 0",
    "t_sim key none",

    /* ---- A 基线 ---- */
    "t_sta",                 /* A: mode=000 standby, motor=000, lift=000 */
    "t_led",                 /* A: 10 键绿（calib 黄）、三灯绿、backlight=1 */

    /* ---- B 手动：黄灯 + 状态位 ---- */
    "t_sim swing 1200",
    "t_sim lift 2203",
    "t_sim key left hold",
    "t_sta",                 /* B: mode=001(manual) motor=001(left) swing=120.0 */
    "t_led",                 /* B: left=0x2 yellow <- output on */
    "t_sim key none",
    "t_sim key up hold",
    "t_led",                 /* B: up=0x2 yellow */
    "t_sta",                 /* B: lift=001(up) */
    "t_sim key none",

    /* ---- C 阀故障：去抖 500ms，1s 后已确认 ---- */
    "t_sim valve 1 0",       /* C: 上升阀开路 */
    "t_sta",                 /* C: up_valve=01(open) */
    "t_led",                 /* C: up=0xC red-blink <- valve open */
    "t_sim valve 2 0",       /* C: 转短路（已确认，不重复投事件） */
    "t_led",                 /* C: up=0x4 red <- valve short */
    "t_sim valve 0 0",       /* C: 清除（立即） */
    "t_led",                 /* C: up 回 0x1 green */

    /* ---- D 一键：绿闪 + work_mode ---- */
    "t_sim swing -500",
    "t_seq_run mid",         /* D: 启动 right_middle */
    "t_sta",                 /* D: mode=010(onekey-expand) */
    "t_led",                 /* D: mid=0x9 green-blink <- sequence running */
    "seq_abort",
    "t_led",                 /* D: mid 回 0x1 green */

    /* ---- E 传感器 2 s：状态位 + 传感器灯 ---- */
    "t_sim lift 4800",
    "t_lift",                /* E: +1s 未确认（sens timing=1 latched=0） */
    "t_lift",                /* E: +2s 边界（约1950ms）仍未确认，属正常 */
    "t_led",                 /* E: +3s 已确认：sensor=0x4 red <- sensor fault */
    "t_sta",                 /* E: sensor=1(FAULT)，角度冻结 */
    "t_sim lift 2203",
    "t_lift",                /* E: +1s 恢复计时中（仍 FAULT_SENSOR） */
    "t_lift",                /* E: +2s 边界，仍故障 */
    "t_led",                 /* E: +3s 已恢复：sensor 回 0x1 green */

    /* ---- F 编码器故障：红闪（t_encfault 只置标志，不动状态机） ---- */
    "t_encfault 1",
    "t_sta",                 /* F: enc=1(FAULT) */
    "t_led",                 /* F: left/right=0xC、motor=0xC <- encoder fault */
    "t_encfault 0",
    "t_led",                 /* F: 回绿 */

    /* ---- G 通信静默：控制器灯红闪 ----
     * 注意：本段 4 步 = 3 s，已超过 veh_fault_ms(2000)，整车会进
     * FAULT(degrade=MANUAL-ONLY)。这是预期，脚本末尾用 t_veh_clr 复位。 */
    "t_sim cansilent 1",
    "t_sta",                 /* G: comm=1(FAULT) */
    "t_led",                 /* G: ctrl=0xC red-blink <- comm fault */
    "t_sim cansilent 0",
    "t_led",                 /* G: ctrl 回黄（未校准），comm=0 */

    /* ---- H 欠压：会触发急停与存档，放在最后 ---- */
    "t_sim bus 10000",
    "t_sta",                 /* H: power=01(undervoltage) */
    "t_sim bus 13200",
    "t_sta",                 /* H: power 回 00(normal) */

    /* ---- 收尾：G 段让整车进了 FAULT，必须清掉，否则后续脚本被门禁拦 ----
     * 注意 t_veh_clr 只是 force_reset 回 OFF；此时 ACC 仍 on、母线仍 13.2V，
     * 下一个 50ms tick 会立刻重新 ACC_ON → STARTED(functional 上就是"可用")。
     * 所以日志里看到 "force reset -> OFF" 紧接 "-> ACC_ON / STARTED" 是正常的，
     * 这不是脚本错误，判据看 t_sta 的 mode=000、power=00。 */
    "t_veh_clr",
    "t_sta",                 /* 收尾：mode=000、power=00、comm=0 */
    "t_sim swing 0",
    RT_NULL
};

/* M19 编码器解算故障：上报 / 恢复 / 手动放行（2026-11-23）
 * 编号说明：m18 留给 P9 校准状态机，m20 留给 P11 离合，故本脚本取 m19。
 * ------------------------------------------------------------
 * 验证的是「解算失效 → encoder_fault」这条真实链路，不是 t_encfault 注入。
 * 触发手段：`t_sim clear swing` 关掉台架模拟，swing_angle_is_solve_valid()
 * 回到真实解算（台架上无绝对编码器 ⇒ 无效）。
 *
 * 判据：
 *   A 基线   —— 角度 60.0°、position=MIDDLE、enc_rep=0、t_sta enc=0
 *   B 失效   —— [Swing] angle solve INVALID / encoder_fault -> 1
 *               [Swing] Enter FAULT state；t_sta enc=1；t_led left/right/motor=0xC
 *               t_seq_run mid 被前置条件拒（-2）
 *   C 恢复   —— [Swing] angle solve recovered；FAULT → IDLE；t_sta enc=0；灯回绿
 *   D 故障中手动 —— 再次失效后按左键应能 LEFT_RUNNING（Q30-a，手动放行）
 *
 * 注意：B 段 t_seq_run mid 被拒是**目的**，不是失败。旧实装这里会接受，
 * 然后序列因 SWING_EVT_SEQ_* 在 FAULT 态无跳转而空跑超时。 */
static const char *const s_seq_m19[] = {
    "t_seq_auto 0",
    "t_sim key none",

    /* ---- A 基线：先让解算有效 ---- */
    "t_swing",               /* A: 初始 IDLE、enc_rep=0 */
    "t_sim swing 600",       /* +60.0 deg，解算有效 */
    "t_swing",               /* A: MIDDLE, enc_rep=0 */
    "t_sta",                 /* A: B1 enc=0 */

    /* ---- B 解算失效：上报 + 状态机 + 显示 + 一键被拒 ---- */
    "t_sim clear swing",     /* B: 关闭模拟 -> 真实解算（台架无效） */
    "t_swing",               /* B: position UNKNOWN、enc_rep=1 */
    "t_sta",                 /* B: B1 enc=1 */
    "t_led",                 /* B: left/right/motor=0xC red-blink */
    "t_seq_run mid",         /* B: -2 rejected (precondition: encoder fault) */

    /* ---- C 恢复：清标志 + FAULT -> IDLE ---- */
    "t_sim swing 600",       /* C: 解算恢复 */
    "t_swing",               /* C: IDLE、enc_rep=0 */
    "t_sta",                 /* C: B1 enc=0 */
    "t_led",                 /* C: 回绿 */

    /* ---- D 故障中手动仍可动（Q30-a）---- */
    "t_sim clear swing",     /* D: 再次失效 -> FAULT */
    "t_sim key left hold",
    "t_swing",               /* D: LEFT_RUNNING（角度不可信也能转回） */
    "t_sim key none",
    "t_sim swing 600",       /* D: 恢复 */
    "t_sim swing 0",
    RT_NULL
};

/* M18 校准状态机（P9 / B-10，选项 A）
 * 前置：t_bench（STARTED）。sim long 会真跑 FOC_PROC_ENC_ALIGN，转子会动一下。
 * 次数口径：进 EXEC 计 1 次，每上电周期 2 次；PREP 超时 / <2s 松开不消耗。
 * 用例覆盖：PREP 超时 / <2s 松开失败 / 成功路径（灯+落盘）/ 次数上限 /
 *           一键运行中拒绝（TC-CAL-011）。 */
static const char *const s_seq_m18[] = {
    "t_calib status",        /* 1  基线：IDLE count 0/2 calibrated=0 */
    "t_calib sim dbl",       /* 2  → PREP（黄） */
    "t_calib status",        /* 3  PREP */
    "t_calib status",        /* 4  */
    "t_calib status",        /* 5  */
    "t_calib status",        /* 6  */
    "t_calib status",        /* 7  */
    "t_calib status",        /* 8  5s 到 → IDLE（prep timeout） */
    "t_calib sim dbl",       /* 9  → PREP */
    "t_calib sim click",     /* 10 → ARMED（绿闪） */
    "t_calib sim up",        /* 11 <2s 松开 → FAIL（红） */
    "t_calib status",        /* 12 FAIL / short press */
    "t_calib sim dbl",       /* 13 → PREP */
    "t_calib sim click",     /* 14 → ARMED */
    "t_calib sim long",      /* 15 → EXEC attempt 1（真跑对齐） */
    "t_calib status",        /* 16 EXEC / DONE */
    "t_calib status",        /* 17 */
    "t_calib status",        /* 18 IDLE calibrated=1 */
    "t_swing",               /* 19 筒角 0.0（zero_here 生效） */
    "t_param values",        /* 20 fast valid=0x03 tube=0 calib_cnt=1 */
    "t_calib start",         /* 21 attempt 2 → EXEC */
    "t_calib status",        /* 22 */
    "t_calib status",        /* 23 IDLE */
    "t_calib start",         /* 24 第 3 次 → reject count limit */
    "t_calib status",        /* 25 FAIL / count limit */
    "t_seq_run mid",         /* 26 一键在跑 */
    "t_calib start",         /* 27 reject onekey running（TC-CAL-011） */
    "seq_abort",             /* 28 */
    "t_calib status",        /* 29 */
    RT_NULL
};

static const test_seq_t s_seqs[] = {
    { "m3", "lift: angle/valve/limit",                  s_seq_m3 },
    { "m4", "swing: angle/brake/limit",                 s_seq_m4 },
    { "m5", "one-key: trigger chain (P2)",              s_seq_m5 },
    { "m6", "valve fault: open/short/recover",          s_seq_m6 },
    { "m7", "teeth: 13/14/11, dedup, clear",            s_seq_m7 },
    { "m9", "undervoltage: enter/gate/recover/save",       s_seq_m9 },
    { "m9a","UV + encoder align (batch 3/4 only)",       s_seq_m9a },
    { "m10","comm fault: 300ms comm/lamp + 2s veh FAULT", s_seq_m10 },
    { "m11","one-key arbitration (P3)",                 s_seq_m11 },
    { "m14","valve fault 500ms debounce (P4)",          s_seq_m14 },
    { "m8", "lift sensor 2s confirm/recover (P5)",      s_seq_m8 },
    { "m15","lift limit 5s hold (P6/B-7)",              s_seq_m15 },
    { "m16","swing manual zero-stop (P6/B-8)",          s_seq_m16 },
    { "m13","home: zero-only descend + 2s hold (P7/B-4)", s_seq_m13 },
    { "m17","status frame + led effects (P8)",           s_seq_m17 },
    { "m19","encoder solve fault: report/recover",      s_seq_m19 },
    { "m18","calib state machine (P9/B-10, opt A)",      s_seq_m18 },
};

#define SEQ_COUNT  ((uint32_t)(sizeof(s_seqs) / sizeof(s_seqs[0])))
#define SEQ_CMD_SZ 96U

static void seq_list(void)
{
    uint32_t i;

    rt_kprintf("[seq] usage: t_seq <name|num> [name|num ...] [interval_ms]\n");
    rt_kprintf("[seq]   e.g. t_seq m18 3 4 500  (num -> m<num>)\n");
    for (i = 0U; i < SEQ_COUNT; i++) {
        rt_kprintf("[seq]   %-4s %s\n", s_seqs[i].name, s_seqs[i].desc);
    }
}

static const test_seq_t *seq_find(const char *arg)
{
    uint32_t i;
    char alt[8];

    for (i = 0U; i < SEQ_COUNT; i++) {
        if (strcmp(arg, s_seqs[i].name) == 0) {
            return &s_seqs[i];
        }
    }

    /* 纯数字参数按 m<数字> 解析，便于 t_seq m18 3 4 */
    if ((arg[0] >= '0') && (arg[0] <= '9')) {
        (void)rt_snprintf(alt, sizeof(alt), "m%s", arg);
        for (i = 0U; i < SEQ_COUNT; i++) {
            if (strcmp(alt, s_seqs[i].name) == 0) {
                return &s_seqs[i];
            }
        }
    }

    return RT_NULL;
}

static void seq_run_one(const test_seq_t *seq, uint32_t interval,
                        uint32_t index, uint32_t count)
{
    uint32_t i;
    uint32_t n = 0U;
    uint32_t total;
    char buf[SEQ_CMD_SZ];

    while (seq->steps[n] != RT_NULL) {
        n++;
    }
    total = (n * interval) / 1000U;

    if (count > 1U) {
        rt_kprintf("\n[seq] ===== %lu/%lu %s =====\n",
                   (unsigned long)index, (unsigned long)count, seq->name);
    }

    rt_kprintf("[seq] %s: %lu steps, %lums apart (~%lus total)\n",
               seq->name, (unsigned long)n, (unsigned long)interval,
               (unsigned long)total);

    for (i = 0U; i < n; i++) {
        size_t len = strlen(seq->steps[i]);

        if (len >= (size_t)SEQ_CMD_SZ) {
            len = (size_t)SEQ_CMD_SZ - 1U;
        }
        memcpy(buf, seq->steps[i], len);
        buf[len] = '\0';

        rt_kprintf("\n[seq] (%lu/%lu) %s\n",
                   (unsigned long)(i + 1U), (unsigned long)n, buf);

        (void)msh_exec(buf, (rt_size_t)len);
        rt_thread_mdelay(interval);
    }

    rt_kprintf("[seq] %s done\n", seq->name);
}

static void t_seq(int argc, char **argv)
{
    const test_seq_t *batch[SEQ_BATCH_MAX];
    uint32_t count = 0U;
    uint32_t interval = 1000U;
    int32_t v;
    int last = argc - 1;
    int i;

    if (argc < 2) {
        seq_list();
        return;
    }

    /* 最后一个 ≥100 的数字是步间隔；其余参数都当脚本名/编号 */
    if ((sim_parse_i32(argv[last], &v) == 0) && (v >= 100) && (v <= 10000)) {
        interval = (uint32_t)v;
        last--;
    }
    if (last < 1) {
        seq_list();
        return;
    }

    for (i = 1; i <= last; i++) {
        const test_seq_t *seq = seq_find(argv[i]);

        if (seq == RT_NULL) {
            rt_kprintf("[seq] unknown script: %s\n", argv[i]);
            seq_list();
            return;
        }
        if (count >= SEQ_BATCH_MAX) {
            rt_kprintf("[seq] too many scripts (max %u)\n",
                       (unsigned)SEQ_BATCH_MAX);
            return;
        }
        batch[count++] = seq;
    }

    if (count > 1U) {
        rt_kprintf("[seq] batch: %lu scripts, %lums apart\n",
                   (unsigned long)count, (unsigned long)interval);
    }

    for (i = 0; i < (int)count; i++) {
        seq_run_one(batch[i], interval, (uint32_t)i + 1U, count);
    }

    if (count > 1U) {
        rt_kprintf("[seq] batch done: %lu scripts\n", (unsigned long)count);
    }
}

MSH_CMD_EXPORT_ALIAS(t_seq, t_seq, run scripted test sequence);

static void t_sim(int argc, char **argv)
{
    input_sim_src_t src;
    int32_t v;
    int32_t v2;
    uint16_t bit;

    if (argc < 2) {
        sim_dump();
        return;
    }

    /* ---- t_sim status：三态总览（REAL/SIM/OFF）---- */
    if (strcmp(argv[1], "status") == 0) {
        input_sim_status();
        return;
    }

    /* ---- t_sim clear [name] ---- */
    if (strcmp(argv[1], "clear") == 0) {
        if (argc < 3) {
            input_sim_clear_all();
            rt_kprintf("[sim] all sources -> real input\n");
            return;
        }
        if (input_sim_name_to_src(argv[2], &src) != 0) {
            rt_kprintf("[sim] unknown source: %s\n", argv[2]);
            return;
        }
        input_sim_clear(src);
        rt_kprintf("[sim] %s -> real input\n", s_names[src]);
        return;
    }

    /* ---- t_sim panel 0|8|10 ---- */
    if (strcmp(argv[1], "panel") == 0) {
        if ((argc < 3) || (sim_parse_i32(argv[2], &v) != 0)) {
            rt_kprintf("[sim] usage: t_sim panel 0|8|10\n");
            return;
        }
        s_panel_type = (uint8_t)((v == 8) ? 8 : ((v == 10) ? 10 : 0));
        rt_kprintf("[sim] panel type -> %u (applied to next 0x18FF8380 frame)\n",
                   (unsigned)s_panel_type);
        return;
    }

    if (input_sim_name_to_src(argv[1], &src) != 0) {
        rt_kprintf("[sim] unknown source: %s\n", argv[1]);
        rt_kprintf("[sim] sources: acc bus swing lift key rpm valve teeth cansilent\n");
        return;
    }

    switch (src) {
    /* ---- ACC：on/off/auto ---- */
    case INPUT_SIM_ACC:
        if (argc < 3) {
            rt_kprintf("[sim] usage: t_sim acc on|off|auto\n");
            return;
        }
        if (strcmp(argv[2], "on") == 0) {
            (void)input_sim_set(src, 0);        /* PB0 低 = ON */
            rt_kprintf("[sim] acc -> ON (raw level 0), debounce still applies\n");
        } else if (strcmp(argv[2], "off") == 0) {
            (void)input_sim_set(src, 1);        /* PB0 高 = OFF */
            rt_kprintf("[sim] acc -> OFF (raw level 1), debounce still applies\n");
        } else if (strcmp(argv[2], "auto") == 0) {
            input_sim_clear(src);
            rt_kprintf("[sim] acc -> real PB0\n");
        } else {
            rt_kprintf("[sim] usage: t_sim acc on|off|auto\n");
        }
        return;

    /* ---- KEY：name [hold|pulse|release] ---- */
    case INPUT_SIM_KEY: {
        uint16_t mask = (uint16_t)s_value[src];

        if (argc < 3) {
            rt_kprintf("[sim] key names: up down left right mid rear home calib ccomb csep\n");
            rt_kprintf("[sim] usage: t_sim key <name> [hold|pulse|release] | none\n");
            return;
        }

        if (strcmp(argv[2], "none") == 0) {
            input_sim_clear(src);
            rt_kprintf("[sim] key released (all)\n");
            return;
        }

        /* 【P3】支持 'mid+rear+home'：**同一帧**注入多个按键位。
         * 用于验证"三键同按全部不启动"（TC-PRI-002）与"手动优先"（PRI-003/007）。
         * 注意：分多次 `t_sim key xxx hold` 是逐帧累加，等价于"先后按下"，
         * 不能用来构造同帧多键。 */
        if (strchr(argv[2], '+') != NULL) {
            char        tok[16];
            uint16_t    bits = 0U;
            const char *p    = argv[2];

            while (*p != '\0') {
                uint32_t n = 0U;
                uint16_t b = 0U;

                while ((p[n] != '\0') && (p[n] != '+') && (n < (uint32_t)(sizeof(tok) - 1U))) {
                    tok[n] = p[n];
                    n++;
                }
                tok[n] = '\0';

                if ((n == 0U) || (sim_key_bit(tok, &b) != 0)) {
                    rt_kprintf("[sim] unknown key: %s\n", (n == 0U) ? "?" : tok);
                    return;
                }
                bits |= b;

                if (p[n] == '+') {
                    p += (n + 1U);
                } else {
                    break;
                }
            }

            mask = (uint16_t)(mask | bits);
            (void)input_sim_set(src, (int32_t)mask);
            rt_kprintf("[sim] key %s hold, mask=0x%04X\n", argv[2], (unsigned)mask);
            return;
        }

        if (sim_key_bit(argv[2], &bit) != 0) {
            rt_kprintf("[sim] unknown key: %s\n", argv[2]);
            return;
        }

        if ((argc >= 4) && (strcmp(argv[3], "release") == 0)) {
            mask = (uint16_t)(mask & (uint16_t)~bit);
            (void)input_sim_set(src, (int32_t)mask);
            if (mask == 0U) {
                input_sim_clear(src);
            }
            rt_kprintf("[sim] key %s released, mask=0x%04X\n",
                       argv[2], (unsigned)mask);
            return;
        }

        mask = (uint16_t)(mask | bit);
        (void)input_sim_set(src, (int32_t)mask);

        if ((argc >= 4) && (strcmp(argv[3], "pulse") == 0)) {
            s_pulse_left = SIM_PULSE_TICKS;
            rt_kprintf("[sim] key %s pulse %dms, mask=0x%04X\n",
                       argv[2], (int)(SIM_PULSE_TICKS * SIM_TICK_MS), (unsigned)mask);
        } else {
            rt_kprintf("[sim] key %s hold, mask=0x%04X\n", argv[2], (unsigned)mask);
        }
        return;
    }

    /* ---- VALVE：up_fault down_fault [up_open] [down_open] ---- */
    case INPUT_SIM_VALVE: {
        uint32_t packed = 0U;

        if ((argc < 4) ||
            (sim_parse_i32(argv[2], &v) != 0) ||
            (sim_parse_i32(argv[3], &v2) != 0)) {
            rt_kprintf("[sim] usage: t_sim valve <up_fault 0-3> <down_fault 0-3> [up_open 0-1] [down_open 0-1]\n");
            return;
        }

        packed |= (uint32_t)(v & 0x3);
        packed |= (uint32_t)((v2 & 0x3) << 2);

        if (argc >= 5) {
            int32_t o;
            if (sim_parse_i32(argv[4], &o) != 0) {
                o = 0;
            }
            packed |= (uint32_t)((o & 0x1) << 4);
        }
        if (argc >= 6) {
            int32_t o;
            if (sim_parse_i32(argv[5], &o) != 0) {
                o = 0;
            }
            packed |= (uint32_t)((o & 0x1) << 5);
        }

        (void)input_sim_set(src, (int32_t)packed);
        rt_kprintf("[sim] valve frame injected: up_fault=%d down_fault=%d packed=0x%02X\n",
                   (int)(v & 0x3), (int)(v2 & 0x3), (unsigned)packed);
        return;
    }

    /* ---- 其余：一个整数 ---- */
    default:
        if ((argc < 3) || (sim_parse_i32(argv[2], &v) != 0)) {
            if (src == INPUT_SIM_TEETH) {
                rt_kprintf("[sim] usage: t_sim teeth 0|1|2|3\n");
            } else {
                rt_kprintf("[sim] usage: t_sim %s <value>\n", s_names[src]);
            }
            return;
        }
        (void)input_sim_set(src, v);
        rt_kprintf("[sim] %s -> %d\n", s_names[src], (int)v);
        return;
    }
}

MSH_CMD_EXPORT_ALIAS(t_sim, t_sim, input simulation for bench test);

#endif /* INPUT_SIM_ENABLE */