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
 * 每步先打印 `[seq] (i/n) <命令>`，再执行，再等待。
 * 用 msh_exec() 执行原文，等于把命令逐条敲一遍（打印/行为完全一致）。
 * ============================================================ */
typedef struct {
    const char *name;
    const char *desc;
    const char *const *steps;      /* 以 RT_NULL 结尾 */
} test_seq_t;

/* M3 举升：角度链 / 传感器故障 / 手动阀输出 / 上限拦截 */
static const char *const s_seq_m3[] = {
    "t_sim lift 2203", "t_lift",
    "t_sim lift 3768", "t_lift",
    "t_sim lift 1111", "t_lift",
    "t_sim lift 4800", "t_lift",
    "t_sim lift 250",  "t_lift",
    "t_sim lift 4450", "t_lift",
    "t_sim lift 2203",
    "t_sim key up hold",    "t_lift",
    "t_sim key up release", "t_lift",
    "t_sim key down hold",  "t_lift",
    "t_sim key down release",
    "t_sim lift 3768", "t_sim key up hold", "t_lift", "t_sim key none",
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

/* M10 通信故障：静默 → FAULT → 恢复 → 清除（不写 Flash） */
static const char *const s_seq_m10[] = {
    "t_veh",
    "t_param info",         /* 基线：证明通信故障不产生 Flash 写 */
    "t_sim cansilent 1",    /* 500ms 判静默，2s 后 FAULT */
    "t_veh",                /* 约 +1s：fault_ms 已过半，state 仍非 FAULT */
    "t_veh",                /* 约 +2s：state=FAULT、started=0、word=0x1 */
    "t_sim cansilent 0",
    "t_veh",                /* fault_word 回 0，但 state 仍 FAULT（只能钥匙清） */
    "t_veh_clr",
    "t_veh",                /* 回 OFF，条件仍满足则自回 ACC_ON/STARTED */
    "t_param info",         /* 与基线一致：未写 Flash */
    RT_NULL
};

static const test_seq_t s_seqs[] = {
    { "m3", "lift: angle/fault/valve/limit",            s_seq_m3 },
    { "m4", "swing: angle/brake/limit",                 s_seq_m4 },
    { "m6", "valve fault: open/short/recover",          s_seq_m6 },
    { "m7", "teeth: 13/14/11, dedup, clear",            s_seq_m7 },
    { "m9", "undervoltage: enter/gate/recover/save",       s_seq_m9 },
    { "m9a","UV + encoder align (batch 3/4 only)",       s_seq_m9a },
    { "m10","comm fault: silent -> FAULT -> recover",   s_seq_m10 },
};

#define SEQ_COUNT  ((uint32_t)(sizeof(s_seqs) / sizeof(s_seqs[0])))
#define SEQ_CMD_SZ 96U

static void seq_list(void)
{
    uint32_t i;

    rt_kprintf("[seq] usage: t_seq <name> [interval_ms]\n");
    for (i = 0U; i < SEQ_COUNT; i++) {
        rt_kprintf("[seq]   %-4s %s\n", s_seqs[i].name, s_seqs[i].desc);
    }
}

static void t_seq(int argc, char **argv)
{
    const test_seq_t *seq = RT_NULL;
    uint32_t interval = 1000U;
    uint32_t i;
    uint32_t n = 0U;
    uint32_t total;
    char buf[SEQ_CMD_SZ];

    if (argc < 2) {
        seq_list();
        return;
    }

    for (i = 0U; i < SEQ_COUNT; i++) {
        if (strcmp(argv[1], s_seqs[i].name) == 0) {
            seq = &s_seqs[i];
            break;
        }
    }
    if (seq == RT_NULL) {
        rt_kprintf("[seq] unknown script: %s\n", argv[1]);
        seq_list();
        return;
    }

    if (argc >= 3) {
        int32_t v;
        if ((sim_parse_i32(argv[2], &v) == 0) && (v >= 100) && (v <= 10000)) {
            interval = (uint32_t)v;
        }
    }

    while (seq->steps[n] != RT_NULL) {
        n++;
    }
    total = (n * interval) / 1000U;

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