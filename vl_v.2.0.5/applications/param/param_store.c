/**
 * @file    param_store.c
 * @brief   业务参数存取层实现
 */

#include "param_store.h"
#include "param_manager.h"

#include "../sys_param/sys_param.h"
#include "../angle/mech_cfg.h"
#include "../angle/mech_cfg_store.h"
#include "../config/sys_config.h"
#include "../device_info/device_info.h"
#include "../mode/mode_config.h"
#include "../motor/motor.h"

#include <string.h>
#include <rtthread.h>

/* ============================================================
 * 记录实例
 * ============================================================ */
static param_motion_rec_t s_motion;
static param_system_rec_t s_system;
static param_am_rec_t     s_am;
static param_dev_rec_t    s_dev;
static param_fast_rec_t   s_fast;

static param_runtime_t s_rt_motion;
static param_runtime_t s_rt_system;
static param_runtime_t s_rt_am;
static param_runtime_t s_rt_dev;
static param_runtime_t s_rt_fast;

static const param_config_t s_cfg_motion = {
    PARAM_SEC_MOTION, PARAM_SEC_MOTION,
    sizeof(param_motion_rec_t), 0U,
    PARAM_MAGIC_MOTION, "motion"
};
static const param_config_t s_cfg_system = {
    PARAM_SEC_SYSTEM, PARAM_SEC_SYSTEM,
    sizeof(param_system_rec_t), 0U,
    PARAM_MAGIC_SYSTEM, "system"
};
static const param_config_t s_cfg_am = {
    PARAM_SEC_ANGLE_MECH, PARAM_SEC_ANGLE_MECH,
    sizeof(param_am_rec_t), 0U,
    PARAM_MAGIC_ANGLE_MECH, "angle_mech"
};
static const param_config_t s_cfg_dev = {
    PARAM_SEC_DEVICE, PARAM_SEC_DEVICE,
    sizeof(param_dev_rec_t), 0U,
    PARAM_MAGIC_DEVICE, "device"
};
static const param_config_t s_cfg_fast = {
    PARAM_SEC_FAST_START, PARAM_SEC_FAST_END,
    sizeof(param_fast_rec_t), PARAM_FAST_MAX_ERASE_LIFE,
    PARAM_MAGIC_FAST, "fast"
};

static uint8_t        s_inited = 0U;
static struct rt_mutex s_lock;
static struct rt_semaphore s_sem;
static volatile uint32_t s_pending = 0U;
static rt_thread_t    s_tid = RT_NULL;

/* ============================================================
 * sys_config 关键项
 * ============================================================ */
static const uint16_t s_motion_ids[PARAM_MOTION_CFG_COUNT] = {
    /* 电流采样 */
    CFG_ID_CUR_ZERO_BUS, CFG_ID_CUR_ZERO_W, CFG_ID_CUR_ZERO_V,
    CFG_ID_CUR_LSB_NV, CFG_ID_CUR_GAIN_X100,
    /* PWM / FOC */
    CFG_ID_PWM_FREQ_HZ, CFG_ID_PWM_DEADTIME_NS, CFG_ID_MOTOR_POLE_PAIRS,
    CFG_ID_MOTOR_RAMP_MS, CFG_ID_FOC_MAX_MODULATION,
    /* 转速窗口 */
    CFG_ID_SPEED_WINDOW_TICKS, CFG_ID_SPEED_TICK_HZ, CFG_ID_SPEED_DT_MAX_RATIO,
    /* 堵转保护 */
    CFG_ID_STALL_OC3_A10, CFG_ID_STALL_OC3_MS,
    CFG_ID_STALL_OC2_A10, CFG_ID_STALL_OC2_MS,
    CFG_ID_STALL_OC1_A10, CFG_ID_STALL_OC1_MS,
    CFG_ID_STALL_LOW_RPM, CFG_ID_STALL_LOW_MS,
    CFG_ID_STALL_EN_MIN_A10, CFG_ID_MOTOR_RATED_A10,
    CFG_ID_STALL_AUTO_CLEAR_MS,
};

static const uint16_t s_system_ids[PARAM_SYSTEM_CFG_COUNT] = {
    /* 整车启停 */
    CFG_ID_VEH_ENTER_RPM, CFG_ID_VEH_EXIT_RPM,
    CFG_ID_VEH_ENTER_VOLT_DV, CFG_ID_VEH_EXIT_VOLT_DV,
    CFG_ID_VEH_ENTER_MS, CFG_ID_VEH_EXIT_MS,
    CFG_ID_ACC_ENTER_MS, CFG_ID_ACC_EXIT_MS, CFG_ID_ACC_POLL_MS,
    CFG_ID_VEH_FAULT_MS, CFG_ID_VEH_CAN_SILENT_MS,
    /* 看门狗 */
    CFG_ID_WDT_CHECKIN_ENABLE, CFG_ID_WDT_TIMEOUT_MS, CFG_ID_WDT_IDLE_HOOK,
    /* 举升限位 */
    CFG_ID_LIFT_UPPER_X10, CFG_ID_LIFT_LOWER_X10,
    /* 机械选择子 */
    CFG_ID_MECH_ENC_SEL, CFG_ID_MECH_IIN_SEL, CFG_ID_MECH_IOUT_SEL,
    /* 回转方向极性 */
    CFG_ID_SWING_DIR_POLARITY,
};

/* ============================================================
 * 默认值
 * ============================================================ */
static void defaults_motion(uint8_t *buf)
{
    memset(buf, 0, sizeof(param_motion_rec_t));
}

static void defaults_system(uint8_t *buf)
{
    mode_config_t mc;
    param_system_rec_t *r = (param_system_rec_t *)buf;

    memset(r, 0, sizeof(*r));

    mode_config_default(&mc);
    r->mode.rec_magic       = PARAM_MODE_REC_MAGIC;
    r->mode.default_mode    = mc.default_mode;
    r->mode.allowed_modes   = mc.allowed_modes;
    r->mode.factory_lock    = mc.factory_lock;
    r->mode.cap_override    = mc.cap_override;
    r->mode.test_timeout_ms = mc.test_timeout_ms;
}

static void defaults_am(uint8_t *buf)
{
    static mech_table_t tbl;
    param_am_rec_t *r = (param_am_rec_t *)buf;

    memset(r, 0, sizeof(*r));
    r->payload.rec_magic = PARAM_AM_REC_MAGIC;

    mech_table_default(&tbl);
    r->payload.row   = tbl.row[0];
    r->payload.active = 0U;
    r->payload.count  = tbl.count;

    r->payload.enc_sel  = MECH_ENC_SEL_555657;
    r->payload.iin_sel  = MECH_IIN_SEL_216;
    r->payload.iout_sel = MECH_IOUT_SEL_65_13;

    r->payload.enc_zero_valid     = 0U;
    r->payload.teeth_type         = TEETH_UNSET;
    r->payload.is_calibrated      = 0U;
    r->payload.teeth_clear_action = TEETH_CLEAR_KEEP;
    r->payload.lift_zero_voltage_mv = 2203U;
}

static void defaults_dev(uint8_t *buf)
{
    param_dev_rec_t *r = (param_dev_rec_t *)buf;

    memset(r, 0, sizeof(*r));
    r->payload.rec_magic = DEVICE_INFO_REC_MAGIC;
}

static void defaults_fast(uint8_t *buf)
{
    param_fast_rec_t *r = (param_fast_rec_t *)buf;

    memset(r, 0, sizeof(*r));
    r->payload.rec_magic         = PARAM_FAST_REC_MAGIC;
    r->payload.abs_cpr           = 4096U;
    r->payload.enc_to_rotor_num  = 1U;
    r->payload.enc_to_rotor_den  = 1U;
    r->payload.pole_pairs        = MECH_DEF_POLE_PAIRS;
    r->payload.elec_dir          = 1U;
    r->payload.valid             = 0U;
}

/* ============================================================
 * 打包 / 解包
 * ============================================================ */
static void cfg_pack(void)
{
    uint16_t i;
    int32_t  v;

    for (i = 0U; i < PARAM_MOTION_CFG_COUNT; i++) {
        if (sys_config_get_i32(s_motion_ids[i], &v) == 0) {
            s_motion.cfg[i] = (uint32_t)v;
        }
    }
    for (i = 0U; i < PARAM_SYSTEM_CFG_COUNT; i++) {
        if (sys_config_get_i32(s_system_ids[i], &v) == 0) {
            s_system.cfg[i] = (uint32_t)v;
        }
    }
}

static void am_pack(void)
{
    uint16_t i;
    int32_t  v;

    s_am.payload.rec_magic = PARAM_AM_REC_MAGIC;

    if (g_mech.is_loaded != 0U) {
        s_am.payload.row    = g_mech.tbl.row[g_mech.tbl.active];
        s_am.payload.active = g_mech.tbl.active;
        s_am.payload.count  = g_mech.tbl.count;
    }

    if (sys_config_get_i32(CFG_ID_MECH_ENC_SEL, &v) == 0) {
        s_am.payload.enc_sel = (uint8_t)v;
    }
    if (sys_config_get_i32(CFG_ID_MECH_IIN_SEL, &v) == 0) {
        s_am.payload.iin_sel = (uint8_t)v;
    }
    if (sys_config_get_i32(CFG_ID_MECH_IOUT_SEL, &v) == 0) {
        s_am.payload.iout_sel = (uint8_t)v;
    }

    s_am.payload.enc_zero_valid     = g_sys_params.enc_zero_valid;
    s_am.payload.teeth_type         = g_sys_params.teeth_type;
    s_am.payload.is_calibrated      = g_sys_params.is_calibrated;
    s_am.payload.teeth_clear_action = g_sys_params.teeth_clear_action;
    s_am.payload.lift_zero_voltage_mv = g_sys_params.lift_zero_voltage_mv;

    for (i = 0U; i < 3U; i++) {
        s_am.payload.enc_zero_raw[i] = g_sys_params.enc_zero_raw[i];
    }
}

static void dev_pack(void)
{
    s_dev.payload.rec_magic = DEVICE_INFO_REC_MAGIC;

    memcpy(s_dev.payload.sn, g_device_info.sn, sizeof(s_dev.payload.sn));
    memcpy(s_dev.payload.name, g_device_info.name, sizeof(s_dev.payload.name));
    memcpy(s_dev.payload.sw_version_str, g_device_info.sw_version_str,
           sizeof(s_dev.payload.sw_version_str));

    s_dev.payload.model               = g_device_info.model;
    s_dev.payload.hw_version          = g_device_info.hw_version;
    s_dev.payload.sw_version          = g_device_info.sw_version;
    s_dev.payload.manufacture_date    = g_device_info.manufacture_date;
    s_dev.payload.first_boot_timestamp = g_device_info.first_boot_timestamp;
}

/* ============================================================
 * 保存线程
 * ============================================================ */
static int motor_is_busy(void)
{
    motor_state_t st = motor_get_state(&g_swing_motor);

    return (st == MOTOR_STATE_RUNNING ||
            st == MOTOR_STATE_RAMPING ||
            st == MOTOR_STATE_BRAKING) ? 1 : 0;
}

static void param_save_thread(void *arg)
{
    (void)arg;

    for (;;) {
        uint32_t mask;

        rt_sem_take(&s_sem, RT_WAITING_FOREVER);

        while (s_pending != 0U) {
            /* 运行中禁止保存：等停机后再落盘 */
            if (motor_is_busy() != 0) {
                rt_thread_mdelay(20);
                continue;
            }

            rt_enter_critical();
            mask = s_pending;
            s_pending = 0U;
            rt_exit_critical();

            (void)param_store_save_now(mask);
        }
    }
}

/* ============================================================
 * 初始化
 * ============================================================ */
int param_store_init(void)
{
    if (s_inited != 0U) {
        return 0;
    }

    rt_mutex_init(&s_lock, "param", RT_IPC_FLAG_PRIO);
    rt_sem_init(&s_sem, "param_sav", 0U, RT_IPC_FLAG_PRIO);

    (void)Param_Init(&s_cfg_motion, &s_rt_motion, (uint8_t *)&s_motion, defaults_motion);
    (void)Param_Init(&s_cfg_system, &s_rt_system, (uint8_t *)&s_system, defaults_system);
    (void)Param_Init(&s_cfg_am,     &s_rt_am,     (uint8_t *)&s_am,     defaults_am);
    (void)Param_Init(&s_cfg_dev,    &s_rt_dev,    (uint8_t *)&s_dev,    defaults_dev);
    (void)Param_Init(&s_cfg_fast,   &s_rt_fast,   (uint8_t *)&s_fast,   defaults_fast);

    s_tid = rt_thread_create("param_sav", param_save_thread, RT_NULL,
                             1024U, 12U, 10U);
    if (s_tid != RT_NULL) {
        rt_thread_startup(s_tid);
    }

    s_inited = 1U;

    rt_kprintf("[PARAM] init: motion=%s system=%s am=%s dev=%s fast=%s\n",
               s_rt_motion.defaults_written ? "default" : "flash",
               s_rt_system.defaults_written ? "default" : "flash",
               s_rt_am.defaults_written     ? "default" : "flash",
               s_rt_dev.defaults_written    ? "default" : "flash",
               s_rt_fast.defaults_written   ? "default" : "flash");
    return 0;
}

/* ============================================================
 * 保存
 * ============================================================ */
int param_store_save_now(uint32_t mask)
{
    int rc = 0;

    if (s_inited == 0U) {
        (void)param_store_init();
    }

    rt_mutex_take(&s_lock, RT_WAITING_FOREVER);

    if ((mask & PARAM_REQ_CFG) != 0U) {
        cfg_pack();
        if (Param_Save(&s_cfg_motion, &s_rt_motion, (uint8_t *)&s_motion) != PARAM_OK) {
            rc = -1;
        }
        if (Param_Save(&s_cfg_system, &s_rt_system, (uint8_t *)&s_system) != PARAM_OK) {
            rc = -1;
        }
    }

    if ((mask & PARAM_REQ_AM) != 0U) {
        am_pack();
        if (Param_Save(&s_cfg_am, &s_rt_am, (uint8_t *)&s_am) != PARAM_OK) {
            rc = -1;
        }
    }

    if ((mask & PARAM_REQ_DEV) != 0U) {
        dev_pack();
        if (Param_Save(&s_cfg_dev, &s_rt_dev, (uint8_t *)&s_dev) != PARAM_OK) {
            rc = -1;
        }
    }

    if ((mask & PARAM_REQ_FAST) != 0U) {
        if (Param_Save(&s_cfg_fast, &s_rt_fast, (uint8_t *)&s_fast) != PARAM_OK) {
            rc = -1;
        }
    }

    rt_mutex_release(&s_lock);
    return rc;
}

int param_store_request(uint32_t mask)
{
    if (s_inited == 0U) {
        (void)param_store_init();
    }

    rt_enter_critical();
    s_pending |= mask;
    rt_exit_critical();

    if (s_tid != RT_NULL) {
        rt_sem_release(&s_sem);
    } else {
        /* 线程创建失败时退化为同步保存 */
        (void)param_store_save_now(mask);
    }
    return 0;
}

/* ============================================================
 * ANGLE_MECH
 * ============================================================ */
int param_store_am_load(void)
{
    if (s_inited == 0U) {
        (void)param_store_init();
    }
    if (s_am.payload.rec_magic != PARAM_AM_REC_MAGIC) {
        return -1;
    }

    g_sys_params.rec_magic              = SYS_PARAM_REC_MAGIC;
    g_sys_params.lift_zero_voltage_mv   = s_am.payload.lift_zero_voltage_mv;
    g_sys_params.enc_zero_valid         = s_am.payload.enc_zero_valid;
    g_sys_params.teeth_type             = s_am.payload.teeth_type;
    g_sys_params.is_calibrated          = s_am.payload.is_calibrated;
    g_sys_params.teeth_clear_action     = s_am.payload.teeth_clear_action;
    g_sys_params.enc_zero_raw[0]        = s_am.payload.enc_zero_raw[0];
    g_sys_params.enc_zero_raw[1]        = s_am.payload.enc_zero_raw[1];
    g_sys_params.enc_zero_raw[2]        = s_am.payload.enc_zero_raw[2];
    return 0;
}

int param_store_am_save(void)
{
    return param_store_request(PARAM_REQ_AM);
}

int param_store_am_get_row(mech_cfg_t *row, uint8_t *active, uint8_t *count)
{
    if (s_inited == 0U) {
        (void)param_store_init();
    }
    if ((row == RT_NULL) || (s_am.payload.rec_magic != PARAM_AM_REC_MAGIC)) {
        return -1;
    }

    *row = s_am.payload.row;
    if (active != RT_NULL) { *active = s_am.payload.active; }
    if (count  != RT_NULL) { *count  = s_am.payload.count;  }
    return 0;
}

int param_store_am_set_row(const mech_cfg_t *row, uint8_t active, uint8_t count)
{
    if ((s_inited == 0U) || (row == RT_NULL)) {
        return -1;
    }

    rt_mutex_take(&s_lock, RT_WAITING_FOREVER);
    s_am.payload.row    = *row;
    s_am.payload.active = active;
    s_am.payload.count  = count;
    rt_mutex_release(&s_lock);
    return 0;
}

int param_store_am_was_default(void)
{
    return (s_rt_am.defaults_written != 0U) ? 1 : 0;
}

/* ============================================================
 * SYSTEM / MOTION
 * ============================================================ */
static void cfg_apply_one(uint16_t id, uint32_t raw)
{
    const cfg_def_t *d = sys_config_find_def(id);
    int32_t v = (int32_t)raw;

    if (d == RT_NULL) {
        return;
    }
    if ((v < d->min_val) || (v > d->max_val)) {
        return;
    }
    (void)sys_config_set_raw(id, v);
}

int param_store_cfg_load(void)
{
    uint16_t i;

    if (s_inited == 0U) {
        (void)param_store_init();
    }

    /* 上电时没有任何有效记录：记录里是 0，不能拿去覆盖默认值 */
    if ((s_rt_motion.defaults_written != 0U) ||
        (s_rt_system.defaults_written != 0U)) {
        return -1;
    }
    if ((s_motion.head != PARAM_MAGIC_MOTION) ||
        (s_system.head != PARAM_MAGIC_SYSTEM)) {
        return -1;
    }

    for (i = 0U; i < PARAM_MOTION_CFG_COUNT; i++) {
        cfg_apply_one(s_motion_ids[i], s_motion.cfg[i]);
    }
    for (i = 0U; i < PARAM_SYSTEM_CFG_COUNT; i++) {
        cfg_apply_one(s_system_ids[i], s_system.cfg[i]);
    }
    return 0;
}

int param_store_cfg_save(void)
{
    return param_store_request(PARAM_REQ_CFG);
}

int param_store_cfg_was_default(void)
{
    return ((s_rt_motion.defaults_written != 0U) ||
            (s_rt_system.defaults_written != 0U)) ? 1 : 0;
}

int param_store_mode_load(mode_config_t *out)
{
    if (s_inited == 0U) {
        (void)param_store_init();
    }
    if ((out == RT_NULL) || (s_system.mode.rec_magic != PARAM_MODE_REC_MAGIC)) {
        return -1;
    }

    out->rec_magic       = MODE_CONFIG_REC_MAGIC;
    out->default_mode    = s_system.mode.default_mode;
    out->allowed_modes   = s_system.mode.allowed_modes;
    out->factory_lock    = s_system.mode.factory_lock;
    out->cap_override    = s_system.mode.cap_override;
    out->test_timeout_ms = s_system.mode.test_timeout_ms;
    return 0;
}

int param_store_mode_save(const mode_config_t *in)
{
    if ((s_inited == 0U) || (in == RT_NULL)) {
        return -1;
    }

    rt_mutex_take(&s_lock, RT_WAITING_FOREVER);
    s_system.mode.rec_magic       = PARAM_MODE_REC_MAGIC;
    s_system.mode.default_mode    = in->default_mode;
    s_system.mode.allowed_modes   = in->allowed_modes;
    s_system.mode.factory_lock    = in->factory_lock;
    s_system.mode.cap_override    = in->cap_override;
    s_system.mode.test_timeout_ms = in->test_timeout_ms;
    rt_mutex_release(&s_lock);

    return param_store_request(PARAM_REQ_CFG);
}

/* ============================================================
 * DEVICE
 * ============================================================ */
int param_store_dev_load(device_info_t *out)
{
    if (s_inited == 0U) {
        (void)param_store_init();
    }
    if ((out == RT_NULL) || (s_dev.payload.rec_magic != DEVICE_INFO_REC_MAGIC)) {
        return -1;
    }

    memcpy(out->sn, s_dev.payload.sn, sizeof(out->sn));
    memcpy(out->name, s_dev.payload.name, sizeof(out->name));
    memcpy(out->sw_version_str, s_dev.payload.sw_version_str,
           sizeof(out->sw_version_str));

    out->model                = s_dev.payload.model;
    out->hw_version           = s_dev.payload.hw_version;
    out->sw_version           = s_dev.payload.sw_version;
    out->manufacture_date     = s_dev.payload.manufacture_date;
    out->first_boot_timestamp = s_dev.payload.first_boot_timestamp;
    out->rec_magic            = DEVICE_INFO_REC_MAGIC;
    return 0;
}

int param_store_dev_save(const device_info_t *in)
{
    if (in != RT_NULL) {
        /* 调用方已先写 g_device_info，dev_pack 以全局值为准；
         * 这里只为保持接口语义。 */
        (void)in;
    }
    return param_store_request(PARAM_REQ_DEV);
}

int param_store_dev_was_default(void)
{
    return (s_rt_dev.defaults_written != 0U) ? 1 : 0;
}

/* ============================================================
 * FAST
 * ============================================================ */
int param_fast_load(param_fast_payload_t *out)
{
    if (s_inited == 0U) {
        (void)param_store_init();
    }
    if ((out == RT_NULL) || (s_fast.payload.rec_magic != PARAM_FAST_REC_MAGIC)) {
        return -1;
    }

    *out = s_fast.payload;
    return 0;
}

int param_fast_save(const param_fast_payload_t *in)
{
    if ((s_inited == 0U) || (in == RT_NULL)) {
        return -1;
    }

    rt_mutex_take(&s_lock, RT_WAITING_FOREVER);
    s_fast.payload = *in;
    s_fast.payload.rec_magic = PARAM_FAST_REC_MAGIC;
    rt_mutex_release(&s_lock);

    return param_store_request(PARAM_REQ_FAST);
}

uint8_t param_fast_offset_is_same(uint32_t old_raw, uint32_t new_raw)
{
    uint32_t cpr = s_fast.payload.abs_cpr;
    uint32_t d;

    if (cpr == 0U) {
        cpr = 4096U;
    }

    d = (old_raw > new_raw) ? (old_raw - new_raw) : (new_raw - old_raw);
    if (d > (cpr / 2U)) {
        d = cpr - d;
    }

    return (d <= (uint32_t)PARAM_FAST_DIFF_TOL_COUNTS) ? 1U : 0U;
}

/* ============================================================
 * 诊断：存储状态 + 各组当前值
 * ============================================================ */
typedef struct {
    const char            *name;
    const param_config_t  *cfg;
    param_runtime_t       *rt;
    uint8_t               *buf;
    void                  (*pack)(void);
} param_group_t;

static param_group_t s_groups[] = {
    { "motion",     &s_cfg_motion, &s_rt_motion, (uint8_t *)&s_motion, cfg_pack },
    { "angle_mech", &s_cfg_am,     &s_rt_am,     (uint8_t *)&s_am,     am_pack  },
    { "system",     &s_cfg_system, &s_rt_system, (uint8_t *)&s_system, cfg_pack },
    { "fast",       &s_cfg_fast,   &s_rt_fast,   (uint8_t *)&s_fast,   RT_NULL  },
    { "device",     &s_cfg_dev,    &s_rt_dev,    (uint8_t *)&s_dev,    dev_pack },
};

#define PARAM_GROUP_COUNT  ((uint16_t)(sizeof(s_groups) / sizeof(s_groups[0])))

static param_group_t *group_find(const char *name)
{
    uint16_t i;

    if (name == RT_NULL) {
        return RT_NULL;
    }
    if (strcmp(name, "am") == 0) {
        name = "angle_mech";
    } else if (strcmp(name, "dev") == 0) {
        name = "device";
    }

    for (i = 0U; i < PARAM_GROUP_COUNT; i++) {
        if (strcmp(s_groups[i].name, name) == 0) {
            return &s_groups[i];
        }
    }
    return RT_NULL;
}

static void dump_info_one(const param_group_t *g)
{
    param_stat_t st;
    uint16_t     sec;
    uint16_t     left;

    Param_GetStat(g->cfg, g->rt, &st);

    rt_kprintf("[PARAM] %-10s rec=%uB sec=%u",
               g->name, (unsigned)st.record_size, (unsigned)st.sec_first);
    if (st.sec_first != st.sec_last) {
        rt_kprintf("~%u", (unsigned)st.sec_last);
    }
    rt_kprintf(" cur=%u left=%u/%u erase=%u seq=%u wr=%u src=%s\n",
               (unsigned)st.curr_sec, (unsigned)st.slots_left,
               (unsigned)st.slots_per_sec, (unsigned)st.erase_count,
               (unsigned)st.last_seq, (unsigned)st.write_count,
               (st.defaults_written != 0U) ? "DEFAULT" : "FLASH");

    /* 多扇区（快区）逐块列出：非当前扇区换扇时会先擦除，故按满容量算 */
    if (st.sec_first != st.sec_last) {
        for (sec = st.sec_first; sec <= st.sec_last; sec++) {
            left = (sec == st.curr_sec) ? st.slots_left : st.slots_per_sec;
            rt_kprintf("[PARAM]   sec %u: left=%u/%u%s\n",
                       (unsigned)sec, (unsigned)left, (unsigned)st.slots_per_sec,
                       (sec == st.curr_sec) ? " *cur" : " (after erase)");
        }
        rt_kprintf("[PARAM]   total_left=%u (%u sectors)\n",
                   (unsigned)(st.slots_left +
                              (uint16_t)((st.sec_last - st.sec_first) * st.slots_per_sec)),
                   (unsigned)(st.sec_last - st.sec_first + 1U));
    }
}

static void dump_cfg_item(uint16_t id)
{
    const cfg_def_t *d = sys_config_find_def(id);
    int32_t v = 0;

    if (d == RT_NULL) {
        rt_kprintf("  0x%04X (unknown)\n", (unsigned)id);
        return;
    }

    if (d->type == CFG_T_U32) {
        uint32_t u = 0U;

        (void)sys_config_get_u32(id, &u);
        rt_kprintf("  0x%04X %-22s = %lu\n",
                   (unsigned)id, d->name, (unsigned long)u);
    } else {
        (void)sys_config_get_i32(id, &v);
        rt_kprintf("  0x%04X %-22s = %d\n",
                   (unsigned)id, d->name, (int)v);
    }
}

static void dump_cfg_group(const char *title, uint32_t magic,
                           const uint16_t *ids, uint16_t n)
{
    uint16_t i;

    rt_kprintf("[PARAM] %s id=0x%08lX (%u items)\n",
               title, (unsigned long)magic, (unsigned)n);
    for (i = 0U; i < n; i++) {
        dump_cfg_item(ids[i]);
    }
}

static void print_fixed_str(const char *tag, const uint8_t *s, uint32_t n)
{
    char buf[40];

    if (n > (sizeof(buf) - 1U)) {
        n = sizeof(buf) - 1U;
    }
    memcpy(buf, s, n);
    buf[n] = '\0';
    rt_kprintf("  %s=%s\n", tag, buf);
}

static void dump_am_values(void)
{
    const param_am_payload_t *p = &s_am.payload;

    rt_kprintf("[PARAM] angle_mech id=0x%08lX\n", (unsigned long)PARAM_MAGIC_ANGLE_MECH);
    rt_kprintf("  active=%u count=%u\n",
               (unsigned)p->active, (unsigned)p->count);
    rt_kprintf("  sel: enc=%u iin=%u iout=%u\n",
               (unsigned)p->enc_sel, (unsigned)p->iin_sel,
               (unsigned)p->iout_sel);
    rt_kprintf("  z01=%u:%u z02=%u:%u iin=%u:%u iout=%u:%u pole=%u\n",
               (unsigned)p->row.enc_z01_num, (unsigned)p->row.enc_z01_den,
               (unsigned)p->row.enc_z02_num, (unsigned)p->row.enc_z02_den,
               (unsigned)p->row.i_in_num,    (unsigned)p->row.i_in_den,
               (unsigned)p->row.i_out_num,   (unsigned)p->row.i_out_den,
               (unsigned)p->row.pole_pairs);
    rt_kprintf("  teeth_tag=%u out=[%d,%d]x10 tol=%d\n",
               (unsigned)p->row.teeth_tag,
               (int)p->row.out_min_x10, (int)p->row.out_max_x10,
               (int)p->row.tol_override_x1000);
    rt_kprintf("  enc_zero=%u,%u,%u valid=%u\n",
               (unsigned)p->enc_zero_raw[0], (unsigned)p->enc_zero_raw[1],
               (unsigned)p->enc_zero_raw[2], (unsigned)p->enc_zero_valid);
    rt_kprintf("  calibrated=%u teeth=%u clear_action=%u lift_zero=%umV\n",
               (unsigned)p->is_calibrated, (unsigned)p->teeth_type,
               (unsigned)p->teeth_clear_action,
               (unsigned)p->lift_zero_voltage_mv);
}

static void dump_dev_values(void)
{
    rt_kprintf("[PARAM] device id=0x%08lX\n", (unsigned long)PARAM_MAGIC_DEVICE);
    print_fixed_str("sn", s_dev.payload.sn, sizeof(s_dev.payload.sn));
    print_fixed_str("name", s_dev.payload.name, sizeof(s_dev.payload.name));
    print_fixed_str("sw_str", s_dev.payload.sw_version_str,
                    sizeof(s_dev.payload.sw_version_str));
    rt_kprintf("  model=%lu hw=0x%08lX sw=0x%08lX date=%lu first_boot=%lu\n",
               (unsigned long)s_dev.payload.model,
               (unsigned long)s_dev.payload.hw_version,
               (unsigned long)s_dev.payload.sw_version,
               (unsigned long)s_dev.payload.manufacture_date,
               (unsigned long)s_dev.payload.first_boot_timestamp);
}

static void dump_fast_values(void)
{
    const param_fast_payload_t *p = &s_fast.payload;

    rt_kprintf("[PARAM] fast id=0x%08lX\n", (unsigned long)PARAM_MAGIC_FAST);
    rt_kprintf("  valid=%u abs_raw=%lu calib_count=%lu cpr=%u ratio=%u:%u pole=%u dir=%u\n",
               (unsigned)p->valid, (unsigned long)p->abs_raw_at_zero,
               (unsigned long)p->calib_count, (unsigned)p->abs_cpr,
               (unsigned)p->enc_to_rotor_num, (unsigned)p->enc_to_rotor_den,
               (unsigned)p->pole_pairs, (unsigned)p->elec_dir);
    rt_kprintf("  diff_tol=%d counts\n", (int)PARAM_FAST_DIFF_TOL_COUNTS);
}

void param_store_dump(uint32_t flags)
{
    uint16_t i;

    if (s_inited == 0U) {
        (void)param_store_init();
    }

    if ((flags & PARAM_DUMP_INFO) != 0U) {
        rt_kprintf("[PARAM] ===== storage status =====\n");
        for (i = 0U; i < PARAM_GROUP_COUNT; i++) {
            dump_info_one(&s_groups[i]);
        }
    }

    if ((flags & PARAM_DUMP_VALUES) != 0U) {
        /* 先用业务侧 RAM 刷新记录镜像，保证打印的就是"下次会存的内容"；
         * cfg 项直接从 sys_config 读，无需打包。 */
        rt_mutex_take(&s_lock, RT_WAITING_FOREVER);
        am_pack();
        dev_pack();
        rt_mutex_release(&s_lock);

        rt_kprintf("[PARAM] ===== current values =====\n");
        dump_cfg_group("motion", PARAM_MAGIC_MOTION,
                       s_motion_ids, PARAM_MOTION_CFG_COUNT);
        dump_cfg_group("system", PARAM_MAGIC_SYSTEM,
                       s_system_ids, PARAM_SYSTEM_CFG_COUNT);
        rt_kprintf("[PARAM] system mode: default=%u allowed=0x%02X lock=%u cap_ovr=0x%02X timeout=%lu\n",
                   (unsigned)s_system.mode.default_mode,
                   (unsigned)s_system.mode.allowed_modes,
                   (unsigned)s_system.mode.factory_lock,
                   (unsigned)s_system.mode.cap_override,
                   (unsigned long)s_system.mode.test_timeout_ms);
        dump_am_values();
        dump_dev_values();
        dump_fast_values();
        rt_kprintf("[PARAM] ===== end =====\n");
    }
}

void param_store_boot_dump(void)
{
#if PARAM_BOOT_DUMP_ENABLE
    param_store_dump(PARAM_DUMP_INFO | PARAM_DUMP_VALUES);
#endif
}

int param_store_save_cmd(uint32_t mask)
{
    int rc;

    if (s_inited == 0U) {
        (void)param_store_init();
    }

    /* 运行中禁止保存：与异步保存线程的安全约束保持一致 */
    if (motor_is_busy() != 0) {
        rt_kprintf("[PARAM] motor is running, save refused, stop first\n");
        return -2;
    }

    rt_kprintf("[PARAM] saving mask=0x%X...\n", (unsigned)mask);
    rc = param_store_save_now(mask);
    if (rc == 0) {
        rt_kprintf("[PARAM] save OK\n");
    } else {
        rt_kprintf("[PARAM] save FAILED\n");
    }
    param_store_dump(PARAM_DUMP_INFO);

    return rc;
}

/* ============================================================
 * 擦除（格式化为当前 RAM 值）
 * ============================================================ */
int param_store_erase_cmd(uint32_t mask)
{
    int rc = 0;

    if (s_inited == 0U) {
        (void)param_store_init();
    }
    if (motor_is_busy() != 0) {
        rt_kprintf("[PARAM] motor is running, erase refused, stop first\n");
        return -2;
    }

    rt_mutex_take(&s_lock, RT_WAITING_FOREVER);

    if ((mask & PARAM_REQ_CFG) != 0U) {
        cfg_pack();
        if (Param_EraseAll(&s_cfg_motion, &s_rt_motion,
                           (uint8_t *)&s_motion) != PARAM_OK) {
            rc = -1;
        }
        if (Param_EraseAll(&s_cfg_system, &s_rt_system,
                           (uint8_t *)&s_system) != PARAM_OK) {
            rc = -1;
        }
    }
    if ((mask & PARAM_REQ_AM) != 0U) {
        am_pack();
        if (Param_EraseAll(&s_cfg_am, &s_rt_am, (uint8_t *)&s_am) != PARAM_OK) {
            rc = -1;
        }
    }
    if ((mask & PARAM_REQ_DEV) != 0U) {
        dev_pack();
        if (Param_EraseAll(&s_cfg_dev, &s_rt_dev, (uint8_t *)&s_dev) != PARAM_OK) {
            rc = -1;
        }
    }
    if ((mask & PARAM_REQ_FAST) != 0U) {
        if (Param_EraseAll(&s_cfg_fast, &s_rt_fast,
                           (uint8_t *)&s_fast) != PARAM_OK) {
            rc = -1;
        }
    }

    rt_mutex_release(&s_lock);

    rt_kprintf("[PARAM] erase mask=0x%X %s, current RAM rewritten\n",
               (unsigned)mask, (rc == 0) ? "OK" : "FAILED");
    param_store_dump(PARAM_DUMP_INFO);

    return rc;
}

/* ============================================================
 * 填充：把当前扇区写到只剩 leave 个槽位
 * ============================================================ */
int param_store_fill_cmd(const char *group, uint16_t leave)
{
    param_group_t *g;
    param_stat_t   st;
    uint32_t       writes = 0U;
    int            rc = 0;

    if (s_inited == 0U) {
        (void)param_store_init();
    }

    g = group_find(group);
    if (g == RT_NULL) {
        rt_kprintf("[PARAM] fill: unknown group '%s'\n",
                   (group != RT_NULL) ? group : "?");
        rt_kprintf("  usage: t_param fill <motion|system|angle_mech|device|fast> [leave]\n");
        return -1;
    }
    if (motor_is_busy() != 0) {
        rt_kprintf("[PARAM] motor is running, fill refused, stop first\n");
        return -2;
    }

    rt_mutex_take(&s_lock, RT_WAITING_FOREVER);

    if (g->pack != RT_NULL) {
        g->pack();
    }

    Param_GetStat(g->cfg, g->rt, &st);
    if (leave >= st.slots_per_sec) {
        rt_mutex_release(&s_lock);
        rt_kprintf("[PARAM] fill: leave must be < %u\n",
                   (unsigned)st.slots_per_sec);
        return -1;
    }

    rt_kprintf("[PARAM] fill %s: left=%u target=%u, writing...\n",
               g->name, (unsigned)st.slots_left, (unsigned)leave);

    /* 每次写入消耗 1 个槽位；上限 4096 次防止异常情况下死循环 */
    while ((st.slots_left > leave) && (writes < 4096U)) {
        if (Param_Save(g->cfg, g->rt, g->buf) != PARAM_OK) {
            rc = -1;
            break;
        }
        writes++;
        Param_GetStat(g->cfg, g->rt, &st);
    }

    rt_mutex_release(&s_lock);

    rt_kprintf("[PARAM] fill %s done: writes=%lu left=%u/%u cur=%u seq=%lu rc=%d\n",
               g->name, (unsigned long)writes, (unsigned)st.slots_left,
               (unsigned)st.slots_per_sec, (unsigned)st.curr_sec,
               (unsigned long)st.last_seq, rc);
    dump_info_one(g);

    return rc;
}

/* ============================================================
 * 精确写入 count 次（某区只存一次 / 手动触发轮转）
 * ============================================================ */
int param_store_write_cmd(const char *group, uint16_t count)
{
    param_group_t *g;
    param_stat_t   st;
    uint32_t       writes = 0U;
    int            rc = 0;

    if (s_inited == 0U) {
        (void)param_store_init();
    }

    g = group_find(group);
    if (g == RT_NULL) {
        rt_kprintf("[PARAM] write: unknown group '%s'\n",
                   (group != RT_NULL) ? group : "?");
        rt_kprintf("  usage: t_param write <motion|system|angle_mech|device|fast> [count]\n");
        return -1;
    }
    if (motor_is_busy() != 0) {
        rt_kprintf("[PARAM] motor is running, write refused, stop first\n");
        return -2;
    }
    if (count == 0U) {
        count = 1U;
    }
    if (count > 4096U) {
        count = 4096U;
    }

    rt_mutex_take(&s_lock, RT_WAITING_FOREVER);

    if (g->pack != RT_NULL) {
        g->pack();
    }
    Param_GetStat(g->cfg, g->rt, &st);

    while (writes < count) {
        if (Param_Save(g->cfg, g->rt, g->buf) != PARAM_OK) {
            rc = -1;
            break;
        }
        writes++;
    }

    Param_GetStat(g->cfg, g->rt, &st);
    rt_mutex_release(&s_lock);

    rt_kprintf("[PARAM] write %s x%u done: left=%u/%u cur=%u erase=%lu seq=%lu rc=%d\n",
               g->name, (unsigned)writes, (unsigned)st.slots_left,
               (unsigned)st.slots_per_sec, (unsigned)st.curr_sec,
               (unsigned long)st.erase_count, (unsigned long)st.last_seq, rc);
    dump_info_one(g);

    return rc;
}

/* ============================================================
 * 设置当前扇区擦除次数（测试寿命轮转用）
 * ============================================================ */
int param_store_set_erase_cmd(const char *group, uint32_t count)
{
    param_group_t *g;
    param_stat_t   st;
    uint32_t       old;

    if (s_inited == 0U) {
        (void)param_store_init();
    }

    g = group_find(group);
    if (g == RT_NULL) {
        rt_kprintf("[PARAM] erasecnt: unknown group '%s'\n",
                   (group != RT_NULL) ? group : "?");
        rt_kprintf("  usage: t_param erasecnt <motion|system|angle_mech|device|fast> <count>\n");
        return -1;
    }
    if (motor_is_busy() != 0) {
        rt_kprintf("[PARAM] motor is running, erasecnt refused, stop first\n");
        return -2;
    }

    rt_mutex_take(&s_lock, RT_WAITING_FOREVER);
    old = g->rt->erase_count;
    g->rt->erase_count = count;
    rt_mutex_release(&s_lock);

    rt_kprintf("[PARAM] %s erase_count: %lu -> %lu\n",
               g->name, (unsigned long)old, (unsigned long)count);

    if (g->cfg->sec_start == g->cfg->sec_end) {
        rt_kprintf("[PARAM] '%s' is single-sector, erasecnt only affects diagnostics\n",
                   g->name);
    } else if ((g->cfg->max_erase_life != 0U) &&
               (count >= g->cfg->max_erase_life)) {
        rt_kprintf("[PARAM] next write on '%s' will rotate sector %u -> %u\n",
                   g->name, (unsigned)g->rt->curr_sec,
                   (g->rt->curr_sec == g->cfg->sec_end)
                       ? (unsigned)g->cfg->sec_start
                       : (unsigned)(g->rt->curr_sec - 1U));
    }

    Param_GetStat(g->cfg, g->rt, &st);
    dump_info_one(g);

    return 0;
}
