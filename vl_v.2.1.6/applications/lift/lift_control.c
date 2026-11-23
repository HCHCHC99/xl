/**
 * @file    vert_control.c
 * @brief   举升控制状态机初始化 + 定时器管理 + 外部状态更新
 */

#include "lift_control.h"

/* 【2026-11-10 彩标 P0】举升控制注册：失败=ERR，成功=OK */
#include "../log/log_color.h"
#include "lift_actions.h"
#include "lift_angle.h"
#include "../mode/sys_mode.h"
#include "framework/state_engine/sm_assert.h"
#include "app_event.h"
#include "../config/sys_config.h"   /* P6：lift_hold_ms / 传感器阈值 */
#include "framework/state_engine/state_engine.h"
#include <string.h>
#include <rtthread.h>

/* ============================================================
 * 全局对象
 * ============================================================ */
lift_control_t g_lift_obj;

/* ============================================================
 * 【P4 / B-5】阀故障 500 ms 去抖 —— 状态定义（函数见下方"公开 API"）
 * ------------------------------------------------------------
 * 背景：0x18FF0E63 的故障码是"当前状态"，不是"事件"。原实现一帧非 0
 *       就投 LIFT_EVT_VALVE_FAULT，等于零容错：250kbit/s 下偶发误码、
 *       线束接触抖动、RGV 上电自检都会让正在上升的卸粮筒瞬间失电。
 *
 * 需求：TC-FLT-012「注入上升阀短路并保持 >500 ms → 一键立即停止」。
 *       关键词是"保持"——判的是**连续**满足，不是累计。
 *
 * 实现：CAN 回调只登记原始码与计时起点；50 ms tick 的 poll 推进计时，
 *       满 confirm_ms 才确认并投事件。故障码消失立即清除（不延时）：
 *       "多停一会儿"是安全方向，"多报一会儿"才是误动作来源。
 *
 * 精度：受 50 ms 轮询限制，实际确认时间落在 [confirm_ms, confirm_ms+50)。
 *       用例判据按"400 ms 不报 / 500~560 ms 报"执行。
 * ============================================================ */
#define LIFT_VFB_CONFIRM_MS_DEF   500U

typedef struct {
    uint8_t  raw;          /* 原始码 0/1/2 */
    uint8_t  latched;      /* 已确认 */
    uint8_t  timing;       /* 计时中 */
    uint32_t since_tick;   /* 计时起点 */
    uint32_t elapsed_ms;   /* 已持续 */
} vfb_ch_t;

static vfb_ch_t  s_vfb[2];                                  /* 0=上升 1=下降 */
static uint8_t   s_vfb_debounce_en = 1U;
static uint16_t  s_vfb_confirm_ms  = LIFT_VFB_CONFIRM_MS_DEF;

static void vfb_reset(void);   /* 注册时复位（实现见文件下方） */

/* ============================================================
 * 5秒超时定时器（RT-Thread 软件定时器）
 * ============================================================ */
static rt_timer_t g_lift_timeout_timer;

static void lift_timeout_callback(void *param)
{
    post_event(LIFT_EVT_TIMEOUT_5S);
}

/* ============================================================
 * 外部引用
 * ============================================================ */
extern const StateMachineConfig_t g_lift_config;

/* ============================================================
 * 初始化并注册
 * ============================================================ */
int lift_control_register(ctrl_manager_t *mgr)
{
    rt_err_t ret;

    SM_ASSERT_PTR(mgr);

    /* 初始化上下文 */
    memset(&g_lift_obj.ctx, 0, sizeof(lift_context_t));
    g_lift_obj.ctx.solenoid_ptr = &g_solenoid;
    g_lift_obj.ctx.lift_ptr = &g_lift_angle;

    /* 【P4】阀故障去抖状态复位（默认开启 / 500 ms） */
    vfb_reset();

    /* 绑定状态机指针 */
    g_lift_obj.base.sm = &g_lift_obj.sm_instance;

    /* 单线程配置 */
    ctrl_obj_runtime_cfg_t runtime = {
        .use_thread = 0,
    };

    /* 初始化控制对象 */
    ret = control_object_init(&g_lift_obj.base,
                              "LiftControl",
                              CTRL_OBJ_TYPE_LIFT,
                              &g_lift_config,
                              &g_lift_obj.ctx,
                              &runtime);
    if (ret != RT_EOK) {
        LOG_ERR("[Lift] control_object_init failed (%d)\n", ret);
        return -1;
    }

    /* 创建超时定时器 */
    g_lift_timeout_timer = rt_timer_create("p_tmo5s",
                                             lift_timeout_callback,
                                             RT_NULL,
                                             5000,
                                             RT_TIMER_FLAG_ONE_SHOT);
    SM_ASSERT_PTR(g_lift_timeout_timer);

    /* 注册到管理器 */
    ret = ctrl_manager_register(mgr, &g_lift_obj.base);
    if (ret != 0) {
        LOG_ERR("[Lift] register failed (%d)\n", ret);
        return -1;
    }

    LOG_OK("[Lift] Control registered.\n");
    return 0;
}


/* ============================================================
 * 公开 API：状态更新
 * ============================================================ */
void lift_control_update_switch(uint8_t up, uint8_t down)
{
    g_lift_obj.ctx.up_sw_pressed = up;
    g_lift_obj.ctx.down_sw_pressed = down;
}

/* ============================================================
 * 【P4 / B-5】阀故障 500 ms 去抖
 * ------------------------------------------------------------
 * 状态定义见文件顶部；此处只放实现。
 * ============================================================ */
/* 把"确认/解除"落到 ctx 并投事件（只在边沿投一次） */
static void vfb_publish(uint8_t ch, uint8_t confirmed)
{
    lift_context_t *c = &g_lift_obj.ctx;
    uint8_t *field = (ch == 0U) ? &c->valve_fault_up : &c->valve_fault_down;
    uint8_t  other = (ch == 0U) ? c->valve_fault_down : c->valve_fault_up;

    if (*field == confirmed) {
        return;
    }
    *field = confirmed;

    if (confirmed != 0U) {
        post_event(LIFT_EVT_VALVE_FAULT);
        rt_kprintf("[VFB] %s valve fault confirmed\n", (ch == 0U) ? "up" : "down");
    } else if ((other == 0U) &&
               (s_vfb[0].raw == 0U) && (s_vfb[1].raw == 0U)) {
        /* 两路都已清（既无已确认故障，也无正在计时的原始码）才投清除，
         * 否则"一路恢复、一路刚开始计时"会先退出 FAULT_VALVE 再进去。 */
        post_event(LIFT_EVT_VALVE_FAULT_CLEAR);
        rt_kprintf("[VFB] valve fault cleared\n");
    }
}

static void vfb_reset(void)
{
    memset(s_vfb, 0, sizeof(s_vfb));
    s_vfb_debounce_en = 1U;
    s_vfb_confirm_ms  = LIFT_VFB_CONFIRM_MS_DEF;
}

void lift_control_valve_fb_poll(void)
{
    uint32_t now = rt_tick_get();
    uint8_t  ch;

    for (ch = 0U; ch < 2U; ch++) {
        vfb_ch_t *v = &s_vfb[ch];

        if (v->raw == 0U) {
            v->timing     = 0U;
            v->elapsed_ms = 0U;
            continue;
        }
        if (v->latched != 0U) {
            v->timing = 0U;
            continue;
        }
        /* 去抖关闭或门限为 0 → 退化为旧行为（立即报），供 t_valve_fb off 对比 */
        if ((s_vfb_debounce_en == 0U) || (s_vfb_confirm_ms == 0U)) {
            v->timing     = 0U;
            v->elapsed_ms = 0U;
            v->latched    = 1U;
            vfb_publish(ch, 1U);
            continue;
        }
        if (v->timing == 0U) {
            v->since_tick = now;    /* 先写起点再置 timing，理由同上 */
            v->elapsed_ms = 0U;
            v->timing     = 1U;
            continue;
        }
        v->elapsed_ms = ((now - v->since_tick) * 1000U) / RT_TICK_PER_SECOND;
        if (v->elapsed_ms >= (uint32_t)s_vfb_confirm_ms) {
            v->timing  = 0U;
            v->latched = 1U;
            vfb_publish(ch, 1U);
        }
    }
}

void lift_control_valve_fb_get(lift_vfb_state_t *out)
{
    uint8_t ch;

    if (out == RT_NULL) {
        return;
    }
    out->debounce_en = s_vfb_debounce_en;
    out->confirm_ms  = s_vfb_confirm_ms;
    for (ch = 0U; ch < 2U; ch++) {
        out->ch[ch].raw        = s_vfb[ch].raw;
        out->ch[ch].latched    = s_vfb[ch].latched;
        out->ch[ch].timing     = s_vfb[ch].timing;
        out->ch[ch].elapsed_ms = s_vfb[ch].elapsed_ms;
    }
}

void lift_control_valve_fb_set(uint8_t debounce_en, uint16_t confirm_ms)
{
    s_vfb_debounce_en = (debounce_en != 0U) ? 1U : 0U;
    s_vfb_confirm_ms  = confirm_ms;
}

void lift_control_update_valve_fault(uint8_t up_fault, uint8_t down_fault)
{
    uint8_t  code[2];
    uint8_t  changed[2];
    uint32_t now = rt_tick_get();
    uint8_t  ch;

    code[0] = (uint8_t)(up_fault   & 0x03U);
    code[1] = (uint8_t)(down_fault & 0x03U);

    /* 第一遍：先把两路原始码都更新完。
     * 必须先于"投清除事件"——同一帧里若"一路恢复、另一路新增故障"，
     * 边更新边判决会在另一路还没写入前误投 CLEAR，
     * 表现为瞬间离开 FAULT_VALVE（阀有可能被重新打开 500 ms）。 */
    for (ch = 0U; ch < 2U; ch++) {
        changed[ch]  = (code[ch] != s_vfb[ch].raw) ? 1U : 0U;
        s_vfb[ch].raw = code[ch];
    }

    /* 第二遍：按新码推进计时 / 确认 / 解除 */
    for (ch = 0U; ch < 2U; ch++) {
        vfb_ch_t *v = &s_vfb[ch];

        if (changed[ch] == 0U) {
            continue;               /* 码未变：计时由 poll 推进，不重起 */
        }

        if (v->raw == 0U) {
            /* 本路故障消失：立即清计时；已确认的立即解除（安全方向） */
            v->timing     = 0U;
            v->since_tick = 0U;
            v->elapsed_ms = 0U;
            if (v->latched != 0U) {
                v->latched = 0U;
                vfb_publish(ch, 0U);
            }
            continue;
        }

        if (v->latched != 0U) {
            continue;               /* 已确认，开路↔短路互转不重复投事件 */
        }
        if ((s_vfb_debounce_en == 0U) || (s_vfb_confirm_ms == 0U)) {
            v->timing     = 0U;
            v->elapsed_ms = 0U;
            v->latched    = 1U;
            vfb_publish(ch, 1U);
        } else {
            /* 先写起点再置 timing：poll 在另一线程，若看到 timing=1 而
             * since_tick 还是 0，会算出巨大的 elapsed 立即确认（提前报故障）。 */
            v->since_tick = now;
            v->elapsed_ms = 0U;
            v->timing     = 1U;
        }
    }
}

void lift_control_update_sensor_fault(uint8_t fault)
{
    uint8_t old_fault = g_lift_obj.ctx.sensor_fault;
    g_lift_obj.ctx.sensor_fault = fault;

    if (!old_fault && fault) {
        post_event(LIFT_EVT_SENSOR_FAULT);
    }
    if (old_fault && !fault) {
        post_event(LIFT_EVT_SENSOR_FAULT_CLEAR);
    }
}

void lift_control_update_valve_fb(uint8_t up_fb, uint8_t down_fb)
{
    g_lift_obj.ctx.valve_fb_up = up_fb;
    g_lift_obj.ctx.valve_fb_down = down_fb;
}

/* ============================================================
 * 公开 API：定时器管理
 * ============================================================ */
/* 【P6 / B-7】保持时长读 CFG_ID_LIFT_HOLD_MS（默认 5000ms）
 * 0 表示"到限位立即关阀"的旧行为，这里夹到 1ms 等价实现。 */
static int32_t lift_hold_ms_cfg(void)
{
    int32_t ms = 5000;

    if (sys_config_get_i32(CFG_ID_LIFT_HOLD_MS, &ms) != 0) {
        ms = 5000;
    }
    if (ms < 0)    { ms = 0; }
    if (ms > 20000){ ms = 20000; }
    return ms;
}

void lift_control_start_timer(void)
{
    int32_t ms = lift_hold_ms_cfg();

    if (ms < 1) { ms = 1; }
    rt_timer_stop(g_lift_timeout_timer);
    (void)rt_timer_control(g_lift_timeout_timer, RT_TIMER_CTRL_SET_TIME, &ms);
    rt_timer_start(g_lift_timeout_timer);
}

int32_t lift_control_hold_remaining_ms(void)
{
    uint32_t elapsed;

    if (lift_control_get_state() != P_STATE_LIMIT_HOLDING) {
        return -1;
    }
    elapsed = (uint32_t)((rt_tick_get() - g_lift_obj.ctx.hold_start_tick) * 1000U)
              / RT_TICK_PER_SECOND;
    return lift_hold_ms_cfg() - (int32_t)elapsed;
}

void lift_control_stop_timer(void)
{
    rt_timer_stop(g_lift_timeout_timer);
}

/* ============================================================
 * 公开 API：获取状态
 * ============================================================ */
lift_state_t  lift_control_get_state(void)
{
    return (lift_state_t)StateMachine_GetState(&g_lift_obj.sm_instance);
}

lift_context_t* lift_control_get_context(void)
{
    return &g_lift_obj.ctx;
}

