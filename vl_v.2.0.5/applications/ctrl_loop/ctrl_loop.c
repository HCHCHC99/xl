/**
 * @file    ctrl_loop.c
 * @brief   通用三环控制模块实现
 */

#include "ctrl_loop.h"
#include "framework/state_engine/sm_assert.h"
#include <string.h>
#include <rtthread.h>

/* ============================================================
 * 实例注册表
 * ------------------------------------------------------------
 * 【为什么用静态数组而不是链表】
 *   上限固定 4 个，数组更省 RAM（无 next 指针）且索引访问 O(1)。
 *   静态分配也避免 malloc 在嵌入式里的碎片问题。
 * ============================================================ */
static ctrl_loop_t *s_inst[CTRL_LOOP_MAX_INST];
static uint8_t      s_inst_cnt;

/* ============================================================
 * 环类型名
 * ============================================================ */
const char *ctrl_loop_kind_name(ctrl_loop_kind_t kind)
{
    switch (kind) {
        case CTRL_LOOP_POSITION: return "position";
        case CTRL_LOOP_SPEED:    return "speed";
        case CTRL_LOOP_CURRENT:  return "current";
        default:                 return "?";
    }
}

/* ============================================================
 * 定点 PID 单步
 * ------------------------------------------------------------
 * 【定点约定】
 *   kp_x100 / ki_x100 / kd_x100 都是 ×100 的整数。
 *   kp=1.00 → kp_x100=100；kp=0.05 → kp_x100=5。
 *
 * 【三项各自的定点推导】
 *   P:  out_p = err * kp
 *            = err * kp_x100 / 100
 *
 *   I:  out_i = ∫err * ki dt
 *             = err * ki_x100 / 100 * dt_ms / 1000
 *             = err * ki_x100 * dt_ms / 100000
 *
 *   D:  out_d = kd * d(err)/dt
 *             = kd_x100/100 * (Δerr / (dt_ms/1000))
 *             = kd_x100 * Δerr * 1000 / (100 * dt_ms)
 *             = kd_x100 * Δerr * 10 / dt_ms
 *
 * 【为什么用 int64 中间量】
 *   err 可达 ±3000（角度 0.1° 单位），kp_x100 可达 10000，
 *   err * kp_x100 = 3e7，再乘 dt_ms(1000) = 3e10 > int32 上限 2.1e9。
 *   必须用 int64，否则溢出后表现为"输出突然反号"这种极难查的故障。
 * ============================================================ */
int32_t ctrl_loop_pid_step(ctrl_loop_ring_t *ring,
                           int32_t target, int32_t meas, uint32_t dt_ms)
{
    int32_t  err;
    int64_t  p_term;
    int64_t  d_term;
    int64_t  out;
    int32_t  new_integ;
    int32_t  d_input;
    uint8_t  integ_freeze;

    if (ring == RT_NULL) {
        return 0;
    }

    /* ---- 旁路：目标直接透传 ----
     * 用于"某环不启用但要保持通道畅通"的场景：
     *   位置环关、速度环开时，位置目标直接作为速度目标。 */
    if (ring->cfg.enable == 0U) {
        ring->last_out = target;
        return target;
    }

    /* ---- dt 为 0：无法算积分/微分，保持上次输出 ----
     * 这种情况只在同一 tick 内被调两次时出现，
     * 返 0 会让输出突降，返上次值更安全。 */
    if (dt_ms == 0U) {
        return ring->last_out;
    }

    err = target - meas;
    ring->err = err;

    /* ---------- P 项 ---------- */
    p_term = (int64_t)err * (int64_t)ring->cfg.kp_x100 / 100;

    /* ---------- I 项 ---------- */
    {
        int64_t inc = (int64_t)err * (int64_t)ring->cfg.ki_x100
                      * (int64_t)dt_ms / 100000;
        int64_t acc = (int64_t)ring->integ + inc;

        /* 积分限幅：钳位法（anti-windup）
         *
         * 为什么必须限幅：
         *   目标长时间达不到（如堵转、到限位）时，误差持续同号，
         *   积分会累积到极大值。一旦脱离阻塞，需要很长时间才能退饱和，
         *   表现为"过冲后迟迟不回落"甚至"反向冲过头"。
         *
         * 限幅值取 integ_max，由参数配置。经验值：
         *   积分项单独作用的最大输出 ≈ out_max 的 30%~50% */
        if (acc > (int64_t)ring->cfg.integ_max) {
            acc = (int64_t)ring->cfg.integ_max;
        } else if (acc < -(int64_t)ring->cfg.integ_max) {
            acc = -(int64_t)ring->cfg.integ_max;
        }
        new_integ = (int32_t)acc;
    }

    /* ---------- D 项 ---------- */
    if (ring->cfg.d_on_measure != 0U) {
        /* 微分先行：作用在测量值上，d = -kd * d(meas)/dt
         *
         * 为什么默认用这个：
         *   本工程目标值来自用户按键，是阶跃变化。
         *   若微分作用在误差上，目标跳变瞬间 Δerr 极大，
         *   会输出一个巨大的尖峰（"微分冲击"），可能直接触发过流保护。
         *   作用在测量值上则完全避开这个问题。 */
        d_input = -(meas - ring->last_meas);
    } else {
        /* 标准式：微分作用在误差上 */
        d_input = err - ring->last_err;
    }

    /* D: kd_x100 * Δinput * 10 / dt_ms */
    d_term = (int64_t)ring->cfg.kd_x100 * (int64_t)d_input * 10
             / (int64_t)dt_ms;

    /* ---------- 合成 ---------- */
    out = p_term + (int64_t)new_integ + d_term;

    /* ---------- 限幅 + 条件积分 ----------
     *
     * 条件积分（conditional integration）：
     *   输出已饱和且误差方向与饱和方向一致时，冻结积分。
     *   与单纯钳位相比，它能在饱和期间保持积分值不变，
     *   脱离饱和时能立刻响应，不会"卡"在饱和点。 */
    integ_freeze = 0U;

    if (out > (int64_t)ring->cfg.out_max) {
        out = (int64_t)ring->cfg.out_max;
        ring->sat_cnt++;
        if (err > 0) { integ_freeze = 1U; }
    } else if (out < (int64_t)ring->cfg.out_min) {
        out = (int64_t)ring->cfg.out_min;
        ring->sat_cnt++;
        if (err < 0) { integ_freeze = 1U; }
    }

    if (integ_freeze == 0U) {
        ring->integ = new_integ;
    }

    /* ---------- 保存历史 ---------- */
    ring->last_meas = meas;
    ring->last_err  = err;
    ring->last_out  = (int32_t)out;
    ring->run_cnt++;

    return (int32_t)out;
}

/* ============================================================
 * 单环执行
 * ------------------------------------------------------------
 * 执行顺序与 target 传递：
 *   位置环：target[pos] 由外部写 → PID → 输出写 target[speed]
 *   速度环：target[speed] 由位置环写 → PID → 输出写 target[current]
 *   电流环：target[current] → PID → 输出给 set_out()
 *
 * ⚠ 关键设计：**上一环的输出写进下一环的 target，而不是直接调用**
 *   好处：
 *     ① 三环可以独立开关，关掉某环时目标自然透传
 *     ② 每环按自己周期跑，不要求同频
 *     ③ 调试时可以从中间任何一环注入目标
 * ============================================================ */
static void ctrl_loop_run_ring(ctrl_loop_t *loop, ctrl_loop_kind_t kind,
                               uint32_t dt_ms)
{
    ctrl_loop_ring_t *ring = &loop->ring[kind];
    int32_t meas;
    int32_t out;

    /* 读反馈 */
    if (loop->get_fb != RT_NULL) {
        meas = loop->get_fb(loop, kind);
    } else {
        meas = 0;
    }

    /* PID 运算（含前馈） */
    out = ctrl_loop_pid_step(ring, loop->target[kind], meas, dt_ms);
    out += loop->ff[kind];

    /* 前馈叠加后可能越界，再限一次 */
    if (out > ring->cfg.out_max) { out = ring->cfg.out_max; }
    if (out < ring->cfg.out_min) { out = ring->cfg.out_min; }

    /* ---- 输出路由：本环输出 → 下一环目标 + 对外 set_out ---- */
    switch (kind) {
        case CTRL_LOOP_POSITION:
            /* 位置环输出 = 速度目标 */
            loop->target[CTRL_LOOP_SPEED] = out;
            break;

        case CTRL_LOOP_SPEED:
            /* 速度环输出 = 电流目标（或直接占空比，取决于驱动模式） */
            loop->target[CTRL_LOOP_CURRENT] = out;
            /* 速度环是最外层的实际执行者：直接把结果写给硬件。
             * 为什么这里也调 set_out：当前是六步方波无电流环，
             * 速度环输出就是占空比，必须落到硬件上。 */
            if (loop->set_out != RT_NULL) {
                loop->set_out(loop, kind, out);
            }
            break;

        case CTRL_LOOP_CURRENT:
            /* 电流环输出 = 占空比 */
            if (loop->set_out != RT_NULL) {
                loop->set_out(loop, kind, out);
            }
            break;

        default:
            break;
    }
}

/* ============================================================
 * 周期驱动
 * ------------------------------------------------------------
 * 【分频机制】
 *   每个环记录 last_tick，只有当 (now - last) >= period_ms 才执行。
 *   这样：
 *     位置环 5ms、速度环 1ms、电流环 20kHz（另路）可共存，
 *     调用方只需以最快环的周期（1ms）驱动本函数。
 *
 * 【为什么电流环不走这里】
 *   20kHz 需要 50μs 精度，而本函数用 ms 级 tick。
 *   电流环由 PWM 谷值 ISR 直接调 ctrl_loop_current_step()。
 *   本函数看到 CTRL_LOOP_CURRENT.enable=1 时**跳过**该环，
 *   避免两处都执行导致重复运算。
 * ============================================================ */
void ctrl_loop_tick(ctrl_loop_t *loop, uint32_t now_tick)
{
    uint8_t k;

    if ((loop == RT_NULL) || (loop->inited == 0U)) {
        return;
    }
    if (loop->enabled == 0U) {
        return;
    }

    for (k = 0U; k < (uint8_t)CTRL_LOOP_RING_MAX; k++) {
        ctrl_loop_ring_t *ring = &loop->ring[k];
        uint32_t elapsed;

        /* 该环未启用：不执行，但目标透传给下一环
         * （在 ctrl_loop_pid_step 的旁路分支里已处理） */
        if (ring->period_ms == 0U) {
            continue;
        }

        /* 【电流环预留标记】
         * 电流环走 20kHz ISR，不参与 ms 分频。
         * 见 ctrl_loop_current_step() 的说明。 */
        if ((ctrl_loop_kind_t)k == CTRL_LOOP_CURRENT) {
            continue;
        }

        elapsed = now_tick - ring->last_tick;   /* 无符号减法天然处理回绕 */
        if (elapsed < ring->period_ms) {
            ring->skip_cnt++;
            continue;
        }

        ring->last_tick = now_tick;

        /* elapsed 可能远大于 period（线程被抢占），
         * 此时按实际 elapsed 算 dt，避免积分项偏小。
         * 但钳到 10 倍 period 以内，防止长时间阻塞后
         * 积分一次性冲爆。 */
        if (elapsed > ring->period_ms * 10U) {
            elapsed = ring->period_ms * 10U;
        }

        ctrl_loop_run_ring(loop, (ctrl_loop_kind_t)k, elapsed);
    }
}

/* ============================================================
 * 电流环单步（20kHz ISR 入口，预留）
 * ------------------------------------------------------------
 * 【MARK-CL-01 · 2026-11-10 预留标记】
 *
 * 当前状态：**接口存在，但无调用者**。
 *
 * 原因：
 *   本工程驱动层是六步方波（motor_drv_bldc_6step.c），
 *   没有 SVPWM / Clarke / Park，电流环无处落地。
 *   20kHz ISR（bsp_pwm_tmr4_isr）当前只做：
 *     清标志 → s_pwm_cycle_cnt++ → foc_adc_trigger()
 *
 * 接入前提（三项都需完成）：
 *   □ 驱动层换 FOC（motor/foc/ 下的 Clarke/Park/SVPWM 实现）
 *   □ 电流反馈单位统一（当前 foc_adc 返回 0.1A，需确认与 out_max 同量纲）
 *   □ 20kHz ISR 里确认无 int64 除法（本函数用 int64，需实测耗时）
 *
 * 接法：在 bsp_pwm_tmr4_isr 的 is_valley 分支里，
 *       foc_adc_trigger() 之后调用（此时本帧电流已搬完）：
 *         ctrl_loop_current_step(&g_ctrl_loop_swing);
 *
 * dt：20kHz 固定 0.05ms，但本函数按 ms 传参，
 *     用 dt_ms=0 的整数会丢失精度 ⇒ 传 dt_x1000=50（见下）。
 * ============================================================ */
void ctrl_loop_current_step(ctrl_loop_t *loop)
{
    if ((loop == RT_NULL) || (loop->inited == 0U) || (loop->enabled == 0U)) {
        return;
    }
    if (loop->ring[CTRL_LOOP_CURRENT].cfg.enable == 0U) {
        return;
    }

    /* ⚠ 20kHz 下 dt_ms 取整为 0 会让积分/微分除零。
     * 这里用 1ms 作为 dt 传入（等价于按 1kHz 的积分增益换算），
     * 实际增益需在整定时按 20 倍关系折算。
     * 真正实现 FOC 时应改为 dt_us 参数版本。 */
    ctrl_loop_run_ring(loop, CTRL_LOOP_CURRENT, 1U);
}

/* ============================================================
 * 初始化
 * ============================================================ */
int ctrl_loop_init(ctrl_loop_t *loop,
                   const char *name,
                   ctrl_loop_get_fb_t get_fb,
                   ctrl_loop_set_out_t set_out,
                   void *user_data)
{
    uint8_t k;

    if ((loop == RT_NULL) || (name == RT_NULL)) {
        return -1;
    }
    if (loop->inited != 0U) {
        return 0;       /* 幂等 */
    }

    memset(loop, 0, sizeof(ctrl_loop_t));
    loop->name      = name;
    loop->get_fb    = get_fb;
    loop->set_out   = set_out;
    loop->user_data = user_data;
    loop->enabled   = 0U;       /* 默认关闭，等 load_cfg 或手工开启 */

    /* 各环默认参数：全部旁路，周期给合理初值 */
    for (k = 0U; k < (uint8_t)CTRL_LOOP_RING_MAX; k++) {
        loop->ring[k].cfg.enable       = 0U;
        loop->ring[k].cfg.d_on_measure = 1U;
        loop->ring[k].cfg.out_max      = 1000;
        loop->ring[k].cfg.out_min      = -1000;
        loop->ring[k].cfg.integ_max    = 300;
        loop->ring[k].period_ms        = 5U;
    }
    loop->ring[CTRL_LOOP_SPEED].period_ms   = 1U;
    loop->ring[CTRL_LOOP_CURRENT].period_ms = 0U;   /* 0=不由 tick 驱动 */

    loop->inited = 1U;

    /* 注册到实例表 */
    if (s_inst_cnt < CTRL_LOOP_MAX_INST) {
        s_inst[s_inst_cnt] = loop;
        s_inst_cnt++;
    }

    return 0;
}

/* ============================================================
 * 参数读写
 * ============================================================ */
void ctrl_loop_set_pid(ctrl_loop_t *loop, ctrl_loop_kind_t kind,
                       const ctrl_pid_cfg_t *cfg)
{
    if ((loop == RT_NULL) || (cfg == RT_NULL) ||
        (kind >= CTRL_LOOP_RING_MAX) || (loop->inited == 0U)) {
        return;
    }
    loop->ring[kind].cfg = *cfg;

    /* 改参数后清积分：旧积分是按旧增益累积的，
     * 沿用会让新参数下出现短暂的错误输出。 */
    loop->ring[kind].integ = 0;
}

void ctrl_loop_get_pid(const ctrl_loop_t *loop, ctrl_loop_kind_t kind,
                       ctrl_pid_cfg_t *out)
{
    if ((loop == RT_NULL) || (out == RT_NULL) ||
        (kind >= CTRL_LOOP_RING_MAX) || (loop->inited == 0U)) {
        return;
    }
    *out = loop->ring[kind].cfg;
}

int ctrl_loop_set_period(ctrl_loop_t *loop, ctrl_loop_kind_t kind,
                         uint32_t period_ms)
{
    if ((loop == RT_NULL) || (kind >= CTRL_LOOP_RING_MAX) ||
        (loop->inited == 0U)) {
        return -1;
    }
    /* 周期 0 表示"不由 tick 驱动"（如电流环），显式允许；
     * 但位置/速度环若设 0 会导致永不执行，视为非法。 */
    if ((period_ms == 0U) && (kind != CTRL_LOOP_CURRENT)) {
        return -1;
    }
    loop->ring[kind].period_ms = period_ms;
    loop->ring[kind].last_tick = 0U;    /* 下次立即执行一次 */
    return 0;
}

void ctrl_loop_reset(ctrl_loop_t *loop, ctrl_loop_kind_t kind)
{
    ctrl_loop_ring_t *ring;

    if ((loop == RT_NULL) || (kind >= CTRL_LOOP_RING_MAX) ||
        (loop->inited == 0U)) {
        return;
    }
    ring = &loop->ring[kind];

    ring->integ     = 0;
    ring->last_meas = 0;
    ring->last_err  = 0;
    ring->last_out  = 0;
    ring->err       = 0;
    ring->run_cnt   = 0U;
    ring->sat_cnt   = 0U;
    ring->skip_cnt  = 0U;
    ring->last_tick = 0U;
}

/* ============================================================
 * 查询
 * ============================================================ */
ctrl_loop_t *ctrl_loop_get(uint8_t inst)
{
    return (inst < s_inst_cnt) ? s_inst[inst] : RT_NULL;
}

uint8_t ctrl_loop_inst_count(void)
{
    return s_inst_cnt;
}