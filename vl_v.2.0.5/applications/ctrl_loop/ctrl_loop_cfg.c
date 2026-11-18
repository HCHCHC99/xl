/**
 * @file    ctrl_loop_cfg.c
 * @brief   三环模块的 sys_config 绑定实现
 */

#include "ctrl_loop_cfg.h"
#include "../config/sys_config.h"
#include <rtthread.h>

/* ============================================================
 * 内部：逐项读 i32，失败返回默认值
 * ------------------------------------------------------------
 * 【为什么逐项 fail-safe 而不是整体失败】
 *   配置表可能因版本升级、flash 损坏而部分项缺失。
 *   若整体失败，整个实例都不能用；
 *   逐项回退则最多是"某一项用默认值"，其余仍生效。
 *   对控制环来说，单参数失效比整体失效更容易排查
 *   （诊断时对比默认值与实测行为即可定位）。
 * ============================================================ */
static int32_t cfg_read_u8(uint8_t inst, uint8_t param, int32_t def)
{
    int32_t v = def;

    if (sys_config_get_i32(CTRL_LOOP_CFG_ID(inst, param), &v) != 0) {
        return def;
    }
    return v;
}

/* ============================================================
 * 装载：sys_config → 实例
 * ============================================================ */
int ctrl_loop_cfg_load(uint8_t inst, ctrl_loop_t *loop)
{
    ctrl_pid_cfg_t *pc;
    ctrl_pid_cfg_t *sc;
    ctrl_pid_cfg_t *cc;
    int32_t v;

    if ((loop == RT_NULL) || (inst >= CTRL_LOOP_MAX_INST)) {
        return -1;
    }
    if (loop->inited == 0U) {
        return -1;
    }

    /* ---- 总开关 ---- */
    v = cfg_read_u8(inst, CLP_ENABLE, CL_DEF_ENABLE);
    loop->enabled = (v != 0) ? 1U : 0U;

    /* ---- 各环使能 ---- */
    v = cfg_read_u8(inst, CLP_POS_EN, CL_DEF_POS_EN);
    loop->ring[CTRL_LOOP_POSITION].cfg.enable = (v != 0) ? 1U : 0U;

    v = cfg_read_u8(inst, CLP_SPD_EN, CL_DEF_SPD_EN);
    loop->ring[CTRL_LOOP_SPEED].cfg.enable = (v != 0) ? 1U : 0U;

    v = cfg_read_u8(inst, CLP_CUR_EN, CL_DEF_CUR_EN);
    loop->ring[CTRL_LOOP_CURRENT].cfg.enable = (v != 0) ? 1U : 0U;

    /* ---- 周期 ----
     * 位置/速度环周期必须 >=1，读到 0 视为配置损坏，回退默认。 */
    v = cfg_read_u8(inst, CLP_POS_PERIOD, (int32_t)CL_DEF_POS_PERIOD);
    if (v < 1) { v = (int32_t)CL_DEF_POS_PERIOD; }
    loop->ring[CTRL_LOOP_POSITION].period_ms = (uint32_t)v;

    v = cfg_read_u8(inst, CLP_SPD_PERIOD, (int32_t)CL_DEF_SPD_PERIOD);
    if (v < 1) { v = (int32_t)CL_DEF_SPD_PERIOD; }
    loop->ring[CTRL_LOOP_SPEED].period_ms = (uint32_t)v;

    /* ---- PID 系数 ---- */
    pc = &loop->ring[CTRL_LOOP_POSITION].cfg;
    pc->kp_x100 = cfg_read_u8(inst, CLP_POS_KP, CL_DEF_POS_KP);
    pc->ki_x100 = cfg_read_u8(inst, CLP_POS_KI, CL_DEF_POS_KI);
    pc->kd_x100 = cfg_read_u8(inst, CLP_POS_KD, CL_DEF_POS_KD);

    sc = &loop->ring[CTRL_LOOP_SPEED].cfg;
    sc->kp_x100 = cfg_read_u8(inst, CLP_SPD_KP, CL_DEF_SPD_KP);
    sc->ki_x100 = cfg_read_u8(inst, CLP_SPD_KI, CL_DEF_SPD_KI);
    sc->kd_x100 = cfg_read_u8(inst, CLP_SPD_KD, CL_DEF_SPD_KD);

    cc = &loop->ring[CTRL_LOOP_CURRENT].cfg;
    cc->kp_x100 = cfg_read_u8(inst, CLP_CUR_KP, CL_DEF_CUR_KP);

    /* ---- 输出/积分限幅：不从配置表读 ----
     *
     * 为什么不做成可配置：
     *   这两个值是"安全边界"，不是"调节参数"。
     *   现场误把 out_max 调到 5000，会让占空比请求超出硬件能力，
     *   可能触发过流保护或损坏功率级。
     *   固定值由硬件能力决定，写在 ctrl_loop_init() 里。
     *   若确需可调，应加 CFG_F_REQ_SAFE 标志（要求处于安全模式）。 */

    /* ---- 清积分：装载后从零开始 ---- */
    ctrl_loop_reset(loop, CTRL_LOOP_POSITION);
    ctrl_loop_reset(loop, CTRL_LOOP_SPEED);
    ctrl_loop_reset(loop, CTRL_LOOP_CURRENT);

    return 0;
}

/* ============================================================
 * 保存：实例 → sys_config（RAM）
 * ============================================================ */
int ctrl_loop_cfg_store(uint8_t inst, const ctrl_loop_t *loop)
{
    const ctrl_pid_cfg_t *pc;
    const ctrl_pid_cfg_t *sc;
    const ctrl_pid_cfg_t *cc;
    int ret = 0;

    if ((loop == RT_NULL) || (inst >= CTRL_LOOP_MAX_INST) ||
        (loop->inited == 0U)) {
        return -1;
    }

    /* 任何一项写失败都记为失败，但继续写其余项 ——
     * 让尽可能多的参数落盘，而不是第一条失败就全放弃。 */
    if (sys_config_set_i32(CTRL_LOOP_CFG_ID(inst, CLP_ENABLE),
                           (int32_t)loop->enabled) != 0) { ret = -1; }

    if (sys_config_set_i32(CTRL_LOOP_CFG_ID(inst, CLP_POS_EN),
                           (int32_t)loop->ring[CTRL_LOOP_POSITION].cfg.enable) != 0) { ret = -1; }
    if (sys_config_set_i32(CTRL_LOOP_CFG_ID(inst, CLP_SPD_EN),
                           (int32_t)loop->ring[CTRL_LOOP_SPEED].cfg.enable) != 0) { ret = -1; }
    if (sys_config_set_i32(CTRL_LOOP_CFG_ID(inst, CLP_CUR_EN),
                           (int32_t)loop->ring[CTRL_LOOP_CURRENT].cfg.enable) != 0) { ret = -1; }

    if (sys_config_set_i32(CTRL_LOOP_CFG_ID(inst, CLP_POS_PERIOD),
                           (int32_t)loop->ring[CTRL_LOOP_POSITION].period_ms) != 0) { ret = -1; }
    if (sys_config_set_i32(CTRL_LOOP_CFG_ID(inst, CLP_SPD_PERIOD),
                           (int32_t)loop->ring[CTRL_LOOP_SPEED].period_ms) != 0) { ret = -1; }

    pc = &loop->ring[CTRL_LOOP_POSITION].cfg;
    if (sys_config_set_i32(CTRL_LOOP_CFG_ID(inst, CLP_POS_KP), pc->kp_x100) != 0) { ret = -1; }
    if (sys_config_set_i32(CTRL_LOOP_CFG_ID(inst, CLP_POS_KI), pc->ki_x100) != 0) { ret = -1; }
    if (sys_config_set_i32(CTRL_LOOP_CFG_ID(inst, CLP_POS_KD), pc->kd_x100) != 0) { ret = -1; }

    sc = &loop->ring[CTRL_LOOP_SPEED].cfg;
    if (sys_config_set_i32(CTRL_LOOP_CFG_ID(inst, CLP_SPD_KP), sc->kp_x100) != 0) { ret = -1; }
    if (sys_config_set_i32(CTRL_LOOP_CFG_ID(inst, CLP_SPD_KI), sc->ki_x100) != 0) { ret = -1; }
    if (sys_config_set_i32(CTRL_LOOP_CFG_ID(inst, CLP_SPD_KD), sc->kd_x100) != 0) { ret = -1; }

    cc = &loop->ring[CTRL_LOOP_CURRENT].cfg;
    if (sys_config_set_i32(CTRL_LOOP_CFG_ID(inst, CLP_CUR_KP), cc->kp_x100) != 0) { ret = -1; }

    return ret;
}

/* ============================================================
 * 落盘
 * ============================================================ */
int ctrl_loop_cfg_save(void)
{
    return sys_config_save();
}

/* ============================================================
 * 登记校验
 * ------------------------------------------------------------
 * 本工程的配置表 s_cfg_def[] 是**编译期静态数组**，
 * 无法在运行时追加表项。所以本函数只做一项工作：
 * 检查静态表里是否真的登记了 4 实例 × 13 项。
 *
 * 为什么需要这个检查：
 *   若将来有人改了 CTRL_LOOP_MAX_INST 或 PARAM_CNT 却忘记
 *   同步 sys_config.c 的 s_cfg_def[]，运行时会表现为
 *   "读到的全是默认值" —— 极难定位（配置项不存在时
 *   sys_config_get_i32 返回失败，我们的 fail-safe 会静默用默认值）。
 *   有了这个检查，上电时就能发现不一致。
 * ============================================================ */
uint32_t ctrl_loop_cfg_register(void)
{
    uint32_t found = 0U;
    uint8_t  inst;
    uint8_t  param;

    for (inst = 0U; inst < CTRL_LOOP_MAX_INST; inst++) {
        for (param = 0U; param < CTRL_LOOP_PARAM_CNT; param++) {
            uint16_t id = CTRL_LOOP_CFG_ID(inst, param);

            if (sys_config_find_def(id) != RT_NULL) {
                found++;
            }
        }
    }

    if (found != (uint32_t)(CTRL_LOOP_MAX_INST * CTRL_LOOP_PARAM_CNT)) {
        rt_kprintf("[CtrlLoop] WARN: cfg registered %u/%u, "
                   "check sys_config.c s_cfg_def[]\n",
                   (unsigned)found,
                   (unsigned)(CTRL_LOOP_MAX_INST * CTRL_LOOP_PARAM_CNT));
    }

    return found;
}