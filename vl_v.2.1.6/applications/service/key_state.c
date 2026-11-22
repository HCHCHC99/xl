/**
 * @file    key_state.c
 * @brief   钥匙（ACC）上电状态检测实现（PB0 低电平有效 + 500ms 去抖）
 * @note    【D-9 / 2026-11-14】符号已统一为 acc_*（Q-76/Q-66），t_key_state 已删（Q-67）。
 *          **文件名保留 key_state.c/.h**：Eclipse 托管构建在 Debug 目录下
 *          自动生成各模块的 subdir.mk，其中写死了源列表；重命名 .c 会丢源（B-03）。
 *          历史文件名不代表术语。
 *          三层拓扑：第0层电闸12VDC → 本系统取电；第1层钥匙(ACC) → 其他机构具备条件；
 *          第2层启车 → 动力可用。本模块只管第1层。
 *
 * 设计要点见 key_state.h 顶部注释（Q33 独立 / Q34 外部上拉）。
 *
 * ⚠️ 关键工程约束（本项目踩过的坑，勿再犯）：
 *   PB0 属于 GPIO 配置，写入必须夹在 LL_PERIPH_WE(LL_PERIPH_GPIO) /
 *   LL_PERIPH_WP(...) 之间，否则寄存器写入被静默忽略，
 *   现象是"GPIO_Init 返回 OK 但引脚行为完全不变"。
 *   参见 bsp_drv_spi.c:78-90 的事故记录（D-57）。
 */

#include "key_state.h"
#include "app_event.h"

#include "hc32_ll.h"
#include "hc32_ll_gpio.h"

#include <rtthread.h>
#include <finsh.h>

#include "../log/log_color.h"
#include "../config/sys_config.h"   /* E-3: 去抖参数从配置表装载 */
#include "input_sim.h"              /* 台架模拟（任务清单 §11） */

/* ============================================================
 * 引脚定义（Q34：外部 10k 上拉到 3.3V，内部上拉必须关闭）
 * ============================================================ */
#define KEY_PIN_PORT            (GPIO_PORT_B)
#define KEY_PIN_PIN             (GPIO_PIN_00)

/* 低电平有效：读 0 表示钥匙上电 */
#define KEY_LEVEL_ON            (PIN_RESET)     /* 0 */

/* ============================================================
 * 判定参数
 * ============================================================ */
/* E-3：ROM 默认值 + RAM 运行副本。init 时用配置表覆盖，
 * 读取失败（sys_config 为 BOOT_OPTIONAL）则保持以下默认值，不静默改行为。 */
acc_state_cfg_t g_acc_state_cfg = {
    .enter_ms = 500U,       /* 上电去抖 500ms */
    .exit_ms  = 500U,       /* 下电去抖 500ms（E-3 裁定值）*/
    .poll_ms  = 50U,        /* 与 50ms tick 一致 */
};

/* ============================================================
 * 模块状态
 * ============================================================ */
static uint8_t  s_acc_on   = 0U;    /* 去抖后的稳定状态 */
static uint16_t s_cond_ms  = 0U;    /* 当前方向累计时长 */
static uint8_t  s_inited   = 0U;


/* ============================================================
 * 初始化
 * ============================================================ */
int acc_state_init(void)
{
    stc_gpio_init_t g;

    LL_PERIPH_WE(LL_PERIPH_GPIO);

    (void)GPIO_StructInit(&g);
    g.u16PinDir   = PIN_DIR_IN;         /* 输入 */
    g.u16PullUp   = PIN_PU_OFF;         /* Q34：外部已有 10k 上拉，内部必须关闭 */
    g.u16Invert   = PIN_INVT_OFF;       /* 不反相：低电平就是低电平 */
    g.u16PinAttr  = PIN_ATTR_DIGITAL;   /* 数字输入 */
    g.u16ExtInt   = PIN_EXTINT_OFF;     /* 不用外部中断，走 50ms 轮询 */
    (void)GPIO_Init(KEY_PIN_PORT, KEY_PIN_PIN, &g);

    LL_PERIPH_WP(LL_PERIPH_GPIO);

    /* ---- E-3：从配置表装载去抖参数到运行副本 ----------------------
     * R6：只在 init 读一次，acc_poll() 不直读配置表。
     * 顺序：sys_config 在 BOOT_LV_STORAGE(10)，本模块 BOOT_LV_SERVICE(60)，
     *       实测 boot_table.c:161 / :222，配置表先就绪。
     * 失败回退：sys_config_get_u32 返回 <0 时保留上面的 ROM 默认值。 */
    {
        uint32_t v;
        if (sys_config_get_u32(CFG_ID_ACC_ENTER_MS, &v) == 0) {
            g_acc_state_cfg.enter_ms = (uint16_t)v;
        }
        if (sys_config_get_u32(CFG_ID_ACC_EXIT_MS, &v) == 0) {
            g_acc_state_cfg.exit_ms = (uint16_t)v;
        }
        if (sys_config_get_u32(CFG_ID_ACC_POLL_MS, &v) == 0) {
            g_acc_state_cfg.poll_ms = (uint16_t)v;
        }
    }

    s_acc_on   = 0U;
    s_cond_ms  = 0U;
    s_inited   = 1U;

    LOG_INFO("[ACC] init PB0 (active-low, external 10k pull-up, internal PU OFF)\n");
    LOG_INFO("[ACC]   on  > %ums low-level\n", (unsigned)g_acc_state_cfg.enter_ms);
    LOG_INFO("[ACC]   off > %ums high-level\n", (unsigned)g_acc_state_cfg.exit_ms);
    return 0;
}

/* ============================================================
 * 周期轮询（50ms tick 调用）
 * ============================================================ */
void acc_poll(void)
{
    uint8_t raw;
    uint8_t cond;       /* 1 = 该方向条件已满足 */

    if (s_inited == 0U) {
        return;
    }

    raw = acc_read_raw_level();

    if (s_acc_on == 0U) {
        /* ---------- 当前为下电：判是否上电（低电平） ---------- */
        cond = (raw == KEY_LEVEL_ON) ? 1U : 0U;

        if (cond == 0U) {
            s_cond_ms = 0U;             /* 条件中断，重新计时 */
            return;
        }

        s_cond_ms += g_acc_state_cfg.poll_ms;
        if (s_cond_ms < g_acc_state_cfg.enter_ms) {
            return;
        }

        /* ---- 确认钥匙上电 ---- */
        s_acc_on  = 1U;
        s_cond_ms = 0U;

        LOG_OK("[ACC] ON  (PB0 low for %ums)\n",
               (unsigned)g_acc_state_cfg.enter_ms);
        post_event(SYS_EVT_ACC_ON);

    } else {
        /* ---------- 当前为上电：判是否下电（高电平） ---------- */
        cond = (raw != KEY_LEVEL_ON) ? 1U : 0U;

        if (cond == 0U) {
            s_cond_ms = 0U;
            return;
        }

        s_cond_ms += g_acc_state_cfg.poll_ms;
        if (s_cond_ms < g_acc_state_cfg.exit_ms) {
            return;
        }

        /* ---- 确认钥匙下电 ---- */
        s_acc_on  = 0U;
        s_cond_ms = 0U;

        LOG_WARN("[ACC] OFF (PB0 high for %ums)\n",
                 (unsigned)g_acc_state_cfg.exit_ms);
        post_event(SYS_EVT_ACC_OFF);
    }
}

/* ============================================================
 * 查询
 * ============================================================ */
uint8_t acc_is_on(void)
{
    return s_acc_on;
}

uint8_t acc_read_raw_level(void)
{
#if (INPUT_SIM_ENABLE && ACC_SIM_ENABLE)
    /* 台架模拟：t_sim acc on|off 覆盖 PB0 电平（去抖仍在 acc_poll 里跑） */
    if (input_sim_active(INPUT_SIM_ACC) != 0U) {
        return (uint8_t)((input_sim_value(INPUT_SIM_ACC) != 0) ? 1U : 0U);
    }
#endif
    return (GPIO_ReadInputPins(KEY_PIN_PORT, KEY_PIN_PIN) != PIN_RESET) ? 1U : 0U;
}

void acc_force_reset(void)
{
    s_acc_on  = 0U;
    s_cond_ms = 0U;
    LOG_WARN("[ACC] force reset -> OFF\n");
}

/* ============================================================
 * 【Q-67 / 2026-11-14】原 t_key_state / t_key_state_clr 两个 MSH 命令已删除。
 *   t_veh 已含 acc_on 字段（vehicle_control.c 的 t_vehicle），观测入口统一。
 *   连函数一并删除：保留未使用的 static 函数会触发 -Wunused-function。
 * ============================================================ */