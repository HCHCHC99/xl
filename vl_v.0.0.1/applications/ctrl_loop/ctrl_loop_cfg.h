/**
 * @file    ctrl_loop_cfg.h
 * @brief   三环模块的 sys_config 绑定
 *
 * @note    【ID 编码规则】
 *            0x0C [实例号 4bit] [参数号 4bit]
 *
 *            实例号 0~3，参数号 0~15
 *            CTRL_LOOP_CFG_ID(inst, param)
 *
 *          【为什么按"实例×参数"编码，而不是每项一个独立 ID】
 *            4 实例 × 13 参数 = 52 项。若按语义命名各建一个 ID
 *            （如 cl0_pos_kp、cl0_pos_ki、cl1_pos_kp…），
 *            维护成本随实例数线性膨胀，且新增实例要改 13 处代码。
 *            位域编码后，读写循环可统一处理，新增实例零改动。
 *
 *          【每实例 13 项参数的分配】
 *            0  总使能          u8
 *            1  位置环使能      u8
 *            2  速度环使能      u8
 *            3  电流环使能      u8
 *            4  位置环周期 ms   u16
 *            5  速度环周期 ms   u16
 *            6  位置 kp_x100    i32
 *            7  位置 ki_x100    i32
 *            8  位置 kd_x100    i32
 *            9  速度 kp_x100    i32
 *            10 速度 ki_x100    i32
 *            11 速度 kd_x100    i32
 *            12 电流 kp_x100    i32
 *
 *          【容量核算（2026-11-10）】
 *            扩前 96 项（已用 50），4 实例需 52 项 → 超
 *            CFG_MAX_ITEMS 已扩到 128：50 + 52 = 102 ≤ 128 ✓
 *            余量 26 项。
 *
 *          【电流环只配 kp 的原因】
 *            FOC 未实现（MARK-CL-01），ki/kd 配了也无处用。
 *            先占一个 ID 位，将来补齐时按 13→15 扩展不影响已有编码。
 */

#ifndef __CTRL_LOOP_CFG_H
#define __CTRL_LOOP_CFG_H

#include <stdint.h>
#include "ctrl_loop.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * ID 编码
 * ============================================================ */
#define CTRL_LOOP_CFG_BASE      0x0C00U
#define CTRL_LOOP_CFG_ID(inst, param)                       \
    ((uint16_t)(CTRL_LOOP_CFG_BASE |                        \
                (((uint16_t)(inst)  & 0x03U) << 4) |        \
                ((uint16_t)(param) & 0x0FU)))

/* ---- 参数号 ---- */
#define CLP_ENABLE          0U   /* 实例总使能 */
#define CLP_POS_EN          1U
#define CLP_SPD_EN          2U
#define CLP_CUR_EN          3U
#define CLP_POS_PERIOD      4U
#define CLP_SPD_PERIOD      5U
#define CLP_POS_KP          6U
#define CLP_POS_KI          7U
#define CLP_POS_KD          8U
#define CLP_SPD_KP          9U
#define CLP_SPD_KI          10U
#define CLP_SPD_KD          11U
#define CLP_CUR_KP          12U

#define CTRL_LOOP_PARAM_CNT 13U

/* ============================================================
 * 默认值（编译期兜底，配置表读失败时使用）
 * ------------------------------------------------------------
 * 这些值必须"安全"：宁可不动，也不能乱动。
 *   enable 全 0     → 旁路，输出等于目标，不会突加驱动
 *   输出限幅 ±1000  → 对应占空比 100%（六步方波满幅）
 *   积分限幅 300    → 约为输出的 30%，常规整定起点
 * ============================================================ */
#define CL_DEF_ENABLE        (0U)
#define CL_DEF_POS_EN        (0U)
#define CL_DEF_SPD_EN        (0U)
#define CL_DEF_CUR_EN        (0U)
#define CL_DEF_POS_PERIOD    (5U)       /* 200 Hz */
#define CL_DEF_SPD_PERIOD    (1U)       /* 1 kHz */
#define CL_DEF_POS_KP        (0)        /* 0 = 纯比例无作用，需现场整定 */
#define CL_DEF_POS_KI        (0)
#define CL_DEF_POS_KD        (0)
#define CL_DEF_SPD_KP        (0)
#define CL_DEF_SPD_KI        (0)
#define CL_DEF_SPD_KD        (0)
#define CL_DEF_CUR_KP        (0)

/* ============================================================
 * API
 * ============================================================ */

/**
 * @brief 从 sys_config 装载某实例的参数到 RAM
 *
 * @param inst  实例序号（0~3）
 * @param loop  实例指针
 * @retval 0 成功（逐项 fail-safe：单项读失败用编译期默认值）
 *
 * @note  所谓的"成功"只表示装载流程走完，
 *        不代表所有项都从 flash 读到了 —— 单项失败静默回退默认值。
 *        要确认实际来源用 `cl show <inst>` 看来源列。
 */
int ctrl_loop_cfg_load(uint8_t inst, ctrl_loop_t *loop);

/**
 * @brief 把实例当前参数写回 sys_config RAM（不落盘）
 */
int ctrl_loop_cfg_store(uint8_t inst, const ctrl_loop_t *loop);

/**
 * @brief 把参数落盘（调 sys_config_save）
 */
int ctrl_loop_cfg_save(void);

/**
 * @brief 注册 4 个实例的配置项到 s_cfg_def[]
 *
 * @note  必须在 sys_config_init() 之前调用（配置表是编译期常量数组，
 *        本工程采用静态登记，见 sys_config.c 的 s_cfg_def[]）。
 *        故本函数实际只做"校验"，真正的登记是静态表项。
 * @retval 已登记的项数
 */
uint32_t ctrl_loop_cfg_register(void);

#ifdef __cplusplus
}
#endif

#endif /* __CTRL_LOOP_CFG_H */