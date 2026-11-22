/**
 * @file    mech_cfg_store.h
 * @brief   机械配置表持久化（mech_param 分区）+ 全局生效实例
 * @note    PLAN-ABS-ANGLE-V1.3 §5.1.9、§6.2 方案 A
 *
 * ============================================================
 * 为什么用独立分区而不是扩 sys_param
 * ============================================================
 * Q-40 裁定为多行表后，整表需 236 B。塞进 sys_param_t 会：
 *   1) 破坏"机械参数(出厂级) / 零点(现场资产)"的解耦，机械参数改一次
 *      连带重写零点区，擦写次数与掉电风险翻倍；
 *   2) sys_param.h:36-38 明确"字段顺序与 sizeof 不得随意变更"，
 *      扩结构体等于让现场已标定设备的零点丢失。
 * 故方案 B 不可行，独占 FLASH_ADDR_MECH_PARAM。
 *
 * ============================================================
 * C-08 切 profile 必须褪零
 * ============================================================
 * cnt0_zero 属于旧 profile 的齿数模型，换行后同一计数的物理角度
 * 完全不同。切行必须原子完成：
 *   ① cnt0_zero = 0
 *   ② is_calibrated = 0
 *   ③ 重跑游标解算
 * 最后才落盘 active 索引 —— 此顺序保证掉电后"未校准"状态可自愈。
 * ============================================================
 */

#ifndef __MECH_CFG_STORE_H
#define __MECH_CFG_STORE_H

#include "mech_cfg.h"
#include "../sys_param/sys_param.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 全局对象
 * ============================================================ */
typedef struct {
    mech_table_t tbl;          /* 落盘表（RAM 镜像） */
    mech_rt_t    rt;           /* 当前 active 行的派生量 */
    mech_cfg_t   cfg;          /* 当前 active 行的配置（冗余，便于访问） */
    int32_t      active_idx;   /* 当前生效行，等同 tbl.active */

    uint8_t      is_loaded;    /* 表已加载 */
    uint8_t      cfg_fault;    /* 上次加载/校验失败，已回默认 */
    uint8_t      src_is_flash; /* 1=来自 Flash，0=来自默认值 */
} mech_ctx_t;

extern mech_ctx_t g_mech;

/* ============================================================
 * API
 * ============================================================ */

/**
 * @brief 初始化：建 storage → 加载表 → 校验 → 派生
 * @return 0 成功（含"回默认并置 cfg_fault"的情形）；<0 存储层失败
 * @note  必须由 boot 表在 storage 之后、abs_angle 之前调用
 */
int mech_cfg_store_init(void);

/**
 * @brief 从 Flash 重新加载整表（不派生）
 */
int mech_table_load(mech_table_t *tbl);

/**
 * @brief 校验后落盘整表
 * @note  写前先 mech_table_validate；非法直接拒绝，不动 Flash
 */
int mech_table_save(const mech_table_t *tbl);

/**
 * @brief 仅刷新派生量（active 行）
 */
int mech_apply_active(void);

/**
 * @brief 切换 active 行（含 C-08 褪零），并落盘
 * @param idx      目标行索引
 * @param force    0 = 只打印警告不执行；1 = 执行
 * @return 0 成功；MECH_ERR_PARAM 索引非法；MECH_ERR_VALIDATE 该行非法
 */
int mech_set_active_profile(uint8_t idx, uint8_t force);

/**
 * @brief 按 teeth_type 自动选行（不改校准状态；仅换行且 new 行合法时褪零）
 * @note  供上电时按机型选默认行；运行中不建议用（会清校准）
 */
int mech_select_by_teeth(uint8_t teeth_type);

/**
 * @brief 按 sys_config 的三个选择子（0x0A01/02/03）上电选行
 *
 * @note  【2026-11-04 新增，用户裁定 Q5="一并纳入"】
 *        传动链三段正交：编码器齿数比 / 电机内部减速比 / 机型齿比，
 *        故用三个独立配置项而非单一"机型码"。本函数把它们合成候选 id，
 *        与 Flash 里已落盘的行比对：
 *          - 相同  → 什么都不做（**不褪零**，保护现场标定）
 *          - 不同  → apply + 褪零 + 重跑解算 + 落盘
 *
 *        由 mech_cfg_store_init() 自动调用，无需外部再调。
 *        单独暴露是为便于 MSH 诊断（改完 t_cfg 后免重启验证选行逻辑）。
 *
 * @return MECH_OK 无需变更或已成功切换；<0 失败（保持原行，不半切）
 */
int mech_apply_cfg_selection(void);

/**
 * @brief 判断一行机械配置与某候选是否等价
 *
 * @note  只比"影响 i_total 与游标解算"的字段（齿数比 / 两个减速比 /
 *        极对数 / 行程），**不比** version / crc32 / tol_override。
 *        后者变化不需要褪零 —— tol_override 是调试参数，改它不该清标定。
 *
 *        供两处复用，避免各写一份比较逻辑：
 *          - mech_cfg_store.c 的上电选行
 *          - mech_cmd.c 的 t_mech_model "无变化则不褪零"判断
 *
 * @return 1 等价；0 不同或候选非法
 */
uint8_t mech_row_equals_cand(const mech_cfg_t *row, uint8_t cand_id);

/**
 * @brief 取当前生效行指针
 */
const mech_cfg_t* mech_get_cfg(void);
const mech_rt_t*  mech_get_rt(void);

/**
 * @brief 打印整表（t_mech_list 用）
 */
void mech_table_dump(const mech_table_t *tbl);

/**
 * @brief 角度层失效通知（切 profile 后重跑游标解算）
 *
 * @note  【W-01 修复 2026-10-27】原先只有 `.c` 里的 RT_WEAK 定义，
 *        头文件缺声明，导致 mech_cmd.c 调用时触发
 *        `warning: implicit declaration of function 'abs_angle_invalidate'`。
 *
 *        隐式声明的真实危害不是警告本身，而是编译器按默认规则
 *        假设返回 int、参数不定 —— 一旦将来此函数改成带参数或非 void
 *        返回，调用点的寄存器/栈布局会与定义不匹配，症状是随机跑飞。
 *
 *        弱符号语义：本工程未接入 abs_angle 时，mech_cfg_store.c 的
 *        空实现兜底；abs_angle.c 接入后提供同名强符号自动覆盖，
 *        无需改本文件。声明放在此处即对所有调用方可见。
 */
void abs_angle_invalidate(void);

#ifdef __cplusplus
}
#endif

#endif /* __MECH_CFG_STORE_H */