/**
 * @file    seq_arbit.h
 * @brief   一键序列的专属仲裁规则（P3）
 * @note    纯规则文件：零 RTOS 依赖、无全局状态，可用 host gcc 單独编译/单测。
 *          只回答"这个一键请求现在允不允许"，不做任何动作、不读硬件。
 *
 *          规则来源（xl.xlsx）：
 *            TC-PRI-002  三个一键同时按下 → 均不启动
 *            TC-ANG-011  超右（>+217°）→ 右转、右后无输出；右中/回位可执行
 *            TC-PRI-008  超右（+218°）→ 右后禁止；右中、回位可执行
 *            TC-ANG-012  超左（<−102°）→ 左转无输出；三种一键均可执行
 *            TC-PRI-009  超左 → 三种一键动作均可执行
 *
 *          注意：超右时"右后"虽然是从 +218° 往 +215° 的**向内左转**，
 *          用例仍要求禁止，故这里按用例判、不按方向判。
 */

#ifndef __SEQ_ARBIT_H
#define __SEQ_ARBIT_H

#include <stdint.h>
#include "seq_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 门槛（单位 0.1°）
 * ------------------------------------------------------------
 * 与 swing_angle 的电子限位一致：右 +215°（软件 +216°）、左 −100°（软件 −101°）。
 * 仲裁用的是"超过即禁"的守卫角，比电子限位再外扩 1°，与 TC-ANG-011/012 的
 * +217° / −102° 取整对应。
 *
 * TODO(batch4)：按 §14.4「新参数只在 sys_config 定义一次」应进 SYSTEM 慢区；
 * 本轮为不动参数区布局（会牵连 PARAM_SYSTEM_CFG_COUNT / 记录长度 / 魔数），
 * 先用常量，批次 4 统一迁移。
 * ============================================================ */
#define SEQ_ARB_RIGHT_GUARD_X10     (2170)      /* +217.0° */
#define SEQ_ARB_LEFT_GUARD_X10      (-1025)     /* −102.5° */

/* ============================================================
 * 判定结果
 * ============================================================ */
typedef enum {
    SEQ_ARB_OK = 0,                 /* 允许 */
    SEQ_ARB_DENY_MULTI_KEY,         /* 同一拍按下 ≥2 个一键 */
    SEQ_ARB_DENY_LIMIT_RIGHT,       /* 超右限位：右后禁止 */
    SEQ_ARB_DENY_ANGLE_INVALID,     /* 角度无效（预留，本轮不启用）*/
} seq_arb_t;

/**
 * @brief 判断一个一键请求当前是否允许
 * @param type       目标序列类型
 * @param swing_x10  当前筒水平角（0.1°），由调用方从角度 getter 取
 * @param key_count  同一拍里按下的一键个数（1 = 单键；≥2 = 多键同按）
 * @return SEQ_ARB_OK 允许；否则为拒绝原因
 */
seq_arb_t seq_arbit_check_onekey(seq_type_t type, int32_t swing_x10, uint8_t key_count);

/**
 * @brief 拒绝原因的可读文本（ASCII，供 RTT 打印）
 */
const char *seq_arbit_reason(seq_arb_t r);

#ifdef __cplusplus
}
#endif

#endif /* __SEQ_ARBIT_H */