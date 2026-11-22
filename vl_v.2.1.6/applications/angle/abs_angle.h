/**
 * @file    abs_angle.h
 * @brief   回转角度解算 —— 两段式算法（上电游标 / 运行增量）
 * @note    PLAN-ABS-ANGLE 落地方案，2026-10-28
 *
 * ============================================================
 * 【为什么分两段，而不是一直用游标】
 * ------------------------------------------------------------
 * 用户 2026-10-28 裁定（原话）：
 *   "回转角度只需要在上电期间采用解算算法，运行过程中因为是增量，
 *    所以回转角度可以采用更加简单的算法。"
 *
 * 这个判断在数学上完全成立，且是**更优**的方案，理由三条：
 *
 * 1. 上电时不知道绝对位置 ⇒ 必须靠游标（Z0/Z1/Z2 三颗齿差）反解圈数 k；
 *    这是"绝对编码"的代价，逃不掉。
 *
 * 2. 上电解出 k 之后，Z0 本身的 12bit 绝对值 + 已知的 k，
 *    就完全确定了输出轴角度。继续读 Z1/Z2 是**冗余**的。
 *
 * 3. 运行期输出轴连续转动，只要保证"采样间隔内 Z0 转过不到半圈"，
 *    就能用相邻两次 Z0 计数差累加。半圈 = 2048 LSB。
 *
 * ------------------------------------------------------------
 * 【但增量法有两个必须防的坑，本模块都处理了】
 *
 * 坑1：转动快时 Z0 会溢出多圈，单纯按差值累加会"吞圈"。
 *   默认 A2 配置 i_total = 1080，电机 3000 rpm 时 Z0 转速
 *   = 3000*1080 = 3,240,000 rpm?? —— 不，是电机转速经减速器后
 *   输出轴 3000/1080 ≈ 2.78 rpm，Z0 随输出轴前的齿轮高速转。
 *   按 Z0 侧转速上限 3000 rpm、1 kHz 采样算：
 *       每 ms 转 3000/60/1000 = 0.05 圈 = 205 LSB
 *   远小于 2048，安全。但**若采样周期变长（如 5ms 控制周期）
 *   或转速更高，必须重新核算**，故本模块把上限做成编译期检查。
 *
 * 坑2：Z0 单圈绝对值在 4095→0 处跳变，增量符号会突变。
 *   处理：先算 raw_delta，再做 ±2048 环绕修正，得到真实增量。
 *
 * 坑3（最隐蔽）：掉电/复位后累计值丢失，但如果不清零直接接着跑，
 *   会得到错误的绝对角度。处理：见 abs_angle_is_valid()，
 *   未完成上电解算前，angle 接口一律返回无效。
 * ============================================================
 */

#ifndef __ABS_ANGLE_H
#define __ABS_ANGLE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 错误码
 * ============================================================ */
typedef enum {
    ABS_ANGLE_OK            = 0,
    ABS_ANGLE_ERR_NOT_INIT  = 1,   /* 上电解算未完成 */
    ABS_ANGLE_ERR_ENC_Z0    = 2,   /* Z0 读失败（CRC 或 SPI） */
    ABS_ANGLE_ERR_ENC_Z1    = 3,   /* Z1 读失败 */
    ABS_ANGLE_ERR_ENC_Z2    = 4,   /* Z2 读失败 */
    ABS_ANGLE_ERR_NO_MATCH  = 5,   /* 游标三颗组合不落在任何合法 k */
    ABS_ANGLE_ERR_AMBIGUOUS = 6,   /* 多个 k 同时满足（齿型选取错误） */
    ABS_ANGLE_ERR_OUT_RANGE = 7,   /* 解出的角度超出行程范围 */
    ABS_ANGLE_ERR_JUMP      = 8,   /* 增量跳变过大（丢步/干扰，需重解算） */
    ABS_ANGLE_ERR_NO_CFG    = 9,   /* 机械配置未加载 */
} abs_angle_err_t;

/* ============================================================
 * 工作模式
 * ============================================================ */
typedef enum {
    ABS_ANGLE_MODE_UNINIT  = 0,    /* 未初始化 */
    ABS_ANGLE_MODE_COARSE  = 1,    /* 已解算，但只用游标（未切增量） */
    ABS_ANGLE_MODE_INCR    = 2,    /* 增量跟踪中（正常运行态） */
    ABS_ANGLE_MODE_LOST    = 3,    /* 增量丢失，需重新游标解算 */
} abs_angle_mode_t;

/* ============================================================
 * 状态/统计（供 MSH 诊断）
 * ============================================================ */
typedef struct {
    abs_angle_mode_t mode;
    abs_angle_err_t  last_err;

    int32_t  k;             /* 上电解算出的 Z0 绝对圈数 */
    int32_t  cnt_accum;     /* Z0 累计计数（可为负，含 k*4096） */
    int32_t  raw_z0;        /* 上次 Z0 原始值，用于算增量（-1 表示未采） */
    uint32_t angle_mdeg;    /* 最近一次输出角度，0.001 deg */

    uint32_t solve_cnt;     /* 游标解算次数 */
    uint32_t incr_cnt;      /* 增量更新次数 */
    uint32_t jump_cnt;      /* 判为跳变的次数 */
    uint32_t fail_cnt;      /* 各类失败累计 */

    /* ---- 【2026-11-02 新增】解算中间量，供方向诊断 ---- */
    int32_t  dbg_raw0;      /* 本次解算用的 Z0 原始值（已减零点） */
    int32_t  dbg_raw1;      /* Z1 原始值（已减零点） */
    int32_t  dbg_raw2;      /* Z2 原始值（已减零点） */
    int32_t  dbg_best_k;    /* 残差最小的 k（失败时也有值） */
    int32_t  dbg_r1;        /* 该 k 下的 Z1 残差（符号有意义） */
    int32_t  dbg_r2;        /* 该 k 下的 Z2 残差 */
    int32_t  dbg_score;     /* |r1| + |r2| */
    int32_t  dbg_second;    /* 次优 score */
    int32_t  dbg_guard;     /* 本次判据用的唯一性容限 */
} abs_angle_state_t;

/* ============================================================
 * API
 * ============================================================ */

/**
 * @brief 上电解算：读三颗编码器，用游标反解 Z0 绝对圈数 k
 * @param[out] err  失败原因，可为 NULL
 * @return ABS_ANGLE_OK 成功
 * @note  必须在 mech_cfg_store_init() 之后调用（依赖减速比与行程）
 *        本函数会做多次采样 + 一致性校验，不是单次读取
 */
abs_angle_err_t abs_angle_solve(abs_angle_err_t *err);

/**
 * @brief 周期调用：增量更新（运行期主路径）
 * @param now_ms  当前毫秒时基
 * @return 当前错误码（OK 表示正常）
 * @note  内部只读 Z0 一颗，SPI 开销 11.04 μs @6.25 Mbps
 *        若检测到跳变，自动切 ABS_ANGLE_MODE_LOST 并提示需重解算
 */
abs_angle_err_t abs_angle_update(uint32_t now_ms);

/**
 * @brief 取当前回转角度
 * @param[out] mdeg  0.001 度为单位
 * @return ABS_ANGLE_OK 成功；其他=无效值，mdeg 内容不可用
 */
abs_angle_err_t abs_angle_get_mdeg(int32_t *mdeg);

/**
 * @brief 取当前回转角度（0.1 度，兼容 mech_cfg 的 out_x10 口径）
 */
abs_angle_err_t abs_angle_get_x10(int32_t *x10);

/**
 * @brief 强制失效（切 profile、标定后调用）
 */
void abs_angle_invalidate(void);

/**
 * @brief 取状态快照
 */
void abs_angle_get_state(abs_angle_state_t *out);

/**
 * @brief 取模式名（打印用）
 */
const char *abs_angle_mode_name(abs_angle_mode_t m);

/**
 * @brief 取错误名（打印用）
 */
const char *abs_angle_err_name(abs_angle_err_t e);

#ifdef __cplusplus
}
#endif

#endif /* __ABS_ANGLE_H */