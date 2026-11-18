/**
 * @file    swing_angle.c
 * @brief   回转角度对象实现
 */

#include "swing_angle.h"

/* 【2026-11-10 彩标 P0】角度解算失效=FAULT（安全事件，红底）*/
#include "../log/log_color.h"
#include "framework/state_engine/sm_assert.h"
#include <stddef.h>
#include <rtthread.h>
#include "../angle/abs_angle.h"
#include "app_event.h"

/* ============================================================
 * 全局对象
 * ============================================================ */
swing_angle_t g_swing_angle;

/* ============================================================
 * 解算有效性判定（2026-11-10 新增，REQ-017.1 前提）
 * ------------------------------------------------------------
 * 为什么需要它：
 *   过零停车/限位判定的前提是"角度数字可信"。
 *   若三编码器解算失败（NO_MATCH / AMBIGUOUS / JUMP），
 *   current_angle_x10 可能是任意值，此时判定"过零"会误触发。
 *
 * 判据：mode ∈ {COARSE, INCR} 且 last_err == OK
 *   - UNINIT：上电解算未完成 → 不可信
 *   - LOST  ：增量丢失需重解算 → 不可信
 *   - COARSE：已解算但未切增量，精度够用于限位/零位判断 → 可信
 *
 * ⚠ 本函数不改角度值，只回答"可不可信"。
 *   解算失败时上层应报编码器故障，而不是拿角度去判断位置。
 * ============================================================ */
uint8_t swing_angle_is_solve_valid(void)
{
    abs_angle_state_t st;

    abs_angle_get_state(&st);

    if (st.last_err != ABS_ANGLE_OK) {
        return 0U;
    }
    if ((st.mode != ABS_ANGLE_MODE_COARSE) &&
        (st.mode != ABS_ANGLE_MODE_INCR)) {
        return 0U;
    }
    return 1U;
}

/* ============================================================
 * 过零检测（2026-11-10 新增，REQ-017.1）
 * ------------------------------------------------------------
 * 用**符号翻转**而不是"是否落在 ±0.5° 窗口内"：
 *
 *   窗口判据的问题（卸粮筒转速 2.40 r/min = 14.40 °/s）：
 *     50ms 采样步进 0.72°（额定）/ 0.84°（最快）
 *     零位窗口只有 ±0.5° = 1.0° 总宽
 *     余量仅 0.16° ~ 0.28°，一旦转速偏高或采样抖动就会整段跳过
 *
 *   符号翻转判据的好处：与转速完全无关。
 *     只要两次采样之间穿过零点，符号必然翻转。
 *
 * 边界处理：0 归到正侧（用 >= 0），避免连续两次采样恰好为 0 时漏判。
 * ============================================================ */
static uint8_t swing_crossed_zero(int32_t prev_x10, int32_t now_x10)
{
    return ((prev_x10 >= 0) != (now_x10 >= 0)) ? 1U : 0U;
}

/* ============================================================
 * 内部辅助：编码器值 → 角度（0.1°精度）
 * @note 假设编码器已转换为角度值（0.1°精度）
 *       实际项目中此函数可能需要根据编码器分辨率进行换算
 * ============================================================ */
static int32_t calc_angle_from_encoder(int32_t encoder_raw, int32_t zero_offset)
{
    return encoder_raw - zero_offset;
}

/* ============================================================
 * 内部辅助：判断位置
 * ============================================================ */
static swing_position_t determine_position(swing_angle_t *angle)
{
    int32_t a = angle->current_angle_x10;

    /* 零位判定（优先级最高） */
    if (a >= (angle->zero_center_x10 - angle->zero_tolerance_x10) &&
        a <= (angle->zero_center_x10 + angle->zero_tolerance_x10)) {
        return SWING_POS_ZERO;
    }

    /* 左限位判定 */
    if (a <= (angle->left_limit_x10 + angle->limit_tolerance_x10)) {
        return SWING_POS_LEFT_LIMIT;
    }

    /* 右限位判定 */
    if (a >= (angle->right_limit_x10 - angle->limit_tolerance_x10)) {
        return SWING_POS_RIGHT_LIMIT;
    }

    /* 其他为中间位 */
    return SWING_POS_MIDDLE;
}

/* ============================================================
 * 内部辅助：更新位置标志
 * ============================================================ */
static void update_position_flags(swing_angle_t *angle)
{
    angle->is_at_left_limit = (angle->position == SWING_POS_LEFT_LIMIT);
    angle->is_at_right_limit = (angle->position == SWING_POS_RIGHT_LIMIT);
    angle->is_at_zero = (angle->position == SWING_POS_ZERO);
    angle->is_at_middle = (angle->position == SWING_POS_MIDDLE);
}

/* ============================================================
 * 内部辅助：位置变化处理
 * ============================================================ */
static void handle_position_change(swing_angle_t *angle)
{
    if (angle->last_position == angle->position) {
        return;
    }

    if (angle->on_position_change != NULL) {
        angle->on_position_change(angle->last_position, angle->position);
    }

    angle->last_position = angle->position;
}

/* ============================================================
 * 公开 API
 * ============================================================ */

void swing_angle_init(swing_angle_t *angle,
                     int32_t encoder_zero_offset,
                     swing_encoder_getter_t getter,
                     void *user_data)
{
    SM_ASSERT_PTR(angle);

    /* 限位常量（符合 REQ001-FUN001）
     *
     * 【2026-11-10 调整：极限电子限位在 ±2° 物理容差外再扩 1°】
     *
     * 为什么需要扩这 1°：
     *   控制目标 2150 与限位 2150 完全重合，而两边容差都是 ±2°：
     *     序列完成点 = target - seq_tol  = 2150 - 20 = 2130
     *     限位触发点 = limit  - lim_tol  = 2150 - 20 = 2130
     *   两者数值完全相同 ⇒ 依赖 50ms tick 内事件处理先后，
     *   一旦 seq_tol 收紧到 10(±1°) 就变成"限位先停、序列永不完成"。
     *
     * 现在的分工（用户 2026-11-10 裁定）：
     *   控制目标误差  = ±2.0°  （序列/定位的验收口径，不变）
     *   极限电子限位  = 物理极限外再放 1.0°
     *
     * 扩 1° 后：
     *   限位触发点 = 2160 - 20 = 2140
     *   序列完成点 = 2150 - 20 = 2130
     *   ⇒ 序列先完成，余量 1.0°，不再依赖事件顺序
     *
     * ⚠ 左侧同理：-1000 → -1010，限位触发点 -1010 + 20 = -990；
     *   若序列到 -1000，完成点 -1000 + 20 = -980 > -990，序列先完成 ✓
     *
     * ⚠ 这 1° 是**软件电子限位**的放宽，不是机械行程的放宽。
     *   机械行程（mech_cfg 的 MECH_DEF_OUT_MIN/MAX_X10）保持不变，
     *   它约束的是角度解算的合法域，与限位触发是两件事。
     */
    angle->left_limit_x10 = -1010;       /* -101.0°（原 -100.0° 外扩 1°） */
    angle->right_limit_x10 = 2160;       /* +216.0°（原 +215.0° 外扩 1°） */
    angle->zero_center_x10 = 0;          /* 0.0° */
    angle->limit_tolerance_x10 = 20;     /* ±2°（保持不变） */
    angle->zero_tolerance_x10 = 5;       /* ±0.5° */

    /* 物理量 */
    angle->encoder_zero_offset = encoder_zero_offset;
    angle->current_angle_x10 = 0;

    /* 位置状态 */
    angle->position = SWING_POS_UNKNOWN;
    angle->last_position = SWING_POS_UNKNOWN;
    angle->is_at_left_limit = 0;
    angle->is_at_right_limit = 0;
    angle->is_at_zero = 0;
    angle->is_at_middle = 0;

    /* 依赖注入 */
    angle->get_encoder = getter;
    angle->user_data = user_data;
    angle->on_position_change = NULL;

    angle->is_initialized = 1;
}

void swing_angle_bind_sensor(swing_angle_t *angle,
                            swing_encoder_getter_t getter,
                            void *user_data)
{
    SM_ASSERT_PTR(angle);
    SM_ASSERT_MSG(angle->is_initialized, "Swing angle not initialized");

    angle->get_encoder = getter;
    angle->user_data = user_data;
}

void swing_angle_update(swing_angle_t *angle)
{
    int32_t  encoder_raw;
    int32_t  prev_angle;
    uint8_t  solve_ok;

    SM_ASSERT_PTR(angle);
    SM_ASSERT_MSG(angle->is_initialized, "Swing angle not initialized");
    SM_ASSERT_PTR(angle->get_encoder);

    /* ---- 1. 先判解算有效性（2026-11-10 Q30-a）----
     *
     * 【为什么必须在取值之前判】
     *   解算失败时 current_position 可能是任意值。
     *   若先算角度再判有效，中间会有一瞬间对外暴露错误角度，
     *   过零/限位判定可能因此误触发。
     *
     * 【解算失败的动作】
     *   置 position = UNKNOWN（让上层知道不可信），
     *   投 ENCODER_FAULT 事件（上层据此置 encoder_fault 标志）。
     *   ⚠ 但不阻断手动模式 —— 用户 2026-11-10 裁定"报故障但不影响手动"。
     *     具体保证方式见 swing_trans_table.c 中 IDLE 保留手动跳转的说明。
     */
    solve_ok = swing_angle_is_solve_valid();

    if (solve_ok == 0U) {
        angle->last_position = angle->position;
        angle->position      = SWING_POS_UNKNOWN;
        update_position_flags(angle);

        /* 只在从"可信"跳变到"不可信"时投一次，避免每 50ms 刷屏 */
        if (angle->last_position != SWING_POS_UNKNOWN) {
            LOG_FAULT("[Swing] angle solve INVALID, report encoder fault\n");
            post_event(SWING_EVT_ENCODER_FAULT);
        }

        /* 不更新 current_angle_x10：保留上次可信值供诊断查看，
         * 但 position=UNKNOWN 已表明它不可用于判定。 */
        return;
    }

    /* 读取编码器原始值 */
    encoder_raw = angle->get_encoder(angle->user_data);

    prev_angle = angle->current_angle_x10;

    /* 计算角度 */
    angle->current_angle_x10 = calc_angle_from_encoder(encoder_raw,
                                                        angle->encoder_zero_offset);

    /* ---- 2. 过零检测（REQ-017.1）----
     *
     * 【只在手动移动过程中判定过零】
     *   序列自己的 SWING_MOVE_TO 需要穿越零位到达对侧目标，
     *   若过零就停会让序列无法完成（如从 +150° 回位到 0° 后继续）。
     *   所以用 from_sequence 区分：
     *     from_sequence = 0（手动）→ 过零停车，置 need_retrigger
     *     from_sequence = 1（序列）→ 忽略过零，由序列自身条件判定
     *
     * 用户 2026-11-10 裁定：
     *   「手动模式，跳过执行过零位的操作方式」
     *   ⇒ 手动模式下**不做过零停车**，只在序列模式保留。
     *   故此处实际上对 from_sequence==0 直接跳过。
     */
    if ((angle->need_zero_stop != 0U) &&
        (swing_crossed_zero(prev_angle, angle->current_angle_x10) != 0U)) {
        post_event(SWING_EVT_ZERO_REACHED);
    }

    /* 更新位置 */
    angle->last_position = angle->position;
    angle->position = determine_position(angle);
    update_position_flags(angle);

    /* 处理位置变化（触发回调） */
    handle_position_change(angle);
}

void swing_angle_calibrate(swing_angle_t *angle, int32_t encoder_raw)
{
    SM_ASSERT_PTR(angle);
    SM_ASSERT_MSG(angle->is_initialized, "Swing angle not initialized");

    angle->encoder_zero_offset = encoder_raw;
    swing_angle_update(angle);
}

/* ============================================================
 * 查询接口
 * ============================================================ */
swing_position_t swing_angle_get_position(const swing_angle_t *angle)
{
    SM_ASSERT_PTR(angle);
    SM_ASSERT_MSG(angle->is_initialized, "Swing angle not initialized");
    return angle->position;
}

uint8_t swing_angle_is_left_limit(const swing_angle_t *angle)
{
    SM_ASSERT_PTR(angle);
    return angle->is_at_left_limit;
}

uint8_t swing_angle_is_right_limit(const swing_angle_t *angle)
{
    SM_ASSERT_PTR(angle);
    return angle->is_at_right_limit;
}

uint8_t swing_angle_is_zero(const swing_angle_t *angle)
{
    SM_ASSERT_PTR(angle);
    return angle->is_at_zero;
}

uint8_t swing_angle_is_middle(const swing_angle_t *angle)
{
    SM_ASSERT_PTR(angle);
    return angle->is_at_middle;
}

int32_t swing_angle_get_value(const swing_angle_t *angle)
{
    SM_ASSERT_PTR(angle);
    return angle->current_angle_x10;
}

uint8_t swing_angle_is_safe(const swing_angle_t *angle)
{
    SM_ASSERT_PTR(angle);
    return (angle->position != SWING_POS_UNKNOWN) && angle->is_initialized;
}

swing_angle_t* swing_angle_get(void)
{
    return &g_swing_angle;
}
