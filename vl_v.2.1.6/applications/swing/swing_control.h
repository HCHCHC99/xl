/**
 * @file    swing_control.h
 * @brief   回转控制对象
 */

#ifndef __SWING_CONTROL_H
#define __SWING_CONTROL_H

#include <stdint.h>
#include "../framework/core/control_object.h"
#include "../framework/manager/ctrl_manager.h"
#include "swing_angle.h"
#include "hw_objects.h"
#include "../motor/motor.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 状态枚举
 * ============================================================ */
typedef enum {
    H_STATE_IDLE = 0,           /* 空闲 */
    H_STATE_UNLOCKING,          /* 刹车盘解锁中 */
    H_STATE_LEFT_RUNNING,       /* 左转中 */
    H_STATE_RIGHT_RUNNING,      /* 右转中 */
    H_STATE_LOCKING,            /* 刹车盘锁止中 */
    H_STATE_FAULT,              /* 故障 */
    H_STATE_MAX
} swing_state_t;

/* ============================================================
 * 超限拦截方向掩码
 * ============================================================ */
#define SWING_FORBID_LEFT    (1 << 0)
#define SWING_FORBID_RIGHT   (1 << 1)

/* ============================================================
 * 逻辑方向（2026-11-09 新增）
 * ------------------------------------------------------------
 * 【为什么不直接用"左/右"】
 *   序列里的 SWING_MOVE_TO 是按**角度目标**决策的，
 *   它需要表达的是"增大角度"还是"减小角度"，而不是"左"或"右"。
 *   手动按键才是"左/右"语义。
 *
 *   二者在**正逻辑**下等价（左=减小、右=增大），
 *   但引入极性配置后可能相反。所以内部统一用"角度增减意图"，
 *   最后由 swing_dir_to_motor() 按极性翻译成电机转向。
 *
 * 【取值与正常逻辑的对应】
 *   SWING_DIR_DECREASE(1) 意图减小角度 → 正逻辑下 = 左转 = MOTOR_DIR_CW
 *   SWING_DIR_INCREASE(2) 意图增大角度 → 正逻辑下 = 右转 = MOTOR_DIR_CCW
 *
 *   ⚠ 1/2 的数值刻意与历史代码的 dir==1(左)/dir==2(右) 一致，
 *     这样所有既有调用点传入的 1/2 语义不变，无需改动。
 * ============================================================ */
#define SWING_DIR_DECREASE   1      /* 意图减小角度（正逻辑=左转） */
#define SWING_DIR_INCREASE   2      /* 意图增大角度（正逻辑=右转） */

/* ============================================================
 * 方向极性（配置项 CFG_ID_SWING_DIR_POLARITY 的运行期镜像）
 * ------------------------------------------------------------
 *   0 = 正逻辑：左转→减小角度，右转→增大角度（默认）
 *   1 = 负逻辑：左转→增大角度，右转→减小角度
 * 由 swing_init() 从 sys_config 装载。
 * ============================================================ */
extern uint8_t g_swing_dir_polarity;

/* ============================================================
 * 上下文结构
 * ============================================================ */
typedef struct {
    /* ----- 硬件依赖 ----- */
    motor_t      *motor_ptr;
    swing_angle_t *angle_ptr;

    /* ----- 手动开关状态 ----- */
    uint8_t left_sw_pressed;
    uint8_t right_sw_pressed;

    /* ----- 零位停止重触发 ----- */
    uint8_t need_retrigger;     /* 到达零位后需重新触发 */
    uint8_t prev_left_sw;       /* 上次的左键状态（上升沿检测） */
    uint8_t prev_right_sw;      /* 上次的右键状态 */

    /* ----- 超限拦截 ----- */
    uint8_t forbidden_dirs;     /* 禁止方向掩码 */

    /* ----- 故障状态 ----- */
    uint8_t encoder_fault;

    /* ----- 序列控制（阶段4使用） ----- */
    uint8_t from_sequence;

    /* ----- 刹车盘：解锁目标方向（1=左, 2=右） ----- */
    uint8_t pending_dir;
} swing_context_t;

/* ============================================================
 * 回转控制对象
 * ============================================================ */
typedef struct {
    control_object_t base;
    StateMachine_t   sm_instance;
    swing_context_t   ctx;
} swing_control_t;

/* ============================================================
 * 外部引用
 * ============================================================ */
extern swing_control_t g_swing_obj;

/* ============================================================
 * API
 * ============================================================ */

int swing_control_register(ctrl_manager_t *mgr);

void swing_control_update_switch(uint8_t left, uint8_t right);

swing_context_t* swing_control_get_context(void);

swing_state_t swing_control_get_state(void);

/**
 * @brief  把"逻辑方向"翻译成电机转向（唯一极性生效点）
 *
 * @param  logical_dir  SWING_DIR_DECREASE(1) / SWING_DIR_INCREASE(2)
 * @retval MOTOR_DIR_CW(1) / MOTOR_DIR_CCW(2)
 *
 * @note   调用方（手动状态机 / 序列动作）仍然传 1=左 2=右 的原有语义，
 *         极性翻译集中在本函数内做，改动面最小。
 *
 *         ⚠ 本函数内含**两个独立硬件约定**的合成：
 *            约定A：左转 ↔ MOTOR_DIR_CW          （电机相序接线）
 *            约定B：MOTOR_DIR_CW 时 encoder_raw 如何变化（编码器安装）
 *         一个极性配置只能覆盖二者的**相对关系**，不能分别指定。
 *         若将来需要独立配置，须拆成两个配置项。
 */
uint8_t swing_dir_to_motor(uint8_t logical_dir);

#ifdef __cplusplus
}
#endif

#endif /* __SWING_CONTROL_H */
