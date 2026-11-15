/*=============================================================================
 * foc_mode.h  —  L6 模式层（IDLE / LOCK / IF / CALIB / CURRENT / SPEED / POSITION / PARAM_IDENT）
 *
 * 职责：
 *   - 维护当前模式与 IF 参数（freq / vq / vd）
 *   - 每拍由 L5 在 Foc_Control_Step() 第 4 步调用，产出 FocCmd_t
 *   - IF 模式：角度自增，输出 vd/vq + theta_e，enable=true
 *   - CURRENT 模式：调用 L6.5 Foc_CurrentLoop_Step 产出 vd/vq/theta_e，enable=true
 *   - SPEED 模式：先跑 L7 速度外环（1ms 抽稀），再跑电流环，enable=true
 *   - POSITION 模式：位置环（2ms）-> 速度环（1ms）-> 电流环，enable=true
 *   - PARAM_IDENT 模式：调用 L9 ParamIdent_Step20kHz 产出 vd/vq/theta_e，enable=true
 *   - IDLE/LOCK/CALIB：输出零，enable=false，由 L5 走静态输出分支
 *=============================================================================*/

#ifndef FOC_MODE_H
#define FOC_MODE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    FOC_MODE_IDLE    = 0,   /* 全低电平，上下管全关（安全停机） */
    FOC_MODE_LOCK    = 1,   /* 三相 50% 零矢量，锁死电机 */
    FOC_MODE_IF      = 2,   /* q 轴加压 + 角度自增，开环拖动 */
    FOC_MODE_CALIB   = 3,   /* 仅用作主循环触发校准的标志；输出全低，动作由主循环接管 */
    FOC_MODE_CURRENT = 4,   /* 有感闭环电流环（依赖编码器电角度已对齐） */
    FOC_MODE_SPEED   = 9,   /* 速度外环 + 有感电流环串级 */
    FOC_MODE_POSITION = 10, /* 位置外环 + 速度环 + 电流环三环串级 */
    FOC_MODE_PARAM_IDENT = 11, /* 离线参数辨识（Rs / Ld / Lq），自成状态机 */
} foc_mode_t;

/* L6 每拍产出的控制指令 */
typedef struct {
    float vd;        /* V */
    float vq;        /* V */
    float theta_e;   /* rad，已归一化到 [0, 2) */
    bool  enable;    /* true  = 走 InvPark→SVPWM→SetDuty（IF / CURRENT）
                        false = 走 IDLE/LOCK/CALIB 静态输出（由 L5 按 g_fm_mode 处理） */
} FocCmd_t;

void       Foc_Mode_Init(void);
void       Foc_Mode_SetMode(foc_mode_t m);
foc_mode_t Foc_Mode_GetMode(void);

/* IF 参数设置（不改变当前模式） */
void Foc_Mode_SetOpenloop(float freq_hz, float vq_volts, float vd_volts);

/* 快捷接口 */
void Foc_Mode_StartOpenloop(void);   /* = SetOpenloop 后切到 FOC_MODE_IF，角度清零 */
void Foc_Mode_Stop(void);            /* = 切到 FOC_MODE_IDLE */

/* 每拍调用（在 Foc_Control_Step 第 4 步），ts = 1/20000 */
void Foc_Mode_Step(float ts, FocCmd_t *out);

extern volatile foc_mode_t g_fm_mode;      /* 当前模式，Watch 只读 */
extern volatile float      g_fm_theta_e;   /* IF 角度 (rad) */
extern volatile float      g_fm_freq_hz;   /* IF 电频率 (Hz) */
extern volatile float      g_fm_vq;        /* IF q 轴电压 (V) */
extern volatile float      g_fm_vd;        /* IF d 轴电压 (V) */
extern volatile uint32_t   g_fm_step_cnt;  /* 步进计数 */

#endif /* FOC_MODE_H */
