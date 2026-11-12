/*=============================================================================
 * param_ident.h  —  L9 离线参数辨识层（Rs / Ld / Lq，mode 11）
 *
 * 定位：
 *   - 电机静止时运行，Lq 段要求外部机械锁轴
 *   - 编码器电角度必须已对齐，电流零偏必须已校准
 *   - 结果只写 RAM 全局对象，不自动回填任何控制参数
 *
 * 时序适配（方案A）：
 *   本工程 TMR4 三角波在谷底触发 ADC，OCCR 影子装载点在峰值，中断里写的
 *   电压只作用于本周期后半段。激励极性每 PI_POL_HOLD_INIT 拍翻转一次，
 *   只在极性一致的第二拍取样，保证配对区间内实际电压恒定。
 *
 * 电压基准：
 *   SVPWM 内部固定用 CFG_SAMPLING_SIM_VBUS_V 归一化，g_pi_vbus_v 为实际
 *   母线电压；拟合时按 g_pi_vbus_v / CFG_SAMPLING_SIM_VBUS_V 还原实际电压。
 *=============================================================================*/

#ifndef PARAM_IDENT_H
#define PARAM_IDENT_H

#include <stdbool.h>
#include <stdint.h>

#include "cfg_sampling.h"

/*=============================================================================
 * 一、辨识参数初值（改这里，编译后成为变量初值）
 *=============================================================================*/

#define PI_CTRL_HZ                 (20000u)
#define PI_DT_S                    (1.0f / (float)PI_CTRL_HZ)

/* Rs 辨识：三个等间隔 d 轴电流点，A */
#define PI_RS_POINT_NUM            (3u)
#define PI_RS_CURRENT_1            (0.3f)
#define PI_RS_CURRENT_2            (0.6f)
#define PI_RS_CURRENT_3            (0.9f)

/* 电流给定每拍变化量，A/拍 */
#define PI_CURRENT_RAMP_STEP       (0.01f)

/* 初始化零电压等待：2000 拍 = 100ms */
#define PI_INIT_COUNT              (2000u)

/* 每个 Rs 电流点稳定 100ms、采样 50ms */
#define PI_RS_SETTLE_COUNT         (2000u)
#define PI_RS_SAMPLE_COUNT         (1000u)

/* 电感辨识中心电压平均窗口：50ms（取 PREPARE 后半段） */
#define PI_CENTER_SAMPLE_COUNT     (1000u)

/* Ld：0.3A 偏置，d 轴 ±0.4V 激励，512 个有效样本 */
#define PI_LD_BIAS_CURRENT_INIT    (0.3f)
#define PI_LD_EXCITE_V_INIT        (0.4f)
#define PI_LD_SAMPLE_COUNT         (512u)

/* Lq：0A 偏置，q 轴 ±0.4V 激励，512 个有效样本 */
#define PI_LQ_BIAS_CURRENT_INIT    (0.0f)
#define PI_LQ_EXCITE_V_INIT        (0.4f)
#define PI_LQ_SAMPLE_COUNT         (512u)

/* 激励极性保持拍数：本板取 2（见文件头时序说明），2~10 可在 Watch 调 */
#define PI_POL_HOLD_INIT           (2u)

/* 辨识期间 d/q 电流环 PI 初值（与参考工程一致） */
#define PI_CUR_KP_INIT             (0.3f)    /* V/A */
#define PI_CUR_KI_INIT             (1000.0f) /* V/A/s */
#define PI_CUR_VMAX_INIT           (3.5f)    /* V，须大于中心电压+激励幅值 */
#define PI_CUR_ITERM_INIT          (3.5f)    /* V，积分项限幅 */

/* 实际母线电压，用于把下发伏特还原成实际施加电压 */
#define PI_VBUS_INIT               (CFG_SAMPLING_SIM_VBUS_V)

/*=============================================================================
 * 二、状态 / 错误枚举
 *=============================================================================*/
typedef enum {
    PI_STATUS_IDLE = 0,
    PI_STATUS_RUNNING,
    PI_STATUS_SUCCESS,
    PI_STATUS_FAILED,
    PI_STATUS_ABORTED
} pi_status_t;

typedef enum {
    PI_STATE_IDLE = 0,
    PI_STATE_INIT,
    PI_STATE_RS_RAMP,
    PI_STATE_RS_SETTLE,
    PI_STATE_RS_SAMPLE,
    PI_STATE_RS_CALC,
    PI_STATE_LD_PREPARE,
    PI_STATE_LD_EXCITE,
    PI_STATE_LD_CALC,
    PI_STATE_LQ_PREPARE,
    PI_STATE_LQ_EXCITE,
    PI_STATE_LQ_CALC,
    PI_STATE_FINISH
} pi_state_t;

typedef enum {
    PI_ERROR_NONE = 0,
    PI_ERROR_NOT_STOP,
    PI_ERROR_FAULT_ACTIVE,
    PI_ERROR_CALCULATION,
    PI_ERROR_INTERRUPTED,
    PI_ERROR_ABORTED_BY_FAULT
} pi_error_t;

/*=============================================================================
 * 三、结果对象（Watch 只看 g_param_ident 即可）
 *=============================================================================*/
typedef struct {
    uint8_t  status;      /* pi_status_t */
    uint8_t  state;       /* pi_state_t */
    uint8_t  progress;    /* 0~100 */
    uint8_t  error;       /* pi_error_t */
    float    result_Rs;   /* Ω */
    float    result_Ld;   /* H */
    float    result_Lq;   /* H */
    float    voltage_offset;    /* V，Rs 拟合截距 */
    float    ld_voltage_offset; /* V，Ld 拟合截距 */
    float    lq_voltage_offset; /* V，Lq 拟合截距 */
    uint16_t sample_count;      /* 当前阶段有效样本数 */
    float    id_avg[PI_RS_POINT_NUM];  /* A */
    float    ud_avg[PI_RS_POINT_NUM];  /* V，实际电压 */
} PARAM_IDENT_RESULT;

extern volatile PARAM_IDENT_RESULT g_param_ident;

/*=============================================================================
 * 四、可调参数变量（Watch 可改，Start 时生效）
 *=============================================================================*/
extern volatile float    g_pi_cur_kp;
extern volatile float    g_pi_cur_ki;
extern volatile float    g_pi_cur_vmax_v;
extern volatile float    g_pi_cur_iterm_v;
extern volatile float    g_pi_vbus_v;
extern volatile float    g_pi_ld_bias_a;
extern volatile float    g_pi_lq_bias_a;
extern volatile float    g_pi_ld_excite_v;
extern volatile float    g_pi_lq_excite_v;
extern volatile uint16_t g_pi_pol_hold;

/*=============================================================================
 * 五、运行观测量（Watch / 断点）
 *=============================================================================*/
extern volatile uint32_t g_pi_tick_cnt;
extern volatile uint8_t  g_pi_rs_index;
extern volatile uint8_t  g_pi_pol;
extern volatile float    g_pi_id_ref;
extern volatile float    g_pi_iq_ref;
extern volatile float    g_pi_center_v;     /* V，实际电压 */
extern volatile float    g_pi_vlast_cmd;    /* V，上一拍轴电压指令 */
extern volatile float    g_pi_axis_current; /* A，被激励轴当前电流 */
extern volatile float    g_pi_axis_slope;   /* A/s，最近一个有效样本的 di/dt */
extern volatile float    g_pi_start_theta;  /* rad，启动时电角度 */
extern volatile float    g_pi_end_theta;    /* rad，结束时电角度 */
extern volatile int32_t  g_pi_start_cnt;    /* 启动时编码器计数 */
extern volatile int32_t  g_pi_end_cnt;      /* 结束时编码器计数 */

/*=============================================================================
 * 六、API
 *=============================================================================*/
void ParamIdent_Init(void);
bool ParamIdent_Start(void);
void ParamIdent_Abort(void);
void ParamIdent_Reset(void);
bool ParamIdent_IsRunning(void);
void ParamIdent_Step20kHz(float *vd, float *vq, float *theta_e, bool *enable);

#endif /* PARAM_IDENT_H */
