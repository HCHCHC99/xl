/*=============================================================================
 * cfg_foc_table.h  —  FOC 运行配置表（纯宏，编译期选定）
 *
 * 用法：
 *   1. 新增配置：复制一段 FOC_CFG_n_* 改成 FOC_CFG_m_*，改值即可
 *   2. 选择配置：只改 FOC_CFG_SELECT 一个数字
 *   3. 代码里用 FOC_CFG_PWM_FREQ 等宏取"当前选中"的值，不直接引用 FOC_CFG_n_*
 *
 * 列：名称 / PWM频率 / 输出模式 / 采样点 / 死区ns / 高有效
 *=============================================================================*/

#ifndef CFG_FOC_TABLE_H
#define CFG_FOC_TABLE_H

#include "hw_tmr4_pwm.h"   /* tmr4_output_type_t 枚举 */

/*---------------------------------------------------------------------------
 * 采样点枚举（语义：采哪一桥臂导通区）
 *   UPPER：上桥臂导通区中点 → 三角波峰值（CNT=period-1），compare = period-1
 *   LOWER：下桥臂导通区中点 → 三角波谷底（CNT=0），compare = 0
 *---------------------------------------------------------------------------*/
typedef enum {
    CFG_SAMPLE_UPPER = 0,   /* 峰值，上桥臂采样 */
    CFG_SAMPLE_LOWER = 1,   /* 谷底，下桥臂采样 */
} cfg_sample_point_t;

/*=============================================================================
 * 配置表 —— 一行一个配置，段内按列对齐
 *
 *  
 *  编号 名称                   频率    输出模式                     采样点             死区ns   高有效
 *  
 *   0   20k_SYNC_UPPER        20000   TMR4_OUTPUT_SYNC            CFG_SAMPLE_UPPER  0        true 
 *   1   20k_SYNC_LOWER        20000   TMR4_OUTPUT_SYNC            CFG_SAMPLE_LOWER  0        true 
 *   2   20k_COMP_LOWER_DT500  20000   TMR4_OUTPUT_COMPLEMENTARY   CFG_SAMPLE_LOWER  500      true 
 *   3   10k_SYNC_LOWER        10000   TMR4_OUTPUT_SYNC            CFG_SAMPLE_LOWER  0        true 
 *   4   20k_COMP_LOWER_DT500  20000   TMR4_OUTPUT_COMPLEMENTARY   CFG_SAMPLE_LOWER  500      true 
 *   5   20k_COMP_UPPER_DT500  20000   TMR4_OUTPUT_COMPLEMENTARY   CFG_SAMPLE_UPPER  500      true 
 *  
 *  下方 #define 与上表逐行对应，新增配置时两处都加
 *  注意：配置 2 与配置 4 当前参数相同（历史遗留），如非必要勿同时使用；
 *        配置 2 为当前 FOC_CFG_SELECT 选中的默认配置。
 *=============================================================================*/

/* ---- 配置 0：20kHz 同相 上桥臂采样 ---- */
#define FOC_CFG_0_NAME          "20k_SYNC_UPPER"
#define FOC_CFG_0_PWM_FREQ      20000u
#define FOC_CFG_0_OUTPUT_TYPE   TMR4_OUTPUT_SYNC
#define FOC_CFG_0_SAMPLE_POINT  CFG_SAMPLE_UPPER
#define FOC_CFG_0_DEADTIME_NS   0u
#define FOC_CFG_0_ACTIVE_HIGH   true

/* ---- 配置 1：20kHz 同相 下桥臂采样 ---- */
#define FOC_CFG_1_NAME          "20k_SYNC_LOWER"
#define FOC_CFG_1_PWM_FREQ      20000u
#define FOC_CFG_1_OUTPUT_TYPE   TMR4_OUTPUT_SYNC
#define FOC_CFG_1_SAMPLE_POINT  CFG_SAMPLE_LOWER
#define FOC_CFG_1_DEADTIME_NS   0u
#define FOC_CFG_1_ACTIVE_HIGH   true

/* ---- 配置 2：20kHz 互补 下桥臂采样 死区 500ns（当前默认选中）---- */
#define FOC_CFG_2_NAME          "20k_COMP_LOWER_DT500"
#define FOC_CFG_2_PWM_FREQ      20000u
#define FOC_CFG_2_OUTPUT_TYPE   TMR4_OUTPUT_COMPLEMENTARY
#define FOC_CFG_2_SAMPLE_POINT  CFG_SAMPLE_LOWER
#define FOC_CFG_2_DEADTIME_NS   50u
#define FOC_CFG_2_ACTIVE_HIGH   true

/* ---- 配置 3：10kHz 同相 下桥臂采样 ---- */
#define FOC_CFG_3_NAME          "10k_SYNC_LOWER"
#define FOC_CFG_3_PWM_FREQ      10000u
#define FOC_CFG_3_OUTPUT_TYPE   TMR4_OUTPUT_SYNC
#define FOC_CFG_3_SAMPLE_POINT  CFG_SAMPLE_LOWER
#define FOC_CFG_3_DEADTIME_NS   0u
#define FOC_CFG_3_ACTIVE_HIGH   true

/* ---- 配置 4：20kHz 互补 下桥臂采样 死区 500ns（与配置 2 参数相同，历史遗留）---- */
#define FOC_CFG_4_NAME          "20k_COMP_LOWER_DT500"
#define FOC_CFG_4_PWM_FREQ      20000u
#define FOC_CFG_4_OUTPUT_TYPE   TMR4_OUTPUT_COMPLEMENTARY
#define FOC_CFG_4_SAMPLE_POINT  CFG_SAMPLE_LOWER
#define FOC_CFG_4_DEADTIME_NS   500u
#define FOC_CFG_4_ACTIVE_HIGH   true

/* ---- 配置 5：20kHz 互补 上桥臂采样 死区 500ns ---- */
#define FOC_CFG_5_NAME          "20k_COMP_UPPER_DT500"
#define FOC_CFG_5_PWM_FREQ      20000u
#define FOC_CFG_5_OUTPUT_TYPE   TMR4_OUTPUT_COMPLEMENTARY
#define FOC_CFG_5_SAMPLE_POINT  CFG_SAMPLE_UPPER
#define FOC_CFG_5_DEADTIME_NS   500u
#define FOC_CFG_5_ACTIVE_HIGH   true
/*=============================================================================
 * 选择当前生效的配置：只改这一个数字
 *=============================================================================*/
#define FOC_CFG_SELECT   2

/*=============================================================================
 * 展开逻辑（一般不需改动）
 *=============================================================================*/
#define FOC_CFG_CAT_(a, b)    a##b
#define FOC_CFG_CAT(a, b)     FOC_CFG_CAT_(a, b)
#define FOC_CFG_PICK_(n, s)   FOC_CFG_CAT(FOC_CFG_, FOC_CFG_CAT(n, s))
#define FOC_CFG_PICK(s)       FOC_CFG_PICK_(FOC_CFG_SELECT, s)

#define FOC_CFG_NAME          FOC_CFG_PICK(_NAME)
#define FOC_CFG_PWM_FREQ      FOC_CFG_PICK(_PWM_FREQ)
#define FOC_CFG_OUTPUT_TYPE   FOC_CFG_PICK(_OUTPUT_TYPE)
#define FOC_CFG_SAMPLE_POINT  FOC_CFG_PICK(_SAMPLE_POINT)
#define FOC_CFG_DEADTIME_NS   FOC_CFG_PICK(_DEADTIME_NS)
#define FOC_CFG_ACTIVE_HIGH   FOC_CFG_PICK(_ACTIVE_HIGH)

#endif /* CFG_FOC_TABLE_H */
