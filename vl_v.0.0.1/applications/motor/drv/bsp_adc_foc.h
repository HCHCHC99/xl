/**
 * @file    bsp_adc_foc.h
 * @brief   FOC 电流采样（ADC1 三路 + DMA1 CH2 循环搬运）
 *
 * ============================================================
 * 【为什么新建本模块，而不复用 bsp_adc.c / drv_adc.c】
 * ------------------------------------------------------------
 * drv_adc.c 的 rt_adc_read() 走的是「ADC_Start + do{查EOC}while」
 * 忙等轮询（drv_adc.c:161-195）。用它采 FOC 电流有三个硬伤：
 *   1) 每次取值都要软件重启动，触发时刻抖动数微秒，无法与 PWM 对齐
 *   2) 一次只能读一个通道，V/W 两相之间隔 1~2 us，转子已转过角度，
 *      iu = -iv - iw 的重构随之引入相位误差
 *   3) ISR 内不应穿过 RT-Thread 设备框架（查找/加锁）
 * 因此电流通路必须走裸 hc32_ll + DMA，与设备框架解耦。
 *
 * bsp_adc.c 是另一个东西（hw_adc_t 抽象层的 TODO 桩，A-18），
 * 动它影响面大且无必要，故本模块独立。
 *
 * ============================================================
 * 【硬件事实（2026-10-29 用户实测确认）】
 * ------------------------------------------------------------
 *   PA3  ADC1_IN3   CURRENT_TOTAL   母线总电流
 *   PA4  ADC12_IN4  CURRENT_W       W 相（上桥臂 MOS 的 D 端）
 *   PA5  ADC12_IN5  CURRENT_V       V 相（上桥臂 MOS 的 D 端）
 *   PA6  ADC12_IN6  AD_VM           电源电压（sys_voltage 占用，同引脚不同 ADC）
 *   PA7  ADC12_IN7  举升角传感器（lift_adc 占用）
 *
 * 三路电流全在 ADC1 ⇒ 单个扫描序列 + 单路 DMA 一次搬完，
 * 不需要 ADC1/ADC2 同步模式。这是引脚分布的天然优势。
 *
 * ============================================================
 * 【采样点拓扑：上桥臂 MOS 的 D 端 = 高边采样】
 * ------------------------------------------------------------
 * 用户明确：V/W 相采集点在**上桥臂 MOS 的 D 端**。
 *
 *   上桥导通（PWM 低谷 CNT≈0）：分流电阻上有完整相电流 → 可采
 *   上桥关断（PWM 峰顶 CNT≈period）：采不到
 *
 * ★ 这与 docs/02-设计/方案-FOC驱动与三控制模式-V2.0.md §9.4 的
 *   描述**方向相反**（该节按低边采样写，采样窗口在峰顶）。
 *   以本文件为准，§9.4 待后续更正。
 *
 * ★ 方案 B（当前实现）用软件触发，触发时刻与 PWM 无关，
 *   采到的是**随机相位**的瞬时值，含开关噪声尖峰。
 *   用途仅限：验证通道映射、验证 26.4 mV/A 标定、量级是否合理。
 *   **不可用于 FOC 闭环**。方案 A 将改为 TMR4_3 SCMP 触发对齐低谷。
 *
 * ============================================================
 * 【标定公式（用户给定）】
 * ------------------------------------------------------------
 *   VOUT(mV) = 1650 + IP(A) × 26.4
 *   ⇒ IP(A) = (VOUT_mV - 1650) / 26.4
 *
 *   12bit / 3.3V ⇒ 1 LSB = 3300/4096 = 0.80566 mV
 *   灵敏度     = 26.4 / 0.80566 = 32.768 LSB/A
 *   满量程     = (3300-1650)/26.4 = ±62.5 A
 *   零点偏移   = 1650 / 0.80566 = 2048 LSB（正好半量程，便于双向测量）
 *
 * 与既有 bsp_dev_adc.c 完全一致：
 *   ADC_DEV_0_CURRENT_VOLTAGE 1650 ✓
 *   ADC_DEV_mV_mA             0.026400 ✓
 *   ADC_DEV_CURRENT_RANGE     50（偏保守，实际可到 62.5A）
 * ============================================================
 */

#ifndef __BSP_ADC_FOC_H__
#define __BSP_ADC_FOC_H__

#include <rtthread.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 一、通道映射
 * ============================================================ */
#define FOC_ADC_UNIT                CM_ADC1

#define FOC_ADC_CH_IBUS             (3U)    /* PA3 CURRENT_TOTAL */
#define FOC_ADC_CH_IW               (4U)    /* PA4 CURRENT_W     */
#define FOC_ADC_CH_IV               (5U)    /* PA5 CURRENT_V     */

/* 扫描序列的通道顺序。这是 DMA 搬运后数组下标的唯一依据，
 * 改动必须同步 foc_iabc_t 的字段顺序和 DMA 长度。 */
#define FOC_ADC_SEQ_LEN             (3U)

/* ============================================================
 * 二、标定常量（整数运算，禁用浮点的前提下也能用）
 * ============================================================ */
#define FOC_ADC_VREF_MV             (3300U)
#define FOC_ADC_FULL_SCALE          (4096U)
#define FOC_ADC_RAW_MAX             (4095U)

#define FOC_CUR_ZERO_MV             (1650U)     /* 零电流输出 */
#define FOC_CUR_MV_PER_A_X1000      (26400U)    /* 26.4 mV/A × 1000 */

/* 零点对应的 ADC 原始码 = 1650*4096/3300 = 2048（正好半量程） */
#define FOC_CUR_ZERO_RAW            ((uint16_t)(((uint32_t)FOC_CUR_ZERO_MV * \
                                                  FOC_ADC_FULL_SCALE) / FOC_ADC_VREF_MV))

/* 采样时间：上桥臂 D 端经采样电阻+运放，属低阻源，
 * 但 PWM 开关噪声大，取 40 ADCLK 让 S/H 充分建立（≈0.4us@100MHz） */
#define FOC_ADC_SAMPLE_TIME_CLK     (40U)

/* ============================================================
 * 电流零点校准总开关（2026-11-09）
 * ------------------------------------------------------------
 *   0 = 不启用：零点固定用 FOC_CUR_ZERO_RAW(2048)，
 *               行为与未加校准功能前**完全一致**
 *   1 = 启用  ：零点用 s_zero_raw[ch]（校准值，未校准则回退 2048）
 *
 * 【为什么现在置 0】
 *   用户裁定：校准功能"先设计、不启用"，
 *   等后续完成相应模块后再打开。
 *   此刻打开会改变已在跑的电流判据（堵转/过流阈值），
 *   而阈值本身还没整定完，两个变量同时变会分不清影响。
 *
 * 【启用前必须确认】
 *   ① t_iadc_cal confirm 已成功执行
 *   ② t_iadc_cal 显示 CALIBRATED 且零点值合理
 *   ③ 堵转阈值 stall_* 已按新零点重新整定
 *
 * 【启用方式】
 *   改本宏为 1U，重新编译即可。无需改其他代码。
 * ============================================================ */
#define FOC_ADC_ZERO_CAL_ENABLE     (0U)

/* ============================================================
 * 上电自动零点校准（2026-11-09）
 * ------------------------------------------------------------
 *   0 = 不启用（当前）：上电只"装载"已存的零点，不重新采集
 *   1 = 启用：上电在 foc_adc_init 内自动采集一次并覆盖零点
 *
 * 【为什么现在置 0】
 *   用户要求："在上电过程中，执行动作前，回转角控制的电机未动作前，
 *   完成校准过程。这个功能先设计，但不启用。"
 *
 * 【启用位置与前置条件（启用时必须一并做）】
 *   位置：必须在 foc_adc_init 末尾、s_inited=1 之前
 *         （此时 DMA 已跑、ADC 在采，且**尚无任何电机动作**）。
 *         启动表顺序天然满足：
 *           foc_adc 第 9 步 → swing 第 13 步，
 *           回转电机最早可在 swing_init 之后才被驱动。
 *
 *   前置条件（三者缺一不可，启用时需在代码中逐项确认）：
 *     ① **电机未通电**：上电时 PWM 必须处于 EMB 封锁状态
 *        （bsp_pwm 的 SOE3=1 已由 tim4_init 末尾保证）
 *     ② **无外部电流**：母线不应有负载电流
 *     ③ ADC 已稳定：至少采到 64 个样本（本模块内部判据）
 *
 *   ⚠️ 若上述条件不满足，自动校准会**把错误值写进零点**，
 *      且因为落盘而持久化，后果比不校准更糟。
 *      这也是当前置 0 的主要原因 —— 需要先验证上述条件。
 * ============================================================ */
#define FOC_ADC_AUTO_CAL_ON_BOOT    (0U)

/* 通道索引（用于零点数组与 raw_to_a10_chn） */
#define FOC_ADC_CHN_BUS             (0U)
#define FOC_ADC_CHN_W               (1U)
#define FOC_ADC_CHN_V               (2U)

/* ============================================================
 * 三、数据结构
 * ============================================================ */

/* DMA 搬运目标缓冲区。volatile：ISR 与任务共享。
 * 对齐到 4 字节，DMA 16bit 传输不做此要求，但避免编译器把
 * 相邻变量塞进来导致越界写。 */
typedef struct
{
    volatile uint16_t raw[FOC_ADC_SEQ_LEN];   /* [0]=ibus [1]=iw [2]=iv */
} foc_adc_dma_buf_t;

/* 一次电流快照，已换算为物理单位 */
typedef struct
{
    int16_t  i_bus_a10;      /* 母线电流，0.1A 单位（±625） */
    int16_t  i_w_a10;        /* W 相，0.1A */
    int16_t  i_v_a10;        /* V 相，0.1A */
    int16_t  i_u_a10;        /* U 相 = -(iv+iw)，0.1A，重构值 */
    uint16_t raw_bus;        /* 原始码，排障用 */
    uint16_t raw_w;
    uint16_t raw_v;
    uint32_t seq;            /* 采样序号，用于判断是否刷新 */
} foc_iabc_t;

/* 运行统计，用于诊断命令 */
typedef struct
{
    uint32_t conv_cnt;       /* DMA 块完成次数 */
    uint32_t err_cnt;        /* 传输错误次数 */
    uint32_t sat_cnt;        /* 饱和（贴 0/4095）次数 */
} foc_adc_stat_t;

/* ============================================================
 * 四、API
 * ============================================================ */

/**
 * @brief  初始化 ADC1 三路电流采样 + DMA1 CH2 循环搬运
 * @retval 0 成功；<0 失败
 * @note   必须在 drv_adc.c 的 rt_hw_adc_init()（INIT_DEVICE_EXPORT）
 *         之后调用，否则会被其 ADC_Init() 覆盖配置。
 *         启动表中位于 BOOT_LV_HW 阶段，天然满足。
 */
int foc_adc_init(void);

/**
 * @brief  停掉采样（关 DMA 通道 + 停 ADC）
 */
void foc_adc_deinit(void);

/**
 * @brief  触发一次三路电流采样（供 PWM 谷值 ISR 调用）
 *
 * @note   【2026-11-04 · Q1 方案 1 新增】
 *
 *         调用点：bsp_pwm.c 的 bsp_pwm_tmr4_isr()，即 TMR4_3 三角波
 *         谷值中断（INT098，20kHz）。
 *
 *         为什么必须由 PWM 中断驱动：
 *           三路电流采的是**高边**（上桥臂 MOS 的 D 端）。只有上桥导通
 *           时才能读到真实电流；三角波谷值（CNT≈0）正是七段式 SVPWM
 *           零矢量 111 段的中点，三相上桥同时导通，三路都有效。
 *           软件随意触发会采到上桥关断的盲区，读数为 0。
 *
 *         内部只做一次 ADC_Start()（写 STR.STRT），不重置通道、
 *         不改采样时间 —— 那些在 foc_adc_init() 里配好了。
 *         未初始化时安全返回，不写寄存器。
 *
 *         为什么不用硬件触发（TRGENA + AOS_SCMP）：
 *           AOS 只能接 TMR4_3 的 SCMP 特殊比较事件，而本工程从未配置
 *           SCCRVx/SCSR/SCMR（全是复位值 0），该事件源压根不产生事件。
 *           详见 bsp_adc_foc.c 文件头的根因分析。
 */
void foc_adc_trigger(void);

/**
 * @brief  取最近一次电流快照
 * @param  out 输出，不可为 NULL
 * @retval 0 成功；<0 未初始化
 * @note   直接读 DMA 缓冲区并换算，不阻塞、不加锁。
 *         读到的 raw 可能来自不同的转换轮次（三轮之间约 1us），
 *         方案 B 阶段可接受；方案 A 会用 DMA 双缓冲+完成中断消除。
 */
int foc_adc_read(foc_iabc_t *out);

/**
 * @brief  取原始码（不换算），排障用
 */
int foc_adc_read_raw(uint16_t raw[FOC_ADC_SEQ_LEN]);

/**
 * @brief  取运行统计
 */
void foc_adc_get_stat(foc_adc_stat_t *st);

/**
 * @brief  取母线电流绝对值（供过流保护使用）
 * @retval 母线电流，0.1A 单位，恒 >= 0
 * @note   【2026-11-04 新增 / R8 修复】
 *         原 motor_sample_current_a10() 是 weak 桩恒返回 0，
 *         导致过流保护永不触发。本接口为它提供真实数据源。
 *
 *         为什么用母线电流而非相电流：
 *           过流保护要看绕组总发热，母线电流在六步换向的 6 个
 *           状态全是有源矢量，全程有效；且不受单相采样盲区影响。
 *
 *         为什么取绝对值：
 *           电流霍尔零点在 2048（半量程），正负都可测。
 *           过流判据只关心幅值，符号会误导"电流变小"的误判
 *           （例如 -55A 转成 uint16 会变成 65481，永远超阈值）。
 *
 *         未初始化或读失败时返回 0（安全侧：不误报过流）。
 */
/**
 * @brief  取母线电流（**无符号**，0.1A）
 * @retval 母线电流，0.1A，恒 >= 0；未初始化或首帧未到时返回 0
 * @note   【2026-11-09 类型裁定：母线电流天然无符号】
 *
 *         母线电流物理上不可能反向（直流母线经前级整流 + 母线电容，
 *         本机为六步方波驱动、无能量回馈通路），
 *         因此用 uint16 表达其物理属性更准确。
 *
 *         与相电流/线电流的区别（关键，不可统一）：
 *           母线电流   ：单向，**无符号**   ← 本函数
 *           线电流 V/W ：双向，**有符号**   → foc_iabc_t.i_w_a10 / i_v_a10
 *           相电流 IU  ：重构，**有符号**   → foc_iabc_t.i_u_a10
 *
 *         a10 < 0 只可能是零点偏移（未校准）或噪声造成的假象，
 *         本函数钳到 0。这是"符合物理的钳位"，不是"取绝对值"：
 *           取绝对值：把 -5A 变成 +5A（凭空造出电流）
 *           钳位到 0：把 -5A 视为 0A（承认没有正向电流）
 */
uint16_t foc_adc_get_bus_current_a10(void);

/**
 * @brief  取母线电流的正向偏离量（raw - zero），**ISR 安全**
 * @retval 正向偏离量；未初始化或 raw <= zero 时返回 0
 * @note   【2026-11-05 新增 / 2026-11-09 改为无符号单向】
 *
 *         为什么需要它：foc_adc_get_bus_current_a10() 内含一次 int64
 *         除法（约 60~100 周期）。堵转保护在 20kHz 中断里每帧判一次，
 *         若用 a10 版本会显著拉长 ISR。
 *
 *         为什么是单向无符号：母线电流物理上不反向。
 *         raw <= zero 表示"无正向电流"，返回 0；
 *         若按无符号直接减会得到 65500 之类的巨大值，误触发满量程。
 *
 *         换算（由 foc_adc_raw_to_a10 反解，gcd=528 约分后向上取整）：
 *           raw = (2048*a10 + 1280000) / 625
 *           dev = raw - 2048
 *         实测：55A→dev 1803、50A→1639、45A→1475、37A→1213、5A→164
 *
 *         零点取 s_zero_raw[FOC_ADC_CHN_BUS]；
 *         FOC_ADC_ZERO_CAL_ENABLE=0 时恒为 2048（行为同旧版）。
 */
uint16_t foc_adc_get_bus_current_dev_raw(void);

/**
 * @brief  清零统计
 */
void foc_adc_clr_stat(void);

/**
 * @brief  原始码 → 0.1A（整数运算，无浮点）
 * @param  raw ADC 原始码 0~4095
 * @retval 电流，0.1A 单位；超出 int16 范围时钳位
 * @note   等价于 foc_adc_raw_to_a10_chn(raw, 0)，用母线零点。
 *         推导见 foc_adc_raw_to_a10_chn 的注释。
 */
int16_t foc_adc_raw_to_a10(uint16_t raw);

/**
 * @brief  原始码 → 0.1A（指定通道零点）
 * @param  raw  0~4095
 * @param  ch   0=母线 / 1=W / 2=V（FOC_ADC_CHN_xxx）
 * @retval 有符号电流，0.1A
 * @note   【2026-11-09 新增，为零点校准预留】
 *
 *         FOC_ADC_ZERO_CAL_ENABLE=0 时零点恒为 2048，
 *         行为与旧版完全一致（校准功能已实现未启用）。
 *         置 1 后按 ch 取 s_zero_raw[ch]。
 */
int16_t foc_adc_raw_to_a10_chn(uint16_t raw, uint8_t ch);

/* ============================================================
 * 电流零点校准（2026-11-09 新增，当前未启用）
 * ============================================================ */

/**
 * @brief  从 sys_config 装载零点（在 foc_adc_init 内自动调用）
 * @note   读 cur_zero_*_raw 与 cur_zero_valid；
 *         valid=0 或读到的值越界（不在 1024~3072）时回退 2048。
 *         本函数只填 s_zero_raw[]，是否生效由 FOC_ADC_ZERO_CAL_ENABLE 决定。
 */
void foc_adc_load_zero(void);

/**
 * @brief  开始零点校准采集（清零累加器）
 * @note   调用方负责确保**此时母线无电流**（电机未通电）。
 *         采集在 foc_adc_read() 内被动累加，不需要单独任务。
 */
void foc_adc_cal_begin(void);

/**
 * @brief  取校准结果（三路 raw 均值）
 * @param  out  长度 >= 3，[0]=bus [1]=w [2]=v
 * @retval 1 成功；0 无有效采样（未 begin 或样本数不足）
 * @note   只读，不写配置。写配置由调用方用 t_cfg set 或
 *         后续的 foc_adc_cal_commit() 完成。
 */
uint8_t foc_adc_cal_get_result(uint16_t out[3]);

/**
 * @brief  取已采集的样本数（诊断用）
 */
uint32_t foc_adc_cal_get_sample_count(void);

/**
 * @brief  把当前零点提交到 sys_config 并落盘
 * @param  raw3  [0]=bus [1]=w [2]=v，来自 foc_adc_cal_get_result
 * @retval 0 成功；<0 失败（值越界 / 配置写失败）
 * @note   同时置 cur_zero_valid=1。
 *         ⚠️ 本函数会写 Flash。调用方应确认值合理后再调。
 */
int foc_adc_cal_commit(const uint16_t raw3[3]);

/**
 * @brief  取当前生效的三路零点（诊断用）
 * @param  out  长度 >= 3；可为 NULL（只查 valid）
 * @retval 1 已校准（cur_zero_valid=1）；0 未校准（用默认 2048）
 * @note   即使返回 0，out 里仍是有效值（默认 2048）。
 */
uint8_t foc_adc_get_zero(uint16_t out[3]);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_ADC_FOC_H__ */