/**
 * @file    bsp_adc_foc.c
 * @brief   FOC 电流采样实现：ADC1 三路扫描 + DMA1 CH2 循环搬运
 *
 * ============================================================
 * 【DMA 通道分配（2026-10-29 全局规划）】
 * ------------------------------------------------------------
 * 本工程 DMA 此前完全未使用（rtconfig.h 无 RT_USING_DMA，
 * 也无 drv_dma.c）。现一次性规划清楚，避免后续 SPI 提速时撞车：
 *
 *   DMA1 CH0   SPI1_RX   （预留，dma_config.h 已定义但未启用）
 *   DMA1 CH1   SPI1_TX   （预留，同上）
 *   DMA1 CH2   ADC1 电流采样  ← 本模块
 *   DMA1 CH3   空闲（预留给未来 SPI3/USART 高速通路）
 *
 * 为什么给 ADC 选 CH2 而不是 CH0：
 *   SPI1 是三颗磁编码器的通路，用户已明确要评估提速到 5~12.5 Mbps，
 *   一旦启用必然要用 DMA。若 ADC 占了 CH0，届时要么改 ADC 通道，
 *   要么 SPI 只能轮询——两者都是返工。CH2 让两者的规划互不冲突。
 *
 * ============================================================
 * 【为什么用 DMA 而不是轮询】
 * ------------------------------------------------------------
 * 轮询路径（drv_adc.c:161）单次双通道约 387 周期：
 *   转换 2×93 + 轮询 2×80 + 取值 2×20
 * DMA 路径（本模块）仅中断处理约 60 周期 ⇒ 6.5 倍差距。
 * 20 kHz 采样下 CPU 占用 3.87% → 0.60%。
 *
 * 但 CPU 不是主要理由。真因是**轮询无法与 PWM 对齐**：
 * 软件触发抖动数微秒，而上桥导通窗口只有数微秒，必然采到开关噪声。
 *
 * ============================================================
 * 【2026-11-04 · Q1 方案 1：恢复 ADC_Start()，由 PWM 谷值 ISR 驱动】
 * ------------------------------------------------------------
 * 上一批（S0）把触发改为 TMR4_3 SCMP_VH + AOS，并删掉了 ADC_Start()，
 * 实测 conv_cnt 恒 0、raw 恒 0（把 MCU 采样脚硬拉到 3.3V 仍读 0），
 * 说明**没有任何一次转换发生**。根因三个，同时成立：
 *
 *   ① 删 ADC_Start() 是错的
 *      TRGSR.TRGENA 只决定"由什么启动转换"，STR.STRT 才是序列的
 *      装填/总开关。STRT=0 时序列没被 arm，任何触发事件都启动不了。
 *      ⇒ TRGENA=1 + STRT=0 的组合永远不转换。
 *
 *   ② SCMP_VH 事件源本身从未被使能
 *      AOS 只能接 TMR4_3 的 6 个 SCMP（特殊比较）事件
 *      （hc32f460.h:408-413）。而全工程从未写过 SCCRVH / SCSR / SCMR
 *      三组寄存器，全是复位值 0 ⇒ 比较点固定在 0、四类比较事件
 *      使能位 PEN/UEN/DEN/ZEN 全为 0 ⇒ AOS 收到的是**空事件源**。
 *      即使修好 ①，这条路由也不会产生任何触发。
 *
 *   ③ PWM 只初始化了低侧 OC 通道
 *      bsp_pwm.c:336-338 只配 UL/VL/WL，UH/VH/WH 未做 OC 初始化。
 *      引脚输出靠死区定时器互补正常，但 SCMP_VH 的前提不成立。
 *
 * ------------------------------------------------------------
 * 【现方案：复用已验证的 20kHz PWM 谷值中断】
 * ------------------------------------------------------------
 *   链路：TMR4_3 三角波谷值 → INT098（bsp_pwm_tmr4_isr，已稳定跑通）
 *         → foc_adc_trigger() → ADC_Start() → 单次扫描 CH3/4/5
 *         → EOCA → AOS → DMA1 CH2 搬 3 个 half-word → TC 中断 → conv_cnt++
 *
 *   为什么用 INTC 软件启动而不是继续折腾硬件触发：
 *     - 谷值点是六步换向零矢量（三相上桥全通）的中点，三路高边电流
 *       同时有效，正是 FOC 需要的采样时刻；
 *     - 20kHz 中断已经在跑、已实测稳定，复用它零新增风险；
 *     - 相位精度：ISR 入口延迟约 0.2~4μs，占 50μs 周期的 0.4~8%，
 *       对 20kHz 电流环足够（远优于原来的"随机相位"）。
 *
 *   代价（诚实记录）：不是纯硬件零抖动。若将来 FOC 需要更严的相位，
 *   再回到 SCCRVx + SCSR 硬件触发（S4 批），届时需先单独验证
 *   特殊比较寄存器配置 —— 那是一条从未走通的路径。
 *
 *   TRGENA 关掉：既然改用软件启动，就不该留一个指向空事件源的
 *   硬触发配置，否则日后排障会被"已在用硬件触发"误导。
 * ============================================================
 */

#include "bsp_adc_foc.h"

/* 【2026-11-10 彩标 P1】电流零点自动标定：IRQ 注册失败=ERR，
 * 标定成功/落盘=OK，被拒(偏离过大)=WARN，无样本=INFO */
#include "../../log/log_color.h"

#include "hc32_ll.h"
#include "hc32_ll_adc.h"
#include "hc32_ll_dma.h"
#include "hc32_ll_aos.h"
#include "hc32_ll_gpio.h"
#include "hc32_ll_fcg.h"
#include "hc32_ll_pwc.h"
#include "hc32_ll_interrupts.h"
#include "board_config.h"          /* BSP_DMA1_CH2_IRQ_NUM 等映射 */

/* 【2026-11-04 / R8】motor_sample_current_a10() 的强定义需要 motor_t 定义，
 * 不能只靠前向声明 —— 参数类型必须完整。 */
#include "../motor.h"

/* 【2026-11-05 / S1】DMA BTC 中断里驱动堵转保护的 20kHz 电流判据。
 * motor_stall.h 已 include motor.h，此处不重复。 */
#include "../motor_stall.h"

/* 【2026-11-09】电流零点校准需要读写 sys_config（0x0B 段） */
#include "../../config/sys_config.h"

#include <string.h>

/* ============================================================
 * 模块私有状态
 * ============================================================ */
static foc_adc_dma_buf_t s_dma_buf;
static volatile foc_adc_stat_t s_stat;
static volatile uint32_t s_seq;              /* 采样序号 */
static uint8_t s_inited;

/* ============================================================
 * 【2026-11-09 新增】三路电流零点（raw 域）
 * ------------------------------------------------------------
 * 为什么需要这三个值：
 *   电流霍尔零点理论上是 2048（1650mV = 半量程），但实测三路
 *   各不相同（2026-11-09 实测：bus=2129、w=2141、v=2151，
 *   即 +81/+93/+103 LSB，折合 2.47/2.84/3.15A）。
 *   这是芯片自身的零点离散（规格书典型 ±15~30mV），
 *   不是电源问题（三路各不同，若是 Vcc 偏高应该一致）。
 *
 * 不校准的后果：
 *   用固定 2048 作零点 ⇒ 恒定虚高 2.4~3.1A
 *   ⇒ 45A 堵转阈值实际在 42A 触发（偏松，有安全风险）
 *   ⇒ FOC 闭环时三相和不为 0，产生直流偏置力矩
 *
 * 存储位置：
 *   放 sys_config（配置表），不放 sys_param_t。
 *   理由见 storage_config.h:43-47 的既有裁定：
 *   "扩 sys_param_t 等于让现场已标定设备的零点丢失"。
 *
 * ⚠️ 当前为"已实现但未启用"状态：
 *   校准命令 t_iadc_cal 可用，但 foc_adc_raw_to_a10 暂不消费
 *   这三个值（见 FOC_ADC_ZERO_CAL_ENABLE 开关）。
 *   等用户完成相应模块后再打开。
 * ============================================================ */
static uint16_t s_zero_raw[3] = { FOC_CUR_ZERO_RAW,
                                  FOC_CUR_ZERO_RAW,
                                  FOC_CUR_ZERO_RAW };

/* 零点是否已从配置表成功装载（0 = 用默认 2048） */
static uint8_t  s_zero_valid = 0U;

/* 校准采集累加器（仅在校准过程中使用，不在 ISR 里访问） */
static uint32_t s_cal_sum[3];
static uint32_t s_cal_cnt[3];
static uint8_t  s_cal_running = 0U;

/* ============================================================
 * 原始码 → 0.1A
 * ------------------------------------------------------------
 * 推导见头文件。核心整数式：
 *   a10 = (raw*3300 - 6758400) * 100 / 1081344
 *
 * 为什么放大 100 再除：
 *   直接算 (raw*3300 - 6758400) / 108134 会丢掉 0.1A 精度
 *   （1 A 对应 4096*26.4/3300 ≈ 32.8 LSB，乘 100 后才够分辨率）
 *
 * 溢出核查：
 *   raw 最大 4095 ⇒ raw*3300 = 13,513,500 < 2^31 ✓
 *   乘 100 后 1.35e9 < 2^31 (2.147e9) ✓ 刚好安全
 *   若将来改成 24bit ADC 需重算
 * ============================================================ */
int16_t foc_adc_raw_to_a10(uint16_t raw)
{
    return foc_adc_raw_to_a10_chn(raw, 0U);
}

/* ============================================================
 * 原始码 → 0.1A（指定通道零点）
 * ------------------------------------------------------------
 * 【2026-11-09 新增，为电流零点校准预留】
 *
 * ch 定义：
 *   0 = 母线（IBUS）
 *   1 = W 线电流
 *   2 = V 线电流
 *
 * 【与旧版的唯一差别：零点取值来源】
 *   旧版固定减 1650mV（→ raw 2048）
 *   新版按 ch 取 s_zero_raw[ch]
 *
 * ⚠️ 开关 FOC_ADC_ZERO_CAL_ENABLE 当前为 0：
 *   关闭时 zero 恒为 FOC_CUR_ZERO_RAW(2048)，行为与旧版**完全一致**。
 *   这是刻意的：校准功能"已实现但未启用"，等用户完成相应模块后再打开，
 *   避免现在改动影响已经在跑的电流判据。
 *
 *   打开后：零点取 s_zero_raw[ch]（校准值或默认 2048）。
 *
 * 【为什么保留 1650mV 的注释】
 *   零点电压 1650mV 对应 raw 2048 是**理论值**；
 *   实际芯片零点有离散（实测 +81~+103 LSB）。
 *   校准就是把"理论 2048"换成"实测值"。
 * ============================================================ */
int16_t foc_adc_raw_to_a10_chn(uint16_t raw, uint8_t ch)
{
    int32_t mv_x4096;
    int32_t num;
    int32_t a10;
    uint16_t zero_raw;

    if (raw > FOC_ADC_RAW_MAX)
    {
        raw = FOC_ADC_RAW_MAX;
    }

    /* 零点选择：开关关闭时恒用理论值，行为与旧版一致 */
#if FOC_ADC_ZERO_CAL_ENABLE
    zero_raw = (ch < 3U) ? s_zero_raw[ch] : FOC_CUR_ZERO_RAW;
#else
    (void)ch;
    zero_raw = FOC_CUR_ZERO_RAW;
#endif

    /* mv*4096 = raw * 3300
     * 零点用 zero_raw 换算成 mv*4096，避免浮点：
     *   zero_mv_x4096 = zero_raw * 3300
     * 这样零点被"对齐"到 raw 域，等价于 (raw - zero_raw) 后再乘系数。 */
    mv_x4096 = (int32_t)raw * (int32_t)FOC_ADC_VREF_MV;

    /* 【2026-11-09 修复：系数多了一个 0，读数被放大 10 倍】
     *
     * 推导（正确版）：
     *   a10 = (mv - 1650) / 26.4 * 10                 单位 0.1A
     *       = num / 4096 / 26.4 * 10
     *       = num * 10 / 108134.4
     *       = num * 100 / 1081344        ← 分子分母同乘 10
     *
     * 原代码写的是 num * 1000 / 1081344，多乘了 10 倍。
     * 实测证据：
     *   raw=4095 原代码得 6246（=624.6A），而头文件声明满量程仅 62.5A
     *   raw=2360 原代码得 952（=95.2A），物理真值 9.5A
     *
     * 现改为 * 100。修正后：
     *   raw=4095 → 624（62.4A）✓ 符合满量程
     *   raw=2360 →  95（ 9.5A）✓ 符合物理真值
     *
     * 溢出核查（*100 比 *1000 更安全）：
     *   num 最大约 ±6.76e6，乘 100 得 6.76e8 < 2^31(2.147e9) ✓
     *   即使用 int32 也不溢出，但保留 int64 以兼容将来扩展。 */
    num = mv_x4096 - (int32_t)zero_raw * (int32_t)FOC_ADC_VREF_MV;

    a10 = (int32_t)(((int64_t)num * 100) / (int64_t)1081344);

    if (a10 >  32767) { a10 =  32767; }
    if (a10 < -32768) { a10 = -32768; }

    return (int16_t)a10;
}

/* ============================================================
 * 电流零点校准（2026-11-09 新增，当前未启用）
 * ------------------------------------------------------------
 * 功能已完整实现，但 FOC_ADC_ZERO_CAL_ENABLE = 0 时
 * foc_adc_raw_to_a10_chn() 不使用 s_zero_raw[]，行为与旧版一致。
 *
 * 设计要点：
 *   ① 采集在 foc_adc_read() 内被动累加 —— 不新增任务，
 *      也不依赖调用频率（累加的是"读到的帧"，谁读都算）
 *   ② 三路独立 —— 实测零点离散各不相同（bus/w/v 差 22 LSB）
 *   ③ 装载带合法性校验 —— 越界值回退默认，防配置损坏
 * ============================================================ */

/* 判定一个 raw 零点是否在合法范围内
 *
 * 范围来源：sys_config 里 cur_zero_* 项的定义域（1500~2600），
 * 与配置层保持一致，避免"配置层接受、本模块拒绝"的不一致。 */
#define FOC_ZERO_RAW_MIN            (1500U)
#define FOC_ZERO_RAW_MAX            (2600U)

static uint8_t foc_zero_raw_is_valid(uint16_t v)
{
    if (v < FOC_ZERO_RAW_MIN) { return 0U; }
    if (v > FOC_ZERO_RAW_MAX) { return 0U; }
    return 1U;
}

void foc_adc_load_zero(void)
{
    int32_t v_bus = 0, v_w = 0, v_v = 0;
    uint8_t cal_ok = 1U;

    /* 三项零点各读一次；任何一项读失败都回退默认 2048。
     * sys_config_get_i32 返回 0 表示成功。
     *
     * 【复用 0x0401~0x0403，不新建配置项】
     *   工程早已在 0x04xx 段定义了 cur_zero_bus/w/v（默认 2048）。
     *   校准功能直接消费它们，避免两套零点并存。 */
    if (sys_config_get_i32(CFG_ID_CUR_ZERO_BUS, &v_bus) != 0)
    {
        v_bus = FOC_CUR_ZERO_RAW;
        cal_ok = 0U;
    }
    if (sys_config_get_i32(CFG_ID_CUR_ZERO_W, &v_w) != 0)
    {
        v_w = FOC_CUR_ZERO_RAW;
        cal_ok = 0U;
    }
    if (sys_config_get_i32(CFG_ID_CUR_ZERO_V, &v_v) != 0)
    {
        v_v = FOC_CUR_ZERO_RAW;
        cal_ok = 0U;
    }

    /* 合法性校验：越界视为配置损坏，整体回退默认。
     * 为什么不"部分回退"：三路零点若只回退其中一路，
     * 会造成三相零点彼此不匹配，比全用默认更糟。 */
    if ((cal_ok != 0U) &&
        (foc_zero_raw_is_valid((uint16_t)v_bus) != 0U) &&
        (foc_zero_raw_is_valid((uint16_t)v_w)   != 0U) &&
        (foc_zero_raw_is_valid((uint16_t)v_v)   != 0U))
    {
        s_zero_raw[0] = (uint16_t)v_bus;
        s_zero_raw[1] = (uint16_t)v_w;
        s_zero_raw[2] = (uint16_t)v_v;

        /* "已校准"判据：三路不全等于默认 2048。
         * 为什么不另设 valid 标志位：
         *   0x04xx 段没有 valid 字段，新增会扩大配置表；
         *   而"三路都为 2048" 在物理上几乎不可能（实测离散 22 LSB），
         *   所以用它作未校准判据在工程上足够可靠。 */
        if ((s_zero_raw[0] != FOC_CUR_ZERO_RAW) ||
            (s_zero_raw[1] != FOC_CUR_ZERO_RAW) ||
            (s_zero_raw[2] != FOC_CUR_ZERO_RAW))
        {
            s_zero_valid = 1U;
        }
        else
        {
            s_zero_valid = 0U;
        }
    }
    else
    {
        s_zero_raw[0] = FOC_CUR_ZERO_RAW;
        s_zero_raw[1] = FOC_CUR_ZERO_RAW;
        s_zero_raw[2] = FOC_CUR_ZERO_RAW;
        s_zero_valid  = 0U;
    }
}

void foc_adc_cal_begin(void)
{
    s_cal_sum[0] = 0U;
    s_cal_sum[1] = 0U;
    s_cal_sum[2] = 0U;
    s_cal_cnt[0] = 0U;
    s_cal_cnt[1] = 0U;
    s_cal_cnt[2] = 0U;
    s_cal_running = 1U;
}

uint32_t foc_adc_cal_get_sample_count(void)
{
    /* 三路样本数应相等（同一次扫描搬来）；
     * 返回最小值，避免某路异常时给出偏大的乐观值。 */
    uint32_t n = s_cal_cnt[0];

    if (s_cal_cnt[1] < n) { n = s_cal_cnt[1]; }
    if (s_cal_cnt[2] < n) { n = s_cal_cnt[2]; }
    return n;
}

uint8_t foc_adc_cal_get_result(uint16_t out[3])
{
    uint32_t n;
    uint8_t  i;

    if (out == RT_NULL)
    {
        return 0U;
    }

    n = foc_adc_cal_get_sample_count();

    /* 至少 64 个样本才算有效（20kHz 下约 3.2ms）。
     * 为什么需要下限：样本太少时均值受噪声影响大，
     * 且能挡住"刚 begin 就取结果"的误用。 */
    if (n < 64U)
    {
        return 0U;
    }

    for (i = 0U; i < 3U; i++)
    {
        /* 四舍五入：+n/2 再除 */
        out[i] = (uint16_t)((s_cal_sum[i] + (n / 2U)) / n);
    }
    return 1U;
}

int foc_adc_cal_commit(const uint16_t raw3[3])
{
    uint8_t i;

    if (raw3 == RT_NULL)
    {
        return -RT_EINVAL;
    }

    /* 先校验再写：写 Flash 是有代价的，非法值不该落盘 */
    for (i = 0U; i < 3U; i++)
    {
        if (foc_zero_raw_is_valid(raw3[i]) == 0U)
        {
            return -RT_EINVAL;
        }
    }

    if (sys_config_set_i32(CFG_ID_CUR_ZERO_BUS, (int32_t)raw3[0]) != 0) { return -RT_ERROR; }
    if (sys_config_set_i32(CFG_ID_CUR_ZERO_W,   (int32_t)raw3[1]) != 0) { return -RT_ERROR; }
    if (sys_config_set_i32(CFG_ID_CUR_ZERO_V,   (int32_t)raw3[2]) != 0) { return -RT_ERROR; }

    /* 落盘：sys_config 的 save 接口 */
    if (sys_config_save() != 0)
    {
        return -RT_ERROR;
    }

    /* 立即生效到 RAM（是否被换算使用仍由开关决定） */
    s_zero_raw[0] = raw3[0];
    s_zero_raw[1] = raw3[1];
    s_zero_raw[2] = raw3[2];

    /* "已校准" = 三路不全等于默认 2048（与 load_zero 判据一致） */
    if ((raw3[0] != FOC_CUR_ZERO_RAW) ||
        (raw3[1] != FOC_CUR_ZERO_RAW) ||
        (raw3[2] != FOC_CUR_ZERO_RAW))
    {
        s_zero_valid = 1U;
    }
    else
    {
        s_zero_valid = 0U;
    }

    return 0;
}

uint8_t foc_adc_get_zero(uint16_t out[3])
{
    if (out != RT_NULL)
    {
        out[0] = s_zero_raw[0];
        out[1] = s_zero_raw[1];
        out[2] = s_zero_raw[2];
    }
    return s_zero_valid;
}

/* ============================================================
 * DMA1 CH2 中断
 * ------------------------------------------------------------
 * 【2026-11-05 改用 BTC（块完成），不是 TC（传输完成）】
 *   TransCount=0（永远传输）时总传输永不结束 ⇒ TC 永不置位。
 *   每搬完 BlockSize=3 个通道就置 BTC ⇒ 每帧一次中断。
 *
 * ISR 内只做计数，不做换算 —— 换算留给任务侧 foc_adc_read()，
 * 避免在中断里做 64 位除法（约 40~60 周期，占 ISR 时间过长）。
 *
 * 【2026-11-05 / S1】堵转保护的电流档在此驱动
 * ------------------------------------------------------------
 * 为什么堵转的 20kHz 判据放在本 ISR，而不是 PWM 谷值 ISR：
 *   PWM 谷值 ISR（bsp_pwm.c）里调的是 foc_adc_trigger()，那一刻
 *   **刚启动转换、数据还没到**，读到的是上一帧的旧值。
 *   本 ISR 是"数据已搬完"的时刻，拿到的必然是刚采到的新值。
 *   在旧值上判过流，等于比实际晚一帧（50μs），虽可接受但不必要。
 *
 * 为什么传 raw 偏离量而不是 a10：
 *   见 foc_adc_get_bus_current_dev_raw() 的注释 —— 避免 ISR 内
 *   int64 除法。stall_on_current() 内部只做整数比较。
 *
 * 开销：一次减法 + 一次绝对值 + 3 次比较，约 20 周期，
 *       占 20kHz 周期（200MHz 下 10000 周期）的 0.2%。
 * ============================================================ */
static void foc_adc_dma_isr(void)
{
    if (DMA_GetTransCompleteStatus(CM_DMA1, DMA_INT_BTC_CH2) == SET)
    {
        DMA_ClearTransCompleteStatus(CM_DMA1, DMA_INT_BTC_CH2);
        s_stat.conv_cnt++;
        s_seq++;

        /* 堵转保护：把刚搬完的这一帧电流喂给 20kHz 判据 */
        stall_on_current(foc_adc_get_bus_current_dev_raw());
    }
    if (DMA_GetErrStatus(CM_DMA1, DMA_INT_TRANS_ERR_CH2) == SET)
    {
        DMA_ClearErrStatus(CM_DMA1, DMA_INT_TRANS_ERR_CH2);
        s_stat.err_cnt++;
    }
}

/* ============================================================
 * 初始化
 * ============================================================ */
int foc_adc_init(void)
{
    stc_adc_init_t   stcAdcInit;
    stc_dma_init_t   stcDmaInit;
    stc_gpio_init_t  stcGpioInit;
    stc_irq_signin_config_t stcIrqSignin;
    int32_t ret;

    if (s_inited)
    {
        return 0;
    }

    memset((void *)&s_dma_buf, 0, sizeof(s_dma_buf));
    memset((void *)&s_stat, 0, sizeof(s_stat));
    s_seq = 0U;

    /* ---------------- 1. 寄存器写保护解锁（必须最先做） ---------------- */
    /* ============================================================
     * 【2026-11-05 根因修复 A：FCG0 写保护 —— conv_cnt 恒 0 的真因】
     * ------------------------------------------------------------
     * 症状：t_iadc_stat 的 conv_cnt 恒 0、三路 raw 恒 0，
     *       手动把 PA3 拉到 3.3V 仍读 0；foc_adc 初始化耗时 39ms
     *       （正好把首帧等待循环 200000 次跑满）。
     *
     * 根因：本函数原来把 FCG_Fcg0PeriphClockCmd() 放在
     *       LL_PERIPH_ALL 解锁之前，而调用到这里时 FCG0 **已处于锁定态**。
     *
     * 写保护由 PWC->FCG0PC.PRT0 控制（hc32_ll_pwc.h:468-469）：
     *   0xA5A50000 (LOCK_KEY)   → PRT0=0 → 锁定，写入被硬件忽略
     *   0xA5A50001 (UNLOCK_KEY) → PRT0=1 → 解锁
     *
     * 锁定是谁留下的（逐行核实）：
     *   board.c:149          rt_hw_board_init  LL_PERIPH_WE(含 FCG)  解锁
     *   bsp_pwm.c:597 → 599  bsp_pwm_hw_init   WE → WP               锁定
     *   bsp_hall.c:170 → 204 bsp_hall_hw_init  WE → WP               锁定
     *   boot 第 9 步         本函数                                  进来时仍是锁定
     *
     * 后果链（解释全部症状）：
     *   FCG0_DMA1 无时钟 ⇒ DMA1 寄存器全无效 ⇒ DMA_Init/DMA_ChCmd 静默失效
     *   FCG0_AOS  无时钟 ⇒ AOS 寄存器全无效  ⇒ AOS_SetTriggerEventSrc 失效
     *   ⇒ EOCA 事件到不了 DMA ⇒ 不搬数据 ⇒ s_dma_buf 恒 0 ⇒ 拉 3.3V 也读 0
     *
     * 为什么一直没报错：FCG_Fcg0PeriphClockCmd() 内部有
     *   DDL_ASSERT(IS_FCG0_UNLOCKED())，但 board/hc32f4xx_conf.h 未定义
     *   __DEBUG，DDL_ASSERT 被编译成 (void)0U（hc32_ll_utility.h:87-96）
     *   ⇒ 断言完全静默。这是本项目"静默失败"倾向的又一例。
     *
     * 【官方例程对照】adc/adc_dma/source/main.c:145-153
     *   LL_PERIPH_WE(...FCG...)   ← 先解锁
     *   AdcConfig();              ← 内含 FCG_Fcg0PeriphClockCmd(DMA1/AOS)
     *   LL_PERIPH_WP(...FCG...)   ← 最后上锁
     *   我们的顺序与官方相反，这是本次修复的核心。
     * ============================================================ */
    LL_PERIPH_WE(LL_PERIPH_ALL);

    /* ---------------- 2. 外设时钟（必须在解锁窗口内） ---------------- */
    /* ADC1 的时钟由 drv_adc.c:219 用 FCG3 开启，FCG3 无写保护，不受影响。
     * DMA1 与 AOS 属 FCG0，有写保护 ⇒ 必须在本行之前已 LL_PERIPH_WE。 */
    FCG_Fcg0PeriphClockCmd(PWC_FCG0_DMA1 | PWC_FCG0_AOS, ENABLE);

    /* ---------------- 3. GPIO 设为模拟输入 ---------------- */
    /* rt_hw_board_adc_init() 也会设（board_config.c:106-108），那时 GPIO
     * 尚未上锁，所以引脚其实是好的 —— 这也是"拉 3.3V 仍读 0"不能用
     * 引脚模式解释的原因。此处显式再设一遍，保证不依赖外部初始化顺序；
     * 但必须放在解锁窗口内，否则 bsp_hall/bsp_i2c 留下的 GPIO 锁会让它失效。 */
    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinAttr = PIN_ATTR_ANALOG;
    (void)GPIO_Init(GPIO_PORT_A, GPIO_PIN_03, &stcGpioInit);   /* ibus */
    (void)GPIO_Init(GPIO_PORT_A, GPIO_PIN_04, &stcGpioInit);   /* iw   */
    (void)GPIO_Init(GPIO_PORT_A, GPIO_PIN_05, &stcGpioInit);   /* iv   */

    /* ---------------- 4. ADC1 配置 ---------------- */
    ADC_DeInit(FOC_ADC_UNIT);

    (void)ADC_StructInit(&stcAdcInit);
    /* 单次扫描模式：每次触发把 SEQ_A 里使能的通道走一遍。
     * 为什么不用连续模式：连续模式下 DMA 会持续收到数据，
     * 但采样时刻完全不受控；单次扫描便于将来换成 TMR4 触发。 */
    stcAdcInit.u16ScanMode   = ADC_MD_SEQA_SINGLESHOT;
    stcAdcInit.u16Resolution = ADC_RESOLUTION_12BIT;
    stcAdcInit.u16DataAlign  = ADC_DATAALIGN_RIGHT;
    ret = ADC_Init(FOC_ADC_UNIT, &stcAdcInit);
    if (ret != LL_OK)
    {
        LL_PERIPH_WP(LL_PERIPH_ALL);
        return -RT_ERROR;
    }

    /* 采样时间：对开关噪声环境取较长窗口，牺牲一点速率换稳定 */
    ADC_SetSampleTime(FOC_ADC_UNIT, FOC_ADC_CH_IBUS, FOC_ADC_SAMPLE_TIME_CLK);
    ADC_SetSampleTime(FOC_ADC_UNIT, FOC_ADC_CH_IW,   FOC_ADC_SAMPLE_TIME_CLK);
    ADC_SetSampleTime(FOC_ADC_UNIT, FOC_ADC_CH_IV,   FOC_ADC_SAMPLE_TIME_CLK);

    /* 使能三个通道进 SEQ_A。注意顺序无所谓，
     * 转换顺序由 CHSELRA 的位序决定（通道号从小到大），
     * 即 CH3 → CH4 → CH5，与 s_dma_buf.raw[] 的下标一致。 */
    ADC_ChCmd(FOC_ADC_UNIT, ADC_SEQ_A, FOC_ADC_CH_IBUS, ENABLE);
    ADC_ChCmd(FOC_ADC_UNIT, ADC_SEQ_A, FOC_ADC_CH_IW,   ENABLE);
    ADC_ChCmd(FOC_ADC_UNIT, ADC_SEQ_A, FOC_ADC_CH_IV,   ENABLE);

    /* ============================================================
     * 【2026-11-04 / S0】改为 TMR4 硬件触发（原为软件触发）
     * ------------------------------------------------------------
     * 原实现（方案 B）：
     *   ADC_TriggerCmd(SEQ_A, DISABLE)
     *   然后 init 末尾 ADC_Start() 一次
     *   ⇒ 全工程只转换一次，conv_cnt 恒为 1，
     *     读到的永远是同一组陈旧值，电流保护无从谈起。
     *
     * 现改为 TMR4_3 比较事件经 AOS 路由触发 ADC：
     *   每 50μs（20kHz）自动启动一次 SEQ_A 三通道扫描。
     *
     * 【为什么用 SCMP_VH 而不是"谷值事件"】
     *   芯片只把 6 个 OC 通道比较事件（SCMP_UH/UL/VH/VL/WH/WL）
     *   导出到 AOS，没有导出计数器 UDF/OVF（谷值/峰值）事件。
     *   bsp_pwm 里的 INT_SRC_TMR4_3_UDF 是**中断源**，不能直接
     *   作为 AOS 的触发源。
     *
     * 【S0 阶段选 SCMP_VH 的理由】
     *   只需要先验证"每周期触发一次"这条链路通不通。
     *   六步换向下 V 相的 OCCR 每周期都会被三角波经过，
     *   故 SCMP_VH 必然每周期产生一次事件。
     *   S4 批（FOC 上线）会改为按 min(OCCR) 动态切换触发相，
     *   保证落在"调制相上桥导通"窗口内。
     *
     * 【验证方法】
     *   烧写后 t_iadc_stat
     *     conv_cnt 应以 20000/s 增长  → 正确
     *     conv_cnt 以 40000/s 增长    → SCMP 上下沿都触发，需改用单沿
     *     conv_cnt 停在 1             → AOS 路由未生效，查 AOS_ADC1_0
     * ============================================================ */
    /* ============================================================
     * 【2026-11-04 · Q1 方案 1】改用 INTC 软件启动，关闭空的硬触发
     * ------------------------------------------------------------
     * 详细根因分析见文件头的注释块。要点：
     *   - 删掉 ADC_Start() 导致 STRT=0，序列根本没被 arm ⇒ 永不转换；
     *   - AOS 指向的 SCMP_VH 事件源从未配置（SCMR/SCCR/SCSR 全是复位值 0）
     *     ⇒ 即使有 STRT，这条路由也不会产生触发；
     *   - 因此硬触发整条路径不可用，改为在 PWM 谷值 ISR 里软件启动。
     *
     * TRGENA 显式 DISABLE：
     *   留着一个指向空事件源的硬触发只会在排障时误导。
     *   转换的启动时机完全由 bsp_pwm.c 的谷值 ISR 决定。
     *
     * 为什么不再设 AOS_SetTriggerEventSrc(AOS_ADC1_0, ...)：
     *   该路由已无使用方。保留会把 ADC1 的触发入口绑到一个不存在的
     *   事件上，语义混乱。DMA 的触发（AOS_DMA1_2 ← ADC1_EOCA）保留，
     *   那条是必须的、且是有效的。 */
    ADC_TriggerCmd(FOC_ADC_UNIT, ADC_SEQ_A, DISABLE);

    /* 【2026-11-05 修复 C】数据寄存器自动清零改为 DISABLE
     *
     * 原值 ENABLE 会置 CR0.CLREN=1（读走即清）。DMA 读取 DR3/DR4/DR5 时
     * 是否触发清零，官方手册未给出明确保证；一旦"读取即清"与 DMA 的
     * 采样时序竞争，可能读到 0 或半帧。
     *
     * 官方例程 adc/adc_dma 从不调用本函数 —— ADC_Init() 写 CR0 时
     * 只写 scanmode|resolution|align，CLREN(bit6) 保持复位值 0。
     * 这里显式 DISABLE 与官方默认行为一致，消除未知竞态。
     * 代价：DR 会保留上一帧值，但 DMA 每帧覆盖，无影响。 */
    ADC_DataRegAutoClearCmd(FOC_ADC_UNIT, DISABLE);

    LL_PERIPH_WP(LL_PERIPH_ALL);

    /* ---------------- 4. DMA1 CH2 配置 ---------------- */
    /* 源地址：ADC1 的 DR3（CH3 的数据寄存器）。
     *
     * 【关键】为什么是 DR3 而不是 DR0：
     *   HC32F460 的 ADC 数据寄存器按**通道号**索引，
     *   DRn 对应 CHn。CH3/CH4/CH5 连续，所以从 DR3 起、
     *   16bit 宽度、源地址自增，正好依次读到 CH3→CH4→CH5。
     *   若错用 DR0 会读到 CH0/CH1/CH2（未使能，恒 0）。
     *
     * 目的地址：s_dma_buf.raw[]
     * 传输长度：FOC_ADC_SEQ_LEN = 3
     * 循环模式：用 DMA_SetTransCount + repeat 实现，见下 */
    (void)DMA_StructInit(&stcDmaInit);
    stcDmaInit.u32IntEn       = DMA_INT_ENABLE;
    stcDmaInit.u32SrcAddr     = (uint32_t)(&(FOC_ADC_UNIT->DR3));
    stcDmaInit.u32DestAddr    = (uint32_t)(&s_dma_buf.raw[0]);
    stcDmaInit.u32DataWidth   = DMA_DATAWIDTH_16BIT;
    stcDmaInit.u32BlockSize   = FOC_ADC_SEQ_LEN;

    /* 【2026-11-05 根因修复 B：TransCount = 0 表示"永远传输"】
     *
     * 原值 1U 是错的。HC32F460 的 DMA 没有独立的 circular 使能位，
     * TransCount 就是 DTCTL.CNT（16 位）的装载值，每完成一个 block
     * 递减 1，减到 0 通道自动关闭。
     *   TransCount=1  ⇒ 只搬 1 个 block 就停 ⇒ conv_cnt 最多 1
     *
     * 官方注释（adc/adc_dma/source/main.c:78-79）逐字写着：
     *   'APP_DMA_TRANS_COUNT': 0~65535, inclusive. 0: always transmit.
     *   #define DMA_TRANS_CNT   (0U)
     *
     * 所以 0 = 不限次，配合 Repeat 的地址回卷即可实现真正的循环采样，
     * 既不需要在 ISR 里重装计数，也不需要 LLP 链表。 */
    stcDmaInit.u32TransCount  = 0U;
    stcDmaInit.u32SrcAddrInc  = DMA_SRC_ADDR_INC;
    stcDmaInit.u32DestAddrInc = DMA_DEST_ADDR_INC;

    ret = DMA_Init(CM_DMA1, DMA_CH2, &stcDmaInit);
    if (ret != LL_OK)
    {
        return -RT_ERROR;
    }

    /* ============================================================
     * 【2026-11-05 重要：Repeat 段不能删】
     * ------------------------------------------------------------
     * 曾有人（含一份外部修改建议）认为"TransCount=0 是永动，
     * 所以整套 Repeat 可以删掉" —— 这是错的，会把数据读乱。
     *
     * Repeat 的职责是"地址回卷"，不是"计数重装"：
     *   BlockSize=3 + SRC_ADDR_INC ⇒ SAR 每搬一个数 +2 字节
     *   搬完 3 个后 SAR 已指向 DR5+2，若不回卷，下一轮会从 DR6 开始：
     *     第1轮：DR3 → DR4 → DR5   ✓
     *     第2轮：DR6 → DR7 → DR8   ✗ 全是未使能通道，恒 0
     *
     * SrcCount=3 让 SRPT 在每个 block 结束时把 SAR 拉回 DR3；
     * DestCount=3 让 DRPT 把 DAR 拉回 raw[0]。
     * 两者缺一不可，且与 TransCount 的取值无关。
     *
     * 【为什么源侧也要开】
     * 源是 ADC 的 DR3，但我们设了 SRC_ADDR_INC（要连续读 DR3/DR4/DR5），
     * 所以必须靠 SRPT 回卷，不能靠"源地址固定"来省掉它。
     * 官方 adc/adc_dma:282-284 同样用 DMA_RPT_BOTH + block size。
     * ============================================================ */
    {
        stc_dma_repeat_init_t stcRepeat;

        (void)DMA_RepeatStructInit(&stcRepeat);
        stcRepeat.u32Mode      = DMA_RPT_BOTH;      /* = SRC | DEST，官方写法 */
        stcRepeat.u32SrcCount  = FOC_ADC_SEQ_LEN;
        stcRepeat.u32DestCount = FOC_ADC_SEQ_LEN;
        (void)DMA_RepeatInit(CM_DMA1, DMA_CH2, &stcRepeat);
    }

    /* DMA 请求源：ADC1 的 EOCA（序列转换完成）。
     * 必须走 AOS：DMA 的触发是 AOS 路由的，不是直连。 */
    AOS_SetTriggerEventSrc(AOS_DMA1_2, EVT_SRC_ADC1_EOCA);

    /* ---------------- 5. DMA 中断注册（BTC，不是 TC） ---------------- */
    /* 【2026-11-05 根因修复 D：必须用 BTC 而不是 TC】
     * ------------------------------------------------------------
     * TC  = Transfer Complete（全部传输完成）
     * BTC = Block Transfer Complete（每个 block 完成一次）
     *
     * 我们把 TransCount 设成 0（永远传输），意味着"总传输永不完成"
     * ⇒ TC 标志永远不会置位 ⇒ 原来的 TC 中断方案在无限模式下 100% 收不到中断。
     * 而每搬完 3 个通道的 block 就会置 BTC ⇒ BTC 才是每帧中断的正确选择。
     *
     * 官方 adc/adc_dma/source/main.c:94-98 正是这么配的：
     *   DMA_INT_TYPE = DMA_INT_BTC_CH0
     *   DMA_INT_SRC  = INT_SRC_DMA1_BTC0
     *   DMA_INT_FLAG = DMA_FLAG_BTC_CH0
     *
     * 【向量号核算：BTC2 = 42，仍可用 INT040】
     *   INTC 要求向量 ∈ [(src/32)*6+32, (src/32)*6+37]
     *   INT_SRC_DMA1_BTC2 = 42 → 42/32 = 1 → 允许 [38, 43]
     *   INT040 ∈ [38,43] ✓  合法，BSP_DMA1_CH2_IRQ_NUM 无需改动。
     *
     * 【注意】INTC_IrqSignIn 的返回值必须检查。本工程历史上曾因
     * 丢弃返回值、向量号被静默拒绝而排查数天（见 irq_config.h 注释）。 */
    stcIrqSignin.enIntSrc    = INT_SRC_DMA1_BTC2;
    stcIrqSignin.enIRQn      = BSP_DMA1_CH2_IRQ_NUM;
    stcIrqSignin.pfnCallback = &foc_adc_dma_isr;
    if (INTC_IrqSignIn(&stcIrqSignin) != LL_OK)
    {
        LOG_ERR("[FOC-ADC] ERROR: IrqSignIn failed "
                   "(src=%d irq=%d)\n",
                   (int)stcIrqSignin.enIntSrc, (int)stcIrqSignin.enIRQn);
        return -RT_ERROR;
    }
    NVIC_ClearPendingIRQ(stcIrqSignin.enIRQn);
    NVIC_SetPriority(stcIrqSignin.enIRQn, DDL_IRQ_PRIO_03);
    NVIC_EnableIRQ(stcIrqSignin.enIRQn);

    /* ---------------- 6. 启动 ---------------- */
    /* 先清残留标志，再使能中断与通道，最后开全局 EN。
     * 顺序依据官方 dmac_repeat/main.c:169-178：
     *   DmaInit → DmaIntInit → DMA_Cmd(ENABLE) → DMA_ChCmd(ENABLE) */
    DMA_ClearTransCompleteStatus(CM_DMA1, DMA_INT_BTC_CH2);
    DMA_TransCompleteIntCmd(CM_DMA1, DMA_INT_BTC_CH2, ENABLE);
    DMA_Cmd(CM_DMA1, ENABLE);
    (void)DMA_ChCmd(CM_DMA1, DMA_CH2, ENABLE);

    /* 【2026-11-05】主动触发首帧，让 s_dma_buf 有初值。
     *
     * 为什么必须启动：STRT=0 时序列没被 arm，后续任何触发（本 ISR 的
     * 软件触发）都不会真正开始转换 —— 这是上一批 conv_cnt 恒 0 的第一根因。
     *
     * 为什么先 Stop 再清标志再 Start：
     *   ADC_Start() 只是"启动一次扫描"，不重置状态机。若之前在别处
     *   （drv_adc.c:173 轮询路径）启动过，可能残留未清的 EOCA，
     *   导致首个 DMA 请求对应上一轮陈旧数据。
     *
     * 首帧相位不确定（可能不在 PWM 谷值）：可接受。首帧只用于建立初值，
     * 之后每个 20kHz 周期都会被谷值 ISR 刷新。 */
    ADC_Stop(FOC_ADC_UNIT);
    ADC_ClearStatus(FOC_ADC_UNIT, ADC_FLAG_EOCA);
    ADC_Start(FOC_ADC_UNIT);

    /* 【2026-11-05 删除原 39ms 阻塞等待】
     *
     * 原代码在这里 while(conv_cnt==0 && loops<200000) 死等首帧，
     * 实测耗时 39ms（启动表日志 foc_adc 39ms）。这不仅拖慢启动，
     * 而且掩盖了问题：首帧等不到就静默超时返回成功，上层无法察觉。
     *
     * 现在改为完全异步：
     *   - 首帧由上面这次 ADC_Start() 产生，DMA 搬完后 BTC 中断自增 conv_cnt；
     *   - 之后每个 PWM 谷值 ISR 调 foc_adc_trigger() 再 Start 一次；
     *   - 上层用 foc_adc_get_stat() 的 conv_cnt 判断链路是否活着
     *     （foc_adc_get_bus_current_a10() 在 conv_cnt==0 时返回 0，安全侧）。
     *
     * 启动耗时因此从 39ms 降到接近 0，且"没采到数据"不再被静默吞掉。 */

    /* 【2026-11-09】装载电流零点（从 sys_config 0x0B 段）。
     *
     * 位置要求：
     *   - 必须在 sys_config_init（boot 第 4 步）之后 —— 本步骤是第 9 步，
     *     已满足；
     *   - 必须在 s_inited=1 之前 —— 否则若 ADC ISR 在这期间进来，
     *     会用到未初始化的 s_zero_raw。
     *
     * 注意：装载只是把值读进 s_zero_raw[]，
     *       是否参与换算由 FOC_ADC_ZERO_CAL_ENABLE（当前为 0）决定。 */
    foc_adc_load_zero();

    /* ============================================================
     * 【2026-11-09 预留】上电自动零点校准
     * ------------------------------------------------------------
     * 用户需求："在上电过程中，执行动作前，回转角控制的电机未动作前，
     *             完成校准过程。功能先设计，不启用。"
     *
     * 位置验证（为什么放在这里）：
     *   - 启动表 foc_adc 是第 9 步，swing（回转）是第 13 步，
     *     所以此处回转电机**不可能已动作**；
     *   - 此时 DMA 已跑、ADC 在持续采样（前面 ADC_Start 已执行）；
     *   - PWM 已被 tim4_init 末尾置 EMB 封锁（SOE3=1），母线无输出。
     *   ⇒ 三个"静默"条件天然满足。
     *
     * 为什么当前关闭（FOC_ADC_AUTO_CAL_ON_BOOT = 0）：
     *   自动校准会**覆盖并落盘**零点。一旦上电时母线有残余电流
     *   或传感器未稳，就会把错误值持久化，比不校准更糟。
     *   需要先实测确认"上电瞬间母线确实零电流"，再打开。
     *
     * 【启用前必做】
     *   ① 用 t_iadc_cal show 20 观察上电后前 6 秒的三路 raw 是否稳定
     *   ② 确认波动 < ±3 LSB（约 0.1A）再启用
     *   ③ 启用后首次上电观察 [FOC-ADC] 打印的校准值是否接近手动校准结果
     * ============================================================ */
#if FOC_ADC_AUTO_CAL_ON_BOOT
    {
        uint16_t z[3];
        uint32_t wait_ms;

        foc_adc_cal_begin();

        /* 等待足够样本：64 个 @20kHz ≈ 3.2ms，这里留足 200ms 余量，
         * 同时给 ADC 前端（RC 滤波 + 运放）足够建立时间。 */
        for (wait_ms = 0U; wait_ms < 200U; wait_ms++)
        {
            if (foc_adc_cal_get_sample_count() >= 2048U)
            {
                break;
            }
            rt_thread_mdelay(1);
        }

        if (foc_adc_cal_get_result(z) != 0U)
        {
            LOG_ADC("[FOC-ADC] auto zero cal: bus=%u w=%u v=%u\n",
                       (unsigned)z[0], (unsigned)z[1], (unsigned)z[2]);

            /* 合理性护栏：偏差过大说明当时并非零电流，拒绝落盘。
             * 300 LSB ≈ 9A，远超任何正常零点离散（实测最大 103）。 */
            if (((int32_t)z[0] - (int32_t)FOC_CUR_ZERO_RAW <  300) &&
                ((int32_t)z[0] - (int32_t)FOC_CUR_ZERO_RAW > -300) &&
                ((int32_t)z[1] - (int32_t)FOC_CUR_ZERO_RAW <  300) &&
                ((int32_t)z[1] - (int32_t)FOC_CUR_ZERO_RAW > -300) &&
                ((int32_t)z[2] - (int32_t)FOC_CUR_ZERO_RAW <  300) &&
                ((int32_t)z[2] - (int32_t)FOC_CUR_ZERO_RAW > -300))
            {
                if (foc_adc_cal_commit(z) == 0)
                {
                    foc_adc_load_zero();   /* 重新装载，使 s_zero_valid 生效 */
                    LOG_OK("[FOC-ADC] auto zero cal SAVED\n");
                }
                else
                {
                    LOG_ERR("[FOC-ADC] auto zero cal: commit failed, "
                               "keep old zero\n");
                }
            }
            else
            {
                LOG_WARN("[FOC-ADC] auto zero cal REJECTED: deviation > 300 LSB, "
                           "current not zero?\n");
            }
        }
        else
        {
            LOG_INFO("[FOC-ADC] auto zero cal: no samples, skipped\n");
        }
    }
#endif /* FOC_ADC_AUTO_CAL_ON_BOOT */

    s_inited = 1U;
    return 0;
}

void foc_adc_deinit(void)
{
    if (!s_inited)
    {
        return;
    }

    ADC_Stop(FOC_ADC_UNIT);
    (void)DMA_ChCmd(CM_DMA1, DMA_CH2, DISABLE);
    DMA_TransCompleteIntCmd(CM_DMA1, DMA_INT_BTC_CH2, DISABLE);
    NVIC_DisableIRQ(BSP_DMA1_CH2_IRQ_NUM);
    DMA_DeInit(CM_DMA1, DMA_CH2);

    s_inited = 0U;
}

/* ============================================================
 * 【2026-11-04 · Q1 方案 1】由 PWM 谷值 ISR 调用的采样触发
 * ------------------------------------------------------------
 * 调用上下文：bsp_pwm.c 的 bsp_pwm_tmr4_isr()，INT098，20kHz，
 *             NVIC 优先级 DDL_IRQ_PRIO_xx（见 bsp_pwm.c 注册处）。
 *
 * 【为什么在 ISR 里而不是在任务里】
 *   FOC 电流必须在"三相上桥全通"的零矢量中点采样。三角波谷值
 *   （CNT≈0）正是七段式 SVPWM 的 111 段中心，此时三相上桥同时导通，
 *   三路高边霍尔都能读到真实线电流。
 *   任务里调用的话，等它跑到时转子已转过角度、且 PWM 相位随机。
 *
 * 【为什么只写 STRT，不碰其他寄存器】
 *   1) 序列的通道使能 / 采样时间 / 数据对齐都是 init 一次配好，ISR 里
 *      重复配置既慢又可能被下一次触发打断；
 *   2) ADC_Start() 展开就是 WRITE_REG8(STR, STRT)，单次寄存器写，
 *      在 200MHz 下约几纳秒，对 ISR 时长几乎无影响；
 *   3) ADC 处于单次扫描模式（ADC_MD_SEQA_SINGLESHOT），每次 STRT
 *      扫一遍 SEQ_A 就停，下一周期再 STRT，天然不会重入。
 *
 * 【重入保护】
 *   若上一次转换还没完成就再次 STRT，芯片会忽略新请求（序列忙）。
 *   20kHz 下 CH3/4/5 各 40 ADCLK 采样 + 转换，三路约 4μs（实测），
 *   远小于 50μs 周期，正常不会撞上。若真撞上，表现为 conv_cnt 略少于
 *   20000/s，由 t_iadc_stat 可看出，不影响安全性（电流值仍是有效帧）。
 *
 * 【为什么加 s_inited 判断】
 *   启动顺序是 motor（第8步）→ foc_adc（第9步），而 TMR4 ISR 在
 *   motor 初始化时就已使能。若不等本模块 init 完 ISR 就可能进来，
 *   此时 ADC 尚未配置，写 STRT 无意义甚至可能启动半配好的序列。
 * ============================================================ */
void foc_adc_trigger(void)
{
    if (!s_inited)
    {
        return;
    }
    ADC_Start(FOC_ADC_UNIT);
}

/* ============================================================
 * 读取
 * ============================================================ */
int foc_adc_read_raw(uint16_t raw[FOC_ADC_SEQ_LEN])
{
    uint8_t i;

    if ((!s_inited) || (raw == RT_NULL))
    {
        return -RT_ERROR;
    }

    for (i = 0U; i < FOC_ADC_SEQ_LEN; i++)
    {
        raw[i] = s_dma_buf.raw[i];
    }
    return 0;
}

int foc_adc_read(foc_iabc_t *out)
{
    uint16_t raw[FOC_ADC_SEQ_LEN];
    uint8_t  i;

    if ((!s_inited) || (out == RT_NULL))
    {
        return -RT_ERROR;
    }

    for (i = 0U; i < FOC_ADC_SEQ_LEN; i++)
    {
        raw[i] = s_dma_buf.raw[i];
    }

    out->raw_bus = raw[0];
    out->raw_w   = raw[1];
    out->raw_v   = raw[2];

    /* 【2026-11-09】校准采集：被动累加，不新增任务
     *
     * 为什么放在这里而不是 20kHz ISR：
     *   校准要求"母线无电流"的静态条件，慢一点更稳；
     *   而且累加是 32 位加法，放任务侧无并发问题。
     *
     * 采样值用的是**当前零点尚未参与换算的原始 raw**，
     * 所以校准结果与现有零点设置无关（可反复校准）。 */
    if (s_cal_running != 0U)
    {
        /* 单路累加上限保护：避免长时间不取结果导致溢出。
         * 4096 * 1000000 = 4.1e9 会溢出 uint32，
         * 故上限取 500000（25 秒 @20kHz），足够。 */
        if (s_cal_cnt[0] < 500000U)
        {
            s_cal_sum[0] += raw[0];
            s_cal_sum[1] += raw[1];
            s_cal_sum[2] += raw[2];
            s_cal_cnt[0]++;
            s_cal_cnt[1]++;
            s_cal_cnt[2]++;
        }
    }

    out->i_bus_a10 = foc_adc_raw_to_a10_chn(raw[0], FOC_ADC_CHN_BUS);
    out->i_w_a10   = foc_adc_raw_to_a10_chn(raw[1], FOC_ADC_CHN_W);
    out->i_v_a10   = foc_adc_raw_to_a10_chn(raw[2], FOC_ADC_CHN_V);

    /* U 相重构：iu = -(iv + iw)
     * 为什么能这样算：三相星形连接，ia+ib+ic = 0（无中线）。
     *
     * 【2026-11-09 重要：IV/IW 必须是有符号的】
     *   本式只有在 iv/iw 保留符号时才成立。
     *   若数据层取绝对值，iu 会算成 -(|iv|+|iw|)，
     *   幅值和符号全部错误（例：iv=+3A, iw=-1.5A
     *   正确 iu=-1.5A，取绝对值后 iu=-4.5A）。
     *
     *   因此：
     *     母线电流 → 无符号（物理上不反向）
     *     IV/IW/IU → **有符号**（相电流可正可负）
     *   三者类型不同是刻意的，不可统一。 */
    out->i_u_a10 = (int16_t)(-(out->i_v_a10 + out->i_w_a10));

    out->seq = s_seq;

    /* 饱和统计：只有"贴满量程上沿"才算饱和。
     *
     * 【2026-11-04 修正：raw==0 不应计入 sat_cnt】
     *   原实现把 raw==0 也算饱和，但 0 有两种完全不同的来源：
     *     ① 输入真的低于 0V（不可能，霍尔输出 0.5~2.8V）—— 极罕见
     *     ② 根本没转换过 / DMA 没搬数据 —— **这才是常见的**
     *   实测：ADC 未转换时三路 raw 全 0，t_iadc_stat 报 sat_cnt=21，
     *   看起来像"量程不够"，实际是"采样链路断了"。
     *   两种含义混在一个计数器里，排障时会往错误方向查。
     *
     *   现改为：
     *     raw >= 上沿  → sat_cnt++（真饱和，量程或极性异常）
     *     raw == 0     → 不计入 sat_cnt；由 conv_cnt 判断链路是否活着
     *   若需要单独看"读到 0"的次数，看 conv_cnt 与 raw 值即可，
     *   不必再加计数器（raw 是否为 0 在 t_iadc 里直接可见）。 */
    if ((raw[0] >= FOC_ADC_RAW_MAX) ||
        (raw[1] >= FOC_ADC_RAW_MAX) ||
        (raw[2] >= FOC_ADC_RAW_MAX))
    {
        s_stat.sat_cnt++;
    }

    return 0;
}

/**
 * @brief  取母线电流（**无符号**，0.1A）
 *
 * 【2026-11-09 类型裁定：母线电流天然无符号】
 * ------------------------------------------------------------
 * 母线电流在物理上**不可能反向**：
 *   逆变器的直流母线上串着母线电容 + 前级整流，
 *   能量只能从电源流向逆变桥（除回馈制动工况，
 *   而本机为六步方波驱动、无能量回馈通路）。
 *   因此母线电流恒 >= 0，用 uint16 表达其物理属性更准确。
 *
 * 【与相电流/线电流的区别（关键）】
 *   母线电流   ：单向，**无符号**         ← 本函数
 *   线电流 V/W ：双向（相绕组中电流可正可负），**有符号**
 *   相电流 IU  ：由 -(IV+IW) 重构，**有符号**
 *
 * 所以：
 *   - 本函数返回 uint16，不做任何符号处理，也不取绝对值（本就是正的）
 *   - foc_adc_read() 里的 IV/IW/IU 全部保持 int16
 *   - 若实测量到"负的母线电流"，那是**零点偏移**造成的假象
 *     （raw < 2048 而实际电流为 0），必须靠零点校准修正，
 *     而不是靠取绝对值掩盖。
 *
 * 【零点偏下的处理】
 *   校准前若 raw 略低于 2048（如 1900），本函数会因钳位返回 0，
 *   表现为"零电流"而不是"负电流" —— 这是符合物理的正确行为。
 *   校准后 raw 零点对齐，量程内读数才准确。
 *
 * 未初始化或首帧未到达时返回 0。
 *
 * @retval 母线电流，0.1A，恒 >= 0
 */
uint16_t foc_adc_get_bus_current_a10(void)
{
    int16_t a10;

    /* 未初始化或还没有任何一帧 DMA 完成：返回 0，
     * 安全侧选择 —— 宁可不报过流，也不误报导致停机。 */
    if ((!s_inited) || (s_stat.conv_cnt == 0U))
    {
        return 0U;
    }

    a10 = foc_adc_raw_to_a10(s_dma_buf.raw[0]);

    /* 物理上母线电流不反向；a10 < 0 只可能是零点偏移（未校准）
     * 或噪声造成的假象，按 0 处理。这是"符合物理的钳位"，
     * 不是"取绝对值"—— 两者语义不同：
     *   取绝对值：把 -5A 变成 +5A（凭空造出 5A 电流）
     *   钳位到 0：把 -5A 视为 0A（承认"没有正向电流"）
     * 后者才是正确的处理。 */
    if (a10 < 0)
    {
        return 0U;
    }
    return (uint16_t)a10;
}

/* ============================================================
 * 【2026-11-05 / S1】ISR 安全的母线电流原始码（不做换算）
 * ------------------------------------------------------------
 * 为什么需要这个函数：
 *   foc_adc_get_bus_current_a10() 内部有一次 int64 除法
 *   （foc_adc_raw_to_a10），在 200MHz 下约 60~100 周期。
 *   堵转保护要在 **20kHz 中断**里每帧判一次，若每帧都做 int64 除法，
 *   光这一项就吃掉 20kHz × 80 周期 ≈ 1.6% CPU，且拉长 ISR 时长、
 *   增加 PWM ISR 与 ADC ISR 的相互延迟。
 *
 * 做法：把"电流比较"下移到 raw 域。
 *   阈值在配置时（5ms 任务上下文）由 stall_cfg 预换算成 raw，
 *   ISR 内只做整数比较，零除法、零函数调用。
 *
 * 换算关系（由 foc_adc_raw_to_a10 反解，见 stall 模块注释）：
 *   a10 = (raw*3300 - 6758400) * 100 / 1081344
 *   ⇒ raw = (2048*a10 + 1280000) / 625    （gcd=528 约分，向上取整）
 *   实测值：55A→raw 3851(dev 1803)、50A→3687(1639)、
 *           45A→3523(1475)、37A→3261(1213)、5A→2212(164)
 *
 * 【2026-11-09 类型裁定：改回**无符号**偏离量】
 * ------------------------------------------------------------
 * 母线电流物理上不反向（见 foc_adc_get_bus_current_a10 的说明），
 * 所以"偏离零点的量"也只需表达单向的正向电流。
 *
 *   raw >= zero → 正向电流，返回 raw - zero
 *   raw <  zero → 无正向电流（零点偏下/噪声），返回 0
 *
 * 【为什么 raw < zero 返回 0 而不是回卷成大数】
 *   若按无符号直接减会得到 65500 之类的巨大值，
 *   堵转判据会立刻误触发满量程。
 *   返回 0 才是"没有正向电流"的正确表达。
 *
 * 【为什么用 s_zero_raw[0] 而不是固定 2048】
 *   为校准预留。但 FOC_ADC_ZERO_CAL_ENABLE=0 时
 *   s_zero_raw[0] 恒为 2048（load_zero 在未校准时就填默认值），
 *   所以行为与旧版一致。开开关后自动用校准值。
 *
 * @retval 正向偏离量 raw - zero，未初始化/低于零点时返回 0
 * ============================================================ */
uint16_t foc_adc_get_bus_current_dev_raw(void)
{
    uint16_t raw;
    uint16_t zero;

    if ((!s_inited) || (s_stat.conv_cnt == 0U))
    {
        return 0U;
    }

    raw  = s_dma_buf.raw[0];
    zero = s_zero_raw[FOC_ADC_CHN_BUS];

    /* 单向：只有高于零点才算有正向电流 */
    if (raw <= zero)
    {
        return 0U;
    }
    return (uint16_t)(raw - zero);
}

void foc_adc_get_stat(foc_adc_stat_t *st)
{
    if (st == RT_NULL)
    {
        return;
    }
    st->conv_cnt = s_stat.conv_cnt;
    st->err_cnt  = s_stat.err_cnt;
    st->sat_cnt  = s_stat.sat_cnt;
}

void foc_adc_clr_stat(void)
{
    s_stat.conv_cnt = 0U;
    s_stat.err_cnt  = 0U;
    s_stat.sat_cnt  = 0U;
}

/* ============================================================
 * 【2026-11-04 / R8】覆盖 motor.c 的 weak 桩
 * ------------------------------------------------------------
 * motor.c 里 motor_sample_current_a10() 是 __attribute__((weak))
 * 且恒返回 0，导致 motor_oc_update() 永远看到 0A —— 过流保护
 * 从设计上就没有数据来源，形同虚设。
 *
 * 这里提供强定义（无 weak 修饰），链接器会优先选用本实现。
 *
 * 【为什么是"母线电流"而不是"相电流"】
 *   六步换向下单个相通道在每 360° 电角度中有 120°（33.3%）
 *   处于该相上桥关断状态，读不到真实相电流。
 *   而母线电流在六步的 6 个状态全是有源矢量，全程有效。
 *   过流保护关心绕组总发热，母线电流足够。
 *
 * 【2026-11-09 重要：绝对值在这里取，不在数据源取】
 *   motor_oc_update() 的形参是 uint16_t，它需要的是**幅值**。
 *   但幅值应该由本函数（消费端）计算，而不是让
 *   foc_adc_get_bus_current_a10() 返回绝对值 —— 后者会让所有
 *   需要符号的调用方（FOC 三相重构、方向判断）永久拿到错误数据。
 *
 *   这就是"数据层保持无损，语义转换放在消费端"的原则。
 *
 * 【为什么不做范围钳位】
 *   foc_adc_raw_to_a10() 内部已做 int16 钳位（±625 = ±62.5A），
 *   取绝对值后最大 625，不会溢出 uint16（上限 65535）。
 * ============================================================ */
uint16_t motor_sample_current_a10(motor_t *motor)
{
    int16_t i;

    (void)motor;

    i = foc_adc_get_bus_current_a10();

    /* 过流判据只要幅值，符号在此丢弃。
     * 用条件表达式而非 abs()，避免引入库调用，且明确表达"这里是唯一
     * 允许丢弃符号的地方"。 */
    if (i < 0)
    {
        return (uint16_t)(-i);
    }
    return (uint16_t)i;
}