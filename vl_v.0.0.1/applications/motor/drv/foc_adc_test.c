/**
 * @file    foc_adc_test.c
 * @brief   FOC 电流采样诊断命令（方案 B 验证用）
 *
 * ============================================================
 * 【验证目标：方案 B 阶段必须证明的四件事】
 * ------------------------------------------------------------
 *   V1 通道映射对不对   → 单路给已知电流，看对应通道变化
 *   V2 零点准不准       → 零电流时三路原码应≈2048
 *   V3 标定准不准       → 已知电流下算出的 A 值应与实测吻合
 *   V4 序列顺序对不对   → 原码与通道号的对应关系不能错位
 *
 * 【为什么 V4 单列一条】
 * 三路扫描 + DMA 最容易错的就是「顺序」。若 CH3/CH4/CH5 的
 * 转换顺序与数组下标错位，表现为「给 W 相加电流，读数却出现在
 * V 相位置」。这种错误在静态零点测试中完全看不出来（三路都是
 * 2048），只有分别给单路加电流才能暴露。
 *
 * 【命令清单】
 *   t_iadc              单次打印三路 raw / mV / A
 *   t_iadc_watch [n] [ms]  连续打印，手给电流看响应
 *   t_iadc_stat         打印 conv_cnt / err_cnt / sat_cnt
 *   t_iadc_cal          辅助标定：提示操作步骤并实时显示
 * ============================================================
 */

#include <rtthread.h>
#include <finsh.h>
#include <stdlib.h>
#include <string.h>          /* strcmp：t_iadc_cal 子命令解析 */

/* 【2026-11-05】t_iadc_stat 新增寄存器级诊断，直接读 CM_ADC1 / CM_DMA1 /
 * CM_PWC，需要 DDL 的寄存器定义。bsp_adc_foc.h 只引 rtthread/stdint，
 * 不含器件头，故此处显式引入。 */
#include "hc32_ll.h"

#include "motor/drv/bsp_adc_foc.h"

/* 【2026-11-10 彩标 P4】只给命令结论行上色（Q4 裁定），
 * 数据表与表头保持单色。*/
#include "../../log/log_color.h"
#include "mode/sys_caps.h"
/* 【2026-11-09】t_iadc_cal 需要读写 cur_zero_* 配置项 */
#include "config/sys_config.h"

/* raw → mV（整数，四舍五入） */
static uint32_t raw_to_mv(uint16_t raw)
{
    return ((uint32_t)raw * FOC_ADC_VREF_MV + FOC_ADC_FULL_SCALE / 2U) /
           FOC_ADC_FULL_SCALE;
}

/* 0.1A 单位转可读字符串（避免浮点） */
static void print_a10(const char *tag, int16_t a10)
{
    int32_t v = a10;
    const char *sign = "";

    if (v < 0)
    {
        sign = "-";
        v = -v;
    }
    rt_kprintf("  %s=%s%d.%d A", tag, sign, (int)(v / 10), (int)(v % 10));
}

/* ============================================================
 * t_iadc
 * ------------------------------------------------------------
 * 单次快照。列宽固定，便于纵向对比。
 * ============================================================ */
static void cmd_iadc(int argc, char **argv)
{
    foc_iabc_t v;
    uint32_t   seq0;

    (void)argc; (void)argv;

    if (sys_cap_require(SYS_CAP_DIAG_READ, "t_iadc") == 0U) { return; }

    if (foc_adc_read(&v) != 0)
    {
        LOG_ERR("[t_iadc] FAIL: foc_adc not initialized\n");
        return;
    }

    seq0 = v.seq;

    rt_kprintf("[t_iadc] snapshot seq=%u\n", (unsigned)seq0);
    rt_kprintf("  ch  signal        raw    mV    dev   A(0.1)\n");
    rt_kprintf("  %-3u %-12s %5u  %4u  %+5d   %+6d\n",
               (unsigned)FOC_ADC_CH_IBUS, "IBUS(TOTAL)",
               v.raw_bus, (unsigned)raw_to_mv(v.raw_bus),
               (int)v.raw_bus - (int)FOC_CUR_ZERO_RAW,
               (int)v.i_bus_a10);
    rt_kprintf("  %-3u %-12s %5u  %4u  %+5d   %+6d\n",
               (unsigned)FOC_ADC_CH_IW, "IW",
               v.raw_w, (unsigned)raw_to_mv(v.raw_w),
               (int)v.raw_w - (int)FOC_CUR_ZERO_RAW,
               (int)v.i_w_a10);
    rt_kprintf("  %-3u %-12s %5u  %4u  %+5d   %+6d\n",
               (unsigned)FOC_ADC_CH_IV, "IV",
               v.raw_v, (unsigned)raw_to_mv(v.raw_v),
               (int)v.raw_v - (int)FOC_CUR_ZERO_RAW,
               (int)v.i_v_a10);
    rt_kprintf("  --  %-12s %5s  %4s  %5s   %+6d  (iu=-iv-iw)\n",
               "IU(recon)", "-", "-", "-", (int)v.i_u_a10);

    /* ============================================================
     * 三相电流和校验：iu + iv + iw 必须为 0
     * ------------------------------------------------------------
     * 【2026-11-09 新增】这是判断"符号是否正确"的唯一硬指标。
     *
     *   星形连接无中线 ⇒ ia+ib+ic = 0（基尔霍夫）。
     *   IU 是由 -(IV+IW) 重构的，所以 iu+iv+iw 恒等于 0
     *   **只要符号正确**。若数据层误取绝对值，这个和会明显偏离 0。
     *
     * 判据：
     *   |sum| <= 2   → 符号正确，重构自洽
     *   |sum| 较大  → 符号/零点有问题
     *
     * 注意：这里只校验 IV/IW 采到的两路 + 重构的 IU，
     *       所以 sum 在数学上恒为 0（重构定义的必然结果）。
     *       真正有诊断价值的是 **IV + IW 的符号关系**：
     *       若两路同号且都很大，说明极性可能接反（星形连接下
     *       两相电流不可能长期同号同幅）。
     * ============================================================ */
    {
        int32_t sum = (int32_t)v.i_u_a10 + (int32_t)v.i_v_a10 +
                      (int32_t)v.i_w_a10;

        rt_kprintf("  sum(iu+iv+iw) = %+d (must be 0 by reconstruction)\n",
                   (int)sum);
        rt_kprintf("  iv+iw = %+d  %s\n",
                   (int)(v.i_v_a10 + v.i_w_a10),
                   ((v.i_v_a10 > 0) && (v.i_w_a10 > 0) &&
                    (v.i_v_a10 > 200) && (v.i_w_a10 > 200))
                       ? "<-- WARN: both large & same sign, check polarity"
                       : "");
    }

    /* 零点判据：1650mV/3300mV*4096 = 2048 */
    rt_kprintf("  zero expect raw=%u (1650mV)\n", (unsigned)FOC_CUR_ZERO_RAW);

    {
        int32_t db = (int32_t)v.raw_bus - (int32_t)FOC_CUR_ZERO_RAW;
        int32_t dw = (int32_t)v.raw_w   - (int32_t)FOC_CUR_ZERO_RAW;
        int32_t dv = (int32_t)v.raw_v   - (int32_t)FOC_CUR_ZERO_RAW;

        rt_kprintf("  zero dev(signed): bus=%+d w=%+d v=%+d LSB\n",
                   (int)db, (int)dw, (int)dv);
        rt_kprintf("  zero dev(abs)   : bus=%d w=%d v=%d LSB\n",
                   (int)((db < 0) ? -db : db),
                   (int)((dw < 0) ? -dw : dw),
                   (int)((dv < 0) ? -dv : dv));

        /* 200 LSB ≈ 65A（接近满量程 62.5A），必然异常。
         * 全用整数，Q-63（float 豁免）未批准前不得引入浮点。 */
        if ((db > 200) || (db < -200) ||
            (dw > 200) || (dw < -200) ||
            (dv > 200) || (dv < -200))
        {
            rt_kprintf("  WARN: |deviation| > 200 LSB (~65A), "
                       "check wiring/polarity\n");
        }
        else if ((db > 40) || (dw > 40) || (dv > 40))
        {
            /* 40 LSB ≈ 1.2A。三路零点离散属正常（霍尔典型 ±30mV），
             * 但超过这个量级建议做零点校准。 */
            rt_kprintf("  NOTE: zero offset > 40 LSB (~1.2A), "
                       "consider zero calibration\n");
        }
    }
}

/* ============================================================
 * t_iadc_watch [n] [ms]
 * ------------------------------------------------------------
 * 连续打印。手给某一相加电流，观察哪一路跟随，
 * 这是验证「通道映射 + 序列顺序」最直接的办法。
 * ============================================================ */
static void cmd_iadc_watch(int argc, char **argv)
{
    int n  = 20;
    int ms = 200;
    int k;
    foc_iabc_t v;

    if (sys_cap_require(SYS_CAP_DIAG_READ, "t_iadc_watch") == 0U) { return; }

    if (argc >= 2) { n  = atoi(argv[1]); }
    if (argc >= 3) { ms = atoi(argv[2]); }
    if (n <= 0)    { n  = 20; }
    if (n > 2000)  { n  = 2000; }
    if (ms < 0)    { ms = 0; }
    if (ms > 5000) { ms = 5000; }

    rt_kprintf("[t_iadc_watch] n=%d interval=%dms\n", n, ms);
    rt_kprintf("  k    seq    rawBus rawW  rawV   a10Bus a10W  a10V\n");

    for (k = 0; k < n; k++)
    {
        if (foc_adc_read(&v) != 0)
        {
            rt_kprintf("  read fail\n");
            return;
        }

        rt_kprintf("  %3d  %6u  %5u  %5u  %5u   %5d  %4d  %4d\n",
                   k, (unsigned)v.seq,
                   v.raw_bus, v.raw_w, v.raw_v,
                   (int)v.i_bus_a10, (int)v.i_w_a10, (int)v.i_v_a10);

        if (k < n - 1) { rt_thread_mdelay(ms); }
    }
}

/* ============================================================
 * t_iadc_stat
 * ------------------------------------------------------------
 * 采样健康度：
 *   conv_cnt 应随时间快速增长（20kHz 软件触发下取决于触发频率）
 *   err_cnt  必须为 0，非 0 说明 DMA 配置或 AOS 路由有问题
 *   sat_cnt  贴边计数，持续增长说明量程或极性异常
 * ============================================================ */
static void cmd_iadc_stat(int argc, char **argv)
{
    foc_adc_stat_t st;

    (void)argc; (void)argv;

    if (sys_cap_require(SYS_CAP_DIAG_READ, "t_iadc_stat") == 0U) { return; }

    foc_adc_get_stat(&st);

    rt_kprintf("[t_iadc_stat]\n");
    rt_kprintf("  conv_cnt = %u\n", (unsigned)st.conv_cnt);
    rt_kprintf("  err_cnt  = %u  %s\n", (unsigned)st.err_cnt,
               (st.err_cnt == 0U) ? "OK" : "ALARM: dma error");
    rt_kprintf("  sat_cnt  = %u  %s\n", (unsigned)st.sat_cnt,
               (st.sat_cnt == 0U) ? "OK" : "WARN: saturated samples");

    /* ============================================================
     * 【2026-11-05 新增：寄存器级诊断】
     * ------------------------------------------------------------
     * conv_cnt 恒 0 时，单看计数无法区分"时钟没开 / 通道没使能 /
     * 转换没启动 / DMA 没搬"这四种情况。下面 6 行直接读硬件状态，
     * 一次定位到底断在哪一环。
     *
     * 判据表：
     *   FCG0.DMA1=0 → DMA1 无时钟 ⇒ 写保护没解对（应=0 表示"时钟开"）
     *   FCG0.AOS =0 → AOS  无时钟 ⇒ 同上
     *   STR.STRT 反复 0     → ADC_Start 未生效或已自动清
     *   CHSELRA bit3/4/5 非全 1 → 通道未使能
     *   CCSR.EOCA 从不置 1  → 转换没发生（模拟侧/时钟问题）
     *   CHCTL2 EN=0         → DMA 通道被关
     *   RPT2 的 SRPT/DRPT=0 → 地址不会回卷，第 2 轮起读 DR6/7/8
     * ============================================================ */
    rt_kprintf("  ---- registers ----\n");
    rt_kprintf("  FCG0   = 0x%08X  (DMA1=%u AOS=%u, 1=clock OFF)\n",
               (unsigned)CM_PWC->FCG0,
               (unsigned)((CM_PWC->FCG0 >> 14U) & 1U),
               (unsigned)((CM_PWC->FCG0 >> 17U) & 1U));
    rt_kprintf("  ADC1 STR     = 0x%02X  (bit0 STRT=%u)\n",
               (unsigned)CM_ADC1->STR,
               (unsigned)(CM_ADC1->STR & 1U));
    rt_kprintf("  ADC1 CR0     = 0x%04X  (bit6 CLREN=%u)\n",
               (unsigned)CM_ADC1->CR0,
               (unsigned)((CM_ADC1->CR0 >> 6U) & 1U));
    rt_kprintf("  ADC1 CHSELRA = 0x%08X  (ch3=%u ch4=%u ch5=%u)\n",
               (unsigned)CM_ADC1->CHSELRA,
               (unsigned)((CM_ADC1->CHSELRA >> 3U) & 1U),
               (unsigned)((CM_ADC1->CHSELRA >> 4U) & 1U),
               (unsigned)((CM_ADC1->CHSELRA >> 5U) & 1U));
    rt_kprintf("  ADC1 TRGSR   = 0x%04X  (bit7 TRGENA=%u)\n",
               (unsigned)CM_ADC1->TRGSR,
               (unsigned)((CM_ADC1->TRGSR >> 7U) & 1U));
    rt_kprintf("  ADC1 ISR     = 0x%02X  (bit0 EOCA=%u)\n",
               (unsigned)CM_ADC1->ISR,
               (unsigned)(CM_ADC1->ISR & 1U));
    rt_kprintf("  DMA1 CHEN    = 0x%08X  (CH2=%u)\n",
               (unsigned)CM_DMA1->CHEN,
               (unsigned)((CM_DMA1->CHEN >> 2U) & 1U));
    rt_kprintf("  DMA1 RPT2    = 0x%08X\n", (unsigned)CM_DMA1->RPT2);
    rt_kprintf("  DMA1 DTCTL2  = 0x%08X  (BLKSIZE=%u CNT=%u)\n",
               (unsigned)CM_DMA1->DTCTL2,
               (unsigned)(CM_DMA1->DTCTL2 & 0x3FFU),
               (unsigned)((CM_DMA1->DTCTL2 >> 16U) & 0xFFFFU));
    rt_kprintf("  DMA1 INTSTAT1= 0x%08X  (BTC2=%u TC2=%u)\n",
               (unsigned)CM_DMA1->INTSTAT1,
               (unsigned)((CM_DMA1->INTSTAT1 >> 18U) & 1U),
               (unsigned)((CM_DMA1->INTSTAT1 >> 2U) & 1U));
    rt_kprintf("  DMA1 SAR2    = 0x%08X  (expect &ADC1->DR3 = 0x%08X)\n",
               (unsigned)CM_DMA1->SAR2,
               (unsigned)(uint32_t)(&(CM_ADC1->DR3)));

    if (st.conv_cnt == 0U)
    {
        rt_kprintf("  NOTE: conv_cnt=0 -> ADC never converted.\n");
        rt_kprintf("        看上面寄存器：\n");
        rt_kprintf("          FCG0 的 DMA1/AOS 位=1 → 时钟没开（写保护未解）\n");
        rt_kprintf("          CHSELRA ch3/4/5 非全 1 → 通道未使能\n");
        rt_kprintf("          STR.STRT 恒 0 且 ISR.EOCA 恒 0 → 序列未启动\n");
        rt_kprintf("          CHEN 的 CH2=0 → DMA 通道被关\n");
    }
    else if (st.conv_cnt < 100U)
    {
        rt_kprintf("  NOTE: conv_cnt small; if PWM already running for >1s,\n");
        rt_kprintf("        expected %u/s (20kHz). Check trigger hook.\n",
                   (unsigned)20000U);
    }
}

/* ============================================================
 * t_iadc_cal [begin|confirm|clear|show]
 * ------------------------------------------------------------
 * 【2026-11-09 升级为真正的零点校准命令】
 *
 * 零电流时三路 raw 各有偏移（实测 bus=2129 w=2141 v=2151，
 * 即 +81/+93/+103 LSB），必须校准，否则：
 *   - 恒定虚高 2.4~3.1A
 *   - 45A 堵转阈值实际在 42A 触发（偏松，有安全风险）
 *
 * 子命令：
 *   （无参）    显示当前零点配置与实时读数
 *   begin       开始采集（要求：母线无电流、电机未通电）
 *   confirm     把采集结果写入 sys_config 并落盘
 *   clear       清除校准标记，回到默认 2048
 *
 * ⚠️ 当前 foc_adc 内 FOC_ADC_ZERO_CAL_ENABLE = 0：
 *   校准值会被保存，但**暂不参与电流换算**。
 *   等用户完成相应模块后改宏为 1 即生效。
 *   这一点在 begin 时会明确提示，避免误以为已生效。
 * ============================================================ */
static void cmd_iadc_cal(int argc, char **argv)
{
    uint16_t z[3] = {0U, 0U, 0U};
    int      n    = 10;

    if (sys_cap_require(SYS_CAP_DIAG_READ, "t_iadc_cal") == 0U) { return; }

    /* ---------- 无参：显示当前状态 ---------- */
    if (argc < 2)
    {
        uint8_t valid = foc_adc_get_zero(z);

        rt_kprintf("[t_iadc_cal] zero cal status: %s\n",
                   (valid != 0U) ? "CALIBRATED" : "NOT CALIBRATED (using 2048)");
        rt_kprintf("  stored: bus=%u w=%u v=%u  (dev %+d %+d %+d LSB)\n",
                   (unsigned)z[0], (unsigned)z[1], (unsigned)z[2],
                   (int)z[0] - (int)FOC_CUR_ZERO_RAW,
                   (int)z[1] - (int)FOC_CUR_ZERO_RAW,
                   (int)z[2] - (int)FOC_CUR_ZERO_RAW);
        rt_kprintf("  effective in math: %s\n",
                   (FOC_ADC_ZERO_CAL_ENABLE != 0U) ? "YES" : "NO (macro=0)");
        rt_kprintf("  --- usage ---\n");
        rt_kprintf("    t_iadc_cal begin    start sampling (no current!)\n");
        rt_kprintf("    t_iadc_cal confirm  save to sys_config\n");
        rt_kprintf("    t_iadc_cal clear    reset to default 2048\n");
        rt_kprintf("    t_iadc_cal show <n>  live read n times\n");
        return;
    }

    /* ---------- begin ---------- */
    if (strcmp(argv[1], "begin") == 0)
    {
        if (sys_cap_require(SYS_CAP_DIAG_WRITE, "t_iadc_cal begin") == 0U) { return; }

        foc_adc_cal_begin();
        rt_kprintf("[t_iadc_cal] sampling started. "
                   "WAIT 1s then 't_iadc_cal confirm'\n");
        rt_kprintf("  REQUIREMENT: bus current must be ZERO (motor unpowered)\n");
        return;
    }

    /* ---------- confirm ---------- */
    if (strcmp(argv[1], "confirm") == 0)
    {
        uint32_t cnt;
        int ret;
        uint8_t i;

        if (sys_cap_require(SYS_CAP_DIAG_WRITE, "t_iadc_cal confirm") == 0U) { return; }

        cnt = foc_adc_cal_get_sample_count();
        rt_kprintf("[t_iadc_cal] samples collected: %u\n", (unsigned)cnt);

        if (foc_adc_cal_get_result(z) == 0U)
        {
            LOG_ERR("  FAIL: not enough samples (need >= 64). "
                       "Run 't_iadc_cal begin' first.\n");
            return;
        }

        rt_kprintf("  result: bus=%u w=%u v=%u  (dev %+d %+d %+d LSB)\n",
                   (unsigned)z[0], (unsigned)z[1], (unsigned)z[2],
                   (int)z[0] - (int)FOC_CUR_ZERO_RAW,
                   (int)z[1] - (int)FOC_CUR_ZERO_RAW,
                   (int)z[2] - (int)FOC_CUR_ZERO_RAW);

        /* 合理性检查：偏差过大说明当时不是零电流 */
        for (i = 0U; i < 3U; i++)
        {
            int32_t d = (int32_t)z[i] - (int32_t)FOC_CUR_ZERO_RAW;

            if ((d > 300) || (d < -300))
            {
                rt_kprintf("  WARN: ch%u deviation %+d LSB > 300 "
                           "(~9A), is current really zero?\n",
                           (unsigned)i, (int)d);
            }
        }

        ret = foc_adc_cal_commit(z);
        if (ret != 0)
        {
            LOG_ERR("  FAIL: commit returned %d (value out of range?)\n", ret);
            return;
        }

        rt_kprintf("  SAVED to sys_config (cur_zero_*_raw)\n");
        if (FOC_ADC_ZERO_CAL_ENABLE == 0U)
        {
            rt_kprintf("  NOTE: FOC_ADC_ZERO_CAL_ENABLE=0, value saved but\n");
            rt_kprintf("        NOT used in current math yet.\n");
            rt_kprintf("        Set the macro to 1U in bsp_adc_foc.h to enable.\n");
        }
        else
        {
            rt_kprintf("  ACTIVE now (takes effect on next read)\n");
        }
        return;
    }

    /* ---------- clear ---------- */
    if (strcmp(argv[1], "clear") == 0)
    {
        uint16_t def[3];
        int ret;

        if (sys_cap_require(SYS_CAP_DIAG_WRITE, "t_iadc_cal clear") == 0U) { return; }

        /* 清除 = 把三路零点写回默认 2048。
         * 不另设 valid 标志：三路都为 2048 即视为"未校准"
         * （判据与 foc_adc_load_zero 一致）。 */
        def[0] = FOC_CUR_ZERO_RAW;
        def[1] = FOC_CUR_ZERO_RAW;
        def[2] = FOC_CUR_ZERO_RAW;

        ret = foc_adc_cal_commit(def);
        if (ret == 0)
        {
            rt_kprintf("  CLEARED, back to default %u\n",
                       (unsigned)FOC_CUR_ZERO_RAW);
            rt_kprintf("  (three channels all == %u => treated as uncalibrated)\n",
                       (unsigned)FOC_CUR_ZERO_RAW);
        }
        else
        {
            LOG_ERR("  FAIL: %d\n", ret);
        }
        return;
    }

    /* ---------- show [n]：实时读数 ---------- */
    if (strcmp(argv[1], "show") == 0)
    {
        foc_iabc_t v;
        int k;

        if (argc >= 3) { n = atoi(argv[2]); }
        if ((n <= 0) || (n > 100)) { n = 10; }

        rt_kprintf("[t_iadc_cal] live read %d times, 300ms apart\n", n);
        rt_kprintf("  target zero raw=%u\n", (unsigned)FOC_CUR_ZERO_RAW);

        for (k = 0; k < n; k++)
        {
            if (foc_adc_read(&v) != 0) { return; }

            rt_kprintf("  raw[%4u,%4u,%4u]  dev[%+5d,%+5d,%+5d]  "
                       "A[%+d,%+d,%+d]\n",
                       v.raw_bus, v.raw_w, v.raw_v,
                       (int)v.raw_bus - (int)FOC_CUR_ZERO_RAW,
                       (int)v.raw_w   - (int)FOC_CUR_ZERO_RAW,
                       (int)v.raw_v   - (int)FOC_CUR_ZERO_RAW,
                       (int)v.i_bus_a10, (int)v.i_w_a10, (int)v.i_v_a10);

            if (k < n - 1) { rt_thread_mdelay(300); }
        }
        return;
    }

    rt_kprintf("[t_iadc_cal] usage: t_iadc_cal [begin|confirm|clear|show <n>]\n");
}

MSH_CMD_EXPORT_ALIAS(cmd_iadc,       t_iadc,       read 3-phase current snapshot);
MSH_CMD_EXPORT_ALIAS(cmd_iadc_watch, t_iadc_watch, watch current continuously);
MSH_CMD_EXPORT_ALIAS(cmd_iadc_stat,  t_iadc_stat,  dma sampling statistics);
MSH_CMD_EXPORT_ALIAS(cmd_iadc_cal,   t_iadc_cal,   calibration helper);