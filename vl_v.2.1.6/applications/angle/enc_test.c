/**
 * @file    enc_test.c
 * @brief   三编码器物理链路诊断命令（B1 批次，2026-10-28）
 *
 * ============================================================
 * 【为什么单独一个文件，而不放进 app_test.c】
 * ------------------------------------------------------------
 * bsp_dev_kth7812.h 里 `#include "app.h"` 会拉进遗留的大头文件，
 * 而 app.h 与 motor/motor.h 同时定义了 MOTOR_STATE_FAULT
 * （app.h:393 与 motor/motor.h:54），两者共存即编译报
 * "redeclaration of enumerator"。app_test.c 已经包含 motor/motor.h，
 * 再引 bsp_dev_kth7812.h 必然冲突。
 *
 * 这是 app.h 遗留定义与新 motor 架构并存的既有问题，不属于 B1 范围，
 * 本轮不去动它（动 app.h 影响面太大）。故把编码器诊断隔离到独立
 * 编译单元：本文件只包含 bsp_dev_kth7812.h 与 sys_caps.h，
 * 不碰 motor.h，从根上避开冲突。
 *
 * ============================================================
 * 【这组命令在 B1 中的作用】
 * 打通 SPI + CRC 之后，必须有手段确认三颗编码器真的在读数、
 * 数值随手动转动变化、CRC 通过率达标，才允许进入 B2 解算层。
 * 没有这组命令，B1 无法验收。
 * ============================================================
 */

#include <rtthread.h>
#include <finsh.h>
#include <stdlib.h>
#include <string.h>          /* strcmp：t_enc_wr 的 confirm 参数判断 */

#include "bsp_dev_kth7812.h"
#include "mode/sys_caps.h"
#include "mech_cfg_store.h"              /* enc_name/ratio 提示从 mech_cfg 取齿数 */
#include "../sys_param/sys_param.h"     /* t_enc_cal 需要读写零点 */

/* CRC 实现与解码函数在 bsp_dev_kth7812.c 内为 static，
 * 本文件需要独立的 CRC 计算来对照"接收值 vs 计算值"，
 * 故在此重新实现一份等价逻辑，专门用于诊断对照。
 *
 * 【重要】这**不是**重复实现导致的双份维护风险，而是刻意的
 * 交叉验证：诊断命令算出的 CRC 与驱动内部算出的 CRC 若不一致，
 * 说明两边有一处写错了。若强行共享同一份代码，就失去了这个校验能力。
 * 因此本文件保留独立副本，并在注释中标明来源。
 */
static uint8_t dbg_crc4_itu_12bit(uint16_t angle12)
{
    uint8_t crc = 0x00U;
    int8_t  i;

    for (i = 11; i >= 0; i--)
    {
        uint8_t bit = (uint8_t)((angle12 >> i) & 0x01U);
        uint8_t top = (uint8_t)((crc >> 3) & 0x01U);

        crc = (uint8_t)((crc << 1) & 0x0FU);
        if ((top ^ bit) != 0U)
        {
            crc ^= 0x03U;
        }
    }

    /* refout = true：4bit 输出反射 */
    return (uint8_t)(((crc & 0x01U) << 3) |
                     ((crc & 0x02U) << 1) |
                     ((crc & 0x04U) >> 1) |
                     ((crc & 0x08U) >> 3));
}

static uint16_t dbg_decode(uint16_t raw16)
{
    uint16_t angle12  = (uint16_t)((raw16 >> 4) & 0x0FFFU);
    uint8_t  crc_recv = (uint8_t)(raw16 & 0x0FU);

    return (dbg_crc4_itu_12bit(angle12) == crc_recv) ? angle12 : 0xFFFFU;
}

/* 设备编号 → 名称
 * 【2026-11-03】齿数不再硬编码。原来写死 "Z0(41)/Z1(42)/Z2(43)"，
 * 一旦用 t_mech_cand 切换齿型，打印出来的齿数就和实际解算用的不一致，
 * 判读时会被误导 —— 这类"标签与实现漂移"正是 C-05 那类静默错误的根源。
 * 改为从 mech_cfg 的当前生效配置取齿数；未加载时回落为设备号。 */
static const char *s_enc_name_cache[3] = { "Z0", "Z1", "Z2" };
static char s_enc_name_buf[3][12];

/* 用当前齿数刷新缓存。调用时机：每次进入命令前刷新一次即可。
 * 注意：齿数在 g_mech.cfg（原始配置），不在 mech_rt_t（只有派生量）。 */
static void enc_name_refresh(void)
{
    uint8_t i;

    if (g_mech.is_loaded == 0U)
    {
        for (i = 0U; i < 3U; i++)
        {
            (void)rt_snprintf(s_enc_name_buf[i], sizeof(s_enc_name_buf[i]),
                              "Z%u", (unsigned)i);
            s_enc_name_cache[i] = s_enc_name_buf[i];
        }
        return;
    }

    (void)rt_snprintf(s_enc_name_buf[0], sizeof(s_enc_name_buf[0]),
                      "Z0(%u)", (unsigned)g_mech.cfg.enc_z01_num);
    (void)rt_snprintf(s_enc_name_buf[1], sizeof(s_enc_name_buf[1]),
                      "Z1(%u)", (unsigned)g_mech.cfg.enc_z01_den);
    (void)rt_snprintf(s_enc_name_buf[2], sizeof(s_enc_name_buf[2]),
                      "Z2(%u)", (unsigned)g_mech.cfg.enc_z02_den);

    for (i = 0U; i < 3U; i++)
    {
        s_enc_name_cache[i] = s_enc_name_buf[i];
    }
}

static const char *enc_name(uint8_t idx)
{
    if (idx > 2U) { return "??"; }
    return s_enc_name_cache[idx];
}

/* ============================================================
 * t_enc_raw [次数]
 * ------------------------------------------------------------
 * 逐颗打印：原始帧 / 角度 / 接收CRC / 计算CRC / 判定
 * 必须用 Raw 版本读取 —— 若用已校验版本，就无法区分
 * "数据读错了" 和 "校验没通过"，排障时会误判方向。
 * ============================================================ */
static void cmd_enc_raw(int argc, char **argv)
{
    int n = 1;
    int k;

    if (sys_cap_require(SYS_CAP_DIAG_READ, "t_enc_raw") == 0U) { return; }

    enc_name_refresh();   /* 齿数标签随 mech_cfg 刷新 */

    if (argc >= 2) { n = atoi(argv[1]); }
    if (n <= 0)    { n = 1; }
    if (n > 500)   { n = 500; }          /* 防止刷屏把控制台卡死 */

    rt_kprintf("[t_enc_raw] loop=%d\n", n);
    rt_kprintf("  dev     raw16   ang  crc    calc  judge\n");

    for (k = 0; k < n; k++)
    {
        uint8_t i;

        for (i = 0; i < (uint8_t)KTH7812_DEV_MAX; i++)
        {
            uint16_t raw16    = KTH78_ReadAngleRaw((KTH7812_DEV_DEF)i);
            uint16_t angle12  = (uint16_t)((raw16 >> 4) & 0x0FFFU);
            uint8_t  crc_recv = (uint8_t)(raw16 & 0x0FU);
            uint8_t  crc_calc = dbg_crc4_itu_12bit(angle12);
            uint16_t decoded  = dbg_decode(raw16);
            const char *judge;

            if (raw16 == 0xFFFFU)
            {
                /* 传输层失败（KTH78_SpiXfer16 的显式失败哨兵），
                 * 或 MISO 悬空被上拉。两种情况都不能当"数据"看。 */
                judge = "NO-XFER";
            }
            else if (raw16 == 0x0000U)
            {
                /* 【关键】0x0000 帧的 CRC 天然自洽，decoded 会给出 "ok"，
                 * 这是假阳性。必须单独标出，否则硬件故障看起来像正常数据。
                 * 2026-10-29 MOSI/MISO 交叉事故正是栽在这里。 */
                judge = "ALLZERO?";
            }
            else
            {
                judge = (decoded == 0xFFFFU) ? "FAIL" : "ok";
            }

            rt_kprintf("  %-7s 0x%04X %4u  %X/%X  %s\n",
                       enc_name(i), raw16, angle12,
                       crc_recv, crc_calc, judge);
        }

        if (n > 1) { rt_thread_mdelay(50); }
    }

    /* 传输层计数：把"物理层失败"与"数据为 0"彻底分开 */
    {
        uint32_t xok = 0U, xerr = 0U;
        KTH78_GetXferStat(&xok, &xerr);
        rt_kprintf("[t_enc_raw] spi xfer: ok=%u err=%u %s\n",
                   (unsigned)xok, (unsigned)xerr,
                   (xerr == 0U) ? "" : "<< ALARM: transfer failed, check wiring");
    }
    rt_kprintf("[t_enc_raw] note: ALLZERO? means frame==0, its CRC passes by nature\n");
}

/* ============================================================
 * t_enc_crc [次数]
 * ------------------------------------------------------------
 * 只统计通过率，不逐帧刷屏。B1 验收要求静止时 100%。
 * ============================================================ */
static void cmd_enc_crc(int argc, char **argv)
{
    int n = 100;
    int k;
    uint32_t total = 0;
    uint32_t fail[3] = {0, 0, 0};
    const char *dev[3];

    if (sys_cap_require(SYS_CAP_DIAG_READ, "t_enc_crc") == 0U) { return; }

    enc_name_refresh();   /* 齿数标签随 mech_cfg 刷新 */
    dev[0] = enc_name(0U);
    dev[1] = enc_name(1U);
    dev[2] = enc_name(2U);

    enc_name_refresh();   /* 齿数标签随 mech_cfg 刷新 */

    if (argc >= 2) { n = atoi(argv[1]); }
    if (n <= 0)     { n = 1; }
    if (n > 10000)  { n = 10000; }

    for (k = 0; k < n; k++)
    {
        uint8_t i;
        for (i = 0; i < (uint8_t)KTH7812_DEV_MAX; i++)
        {
            if (KTH78_ReadAngle((KTH7812_DEV_DEF)i) == KTH7812_ANGLE_INVALID)
            {
                fail[i]++;
            }
            total++;
        }
    }

    rt_kprintf("[t_enc_crc] samples=%u (per device %d)\n", (unsigned)total, n);

    {
        uint8_t i;
        for (i = 0; i < (uint8_t)KTH7812_DEV_MAX; i++)
        {
            uint32_t ok = (uint32_t)n - fail[i];
            rt_kprintf("  %s pass=%u/%d  fail=%u  %s\n",
                       dev[i], (unsigned)ok, n, (unsigned)fail[i],
                       (fail[i] == 0U) ? "PERFECT" : "CHECK");
        }
    }

    rt_kprintf("[t_enc_crc] criteria: static expect 100%%; <99.9%% check SPI/CS\n");
}

/* ============================================================
 * t_enc_crc_selftest
 * ------------------------------------------------------------
 * 与硬件无关的反证实验：证明 CRC 实现真的能抓错。
 * 若单比特检出率不是 100%，则任何"CRC 通过"的结论都不可信 ——
 * 这正是旧 CalCRC4() 恒返回 0 时的情形（形同虚设）。
 * ============================================================ */
static void cmd_enc_crc_selftest(int argc, char **argv)
{
    uint32_t a;
    uint32_t single_total = 0, single_det = 0;
    uint32_t double_total = 0, double_det = 0;

    (void)argc; (void)argv;

    if (sys_cap_require(SYS_CAP_DIAG_READ, "t_enc_crc_st") == 0U) { return; }

    enc_name_refresh();   /* 齿数标签随 mech_cfg 刷新 */

    /* 1) 全 4096 点合法帧必须能解回原值 */
    for (a = 0; a < 4096U; a++)
    {
        uint16_t frame = (uint16_t)((a << 4) | dbg_crc4_itu_12bit((uint16_t)a));
        if (dbg_decode(frame) != (uint16_t)a)
        {
            rt_kprintf("[selftest] FAIL: angle=%u cannot decode back\n", (unsigned)a);
            return;
        }
    }
    rt_kprintf("[selftest] 4096-point roundtrip ......... OK\n");

    /* 2) 规格书算例 0x0FF -> 0x2 -> 帧 0x0FF2 */
    {
        uint16_t c = dbg_crc4_itu_12bit(0x0FFU);
        uint16_t f = (uint16_t)((0x0FFU << 4) | c);
        rt_kprintf("[selftest] datasheet case 0x0FF -> 0x%04X ... %s\n",
                   f, (f == 0x0FF2U) ? "OK" : "FAIL");
        if (f != 0x0FF2U) { return; }
    }

    /* 3) 单比特翻转：CRC-4 理论必须 100% 检出 */
    for (a = 0; a < 4096U; a += 3U)
    {
        uint16_t good = (uint16_t)((a << 4) | dbg_crc4_itu_12bit((uint16_t)a));
        uint8_t  b;

        for (b = 0; b < 16U; b++)
        {
            uint16_t bad = (uint16_t)(good ^ (uint16_t)(1U << b));
            single_total++;
            if (dbg_decode(bad) == 0xFFFFU) { single_det++; }
        }
    }
    rt_kprintf("[selftest] single-bit flip detect %u/%u = %u.%02u%%  %s\n",
               (unsigned)single_det, (unsigned)single_total,
               (unsigned)(single_total ? (single_det * 100U / single_total) : 0U),
               (unsigned)(single_total ?
                   ((single_det * 10000U / single_total) % 100U) : 0U),
               (single_det == single_total) ? "OK" : "FAIL(impl wrong)");

    /* 4) 双比特翻转：CRC-4 理论上限，仅作参考不作判定 */
    for (a = 0; a < 4096U; a += 7U)
    {
        uint16_t good = (uint16_t)((a << 4) | dbg_crc4_itu_12bit((uint16_t)a));
        uint8_t  b1, b2;

        for (b1 = 0; b1 < 16U; b1 += 5U)
        {
            for (b2 = (uint8_t)(b1 + 1U); b2 < 16U; b2 += 5U)
            {
                uint16_t bad = (uint16_t)(good ^
                                          (uint16_t)(1U << b1) ^
                                          (uint16_t)(1U << b2));
                double_total++;
                if (dbg_decode(bad) == 0xFFFFU) { double_det++; }
            }
        }
    }
    rt_kprintf("[selftest] double-bit flip detect %u/%u = %u%% (ref)\n",
               (unsigned)double_det, (unsigned)double_total,
               (unsigned)(double_total ? (double_det * 100U / double_total) : 0U));

    rt_kprintf("[selftest] conclusion: %s\n",
               (single_det == single_total) ? "PASS" : "FAIL");
}

/* ============================================================
 * t_enc_reg <dev> <addr>
 * ------------------------------------------------------------
 * 读单个寄存器，打印完整 16bit 响应。
 *
 * 【这不是"验证 MGL/MGH 读不到"的结论工具，而是"把值取出来"的工具】
 * 用户 2026-10-28 指示：寄存器地址已给定，只管尝试读，由用户判定。
 * 因此本命令**不下任何结论**，只如实打印：
 *   resp16  完整响应
 *   hi      resp16 >> 8      （按现有 KTH78_ReadReg 的口径）
 *   lo      resp16 & 0xFF
 *
 * 判读要点（写给操作者，不是给程序）：
 *   0xFFFF  → MISO 悬空，器件未响应
 *   0x0000  → 从机未驱动总线（未供电 / CS 选错 / MISO 断）
 *   0xXX00  → 正常应答，XX 为寄存器值
 *   其他    → 需结合 datasheet 判读，不要自行假设
 * ============================================================ */
static void cmd_enc_reg(int argc, char **argv)
{
    uint8_t  dev;
    uint8_t  addr;
    uint16_t resp = 0;

    if (sys_cap_require(SYS_CAP_DIAG_READ, "t_enc_reg") == 0U) { return; }

    enc_name_refresh();   /* 齿数标签随 mech_cfg 刷新 */

    if (argc < 3)
    {
        rt_kprintf("usage: t_enc_reg <dev 0~2> <addr 0x00~0x3F>\n");
        rt_kprintf("  e.g. t_enc_reg 0 0x06      read MGH_MGL\n");
        rt_kprintf("       t_enc_reg 0 0x00      read Z7 (control/status)\n");
        return;
    }

    dev  = (uint8_t)atoi(argv[1]);
    addr = (uint8_t)strtoul(argv[2], NULL, 0);

    if (dev >= (uint8_t)KTH7812_DEV_MAX)
    {
        rt_kprintf("[t_enc_reg] dev out of range (0~%d)\n", (int)KTH7812_DEV_MAX - 1);
        return;
    }
    if (addr > 0x3FU)
    {
        rt_kprintf("[t_enc_reg] addr out of range, cmd frame has 6bit addr (0x00~0x3F)\n");
        return;
    }

    if (KTH78_ReadRegRaw((KTH7812_DEV_DEF)dev, addr, &resp) != 0)
    {
        rt_kprintf("[t_enc_reg] read failed (bad param)\n");
        return;
    }

    rt_kprintf("[t_enc_reg] dev=%s addr=0x%02X\n", enc_name(dev), addr);
    rt_kprintf("  resp16 = 0x%04X\n", resp);
    rt_kprintf("  hi     = 0x%02X   (what KTH78_ReadReg returns)\n",
               (unsigned)((resp >> 8) & 0xFFU));
    rt_kprintf("  lo     = 0x%02X\n", (unsigned)(resp & 0xFFU));

    if      (resp == 0xFFFFU) { rt_kprintf("  NOTE: frame all-1, MISO floating / device not responding\n"); }
    else if (resp == 0x0000U) { rt_kprintf("  NOTE: frame all-0, slave not driving bus (power/CS/MISO)\n"); }
    else                      { rt_kprintf("  NOTE: non-zero response, operator decides meaning\n"); }
}

/* ============================================================
 * t_enc_regscan <dev> [start] [end]
 * ------------------------------------------------------------
 * 遍历地址空间，打印每个地址的响应，找出"有反应的"地址。
 *
 * 【为什么要全扫，而不是只读 0x06】
 * 我此前断言"SPI 读不到 MGL/MGH"是推断越界（见 PLAN-FOC-V2.0 附录 B-2）。
 * 既然寄存器地址已给定，就不该替芯片下结论。全扫一遍，
 * 哪些地址有值、哪些恒 0、哪些恒 1，一目了然，由用户判定。
 *
 * 【安全边界】
 *   - 默认只扫 0x00~0x09（现有枚举覆盖范围），不碰 0x0A~0x3F 的未知区；
 *   - 只读不写，绝不写 MTP；
 *   - 每地址之间有 1ms 间隔，避免器件来不及响应。
 * ============================================================ */
static void cmd_enc_regscan(int argc, char **argv)
{
    uint8_t  dev;
    uint8_t  a;
    uint8_t  start = 0x00U;
    uint8_t  end   = 0x09U;
    uint32_t same_cnt = 0;
    uint16_t first = 0;
    uint8_t  first_a = 0;

    if (sys_cap_require(SYS_CAP_DIAG_READ, "t_enc_regscan") == 0U) { return; }

    enc_name_refresh();   /* 齿数标签随 mech_cfg 刷新 */

    if (argc < 2)
    {
        rt_kprintf("usage: t_enc_regscan <dev 0~2> [start] [end]\n");
        rt_kprintf("  default scan 0x00~0x09\n");
        return;
    }

    dev = (uint8_t)atoi(argv[1]);
    if (dev >= (uint8_t)KTH7812_DEV_MAX)
    {
        rt_kprintf("[t_enc_regscan] dev out of range (0~%d)\n", (int)KTH7812_DEV_MAX - 1);
        return;
    }
    if (argc >= 3) { start = (uint8_t)strtoul(argv[2], NULL, 0); }
    if (argc >= 4) { end   = (uint8_t)strtoul(argv[3], NULL, 0); }
    if (start > 0x3FU) { start = 0x00U; }
    if (end   > 0x3FU) { end   = 0x09U; }
    if (end < start)   { uint8_t t = start; start = end; end = t; }

    rt_kprintf("[t_enc_regscan] dev=%s  range 0x%02X~0x%02X\n",
               enc_name(dev), start, end);
    rt_kprintf("  addr  resp16   hi   lo   note\n");

    for (a = start; ; a++)
    {
        uint16_t resp = 0;

        if (KTH78_ReadRegRaw((KTH7812_DEV_DEF)dev, a, &resp) != 0)
        {
            rt_kprintf("  0x%02X  read err\n", a);
        }
        else
        {
            const char *note = "";
            if      (resp == 0xFFFFU) { note = "all-1 (no resp)"; }
            else if (resp == 0x0000U) { note = "all-0 (no drive)"; }

            rt_kprintf("  0x%02X  0x%04X  %02X   %02X   %s\n",
                       a, resp,
                       (unsigned)((resp >> 8) & 0xFFU),
                       (unsigned)(resp & 0xFFU), note);
        }

        if (a == end) { break; }
        rt_thread_mdelay(1);
    }

    /* 汇总：若全range 响应完全一致，就提示"可能整条链路不通"，
     * 避免把"所有寄存器都读到同一个值"误判成"每个寄存器都有内容"。 */
    {
        uint16_t r0 = 0;
        if (KTH78_ReadRegRaw((KTH7812_DEV_DEF)dev, start, &r0) == 0)
        {
            first = r0;
            first_a = start;
            for (a = start; ; a++)
            {
                uint16_t r = 0;
                if (KTH78_ReadRegRaw((KTH7812_DEV_DEF)dev, a, &r) == 0 && r == first)
                {
                    same_cnt++;
                }
                if (a == end) { break; }
                rt_thread_mdelay(1);
            }
            if (same_cnt == (uint32_t)(end - start + 1U))
            {
                rt_kprintf("[t_enc_regscan] WARN: all-range response is 0x%04X (same 0x%02X), "
                           "link may be down or device has no register read\n", first, first_a);
            }
        }
    }
}

/* ============================================================
 * t_enc_chk <dev> [n] [ms]
 * ------------------------------------------------------------
 * 角度正确性体检：回答"SPI 读回来的 12bit 到底是不是角度"。
 *
 * 【为什么 CRC 通过还不足以证明角度正确】
 *   CRC-4 只校验 [15:4] 与 [3:0] 的对应关系。以下错误它抓不到：
 *     1) 帧合法但内容不是"此刻的角度"（缓存滞后 / 读了别的通道）；
 *     2) 12bit 内部位序整体重排而 CRC 恰好也匹配（概率极低但非 0）；
 *     3) 某些 bit 恒 0 或恒 1（编码盘污损、磁铁偏心、位数配置错）。
 *   所以必须再补一层**物理层证据**：转一圈看位覆盖。
 *   这是唯一能不依赖芯片自述、直接判定"数据是否真是角度"的手段。
 *
 * 【四条判据】
 *   C1 通过率   静态应为 100%
 *   C2 重复性   静止时相邻两次差值应为 0 或 +-1 LSB
 *   C3 位覆盖   转满一圈后 or_acc 应为 0x0FFF、and_acc 应为 0x0000
 *               —— 哪个 bit 恒 0 / 恒 1 直接打印出来
 *   C4 步进     相邻采样差值超 2048 视为跳变（半圈），
 *               并按 1 圈/秒 给出量级参考值
 *
 * 【操作步骤】
 *   1) 手不动，跑 t_enc_chk 0 200 10   → 判 C1 C2
 *   2) 手匀速转满一圈，再跑同一条命令 → 判 C3 C4
 *   3) 三颗都过，才允许认为 SPI 角度通路正确
 *
 * 【为什么用 Raw 而不是 ReadAngle】
 *   用已校验版本会把"CRC 失败"压成 0xFFFF，与"角度恰好是 4095 附近"
 *   混在一起，无法分开统计。诊断必须看原始帧。
 * ============================================================ */
static int32_t dbg_wrap_diff(int32_t a, int32_t b)
{
    int32_t d = (a - b) % 4096;
    if (d >  2048) { d -= 4096; }
    if (d < -2048) { d += 4096; }
    return d;
}

static void cmd_enc_chk(int argc, char **argv)
{
    uint8_t  dev = 0U;
    int      n   = 200;
    int      ms  = 10;
    int      k;

    uint32_t ok = 0U, fail = 0U;
    int32_t  prev = -1;
    int32_t  min_a = 4096, max_a = -1;
    int32_t  max_step = 0;
    uint32_t or_acc  = 0x0000U;
    uint32_t and_acc = 0x0FFFU;
    uint32_t same_cnt = 0U;
    uint32_t bin[12];

    if (sys_cap_require(SYS_CAP_DIAG_READ, "t_enc_chk") == 0U) { return; }

    enc_name_refresh();   /* 齿数标签随 mech_cfg 刷新 */

    if (argc >= 2) { dev = (uint8_t)atoi(argv[1]); }
    if (argc >= 3) { n   = atoi(argv[2]); }
    if (argc >= 4) { ms  = atoi(argv[3]); }

    if (dev >= (uint8_t)KTH7812_DEV_MAX)
    {
        rt_kprintf("[t_enc_chk] dev out of range (0~%d)\n", (int)KTH7812_DEV_MAX - 1);
        return;
    }
    if (n  <= 0)   { n  = 200; }
    if (n  > 20000) { n = 20000; }
    if (ms < 0)    { ms = 0; }
    if (ms > 1000) { ms = 1000; }

    for (k = 0; k < 12; k++) { bin[k] = 0U; }

    rt_kprintf("[t_enc_chk] dev=%s n=%d interval=%dms\n", enc_name(dev), n, ms);
    rt_kprintf("  tip: keep still for C1/C2, then rotate one full turn for C3/C4\n");

    for (k = 0; k < n; k++)
    {
        uint16_t raw16 = KTH78_ReadAngleRaw((KTH7812_DEV_DEF)dev);
        uint16_t ang   = (uint16_t)((raw16 >> 4) & 0x0FFFU);
        uint8_t  crc_r = (uint8_t)(raw16 & 0x0FU);
        uint8_t  crc_c = dbg_crc4_itu_12bit(ang);

        if (crc_r != crc_c)
        {
            fail++;
        }
        else
        {
            int32_t a32 = (int32_t)ang;

            ok++;
            if (a32 < min_a) { min_a = a32; }
            if (a32 > max_a) { max_a = a32; }

            or_acc  |= (uint32_t)ang;
            and_acc &= (uint32_t)ang;
            bin[ang / 342U]++;

            if (prev >= 0)
            {
                int32_t d = dbg_wrap_diff(a32, prev);
                int32_t ad = (d < 0) ? -d : d;

                if (ad > max_step) { max_step = ad; }
                if (d == 0) { same_cnt++; }
            }
            prev = a32;
        }

        if (k < n - 1)
        {
            if (ms > 0) { rt_thread_mdelay(ms); }
        }
    }

    /* ---------------- 报告 ---------------- */
    rt_kprintf("  ---- result ----\n");
    rt_kprintf("  C1 pass=%u/%d fail=%u  %s\n",
               (unsigned)ok, n, (unsigned)fail,
               (fail == 0U) ? "OK" : "CRC-FAIL");

    if (ok == 0U)
    {
        rt_kprintf("  no valid frame at all: check power / CS / MISO / mode\n");
        return;
    }

    rt_kprintf("  C2 min=%d max=%d span=%d  same_cnt=%u\n",
               (int)min_a, (int)max_a, (int)(max_a - min_a), (unsigned)same_cnt);
    rt_kprintf("  C4 max_step=%d LSB  %s\n", (int)max_step,
               (max_step > 2048) ? "JUMP(>half turn, suspect bit error)" : "OK");

    /* C3：位覆盖 */
    {
        uint32_t stuck0 = 0x0FFFU & ~or_acc;
        uint32_t stuck1 = and_acc & 0x0FFFU;
        int b;
        int first = 1;

        rt_kprintf("  C3 or=0x%03X and=0x%03X\n", (unsigned)or_acc, (unsigned)and_acc);

        if (stuck0 != 0U)
        {
            rt_kprintf("     bit never set  : ");
            first = 1;
            for (b = 11; b >= 0; b--)
            {
                if (stuck0 & (1U << b))
                {
                    rt_kprintf("%s%d", first ? "" : ",", b);
                    first = 0;
                }
            }
            rt_kprintf("\n");
        }
        if (stuck1 != 0U)
        {
            rt_kprintf("     bit never clear: ");
            first = 1;
            for (b = 11; b >= 0; b--)
            {
                if (stuck1 & (1U << b))
                {
                    rt_kprintf("%s%d", first ? "" : ",", b);
                    first = 0;
                }
            }
            rt_kprintf("\n");
        }

        if ((stuck0 == 0U) && (stuck1 == 0U))
        {
            rt_kprintf("     full 12bit coverage: OK (all bits toggled)\n");
        }
        else if (max_a - min_a < 512)
        {
            rt_kprintf("     NOTE: span only %d LSB, did you rotate a full turn?\n",
                       (int)(max_a - min_a));
        }
        else
        {
            rt_kprintf("     NOTE: partial coverage, rotate a full turn to confirm\n");
        }
    }

    /* 12 扇区直方图：每扇区 342 LSB */
    {
        int b;
        rt_kprintf("  sector hist(342LSB each):");
        for (b = 0; b < 12; b++) { rt_kprintf(" %u", (unsigned)bin[b]); }
        rt_kprintf("\n");
    }

    rt_kprintf("  verdict: %s\n",
               ((fail == 0U) && (max_step <= 2048)) ? "link+frame OK, check C3 for bit health"
                                                    : "NOT OK");
}

/* ============================================================
 * t_enc_cmp <dev> [n]
 * ------------------------------------------------------------
 * 交叉验证：同一时刻用两条不同路径取值，比对是否一致。
 *   路径 A  角度命令（op=00, 全双工，MISO 返回角度帧）
 *   路径 B  寄存器 0x09 = RD，按 KTH78_ReadRegRaw 的读寄存器时序
 *
 * 【为什么这条命令"不下结论"】
 *   RD 寄存器的确切语义要看 datasheet，我不替芯片下判断。
 *   本命令只做三件事：
 *     1) 把两条路径的原始值并排打印；
 *     2) 按几种常见对齐方式各算一次差值，看哪种能对上；
 *     3) 若某种对齐长期差值为 0，即为该对齐成立的强证据。
 *   由操作者结合 datasheet 判定。
 * ============================================================ */
static void cmd_enc_cmp(int argc, char **argv)
{
    uint8_t  dev = 0U;
    int      n   = 20;
    int      k;
    uint32_t eq_frame = 0U, eq_hi = 0U, eq_hi12 = 0U, eq_lo = 0U;
    uint32_t valid = 0U;

    if (sys_cap_require(SYS_CAP_DIAG_READ, "t_enc_cmp") == 0U) { return; }

    enc_name_refresh();   /* 齿数标签随 mech_cfg 刷新 */

    if (argc >= 2) { dev = (uint8_t)atoi(argv[1]); }
    if (argc >= 3) { n   = atoi(argv[2]); }

    if (dev >= (uint8_t)KTH7812_DEV_MAX)
    {
        rt_kprintf("[t_enc_cmp] dev out of range (0~%d)\n", (int)KTH7812_DEV_MAX - 1);
        return;
    }
    if (n <= 0)  { n = 20; }
    if (n > 200) { n = 200; }

    rt_kprintf("[t_enc_cmp] dev=%s n=%d\n", enc_name(dev), n);
    rt_kprintf("  k  angleFrame  ang12   rdResp16  hi   lo\n");

    for (k = 0; k < n; k++)
    {
        uint16_t frame = KTH78_ReadAngleRaw((KTH7812_DEV_DEF)dev);
        uint16_t ang12 = (uint16_t)((frame >> 4) & 0x0FFFU);
        uint16_t resp  = 0U;
        uint8_t  hi    = 0U;
        uint8_t  lo    = 0U;

        if (KTH78_ReadRegRaw((KTH7812_DEV_DEF)dev, 0x09U, &resp) == 0)
        {
            hi = (uint8_t)((resp >> 8) & 0xFFU);
            lo = (uint8_t)(resp & 0xFFU);
            valid++;

            if (resp == frame)                  { eq_frame++; }
            if ((uint16_t)hi == ang12)          { eq_hi++;   }
            if ((uint16_t)((resp >> 4) & 0x0FFFU) == ang12) { eq_hi12++; }
            if ((uint16_t)lo == ang12)          { eq_lo++;   }
        }

        rt_kprintf("  %2d 0x%04X     %4u   0x%04X    %02X   %02X\n",
                   k, frame, ang12, resp, (unsigned)hi, (unsigned)lo);

        if (k < n - 1) { rt_thread_mdelay(20); }
    }

    rt_kprintf("  ---- match stats (n=%d) ----\n", n);
    rt_kprintf("    resp16 == frame       : %u\n", (unsigned)eq_frame);
    rt_kprintf("    resp>>4 &0xFFF == ang12: %u\n", (unsigned)eq_hi12);
    rt_kprintf("    hi8 == ang12          : %u\n", (unsigned)eq_hi);
    rt_kprintf("    lo8 == ang12          : %u\n", (unsigned)eq_lo);
    rt_kprintf("  NOTE: RD semantics per datasheet, operator decides\n");

    if (valid == 0U)
    {
        rt_kprintf("  WARN: register read returned nothing, RD path unusable\n");
    }
}

/* 寄存器地址 → 名称（仅用于打印，便于对照 datasheet） */
static const char *enc_reg_name(uint8_t addr)
{
    switch (addr)
    {
        case 0x00: return "Z7";
        case 0x01: return "Z15";
        case 0x02: return "GAIN_TRIM";
        case 0x03: return "Y_X_TRIM";
        case 0x04: return "PPT1_ZL_ZD";
        case 0x05: return "PPT9";
        case 0x06: return "MGH_MGL";
        case 0x07: return "NPP";
        case 0x08: return "ABZ_LIMIT";
        case 0x09: return "RD";
        default:   return "";
    }
}

/* ============================================================
 * t_enc_all [start] [end]
 * ------------------------------------------------------------
 * 【三颗一起读，并排对照】—— 本命令的唯一目的
 *
 * 单独读一颗只能知道"这个地址有值"，无法判断值是否合理。
 * 三颗是同一型号、同一批配置，把三颗并排放在一起，
 * 就能立刻看出哪颗异常：
 *   SAME   三颗一致  → 正常（配置类寄存器理应一致）
 *   DIFF   三颗不一致 → 若为配置类寄存器，则有一颗被改过/读错
 *   ----   三颗都是 0xFFFF → 该地址不存在或链路不通
 *   ZERO   三颗都是 0x0000 → 该地址未定义或从机未驱动
 *
 * 【为什么默认扫 0x00~0x0A】
 * 这是头文件 KTH7812_REG_ADD_DEF 枚举覆盖的范围，
 * 都是有名字的寄存器。要看更大空间用参数指定，最大 0x3F
 * （命令帧里地址域只有 6 bit）。
 *
 * 【安全边界】
 * 只读，绝不写。写 MTP 不可逆，诊断命令不碰。
 * ============================================================ */
static void cmd_enc_all(int argc, char **argv)
{
    uint8_t start = 0x00U;
    uint8_t end   = 0x0AU;
    uint8_t a;
    uint8_t d;
    uint32_t nff[3]  = {0U, 0U, 0U};   /* 每颗 all-FF 计数 */
    uint32_t nz[3]   = {0U, 0U, 0U};   /* 每颗 all-00 计数 */
    uint32_t same_all = 0U;
    uint32_t diff_cnt = 0U;

    if (sys_cap_require(SYS_CAP_DIAG_READ, "t_enc_all") == 0U) { return; }

    enc_name_refresh();   /* 齿数标签随 mech_cfg 刷新 */

    if (argc >= 2) { start = (uint8_t)strtoul(argv[1], NULL, 0); }
    if (argc >= 3) { end   = (uint8_t)strtoul(argv[2], NULL, 0); }
    if (start > 0x3FU) { start = 0x00U; }
    if (end   > 0x3FU) { end   = 0x0AU; }
    if (end < start)   { uint8_t t = start; start = end; end = t; }

    rt_kprintf("[t_enc_all] range 0x%02X~0x%02X, read ALL 3 devices\n", start, end);
    /* 【2026-11-03】表头齿数用 enc_name() 动态取，避免切齿型后标签与实际不符 */
    rt_kprintf("  addr name        %-7s %-7s %-7s  cmp\n",
               enc_name(0U), enc_name(1U), enc_name(2U));

    for (a = start; ; a++)
    {
        uint16_t v[3] = {0U, 0U, 0U};
        int      ok[3] = {0, 0, 0};

        for (d = 0U; d < (uint8_t)KTH7812_DEV_MAX; d++)
        {
            if (KTH78_ReadRegRaw((KTH7812_DEV_DEF)d, a, &v[d]) == 0)
            {
                ok[d] = 1;
                if (v[d] == 0xFFFFU) { nff[d]++; }
                if (v[d] == 0x0000U) { nz[d]++;  }
            }
        }

        if (ok[0] && ok[1] && ok[2])
        {
            const char *cmp;

            if ((v[0] == v[1]) && (v[1] == v[2]))
            {
                cmp = "SAME";
                same_all++;
                if (v[0] == 0xFFFFU) { cmp = "----"; }
                else if (v[0] == 0x0000U) { cmp = "ZERO"; }
            }
            else
            {
                cmp = "DIFF";
                diff_cnt++;
            }

            rt_kprintf("  0x%02X %-11s 0x%04X  0x%04X  0x%04X  %s\n",
                       a, enc_reg_name(a), v[0], v[1], v[2], cmp);
        }
        else
        {
            rt_kprintf("  0x%02X %-11s  have read error\n", a, enc_reg_name(a));
        }

        if (a == end) { break; }
        rt_thread_mdelay(1);
    }

    /* ---------------- 汇总 ---------------- */
    rt_kprintf("  ---- summary ----\n");
    rt_kprintf("    SAME=%u  DIFF=%u  total=%u\n",
               (unsigned)same_all, (unsigned)diff_cnt,
               (unsigned)(end - start + 1U));

    for (d = 0U; d < (uint8_t)KTH7812_DEV_MAX; d++)
    {
        rt_kprintf("    %s  all-FFFF=%u  all-0000=%u\n",
                   enc_name(d), (unsigned)nff[d], (unsigned)nz[d]);
    }

    /* ============================================================
     * 【2026-10-29 A-02】链路健康度判定
     * ------------------------------------------------------------
     * 2026-10-29 实测：MOSI/MISO 交叉，三颗全寄存器读出 0x0000，
     * 而 CRC 显示 "ok"（0x0000 帧的 CRC 天然自洽）。这是假阳性。
     *
     * 判据不要靠"值看起来对不对"，要靠结构特征：
     *   全部地址全 0  → 器件根本没驱动 MISO，或命令没送到
     *   全部地址全 1  → MISO 悬空被上拉
     *   传输错误计数>0 → SPI 物理层失败（时钟/CS/接线）
     * 三者任一命中，就明确告警，避免把故障当数据。
     * ============================================================ */
    {
        uint32_t xok = 0U, xerr = 0U;
        uint32_t total_addr = (uint32_t)(end - start + 1U);
        int      all_zero = 1;
        int      all_ff   = 1;

        KTH78_GetXferStat(&xok, &xerr);

        for (d = 0U; d < (uint8_t)KTH7812_DEV_MAX; d++)
        {
            if (nz[d]  != total_addr) { all_zero = 0; }
            if (nff[d] != total_addr) { all_ff   = 0; }
        }

        rt_kprintf("  ---- link health ----\n");
        rt_kprintf("    spi xfer ok=%u err=%u\n", (unsigned)xok, (unsigned)xerr);

        if (xerr > 0U)
        {
            rt_kprintf("    ALARM: SPI transfer layer FAILED (%u errors).\n", (unsigned)xerr);
            rt_kprintf("           check SCK/MOSI/MISO wiring, CS level, clock enable\n");
        }
        if (all_zero)
        {
            rt_kprintf("    ALARM: ALL devices returned 0x0000 on ALL addresses.\n");
            rt_kprintf("           This is NOT valid data. Most likely:\n");
            rt_kprintf("             - MOSI/MISO swapped (both ends driven wrong)\n");
            rt_kprintf("             - encoder not powered / CS not asserted\n");
            rt_kprintf("             - MISO not connected\n");
        }
        if (all_ff)
        {
            rt_kprintf("    ALARM: ALL devices returned 0xFFFF on ALL addresses.\n");
            rt_kprintf("           MISO floating / pulled up, slave not driving bus\n");
        }
        if ((xerr == 0U) && (all_zero == 0) && (all_ff == 0))
        {
            rt_kprintf("    LINK OK: responses vary, device is driving MISO\n");
        }
    }

    rt_kprintf("  ---- how to read ----\n");
    rt_kprintf("    config regs identical across 3 units is EXPECTED\n");
    rt_kprintf("    DIFF on config regs -> one unit differs, check it\n");
    rt_kprintf("    all-FFFF on an addr -> addr absent or link down\n");

    /* 附带打印三颗当前角度帧，作为"SPI 通路是否活着"的即时佐证 */
    rt_kprintf("  ---- live angle frame ----\n");
    for (d = 0U; d < (uint8_t)KTH7812_DEV_MAX; d++)
    {
        uint16_t raw16 = KTH78_ReadAngleRaw((KTH7812_DEV_DEF)d);
        uint16_t ang   = (uint16_t)((raw16 >> 4) & 0x0FFFU);
        uint8_t  crc_r = (uint8_t)(raw16 & 0x0FU);
        uint8_t  crc_c = dbg_crc4_itu_12bit(ang);

        rt_kprintf("    %s raw=0x%04X ang=%4u crc=%X/%X %s\n",
                   enc_name(d), raw16, ang, crc_r, crc_c,
                   (crc_r == crc_c) ? "ok" : "FAIL");
    }
}

/* ============================================================
 * t_enc_ang <dev> [n] [ms]
 * ------------------------------------------------------------
 * 只读指定那一颗编码器的角度值，不打扰另外两颗。
 *
 * 【为什么需要这条命令（现有命令都做不到）】
 *   t_enc_raw      一次读三颗，无法单独看某一颗
 *   t_enc_chk      只给 min/max/span 统计，不逐次打印角度
 *   t_enc_reg 0x09 读的是 RD 寄存器，不是角度帧
 *   t_ang_watch    是三颗游标联算的绝对角，需先 t_ang_solve
 * 逐颗单独验证时（比如换磁铁、查某颗装反），上面都不顺手。
 *
 * 【精度说明】
 * 12bit 角度 → 度数，用 x100 整数表示（两位小数，如 59.50）。
 * 不用浮点：Q-63（float 豁免）尚未批准，且 x100 已足够判读。
 *   1 LSB = 36000/4096 = 8.789 m度  (即 0.0088°)
 *   换算：deg_x100 = ang12 * 36000 / 4096
 *
 * 【判据标签沿用 t_enc_raw】
 *   NO-XFER   传输失败 / MISO 悬空上拉
 *   ALLZERO?  整帧全 0，CRC 会假阳性通过，必须单独标出
 * ============================================================ */
static void cmd_enc_ang(int argc, char **argv)
{
    uint8_t dev = 0U;
    int     n   = 1;
    int     ms  = 200;
    int     k;

    /* 降速排查用的统计计数器 */
    uint32_t stat_crcok  = 0U;
    uint32_t stat_crcfail = 0U;
    uint32_t stat_noxfer = 0U;
    uint32_t stat_zero   = 0U;

    if (sys_cap_require(SYS_CAP_DIAG_READ, "t_enc_ang") == 0U) { return; }

    enc_name_refresh();   /* 齿数标签随 mech_cfg 刷新 */

    if (argc < 2)
    {
        rt_kprintf("usage: t_enc_ang <dev 0~%d> [n] [interval_ms]\n",
                   (int)KTH7812_DEV_MAX - 1);
        rt_kprintf("  dev 0 = Z0(41/CS=PB15)  1 = Z1(42/CS=PA8)  2 = Z2(43/CS=PA9)\n");
        rt_kprintf("  e.g. t_enc_ang 0        read once\n");
        rt_kprintf("       t_enc_ang 0 20 100 read 20 times, 100ms apart\n");
        return;
    }

    dev = (uint8_t)atoi(argv[1]);
    if (argc >= 3) { n  = atoi(argv[2]); }
    if (argc >= 4) { ms = atoi(argv[3]); }

    if (dev >= (uint8_t)KTH7812_DEV_MAX)
    {
        rt_kprintf("[t_enc_ang] dev out of range (0~%d)\n", (int)KTH7812_DEV_MAX - 1);
        return;
    }
    if (n  <= 0)    { n  = 1; }
    if (n  > 20000) { n  = 20000; }
    if (ms < 0)     { ms = 0; }
    if (ms > 5000)  { ms = 5000; }

    rt_kprintf("[t_enc_ang] dev=%s n=%d interval=%dms\n", enc_name(dev), n, ms);

    if (n == 1)
    {
        rt_kprintf("  raw16    ang12   deg     crc    calc  judge\n");
    }
    else
    {
        rt_kprintf("  k    raw16    ang12   deg      crc    calc  delta  judge\n");
    }

    {
        int32_t prev_ang = -1;

        for (k = 0; k < n; k++)
        {
            uint16_t raw16   = KTH78_ReadAngleRaw((KTH7812_DEV_DEF)dev);
            uint16_t ang12   = (uint16_t)((raw16 >> 4) & 0x0FFFU);
            uint8_t  crc_r   = (uint8_t)(raw16 & 0x0FU);
            uint8_t  crc_c   = dbg_crc4_itu_12bit(ang12);
            const char *judge;

            /* deg_x100 = ang12 * 36000 / 4096，四舍五入 */
            uint32_t deg_x100 = ((uint32_t)ang12 * 36000U + 2048U) / 4096U;

            /* 注意：这里的 crc_r/crc_c 是"按 12bit+CRC4 假设"算出的对照值。
             * 2026-10-29 实测已确认芯片为 16bit 无 CRC 版本，故 CRC-FAIL
             * 是预期结果，不代表通信错误 —— 保留打印是为了让操作者自己
             * 核验"假设不成立"，而不是由程序替人下结论。 */
            if (raw16 == 0xFFFFU)
            {
                judge = "NO-XFER";
            }
            else if (raw16 == 0x0000U)
            {
                judge = "ALLZERO?";
            }
            else if (crc_r != crc_c)
            {
                judge = "CRC-FAIL";
            }
            else
            {
                judge = "ok";
            }

            if (n == 1)
            {
                rt_kprintf("  0x%04X   %4u  %3u.%02u  %X/%X  %s\n",
                           raw16, ang12,
                           (unsigned)(deg_x100 / 100U),
                           (unsigned)(deg_x100 % 100U),
                           crc_r, crc_c, judge);
            }
            else
            {
                /* delta：环绕归一后的差值，判断单调性用。
                 *
                 * 【2026-10-29 修复】原实现把 delta 计算绑在 ok 条件里，
                 * 而 ok 依赖 CRC 判定。实测发现芯片是 16bit 无 CRC 版本，
                 * CRC 恒失败 ⇒ ok 恒 0 ⇒ delta 恒 +0，完全失去意义。
                 *
                 * 现在解耦：delta 只用原始值，不看 CRC 结论。
                 * 理由：差值是几何量，与"帧校验是否通过"无关。
                 * 但 raw16 为 0xFFFF/0x0000 这两个哨兵值时不参与，
                 * 否则会把链路故障值当成真实角度算出巨大跳变。 */
                int32_t delta = 0;

                if ((raw16 != 0xFFFFU) && (raw16 != 0x0000U) && (prev_ang >= 0))
                {
                    delta = (int32_t)ang12 - prev_ang;
                    if (delta >  2048) { delta -= 4096; }
                    if (delta < -2048) { delta += 4096; }
                }

                /* 【2026-10-29】多行模式补上 crc 数值（recv/calc），
                 * 排版与单行模式一致，便于逐帧核验 CRC 实现是否匹配芯片。 */
                rt_kprintf("  %3d  0x%04X   %4u  %3u.%02u  %X/%X  %+5d  %s\n",
                           k, raw16, ang12,
                           (unsigned)(deg_x100 / 100U),
                           (unsigned)(deg_x100 % 100U),
                           crc_r, crc_c,
                           (int)delta, judge);
            }

            /* prev_ang 的更新不依赖 ok：
             * 只看是否拿到了"看起来像真实角度"的帧（非两个哨兵值）。
             * 这样即使 CRC 恒失败，delta 仍能反映角度变化趋势。 */
            if ((raw16 != 0xFFFFU) && (raw16 != 0x0000U))
            {
                prev_ang = (int32_t)ang12;
            }

            /* 统计：降速排查时需要量化对比 CRC 通过率 */
            if (raw16 == 0xFFFFU)      { stat_noxfer++; }
            else if (raw16 == 0x0000U) { stat_zero++;   }
            else if (crc_r == crc_c)   { stat_crcok++;  }
            else                       { stat_crcfail++; }

            if (k < n - 1)
            {
                if (ms > 0) { rt_thread_mdelay(ms); }
            }
        }
    }

    /* ---- CRC 通过率汇总（降速排查的关键量化指标）---- */
    if (n > 1)
    {
        uint32_t good = stat_crcok;
        uint32_t tot  = (uint32_t)n;
        /* 万分比，避免浮点（Q-63 未批准） */
        uint32_t ppm  = (tot > 0U) ? ((good * 10000U) / tot) : 0U;

        rt_kprintf("  ---- crc summary ----\n");
        rt_kprintf("    total=%u  crc_ok=%u  crc_fail=%u  no_xfer=%u  zero=%u\n",
                   (unsigned)tot, (unsigned)stat_crcok,
                   (unsigned)stat_crcfail,
                   (unsigned)stat_noxfer, (unsigned)stat_zero);
        rt_kprintf("    pass rate = %u.%02u%%\n",
                   (unsigned)(ppm / 100U), (unsigned)(ppm % 100U));

        if (ppm == 10000U)
        {
            rt_kprintf("    PERFECT: CRC always passes, comm is clean\n");
        }
        else if (ppm == 0U)
        {
            rt_kprintf("    ZERO pass: frame format or CRC impl mismatch, not a speed issue\n");
        }
        else
        {
            rt_kprintf("    PARTIAL pass: typical of signal integrity problem, lowering speed should help\n");
        }
    }

    rt_kprintf("[t_enc_ang] crc column = recv/calc under 12bit+CRC4 assumption\n");
    rt_kprintf("[t_enc_ang] if chip is 16bit no-CRC version, CRC-FAIL is EXPECTED\n");
    rt_kprintf("[t_enc_ang] 1 LSB(12bit) = 0.0879 deg, 1 LSB(16bit) = 0.00549 deg\n");
}

/* ============================================================
 * t_enc_wr <dev> <addr> <value> [confirm]
 * ------------------------------------------------------------
 * 写编码器寄存器（三颗均可单独指定）。
 *
 * ============================================================
 * 【这是全工程第一条会写 MTP 的命令，风险等级不同于只读命令】
 * ============================================================
 * MTP（多次可编程存储器）寿命约 1000 次。写超了芯片报废。
 * 因此默认**不写入**，只打印将要做什么 + 当前值 + 写后校验计划，
 * 必须显式带上第 4 个参数 confirm 才真正执行。
 *
 * 【典型用途：翻转旋转方向 RD 寄存器】
 * 手册 §8.1：
 *   寄存器地址 0x09，RD 占用 bit[7]
 *   RD = 1（出厂默认）：CW 顺时针转 → 角度增加
 *   RD = 0            ：CCW 逆时针转 → 角度增加（即方向翻转）
 *
 * Z0 与 Z1/Z2 是外啮合，转向相反。要让三颗方向统一，
 * 把 Z1/Z2 的 RD 置 0 即可（或按实测决定哪颗要翻）：
 *   t_enc_wr 1 0x09 0x00 confirm
 *   t_enc_wr 2 0x09 0x00 confirm
 *
 * 【为什么必须回读校验】
 * 写 MTP 可能因 WRDIS（写保护）已置位而静默失败。手册 §5.5：
 * 若 WRDIS = 1，芯片拒绝一切寄存器写操作。必须写后立刻回读，
 * 否则会误以为改成功，后面一直用错误配置。
 *
 * 【安全措施清单】
 *   1) 参数合法性校验（dev < 3，addr <= 0x3F，value <= 0xFF）
 *   2) 写前读出当前值并打印，供操作者确认这是预期改动
 *   3) 需 confirm 参数才执行
 *   4) 写后自动回读校验，不一致明确告警
 *   5) 提示 ABZ/UVW 输出方向会随 RD 一起改变
 * ============================================================ */
static void cmd_enc_wr(int argc, char **argv)
{
    uint8_t dev;
    uint8_t addr;
    uint8_t value;
    uint8_t before = 0U;
    uint8_t after  = 0U;
    uint16_t raw_before = 0U;
    uint16_t raw_after  = 0U;
    int     confirmed = 0;

    if (sys_cap_require(SYS_CAP_DIAG_WRITE, "t_enc_wr") == 0U) { return; }

    enc_name_refresh();   /* 齿数标签随 mech_cfg 刷新 */

    if (argc < 4)
    {
        rt_kprintf("usage: t_enc_wr <dev 0~2> <addr> <value> [confirm]\n");
        rt_kprintf("  WITHOUT 'confirm' this only PREVIEWS, does not write.\n");
        rt_kprintf("\n");
        rt_kprintf("  MTP write life is ~1000 cycles. Do NOT loop this command.\n");
        rt_kprintf("\n");
        rt_kprintf("  common: RD register (0x09 bit7) controls rotation direction\n");
        rt_kprintf("    RD=1 (default): CW rotate -> angle increases\n");
        rt_kprintf("    RD=0          : CCW rotate -> angle increases (flip)\n");
        rt_kprintf("  e.g.  t_enc_wr 1 0x09 0x80 confirm  <- RD=1 (CW->inc, default)\n");
        rt_kprintf("        t_enc_wr 1 0x09 0x00 confirm  <- RD=0 (CCW->inc, flip)\n");
        rt_kprintf("  value 0x80 sets bit7; 0x01 sets bit0 which is NOT RD\n");
        return;
    }

    dev   = (uint8_t)atoi(argv[1]);
    addr  = (uint8_t)strtoul(argv[2], NULL, 0);
    value = (uint8_t)strtoul(argv[3], NULL, 0);
    if (argc >= 5)
    {
        confirmed = (strcmp(argv[4], "confirm") == 0) ? 1 : 0;
    }

    /* ---- 参数校验 ---- */
    if (dev >= (uint8_t)KTH7812_DEV_MAX)
    {
        rt_kprintf("[t_enc_wr] dev out of range (0~%d)\n", (int)KTH7812_DEV_MAX - 1);
        return;
    }
    if (addr > 0x3FU)
    {
        rt_kprintf("[t_enc_wr] addr out of range (0~0x3F, 6bit)\n");
        return;
    }

    /* ---- 写前：读当前值 ---- */
    if (KTH78_ReadRegRaw((KTH7812_DEV_DEF)dev, addr, &raw_before) != 0)
    {
        rt_kprintf("[t_enc_wr] FAIL: cannot read current value, abort\n");
        return;
    }
    before = (uint8_t)((raw_before >> 8) & 0xFFU);

    rt_kprintf("[t_enc_wr] dev=%s addr=0x%02X\n", enc_name(dev), addr);
    rt_kprintf("  current raw16 = 0x%04X  ->  reg value = 0x%02X\n",
               raw_before, before);
    /* rt_kprintf 不支持 %b，这里用十六进制 + 单独标注 bit7（RD 位）*/
    rt_kprintf("  new     value = 0x%02X   bit7(RD)=%u\n",
               value, (unsigned)((value >> 7) & 1U));
    rt_kprintf("  changed bits  = 0x%02X\n", (uint8_t)(before ^ value));

    if (before == value)
    {
        rt_kprintf("  NOTE: value unchanged, nothing to write\n");
        return;
    }

    if (addr == 0x09U)
    {
        rt_kprintf("  >>> this is RD (rotation direction) register\n");
        rt_kprintf("      RD bit7: 1=CW increases, 0=CCW increases\n");
        rt_kprintf("      WARN: ABZ / UVW output directions change together\n");
    }

    if (!confirmed)
    {
        rt_kprintf("\n  PREVIEW ONLY - not written.\n");
        rt_kprintf("  To actually write, append 'confirm':\n");
        rt_kprintf("    t_enc_wr %u 0x%02X 0x%02X confirm\n",
                   (unsigned)dev, addr, value);
        return;
    }

    /* ---- 执行写入 ---- */
    rt_kprintf("\n  writing...\n");
    KTH78_WriteReg(addr, value, (KTH7812_DEV_DEF)dev);

    /* ---- 写后：回读校验 ---- */
    /* 必须校验：若 WRDIS=1，芯片拒绝写入，且不会有任何错误回报。
     * 不回读就会误以为成功。 */
    if (KTH78_ReadRegRaw((KTH7812_DEV_DEF)dev, addr, &raw_after) != 0)
    {
        rt_kprintf("  WARN: write sent but readback FAILED, state unknown\n");
        return;
    }
    after = (uint8_t)((raw_after >> 8) & 0xFFU);

    rt_kprintf("  readback raw16 = 0x%04X  ->  reg value = 0x%02X\n",
               raw_after, after);

    if (after == value)
    {
        rt_kprintf("  RESULT: OK, register updated\n");
    }
    else
    {
        rt_kprintf("  RESULT: FAIL, value not applied (expect 0x%02X got 0x%02X)\n",
                   value, after);
        rt_kprintf("  possible causes:\n");
        rt_kprintf("    - WRDIS (write disable) already set, need power cycle\n");
        rt_kprintf("    - MTP write cycle exhausted\n");
        rt_kprintf("    - SPI write command not accepted\n");
    }
}

/* ============================================================
 * t_enc_cal [confirm] / t_enc_cal clear
 * ------------------------------------------------------------
 * 回转臂机械零位的三颗编码器零点标定。
 *
 * ============================================================
 * 【业务语义（用户 2026-11-02 明确）】
 * ============================================================
 *   把回转臂移到**机械零位**（完全收回位置），执行本命令，
 *   记录此刻三颗编码器的角度作为零点基准。
 *   之后每次上电解算，三颗都会先减掉这个零点，
 *   使解算结果与机械位置严格对应。
 *
 * 【三条使用规则】
 *   1) t_enc_cal              仅预览，不写 flash
 *   2) t_enc_cal confirm      真正写入（置 enc_zero_valid + is_calibrated）
 *   3) t_enc_cal clear        清除零点并复位校准标记，允许重新校准
 *
 * 【为什么必须先 clear 再重新校准】
 *   校准成功后会置 is_calibrated=1，上层据此放行运动。
 *   若直接重复执行 confirm 而不 clear，虽然值会被覆盖，
 *   但"已校准"标志一直是 1 —— 中间掉电可能出现
 *   "零点已改但标志没变"的不一致状态。显式 clear 让状态机清晰。
 *   实际实现中 confirm 本身会重写全部三个值 + 两个标志，
 *   所以直接重校也是安全的；clear 主要用于"取消校准"场景。
 * ============================================================ */
static void cmd_enc_cal(int argc, char **argv)
{
    uint16_t cur[3] = {0U, 0U, 0U};
    uint16_t old[3] = {0U, 0U, 0U};
    uint8_t  old_valid;
    int      confirmed = 0;
    int      do_clear  = 0;
    uint8_t  i;

    if (sys_cap_require(SYS_CAP_DIAG_WRITE, "t_enc_cal") == 0U) { return; }

    enc_name_refresh();   /* 齿数标签随 mech_cfg 刷新 */

    if (argc >= 2)
    {
        if (strcmp(argv[1], "confirm") == 0) { confirmed = 1; }
        else if (strcmp(argv[1], "clear") == 0) { do_clear = 1; }
    }

    /* ---------- clear 分支 ---------- */
    if (do_clear)
    {
        sys_param_clear_enc_zero();
        rt_kprintf("[t_enc_cal] clear done, run 't_enc_cal confirm' to recalibrate\n");
        return;
    }

    /* ---------- 读当前三颗角度 ---------- */
    for (i = 0U; i < 3U; i++)
    {
        uint16_t raw = KTH78_ReadAngle((KTH7812_DEV_DEF)i);

        if (raw == KTH7812_ANGLE_INVALID)
        {
            rt_kprintf("[t_enc_cal] FAIL: %s read invalid (0xFFFF), abort\n",
                       enc_name(i));
            rt_kprintf("  check wiring / power before calibrating\n");
            return;
        }
        cur[i] = raw;
    }

    old_valid = sys_param_get_enc_zero(old);

    rt_kprintf("[t_enc_cal] current angles at mechanical zero:\n");
    /* 【2026-11-03】齿数用 enc_name() 动态取，不再硬编码 41/42/43 */
    rt_kprintf("  %s = %4u  (0x%03X)\n", enc_name(0U), (unsigned)cur[0], cur[0]);
    rt_kprintf("  %s = %4u  (0x%03X)\n", enc_name(1U), (unsigned)cur[1], cur[1]);
    rt_kprintf("  %s = %4u  (0x%03X)\n", enc_name(2U), (unsigned)cur[2], cur[2]);

    if (old_valid)
    {
        rt_kprintf("[t_enc_cal] previous zero: Z0=%u Z1=%u Z2=%u\n",
                   (unsigned)old[0], (unsigned)old[1], (unsigned)old[2]);
        rt_kprintf("  delta: %+d / %+d / %+d\n",
                   (int)cur[0] - (int)old[0],
                   (int)cur[1] - (int)old[1],
                   (int)cur[2] - (int)old[2]);
    }
    else
    {
        rt_kprintf("[t_enc_cal] not calibrated yet (no valid zero)\n");
    }

    if (!confirmed)
    {
        rt_kprintf("\n  PREVIEW ONLY - not saved.\n");
        rt_kprintf("  Make sure the arm is at MECHANICAL ZERO, then:\n");
        rt_kprintf("    t_enc_cal confirm\n");
        rt_kprintf("  To erase calibration: t_enc_cal clear\n");
        return;
    }

    /* ---------- 写入 flash ---------- */
    rt_kprintf("\n  saving to flash...\n");
    if (sys_param_set_enc_zero(cur[0], cur[1], cur[2]) != 0)
    {
        rt_kprintf("[t_enc_cal] FAIL: flash save error\n");
        return;
    }

    /* 回读校验：确认真的落盘了 */
    {
        uint16_t verify[3];

        if (sys_param_get_enc_zero(verify) == 0U)
        {
            rt_kprintf("[t_enc_cal] FAIL: verify readback says invalid\n");
            return;
        }
        if ((verify[0] != cur[0]) || (verify[1] != cur[1]) || (verify[2] != cur[2]))
        {
            rt_kprintf("[t_enc_cal] FAIL: verify mismatch %u/%u/%u\n",
                       (unsigned)verify[0], (unsigned)verify[1],
                       (unsigned)verify[2]);
            return;
        }
    }

    rt_kprintf("[t_enc_cal] OK, zero saved and verified\n");
    rt_kprintf("  now run 't_ang_solve' to build the vernier solution\n");
}

/* ============================================================
 * t_enc_tri : 三颗同步监控（角度 + 角度增量 + 同步性判定）
 * ------------------------------------------------------------
 * 【为什么需要这个命令】
 *   游标解算依赖三颗同时有效。一旦某颗不跟随转动（齿轮打滑、
 *   磁铁偏移、装配卡滞），解算就报 NO_MATCH，但从单颗读数上
 *   完全看不出来 —— 它只是"不变"，而"不变"也可能是真静止。
 *
 *   本命令把三颗的角度和**每帧增量**并排列出，一眼就能看出：
 *     - 哪些在动、哪些不动
 *     - 动的方向是否一致（同增同减）
 *     - 增量比例是否符合齿数比（41:42:43 应近似 1 : 41/42 : 41/43）
 *
 * 【用法】
 *   t_enc_tri              默认 20 次，间隔 200ms（约 4 秒）
 *   t_enc_tri 50 100       50 次，间隔 100ms
 *   手动转 Z0 的轴，观察三列是否同步变化
 *
 * 【判读】
 *   d0/d1/d2 全 0        -> 静止（正常，转起来再看）
 *   三者都非 0 且同向     -> 三颗都正常跟随
 *   某列恒 0 而其余在动   -> 该颗不跟随（机械/磁铁问题）
 *   某列方向与其余相反    -> RD 方向配置不一致
 * ============================================================ */
static void cmd_enc_tri(int argc, char **argv)
{
    int      n  = 20;
    int      ms = 200;
    int      k;
    int32_t  prev[3] = { -1, -1, -1 };
    uint32_t sum_abs[3] = { 0U, 0U, 0U };
    uint32_t moved[3]   = { 0U, 0U, 0U };   /* 该颗发生变化的帧数 */
    uint32_t frame_ok   = 0U;               /* 三颗都读到有效值的帧数 */

    if (sys_cap_require(SYS_CAP_DIAG_READ, "t_enc_tri") == 0U) { return; }

    enc_name_refresh();   /* 齿数标签随 mech_cfg 刷新 */

    if (argc >= 2) { n  = atoi(argv[1]); }
    if (argc >= 3) { ms = atoi(argv[2]); }
    if (n <= 0)    { n = 20; }

    /* 【2026-11-05】上限钳位时明确告知，不要静默修改用户输入。
     *
     * 实测踩坑：用户敲 `t_enc_tri 2000 1`，输出 "n=500"，
     * 但没有任何提示说明是钳位 —— 会误以为命令没生效或参数解析错。
     * 上限 500 的由来：每帧 3 次 SPI 事务 + 打印，1ms 间隔下
     * 500 帧约 0.5~1s 出完，再长会刷屏且肉眼跟不上。
     */
    if (n > 500)
    {
        rt_kprintf("[t_enc_tri] n=%d exceeds max 500, clamped to 500\n", n);
        n = 500;
    }
    if (ms < 0)    { ms = 0; }

    rt_kprintf("[t_enc_tri] n=%d interval=%dms\n", n, ms);
    rt_kprintf("  turn Z0 shaft by hand and watch whether all 3 follow\n");
    rt_kprintf("  k      Z0 ang  d0     Z1 ang  d1     Z2 ang  d2\n");

    for (k = 0; k < n; k++)
    {
        uint16_t now[3];
        int32_t  d[3];
        uint8_t  i;
        uint8_t  all_ok = 1U;

        for (i = 0U; i < 3U; i++)
        {
            now[i] = KTH78_ReadAngle((KTH7812_DEV_DEF)i);
            if (now[i] == KTH7812_ANGLE_INVALID)
            {
                all_ok = 0U;
            }
        }

        /* 增量：环绕归一（4096 一圈），首次只建基线 */
        for (i = 0U; i < 3U; i++)
        {
            if ((now[i] == KTH7812_ANGLE_INVALID) || (prev[i] < 0))
            {
                d[i] = 0;
            }
            else
            {
                int32_t raw_d = (int32_t)now[i] - prev[i];

                if (raw_d >  2048) { raw_d -= 4096; }
                if (raw_d < -2048) { raw_d += 4096; }
                d[i] = raw_d;

                if (raw_d != 0)
                {
                    moved[i]++;
                    sum_abs[i] += (uint32_t)((raw_d < 0) ? -raw_d : raw_d);
                }
            }
        }

        if (all_ok != 0U)
        {
            frame_ok++;
        }

        rt_kprintf("  %3d   %5d %+5d", k,
                   (now[0] == KTH7812_ANGLE_INVALID) ? -1 : (int)now[0],
                   (int)d[0]);
        rt_kprintf("   %5d %+5d",
                   (now[1] == KTH7812_ANGLE_INVALID) ? -1 : (int)now[1],
                   (int)d[1]);
        rt_kprintf("   %5d %+5d", 
                   (now[2] == KTH7812_ANGLE_INVALID) ? -1 : (int)now[2],
                   (int)d[2]);

        if (all_ok == 0U)
        {
            rt_kprintf("   <-- READ FAIL");
        }
        rt_kprintf("\n");

        for (i = 0U; i < 3U; i++)
        {
            prev[i] = (now[i] == KTH7812_ANGLE_INVALID) ? -1 : (int32_t)now[i];
        }

        if (k < n - 1) { rt_thread_mdelay(ms); }
    }

    /* ---- 汇总：哪颗在动，哪颗不动 ---- */
    rt_kprintf("  ---- summary ----\n");
    rt_kprintf("    frames=%u  all-3-readable=%u\n",
               (unsigned)n, (unsigned)frame_ok);

    for (k = 0; k < 3; k++)
    {
        /* moved[k] 是该颗"有变化"的帧数占比 */
        uint32_t pct = (n > 0) ? ((moved[k] * 100U) / (uint32_t)n) : 0U;

        rt_kprintf("    %s: moved %3u%% of frames, total travel %u LSB\n",
                   enc_name((uint8_t)k), (unsigned)pct, (unsigned)sum_abs[k]);
    }

    /* ---- 判定 ---- */
    rt_kprintf("  ---- verdict ----\n");

    if (frame_ok == 0U)
    {
        rt_kprintf("    no frame had all 3 readable, run 't_enc_raw 1' first\n");
    }
    else if ((moved[0] == 0U) && (moved[1] == 0U) && (moved[2] == 0U))
    {
        rt_kprintf("    all 3 static (no angle change). shaft not turned?\n");
        rt_kprintf("    turn the Z0 shaft while this command runs\n");
    }
    else
    {
        /* 以 Z0 为基准，比较另两颗是否跟随 */
        for (k = 1; k < 3; k++)
        {
            if ((moved[0] > 0U) && (moved[k] == 0U))
            {
                rt_kprintf("    >>> %s did NOT follow Z0 (static while Z0 moved)\n",
                           enc_name((uint8_t)k));
                rt_kprintf("        check: gear meshing / magnet position / shaft stuck\n");
            }
            else if ((moved[0] == 0U) && (moved[k] > 0U))
            {
                rt_kprintf("    >>> %s moved but Z0 did not (unexpected)\n",
                           enc_name((uint8_t)k));
            }
        }

        if ((moved[0] > 0U) && (moved[1] > 0U) && (moved[2] > 0U))
        {
            rt_kprintf("    all 3 follow the rotation\n");

            /* 【2026-11-03】期望比值改为从 mech_cfg 现算，不再写死 41/42/43。
             * 并且必须提示"间隔 >1ms 时比值不可信"——手转 80~226 LSB/ms，
             * 间隔 >=10ms 单帧就超 2048 半圈，发生角度混叠，比值失真
             * （曾因此在 100ms 组误判"Z2 不跟随"）。
             * 用约分后的 z01_p_num/p_den 算比值，避免大齿数时乘法溢出。 */
            {
                const mech_rt_t *rt = mech_get_rt();
                uint16_t n1 = g_mech.cfg.enc_z01_num;
                uint16_t d1 = g_mech.cfg.enc_z01_den;
                uint16_t n2 = g_mech.cfg.enc_z02_num;
                uint16_t d2 = g_mech.cfg.enc_z02_den;

                if ((rt != RT_NULL) && (g_mech.is_loaded != 0U))
                {
                    n1 = rt->z01_p_num; d1 = rt->z01_p_den;
                    n2 = rt->z02_p_num; d2 = rt->z02_p_den;

                    rt_kprintf("    expect ratio 1 : %u/%u : %u/%u  (Z0:Z1:Z2 = %u:%u:%u)\n",
                               (unsigned)n1, (unsigned)d1,
                               (unsigned)n2, (unsigned)d2,
                               (unsigned)g_mech.cfg.enc_z01_num,
                               (unsigned)g_mech.cfg.enc_z01_den,
                               (unsigned)g_mech.cfg.enc_z02_den);
                }
                else
                {
                    rt_kprintf("    expect ratio: mech_cfg not loaded, cannot compute\n");
                }
            }

            if (sum_abs[0] > 0U)
            {
                unsigned r1 = (unsigned)((uint64_t)sum_abs[1] * 1000U / sum_abs[0]);
                unsigned r2 = (unsigned)((uint64_t)sum_abs[2] * 1000U / sum_abs[0]);

                rt_kprintf("      Z1/Z0 = %u/1000   Z2/Z0 = %u/1000\n", r1, r2);

                /* 实测比对的定性结论，省去手工算 */
                if ((mech_get_rt() != RT_NULL) && (g_mech.is_loaded != 0U))
                {
                    const mech_rt_t *rt = mech_get_rt();
                    unsigned e1 = (unsigned)((uint64_t)rt->z01_p_num * 1000U
                                             / rt->z01_p_den);
                    unsigned e2 = (unsigned)((uint64_t)rt->z02_p_num * 1000U
                                             / rt->z02_p_den);
                    unsigned d1 = (r1 > e1) ? (r1 - e1) : (e1 - r1);
                    unsigned d2 = (r2 > e2) ? (r2 - e2) : (e2 - r2);

                    rt_kprintf("      deviation: Z1 %s%u/1000  Z2 %s%u/1000  (tol 10 = 1%%)\n",
                               (r1 >= e1) ? "+" : "-", d1,
                               (r2 >= e2) ? "+" : "-", d2);

                    if ((d1 > 30U) || (d2 > 30U))
                    {
                        rt_kprintf("      >>> ratio mismatch >3%%, encoder teeth config wrong?\n");
                        rt_kprintf("          run 't_mech_cand' to check active profile\n");
                        rt_kprintf("          NOTE: only trust this at interval<=1ms\n");
                    }
                }
            }

            if (ms > 1)
            {
                rt_kprintf("    WARN: interval=%dms too long, ratio may alias (>2048/4096)\n",
                           ms);
                rt_kprintf("          re-run with 't_enc_tri 500 1' for trustworthy ratio\n");
            }

            rt_kprintf("    now run 't_ang_solve' to solve absolute angle\n");
        }
    }
}

/* 【2026-11-05 修复：cmd_enc_raw 此前从未导出】
 *
 * 现象：cmd_enc_raw() 有完整实现（enc_test.c:129）、有 DIAG_READ 门禁，
 *       但**没有任何 MSH_CMD_EXPORT_ALIAS**，运行时报"命令未找到"。
 *       而 angle_cmd.c:264、enc_test.c:987/999/1605 三处提示文字
 *       仍在引导用户"run t_enc_raw 1"，形成死链。
 *
 * 为什么必须补：t_enc_raw 是一次性看三颗编码器原始帧的唯一入口，
 * 排查 ENC_Z1_FAIL / NO_MATCH 时第一步就要用它区分
 * "通信断（FFFF）" 还是 "数据真实但越界"。
 *
 * 放在这里而非列表最前，是为了与其它导出保持"按功能分组"的顺序。
 */
MSH_CMD_EXPORT_ALIAS(cmd_enc_raw,          t_enc_raw,   read raw frames of 3 encoders);
MSH_CMD_EXPORT_ALIAS(cmd_enc_tri,          t_enc_tri,   watch 3 encoders angle+delta);
MSH_CMD_EXPORT_ALIAS(cmd_enc_crc,          t_enc_crc,   encoder crc pass rate);
MSH_CMD_EXPORT_ALIAS(cmd_enc_crc_selftest, t_enc_crc_st, crc impl self test);
MSH_CMD_EXPORT_ALIAS(cmd_enc_reg,          t_enc_reg,   read one encoder register);
MSH_CMD_EXPORT_ALIAS(cmd_enc_regscan,      t_enc_regscan, scan encoder registers);
MSH_CMD_EXPORT_ALIAS(cmd_enc_chk,          t_enc_chk,   verify spi raw angle correctness);
MSH_CMD_EXPORT_ALIAS(cmd_enc_cmp,          t_enc_cmp,   cross check angle vs RD reg);
MSH_CMD_EXPORT_ALIAS(cmd_enc_all,          t_enc_all,   read ALL regs of ALL 3 encoders);
MSH_CMD_EXPORT_ALIAS(cmd_enc_ang,          t_enc_ang,   read angle of ONE encoder);
MSH_CMD_EXPORT_ALIAS(cmd_enc_wr,           t_enc_wr,    write ONE encoder register);
MSH_CMD_EXPORT_ALIAS(cmd_enc_cal,          t_enc_cal,   calibrate 3 enc zero offset);