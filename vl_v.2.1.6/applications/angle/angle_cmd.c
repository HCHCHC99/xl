/**
 * @file    angle_cmd.c
 * @brief   回转角度验证命令（MSH）—— 2026-10-28
 *
 * ============================================================
 * 【为什么单独文件】
 * ------------------------------------------------------------
 * 与 enc_test.c 同理：本文件需要 abs_angle.h + mech_cfg，
 * 但**不能**引 motor/motor.h（会与 app.h 的 MOTOR_STATE_FAULT 冲突）。
 * 故隔离到独立编译单元。
 *
 * ============================================================
 * 【命令设计意图：把"算法对不对"和"机械对不对"分开验】
 * ------------------------------------------------------------
 * t_ang_solve    只跑一次上电解算，看 k 和残差     → 验游标算法
 * t_ang_watch    连续打印角度，转轴观察单调性      → 验增量跟踪
 * t_ang_state    打印内部状态与统计                → 验健康度
 * t_ang_bench    计时 N 次读取与解算，出耗时       → 验性能预算
 * t_ang_probe    Z0 单颗 vs 三颗对比（增量收益量化）→ 验"简算法"是否成立
 * ============================================================
 */

#include <rtthread.h>
#include <finsh.h>
#include <stdlib.h>

#include "abs_angle.h"

/* 【2026-11-10 彩标 P4】命令结论行：FAIL/WARN 上色，数据表保持单色 */
#include "../log/log_color.h"
#include "mech_cfg_store.h"
#include "bsp_dev_kth7812.h"
#include "mode/sys_caps.h"
#include "motor_angle.h"     /* 【2026-11-03】t_ang_solve 成功后同步生产路径 */
#include "motor_speed.h"     /* 同步后必须重置速度窗口 */

/* ============================================================
 * 本地工具（只为 t_ang_probe 打印残差用，与 abs_angle.c 内部实现
 * 保持独立 —— 这是刻意的交叉验证：若两边算出的残差不一致，
 * 说明有一处写错了。共享同一份代码会失去这个校验能力。）
 * ============================================================ */
#define ABS_ENC_FULL_LOCAL   4096
#define ABS_ENC_HALF_LOCAL   2048

static int32_t wrap_local(int32_t a, int32_t b)
{
    int32_t d = (a - b) % ABS_ENC_FULL_LOCAL;
    if (d >  ABS_ENC_HALF_LOCAL) { d -= ABS_ENC_FULL_LOCAL; }
    if (d < -ABS_ENC_HALF_LOCAL) { d += ABS_ENC_FULL_LOCAL; }
    return d;
}

static int32_t pred_local(int32_t cnt, uint16_t p_num, uint16_t p_den)
{
    int64_t num = (int64_t)cnt * (int64_t)p_num;
    int64_t den = (int64_t)p_den;
    int64_t q;
    int32_t r;

    if (den <= 0) { return 0; }
    if (num >= 0) { q = (num + den / 2) / den; }
    else          { q = -((-num + den / 2) / den); }

    r = (int32_t)(q % ABS_ENC_FULL_LOCAL);
    if (r < 0) { r += ABS_ENC_FULL_LOCAL; }
    return r;
}

/* ============================================================
 * t_ang_solve
 * ------------------------------------------------------------
 * 执行一次上电解算，打印 k、累计计数、以及三颗原始值。
 * 判据（人为判读，不是程序判定）：
 *   - 返回 OK          ⇒ 游标解算成功，k 落在 [k_min, k_max]
 *   - 返回 NO_MATCH    ⇒ 齿型/接线不对，或机械打滑
 *   - 返回 AMBIGUOUS    ⇒ 齿型选取错误（Δ 太小，区分不开）
 *   - 返回 OUT_OF_RANGE ⇒ 机械不在标定行程内（正常，需先转到行程内）
 * ============================================================ */
static void cmd_ang_solve(int argc, char **argv)
{
    abs_angle_err_t e = ABS_ANGLE_OK;
    abs_angle_state_t st;
    const mech_rt_t *rt;
    uint16_t r0, r1, r2;

    (void)argc; (void)argv;

    if (sys_cap_require(SYS_CAP_DIAG_READ, "t_ang_solve") == 0U) { return; }

    rt = mech_get_rt();
    if (rt == RT_NULL) {
        rt_kprintf("[t_ang_solve] mech_cfg NOT loaded, check mech_cfg boot step\n");
        return;
    }

    /* 先打印原始值，方便对照 */
    r0 = KTH78_ReadAngle((KTH7812_DEV_DEF)0);
    r1 = KTH78_ReadAngle((KTH7812_DEV_DEF)1);
    r2 = KTH78_ReadAngle((KTH7812_DEV_DEF)2);

    rt_kprintf("[t_ang_solve] raw: Z0=%d Z1=%d Z2=%d\n",
               (r0 == KTH7812_ANGLE_INVALID) ? -1 : (int)r0,
               (r1 == KTH7812_ANGLE_INVALID) ? -1 : (int)r1,
               (r2 == KTH7812_ANGLE_INVALID) ? -1 : (int)r2);

    rt_kprintf("[t_ang_solve] teeth Z0:Z1:Z2 = %u:%u:%u  reduced %u/%u  %u/%u\n",
               (unsigned)g_mech.cfg.enc_z01_num, (unsigned)g_mech.cfg.enc_z01_den,
               (unsigned)g_mech.cfg.enc_z02_den,
               (unsigned)rt->z01_p_num, (unsigned)rt->z01_p_den,
               (unsigned)rt->z02_p_num, (unsigned)rt->z02_p_den);
    rt_kprintf("[t_ang_solve] k range [%d, %d] (total %d turns)  cnt range [%d, %d]\n",
               (int)rt->k_min, (int)rt->k_max, (int)rt->k_count,
               (int)rt->cnt_min, (int)rt->cnt_max);
    rt_kprintf("[t_ang_solve] vernier lift D1=%d D2=%d (0.001deg)  tol=%d\n",
               (int)rt->d1_x1000, (int)rt->d2_x1000, (int)rt->tol_x1000);

    e = abs_angle_solve(&e);
    abs_angle_get_state(&st);

    if (e == ABS_ANGLE_OK) {
        int32_t mdeg = 0, x10 = 0;
        abs_angle_get_mdeg(&mdeg);
        abs_angle_get_x10(&x10);
        rt_kprintf("[t_ang_solve] OK  k=%d  cnt=%d\n", (int)st.k, (int)st.cnt_accum);
        rt_kprintf("              SWING angle = %d.%03d deg (%d x0.1deg)\n",
                   mdeg / 1000, (mdeg < 0 ? -mdeg : mdeg) % 1000, (int)x10);
        rt_kprintf("              mode = %s\n", abs_angle_mode_name(st.mode));

        /* ============================================================
         * 【2026-11-03 新增】解算成功后同步生产路径 motor_angle
         * ------------------------------------------------------------
         * 问题：abs_angle 与 motor_angle 是两条独立的状态。
         *   开机时 motor_angle_solve() 若失败，motor_angle 进入
         *   degenerate=1 零基点模式；此后即使手工 t_ang_solve 成功，
         *   motor_angle 仍停在零基点，t_mangle 一直显示 ZERO-BASE，
         *   角度也还是"相对开机位置"，不是刚解出的绝对角。
         *   实际表现：校准完 t_ang_solve 报 OK，但 t_mangle 仍报警告，
         *   两条路径的数据互相矛盾 —— 这是纯状态不同步，不是算法问题。
         *
         * 修法：解算成功后让 motor_angle 用当前 abs_angle 结果重建基线。
         *   motor_angle_solve() 内部会再走一次 abs_angle_solve()，
         *   此时齿数/零点都已就绪，必然成功，于是 degenerate 清零。
         *   之后必须 motor_speed_reset()，否则速度窗口会用"解算前"
         *   的旧 cnt 算出一个巨大假转速。
         * ============================================================ */
        if (motor_angle_solve() == MOTOR_ANGLE_OK) {
            motor_speed_reset();
            rt_kprintf("              motor_angle SYNCED (degenerate cleared, speed reset)\n");
        } else {
            LOG_WARN("              WARN: motor_angle resync FAILED, t_mangle may still\n");
            rt_kprintf("                    report ZERO-BASE mode\n");
        }
    } else {
        LOG_ERR("[t_ang_solve] FAIL: %s\n", abs_angle_err_name(e));

        /* ---- 【2026-11-02 新增】失败也必须给出中间量 ----
         * 以前只打印"三种常见原因"，等于让操作者盲猜。
         * 现在把 raw / best_k / r1 / r2 / guard 全部摊开，
         * 靠残差的**符号与量级**就能判定是哪一类故障。
         * 判定表见下方 hint。 */
        rt_kprintf("              ---- diagnostics ----\n");
        rt_kprintf("              raw  Z0=%d  Z1=%d  Z2=%d\n",
                   (int)st.dbg_raw0, (int)st.dbg_raw1, (int)st.dbg_raw2);
        rt_kprintf("              best k=%d  r1=%+d  r2=%+d  score=%d\n",
                   (int)st.dbg_best_k, (int)st.dbg_r1, (int)st.dbg_r2,
                   (int)st.dbg_score);
        rt_kprintf("              second score=%d  guard=%d\n",
                   (int)st.dbg_second, (int)st.dbg_guard);

        /* ============================================================
         * 【2026-11-03 重写】方向诊断改为"直读 RD 寄存器"，不再从残差反推
         * ------------------------------------------------------------
         * 上一版靠"哪个残差大就猜哪颗反"，实测给出了**错误建议**：
         *   实测 r1=+8(合格) r2=+84(超 guard=48) → 提示"Z2 反向"
         *   但穷举 4 种方向组合后，真正能解出来的是"Z0 反向"，
         *   把 Z2 反向反而依然 NO_MATCH。
         *
         * 根因：钳位残差(wrap_diff 到 ±2048)在方向错时会**任意象限**
         * 出现，残差大小与"是否反向"没有稳定映射。
         *
         * 正确做法：直接读 RD 寄存器（0x09 的 bit7）。
         *
         * 【2026-11-03 二次纠正 —— 上一版判据依然错，且会毁掉正确配置】
         * 上一版写的是"三颗 RD 必须一致，不一致就是根因"，并给出
         *   t_enc_wr 0 0x09 0x80 confirm
         * 这条建议是**有害的**：
         *   机械事实是 Z0 分别与 Z1/Z2 **外啮合**，物理转向必然相反。
         *   所以 RD 的正确配置本来就是**不一致**：
         *     Z0 RD=0（反向，补偿机械反向）、Z1 RD=1、Z2 RD=1。
         *   这样三颗在软件视角下才同向，游标算法才能工作。
         *   照上一版提示把 Z0 改成 RD=1，等于把正确配置改坏，
         *   会让原本勉强能解的场景彻底失效。
         *
         * 正确判据（机械拓扑决定，不是"必须一致"）：
         *   啮合关系     期望 RD 组合            说明
         *   ----------  ---------------------  --------------------------
         *   外啮合(现状) Z0=0, Z1=1, Z2=1       物理反向 -> RD 需补偿
         *   同向(直连)   Z0=1, Z1=1, Z2=1       三颗同转 -> RD 全一致
         * 故这里只做**信息展示**，不再断言哪种是错的、也不再给改法。
         * ============================================================ */
        {
            uint8_t  rd[3];
            uint8_t  i;
            uint8_t  rd_ok = 0U;

            rt_kprintf("              ---- RD register check (informational) ----\n");

            for (i = 0U; i < 3U; i++)
            {
                uint16_t raw9 = 0U;

                /* 读 0x09 原始响应：高 8 位是寄存器值，bit7 即 RD。
                 * 返回值是错误码，实际数据经 resp 指针带出。 */
                if (KTH78_ReadRegRaw((KTH7812_DEV_DEF)i, 0x09U, &raw9) == 0)
                {
                    rd[i] = (uint8_t)(((raw9 >> 8) & 0x80U) ? 1U : 0U);
                    rd_ok++;

                    rt_kprintf("              Z%u: raw=0x%04X  RD(bit7)=%u  (%s)\n",
                               (unsigned)i, (unsigned)raw9, (unsigned)rd[i],
                               (rd[i] != 0U) ? "CW->inc" : "CCW->inc");
                }
                else
                {
                    rd[i] = 0xFFU;      /* 读取失败，标记为未知 */
                    LOG_ERR("              Z%u: READ FAILED (SPI/CRC)\n",
                               (unsigned)i);
                }
            }

            if (rd_ok == 3U)
            {
                if ((rd[0] == 0U) && (rd[1] == 1U) && (rd[2] == 1U))
                {
                    rt_kprintf("              => Z0=0,Z1=1,Z2=1 = EXPECTED for EXTERNAL\n");
                    rt_kprintf("                 MESH (Z0 physically reverses vs Z1/Z2).\n");
                    rt_kprintf("                 This config is CORRECT, do NOT 'fix' it.\n");
                }
                else if ((rd[0] == 1U) && (rd[1] == 1U) && (rd[2] == 1U))
                {
                    rt_kprintf("              => all RD=1 = expected for DIRECT-DRIVE\n");
                    rt_kprintf("                 (three encoders turn the same way).\n");
                }
                else
                {
                    rt_kprintf("              => RD combo Z0=%u Z1=%u Z2=%u\n",
                               (unsigned)rd[0], (unsigned)rd[1], (unsigned)rd[2]);
                    rt_kprintf("                 not either standard combo. If gears are\n");
                    rt_kprintf("                 EXTERNAL MESH, expected is Z0=0 Z1=1 Z2=1.\n");
                    rt_kprintf("                 Only change RD if you physically verified\n");
                    rt_kprintf("                 the mesh type. Do NOT guess from residuals.\n");
                }
            }
            rt_kprintf("              (RD does NOT explain this failure by itself;\n");
            rt_kprintf("               check teeth ratio / calibration / travel range)\n");
        }

        /* 残差仅作参考，不再作为方向判据 */
        rt_kprintf("              ---- residuals (reference only) ----\n");
        rt_kprintf("              |r1|=%d  |r2|=%d  guard=%d\n",
                   (int)((st.dbg_r1 < 0) ? -st.dbg_r1 : st.dbg_r1),
                   (int)((st.dbg_r2 < 0) ? -st.dbg_r2 : st.dbg_r2),
                   (int)st.dbg_guard);

        rt_kprintf("              ---- other causes ----\n");
        rt_kprintf("              1) mech outside travel (move to middle then retry)\n");
        rt_kprintf("              2) encoder read failure (run t_enc_raw 1)\n");
    }
}

/* ============================================================
 * t_ang_watch [次数] [间隔ms]
 * ------------------------------------------------------------
 * 连续打印角度。**手动慢慢转回转轴**，观察：
 *   - 角度单调变化（不跳变、不回头）
 *   - 跨越 Z0 的 4095→0 边界时"角度值连续"（这是增量法正确性的关键验证点）
 *   - 反方向转，角度回退
 * 若在 4095→0 处出现约 360/1080 ≈ 0.33° 的阶跃，说明环绕修正有问题。
 * ============================================================ */
static void cmd_ang_watch(int argc, char **argv)
{
    int n = 20;
    int interval = 200;
    int k;
    abs_angle_state_t st;
    int32_t last_x10 = 0;
    int32_t printed = 0;

    if (sys_cap_require(SYS_CAP_DIAG_READ, "t_ang_watch") == 0U) { return; }

    if (argc >= 2) { n = atoi(argv[1]); }
    if (argc >= 3) { interval = atoi(argv[2]); }
    if (n <= 0)   { n = 20; }
    if (n > 2000) { n = 2000; }
    if (interval < 20)   { interval = 20; }
    if (interval > 5000) { interval = 5000; }

    abs_angle_get_state(&st);
    if (st.mode != ABS_ANGLE_MODE_INCR && st.mode != ABS_ANGLE_MODE_COARSE) {
        rt_kprintf("[t_ang_watch] not solved yet, run t_ang_solve first\n");
        return;
    }

    rt_kprintf("[t_ang_watch] n=%d interval=%dms  turn SWING axis, watch monotonic\n", n, interval);
    rt_kprintf("  idx   cnt_accum   raw0   angle_x10   Δx10   err\n");

    for (k = 0; k < n; k++) {
        abs_angle_err_t e;
        abs_angle_state_t s2;
        int32_t x10 = 0;
        uint16_t raw;

        e   = abs_angle_update((uint32_t)rt_tick_get_millisecond());
        raw = KTH78_ReadAngleRaw((KTH7812_DEV_DEF)0);
        abs_angle_get_state(&s2);

        if (abs_angle_get_x10(&x10) == ABS_ANGLE_OK) {
            int32_t d = x10 - last_x10;
            if (printed > 0) {
                rt_kprintf("  %4d  %9d   %4u   %8d   %+5d   %s\n",
                           k, (int)s2.cnt_accum, raw, (int)x10, (int)d,
                           abs_angle_err_name(e));
            } else {
                rt_kprintf("  %4d  %9d   %4u   %8d       -   %s\n",
                           k, (int)s2.cnt_accum, raw, (int)x10,
                           abs_angle_err_name(e));
            }
            last_x10 = x10;
            printed++;
        } else {
            rt_kprintf("  %4d  %9d   %4u        ---       -   %s\n",
                       k, (int)s2.cnt_accum, raw, abs_angle_err_name(e));
        }

        rt_thread_mdelay(interval);
    }

    abs_angle_get_state(&st);
    rt_kprintf("[t_ang_watch] stat: incr=%u jump=%u fail=%u  mode=%s\n",
               (unsigned)st.incr_cnt, (unsigned)st.jump_cnt,
               (unsigned)st.fail_cnt, abs_angle_mode_name(st.mode));
}

/* ============================================================
 * t_ang_state
 * ------------------------------------------------------------
 * 打印完整内部状态。排障第一命令。
 * ============================================================ */
static void cmd_ang_state(int argc, char **argv)
{
    abs_angle_state_t st;
    const mech_rt_t *rt;

    (void)argc; (void)argv;

    if (sys_cap_require(SYS_CAP_DIAG_READ, "t_ang_state") == 0U) { return; }

    abs_angle_get_state(&st);
    rt = mech_get_rt();

    rt_kprintf("=== abs_angle state ===\n");
    rt_kprintf("  mode         = %s\n", abs_angle_mode_name(st.mode));
    rt_kprintf("  last_err     = %s\n", abs_angle_err_name(st.last_err));
    rt_kprintf("  k            = %d\n", (int)st.k);
    rt_kprintf("  cnt_accum    = %d\n", (int)st.cnt_accum);
    rt_kprintf("  raw_z0       = %d\n", (int)st.raw_z0);
    rt_kprintf("  solve_cnt    = %u\n", (unsigned)st.solve_cnt);
    rt_kprintf("  incr_cnt     = %u\n", (unsigned)st.incr_cnt);
    rt_kprintf("  jump_cnt     = %u\n", (unsigned)st.jump_cnt);
    rt_kprintf("  fail_cnt     = %u\n", (unsigned)st.fail_cnt);

    if (rt != RT_NULL) {
        rt_kprintf("  i_total      = %d.%02d\n",
                   rt->i_total_x100 / 100, rt->i_total_x100 % 100);
        rt_kprintf("  resolution   = %u e-9 deg/LSB\n", (unsigned)rt->res_ndeg_per_lsb);
        rt_kprintf("  k range      = [%d, %d]\n", (int)rt->k_min, (int)rt->k_max);
        rt_kprintf("  cnt range    = [%d, %d]\n", (int)rt->cnt_min, (int)rt->cnt_max);
        rt_kprintf("  travel       = [%d.%d, %d.%d] deg\n",
                   rt->out_min_x10 / 10, (rt->out_min_x10 < 0 ? -rt->out_min_x10 : rt->out_min_x10) % 10,
                   rt->out_max_x10 / 10, (rt->out_max_x10 < 0 ? -rt->out_max_x10 : rt->out_max_x10) % 10);
    } else {
        rt_kprintf("  mech_rt      = NULL (mech_cfg NOT loaded)\n");
    }
}

/* ============================================================
 * t_ang_bench [次数]
 * ------------------------------------------------------------
 * 量化两条路径的耗时差异，直接回答"增量法省多少"：
 *   A. 三颗全读（游标法，上电用）   = 3 × 2 帧 SPI
 *   B. 只读 Z0（增量法，运行用）    = 1 × 2 帧 SPI  ... 实际 1 帧足够
 * 用 DWT CYCCNT 计时（若未使能则退回 rt_tick，精度差但可看数量级）。
 * ============================================================ */
static void cmd_ang_bench(int argc, char **argv)
{
    int n = 200;
    int k;
    uint32_t t0, t1;
    volatile uint16_t sink = 0;
    uint32_t cyc_a = 0, cyc_b = 0, cyc_u0 = 0;

    if (sys_cap_require(SYS_CAP_DIAG_READ, "t_ang_bench") == 0U) { return; }

    if (argc >= 2) { n = atoi(argv[1]); }
    if (n <= 0)   { n = 200; }
    if (n > 5000) { n = 5000; }

    /* 使能 DWT CYCCNT（幂等，重复使能无副作用） */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

    /* --- A: 三颗全读（模拟上电解算的一次采样）--- */
    t0 = DWT->CYCCNT;
    for (k = 0; k < n; k++) {
        sink ^= KTH78_ReadAngle((KTH7812_DEV_DEF)0);
        sink ^= KTH78_ReadAngle((KTH7812_DEV_DEF)1);
        sink ^= KTH78_ReadAngle((KTH7812_DEV_DEF)2);
    }
    t1 = DWT->CYCCNT;
    cyc_a = (t1 - t0) / (uint32_t)n;

    /* --- B: 只读 Z0（运行期增量）--- */
    t0 = DWT->CYCCNT;
    for (k = 0; k < n; k++) {
        sink ^= KTH78_ReadAngle((KTH7812_DEV_DEF)0);
    }
    t1 = DWT->CYCCNT;
    cyc_b = (t1 - t0) / (uint32_t)n;

    /* --- C: 纯算法（不含 SPI）--- */
    t0 = DWT->CYCCNT;
    for (k = 0; k < n; k++) {
        abs_angle_update(0U);
    }
    t1 = DWT->CYCCNT;
    cyc_u0 = (t1 - t0) / (uint32_t)n;

    rt_kprintf("[t_ang_bench] n=%d  (200MHz, 1 cycle=5ns)\n", n);
    rt_kprintf("  3-enc read  (vernier): %6u cyc = %5.2f us\n",
               (unsigned)cyc_a, cyc_a * 0.005);
    rt_kprintf("  Z0 only     (increm): %6u cyc = %5.2f us\n",
               (unsigned)cyc_b, cyc_b * 0.005);
    if (cyc_a > cyc_b) {
        rt_kprintf("  => increm saves %u cyc = %.2f us (%.0f%%)\n",
                   (unsigned)(cyc_a - cyc_b), (cyc_a - cyc_b) * 0.005,
                   100.0 * (cyc_a - cyc_b) / cyc_a);
    }
    rt_kprintf("  update() with SPI  : %6u cyc = %5.2f us\n",
               (unsigned)cyc_u0, cyc_u0 * 0.005);
    rt_kprintf("  (sink=%u, anti-optimize)\n", (unsigned)sink);
}

/* ============================================================
 * t_ang_probe [次数]
 * ------------------------------------------------------------
 * 三颗同步采样，打印"游标残差"的实时值。
 * 用途：判断机械是否真的按齿数比啮合。
 *   - 静止时残差应稳定且很小（< guard，A2 约 48 LSB）
 *   - 转一圈后残差若持续增大 ⇒ 齿数比与实际不符（换了齿型但没切 profile）
 * ============================================================ */
static void cmd_ang_probe(int argc, char **argv)
{
    int n = 10;
    int k;
    const mech_rt_t *rt;

    if (sys_cap_require(SYS_CAP_DIAG_READ, "t_ang_probe") == 0U) { return; }

    if (argc >= 2) { n = atoi(argv[1]); }
    if (n <= 0)  { n = 10; }
    if (n > 500) { n = 500; }

    rt = mech_get_rt();
    if (rt == RT_NULL) {
        rt_kprintf("[t_ang_probe] mech_cfg NOT loaded\n");
        return;
    }

    rt_kprintf("[t_ang_probe] teeth %u:%u:%u  reduced %u/%u  %u/%u\n",
               (unsigned)g_mech.cfg.enc_z01_num, (unsigned)g_mech.cfg.enc_z01_den,
               (unsigned)g_mech.cfg.enc_z02_den,
               (unsigned)rt->z01_p_num, (unsigned)rt->z01_p_den,
               (unsigned)rt->z02_p_num, (unsigned)rt->z02_p_den);
    rt_kprintf("  idx   raw0   raw1   raw2   best_k   r1    r2   score\n");

    for (k = 0; k < n; k++) {
        uint16_t raw0 = KTH78_ReadAngle((KTH7812_DEV_DEF)0);
        uint16_t raw1 = KTH78_ReadAngle((KTH7812_DEV_DEF)1);
        uint16_t raw2 = KTH78_ReadAngle((KTH7812_DEV_DEF)2);

        if (raw0 == KTH7812_ANGLE_INVALID || raw1 == KTH7812_ANGLE_INVALID ||
            raw2 == KTH7812_ANGLE_INVALID) {
            LOG_ERR("  %4d   CRC/SPI FAIL\n", k);
            rt_thread_mdelay(100);
            continue;
        }

        {
            int32_t kk;
            int32_t bk = 0, bs = 0x7FFFFFFF, br1 = 0, br2 = 0;

            for (kk = rt->k_min; kk <= rt->k_max; kk++) {
                int32_t cnt = kk * ABS_ENC_FULL_LOCAL + (int32_t)raw0;
                int32_t p1  = pred_local(cnt, rt->z01_p_num, rt->z01_p_den);
                int32_t p2  = pred_local(cnt, rt->z02_p_num, rt->z02_p_den);
                int32_t r1  = wrap_local(p1, (int32_t)raw1);
                int32_t r2  = wrap_local(p2, (int32_t)raw2);
                int32_t sc  = ((r1 < 0) ? -r1 : r1) + ((r2 < 0) ? -r2 : r2);

                if (sc < bs) { bs = sc; bk = kk; br1 = r1; br2 = r2; }
            }
            rt_kprintf("  %4d   %4u   %4u   %4u   %6d  %+4d  %+4d   %d\n",
                       k, raw0, raw1, raw2, (int)bk, (int)br1, (int)br2, (int)bs);
        }

        rt_thread_mdelay(100);
    }
}

MSH_CMD_EXPORT_ALIAS(cmd_ang_solve, t_ang_solve, solve absolute angle by vernier);
MSH_CMD_EXPORT_ALIAS(cmd_ang_watch, t_ang_watch, watch angle continuous update);
MSH_CMD_EXPORT_ALIAS(cmd_ang_state, t_ang_state, print abs_angle internal state);
MSH_CMD_EXPORT_ALIAS(cmd_ang_bench, t_ang_bench, benchmark enc read paths);
MSH_CMD_EXPORT_ALIAS(cmd_ang_probe, t_ang_probe, print vernier residuals);