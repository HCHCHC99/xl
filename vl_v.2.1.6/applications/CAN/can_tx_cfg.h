/**
 * @file    can_tx_cfg.h
 * @brief   CAN 周期发送模块级开关 —— 逐级启用/隔离（S0~S3）
 * @note    用户需求（2026-10-27）：
 *          「逐步分隔不同模块的 can 发送……或者在备份版本上慢慢增加 can 发送功能」
 *
 * ============================================================
 * 为什么需要这个开关（而不是直接改 boot_table 删行）
 * ============================================================
 * 1. 隔离验证要能**一处处加、一处处回退**。改 boot_table 删行会让
 *    "谁依赖谁"变模糊：删掉 status_tx 后 can_tx 仍启动、slots=0，
 *    现象是"CAN 安静但正常"，与"CAN 没起来"无法区分。
 * 2. 开关放在**注册入口**（*_init() 第一行），关掉时模块完全不注册、
 *    不占 slot、不调 pack，连 stderr 都没有 —— 才是真正的"没打开"。
 * 3. 全部默认 1（保持现状），只在隔离排查时逐个置 0，不改任何逻辑代码。
 *
 * ============================================================
 * 使用方式（四级验证表）
 * ============================================================
 *  步骤  开关状态                     预期 t_cantx 输出
 *  ---------------------------------------------------------------------
 *  S0  全部置 0                      slots=0，CAN 总线静默
 *  S1  status=1                      slots=1，仅 0x18FF11D1 每 100ms
 *  S2  status=1, led=1               slots=2，+0x18FFA4F0 每 100ms(相位50ms)
 *  S3  全开（默认）                   slots=3，+0x18FF1D60 每 200ms
 *
 *  每级都在 MSH 用 `t_cantx` 核对 slots 与 ok 计数，并观察是否复现
 *  `thread:can_tx stack overflow`。若某级复现，则问题在该模块的 pack 回调。
 *
 * ============================================================
 * 与栈溢出的关系（重要，避免误判）
 * ============================================================
 * 2026-10-27 已查明 can_tx 溢出的**真根因不在 CAN 发送本身**，而在：
 *   D-57 堆与 MSP 栈重叠 16,384 B（board.h HEAP_BEGIN 错）
 *   D-61 断言走 newlib assert 链，单次 692 B（sm_assert.h）
 * 这两条已修复，can_tx 实测峰值 632 B / 分配 1024 B（62%）。
 *
 * 所以本开关的定位是**回归验证与二分定位工具**，不是"绕过溢出"。
 * 若修复后仍溢出，才用它二分到具体模块 —— 此时最可能的原因是某个
 * pack 回调内部新增了大局部数组或 rt_kprintf（见 can_tx.c 顶部注释）。
 */
#ifndef __CAN_TX_CFG_H
#define __CAN_TX_CFG_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 模块级开关：1 = 启用并注册周期帧，0 = 完全不注册
 *
 * 修改后需重新编译。若要运行期切换，用 MSH `t_cantx_en <模块> <0|1>`
 * （仅在 CAN_TX_CFG_RUNTIME 打开时可用，见下）。
 * ============================================================ */

/** 0x18FF11D1 状态帧，100ms，相位 0 —— 最先打开，验证总线通 */
#ifndef CAN_TX_EN_STATUS
#define CAN_TX_EN_STATUS        1
#endif

/** 0x18FFA4F0 LED 状态帧，100ms，相位 50ms —— 与 status 错开同拍 */
#ifndef CAN_TX_EN_LED
#define CAN_TX_EN_LED           1
#endif

/** 0x18FF1D60 举升电磁阀控制帧，200ms —— 依赖 lift 模块 */
#ifndef CAN_TX_EN_VALVE
#define CAN_TX_EN_VALVE         1
#endif

/* ============================================================
 * 运行期开关（默认关闭）
 *
 * 打开后 can_tx 增加一个 RAM 位图，可在不重编译的前提下启停模块，
 * 便于"同一份固件跑完 S0~S3"。
 *
 * 代价：can_tx_poll 每帧多一次位图判断（约 3 条指令），
 *      且在需要最小代码体积/最强可预测性的场合没必要。
 * 默认 0，保持编译期裁剪的零开销。
 * ============================================================ */
#ifndef CAN_TX_CFG_RUNTIME
#define CAN_TX_CFG_RUNTIME      0
#endif

/* 模块位定义（运行期开关用，与上面的宏一一对应） */
#define CAN_TX_MOD_STATUS       (1UL << 0)
#define CAN_TX_MOD_LED          (1UL << 1)
#define CAN_TX_MOD_VALVE        (1UL << 2)

/* 编译期开关转成位图默认值 */
#if CAN_TX_EN_STATUS
#define CAN_TX_MOD_DEFAULT      (CAN_TX_MOD_STATUS | \
                                 (CAN_TX_EN_LED   ? CAN_TX_MOD_LED   : 0) | \
                                 (CAN_TX_EN_VALVE ? CAN_TX_MOD_VALVE : 0))
#else
#define CAN_TX_MOD_DEFAULT      ((CAN_TX_EN_LED   ? CAN_TX_MOD_LED   : 0) | \
                                 (CAN_TX_EN_VALVE ? CAN_TX_MOD_VALVE : 0))
#endif

#ifdef __cplusplus
}
#endif
#endif /* __CAN_TX_CFG_H */