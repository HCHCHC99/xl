/**
 * @file    log_color.h
 * @brief   彩色日志 —— 语义色定义、颜色码、开关控制
 * @date    2026-11-10
 *
 * ============================================================
 * 背景与约束
 * ------------------------------------------------------------
 * 控制台设备为 "jlinkRtt"（rtconfig.h:48），数据经
 * drv_rtt.c:113 的 SEGGER_RTT_Write() 原样透传到 J-Link RTT Viewer。
 *
 * RTT Viewer 原生支持 ANSI 转义序列（SEGGER 官方文档：
 *   "RTT Viewer supports changing the text color and background color"），
 * 因此 ANSI 码在这条链路上可直接生效，无需宿主端额外配置。
 *
 * SEGGER_RTT.h:377-410 已预定义 RTT_CTRL_* 系列宏，但本模块
 * **不使用**它们，原因：
 *   ① 它们定义在 packages/ 下，日志模块不应依赖第三方包的内部路径；
 *   ② 本模块允许 sys_config 关色，关闭时不能输出任何字节，
 *      用自建表更便于统一判断；
 *   ③ RTT_CTRL_* 的语义是"颜色名"，本模块需要的是"日志级别"，
 *      中间要一层映射，直接用自建表少一次跳转。
 *
 * ============================================================
 * 为什么是"宏 + 独立缓冲"而不是"改 rt_kprintf"
 * ------------------------------------------------------------
 * rt_kprintf 的实现（kservice.c:1302）：
 *   static char rt_log_buf[RT_CONSOLEBUF_SIZE];   // 128 B，静态复用
 *   length = rt_vsnprintf(rt_log_buf, sizeof - 1, fmt, args);
 *   if (length > RT_CONSOLEBUF_SIZE - 1) length = RT_CONSOLEBUF_SIZE - 1;
 *   rt_device_write(_console_device, 0, rt_log_buf, length);
 *
 * 三个关键事实：
 *   ① 缓冲是 static（非重入），且格式化长度超限时**静默截断**
 *      —— 颜色码占 11 字节，若继续复用这个 128 B 缓冲，
 *         等于把可用文本压到 117 B，超长打印会被砍尾且无任何提示；
 *   ② rt_log_buf 是 static，多线程并发 rt_kprintf 本就存在竞态
 *      （工程现状，非本次引入），但日志模块不能加剧它；
 *   ③ 单次 rt_device_write 是原子的（serial.c:868 → mcu_transmit 一次
 *      SEGGER_RTT_Write），所以**一行必须一次性写出**，否则会出现
 *      颜色码与正文被其他线程输出穿插，导致整屏染色错乱。
 *
 * 因此本模块自带一个 256 B 的独立缓冲（比 rt_kprintf 宽一倍），
 * 并用互斥锁保护"格式化 + 写出"这一段，保证：
 *   · 一条日志的颜色码与正文不可分割
 *   · 不挤占 rt_kprintf 的 128 B 空间
 *   · 超长时显式打标记而不是静默砍尾
 *
 * 为什么缓冲选 256 而不是 128：
 *   现有最长格式串 83 字符，展开后最坏约 100 B；再加 11 B 颜色码
 *   与 1 B 结尾，128 已无余量。256 给出一倍余量，
 *   且单个缓冲仅占 .bss 256 B，对 192 KB SRAM 可忽略。
 *
 * ============================================================
 * 中断上下文
 * ------------------------------------------------------------
 * 互斥锁在中断里不可用（会断言）。因此本模块在中断上下文
 * **跳过取锁**，退化为"直接写"——与现有 rt_kprintf 行为一致，
 * 不引入新风险。判定用 rt_interrupt_get_nest()。
 *
 * 但要注意：中断里打印彩色日志本身就不是好实践（20 kHz ISR
 * 里 rt_kprintf 会拖长中断）。本模块不阻止，因为现有代码已有
 * 中断打印，改成静默反而丢信息。
 * ============================================================
 */

#ifndef __LOG_COLOR_H__
#define __LOG_COLOR_H__

#include <rtthread.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 1. 日志级别（语义色）
 * ------------------------------------------------------------
 * 用"级别"而非"颜色名"作为接口：
 *   调用点表达的是"这是错误"，不是"这是红色"。
 *   将来若想把错误改成紫底，只改 log_color.c 的映射表即可，
 *   1205 个调用点一处不动。
 *
 * 级别与配色的对应关系见 log_color.c 的 s_palette[]。
 * ============================================================ */
typedef enum {
    LOG_LVL_INFO = 0,   /* 常规信息：模块初始化、配置装载 */
    LOG_LVL_OK,         /* 成功/通过：OK / pass / done */
    LOG_LVL_WARN,       /* 警告：可继续但需关注 */
    LOG_LVL_ERR,        /* 错误：功能失败，需处理 */
    LOG_LVL_FAULT,      /* 故障/急停：需立即停机（唯一使用背景色）*/
    LOG_LVL_STATE,      /* 状态跃迁：状态机 Enter/Exit */
    LOG_LVL_ADC,        /* 采样/测量数据：raw、电流、角度 */
    LOG_LVL_TRACE,      /* 高频跟踪：低视觉权重，用于刷屏类输出 */
    LOG_LVL_CMD,        /* MSH 命令输出 */
    LOG_LVL_MAX
} log_lvl_t;

/* ============================================================
 * 2. 全局开关与配色查询
 * ============================================================ */

/** @brief 取某级别的 ANSI 前景+背景码；关闭或非法时返回 "" */
const char *log_color_code(log_lvl_t lvl);

/** @brief 取复位码；关闭时返回 "" */
const char *log_color_reset(void);

/**
 * @brief  运行时开关彩色输出
 * @param  on  0=关闭（输出纯文本，便于存文件分析），非 0=开启
 * @note   关闭时 log_color_code() 返回空串，宏退化为普通打印，
 *         不损失任何文本内容，只是没有颜色。
 */
void log_color_set(uint8_t on);

/** @brief 查询当前是否开启 */
uint8_t log_color_enabled(void);

/**
 * @brief  日志统一输出入口（供 LOG_* 宏调用）
 * @param  lvl  语义级别
 * @param  fmt  格式串（同 printf）
 * @note   内部：取锁 → vsnprintf 到独立缓冲 → 加颜色码 → 单次写出。
 *         中断上下文自动跳过取锁。
 *         超长时截断并在尾部加 "<cut>" 标记（不静默砍尾）。
 */
void log_printf(log_lvl_t lvl, const char *fmt, ...);

/**
 * @brief  取锁超时计数（并发压力指标，供 t_log stat）
 * @note   为 0 表示从未发生"20ms 内拿不到锁"。
 *         非 0 说明某处日志过于频繁，或某线程长期持有锁。
 */
uint32_t log_color_get_drop_cnt(void);

/**
 * @brief  取"中断里被闸门丢弃"的日志条数
 * @note   中断上下文只放行 FAULT 级（见 log_color.c 的
 *         LOG_ISR_MAX_LVL）。本值 >0 说明有人在 ISR 里
 *         用了 LOG_INFO/WARN/ERR 等，应立即改为
 *         "置标志 + 线程补打"（参考 hw_brake.c 的做法）。
 */
uint32_t log_color_get_isr_drop_cnt(void);

/* ============================================================
 * 3. 便捷宏
 * ------------------------------------------------------------
 * 【为什么用 ##__VA_ARGS__ 而不是 C23 的 __VA_OPT__】
 *   本工程工具链是 arm-none-eabi-gcc 5.4.1（见项目记忆"统一用 5.4.1"），
 *   GCC 5.4 不支持 __VA_OPT__ 与 C23 语义。
 *   ##__VA_ARGS__ 是 GCC 扩展，在 5.4 上可用，且在无变参时
 *   会正确地吃掉前面的逗号。
 *
 * 【为什么宏里不做字符串拼接】
 *   LOG_ERR("[X] failed: %s", s) 这种调用，宏无法在编译期
 *   把颜色码与格式串合并（颜色码是运行期可取空的）。
 *   所以颜色码作为额外参数传入 log_printf()，
 *   由它在 vsnprintf 之后拼装——这样关闭彩色时零额外字节。
 * ============================================================ */

/** 通用带级别输出 */
#define LOGF(lvl, fmt, ...)     log_printf((lvl), (fmt), ##__VA_ARGS__)

#define LOG_INFO(fmt, ...)      LOGF(LOG_LVL_INFO,  fmt, ##__VA_ARGS__)
#define LOG_OK(fmt, ...)        LOGF(LOG_LVL_OK,    fmt, ##__VA_ARGS__)
#define LOG_WARN(fmt, ...)      LOGF(LOG_LVL_WARN,  fmt, ##__VA_ARGS__)
#define LOG_ERR(fmt, ...)       LOGF(LOG_LVL_ERR,   fmt, ##__VA_ARGS__)
#define LOG_FAULT(fmt, ...)     LOGF(LOG_LVL_FAULT, fmt, ##__VA_ARGS__)
#define LOG_STATE(fmt, ...)     LOGF(LOG_LVL_STATE, fmt, ##__VA_ARGS__)
#define LOG_ADC(fmt, ...)       LOGF(LOG_LVL_ADC,   fmt, ##__VA_ARGS__)
#define LOG_TRACE(fmt, ...)     LOGF(LOG_LVL_TRACE, fmt, ##__VA_ARGS__)
#define LOG_CMD(fmt, ...)       LOGF(LOG_LVL_CMD,   fmt, ##__VA_ARGS__)

/* ============================================================
 * 4. MSH 命令
 * ============================================================ */
void log_cmd_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __LOG_COLOR_H__ */