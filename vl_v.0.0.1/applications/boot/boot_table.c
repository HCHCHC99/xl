/**
 * @file    boot_table.c
 * @brief   应用层启动序列表实例 —— 唯一维护点
 * @note    表顺序 = 执行顺序；level 仅用于一致性校验；
 *          retry_max 仅 BOOT_RETRY 有效
 */

#include "boot_table.h"
#include "board_config.h"

/* 【2026-11-10 彩标 P3】hw_check 结果：错误位图=ERR，全部通过=OK */
#include "../log/log_color.h"
#include <rtthread.h>
#include "../framework/manager/ctrl_manager.h"
#include "../tick/tick_50ms.h"
#include "../bsp/bsp_hw_init.h"
#include "../motor/foc/foc_vofa.h"


/* ===================== 外部入口声明 ===================== */
extern int  ipc_center_init(void);
extern int  storage_all_init(void);
extern int  mech_cfg_store_init(void);   /* 机械配置表：齿数比/减速比查表（V1.3 / Q-40） */
extern int  sys_config_init(void);       /* 系统配置表：运行参数（2026-11-03 / DES-SYSCFG-V1.0） */
extern int  bg_scheduler_init(void);
extern int  app_hw_init(void);
extern int  motor_module_init(void);
extern int  foc_adc_init(void);          /* FOC 电流采样：ADC1 四路 + DMA1 CH2 */
extern void Foc_Control_Init(void);        /* FOC 20kHz 控制层（挂采样帧回调） */
extern int  motor_angle_task_init(void);   /* 电机虚拟角度 + 转速（1ms 软定时器）*/
/* 堵转保护（2026-11-05 / S1）。
 * 参数用 void* 而不是 motor_t*：boot_table.c 刻意不 include motor.h
 * （启动表层应保持对业务类型的零依赖），由 motor_stall_init 内部转回。 */
extern int  motor_stall_init(void *motor);
extern int  mode_init(void);
extern int  lift_init(void);
extern int  swing_init(void);
extern int  seq_init(void);
extern int  ctrl_loop_init_all(void);   /* 三环模块（2026-11-10 新增） */
extern int  can_init(void);
extern int  can_tx_init(void);
extern int  status_tx_init(void);
extern int  led_tx_init(void);
extern int  sys_voltage_init(void);
extern int  vehicle_state_init(void);
extern int  acc_state_init(void);       /* 钥匙上电检测（PB0）*/
extern int  eeprom_init(void);
extern int  kb_init(void);
extern int  wdt_monitor_init(void);
extern int  log_color_init(void);        /* 彩色日志（2026-11-10）*/

extern void app_main_thread_entry(void *param);
extern void bg_scheduler_thread_entry(void *param);

extern ctrl_manager_t g_ctrl_mgr;

/* ===================== 适配层：包成 (void) 形式 ===================== */
static int boot_ctrl_mgr_init(void)  { return (int)ctrl_manager_init(&g_ctrl_mgr); }
static int boot_ctrl_mgr_start(void) { return (int)ctrl_manager_start(&g_ctrl_mgr); }

/* ===================== 硬件初始化校验 ===================== */
/**
 * @brief  校验 MCU 硬件资源初始化结果
 * @note   MCU 资源由 INIT_DEVICE_EXPORT(bsp_hw_init) 在 main() 之前完成，
 *         此处只读 g_hw_init_error_map，不重复初始化硬件。
 *         这是「硬件表」与「对象表」之间唯一的衔接点。
 */
static int boot_hw_check(void)
{
    if (g_hw_init_error_map != 0U) {
        LOG_ERR("[BOOT] hw init error map = 0x%08X\n",
                   (unsigned int)g_hw_init_error_map);
        return -1;
    }
    LOG_OK("[BOOT] hw_check pass\n");
    return 0;
}

/* ===================== 堵转保护 ===================== */
/**
 * @brief 堵转保护初始化包装
 *
 * 为什么需要包装而不是直接把 motor_stall_init 放进表里：
 *   启动表的函数签名统一为 `int (*)(void)`，而 motor_stall_init()
 *   需要电机对象指针（触发时用它停机）。包装函数消掉这个差异，
 *   同时把"绑哪个电机"这个决策收敛在 boot 层，而不是散在模块内部。
 *
 * 【为什么是 BOOT_OPTIONAL 而不是 FATAL】
 *   当前阶段堵转配置参数尚未实测整定（stall_low_rpm 等），
 *   若初始化因某个配置项异常返回错误而 FATAL，会连带阻断整机启动，
 *   不利于调试。
 *   ⚠️ **量产/交付前必须改为 BOOT_FATAL** —— 堵转保护失效会让
 *      电机在卡死状态下持续大电流，属安全性缺陷。
 *
 * 【为什么在 boot 层而不是 motor_module_init 里调】
 *   motor_stall_init 要读 sys_config（第 4 步），而 motor_module_init
 *   在第 8 步执行、早于 sys_config 的可用性无需保证。
 *   分成独立步骤后，依赖关系由启动表的 requires 字段显式表达。
 */
static int boot_stall_init(void)
{
    extern void *boot_get_swing_motor(void);

    return motor_stall_init(boot_get_swing_motor());
}

/* ===================== FOC 控制层 ===================== */
static int boot_foc_ctrl_init(void)
{
    Foc_Control_Init();
    return 0;
}

/* ===================== VOFA+ 输出 ===================== */
static int boot_vofa_init(void)
{
    return foc_vofa_init();
}

/* ===================== 线程创建 ===================== */
static int boot_create_threads(void)
{
    rt_thread_t tid;

    /* ============================================================
     * Phase 4: 创建前台主线程 (app_main)
     * 优先级 6，栈大小 2048，事件驱动
     *
     * 【2026-10-27 · D-59】原 4096，按实测下调。
     * 实测（展开 ELF 调用图，含断言/打印路径）：
     *   正常路径 328 B / 含打印 832 B
     *   叠加硬件压栈(104) + 调度现场(40) + FPU(64) ≈ 240 B
     *   → 峰值约 1072 B
     * 取 2048：留 1.9 倍余量。此处保留较大余量是因为
     *   app_main 会经 StateMachine_SendEvent 走函数指针进各业务
     *   状态机（静态调用图看不到），且 50ms tick 路径较宽。
     * ============================================================ */
    tid = rt_thread_create("app_main", app_main_thread_entry, RT_NULL, 2048, 6, 10);
    if (tid == RT_NULL) return -1;
    rt_thread_startup(tid);

    /* ============================================================
     * Phase 5: 创建后台调度线程 (bg_sched)
     * 优先级 15，栈大小 1024，时间轮询
     *
     * 【2026-10-27 · D-59】原 1536，按实测下调。
     * 实测：正常 272 B / 含打印 400 B，+240 B 开销 → 峰值约 640 B。
     * 取 1024：留 1.6 倍余量。
     * ⚠️ bg_sched 通过 g_bg_modules[] 函数指针轮询各模块，静态调用图
     *    看不到被调模块内部。当前仅注册 1 个模块（vehicle_poll，24 B）。
     *    若后续注册的模块内含大局部数组，必须重新核算。
     * ============================================================ */
    tid = rt_thread_create("bg_sched", bg_scheduler_thread_entry, RT_NULL, 1024, 15, 10);
    if (tid == RT_NULL) return -1;
    rt_thread_startup(tid);

    return 0;
}

/* ===================== 启动表 ===================== */
const boot_step_t g_boot_steps[] = {
    /*  name              fn                    level               policy         retry  deps */
    {"hw_check",         boot_hw_check,        BOOT_LV_EARLY,      BOOT_FATAL,    0, {NULL}},
    {"ipc_center",       ipc_center_init,      BOOT_LV_IPC,        BOOT_FATAL,    0, {NULL}},
    {"storage",          storage_all_init,     BOOT_LV_STORAGE,    BOOT_FATAL,    0, {"ipc_center"}},
    /* 系统配置表（2026-11-03，DES-SYSCFG-V1.0）
     * 位置约束：
     *   - 必须在 storage 之后（依赖 storage_find_config + flash_init）
     *   - 必须在 mech_cfg 之前：0x08xx 齿数镜像要读 g_mech，
     *     但本步只用默认值填表，镜像在 dump/get 时才刷新，
     *     故放前面也不会读到未初始化数据
     *   - 必须在各业务模块之前：配置是它们的输入
     * 失败策略：OPTIONAL —— 配置表起不来时 sys_config_init 内部会
     *           全量使用编译期默认值，系统仍可正常启动，
     *           这正是"默认值永远兜底"设计的意义。
     *           绝不能设为 FATAL，否则配置区损坏就整机开不了机。 */
    {"sys_config",       sys_config_init,      BOOT_LV_STORAGE,    BOOT_OPTIONAL, 0, {"storage"}},
    /* 机械配置表（齿数比 / 减速比查表，V1.3 / Q-40）。
     * 位置约束：必须在 storage 之后（依赖 storage_find_config + flash_init），
     *           必须在 swing / lift 之前（它们用 mech_get_rt() 的换算系数）。
     * 失败策略：FATAL —— 配置表算不出来就没有可靠的机械量纲，
     *           宁可停在启动自检，也不能带病进入控制。 */
    {"mech_cfg",         mech_cfg_store_init,  BOOT_LV_STORAGE,    BOOT_FATAL,    0, {"storage"}},
    {"ctrl_mgr_init",    boot_ctrl_mgr_init,   BOOT_LV_FRAMEWORK,  BOOT_FATAL,    0, {NULL}},
    {"app_hw",           app_hw_init,          BOOT_LV_HW,         BOOT_FATAL,    0, {"storage"}},
    {"motor",            motor_module_init,    BOOT_LV_HW,         BOOT_FATAL,    0, {"app_hw"}},
    /* FOC 电流采样（ADC1 三路 + DMA1 CH2）。
     * 位置约束：
     *   - 必须在 drv_adc.c 的 INIT_DEVICE_EXPORT(rt_hw_adc_init) 之后，
     *     否则 ADC_Init 会被其覆盖（该导出在 main 之前执行，天然满足）；
     *   - 必须在 motor 之后：motor 初始化可能调整 PWM，电流采样要跟随时基；
     *   - 用 NONFATAL：电流采样起不来不应阻断整机启动，
     *     由 foc_adc_read() 返回错误让上层降级（FOC 未上线前无影响）。 */
    {"foc_adc",          foc_adc_init,         BOOT_LV_HW,         BOOT_OPTIONAL, 0, {"motor"}},
    /* FOC 20kHz 控制层（2026-11-17）
     * 依赖 foc_adc：注册采样帧完成回调，之后每个 DMA BTC 中断执行控制链。
     * BOOT_OPTIONAL：控制层起不来时 PWM 仍保持 IDLE 全低，安全侧。 */
    {"foc_ctrl",         boot_foc_ctrl_init,   BOOT_LV_HW,         BOOT_OPTIONAL, 0, {"foc_adc"}},
    /* VOFA+：USART3 PB12/PB13 921600，DMA2 CH0；依赖 FOC 采样变量 */
    {"vofa",             boot_vofa_init,       BOOT_LV_APP,        BOOT_OPTIONAL, 0, {"foc_adc"}},
    /* 电机虚拟角度 + 转速（FOC 基础设施，2026-11-02）
     * 位置约束：
     *   - 必须在 mech_cfg 之后：motor_angle_solve 需要减速比与行程
     *   - 必须在 foc_adc 之后：虽然本步只用 SPI，但 FOC 阶段会在
     *     同一批硬件上工作，保持顺序一致便于排查
     *   - BOOT_OPTIONAL：角度起不来不应阻断整机（HMI/手动模式仍可用）
     * 注意：本步只**启动定时器**，不做三颗解算。
     *       解算需 mech_cfg 就绪且编码器可读，由 t_mangle_solve 显式触发。 */
#if !BSP_USING_ABS_ENCODER
    {"mangle",           motor_angle_task_init, BOOT_LV_APP,       BOOT_OPTIONAL, 0, {"mech_cfg", "foc_adc"}},
#endif
    /* 【2026-11-05 / S1】堵转保护初始化
     * 位置约束：
     *   - 必须在 sys_config（第 4 步）之后：stall_load_cfg 要读 0x09xx 十一项
     *   - 必须在 foc_adc 之后：stall_on_tick 内部调 foc_adc_get_bus_current_a10
     *   - 必须在 bg_sched_init 之前：否则 5ms 任务先跑起来，
     *     而 stall_on_tick 内部 s_inited=0 会直接返回，首 5ms 空转
     *   - BOOT_OPTIONAL：保护功能起不来不应阻断整机（会打印告警），
     *     但这是**安全功能**，量产时应改为 FATAL —— 见下方注释 */
    {"stall",            boot_stall_init,      BOOT_LV_APP,       BOOT_OPTIONAL, 0, {"sys_config", "foc_adc"}},
    {"mode",             mode_init,            BOOT_LV_APP,        BOOT_FATAL,    0, {"ctrl_mgr_init"}},
    {"lift",            lift_init,           BOOT_LV_APP,        BOOT_FATAL,    0, {"storage", "ctrl_mgr_init"}},
    {"swing",             swing_init,            BOOT_LV_APP,        BOOT_FATAL,    0, {"app_hw", "ctrl_mgr_init"}},
    {"seq",              seq_init,             BOOT_LV_APP,        BOOT_FATAL,    0, {"ctrl_mgr_init"}},

    /* 【2026-11-10 新增】三环控制模块
     *
     * 依赖 sys_config：要从配置表装载 PID 参数与周期
     * 依赖 swing / lift：两个实例分别接管回转电机与举升阀
     * 必须在 ctrl_mgr_start 之后：钩子里会读写 motor / lift 对象
     *
     * BOOT_OPTIONAL：控制环起不来不应阻断整机 ——
     *   当前驱动仍是开环 50% 占空比，控制环只是"可选的增强"。
     *   若将来把控制环作为唯一驱动路径，须改为 BOOT_FATAL。
     *
     * ⚠ 默认 enabled=0（配置表 cl0_en 默认 0），
     *   即控制环线程会跑但全部旁路，不影响现有行为。 */
    {"ctrl_loop",        ctrl_loop_init_all,   BOOT_LV_APP,        BOOT_OPTIONAL, 0, {"sys_config", "swing", "lift"}},
    {"ctrl_mgr_start",   boot_ctrl_mgr_start,  BOOT_LV_START,      BOOT_FATAL,    0, {"ctrl_mgr_init"}},
    {"bg_sched_init",    bg_scheduler_init,    BOOT_LV_SERVICE,    BOOT_FATAL,    0, {NULL}},
    {"sys_voltage",      sys_voltage_init,     BOOT_LV_SERVICE,    BOOT_FATAL,    0, {"storage"}},
    {"eeprom",           eeprom_init,          BOOT_LV_SERVICE,    BOOT_FATAL,    0, {"storage"}},
    {"keyboard",         kb_init,              BOOT_LV_SERVICE,    BOOT_FATAL,    0, {"seq", "ipc_center"}},
    {"vehicle",          vehicle_state_init,   BOOT_LV_SERVICE,    BOOT_FATAL,    0, {"sys_voltage"}},
    /* 钥匙上电检测（PB0）：与 vehicle 相互独立（Q33），无依赖关系 */
    {"acc_state",        acc_state_init,       BOOT_LV_SERVICE,    BOOT_FATAL,    0, {NULL}},
    {"can",              can_init,             BOOT_LV_SERVICE,    BOOT_RETRY,    3, {"ipc_center"}},
    {"can_tx",           can_tx_init,          BOOT_LV_SERVICE,    BOOT_FATAL,    0, {"can"}},
    {"status_tx",        status_tx_init,       BOOT_LV_SERVICE,    BOOT_FATAL,    0, {"can_tx"}},
    {"led_tx",           led_tx_init,          BOOT_LV_SERVICE,    BOOT_FATAL,    0, {"can_tx"}},
    /* 彩色日志（2026-11-10）
     *
     * 为什么放这里：
     *   它只创建一把互斥锁并打印两行提示，不依赖任何业务模块，
     *   也不被任何模块依赖（log_printf 在 s_lock 为 NULL 时
     *   自动退化为无锁输出，不会失败）。
     *
     * 为什么用 BOOT_OPTIONAL：
     *   日志配色纯属调试辅助，创建锁失败也只影响"可能交错"，
     *   绝不能因为它让整机起不来。
     *
     * 为什么在 threads 之前：
     *   线程启动后才会有并发日志，锁要先就绪。 */
    {"log_color",        log_color_init,       BOOT_LV_SERVICE,    BOOT_OPTIONAL, 0, {NULL}},
    {"threads",          boot_create_threads,  BOOT_LV_THREAD,     BOOT_FATAL,    0, {"ipc_center"}},
    {"tick_50ms",        tick_50ms_init,       BOOT_LV_THREAD,     BOOT_FATAL,    0, {"threads"}},
    {"wdt",              wdt_monitor_init,     BOOT_LV_WDT,        BOOT_FATAL,    0, {"threads"}},
};

const uint16_t g_boot_step_count = sizeof(g_boot_steps) / sizeof(g_boot_steps[0]);

/* ============================================================
 * 编译期容量断言（2026-11-10 新增）
 * ------------------------------------------------------------
 * 为什么必须有这一条：
 *   boot.c:14  static boot_result_t s_results[BOOT_STEP_MAX];
 *   而执行循环用 g_boot_step_count 遍历。
 *   若 g_boot_step_count > BOOT_STEP_MAX，s_results 会**越界写**，
 *   踩坏 .bss 里相邻的静态变量 —— 编译器完全不报错，
 *   运行时表现为"某个不相干的模块突然坏了"。
 *
 *   2026-11-10 就真实发生过一次：新增 ctrl_loop 后总条目到 54，
 *   超过当时的 BOOT_STEP_MAX=32，越界写把 ctrl_loop 的实例注册表清零，
 *   表现为 `cl list` 输出 "0 instance(s)"。
 *
 * 有了这条断言，超限时**链接/编译期直接失败**，
 * 而不是运行期静默踩内存。
 * ============================================================ */
typedef char boot_step_count_fits[(sizeof(g_boot_steps) / sizeof(g_boot_steps[0])
                                   <= BOOT_STEP_MAX) ? 1 : -1];
