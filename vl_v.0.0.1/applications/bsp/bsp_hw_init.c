/**
 * @file    bsp_hw_init.c
 * @brief   MCU 硬件资源初始化调度表实现
 * @note
 *   触发时机：INIT_DEVICE_EXPORT
 *     理由 —— SEGGER RTT console 由 drv_rtt.c 的 INIT_BOARD_EXPORT 完成
 *     （RT_USING_SERIAL_V1 路径），故 INIT_DEVICE 阶段的 rt_kprintf
 *     100% 可见。若放在 INIT_BOARD 阶段，与 console 初始化同段，
 *     先后顺序由链接决定，日志可能丢失。
 *
 *   失败策略：
 *     critical=1 的步骤失败 → 立即 break，避免后续依赖它的步骤
 *     在未定义状态下继续执行；critical=0 失败 → 记录位图后继续，
 *     便于一次上电暴露全部问题。
 */

#include "bsp_hw_init.h"
#include "board_config.h"
#include "bsp_pwm.h"
#include "bsp_hall.h"
#include "bsp_adc.h"
#include "bsp_drv_spi.h"          /* 【A-01，2026-10-28】bsp_dev_spi_init() */
#include "../hw/hw_pwm.h"

#include <rtthread.h>

/* ============================================================
 * 板级引脚步骤适配（rt_err_t(void) → int(void*)）
 * ============================================================ */
static int hw_step_brake_pin(void *arg)
{
    (void)arg;
    return (int)rt_hw_board_brake_init();
}

static int hw_step_pwm_pin(void *arg)
{
    (void)arg;
    return (int)rt_hw_board_pwm_init(CM_TMR4_3);
}

/* 【A-01，2026-10-28】SPI1 初始化接入。
 *
 * 缺陷背景：bsp_dev_spi_init() 全工程零调用者（原仅 bsp_mod_abs_angle.c:560
 * 的注释里出现过）⇒ SPI1 从未执行 SPI_Init()/SPI_Cmd(ENABLE)，
 * 而 KTH78_ReadAngleRaw() 里已经在用 SPI_TransReceive(CM_SPI1, ...)，
 * 等于操作一个未配置的外设 ⇒ 读回值无意义，三编码器链路整条不通。
 *
 * 失败策略取 critical=0（非关键）：
 *   编码器故障不应阻塞整机上电。角度不可用时由上层置 encoder_fault，
 *   拒绝输出角度、禁止依赖角度的动作，而不是让设备无法启动。
 *   但失败会进 g_hw_init_error_map，boot_table 的 hw_check 会打印出来。
 *
 * 顺序约束：PWM/Hall 之后。SPI 与 TIM4、EXTI 无资源共享，
 *   放最后只为保证"先建立安全态（刹车），再开数据通道"这一原则。 */
static int hw_step_spi_encoder(void *arg)
{
    (void)arg;
    bsp_dev_spi_init();
    return 0;      /* bsp_dev_spi_init() 无返回值，内部失败已由日志体现 */
}

/* ADC 外设当前由 bsp_adc.c 的 bsp_adc_init() 承担（内部仍为 TODO 桩），
 * 暂以非关键步骤接入，避免阻塞其他硬件初始化。 */
static int hw_step_adc_unit(void *arg)
{
    (void)arg;
    return (int)bsp_adc_init(&g_hw_adc, RT_NULL);
}

/* ============================================================
 * 硬件初始化表
 * ============================================================
 * 顺序设计原则：
 *   1. 刹车引脚最先 —— 上电即建立锁止电平，符合刹车安全强需求
 *   2. 引脚复用先于外设寄存器 —— TIM4 使能前输出脚必须已切到复用功能
 *   3. 中断最后 —— EXTI/NVIC 打开后随时可能进 ISR，必须在所有
 *      相关资源就绪之后
 * ============================================================ */
const hw_init_step_t g_hw_init_steps[] = {
    /*   name          fn                 arg                      critical */
    { "brake_pin",  hw_step_brake_pin,  RT_NULL,                 0 },
    { "pwm_pin",    hw_step_pwm_pin,    RT_NULL,                 0 },
    /* tim4 的参数为 const 配置表指针；此处去掉 const 仅为适配 void* 签名，
     * 实现侧会立即转回 const hw_pwm_cfg_t*，不会写回。 */
    { "tim4",       bsp_pwm_hw_init,    (void *)&g_swing_pwm_cfg, 1 },
    { "hall_exti",  bsp_hall_hw_init,   RT_NULL,                 1 },
    { "adc_unit",   hw_step_adc_unit,   RT_NULL,                 0 },
    /* 【A-01，2026-10-28】三编码器 SPI1，必须在任何编码器读取之前完成。
     * 见 hw_step_spi_encoder() 上方说明。 */
    { "spi_enc",    hw_step_spi_encoder, RT_NULL,                0 },
};

const uint16_t g_hw_init_step_count =
    (uint16_t)(sizeof(g_hw_init_steps) / sizeof(g_hw_init_steps[0]));

/* 失败位图：bitN = 第 N 步失败 */
uint32_t g_hw_init_error_map = 0U;

/* ============================================================
 * 调度入口
 * ============================================================ */
int bsp_hw_init(void)
{
    uint16_t i;

    g_hw_init_error_map = 0U;
    rt_kprintf("\n\n");

    for (i = 0U; i < g_hw_init_step_count; i++)
    {
        const hw_init_step_t *step = &g_hw_init_steps[i];
        int ret;

        if ((step->fn == RT_NULL) || (step->name == RT_NULL)) {
            g_hw_init_error_map |= (1UL << i);
            rt_kprintf("[HW] step[%u] invalid table entry\n", (unsigned)i);
            continue;
        }

        ret = step->fn(step->arg);

        if (ret != 0) {
            g_hw_init_error_map |= (1UL << i);
            rt_kprintf("[HW] %-10s FAILED (%d)\n", step->name, ret);

            if (step->critical != 0U) {
                rt_kprintf("[HW] critical step failed, abort remaining\n");
                break;
            }
        } else {
            rt_kprintf("[HW] %-10s ok\n", step->name);
        }
    }

    if (g_hw_init_error_map == 0U) {
        rt_kprintf("[HW] all %u steps passed\n\n", (unsigned)g_hw_init_step_count);
        return 0;
    }

    rt_kprintf("[HW] error map = 0x%08X\n\n", (unsigned int)g_hw_init_error_map);
    return -1;
}

/* 触发时机见文件头说明 */
INIT_DEVICE_EXPORT(bsp_hw_init);
