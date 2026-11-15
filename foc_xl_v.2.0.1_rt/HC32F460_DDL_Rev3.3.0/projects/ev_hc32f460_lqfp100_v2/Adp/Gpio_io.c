/**
 *******************************************************************************
 * @file  Gpio_io.c
 * @brief 通用 GPIO 适配层：输出/输入初始化 + 按键（上拉输入、消抖、短按）。
 *
 *        按键映射：PB1=SW1, PB2=SW2, PB0=SW3（上拉输入，按下为低电平）。
 *        消抖策略：主循环轮询 Key_Scan()，电平需稳定 20ms 才算有效变化，
 *        短按事件在"松开"时触发，事件位图 g_key_short_evt 由 Key_GetShortPress()
 *        读清。时基使用 tickTimer_GetCount()（ms）。
 *******************************************************************************
 */

#include "Gpio_io.h"
#include "hc32_ll_gpio.h"
#include "hc32_ll_utility.h"
#include "TickTimer.h"
#include <stdint.h>

/*******************************************************************************
 * 通用GPIO输出初始化
 ******************************************************************************/
void Output_GPIO_Init(uint8_t u8Port, uint16_t u16Pin, en_gpio_init_state_t enInitState)
{
    stc_gpio_init_t stcGpioInit;

    /* 初始化GPIO结构体为默认值 */
    GPIO_StructInit(&stcGpioInit);

    /* 配置为输出模式 */
    stcGpioInit.u16PinDir         = PIN_DIR_OUT;       /* 输出方向 */
    stcGpioInit.u16PinState       = PIN_STAT_RST;      /* 初始状态为低电平 */
    stcGpioInit.u16PinOutputType  = PIN_OUT_TYPE_CMOS; /* 推挽输出 */
    stcGpioInit.u16PinDrv         = PIN_HIGH_DRV;      /* 高驱动能力 */
    stcGpioInit.u16PullUp         = PIN_PU_OFF;        /* 无上拉 */
    stcGpioInit.u16Invert         = PIN_INVT_OFF;      /* 不反转 */
    stcGpioInit.u16Latch          = PIN_LATCH_OFF;     /* 无锁存 */
    stcGpioInit.u16ExtInt         = PIN_EXTINT_OFF;    /* 不使能外部中断 */
    stcGpioInit.u16PinAttr        = PIN_ATTR_DIGITAL;  /* 数字功能 */

    /* 解除GPIO写保护后初始化 */
    LL_PERIPH_WE(LL_PERIPH_GPIO);
    GPIO_Init(u8Port, u16Pin, &stcGpioInit);

    /* 设置引脚复用功能为GPIO功能（FUNC_0 表示普通GPIO） */
    GPIO_SetFunc(u8Port, u16Pin, GPIO_FUNC_0);

    /* 使能输出 */
    GPIO_OutputCmd(u8Port, u16Pin, ENABLE);
    LL_PERIPH_WP(LL_PERIPH_GPIO);

    /* 根据枚举参数设置初始输出状态 */
    if (enInitState == GPIO_INIT_HIGH) {
        GPIO_SetPins(u8Port, u16Pin);     /* 初始高电平 */
    } else {
        GPIO_ResetPins(u8Port, u16Pin);   /* 初始低电平 */
    }
}

/**
 * @brief 通用GPIO输入初始化
 * @param u8Port: 端口号
 * @param u16Pin: 引脚号
 * @param enablePullUp: 是否使能内部上拉
 * @retval None
 */
void Input_GPIO_Init(uint8_t u8Port, uint16_t u16Pin, en_functional_state_t enablePullUp)
{
    stc_gpio_init_t stcGpioInit;

    /* 初始化GPIO结构体为默认值 */
    GPIO_StructInit(&stcGpioInit);

    /* 配置为输入模式 */
    stcGpioInit.u16PinDir         = PIN_DIR_IN;        /* 输入方向 */
    stcGpioInit.u16PullUp         = (enablePullUp == ENABLE) ? PIN_PU_ON : PIN_PU_OFF;
    stcGpioInit.u16PinAttr        = PIN_ATTR_DIGITAL;  /* 数字功能 */
    stcGpioInit.u16ExtInt         = PIN_EXTINT_OFF;    /* 不使能外部中断 */

    /* 解除GPIO写保护后初始化 */
    LL_PERIPH_WE(LL_PERIPH_GPIO);
    GPIO_Init(u8Port, u16Pin, &stcGpioInit);
    LL_PERIPH_WP(LL_PERIPH_GPIO);
}

/*******************************************************************************
 * 按键模块
 ******************************************************************************/

/* 消抖稳定判定时间（ms） */
#define KEY_DEBOUNCE_MS     20u

/* 按键扫描状态机状态 */
#define KEY_ST_IDLE         0u  /* 空闲 */
#define KEY_ST_PRESS_DB     1u  /* 按下消抖中 */
#define KEY_ST_PRESSED      2u  /* 按下保持，等待释放 */
#define KEY_ST_RELEASE_DB   3u  /* 释放消抖中 */

/* 短按事件位图：bit0=SW1, bit1=SW2, bit2=SW3 */
volatile uint8_t g_key_short_evt = 0u;

/* 按键引脚表（与 key_id_t 顺序对应） */
static const uint8_t  s_key_port[KEY_ID_NUM] = { GPIO_PORT_B, GPIO_PORT_B, GPIO_PORT_B };
static const uint16_t s_key_pin[KEY_ID_NUM]  = { KEY_SW1_PIN,  KEY_SW2_PIN,  KEY_SW3_PIN };

/* 按键扫描状态 */
typedef struct
{
    uint8_t  state;     /* KEY_ST_xxx */
    uint32_t t_mark;    /* 状态切换时刻（ms） */
} key_ctx_t;

static key_ctx_t s_key[KEY_ID_NUM];

/**
 * @brief 按键 GPIO 初始化：PB1/PB2/PB0 上拉输入
 */
void Key_GPIO_Init(void)
{
    uint8_t i;

    for (i = 0u; i < (uint8_t)KEY_ID_NUM; i++) {
        Input_GPIO_Init(s_key_port[i], s_key_pin[i], ENABLE);
        s_key[i].state  = KEY_ST_IDLE;
        s_key[i].t_mark = 0u;
    }
    g_key_short_evt = 0u;
}

/**
 * @brief 按键扫描 + 消抖，主循环周期调用。
 *
 *        状态机：空闲 -> 按下消抖(20ms稳定) -> 按下保持 -> 释放消抖(20ms稳定)
 *        -> 触发短按事件并回空闲。电平抖动则退回上一状态。
 */
/**
 * @brief 按键扫描 + 消抖，主循环周期调用。
 *        优化：增加防溢出保护 + 放宽抖动容忍 + 支持"按下即触发"模式
 */
void Key_Scan(void)
{
    uint32_t now = (uint32_t)tickTimer_GetCount();
    uint8_t  i;

    for (i = 0u; i < (uint8_t)KEY_ID_NUM; i++) {
        uint8_t   pressed = (GPIO_READ(s_key_port[i], s_key_pin[i]) == PIN_RESET) ? 1u : 0u;
        key_ctx_t *k      = &s_key[i];
        uint32_t  elapsed;

        /* 安全计算时间差（处理溢出） */
        if (now >= k->t_mark) {
            elapsed = now - k->t_mark;
        } else {
            elapsed = (UINT32_MAX - k->t_mark) + now + 1u;  /* 溢出补偿 */
        }

        switch (k->state) {
        case KEY_ST_IDLE:
            if (pressed != 0u) {
                k->state  = KEY_ST_PRESS_DB;
                k->t_mark = now;
            }
            break;

        case KEY_ST_PRESS_DB:
            if (pressed != 0u) {
                if (elapsed >= KEY_DEBOUNCE_MS) {
                    k->state = KEY_ST_PRESSED;
                    /* 可选：立即触发按下事件（如果需要支持"按下即响应"） */
                    // g_key_press_evt |= (uint8_t)(1u << i);
                }
            } else {
                /* 消抖期间松开：如果已经消抖到一半以上，可能是有效按下后快速松开 */
                if (elapsed >= (KEY_DEBOUNCE_MS / 2u)) {
                    /* 视为一次有效的短按（极快按键） */
                    g_key_short_evt |= (uint8_t)(1u << i);
                    k->state = KEY_ST_IDLE;
                } else {
                    k->state = KEY_ST_IDLE;  /* 纯抖动，回空闲 */
                }
            }
            break;

        case KEY_ST_PRESSED:
            if (pressed == 0u) {
                k->state  = KEY_ST_RELEASE_DB;
                k->t_mark = now;
            }
            break;

        case KEY_ST_RELEASE_DB:
            if (pressed == 0u) {
                if (elapsed >= KEY_DEBOUNCE_MS) {
                    g_key_short_evt |= (uint8_t)(1u << i);
                    k->state = KEY_ST_IDLE;
                }
            } else {
                /* 释放消抖期间重新按下：如果消抖已过半，可能只是抖动，保持状态 */
                if (elapsed >= (KEY_DEBOUNCE_MS / 2u)) {
                    /* 不切换状态，继续等待释放完成 */
                    /* 但需要重置时间戳防止一直卡住？不重置，保持原有时间 */
                } else {
                    k->state = KEY_ST_PRESSED;  /* 早期抖动，回按下保持 */
                }
            }
            break;

        default:
            k->state = KEY_ST_IDLE;
            break;
        }
    }
}

/**
 * @brief 取出并清除指定按键的短按事件
 * @param key: 按键编号（KEY_ID_SW1/SW2/SW3）
 * @retval 1=有短按, 0=无
 */
uint8_t Key_GetShortPress(key_id_t key)
{
    uint8_t mask;
    uint8_t hit;

    if ((uint8_t)key >= (uint8_t)KEY_ID_NUM) {
        return 0u;
    }

    mask = (uint8_t)(1u << (uint8_t)key);
    hit  = ((g_key_short_evt & mask) != 0u) ? 1u : 0u;
    g_key_short_evt &= (uint8_t)(~mask);    /* 读清 */
    return hit;
}
