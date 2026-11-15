/* hw_gpio_init.c */
#include "hw_gpio_init.h"
#include "hc32_ll.h"
#include "hc32_ll_gpio.h"   /* GPIO_SetDebugPort / GPIO_PIN_DEBUG_JTAG / GPIO_PIN_DEBUG_SWD */
#include "hw_board_pins.h"

void hw_gpio_init_all(void)
{
    stc_gpio_init_t stcGpio;

    LL_PERIPH_WE(LL_PERIPH_GPIO);

    /*=========================================================================
     * PB4 默认是 NJTRST（JTAG TRST 引脚）。
     *   必须先关闭 JTAG（保留 SWD：PA13/PA14），否则 PB4 无法切到
     *   TMR4_3_OWL，表现为一直高电平。
     *=========================================================================*/
    GPIO_SetDebugPort((uint8_t)(GPIO_PIN_DEBUG_JTAG & ~GPIO_PIN_DEBUG_SWD), DISABLE);

    /* ---- PWM 六路：复用功能 ---- */
    GPIO_SetFunc(HW_PIN_PWM_UH_PORT, HW_PIN_PWM_UH_PIN, HW_PIN_PWM_UH_FUNC);
    GPIO_SetFunc(HW_PIN_PWM_UL_PORT, HW_PIN_PWM_UL_PIN, HW_PIN_PWM_UL_FUNC);
    GPIO_SetFunc(HW_PIN_PWM_VH_PORT, HW_PIN_PWM_VH_PIN, HW_PIN_PWM_VH_FUNC);
    GPIO_SetFunc(HW_PIN_PWM_VL_PORT, HW_PIN_PWM_VL_PIN, HW_PIN_PWM_VL_FUNC);
    GPIO_SetFunc(HW_PIN_PWM_WH_PORT, HW_PIN_PWM_WH_PIN, HW_PIN_PWM_WH_FUNC);
    GPIO_SetFunc(HW_PIN_PWM_WL_PORT, HW_PIN_PWM_WL_PIN, HW_PIN_PWM_WL_FUNC);

    /* ---- 编码器 A/B：复用功能 ---- */
    GPIO_SetFunc(HW_PIN_ENC_A_PORT, HW_PIN_ENC_A_PIN, HW_PIN_ENC_A_FUNC);
    GPIO_SetFunc(HW_PIN_ENC_B_PORT, HW_PIN_ENC_B_PIN, HW_PIN_ENC_B_FUNC);

    /* ---- 编码器 Z：数字输入 ---- */
    GPIO_StructInit(&stcGpio);
    stcGpio.u16PinDir  = PIN_DIR_IN;
    stcGpio.u16PinAttr = PIN_ATTR_DIGITAL;
    GPIO_Init(HW_PIN_ENC_Z_PORT, HW_PIN_ENC_Z_PIN, &stcGpio);

    /* ---- ADC 四路：模拟输入 ----
     *   PA3 = 母线电流 / U 相电流, PA4 = W 相, PA5 = V 相, PA6 = 母线电压
     */
    GPIO_StructInit(&stcGpio);
    stcGpio.u16PinAttr = PIN_ATTR_ANALOG;
    GPIO_Init(HW_PIN_ADC_I3_PORT,     HW_PIN_ADC_I3_PIN,     &stcGpio);  /* PA3 */
    GPIO_Init(HW_PIN_ADC_IW_PORT,     HW_PIN_ADC_IW_PIN,     &stcGpio);  /* PA4 */
    GPIO_Init(HW_PIN_ADC_IV_PORT,     HW_PIN_ADC_IV_PIN,     &stcGpio);  /* PA5 */
    GPIO_Init(HW_PIN_ADC_VBUS_PORT,   HW_PIN_ADC_VBUS_PIN,   &stcGpio);  /* PA6 */

    /* ---- 调试翻转：推挽输出 ---- */
    GPIO_StructInit(&stcGpio);
    stcGpio.u16PinDir  = PIN_DIR_OUT;
    stcGpio.u16PinAttr = PIN_ATTR_DIGITAL;
    GPIO_Init(HW_PIN_DEBUG_TOGGLE_PORT, HW_PIN_DEBUG_TOGGLE_PIN, &stcGpio);
    GPIO_SetPins(HW_PIN_DEBUG_TOGGLE_PORT, HW_PIN_DEBUG_TOGGLE_PIN);

    LL_PERIPH_WP(LL_PERIPH_GPIO);
}
