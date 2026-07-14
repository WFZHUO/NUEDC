#include "bsp_board.h"
#include "bsp_uart.h"
#include "bsp_motor.h"
#include "bsp_encoder.h"
#include "protocol.h"

static const DL_SYSCTL_SYSPLLConfig gSYSPLLConfig = {
    .inputFreq    = DL_SYSCTL_SYSPLL_INPUT_FREQ_16_32_MHZ,
    .rDivClk2x    = 3,
    .rDivClk1     = 0,
    .rDivClk0     = 0,
    .enableCLK2x  = DL_SYSCTL_SYSPLL_CLK2X_ENABLE,
    .enableCLK1   = DL_SYSCTL_SYSPLL_CLK1_DISABLE,
    .enableCLK0   = DL_SYSCTL_SYSPLL_CLK0_DISABLE,
    .sysPLLMCLK   = DL_SYSCTL_SYSPLL_MCLK_CLK2X,
    .sysPLLRef    = DL_SYSCTL_SYSPLL_REF_SYSOSC,
    .qDiv         = 9,
    .pDiv         = DL_SYSCTL_SYSPLL_PDIV_2,
};

static void board_power_init(void)
{
    DL_GPIO_reset(GPIOA);
    DL_GPIO_reset(GPIOB);

    DL_GPIO_enablePower(GPIOA);
    DL_GPIO_enablePower(GPIOB);

    delay_cycles(POWER_STARTUP_DELAY);
}

static void board_clock_init(void)
{
    DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL_0);
    DL_SYSCTL_setFlashWaitState(DL_SYSCTL_FLASH_WAIT_STATE_2);
    DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);
    DL_SYSCTL_disableHFXT();
    DL_SYSCTL_disableSYSPLL();
    DL_SYSCTL_configSYSPLL((DL_SYSCTL_SYSPLLConfig *)&gSYSPLLConfig);
    DL_SYSCTL_setULPCLKDivider(DL_SYSCTL_ULPCLK_DIV_2);
    DL_SYSCTL_enableMFCLK();
    DL_SYSCTL_setMCLKSource(SYSOSC, HSCLK, DL_SYSCTL_HSCLK_SOURCE_SYSPLL);
}

static void board_led_init(void)
{
    /* PA6 -> LED，原理图为高电平点亮 */
    DL_GPIO_initDigitalOutput(LED_IOMUX);
    DL_GPIO_clearPins(LED_PORT, LED_PIN);
    DL_GPIO_enableOutput(LED_PORT, LED_PIN);
}

void BSP_Board_Init(void)
{
    board_power_init();
    board_clock_init();
    board_led_init();

    BSP_UART_Init();
    BSP_Motor_Init();
    BSP_Encoder_Init();
    Protocol_Init();

    /* 1 ms SysTick */
    DL_SYSTICK_init(APP_SYSTICK_RELOAD);
    DL_SYSTICK_enableInterrupt();
    DL_SYSTICK_enable();

    __enable_irq();
}
