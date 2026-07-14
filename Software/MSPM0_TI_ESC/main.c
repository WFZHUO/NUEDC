#include "bsp_board.h"
#include "bsp_uart.h"
#include "bsp_encoder.h"
#include "protocol.h"

volatile uint32_t g_ms = 0;
static volatile bool g_1ms_flag = false;
static volatile uint16_t g_led_ms = 0;

int main(void)
{
    BSP_Board_Init();

    while (1) {
        if (g_1ms_flag) {
            g_1ms_flag = false;
            BSP_Encoder_1msTask();
            Protocol_1msTask();
        }
        __WFI();
    }
}

void SysTick_Handler(void)
{
    g_ms++;
    g_1ms_flag = true;

    /* Independent 1 Hz heartbeat: toggle every 500 ms. */
    g_led_ms++;
    if (g_led_ms >= 500U) {
        g_led_ms = 0U;
        DL_GPIO_togglePins(LED_PORT, LED_PIN);
    }
}

void UART0_IRQHandler(void)
{
    BSP_UART_RxIRQHandler();
}

void GROUP1_IRQHandler(void)
{
    BSP_Encoder_GPIOIRQHandler();
}
