#include "bsp_uart.h"
#include "protocol.h"

/*
 * UART使用40 MHz BUSCLK和16倍过采样。
 * 500 kbaud时分频为5 + 0/64，实际波特率精确为500000 baud。
 * 波特率分频值根据app_config.h中的时钟和波特率自动计算。
 */
#define HOST_UART_OVERSAMPLING_VALUE       (16ULL)
#define HOST_UART_DIVISOR_X64              \
    ((((uint64_t)HOST_UART_CLOCK_HZ * 64ULL) + \
      (((uint64_t)HOST_UART_BAUD * HOST_UART_OVERSAMPLING_VALUE) / 2ULL)) / \
     ((uint64_t)HOST_UART_BAUD * HOST_UART_OVERSAMPLING_VALUE))
#define HOST_UART_INTEGER_DIVISOR          ((uint32_t)(HOST_UART_DIVISOR_X64 / 64ULL))
#define HOST_UART_FRACTIONAL_DIVISOR       ((uint32_t)(HOST_UART_DIVISOR_X64 % 64ULL))


static const DL_UART_Main_ClockConfig gUartClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1,
};

static const DL_UART_Main_Config gUartConfig = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE,
};

static void uart_drain_rx_fifo(void)
{
    while (!DL_UART_Main_isRXFIFOEmpty(HOST_RX_UART)) {
        (void)DL_UART_Main_receiveData(HOST_RX_UART);
    }
}

void BSP_UART_Init(void)
{
    DL_UART_Main_reset(HOST_RX_UART);
    DL_UART_Main_reset(HOST_TX_UART);
    DL_UART_Main_enablePower(HOST_RX_UART);
    DL_UART_Main_enablePower(HOST_TX_UART);
    delay_cycles(POWER_STARTUP_DELAY);

    DL_GPIO_initPeripheralInputFunction(HOST_RX_IOMUX, HOST_RX_IOMUX_FUNC);
    DL_GPIO_initPeripheralOutputFunction(HOST_TX_IOMUX, HOST_TX_IOMUX_FUNC);

    /* RX: UART0 / PA11 */
    DL_UART_Main_setClockConfig(
        HOST_RX_UART, (DL_UART_Main_ClockConfig *)&gUartClockConfig);
    DL_UART_Main_init(HOST_RX_UART, (DL_UART_Main_Config *)&gUartConfig);
    DL_UART_Main_setOversampling(
        HOST_RX_UART, DL_UART_MAIN_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(HOST_RX_UART,
                                    HOST_UART_INTEGER_DIVISOR,
                                    HOST_UART_FRACTIONAL_DIVISOR);

    /* 明确启用FIFO，并让每收到至少1字节就触发RX中断。 */
    DL_UART_Main_enableFIFOs(HOST_RX_UART);
    DL_UART_Main_setRXFIFOThreshold(
        HOST_RX_UART, DL_UART_MAIN_RX_FIFO_LEVEL_ONE_ENTRY);

    /* 16倍过采样时启用多数表决，提高抗噪声能力。 */
    DL_UART_Main_enableMajorityVoting(HOST_RX_UART);

    DL_UART_Main_enableInterrupt(
        HOST_RX_UART,
        DL_UART_MAIN_INTERRUPT_RX |
        DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
        DL_UART_MAIN_INTERRUPT_BREAK_ERROR |
        DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |
        DL_UART_MAIN_INTERRUPT_NOISE_ERROR);

    NVIC_SetPriority(HOST_RX_IRQN, 0);
    NVIC_ClearPendingIRQ(HOST_RX_IRQN);
    NVIC_EnableIRQ(HOST_RX_IRQN);
    DL_UART_Main_enable(HOST_RX_UART);

    /* TX: UART1 / PB6 */
    DL_UART_Main_setClockConfig(
        HOST_TX_UART, (DL_UART_Main_ClockConfig *)&gUartClockConfig);
    DL_UART_Main_init(HOST_TX_UART, (DL_UART_Main_Config *)&gUartConfig);
    DL_UART_Main_setOversampling(
        HOST_TX_UART, DL_UART_MAIN_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(HOST_TX_UART,
                                    HOST_UART_INTEGER_DIVISOR,
                                    HOST_UART_FRACTIONAL_DIVISOR);
    DL_UART_Main_enableFIFOs(HOST_TX_UART);
    DL_UART_Main_enable(HOST_TX_UART);
}

void BSP_UART_SendByte(uint8_t byte)
{
    /*
     * 只等待TX FIFO出现空位，不等待整个UART空闲。
     * 超时保护可避免UART异常时永久阻塞主循环。
     */
    uint32_t timeout = 100000U;

    while (DL_UART_Main_isTXFIFOFull(HOST_TX_UART)) {
        if (--timeout == 0U) {
            return;
        }
    }

    DL_UART_Main_transmitData(HOST_TX_UART, byte);
}

void BSP_UART_SendBuffer(const uint8_t *buf, uint16_t len)
{
    if (buf == 0 || len == 0U) {
        return;
    }

    for (uint16_t i = 0U; i < len; i++) {
        BSP_UART_SendByte(buf[i]);
    }
}

void BSP_UART_RxIRQHandler(void)
{
    for (;;) {
        DL_UART_IIDX interrupt_index =
            DL_UART_Main_getPendingInterrupt(HOST_RX_UART);

        if (interrupt_index == DL_UART_MAIN_IIDX_NO_INTERRUPT) {
            break;
        }

        switch (interrupt_index) {
            case DL_UART_MAIN_IIDX_RX:
            {
                /* 一次中断中取空FIFO，避免高波特率下积压。 */
                while (!DL_UART_Main_isRXFIFOEmpty(HOST_RX_UART)) {
                    Protocol_ParseByte(
                        DL_UART_Main_receiveData(HOST_RX_UART));
                }
                break;
            }

            case DL_UART_MAIN_IIDX_OVERRUN_ERROR:
            {
                DL_UART_Main_clearInterruptStatus(
                    HOST_RX_UART,
                    DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR);
                uart_drain_rx_fifo();
                break;
            }

            case DL_UART_MAIN_IIDX_BREAK_ERROR:
            {
                DL_UART_Main_clearInterruptStatus(
                    HOST_RX_UART,
                    DL_UART_MAIN_INTERRUPT_BREAK_ERROR);
                uart_drain_rx_fifo();
                break;
            }

            case DL_UART_MAIN_IIDX_FRAMING_ERROR:
            {
                DL_UART_Main_clearInterruptStatus(
                    HOST_RX_UART,
                    DL_UART_MAIN_INTERRUPT_FRAMING_ERROR);
                uart_drain_rx_fifo();
                break;
            }

            case DL_UART_MAIN_IIDX_NOISE_ERROR:
            {
                DL_UART_Main_clearInterruptStatus(
                    HOST_RX_UART,
                    DL_UART_MAIN_INTERRUPT_NOISE_ERROR);
                uart_drain_rx_fifo();
                break;
            }

            default:
            {
                /* 未启用的中断源不应进入这里，直接退出避免异常死循环。 */
                return;
            }
        }
    }
}
