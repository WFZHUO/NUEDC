#include "bsp_uart.h"
#include "protocol.h"

static const DL_UART_Main_ClockConfig gUartClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_MFCLK,
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

void BSP_UART_Init(void)
{
    DL_UART_Main_reset(HOST_RX_UART);
    DL_UART_Main_reset(HOST_TX_UART);
    DL_UART_Main_enablePower(HOST_RX_UART);
    DL_UART_Main_enablePower(HOST_TX_UART);
    delay_cycles(POWER_STARTUP_DELAY);

    DL_GPIO_initPeripheralInputFunction(HOST_RX_IOMUX, HOST_RX_IOMUX_FUNC);
    DL_GPIO_initPeripheralOutputFunction(HOST_TX_IOMUX, HOST_TX_IOMUX_FUNC);

    DL_UART_Main_setClockConfig(HOST_RX_UART, (DL_UART_Main_ClockConfig *)&gUartClockConfig);
    DL_UART_Main_init(HOST_RX_UART, (DL_UART_Main_Config *)&gUartConfig);
    DL_UART_Main_setOversampling(HOST_RX_UART, DL_UART_OVERSAMPLING_RATE_3X);
    /* 4 MHz / (3 * (1 + 21/64)) = 1.0039 Mbps, error about +0.39%. */
    DL_UART_Main_setBaudRateDivisor(HOST_RX_UART, 1, 21);
    DL_UART_Main_enableInterrupt(HOST_RX_UART, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_SetPriority(HOST_RX_IRQN, 0);
    NVIC_ClearPendingIRQ(HOST_RX_IRQN);
    NVIC_EnableIRQ(HOST_RX_IRQN);
    DL_UART_Main_enable(HOST_RX_UART);

    DL_UART_Main_setClockConfig(HOST_TX_UART, (DL_UART_Main_ClockConfig *)&gUartClockConfig);
    DL_UART_Main_init(HOST_TX_UART, (DL_UART_Main_Config *)&gUartConfig);
    DL_UART_Main_setOversampling(HOST_TX_UART, DL_UART_OVERSAMPLING_RATE_3X);
    /* 4 MHz / (3 * (1 + 21/64)) = 1.0039 Mbps, error about +0.39%. */
    DL_UART_Main_setBaudRateDivisor(HOST_TX_UART, 1, 21);
    DL_UART_Main_enable(HOST_TX_UART);
}

void BSP_UART_SendByte(uint8_t byte)
{
    /*
     * Wait only for TX FIFO space, not for the whole UART to become idle.
     * A timeout prevents a UART fault from blocking the main loop forever.
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
    for (uint16_t i = 0; i < len; i++) {
        BSP_UART_SendByte(buf[i]);
    }
}

void BSP_UART_RxIRQHandler(void)
{
    /*
     * At 1 Mbps several bytes can already be in the RX FIFO when the ISR runs.
     * Drain all available bytes instead of consuming only one byte per entry.
     */
    while (!DL_UART_Main_isRXFIFOEmpty(HOST_RX_UART)) {
        Protocol_ParseByte(DL_UART_Main_receiveData(HOST_RX_UART));
    }
}
