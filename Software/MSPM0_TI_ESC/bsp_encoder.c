#include "bsp_encoder.h"

static volatile int32_t s_count[MOTOR_NUM];
static volatile int32_t s_speed_cps[MOTOR_NUM];
static int32_t s_count_history[MOTOR_NUM][ENCODER_SPEED_WINDOW_MS];
static uint8_t s_history_index;
static uint8_t s_last_ab[MOTOR_NUM];

static const int8_t s_quad_table[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};

static uint8_t read_pin(GPIO_Regs *port, uint32_t pin)
{
    return (DL_GPIO_readPins(port, pin) != 0U) ? 1U : 0U;
}

static uint8_t read_ab(Motor_ID_t id)
{
    if (id == MOTOR_A) {
        return (uint8_t)((read_pin(MA_ENCA_PORT, MA_ENCA_PIN) << 1) |
                         read_pin(MA_ENCB_PORT, MA_ENCB_PIN));
    }

    return (uint8_t)((read_pin(MB_ENCA_PORT, MB_ENCA_PIN) << 1) |
                     read_pin(MB_ENCB_PORT, MB_ENCB_PIN));
}

static void encoder_update(Motor_ID_t id)
{
    uint8_t now = read_ab(id) & 0x03U;
    uint8_t idx = (uint8_t)(((s_last_ab[id] & 0x03U) << 2) | now);
    s_count[id] += s_quad_table[idx];
    s_last_ab[id] = now;
}

static void gpio_input_both_edge(uint32_t iomux)
{
    DL_GPIO_initDigitalInputFeatures(iomux,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_ENABLE,
        DL_GPIO_WAKEUP_DISABLE);
}

void BSP_Encoder_Init(void)
{
    gpio_input_both_edge(MA_ENCA_IOMUX);
    gpio_input_both_edge(MA_ENCB_IOMUX);
    gpio_input_both_edge(MB_ENCA_IOMUX);
    gpio_input_both_edge(MB_ENCB_IOMUX);

    for (uint8_t i = 0; i < MOTOR_NUM; i++) {
        s_count[i] = 0;
        s_speed_cps[i] = 0;
        s_last_ab[i] = read_ab((Motor_ID_t)i);

        for (uint8_t sample = 0; sample < ENCODER_SPEED_WINDOW_MS; sample++) {
            s_count_history[i][sample] = 0;
        }
    }
    s_history_index = 0U;

    DL_GPIO_setUpperPinsPolarity(GPIOA,
        DL_GPIO_PIN_21_EDGE_RISE_FALL | DL_GPIO_PIN_22_EDGE_RISE_FALL);
    DL_GPIO_setUpperPinsPolarity(GPIOB,
        DL_GPIO_PIN_19_EDGE_RISE_FALL | DL_GPIO_PIN_24_EDGE_RISE_FALL);

    DL_GPIO_clearInterruptStatus(GPIOA, ENCODER_GPIOA_MASK);
    DL_GPIO_clearInterruptStatus(GPIOB, ENCODER_GPIOB_MASK);
    DL_GPIO_enableInterrupt(GPIOA, ENCODER_GPIOA_MASK);
    DL_GPIO_enableInterrupt(GPIOB, ENCODER_GPIOB_MASK);

    /* GPIOA and GPIOB share GROUP1 / IRQn 1 on MSPM0G3507. */
    NVIC_SetPriority(GPIOA_INT_IRQn, 1);
    NVIC_ClearPendingIRQ(GPIOA_INT_IRQn);
    NVIC_EnableIRQ(GPIOA_INT_IRQn);
}

void BSP_Encoder_GPIOIRQHandler(void)
{
    uint32_t statusA =
        DL_GPIO_getEnabledInterruptStatus(GPIOA, ENCODER_GPIOA_MASK);
    uint32_t statusB =
        DL_GPIO_getEnabledInterruptStatus(GPIOB, ENCODER_GPIOB_MASK);

    if (((statusA & MA_ENCB_PIN) != 0U) ||
        ((statusB & MA_ENCA_PIN) != 0U)) {
        encoder_update(MOTOR_A);
    }

    if (((statusB & MB_ENCA_PIN) != 0U) ||
        ((statusA & MB_ENCB_PIN) != 0U)) {
        encoder_update(MOTOR_B);
    }

    DL_GPIO_clearInterruptStatus(GPIOA, statusA);
    DL_GPIO_clearInterruptStatus(GPIOB, statusB);
}

void BSP_Encoder_1msTask(void)
{
    /*
     * Update speed every 1 ms, but calculate it over a rolling 10 ms window.
     * This keeps the feedback rate at 1 kHz without the severe quantization of
     * a one-millisecond speed measurement.
     */
    for (uint8_t i = 0; i < MOTOR_NUM; i++) {
        int32_t now = s_count[i];
        int32_t old = s_count_history[i][s_history_index];
        s_count_history[i][s_history_index] = now;
        s_speed_cps[i] =
            ((now - old) * 1000) / (int32_t)ENCODER_SPEED_WINDOW_MS;
    }

    s_history_index++;
    if (s_history_index >= ENCODER_SPEED_WINDOW_MS) {
        s_history_index = 0U;
    }
}

int32_t BSP_Encoder_GetCount(Motor_ID_t id)
{
    return (id < MOTOR_NUM) ? s_count[id] : 0;
}

int32_t BSP_Encoder_GetSpeedCPS(Motor_ID_t id)
{
    return (id < MOTOR_NUM) ? s_speed_cps[id] : 0;
}

void BSP_Encoder_Clear(Motor_ID_t id)
{
    if (id < MOTOR_NUM) {
        s_count[id] = 0;
        s_speed_cps[id] = 0;
        for (uint8_t sample = 0; sample < ENCODER_SPEED_WINDOW_MS; sample++) {
            s_count_history[id][sample] = 0;
        }
    }
}
