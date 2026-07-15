#include "bsp_motor.h"

static int16_t s_pwm[MOTOR_NUM];

static int16_t limit_pwm(int16_t pwm)
{
    if (pwm > PWM_ABS_MAX) {
        return PWM_ABS_MAX;
    }
    if (pwm < -PWM_ABS_MAX) {
        return -PWM_ABS_MAX;
    }
    return pwm;
}

/* Convert the normalized 0..4000 command into timer compare ticks.
 * Edge-aligned down-count PWM is high from LOAD to CC, therefore:
 *   compare = period - high_ticks
 */
static uint32_t command_to_compare(uint16_t command)
{
    if (command >= (uint16_t)PWM_ABS_MAX) {
        return 0U;
    }

    uint32_t high_ticks =
        ((uint32_t)command * PWM_PERIOD_TICKS + (uint32_t)PWM_ABS_MAX / 2U) /
        (uint32_t)PWM_ABS_MAX;

    if (high_ticks == 0U) {
        /* The corresponding driver is put to sleep for a zero command.
         * Keep CC inside the valid counter range as an additional safeguard.
         */
        return PWM_PERIOD_TICKS - 1U;
    }

    if (high_ticks >= PWM_PERIOD_TICKS) {
        return 0U;
    }

    return PWM_PERIOD_TICKS - high_ticks;
}

static void gpio_output_init(
    uint32_t iomux, GPIO_Regs *port, uint32_t pin, bool init_high)
{
    DL_GPIO_initDigitalOutput(iomux);
    if (init_high) {
        DL_GPIO_setPins(port, pin);
    } else {
        DL_GPIO_clearPins(port, pin);
    }
    DL_GPIO_enableOutput(port, pin);
}

static void timer_pwm_init(
    GPTIMER_Regs *inst, DL_TIMER_CC_INDEX cc)
{
    static const DL_TimerG_ClockConfig clock_config = {
        .clockSel    = DL_TIMER_CLOCK_BUSCLK,
        .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
        .prescale    = 0U,
    };

    static const DL_TimerG_PWMConfig pwm_config = {
        .period            = PWM_PERIOD_TICKS,
        .pwmMode           = DL_TIMER_PWM_MODE_EDGE_ALIGN,
        .isTimerWithFourCC = false,
        .startTimer        = DL_TIMER_STOP,
    };

    DL_TimerG_setClockConfig(inst, (DL_TimerG_ClockConfig *)&clock_config);
    DL_TimerG_initPWMMode(inst, (DL_TimerG_PWMConfig *)&pwm_config);

    /* Match TI SysConfig-generated PWM initialization. */
    DL_TimerG_setCounterControl(inst,
        DL_TIMER_CZC_CCCTL0_ZCOND,
        DL_TIMER_CAC_CCCTL0_ACOND,
        DL_TIMER_CLC_CCCTL0_LCOND);

    DL_TimerG_setCaptureCompareOutCtl(inst,
        DL_TIMER_CC_OCTL_INIT_VAL_LOW,
        DL_TIMER_CC_OCTL_INV_OUT_DISABLED,
        DL_TIMER_CC_OCTL_SRC_FUNCVAL,
        cc);

    DL_TimerG_setCaptCompUpdateMethod(
        inst, DL_TIMER_CC_UPDATE_METHOD_ZERO_EVT, cc);
    DL_TimerG_setCaptureCompareValue(
        inst, PWM_PERIOD_TICKS - 1U, cc);

    DL_TimerG_enableClock(inst);
    DL_TimerG_setCCPDirection(inst, DL_TIMER_CC0_OUTPUT);
    DL_TimerG_enableShadowFeatures(inst);

    DL_TimerG_startCounter(inst);
}

static void motor_set_sleep(Motor_ID_t id, bool sleep)
{
    GPIO_Regs *port = (id == MOTOR_A) ? MA_EN_PORT : MB_EN_PORT;
    uint32_t pin = (id == MOTOR_A) ? MA_EN_PIN : MB_EN_PIN;

    if (sleep) {
        DL_GPIO_clearPins(port, pin);
    } else {
        DL_GPIO_setPins(port, pin);
    }
}

void BSP_Motor_Init(void)
{
    /* Keep all four gate drivers asleep while GPIO and timers are configured. */
    gpio_output_init(MA_EN_IOMUX, MA_EN_PORT, MA_EN_PIN, false);
    gpio_output_init(MB_EN_IOMUX, MB_EN_PORT, MB_EN_PIN, false);
    gpio_output_init(MC_EN_IOMUX, MC_EN_PORT, MC_EN_PIN, false);
    gpio_output_init(MD_EN_IOMUX, MD_EN_PORT, MD_EN_PIN, false);

    gpio_output_init(MA_DIR_IOMUX, MA_DIR_PORT, MA_DIR_PIN, false);
    gpio_output_init(MB_DIR_IOMUX, MB_DIR_PORT, MB_DIR_PIN, false);

    DL_GPIO_initPeripheralOutputFunction(PWM_A_IOMUX, PWM_A_IOMUX_FUNC);
    DL_GPIO_enableOutput(PWM_A_PORT, PWM_A_PIN);
    DL_GPIO_initPeripheralOutputFunction(PWM_B_IOMUX, PWM_B_IOMUX_FUNC);
    DL_GPIO_enableOutput(PWM_B_PORT, PWM_B_PIN);

    DL_TimerG_reset(PWM_A_INST);
    DL_TimerG_reset(PWM_B_INST);
    DL_TimerG_enablePower(PWM_A_INST);
    DL_TimerG_enablePower(PWM_B_INST);
    delay_cycles(POWER_STARTUP_DELAY);

    timer_pwm_init(PWM_A_INST, PWM_A_CC_INDEX);
    timer_pwm_init(PWM_B_INST, PWM_B_CC_INDEX);

    s_pwm[MOTOR_A] = 0;
    s_pwm[MOTOR_B] = 0;

    /* A/B wake only when a non-zero command is applied. C/D remain asleep. */
    motor_set_sleep(MOTOR_A, true);
    motor_set_sleep(MOTOR_B, true);
}

void BSP_Motor_SetPWM(Motor_ID_t id, int16_t pwm)
{
    if (id >= MOTOR_NUM) {
        return;
    }

    pwm = limit_pwm(pwm);

    GPTIMER_Regs *inst = (id == MOTOR_A) ? PWM_A_INST : PWM_B_INST;
    DL_TIMER_CC_INDEX cc =
        (id == MOTOR_A) ? PWM_A_CC_INDEX : PWM_B_CC_INDEX;
    GPIO_Regs *dir_port = (id == MOTOR_A) ? MA_DIR_PORT : MB_DIR_PORT;
    uint32_t dir_pin = (id == MOTOR_A) ? MA_DIR_PIN : MB_DIR_PIN;

    if (pwm == 0) {
        /* Disable the gate driver first, then prepare a safe near-zero compare. */
        motor_set_sleep(id, true);
        DL_TimerG_setCaptureCompareValue(
            inst, PWM_PERIOD_TICKS - 1U, cc);
        s_pwm[id] = 0;
        return;
    }

    if (pwm > 0) {
        DL_GPIO_clearPins(dir_port, dir_pin);
    } else {
        DL_GPIO_setPins(dir_port, dir_pin);
    }

    uint16_t magnitude = (uint16_t)((pwm > 0) ? pwm : -pwm);
    DL_TimerG_setCaptureCompareValue(
        inst, command_to_compare(magnitude), cc);

    /* Compare and direction are configured before the driver is awakened. */
    motor_set_sleep(id, false);
    s_pwm[id] = pwm;
}

void BSP_Motor_SetPWM2(int16_t pwm_a, int16_t pwm_b)
{
    BSP_Motor_SetPWM(MOTOR_A, pwm_a);
    BSP_Motor_SetPWM(MOTOR_B, pwm_b);
}

void BSP_Motor_StopAll(void)
{
    BSP_Motor_SetPWM2(0, 0);
}

int16_t BSP_Motor_GetPWM(Motor_ID_t id)
{
    return (id < MOTOR_NUM) ? s_pwm[id] : 0;
}
