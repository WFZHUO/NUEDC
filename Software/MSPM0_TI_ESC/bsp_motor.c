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

static uint32_t duty_to_compare(uint16_t duty)
{
    if (duty > PWM_PERIOD_TICKS) {
        duty = PWM_PERIOD_TICKS;
    }
    return PWM_PERIOD_TICKS - duty; /* active-high edge-aligned PWM */
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

static void timer_pwm_basic_init(GPTIMER_Regs *inst)
{
    static const DL_Timer_ClockConfig clockConfig = {
        .clockSel    = DL_TIMER_CLOCK_BUSCLK,
        .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
        .prescale    = 0U,
    };
    static const DL_Timer_PWMConfig pwmConfig = {
        .period            = PWM_PERIOD_TICKS,
        .pwmMode           = DL_TIMER_PWM_MODE_EDGE_ALIGN,
        .isTimerWithFourCC = false,
        .startTimer        = DL_TIMER_STOP,
    };

    DL_Timer_setClockConfig(inst, (DL_Timer_ClockConfig *)&clockConfig);
    DL_Timer_initFourCCPWMMode(inst, (DL_Timer_PWMConfig *)&pwmConfig);
}

static void timer_pwm_channel_init(
    GPTIMER_Regs *inst, DL_TIMER_CC_INDEX cc)
{
    DL_Timer_setCaptureCompareOutCtl(inst,
        DL_TIMER_CC_OCTL_INIT_VAL_LOW,
        DL_TIMER_CC_OCTL_INV_OUT_DISABLED,
        DL_TIMER_CC_OCTL_SRC_FUNCVAL,
        cc);
    DL_Timer_setCaptCompUpdateMethod(
        inst, DL_TIMER_CC_UPDATE_METHOD_ZERO_EVT, cc);
    DL_Timer_setCaptureCompareValue(
        inst, PWM_PERIOD_TICKS, cc); /* 0% duty */
}

void BSP_Motor_Init(void)
{
    /* Keep all four DRV8701 devices asleep during configuration. */
    gpio_output_init(MA_EN_IOMUX, MA_EN_PORT, MA_EN_PIN, false);
    gpio_output_init(MB_EN_IOMUX, MB_EN_PORT, MB_EN_PIN, false);
    gpio_output_init(MC_EN_IOMUX, MC_EN_PORT, MC_EN_PIN, false);
    gpio_output_init(MD_EN_IOMUX, MD_EN_PORT, MD_EN_PIN, false);

    gpio_output_init(MA_DIR_IOMUX, MA_DIR_PORT, MA_DIR_PIN, false);
    gpio_output_init(MB_DIR_IOMUX, MB_DIR_PORT, MB_DIR_PIN, false);

    DL_GPIO_initPeripheralOutputFunction(PWM_A_IOMUX, PWM_A_IOMUX_FUNC);
    DL_GPIO_initPeripheralOutputFunction(PWM_B_IOMUX, PWM_B_IOMUX_FUNC);

    DL_TimerG_reset(TIMG8);
    DL_TimerG_reset(TIMG12);
    DL_TimerG_enablePower(TIMG8);
    DL_TimerG_enablePower(TIMG12);
    delay_cycles(POWER_STARTUP_DELAY);

    timer_pwm_basic_init(TIMG8);
    timer_pwm_basic_init(TIMG12);
    timer_pwm_channel_init(PWM_A_INST, PWM_A_CC_INDEX);
    timer_pwm_channel_init(PWM_B_INST, PWM_B_CC_INDEX);

    DL_TimerG_enableClock(TIMG8);
    DL_TimerG_enableClock(TIMG12);
    DL_TimerG_startCounter(TIMG8);
    DL_TimerG_startCounter(TIMG12);

    s_pwm[MOTOR_A] = 0;
    s_pwm[MOTOR_B] = 0;

    /* Wake only A and B. C and D remain asleep permanently. */
    DL_GPIO_setPins(MA_EN_PORT, MA_EN_PIN);
    DL_GPIO_setPins(MB_EN_PORT, MB_EN_PIN);
}

void BSP_Motor_SetPWM(Motor_ID_t id, int16_t pwm)
{
    if (id >= MOTOR_NUM) {
        return;
    }

    pwm = limit_pwm(pwm);
    uint16_t duty = (uint16_t)((pwm >= 0) ? pwm : -pwm);

    if (id == MOTOR_A) {
        if (pwm >= 0) {
            DL_GPIO_clearPins(MA_DIR_PORT, MA_DIR_PIN);
        } else {
            DL_GPIO_setPins(MA_DIR_PORT, MA_DIR_PIN);
        }
        DL_Timer_setCaptureCompareValue(
            PWM_A_INST, duty_to_compare(duty), PWM_A_CC_INDEX);
    } else {
        if (pwm >= 0) {
            DL_GPIO_clearPins(MB_DIR_PORT, MB_DIR_PIN);
        } else {
            DL_GPIO_setPins(MB_DIR_PORT, MB_DIR_PIN);
        }
        DL_Timer_setCaptureCompareValue(
            PWM_B_INST, duty_to_compare(duty), PWM_B_CC_INDEX);
    }

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
