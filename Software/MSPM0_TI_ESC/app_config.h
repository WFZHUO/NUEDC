#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>
#include <stdbool.h>
#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>

/* Board: V4_2_t_xt30, MCU: MSPM0G3507 LQFP64.
 * Role: two-channel brushed motor ESC controlled by an upper-level MCU.
 * Active channels: motor A and motor B only.
 * Motor C and motor D are held in sleep permanently.
 */

#define APP_MCLK_HZ                 (80000000UL)
#define APP_SYSTICK_HZ              (1000UL)
#define APP_SYSTICK_RELOAD          (APP_MCLK_HZ / APP_SYSTICK_HZ)
#define POWER_STARTUP_DELAY         (16U)

#define PWM_FREQ_HZ                 (20000UL)
#define PWM_PERIOD_TICKS            (APP_MCLK_HZ / PWM_FREQ_HZ)  /* 4000 @ 80 MHz */
#define PWM_ABS_MAX                 (4000)

#define FEEDBACK_PERIOD_MS          (1U)      /* 1 kHz feedback */
#define ENCODER_SPEED_WINDOW_MS     (10U)     /* rolling 10 ms speed window, updated every 1 ms */
#define COMMAND_TIMEOUT_MS          (100U)    /* no valid motor command => stop */

/* The current software quadrature decoder counts every valid A/B transition.
 * The exact output-shaft counts per revolution must be verified on the real motor.
 */
#define ENCODER_SPEED_MIN           (-32768)
#define ENCODER_SPEED_MAX           (32767)

#define PROTOCOL_SOF0               (0xA5U)
#define PROTOCOL_SOF1               (0x5AU)
#define PROTOCOL_CMD_SET_PWM        (0x10U)
#define PROTOCOL_CMD_STOP           (0x11U)
#define PROTOCOL_CMD_PING           (0x12U)
#define PROTOCOL_CMD_FEEDBACK       (0x90U)
#define PROTOCOL_CMD_PONG           (0x91U)

/* Compact two-motor protocol payload lengths. */
#define PROTOCOL_SET_PWM_LEN        (4U)  /* pwm_a i16 + pwm_b i16 */
#define PROTOCOL_FEEDBACK_LEN       (13U) /* count_a i32 + count_b i32 + speed_a i16 + speed_b i16 + status u8 */

#define PROTOCOL_STATUS_CMD_ACTIVE  (1U << 0)
#define PROTOCOL_STATUS_TIMEOUT     (1U << 1)

/* Only two logical motors exist in this build. */
typedef enum {
    MOTOR_A = 0,
    MOTOR_B = 1,
    MOTOR_NUM = 2,
} Motor_ID_t;

/* ---------------- Motor GPIO mapping ----------------
 * Motor A: PWM_A PA3,  DIR PA31, EN(nSLEEP) PA4,  ENC_A PB24, ENC_B PA22
 * Motor B: PWM_B PB20, DIR PA18, EN(nSLEEP) PA17, ENC_A PB19, ENC_B PA21
 * Unused C/D nSLEEP pins are still driven low for safety.
 */

#define MA_DIR_PORT                 GPIOA
#define MA_DIR_PIN                  DL_GPIO_PIN_31
#define MA_DIR_IOMUX                IOMUX_PINCM6

#define MA_EN_PORT                  GPIOA
#define MA_EN_PIN                   DL_GPIO_PIN_4
#define MA_EN_IOMUX                 IOMUX_PINCM9

#define MB_DIR_PORT                 GPIOA
#define MB_DIR_PIN                  DL_GPIO_PIN_18
#define MB_DIR_IOMUX                IOMUX_PINCM40

#define MB_EN_PORT                  GPIOA
#define MB_EN_PIN                   DL_GPIO_PIN_17
#define MB_EN_IOMUX                 IOMUX_PINCM39

/* C/D are unused but their driver sleep pins are forced low. */
#define MC_EN_PORT                  GPIOB
#define MC_EN_PIN                   DL_GPIO_PIN_3
#define MC_EN_IOMUX                 IOMUX_PINCM16

#define MD_EN_PORT                  GPIOB
#define MD_EN_PIN                   DL_GPIO_PIN_16
#define MD_EN_IOMUX                 IOMUX_PINCM33

#define LED_PORT                    GPIOA
#define LED_PIN                     DL_GPIO_PIN_6
#define LED_IOMUX                   IOMUX_PINCM11

/* PWM pins/peripherals */
#define PWM_A_INST                  TIMG8
#define PWM_A_CC_INDEX              DL_TIMER_CC_0_INDEX
#define PWM_A_IOMUX                 IOMUX_PINCM8
#define PWM_A_IOMUX_FUNC            IOMUX_PINCM8_PF_TIMG8_CCP0

#define PWM_B_INST                  TIMG12
#define PWM_B_CC_INDEX              DL_TIMER_CC_0_INDEX
#define PWM_B_IOMUX                 IOMUX_PINCM48
#define PWM_B_IOMUX_FUNC            IOMUX_PINCM48_PF_TIMG12_CCP0

/* Encoder pins */
#define MA_ENCA_PORT                GPIOB
#define MA_ENCA_PIN                 DL_GPIO_PIN_24
#define MA_ENCA_IOMUX               IOMUX_PINCM52
#define MA_ENCB_PORT                GPIOA
#define MA_ENCB_PIN                 DL_GPIO_PIN_22
#define MA_ENCB_IOMUX               IOMUX_PINCM47

#define MB_ENCA_PORT                GPIOB
#define MB_ENCA_PIN                 DL_GPIO_PIN_19
#define MB_ENCA_IOMUX               IOMUX_PINCM45
#define MB_ENCB_PORT                GPIOA
#define MB_ENCB_PIN                 DL_GPIO_PIN_21
#define MB_ENCB_IOMUX               IOMUX_PINCM46

#define ENCODER_GPIOA_MASK          (MA_ENCB_PIN | MB_ENCB_PIN)
#define ENCODER_GPIOB_MASK          (MA_ENCA_PIN | MB_ENCA_PIN)

/* UART mapping: board exposes RX on PA11 and TX on PB6.
 * These are not the same UART instance, so RX uses UART0_RX and TX uses UART1_TX.
 */
#define HOST_RX_UART                UART0
#define HOST_RX_IRQN                UART0_INT_IRQn
#define HOST_RX_IOMUX               IOMUX_PINCM22
#define HOST_RX_IOMUX_FUNC          IOMUX_PINCM22_PF_UART0_RX

#define HOST_TX_UART                UART1
#define HOST_TX_IOMUX               IOMUX_PINCM23
#define HOST_TX_IOMUX_FUNC          IOMUX_PINCM23_PF_UART1_TX

#define HOST_UART_CLOCK_HZ          (4000000UL)
#define HOST_UART_BAUD              (1000000UL)

#endif
