#include "protocol.h"
#include "crc16.h"
#include "bsp_encoder.h"
#include "bsp_motor.h"
#include "bsp_uart.h"

extern volatile uint32_t g_ms;

typedef enum {
    PENDING_COMMAND_NONE = 0,
    PENDING_COMMAND_SET_PWM,
    PENDING_COMMAND_STOP,
} Pending_Command_t;

/* Parser state is used only by the UART RX ISR. */
static uint8_t s_rx_buf[32];
static uint8_t s_state;
static uint8_t s_index;
static uint8_t s_cmd;
static uint8_t s_len;
static uint32_t s_last_rx_byte_ms;

/* ISR -> main-loop mailbox. Every asynchronously shared object is volatile. */
static volatile Pending_Command_t s_pending_command;
static volatile int16_t s_pending_pwm_a;
static volatile int16_t s_pending_pwm_b;
static volatile bool s_pong_pending;
static volatile bool s_has_received_command;
static volatile uint32_t s_last_cmd_ms;

/* Main-loop-owned state. All UART TX is performed from the main context. */
static bool s_command_timeout;
static uint8_t s_tx_seq;
static uint16_t s_feedback_ms_acc;

static void put_u16(uint8_t *buf, uint16_t value)
{
    buf[0] = (uint8_t)(value >> 8);
    buf[1] = (uint8_t)value;
}

static void put_i16(uint8_t *buf, int16_t value)
{
    put_u16(buf, (uint16_t)value);
}

static void put_u32(uint8_t *buf, uint32_t value)
{
    buf[0] = (uint8_t)(value >> 24);
    buf[1] = (uint8_t)(value >> 16);
    buf[2] = (uint8_t)(value >> 8);
    buf[3] = (uint8_t)value;
}

static void put_i32(uint8_t *buf, int32_t value)
{
    put_u32(buf, (uint32_t)value);
}

static int16_t get_i16(const uint8_t *buf)
{
    return (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
}

static int16_t clamp_i32_to_i16(int32_t value)
{
    if (value > ENCODER_SPEED_MAX) {
        return ENCODER_SPEED_MAX;
    }
    if (value < ENCODER_SPEED_MIN) {
        return ENCODER_SPEED_MIN;
    }
    return (int16_t)value;
}

static void send_frame(uint8_t cmd, const uint8_t *payload, uint8_t len)
{
    uint8_t frame[32];
    uint8_t index = 0U;

    frame[index++] = PROTOCOL_SOF0;
    frame[index++] = PROTOCOL_SOF1;
    frame[index++] = s_tx_seq++;
    frame[index++] = cmd;
    frame[index++] = len;

    for (uint8_t i = 0U; i < len; i++) {
        frame[index++] = payload[i];
    }

    uint16_t crc = CRC16_Modbus(frame, index);
    frame[index++] = (uint8_t)(crc & 0xFFU);
    frame[index++] = (uint8_t)(crc >> 8);

    BSP_UART_SendBuffer(frame, index);
}

/* Called from the UART RX ISR after a complete frame passes CRC. */
static void accept_frame(uint8_t cmd, const uint8_t *payload, uint8_t len)
{
    switch (cmd) {
        case PROTOCOL_CMD_SET_PWM:
            if (len == PROTOCOL_SET_PWM_LEN) {
                s_pending_pwm_a = get_i16(&payload[0]);
                s_pending_pwm_b = get_i16(&payload[2]);
                s_pending_command = PENDING_COMMAND_SET_PWM;
                s_last_cmd_ms = g_ms;
                s_has_received_command = true;
            }
            break;

        case PROTOCOL_CMD_STOP:
            if (len == 0U) {
                s_pending_command = PENDING_COMMAND_STOP;
                s_last_cmd_ms = g_ms;
                s_has_received_command = true;
            }
            break;

        case PROTOCOL_CMD_PING:
            if (len == 0U) {
                /* Do not transmit from the RX ISR: it could interleave with the
                 * 1 kHz feedback frame being transmitted by the main loop.
                 */
                s_pong_pending = true;
            }
            break;

        default:
            break;
    }
}

void Protocol_Init(void)
{
    s_state = 0U;
    s_index = 0U;
    s_cmd = 0U;
    s_len = 0U;
    s_last_rx_byte_ms = 0U;

    s_pending_command = PENDING_COMMAND_NONE;
    s_pending_pwm_a = 0;
    s_pending_pwm_b = 0;
    s_pong_pending = false;
    s_has_received_command = false;
    s_last_cmd_ms = 0U;

    s_command_timeout = true;
    s_tx_seq = 0U;
    s_feedback_ms_acc = 0U;
}

void Protocol_ParseByte(uint8_t byte)
{
    uint32_t now = g_ms;

    if ((s_state != 0U) &&
        ((uint32_t)(now - s_last_rx_byte_ms) > RX_FRAME_TIMEOUT_MS)) {
        s_state = 0U;
        s_index = 0U;
    }
    s_last_rx_byte_ms = now;

    switch (s_state) {
        case 0U:
            if (byte == PROTOCOL_SOF0) {
                s_rx_buf[0] = byte;
                s_state = 1U;
            }
            break;

        case 1U:
            if (byte == PROTOCOL_SOF1) {
                s_rx_buf[1] = byte;
                s_index = 2U;
                s_state = 2U;
            } else if (byte == PROTOCOL_SOF0) {
                /* A5 A5 5A: keep the second A5 as a possible new frame start. */
                s_rx_buf[0] = byte;
                s_state = 1U;
            } else {
                s_state = 0U;
            }
            break;

        case 2U: /* SEQ */
            s_rx_buf[s_index++] = byte;
            s_state = 3U;
            break;

        case 3U: /* CMD */
            s_rx_buf[s_index++] = byte;
            s_cmd = byte;
            s_state = 4U;
            break;

        case 4U: /* LEN */
            s_rx_buf[s_index++] = byte;
            s_len = byte;

            if (s_len > 16U) {
                s_state = 0U;
                s_index = 0U;
            } else if (s_len == 0U) {
                s_state = 6U;
            } else {
                s_state = 5U;
            }
            break;

        case 5U: /* PAYLOAD */
            s_rx_buf[s_index++] = byte;
            if (s_index >= (uint8_t)(5U + s_len)) {
                s_state = 6U;
            }
            break;

        case 6U: /* CRC low */
            s_rx_buf[s_index++] = byte;
            s_state = 7U;
            break;

        case 7U: { /* CRC high */
            s_rx_buf[s_index++] = byte;

            uint16_t received_crc =
                (uint16_t)s_rx_buf[s_index - 2U] |
                ((uint16_t)s_rx_buf[s_index - 1U] << 8);
            uint16_t calculated_crc =
                CRC16_Modbus(s_rx_buf, (uint16_t)(s_index - 2U));

            if (received_crc == calculated_crc) {
                accept_frame(s_cmd, &s_rx_buf[5], s_len);
            }

            s_state = 0U;
            s_index = 0U;
        } break;

        default:
            s_state = 0U;
            s_index = 0U;
            break;
    }
}

void Protocol_SendFeedback(void)
{
    uint8_t payload[PROTOCOL_FEEDBACK_LEN];
    uint8_t status = 0U;

    int32_t count_a = BSP_Encoder_GetCount(MOTOR_A);
    int32_t count_b = BSP_Encoder_GetCount(MOTOR_B);
    int16_t speed_a =
        clamp_i32_to_i16(BSP_Encoder_GetSpeedCPS(MOTOR_A));
    int16_t speed_b =
        clamp_i32_to_i16(BSP_Encoder_GetSpeedCPS(MOTOR_B));

    put_i32(&payload[0], count_a);
    put_i32(&payload[4], count_b);
    put_i16(&payload[8], speed_a);
    put_i16(&payload[10], speed_b);

    if (s_has_received_command && !s_command_timeout) {
        status |= PROTOCOL_STATUS_CMD_ACTIVE;
    }
    if (s_command_timeout) {
        status |= PROTOCOL_STATUS_TIMEOUT;
    }
    payload[12] = status;

    send_frame(PROTOCOL_CMD_FEEDBACK, payload, PROTOCOL_FEEDBACK_LEN);
}

void Protocol_1msTask(void)
{
    Pending_Command_t pending_command;
    int16_t pending_pwm_a;
    int16_t pending_pwm_b;
    bool pong_pending;

    /* Take an atomic snapshot of the ISR mailbox. */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    pending_command = s_pending_command;
    pending_pwm_a = s_pending_pwm_a;
    pending_pwm_b = s_pending_pwm_b;
    s_pending_command = PENDING_COMMAND_NONE;
    pong_pending = s_pong_pending;
    s_pong_pending = false;
    if (primask == 0U) {
        __enable_irq();
    }

    if (pending_command == PENDING_COMMAND_SET_PWM) {
        BSP_Motor_SetPWM2(pending_pwm_a, pending_pwm_b);
    } else if (pending_command == PENDING_COMMAND_STOP) {
        BSP_Motor_StopAll();
    }

    s_command_timeout =
        (!s_has_received_command) ||
        ((uint32_t)(g_ms - s_last_cmd_ms) > COMMAND_TIMEOUT_MS);

    if (s_command_timeout) {
        BSP_Motor_StopAll();
    }

    if (pong_pending) {
        send_frame(PROTOCOL_CMD_PONG, 0, 0U);
    }

    s_feedback_ms_acc++;
    if (s_feedback_ms_acc >= FEEDBACK_PERIOD_MS) {
        s_feedback_ms_acc = 0U;
        Protocol_SendFeedback();
    }
}

uint32_t Protocol_GetLastCommandTimeMs(void)
{
    return s_last_cmd_ms;
}
