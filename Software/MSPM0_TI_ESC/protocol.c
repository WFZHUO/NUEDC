#include "protocol.h"
#include "crc16.h"
#include "bsp_motor.h"
#include "bsp_encoder.h"
#include "bsp_uart.h"

extern volatile uint32_t g_ms;

static uint8_t s_rx_buf[32];
static uint8_t s_state;
static uint8_t s_index;
static uint8_t s_cmd;
static uint8_t s_len;
static uint32_t s_last_cmd_ms;
static uint8_t s_tx_seq;
static uint16_t s_feedback_ms_acc;
static bool s_has_received_command;
static bool s_command_timeout;

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
    uint8_t index = 0;

    frame[index++] = PROTOCOL_SOF0;
    frame[index++] = PROTOCOL_SOF1;
    frame[index++] = s_tx_seq++;
    frame[index++] = cmd;
    frame[index++] = len;

    for (uint8_t i = 0; i < len; i++) {
        frame[index++] = payload[i];
    }

    uint16_t crc = CRC16_Modbus(frame, index);
    frame[index++] = (uint8_t)(crc & 0xFFU);
    frame[index++] = (uint8_t)(crc >> 8);

    BSP_UART_SendBuffer(frame, index);
}

static void handle_frame(uint8_t cmd, const uint8_t *payload, uint8_t len)
{
    switch (cmd) {
        case PROTOCOL_CMD_SET_PWM:
            if (len == PROTOCOL_SET_PWM_LEN) {
                int16_t pwm_a = get_i16(&payload[0]);
                int16_t pwm_b = get_i16(&payload[2]);

                BSP_Motor_SetPWM2(pwm_a, pwm_b);
                s_last_cmd_ms = g_ms;
                s_has_received_command = true;
                s_command_timeout = false;
            }
            break;

        case PROTOCOL_CMD_STOP:
            if (len == 0U) {
                BSP_Motor_StopAll();
                s_last_cmd_ms = g_ms;
                s_has_received_command = true;
                s_command_timeout = false;
            }
            break;

        case PROTOCOL_CMD_PING:
            if (len == 0U) {
                send_frame(PROTOCOL_CMD_PONG, 0, 0);
            }
            break;

        default:
            break;
    }
}

void Protocol_Init(void)
{
    s_state = 0;
    s_index = 0;
    s_cmd = 0;
    s_len = 0;
    s_last_cmd_ms = 0;
    s_tx_seq = 0;
    s_feedback_ms_acc = 0;
    s_has_received_command = false;
    s_command_timeout = true;
}

void Protocol_ParseByte(uint8_t byte)
{
    switch (s_state) {
        case 0:
            if (byte == PROTOCOL_SOF0) {
                s_rx_buf[0] = byte;
                s_state = 1;
            }
            break;

        case 1:
            if (byte == PROTOCOL_SOF1) {
                s_rx_buf[1] = byte;
                s_state = 2;
                s_index = 2;
            } else {
                s_state = 0;
            }
            break;

        case 2: /* seq */
            s_rx_buf[s_index++] = byte;
            s_state = 3;
            break;

        case 3: /* cmd */
            s_rx_buf[s_index++] = byte;
            s_cmd = byte;
            s_state = 4;
            break;

        case 4: /* len */
            s_rx_buf[s_index++] = byte;
            s_len = byte;

            if (s_len > 16U) {
                s_state = 0;
            } else if (s_len == 0U) {
                s_state = 6;
            } else {
                s_state = 5;
            }
            break;

        case 5: /* payload */
            s_rx_buf[s_index++] = byte;
            if (s_index >= (uint8_t)(5U + s_len)) {
                s_state = 6;
            }
            break;

        case 6: /* crc low */
            s_rx_buf[s_index++] = byte;
            s_state = 7;
            break;

        case 7: { /* crc high */
            s_rx_buf[s_index++] = byte;

            uint16_t received_crc =
                (uint16_t)s_rx_buf[s_index - 2U] |
                ((uint16_t)s_rx_buf[s_index - 1U] << 8);
            uint16_t calculated_crc =
                CRC16_Modbus(s_rx_buf, (uint16_t)(s_index - 2U));

            if (received_crc == calculated_crc) {
                handle_frame(s_cmd, &s_rx_buf[5], s_len);
            }

            s_state = 0;
        } break;

        default:
            s_state = 0;
            break;
    }
}

void Protocol_SendFeedback(void)
{
    uint8_t payload[PROTOCOL_FEEDBACK_LEN];
    uint8_t status = 0U;

    int32_t count_a = BSP_Encoder_GetCount(MOTOR_A);
    int32_t count_b = BSP_Encoder_GetCount(MOTOR_B);
    int16_t speed_a = clamp_i32_to_i16(
        BSP_Encoder_GetSpeedCPS(MOTOR_A));
    int16_t speed_b = clamp_i32_to_i16(
        BSP_Encoder_GetSpeedCPS(MOTOR_B));

    /* Signed multi-byte fields are transmitted big-endian. */
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
    s_command_timeout =
        (!s_has_received_command) ||
        ((g_ms - s_last_cmd_ms) > COMMAND_TIMEOUT_MS);

    if (s_command_timeout) {
        BSP_Motor_StopAll();
    }

    s_feedback_ms_acc++;
    if (s_feedback_ms_acc >= FEEDBACK_PERIOD_MS) {
        s_feedback_ms_acc = 0;
        Protocol_SendFeedback();
    }
}

uint32_t Protocol_GetLastCommandTimeMs(void)
{
    return s_last_cmd_ms;
}
