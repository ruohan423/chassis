#include "uart_commun.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "can_commun.h"
#include "judge.h"
#include "usart.h"

#define UART_RX_BUFFER_SIZE 1024U
#define UART_CONTROL_FRAME_HEADER 0x1002U
#define UART_SENTRY_FRAME_HEADER  0x1003U
#define UART_CONTROL_FRAME_SIZE   sizeof(uart_receive_cmd_t)
#define UART_SENTRY_FRAME_SIZE    sizeof(uart_receive_sentry_t)
#define UART_JUDGE_PACKET_COUNT 16U
#define UART_JUDGE_PACKET_MAX_DATA_SIZE sizeof(ground_robot_position_t)
#define UART_TX_BUFFER_SIZE (8U + sizeof(uint32_t) + UART_JUDGE_PACKET_MAX_DATA_SIZE + 1U)

typedef struct __attribute__((packed))
{
    uint8_t sof;
    uint8_t length;
    uint16_t id;
} uart_frame_header_t;

typedef struct
{
    uint16_t command_id;
    judge_data_id_t data_id;
    uint16_t offset;
    uint8_t length;
} uart_judge_packet_t;

_Static_assert(sizeof(float) == 4U && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__, "UART 协议要求小端 32 位浮点数");
_Static_assert(sizeof(bool) == 1U, "UART 协议要求 bool 占 1 字节");
_Static_assert(offsetof(uart_receive_cmd_t, data) == 8U, "UART 控制帧数据偏移不匹配");
_Static_assert(offsetof(uart_receive_cmd_t, data.target_yaw) == 20U, "UART 控制帧目标朝向偏移不匹配");
_Static_assert(offsetof(uart_receive_cmd_t, data.chassis_behavior) == 24U, "UART 控制帧行为字段偏移不匹配");
_Static_assert(offsetof(uart_receive_cmd_t, data.gimabal_tar_angle) == 28U, "UART 控制帧云台角偏移不匹配");
_Static_assert(sizeof(uart_receive_cmd_t) == 44U, "UART 控制帧大小不匹配");
_Static_assert(offsetof(uart_receive_sentry_t, data) == 8U, "UART 哨兵帧数据偏移不匹配");
_Static_assert(offsetof(uart_receive_sentry_t, eof) == 12U, "UART 哨兵帧尾偏移不匹配");
_Static_assert(sizeof(uart_receive_sentry_t) == 16U, "UART 哨兵帧大小不匹配");
_Static_assert(offsetof(uart_send_cmd_t, data) == 8U, "UART 状态帧数据偏移不匹配");
_Static_assert(offsetof(uart_send_cmd_t, eof) == 20U, "UART 状态帧尾偏移不匹配");
_Static_assert(sizeof(uart_send_cmd_t) == 24U, "UART 状态帧大小不匹配");
_Static_assert(sizeof(ground_robot_position_t) == 40U, "UART 地面机器人位置载荷大小不匹配");

static uint8_t dma_buffer[UART_RX_BUFFER_SIZE];
uart_rx_t host_uart_rx = {
    .handle = &huart1,
    .buffer = dma_buffer,
    .size = UART_RX_BUFFER_SIZE,
};

static const uart_judge_packet_t judge_packets[UART_JUDGE_PACKET_COUNT] = {
    {0x0001U, JUDGE_DATA_GAME_STATUS, offsetof(judge_data_package_t, game_status), sizeof(game_status_t)},
    {0x0002U, JUDGE_DATA_GAME_RESULT, offsetof(judge_data_package_t, game_result), sizeof(game_result_t)},
    {0x0003U, JUDGE_DATA_ROBOT_HP, offsetof(judge_data_package_t, robot_hp), sizeof(game_robot_hp_t)},
    {0x0101U, JUDGE_DATA_EVENT, offsetof(judge_data_package_t, event), sizeof(event_data_t)},
    {0x0104U, JUDGE_DATA_WARNING, offsetof(judge_data_package_t, warning), sizeof(referee_warning_t)},
    {0x0105U, JUDGE_DATA_DART_INFO, offsetof(judge_data_package_t, dart_info), sizeof(dart_info_t)},
    {0x0201U, JUDGE_DATA_ROBOT_STATUS, offsetof(judge_data_package_t, robot_status), sizeof(robot_status_t)},
    {0x0202U, JUDGE_DATA_POWER_HEAT, offsetof(judge_data_package_t, power_heat), sizeof(power_heat_data_t)},
    {0x0203U, JUDGE_DATA_ROBOT_POS, offsetof(judge_data_package_t, robot_pos), sizeof(robot_pos_t)},
    {0x0204U, JUDGE_DATA_BUFF, offsetof(judge_data_package_t, buff), sizeof(buff_t)},
    {0x0206U, JUDGE_DATA_HURT, offsetof(judge_data_package_t, hurt), sizeof(hurt_data_t)},
    {0x0207U, JUDGE_DATA_SHOOT, offsetof(judge_data_package_t, shoot), sizeof(shoot_data_t)},
    {0x0208U, JUDGE_DATA_PROJECTILE_ALLOWANCE, offsetof(judge_data_package_t, projectile_allowance), sizeof(projectile_allowance_t)},
    {0x0209U, JUDGE_DATA_RFID, offsetof(judge_data_package_t, rfid), sizeof(rfid_status_t)},
    {0x020BU, JUDGE_DATA_GROUND_POSITION, offsetof(judge_data_package_t, ground_position), sizeof(ground_robot_position_t)},
    {0x020DU, JUDGE_DATA_SENTRY_INFO, offsetof(judge_data_package_t, sentry_info), sizeof(sentry_info_t)},
};
_Static_assert(sizeof(judge_packets) / sizeof(judge_packets[0]) == UART_JUDGE_PACKET_COUNT, "UART 裁判数据包数量不匹配");

static uint8_t receive_frame[UART_CONTROL_FRAME_SIZE];
static uint8_t receive_frame_length;
static uint32_t last_receive_tick;
static uart_receive_cmd_t received_command;
static bool command_pending;
static float chassis_yaw_rad;
static float chassis_wz_rad_s;
static uart_send_cmd_t state_packet = {.frame_header = 0x1001U, .eof = 0xA5U};
static uint8_t tx_buffer[UART_TX_BUFFER_SIZE];
static uint32_t judge_sent_ticks[UART_JUDGE_PACKET_COUNT];
static uint16_t judge_sent_mask;
static uint8_t next_judge_packet;
static uint32_t last_send_tick;
static uint32_t last_state_send_tick;
static uint32_t tx_start_tick;
static bool has_sent_packet;
static bool has_sent_state;

void uart_commun_update_imu(float yaw_rad, float yaw_rate_rad_s)
{
    const int32_t previous_lock = osKernelLock();
    if (previous_lock < 0)
    {
        return;
    }

    chassis_yaw_rad = yaw_rad;
    chassis_wz_rad_s = yaw_rate_rad_s;
    if (previous_lock == 0)
    {
        (void)osKernelRestoreLock(previous_lock);
    }
}

void uart_commun_reset(void)
{
    receive_frame_length = 0U;
    taskENTER_CRITICAL();
    command_pending = false;
    taskEXIT_CRITICAL();
}

static bool uart_transmit(uint16_t length, uint32_t now_tick)
{
    if (huart1.gState != HAL_UART_STATE_READY)
    {
        if (now_tick - tx_start_tick >= 50U)
        {
            (void)HAL_UART_AbortTransmit(&huart1);
        }
        return false;
    }

    if (HAL_UART_Transmit_DMA(&huart1, tx_buffer, length) != HAL_OK)
    {
        return false;
    }

    tx_start_tick = now_tick;
    last_send_tick = now_tick;
    has_sent_packet = true;
    return true;
}

static bool uart_send_state(uint32_t now_tick)
{
    static gimbal_data_t gimbal_feedback;
    const bool gimbal_online = can_commun_read_gimbal_data(&gimbal_feedback);
    state_packet.frame_timestamp = now_tick;

    const int32_t previous_lock = osKernelLock();
    if (previous_lock < 0)
    {
        return false;
    }
    state_packet.data.chassis_yaw = chassis_yaw_rad;
    state_packet.data.chassis_wz = chassis_wz_rad_s;
    if (previous_lock == 0)
    {
        (void)osKernelRestoreLock(previous_lock);
    }

    state_packet.data.pitch_status = gimbal_online && gimbal_feedback.pitch_status;
    state_packet.data.energy_mechanis_done = gimbal_online && gimbal_feedback.energy_mechanis_done;
    memcpy(tx_buffer, &state_packet, sizeof(state_packet));
    if (!uart_transmit(sizeof(state_packet), now_tick))
    {
        return false;
    }
    last_state_send_tick = now_tick;
    has_sent_state = true;
    return true;
}

static bool uart_send_judge_data(uint32_t now_tick, uint8_t index)
{
    const uart_judge_packet_t *packet = &judge_packets[index];
    const uint16_t sent_bit = (uint16_t)(1U << index);
    const int32_t previous_lock = osKernelLock();
    if (previous_lock < 0)
    {
        return false;
    }

    const uint32_t update_tick = judge_data.update_tick[packet->data_id];
    if ((judge_data.valid_mask & (1UL << packet->data_id)) == 0U
        || ((judge_sent_mask & sent_bit) != 0U && judge_sent_ticks[index] == update_tick))
    {
        if (previous_lock == 0)
        {
            (void)osKernelRestoreLock(previous_lock);
        }
        return false;
    }

    tx_buffer[0] = 0x5AU;
    tx_buffer[1] = packet->length;
    tx_buffer[2] = (uint8_t)packet->command_id;
    tx_buffer[3] = (uint8_t)(packet->command_id >> 8U);
    memcpy(&tx_buffer[4], &now_tick, sizeof(now_tick));
    memcpy(&tx_buffer[8], (const uint8_t *)&judge_data + packet->offset, packet->length);
    tx_buffer[8U + packet->length] = 0xA5U;
    if (previous_lock == 0)
    {
        (void)osKernelRestoreLock(previous_lock);
    }

    if (!uart_transmit((uint16_t)(sizeof(uart_frame_header_t) + sizeof(now_tick) + packet->length + 1U), now_tick))
    {
        return false;
    }
    judge_sent_ticks[index] = update_tick;
    judge_sent_mask |= sent_bit;
    return true;
}

void uart_send_update(uint32_t now_tick)
{
    if (has_sent_packet && now_tick - last_send_tick < UART_SEND_MIN_INTERVAL_MS)
    {
        return;
    }
    if (huart1.gState != HAL_UART_STATE_READY)
    {
        if (now_tick - tx_start_tick >= 50U)
        {
            (void)HAL_UART_AbortTransmit(&huart1);
        }
        return;
    }
    if (!has_sent_state || now_tick - last_state_send_tick >= UART_STATE_SEND_INTERVAL_MS)
    {
        (void)uart_send_state(now_tick);
        return;
    }

    for (uint8_t checked = 0U; checked < UART_JUDGE_PACKET_COUNT; checked++)
    {
        const uint8_t index = (uint8_t)((next_judge_packet + checked) % UART_JUDGE_PACKET_COUNT);
        if (uart_send_judge_data(now_tick, index))
        {
            next_judge_packet = (uint8_t)((index + 1U) % UART_JUDGE_PACKET_COUNT);
            return;
        }
        if (huart1.gState != HAL_UART_STATE_READY)
        {
            return;
        }
    }
}

static void process_frame(void)
{
    uint32_t frame_header;
    memcpy(&frame_header, receive_frame, sizeof(frame_header));
    const uint8_t frame_size = frame_header == UART_CONTROL_FRAME_HEADER ? UART_CONTROL_FRAME_SIZE
        : frame_header == UART_SENTRY_FRAME_HEADER ? UART_SENTRY_FRAME_SIZE : 0U;
    uint8_t consumed = 1U;

    if (frame_size != 0U && receive_frame_length >= frame_size)
    {
        const bool valid = frame_header == UART_CONTROL_FRAME_HEADER
            ? (receive_frame[offsetof(uart_receive_cmd_t, eof)] == 0xA5U
               && receive_frame[offsetof(uart_receive_cmd_t, data.chassis_behavior)] <= UART_CHASSIS_BEHAVIOR_MAX
               && receive_frame[offsetof(uart_receive_cmd_t, data.pitch_cmd)] <= 1U
               && receive_frame[offsetof(uart_receive_cmd_t, data.gimabal_model)] <= 2U
               && receive_frame[offsetof(uart_receive_cmd_t, data.activate_energy)] <= 1U)
            : receive_frame[offsetof(uart_receive_sentry_t, eof)] == 0xA5U;
        if (valid && frame_header == UART_CONTROL_FRAME_HEADER)
        {
            taskENTER_CRITICAL();
            memcpy(&received_command, receive_frame, sizeof(received_command));
            command_pending = true;
            taskEXIT_CRITICAL();

            gimbal_active_t active = {
                .pitch_cmd = receive_frame[offsetof(uart_receive_cmd_t, data.pitch_cmd)] != 0U,
                .gimabal_model = receive_frame[offsetof(uart_receive_cmd_t, data.gimabal_model)],
                .activate_energy = receive_frame[offsetof(uart_receive_cmd_t, data.activate_energy)] != 0U,
            };
            memcpy(&active.gimabal_tar_angle, &receive_frame[offsetof(uart_receive_cmd_t, data.gimabal_tar_angle)], sizeof(active.gimabal_tar_angle));
            memcpy(&active.gimabal_pat_angle, &receive_frame[offsetof(uart_receive_cmd_t, data.gimabal_pat_angle)], sizeof(active.gimabal_pat_angle));
            if (isfinite(active.gimabal_tar_angle) && isfinite(active.gimabal_pat_angle))
            {
                can_commun_update_gimbal_active(&active);
            }
            consumed = frame_size;
        }
        else if (valid)
        {
            sentry_cmd_t sentry_command;
            memcpy(&sentry_command, &receive_frame[offsetof(uart_receive_sentry_t, data)], sizeof(sentry_command));
            judge_send_sentry(&sentry_command);
            consumed = frame_size;
        }
    }

    receive_frame_length -= consumed;
    memmove(receive_frame, &receive_frame[consumed], receive_frame_length);
}

void uart_receive_process(uint32_t now_tick, uint32_t frame_timeout_ticks)
{
    static uint8_t received_data[UART_RX_BUFFER_SIZE];
    bool overrun;
    const uint16_t length = uart_rx_read(&host_uart_rx, received_data, &overrun);
    if (overrun)
    {
        receive_frame_length = 0U;
    }
    if (receive_frame_length != 0U && now_tick - last_receive_tick >= frame_timeout_ticks)
    {
        receive_frame_length = 0U;
    }

    for (uint16_t index = 0U; index < length; index++)
    {
        receive_frame[receive_frame_length++] = received_data[index];
        while (receive_frame_length >= sizeof(uint32_t))
        {
            uint32_t frame_header;
            memcpy(&frame_header, receive_frame, sizeof(frame_header));
            const uint8_t frame_size = frame_header == UART_CONTROL_FRAME_HEADER ? UART_CONTROL_FRAME_SIZE
                : frame_header == UART_SENTRY_FRAME_HEADER ? UART_SENTRY_FRAME_SIZE : 0U;
            if (frame_size != 0U && receive_frame_length < frame_size)
            {
                break;
            }
            process_frame();
        }
    }
    if (length != 0U)
    {
        last_receive_tick = now_tick;
    }
}

bool uart_read_command(uart_receive_cmd_t *command)
{
    bool available = false;
    taskENTER_CRITICAL();
    if (command_pending)
    {
        *command = received_command;
        command_pending = false;
        available = true;
    }
    taskEXIT_CRITICAL();
    return available;
}
