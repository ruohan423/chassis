#include "judge.h"

#include <stddef.h>
#include <string.h>

#include "cmsis_os2.h"

extern osSemaphoreId_t JudgeSemHandle;

_Static_assert(sizeof(game_status_t) == 11U, "game_status_t size");
_Static_assert(sizeof(game_result_t) == 1U, "game_result_t size");
_Static_assert(sizeof(game_robot_hp_t) == 24U, "game_robot_hp_t size");
_Static_assert(sizeof(event_data_t) == 8U, "event_data_t size");
_Static_assert(sizeof(referee_warning_t) == 3U, "judge_warning_t size");
_Static_assert(sizeof(judge_dart_info_t) == 3U, "judge_dart_info_t size");
_Static_assert(sizeof(robot_status_t) == 17U, "robot_status_t size");
_Static_assert(sizeof(power_heat_data_t) == 14U, "power_heat_data_t size");
_Static_assert(sizeof(robot_pos_t) == 12U, "robot_pos_t size");
_Static_assert(sizeof(buff_t) == 7U, "buff_t size");
_Static_assert(sizeof(hurt_data_t) == 7U, "hurt_data_t size");
_Static_assert(sizeof(shoot_data_t) == 7U, "shoot_data_t size");
_Static_assert(sizeof(projectile_allowance_t) == 8U, "projectile_allowance_t size");
_Static_assert(sizeof(rfid_status_t) == 6U, "rfid_status_t size");
_Static_assert(sizeof(ground_robot_position_t) == 40U, "ground_robot_position_t size");
_Static_assert(sizeof(sentry_info_t) == 14U, "sentry_info_t size");
_Static_assert(sizeof(sentry_cmd_t) == 4U, "sentry_cmd_t size");
_Static_assert(sizeof(robot_interaction_data_t) == 118U, "robot_interaction_data_t size");
_Static_assert(sizeof(map_command_t) == 12U, "map_command_t size");
_Static_assert(JUDGE_DATA_COUNT <= 32U, "valid_mask capacity");

typedef struct
{
    uint16_t command;
    uint16_t offset;
    uint16_t size;
    judge_data_id_t id;
} judge_field_t;

/* 表中只描述协议字段，不进行行为模式映射。 */
#define JUDGE_FIELD(command, field, id) {command, offsetof(judge_data_package_t, field), sizeof(((judge_data_package_t *)0)->field), id}
static const judge_field_t commands[] = {
    JUDGE_FIELD(0x0001U, game_status, JUDGE_DATA_GAME_STATUS),
    JUDGE_FIELD(0x0002U, game_result, JUDGE_DATA_GAME_RESULT),
    JUDGE_FIELD(0x0003U, robot_hp, JUDGE_DATA_ROBOT_HP),
    JUDGE_FIELD(0x0101U, event, JUDGE_DATA_EVENT),
    JUDGE_FIELD(0x0104U, warning, JUDGE_DATA_WARNING),
    JUDGE_FIELD(0x0105U, dart_info, JUDGE_DATA_DART_INFO),
    JUDGE_FIELD(0x0201U, robot_status, JUDGE_DATA_ROBOT_STATUS),
    JUDGE_FIELD(0x0202U, power_heat, JUDGE_DATA_POWER_HEAT),
    JUDGE_FIELD(0x0203U, robot_pos, JUDGE_DATA_ROBOT_POS),
    JUDGE_FIELD(0x0204U, buff, JUDGE_DATA_BUFF),
    JUDGE_FIELD(0x0206U, hurt, JUDGE_DATA_HURT),
    JUDGE_FIELD(0x0207U, shoot, JUDGE_DATA_SHOOT),
    JUDGE_FIELD(0x0208U, projectile_allowance, JUDGE_DATA_PROJECTILE_ALLOWANCE),
    JUDGE_FIELD(0x0209U, rfid, JUDGE_DATA_RFID),
    JUDGE_FIELD(0x020BU, ground_position, JUDGE_DATA_GROUND_POSITION),
    JUDGE_FIELD(0x020DU, sentry_info, JUDGE_DATA_SENTRY_INFO),
    JUDGE_FIELD(0x0301U, interaction, JUDGE_DATA_INTERACTION),
    JUDGE_FIELD(0x0303U, map_command, JUDGE_DATA_MAP_COMMAND),
};
#undef JUDGE_FIELD

/* 查表值与协议附录一一致，CRC8/CRC16 初值分别为 0xFF/0xFFFF。 */
static const uint8_t crc8_table[256] = {
    0x00U, 0x5EU, 0xBCU, 0xE2U, 0x61U, 0x3FU, 0xDDU, 0x83U, 0xC2U, 0x9CU, 0x7EU, 0x20U, 0xA3U, 0xFDU, 0x1FU, 0x41U,
    0x9DU, 0xC3U, 0x21U, 0x7FU, 0xFCU, 0xA2U, 0x40U, 0x1EU, 0x5FU, 0x01U, 0xE3U, 0xBDU, 0x3EU, 0x60U, 0x82U, 0xDCU,
    0x23U, 0x7DU, 0x9FU, 0xC1U, 0x42U, 0x1CU, 0xFEU, 0xA0U, 0xE1U, 0xBFU, 0x5DU, 0x03U, 0x80U, 0xDEU, 0x3CU, 0x62U,
    0xBEU, 0xE0U, 0x02U, 0x5CU, 0xDFU, 0x81U, 0x63U, 0x3DU, 0x7CU, 0x22U, 0xC0U, 0x9EU, 0x1DU, 0x43U, 0xA1U, 0xFFU,
    0x46U, 0x18U, 0xFAU, 0xA4U, 0x27U, 0x79U, 0x9BU, 0xC5U, 0x84U, 0xDAU, 0x38U, 0x66U, 0xE5U, 0xBBU, 0x59U, 0x07U,
    0xDBU, 0x85U, 0x67U, 0x39U, 0xBAU, 0xE4U, 0x06U, 0x58U, 0x19U, 0x47U, 0xA5U, 0xFBU, 0x78U, 0x26U, 0xC4U, 0x9AU,
    0x65U, 0x3BU, 0xD9U, 0x87U, 0x04U, 0x5AU, 0xB8U, 0xE6U, 0xA7U, 0xF9U, 0x1BU, 0x45U, 0xC6U, 0x98U, 0x7AU, 0x24U,
    0xF8U, 0xA6U, 0x44U, 0x1AU, 0x99U, 0xC7U, 0x25U, 0x7BU, 0x3AU, 0x64U, 0x86U, 0xD8U, 0x5BU, 0x05U, 0xE7U, 0xB9U,
    0x8CU, 0xD2U, 0x30U, 0x6EU, 0xEDU, 0xB3U, 0x51U, 0x0FU, 0x4EU, 0x10U, 0xF2U, 0xACU, 0x2FU, 0x71U, 0x93U, 0xCDU,
    0x11U, 0x4FU, 0xADU, 0xF3U, 0x70U, 0x2EU, 0xCCU, 0x92U, 0xD3U, 0x8DU, 0x6FU, 0x31U, 0xB2U, 0xECU, 0x0EU, 0x50U,
    0xAFU, 0xF1U, 0x13U, 0x4DU, 0xCEU, 0x90U, 0x72U, 0x2CU, 0x6DU, 0x33U, 0xD1U, 0x8FU, 0x0CU, 0x52U, 0xB0U, 0xEEU,
    0x32U, 0x6CU, 0x8EU, 0xD0U, 0x53U, 0x0DU, 0xEFU, 0xB1U, 0xF0U, 0xAEU, 0x4CU, 0x12U, 0x91U, 0xCFU, 0x2DU, 0x73U,
    0xCAU, 0x94U, 0x76U, 0x28U, 0xABU, 0xF5U, 0x17U, 0x49U, 0x08U, 0x56U, 0xB4U, 0xEAU, 0x69U, 0x37U, 0xD5U, 0x8BU,
    0x57U, 0x09U, 0xEBU, 0xB5U, 0x36U, 0x68U, 0x8AU, 0xD4U, 0x95U, 0xCBU, 0x29U, 0x77U, 0xF4U, 0xAAU, 0x48U, 0x16U,
    0xE9U, 0xB7U, 0x55U, 0x0BU, 0x88U, 0xD6U, 0x34U, 0x6AU, 0x2BU, 0x75U, 0x97U, 0xC9U, 0x4AU, 0x14U, 0xF6U, 0xA8U,
    0x74U, 0x2AU, 0xC8U, 0x96U, 0x15U, 0x4BU, 0xA9U, 0xF7U, 0xB6U, 0xE8U, 0x0AU, 0x54U, 0xD7U, 0x89U, 0x6BU, 0x35U,
};

static const uint16_t crc16_table[256] = {
    0x0000U, 0x1189U, 0x2312U, 0x329BU, 0x4624U, 0x57ADU, 0x6536U, 0x74BFU,
    0x8C48U, 0x9DC1U, 0xAF5AU, 0xBED3U, 0xCA6CU, 0xDBE5U, 0xE97EU, 0xF8F7U,
    0x1081U, 0x0108U, 0x3393U, 0x221AU, 0x56A5U, 0x472CU, 0x75B7U, 0x643EU,
    0x9CC9U, 0x8D40U, 0xBFDBU, 0xAE52U, 0xDAEDU, 0xCB64U, 0xF9FFU, 0xE876U,
    0x2102U, 0x308BU, 0x0210U, 0x1399U, 0x6726U, 0x76AFU, 0x4434U, 0x55BDU,
    0xAD4AU, 0xBCC3U, 0x8E58U, 0x9FD1U, 0xEB6EU, 0xFAE7U, 0xC87CU, 0xD9F5U,
    0x3183U, 0x200AU, 0x1291U, 0x0318U, 0x77A7U, 0x662EU, 0x54B5U, 0x453CU,
    0xBDCBU, 0xAC42U, 0x9ED9U, 0x8F50U, 0xFBEFU, 0xEA66U, 0xD8FDU, 0xC974U,
    0x4204U, 0x538DU, 0x6116U, 0x709FU, 0x0420U, 0x15A9U, 0x2732U, 0x36BBU,
    0xCE4CU, 0xDFC5U, 0xED5EU, 0xFCD7U, 0x8868U, 0x99E1U, 0xAB7AU, 0xBAF3U,
    0x5285U, 0x430CU, 0x7197U, 0x601EU, 0x14A1U, 0x0528U, 0x37B3U, 0x263AU,
    0xDECDU, 0xCF44U, 0xFDDFU, 0xEC56U, 0x98E9U, 0x8960U, 0xBBFBU, 0xAA72U,
    0x6306U, 0x728FU, 0x4014U, 0x519DU, 0x2522U, 0x34ABU, 0x0630U, 0x17B9U,
    0xEF4EU, 0xFEC7U, 0xCC5CU, 0xDDD5U, 0xA96AU, 0xB8E3U, 0x8A78U, 0x9BF1U,
    0x7387U, 0x620EU, 0x5095U, 0x411CU, 0x35A3U, 0x242AU, 0x16B1U, 0x0738U,
    0xFFCFU, 0xEE46U, 0xDCDDU, 0xCD54U, 0xB9EBU, 0xA862U, 0x9AF9U, 0x8B70U,
    0x8408U, 0x9581U, 0xA71AU, 0xB693U, 0xC22CU, 0xD3A5U, 0xE13EU, 0xF0B7U,
    0x0840U, 0x19C9U, 0x2B52U, 0x3ADBU, 0x4E64U, 0x5FEDU, 0x6D76U, 0x7CFFU,
    0x9489U, 0x8500U, 0xB79BU, 0xA612U, 0xD2ADU, 0xC324U, 0xF1BFU, 0xE036U,
    0x18C1U, 0x0948U, 0x3BD3U, 0x2A5AU, 0x5EE5U, 0x4F6CU, 0x7DF7U, 0x6C7EU,
    0xA50AU, 0xB483U, 0x8618U, 0x9791U, 0xE32EU, 0xF2A7U, 0xC03CU, 0xD1B5U,
    0x2942U, 0x38CBU, 0x0A50U, 0x1BD9U, 0x6F66U, 0x7EEFU, 0x4C74U, 0x5DFDU,
    0xB58BU, 0xA402U, 0x9699U, 0x8710U, 0xF3AFU, 0xE226U, 0xD0BDU, 0xC134U,
    0x39C3U, 0x284AU, 0x1AD1U, 0x0B58U, 0x7FE7U, 0x6E6EU, 0x5CF5U, 0x4D7CU,
    0xC60CU, 0xD785U, 0xE51EU, 0xF497U, 0x8028U, 0x91A1U, 0xA33AU, 0xB2B3U,
    0x4A44U, 0x5BCDU, 0x6956U, 0x78DFU, 0x0C60U, 0x1DE9U, 0x2F72U, 0x3EFBU,
    0xD68DU, 0xC704U, 0xF59FU, 0xE416U, 0x90A9U, 0x8120U, 0xB3BBU, 0xA232U,
    0x5AC5U, 0x4B4CU, 0x79D7U, 0x685EU, 0x1CE1U, 0x0D68U, 0x3FF3U, 0x2E7AU,
    0xE70EU, 0xF687U, 0xC41CU, 0xD595U, 0xA12AU, 0xB0A3U, 0x8238U, 0x93B1U,
    0x6B46U, 0x7ACFU, 0x4854U, 0x59DDU, 0x2D62U, 0x3CEBU, 0x0E70U, 0x1FF9U,
    0xF78FU, 0xE606U, 0xD49DU, 0xC514U, 0xB1ABU, 0xA022U, 0x92B9U, 0x8330U,
    0x7BC7U, 0x6A4EU, 0x58D5U, 0x495CU, 0x3DE3U, 0x2C6AU, 0x1EF1U, 0x0F78U,
};

judge_data_package_t judge_data;
static uint8_t receive_buffer[JUDGE_FRAME_MAX_SIZE];
static uint16_t received_length;
static judge_received_command_t command_queue[JUDGE_COMMAND_QUEUE_DEPTH];
static uint8_t command_head;
static uint8_t command_count;
static bool command_lost;
static judge_tx_request_t tx_queue[JUDGE_TX_QUEUE_DEPTH];
static uint8_t tx_head;
static uint8_t tx_count;
judge_tx_status_t judge_tx_status = {
    .result = JUDGE_TX_NOT_READY,
    .byte_limit_per_second = JUDGE_TX_DEFAULT_BYTE_LIMIT,
};



static uint8_t crc8(const uint8_t *data, uint16_t length)
{
    uint8_t crc = 0xFFU;
    for (uint16_t index = 0U; index < length; index++)
    {
        crc = crc8_table[crc ^ data[index]];
    }
    return crc;
}

static uint16_t crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    for (uint16_t index = 0U; index < length; index++)
    {
        crc = (uint16_t)((crc >> 8) ^ crc16_table[(crc ^ data[index]) & 0xFFU]);
    }
    return crc;
}


static bool dispatch_command(uint16_t cmd_id, const uint8_t *data, uint16_t length, uint8_t sequence, uint32_t now_tick)
{
    for (uint16_t index = 0U; index < sizeof(commands) / sizeof(commands[0]); index++)
    {
        const judge_field_t *field = &commands[index];
        if (cmd_id != field->command)
        {
            continue;
        }
        if (field->id == JUDGE_DATA_INTERACTION)
        {
            if (length < 6U || length > field->size)
            {
                return false;
            }
        }
        else if (length != field->size)
        {
            return false;
        }

        const int32_t previous_lock = osKernelLock();
        if (field->id == JUDGE_DATA_INTERACTION || field->id == JUDGE_DATA_MAP_COMMAND)
        {
            if (command_count == JUDGE_COMMAND_QUEUE_DEPTH)
            {
                command_lost = true;
                if (previous_lock == 0)
                {
                    (void)osKernelUnlock();
                }
                return true;
            }
            judge_received_command_t *command = &command_queue[(command_head + command_count) % JUDGE_COMMAND_QUEUE_DEPTH];
            command->cmd_id = cmd_id;
            command->data_length = length;
            command->sequence = sequence;
            command->received_tick = now_tick;
            memcpy(command->data, data, length);
            command_count++;
        }

        if (field->id == JUDGE_DATA_INTERACTION)
        {
            memset(&judge_data.interaction, 0, sizeof(judge_data.interaction));
            judge_data.interaction_length = length - 6U;
        }
        memcpy((uint8_t *)&judge_data + field->offset, data, length);
        judge_data.valid_mask |= 1UL << field->id;
        judge_data.update_tick[field->id] = now_tick;
        if (previous_lock == 0)
        {
            (void)osKernelUnlock();
        }
        return true;
    }
    return false;
}



bool judge_receive(const uint8_t *data, uint16_t length, uint32_t now_tick)
{
    bool has_valid_frame = false;

    for (uint16_t index = 0U; index < length; index++)
    {
        receive_buffer[received_length++] = data[index];
        while (received_length >= JUDGE_FRAME_HEADER_SIZE)
        {
            const uint16_t data_length = (uint16_t)(receive_buffer[1] | ((uint16_t)receive_buffer[2] << 8));
            uint16_t consumed = 1U;

            if (receive_buffer[0] == 0xA5U && data_length <= JUDGE_FRAME_DATA_SIZE
                && crc8(receive_buffer, JUDGE_FRAME_HEADER_SIZE - 1U) == receive_buffer[4])
            {
                const uint16_t frame_length = data_length + JUDGE_FRAME_OVERHEAD;
                if (received_length < frame_length)
                {
                    break;
                }

                const uint16_t frame_crc16 = (uint16_t)(receive_buffer[frame_length - 2U] | ((uint16_t)receive_buffer[frame_length - 1U] << 8));
                if (crc16(receive_buffer, frame_length - 2U) == frame_crc16)
                {
                    /* 分发同步复制载荷，不保留指针；完成后才能移动接收缓冲区。 */
                    if (dispatch_command((uint16_t)(receive_buffer[5] | ((uint16_t)receive_buffer[6] << 8)),
                                         &receive_buffer[7], data_length, receive_buffer[3], now_tick))
                    {
                        has_valid_frame = true;
                    }
                    consumed = frame_length;
                }
            }
            /* 坏帧只前移一个字节，保留其中可能包含的下一帧同步字。 */
            received_length -= consumed;
            memmove(receive_buffer, receive_buffer + consumed, received_length);
        }
    }
    return has_valid_frame;
}

void judge_reset_parser(void)
{
    received_length = 0U;
}

void judge_clear(void)
{
    received_length = 0U;
    const int32_t previous_lock = osKernelLock();
    memset(&judge_data, 0, sizeof(judge_data));
    if (command_count != 0U)
    {
        command_lost = true;
    }
    command_head = 0U;
    command_count = 0U;
    tx_head = 0U;
    tx_count = 0U;
    if (previous_lock == 0)
    {
        (void)osKernelUnlock();
    }
}

bool judge_data_is_fresh(const judge_data_package_t *data, judge_data_id_t id, uint32_t now_tick)
{
    const uint32_t timeout_ticks = (osKernelGetTickFreq() * JUDGE_LINK_TIMEOUT_MS + 999U) / 1000U;
    return (data->valid_mask & (1UL << id)) != 0U && now_tick - data->update_tick[id] < timeout_ticks;
}

judge_command_result_t judge_read_command(judge_received_command_t *command)
{
    judge_command_result_t result = JUDGE_COMMAND_EMPTY;
    const int32_t previous_lock = osKernelLock();
    if (command_lost)
    {
        command_lost = false;
        result = JUDGE_COMMAND_LOST;
    }
    else if (command_count > 0U)
    {
        *command = command_queue[command_head];
        command_head = (command_head + 1U) % JUDGE_COMMAND_QUEUE_DEPTH;
        command_count--;
        result = JUDGE_COMMAND_READY;
    }
    if (previous_lock == 0)
    {
        (void)osKernelUnlock();
    }
    return result;
}

judge_tx_result_t judge_send_interaction(uint16_t data_cmd_id, uint16_t receiver_id, const uint8_t *data, uint16_t length)
{
    const bool sentry_command = data_cmd_id == 0x0120U;
    if ((length > 0U && data == NULL) || length > JUDGE_INTERACTION_DATA_SIZE
        || (sentry_command && (receiver_id != 0x8080U || length != sizeof(sentry_cmd_t)))
        || (!sentry_command && (data_cmd_id < 0x0200U || data_cmd_id > 0x02FFU)))
    {
        return JUDGE_TX_INVALID;
    }
    if (JudgeSemHandle == NULL)
    {
        return JUDGE_TX_NOT_READY;
    }

    const int32_t previous_lock = osKernelLock();
    if (previous_lock < 0)
    {
        return JUDGE_TX_NOT_READY;
    }
    const uint32_t now_tick = osKernelGetTickCount();
    const uint16_t sender_id = judge_data.robot_status.robot_id;
    const uint16_t peer_id = receiver_id >= 100U ? receiver_id - 100U : receiver_id;
    judge_tx_result_t result;

    if (!judge_data_is_fresh(&judge_data, JUDGE_DATA_ROBOT_STATUS, now_tick))
    {
        result = JUDGE_TX_OFFLINE;
    }
    else if ((sender_id != 7U && sender_id != 107U)
        || (!sentry_command && ((sender_id >= 100U) != (receiver_id >= 100U) || peer_id == 0U || peer_id == 2U || peer_id > 9U)))
    {
        result = JUDGE_TX_INVALID;
    }
    else if (judge_tx_status.result == JUDGE_TX_CONFIG_ERROR || judge_tx_status.result == JUDGE_TX_NOT_READY)
    {
        result = judge_tx_status.result;
    }
    else if (tx_count == JUDGE_TX_QUEUE_DEPTH)
    {
        result = JUDGE_TX_QUEUE_FULL;
    }
    else
    {
        judge_tx_request_t *request = &tx_queue[(tx_head + tx_count) % JUDGE_TX_QUEUE_DEPTH];
        request->data_cmd_id = data_cmd_id;
        request->sender_id = sender_id;
        request->receiver_id = receiver_id;
        if (length > 0U)
        {
            memcpy(request->data, data, length);
        }
        request->data_length = length;
        request->queued_tick = now_tick;
        tx_count++;
        result = JUDGE_TX_QUEUED;
    }
    if (previous_lock == 0)
    {
        (void)osKernelUnlock();
    }
    if (result == JUDGE_TX_QUEUED)
    {
        (void)osSemaphoreRelease(JudgeSemHandle);
    }
    return result;
}

judge_tx_result_t judge_send_sentry(const sentry_cmd_t *command)
{
    if (command == NULL)
    {
        return JUDGE_TX_INVALID;
    }
    const uint8_t data[4] = {
        (uint8_t)command->sentry_cmd, (uint8_t)(command->sentry_cmd >> 8),
        (uint8_t)(command->sentry_cmd >> 16), (uint8_t)(command->sentry_cmd >> 24),
    };
    return judge_send_interaction(0x0120U, 0x8080U, data, sizeof(data));
}

void judge_set_tx_bandwidth(uint16_t bytes_per_second)
{
    const int32_t previous_lock = osKernelLock();
    judge_tx_status.byte_limit_per_second = bytes_per_second;
    if (previous_lock == 0)
    {
        (void)osKernelUnlock();
    }
    if (JudgeSemHandle != NULL)
    {
        (void)osSemaphoreRelease(JudgeSemHandle);
    }
}

void judge_read_tx_status(judge_tx_status_t *status)
{
    const int32_t previous_lock = osKernelLock();
    *status = judge_tx_status;
    if (previous_lock == 0)
    {
        (void)osKernelUnlock();
    }
}

bool judge_take_tx(judge_tx_request_t *request)
{
    const int32_t previous_lock = osKernelLock();
    const bool available = tx_count > 0U;
    if (available)
    {
        *request = tx_queue[tx_head];
        tx_head = (tx_head + 1U) % JUDGE_TX_QUEUE_DEPTH;
        tx_count--;
    }
    if (previous_lock == 0)
    {
        (void)osKernelUnlock();
    }
    return available;
}

judge_tx_result_t judge_prepare_tx(const judge_tx_request_t *request, uint8_t sequence, uint32_t now_tick,
                                    uint8_t *buffer, uint16_t *length)
{
    const uint32_t age_ticks = (osKernelGetTickFreq() * JUDGE_TX_MAX_AGE_MS + 999U) / 1000U;
    if (now_tick - request->queued_tick >= age_ticks)
    {
        return JUDGE_TX_EXPIRED;
    }
    const int32_t previous_lock = osKernelLock();
    const bool current_source = judge_data_is_fresh(&judge_data, JUDGE_DATA_ROBOT_STATUS, now_tick)
        && judge_data.robot_status.robot_id == request->sender_id;
    if (previous_lock == 0)
    {
        (void)osKernelUnlock();
    }
    if (!current_source)
    {
        return JUDGE_TX_OFFLINE;
    }

    const uint16_t data_length = request->data_length + 6U;
    buffer[0] = 0xA5U;
    buffer[1] = (uint8_t)data_length;
    buffer[2] = (uint8_t)(data_length >> 8);
    buffer[3] = sequence;
    buffer[4] = crc8(buffer, JUDGE_FRAME_HEADER_SIZE - 1U);
    buffer[5] = 0x01U;
    buffer[6] = 0x03U;
    buffer[7] = (uint8_t)request->data_cmd_id;
    buffer[8] = (uint8_t)(request->data_cmd_id >> 8);
    buffer[9] = (uint8_t)request->sender_id;
    buffer[10] = (uint8_t)(request->sender_id >> 8);
    buffer[11] = (uint8_t)request->receiver_id;
    buffer[12] = (uint8_t)(request->receiver_id >> 8);
    memcpy(buffer + 13U, request->data, request->data_length);
    *length = data_length + JUDGE_FRAME_OVERHEAD;
    const uint16_t frame_crc16 = crc16(buffer, *length - sizeof(frame_tail_t));
    buffer[*length - 2U] = (uint8_t)frame_crc16;
    buffer[*length - 1U] = (uint8_t)(frame_crc16 >> 8);
    return JUDGE_TX_READY;
}
