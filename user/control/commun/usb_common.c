#include "usb_common.h"

#include <stddef.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"

#include "judge.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

extern USBD_HandleTypeDef hUsbDeviceFS;


typedef struct __attribute__((packed))
{
    uint8_t sof;
    uint8_t length;
    uint16_t id;
} usb_frame_header_t;

typedef struct __attribute__((packed))
{
    usb_frame_header_t frame_header;
    uint32_t time_stamp;
    struct __attribute__((packed))
    {
        float gimbal_yaw;
        float yaw_chassis;
        float yaw;
    } data;
    uint8_t eof;
} usb_imu_packet_t;

_Static_assert(sizeof(float) == 4U && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__, "USB 协议要求小端 32 位浮点数");
_Static_assert(sizeof(usb_frame_header_t) == 4U, "USB 帧头线格式大小不匹配");
_Static_assert(sizeof(usb_imu_packet_t) == 21U, "USB imu 线格式大小不匹配");
_Static_assert(sizeof(ground_robot_position_t) == 40U, "USB 地面机器人位置载荷大小不匹配");

static usb_imu_packet_t imu_packet = {
    .frame_header = {0x5AU, sizeof(imu_packet.data), 0x0010U},
    .eof = 0xA5U,
};

typedef struct
{
    uint16_t command_id;
    judge_data_id_t data_id;
    uint16_t offset;
    uint8_t length;
} usb_judge_packet_t;

static const usb_judge_packet_t judge_packets[] = {
    {0x0001U, JUDGE_DATA_GAME_STATUS, offsetof(judge_data_package_t, game_status), sizeof(game_status_t)},
    {0x0002U, JUDGE_DATA_GAME_RESULT, offsetof(judge_data_package_t, game_result), sizeof(game_result_t)},
    {0x0003U, JUDGE_DATA_ROBOT_HP, offsetof(judge_data_package_t, robot_hp), sizeof(game_robot_hp_t)},
    {0x0101U, JUDGE_DATA_EVENT, offsetof(judge_data_package_t, event), sizeof(event_data_t)},
    {0x0104U, JUDGE_DATA_WARNING, offsetof(judge_data_package_t, warning), sizeof(referee_warning_t)},
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

_Static_assert(sizeof(judge_packets) / sizeof(judge_packets[0]) == 15U, "USB 裁判数据包数量不匹配");

#define USB_JUDGE_PACKET_MAX_DATA_SIZE sizeof(ground_robot_position_t)
#define USB_JUDGE_PACKET_COUNT (sizeof(judge_packets) / sizeof(judge_packets[0]))

static uint8_t judge_packet_buffer[sizeof(usb_frame_header_t) + sizeof(uint32_t) + USB_JUDGE_PACKET_MAX_DATA_SIZE + 1U];

typedef struct
{
    uint32_t judge_sent_ticks[USB_JUDGE_PACKET_COUNT];
    uint16_t judge_sent_mask;
    uint16_t judge_sent_valid_mask;
    uint8_t next_judge_packet;
    uint32_t last_send_tick;
    uint32_t last_imu_send_tick;
    uint32_t last_judge_send_tick;
    bool has_sent_packet;
    bool has_sent_imu;
    bool has_sent_judge_packet;
} usb_send_state_t;

typedef enum
{
    USB_SEND_NOT_DUE,
    USB_SEND_SENT,
    USB_SEND_BLOCKED,
} usb_send_result_t;

static usb_send_state_t send_state;

volatile float usb_chassis_yaw_deg;

#define USB_POSTURE_QUEUE_DEPTH 4U  /* 完整姿态命令的保序队列深度，满时丢新并报告。 */
#define USB_POSTURE_FRAME_SIZE (8U + sizeof(usb_posture_data_t))  /* 固定开销加 5 字节载荷，共 13 字节。 */

_Static_assert(sizeof(usb_posture_data_t) == 5U, "USB 姿态载荷大小不匹配");

static uint8_t receive_buffer[CDC_DATA_FS_OUT_PACKET_SIZE];
static uint8_t *volatile receive_endpoint_buffer;
static volatile uint32_t receive_length;
static volatile bool receive_reset_pending;
static volatile bool receive_error_pending;
static uint8_t receive_frame[USB_POSTURE_FRAME_SIZE];
static uint8_t receive_frame_length;
static uint32_t last_receive_tick;
static usb_posture_command_t posture_queue[USB_POSTURE_QUEUE_DEPTH];
static uint8_t posture_head;
static uint8_t posture_count;
static uint8_t posture_events;

static bool usb_transmit(uint8_t *buffer, uint16_t length);
static bool usb_send_imu(uint32_t now_tick);
static bool usb_send_judge_data(uint32_t now_tick, uint8_t index, bool *pending);
static bool usb_is_ready(void);
static usb_send_result_t usb_send_imu_if_due(uint32_t now_tick);
static usb_send_result_t usb_send_judge_if_due(uint32_t now_tick);

void usb_send_update(uint32_t now_tick)
{
    usb_send_result_t result;

    if (send_state.has_sent_packet && now_tick - send_state.last_send_tick < USB_SEND_MIN_INTERVAL_MS)
    {
        return;
    }

    if (!usb_is_ready())
    {
        return;
    }

    result = usb_send_imu_if_due(now_tick);
    if (result != USB_SEND_NOT_DUE)
    {
        return;
    }

    (void)usb_send_judge_if_due(now_tick);
}

static bool usb_is_ready(void)
{
    bool ready;

    taskENTER_CRITICAL();
    const USBD_CDC_HandleTypeDef *cdc = hUsbDeviceFS.pClassData;
    ready = hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED && cdc != NULL && cdc->TxState == 0U;
    taskEXIT_CRITICAL();
    return ready;
}

static usb_send_result_t usb_send_imu_if_due(uint32_t now_tick)
{
    if (send_state.has_sent_imu && now_tick - send_state.last_imu_send_tick < USB_IMU_SEND_INTERVAL_MS)
    {
        return USB_SEND_NOT_DUE;
    }

    if (!usb_send_imu(now_tick))
    {
        return USB_SEND_BLOCKED;
    }

    send_state.has_sent_imu = true;
    send_state.last_imu_send_tick = now_tick;
    send_state.has_sent_packet = true;
    send_state.last_send_tick = now_tick;
    return USB_SEND_SENT;
}

static usb_send_result_t usb_send_judge_if_due(uint32_t now_tick)
{
    if (send_state.has_sent_judge_packet
        && now_tick - send_state.last_judge_send_tick < USB_JUDGE_SEND_INTERVAL_MS)
    {
        return USB_SEND_NOT_DUE;
    }

    for (uint8_t checked = 0U; checked < USB_JUDGE_PACKET_COUNT; checked++)
    {
        const uint8_t index = (uint8_t)((send_state.next_judge_packet + checked) % USB_JUDGE_PACKET_COUNT);
        bool pending;
        if (!usb_send_judge_data(now_tick, index, &pending))
        {
            if (pending)
            {
                return USB_SEND_BLOCKED;
            }
            continue;
        }

        send_state.last_judge_send_tick = now_tick;
        send_state.has_sent_judge_packet = true;
        send_state.has_sent_packet = true;
        send_state.last_send_tick = now_tick;
        send_state.next_judge_packet = (uint8_t)((index + 1U) % USB_JUDGE_PACKET_COUNT);
        return USB_SEND_SENT;
    }

    return USB_SEND_NOT_DUE;
}

static bool usb_transmit(uint8_t *buffer, uint16_t length)
{
    bool accepted = false;

    /* 填包期间允许 USB 中断，提交前重新确认类对象与端点状态。 */
    taskENTER_CRITICAL();
    const USBD_CDC_HandleTypeDef *cdc = hUsbDeviceFS.pClassData;
    if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED && cdc != NULL && cdc->TxState == 0U)
    {
        accepted = CDC_Transmit_FS(buffer, length) == USBD_OK;
    }
    taskEXIT_CRITICAL();
    return accepted;
}

static bool usb_send_imu(uint32_t now_tick)
{
    imu_packet.time_stamp = now_tick;
    const int32_t previous_lock = osKernelLock();
    if (previous_lock < 0)
    {
        return false;
    }
    imu_packet.data.yaw_chassis = usb_chassis_yaw_deg * (3.14159265f / 180.0f);
    /* 云台相对角与绝对角尚未接入，保持零值；此时间戳不是传感器原始采样时间。 */
    if (previous_lock == 0)
    {
        (void)osKernelRestoreLock(previous_lock);
    }
    return usb_transmit((uint8_t *)&imu_packet, sizeof(imu_packet));
}

static bool usb_send_judge_data(uint32_t now_tick, uint8_t index, bool *pending)
{
    const usb_judge_packet_t *packet = &judge_packets[index];
    const uint16_t sent_bit = (uint16_t)(1U << index);
    const int32_t previous_lock = osKernelLock();
    if (previous_lock < 0)
    {
        *pending = false;
        return false;
    }

    const bool fresh = judge_data_is_fresh(&judge_data, packet->data_id, now_tick);
    const uint32_t update_tick = judge_data.update_tick[packet->data_id];
    *pending = fresh
        ? ((send_state.judge_sent_mask & sent_bit) == 0U || send_state.judge_sent_ticks[index] != update_tick)
        : (send_state.judge_sent_valid_mask & sent_bit) != 0U;
    if (!*pending)
    {
        if (previous_lock == 0)
        {
            (void)osKernelRestoreLock(previous_lock);
        }
        return false;
    }

    judge_packet_buffer[0] = 0x5AU;
    judge_packet_buffer[1] = packet->length;
    judge_packet_buffer[2] = (uint8_t)packet->command_id;
    judge_packet_buffer[3] = (uint8_t)(packet->command_id >> 8U);
    memcpy(&judge_packet_buffer[4], &now_tick, sizeof(now_tick));
    if (fresh)
    {
        memcpy(&judge_packet_buffer[8], (const uint8_t *)&judge_data + packet->offset, packet->length);
    }
    else
    {
        memset(&judge_packet_buffer[8], 0, packet->length);
    }
    judge_packet_buffer[8U + packet->length] = 0xA5U;
    if (previous_lock == 0)
    {
        (void)osKernelRestoreLock(previous_lock);
    }
    if (!usb_transmit(judge_packet_buffer, sizeof(usb_frame_header_t) + sizeof(now_tick) + packet->length + 1U))
    {
        return false;
    }

    if (fresh)
    {
        send_state.judge_sent_ticks[index] = update_tick;
        send_state.judge_sent_mask |= sent_bit;
        send_state.judge_sent_valid_mask |= sent_bit;
    }
    else
    {
        send_state.judge_sent_mask &= (uint16_t)~sent_bit;
        send_state.judge_sent_valid_mask &= (uint16_t)~sent_bit;
    }
    return true;
}

void usb_receive_reset(void)
{
    receive_endpoint_buffer = NULL;
    receive_length = 0U;
    receive_error_pending = false;
    receive_reset_pending = true;
}

void usb_receive_from_isr(uint8_t *buffer, uint32_t length)
{
    /* OUT 端点尚未重装，只有一个待处理包；先复制，避免任务依赖驱动缓冲区的内容。 */
    if (length > sizeof(receive_buffer))
    {
        receive_error_pending = true;
        receive_length = 0U;
    }
    else
    {
        memcpy(receive_buffer, buffer, length);
        receive_length = length;
    }
    receive_endpoint_buffer = buffer;
}

void usb_receive_process(uint32_t now_tick, uint32_t frame_timeout_ticks, uint32_t command_max_age_ticks)
{
    uint8_t data[CDC_DATA_FS_OUT_PACKET_SIZE];
    uint32_t length = 0U;

    /* 只在交接区内取走一包、更新队列状态和重装端点，不在这里拼帧或校验。 */
    taskENTER_CRITICAL();
    const bool connected = hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED && hUsbDeviceFS.pClassData != NULL;
    if (receive_reset_pending || !connected)
    {
        if (receive_reset_pending || posture_count != 0U || receive_frame_length != 0U || receive_endpoint_buffer != NULL)
        {
            posture_events |= USB_POSTURE_RESET;
        }
        receive_reset_pending = false;
        receive_frame_length = 0U;
        posture_head = 0U;
        posture_count = 0U;
        if (!connected)
        {
            receive_endpoint_buffer = NULL;
            receive_length = 0U;
            receive_error_pending = false;
            taskEXIT_CRITICAL();
            return;
        }
    }
    while (posture_count != 0U && now_tick - posture_queue[posture_head].received_tick >= command_max_age_ticks)
    {
        posture_head = (posture_head + 1U) % USB_POSTURE_QUEUE_DEPTH;
        posture_count--;
        posture_events |= USB_POSTURE_EXPIRED;
    }
    if (receive_error_pending)
    {
        receive_error_pending = false;
        receive_frame_length = 0U;
        posture_events |= USB_POSTURE_RX_ERROR;
    }
    if (receive_endpoint_buffer != NULL)
    {
        length = receive_length;
        memcpy(data, receive_buffer, length);
        receive_length = 0U;
        /* 清长度后重试重装，不重复入队；退出交接区后 ISR 可以接收下一包。 */
        if (USBD_CDC_SetRxBuffer(&hUsbDeviceFS, receive_endpoint_buffer) == USBD_OK
            && USBD_CDC_ReceivePacket(&hUsbDeviceFS) == USBD_OK)
        {
            receive_endpoint_buffer = NULL;
        }
        else
        {
            posture_events |= USB_POSTURE_RX_ERROR;
        }
    }
    taskEXIT_CRITICAL();

    if (receive_frame_length != 0U && now_tick - last_receive_tick >= frame_timeout_ticks)
    {
        receive_frame_length = 0U;
    }
    for (uint32_t index = 0U; index < length; index++)
    {
        receive_frame[receive_frame_length++] = data[index];
        if (receive_frame_length < USB_POSTURE_FRAME_SIZE)
        {
            continue;
        }
        if (receive_frame[0] == 0x5AU && receive_frame[1] == sizeof(usb_posture_data_t)
            && receive_frame[2] == 0x12U && receive_frame[USB_POSTURE_FRAME_SIZE - 1U] == 0xA5U
            && receive_frame[7] >= USB_POSTURE_ATTACK && receive_frame[7] <= USB_POSTURE_MOVE
            && receive_frame[8] <= 1U && receive_frame[9] <= 1U && receive_frame[10] <= 1U && receive_frame[11] <= 1U)
        {
            taskENTER_CRITICAL();
            /* 解析期间若重新枚举，当前局部数据属于旧连接，不能继续发布。 */
            if (receive_reset_pending || hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED || hUsbDeviceFS.pClassData == NULL)
            {
                taskEXIT_CRITICAL();
                receive_frame_length = 0U;
                break;
            }
            if (posture_count == USB_POSTURE_QUEUE_DEPTH)
            {
                posture_events |= USB_POSTURE_OVERFLOW;
            }
            else
            {
                usb_posture_command_t *command = &posture_queue[(posture_head + posture_count) % USB_POSTURE_QUEUE_DEPTH];
                memcpy(&command->data, &receive_frame[7], sizeof(command->data));
                memcpy(&command->time_stamp, &receive_frame[3], sizeof(command->time_stamp));
                command->received_tick = now_tick;
                posture_count++;
            }
            taskEXIT_CRITICAL();
            receive_frame_length = 0U;
        }
        else
        {
            /* 坏帧前移一个字节，保留其中可能存在的下一帧帧头。 */
            memmove(receive_frame, &receive_frame[1], USB_POSTURE_FRAME_SIZE - 1U);
            receive_frame_length = USB_POSTURE_FRAME_SIZE - 1U;
        }
    }
    if (length != 0U)
    {
        last_receive_tick = now_tick;
    }
}

usb_posture_result_t usb_read_posture(usb_posture_command_t *command)
{
    usb_posture_result_t result = USB_POSTURE_EMPTY;

    taskENTER_CRITICAL();
    if (!receive_reset_pending && hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED && hUsbDeviceFS.pClassData != NULL)
    {
        if (posture_events != 0U)
        {
            result = (usb_posture_result_t)posture_events;
            posture_events = 0U;
        }
        else if (posture_count != 0U)
        {
            *command = posture_queue[posture_head];
            posture_head = (posture_head + 1U) % USB_POSTURE_QUEUE_DEPTH;
            posture_count--;
            result = USB_POSTURE_READY;
        }
    }
    taskEXIT_CRITICAL();
    return result;
}
