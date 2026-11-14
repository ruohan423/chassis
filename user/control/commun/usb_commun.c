#include "usb_commun.h"

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

_Static_assert(sizeof(float) == 4U && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__, "USB 协议要求小端 32 位浮点数");
_Static_assert(sizeof(bool) == 1U, "USB 协议要求 bool 占 1 字节");
_Static_assert(offsetof(receive_cmd_t, data) == 8U, "USB 控制帧数据偏移不匹配");
_Static_assert(offsetof(receive_cmd_t, data.chassis_spin) == 20U, "USB 小陀螺字段偏移不匹配");
_Static_assert(offsetof(receive_cmd_t, data.gimabal_model) == 21U, "USB 云台模式字段偏移不匹配");
_Static_assert(offsetof(receive_cmd_t, data.activate_energy) == 32U, "USB 能量机关字段偏移不匹配");
_Static_assert(sizeof(receive_cmd_t) == 40U, "USB 控制帧大小不匹配");
_Static_assert(offsetof(receive_sentry_t, data) == 8U, "USB 哨兵指令帧数据偏移不匹配");
_Static_assert(offsetof(receive_sentry_t, eof) == 12U, "USB 哨兵指令帧尾偏移不匹配");
_Static_assert(sizeof(receive_sentry_t) == 16U, "USB 哨兵指令帧大小不匹配");
_Static_assert(offsetof(send_cmd_t, data) == 8U, "USB 状态帧数据偏移不匹配");
_Static_assert(sizeof(send_cmd_t) == 20U, "USB 状态帧大小不匹配");
_Static_assert(sizeof(ground_robot_position_t) == 40U, "USB 地面机器人位置载荷大小不匹配");

static send_cmd_t chassis_packet = {
    .frame_header = 0x1001U,
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

_Static_assert(sizeof(judge_packets) / sizeof(judge_packets[0]) == 16U, "USB 裁判数据包数量不匹配");

#define USB_JUDGE_PACKET_MAX_DATA_SIZE sizeof(ground_robot_position_t)
#define USB_JUDGE_PACKET_COUNT (sizeof(judge_packets) / sizeof(judge_packets[0]))

static uint8_t judge_packet_buffer[sizeof(usb_frame_header_t) + sizeof(uint32_t) + USB_JUDGE_PACKET_MAX_DATA_SIZE + 1U];

typedef struct
{
    uint32_t judge_sent_ticks[USB_JUDGE_PACKET_COUNT];
    uint16_t judge_sent_mask;
    uint8_t next_judge_packet;
    uint32_t last_send_tick;
    uint32_t last_state_send_tick;
    bool has_sent_packet;
    bool has_sent_state;
} usb_send_state_t;

typedef enum
{
    USB_SEND_NOT_DUE,
    USB_SEND_SENT,
    USB_SEND_BLOCKED,
} usb_send_result_t;

static usb_send_state_t send_state;

volatile float usb_chassis_yaw_rad;
volatile float usb_chassis_wz_rad_s;
volatile bool usb_chassis_imu_valid;

void usb_commun_update_imu(float yaw_rad, float yaw_rate_rad_s)
{
    const int32_t previous_lock = osKernelLock();
    if (previous_lock < 0)
    {
        return;
    }

    usb_chassis_yaw_rad = yaw_rad;
    usb_chassis_wz_rad_s = yaw_rate_rad_s;
    usb_chassis_imu_valid = true;
    if (previous_lock == 0)
    {
        (void)osKernelRestoreLock(previous_lock);
    }
}

#define USB_CONTROL_FRAME_HEADER 0x1002U
#define USB_SENTRY_FRAME_HEADER  0x1003U
#define USB_CONTROL_FRAME_SIZE   sizeof(receive_cmd_t)
#define USB_SENTRY_FRAME_SIZE    sizeof(receive_sentry_t)

static uint8_t receive_buffer[CDC_DATA_FS_OUT_PACKET_SIZE];
static uint8_t *volatile receive_endpoint_buffer;
static volatile uint32_t receive_length;
static volatile bool receive_reset_pending;
static volatile bool receive_error_pending;
static uint8_t receive_frame[USB_CONTROL_FRAME_SIZE];
static uint8_t receive_frame_length;
static uint32_t last_receive_tick;
static receive_cmd_t received_command;
static bool command_pending;

static bool usb_transmit(uint8_t *buffer, uint16_t length);
static bool usb_send_chassis(uint32_t now_tick);
static usb_send_result_t usb_send_judge_data(uint32_t now_tick, uint8_t index);
static bool usb_is_ready(void);
static usb_send_result_t usb_send_state_if_due(uint32_t now_tick);

void usb_send_update(uint32_t now_tick)
{
    if (send_state.has_sent_packet && now_tick - send_state.last_send_tick < USB_SEND_MIN_INTERVAL_MS)
    {
        return;
    }

    if (!usb_is_ready())
    {
        return;
    }

    if (usb_send_state_if_due(now_tick) != USB_SEND_NOT_DUE)
    {
        return;
    }

    /* 裁判包不设独立周期：每次至多提交一包，轮询起点后移，避免高频包饿死低频包。 */
    for (uint8_t checked = 0U; checked < USB_JUDGE_PACKET_COUNT; checked++)
    {
        const uint8_t index = (uint8_t)((send_state.next_judge_packet + checked) % USB_JUDGE_PACKET_COUNT);
        const usb_send_result_t result = usb_send_judge_data(now_tick, index);
        if (result == USB_SEND_BLOCKED)
        {
            return;
        }
        if (result == USB_SEND_SENT)
        {
            send_state.has_sent_packet = true;
            send_state.last_send_tick = now_tick;
            send_state.next_judge_packet = (uint8_t)((index + 1U) % USB_JUDGE_PACKET_COUNT);
            return;
        }
    }
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

static usb_send_result_t usb_send_state_if_due(uint32_t now_tick)
{
    if (send_state.has_sent_state && now_tick - send_state.last_state_send_tick < USB_STATE_SEND_INTERVAL_MS)
    {
        return USB_SEND_NOT_DUE;
    }

    if (!usb_send_chassis(now_tick))
    {
        return USB_SEND_BLOCKED;
    }

    send_state.has_sent_state = true;
    send_state.last_state_send_tick = now_tick;
    send_state.has_sent_packet = true;
    send_state.last_send_tick = now_tick;
    return USB_SEND_SENT;
}

static bool usb_transmit(uint8_t *buffer, uint16_t length)
{
    bool accepted = false;
    taskENTER_CRITICAL();
    const USBD_CDC_HandleTypeDef *cdc = hUsbDeviceFS.pClassData;
    if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED && cdc != NULL && cdc->TxState == 0U)
    {
        accepted = CDC_Transmit_FS(buffer, length) == USBD_OK;
    }
    taskEXIT_CRITICAL();
    return accepted;
}

static bool usb_send_chassis(uint32_t now_tick)
{
    memset(&chassis_packet, 0, sizeof(chassis_packet));
    chassis_packet.frame_timestamp = now_tick;
    const int32_t previous_lock = osKernelLock();
    if (previous_lock < 0)
    {
        return false;
    }
    chassis_packet.data.chassis_yaw = usb_chassis_yaw_rad;
    chassis_packet.data.chassis_wz = usb_chassis_wz_rad_s;
    if (previous_lock == 0)
    {
        (void)osKernelRestoreLock(previous_lock);
    }
    chassis_packet.frame_header = 0x1001U;
    chassis_packet.eof = 0xA5U;
    return usb_transmit((uint8_t *)&chassis_packet, sizeof(chassis_packet));
}

static usb_send_result_t usb_send_judge_data(uint32_t now_tick, uint8_t index)
{
    const usb_judge_packet_t *packet = &judge_packets[index];
    const uint16_t sent_bit = (uint16_t)(1U << index);
    const int32_t previous_lock = osKernelLock();
    if (previous_lock < 0)
    {
        return USB_SEND_NOT_DUE;
    }

    /* 只在 Judge_Task 收到该命令的新帧后转发一次，发送频率跟随裁判系统下发频率；
     * 没有新帧、链路超时或数据被清空时不发送，也不补发清零包，时效由上位机按收包时间判断。 */
    const uint32_t update_tick = judge_data.update_tick[packet->data_id];
    if ((judge_data.valid_mask & (1UL << packet->data_id)) == 0U
        || ((send_state.judge_sent_mask & sent_bit) != 0U && send_state.judge_sent_ticks[index] == update_tick))
    {
        if (previous_lock == 0)
        {
            (void)osKernelRestoreLock(previous_lock);
        }
        return USB_SEND_NOT_DUE;
    }

    judge_packet_buffer[0] = 0x5AU;
    judge_packet_buffer[1] = packet->length;
    judge_packet_buffer[2] = (uint8_t)packet->command_id;
    judge_packet_buffer[3] = (uint8_t)(packet->command_id >> 8U);
    memcpy(&judge_packet_buffer[4], &now_tick, sizeof(now_tick));
    memcpy(&judge_packet_buffer[8], (const uint8_t *)&judge_data + packet->offset, packet->length);
    judge_packet_buffer[8U + packet->length] = 0xA5U;
    if (previous_lock == 0)
    {
        (void)osKernelRestoreLock(previous_lock);
    }
    if (!usb_transmit(judge_packet_buffer, sizeof(usb_frame_header_t) + sizeof(now_tick) + packet->length + 1U))
    {
        return USB_SEND_BLOCKED;
    }

    send_state.judge_sent_ticks[index] = update_tick;
    send_state.judge_sent_mask |= sent_bit;
    return USB_SEND_SENT;
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

void usb_receive_process(uint32_t now_tick, uint32_t frame_timeout_ticks)
{
    uint8_t data[CDC_DATA_FS_OUT_PACKET_SIZE];
    uint32_t length = 0U;

    /* 只在交接区内取走一包并重装端点，拼帧和校验在临界区外执行。 */
    taskENTER_CRITICAL();
    const bool connected = hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED && hUsbDeviceFS.pClassData != NULL;
    if (receive_reset_pending || !connected)
    {
        receive_reset_pending = false;
        receive_frame_length = 0U;
        command_pending = false;
        if (!connected)
        {
            receive_endpoint_buffer = NULL;
            receive_length = 0U;
            receive_error_pending = false;
            taskEXIT_CRITICAL();
            return;
        }
    }
    if (receive_error_pending)
    {
        receive_error_pending = false;
        receive_frame_length = 0U;
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
    }
    taskEXIT_CRITICAL();

    if (receive_frame_length != 0U && now_tick - last_receive_tick >= frame_timeout_ticks)
    {
        receive_frame_length = 0U;
    }
    for (uint32_t index = 0U; index < length; index++)
    {
        receive_frame[receive_frame_length++] = data[index];
        /* 帧头决定帧长；滑动后剩余字节里可能已有完整短帧，需循环处理到数据不足为止。 */
        while (receive_frame_length >= sizeof(uint32_t))
        {
            uint32_t frame_header;
            memcpy(&frame_header, receive_frame, sizeof(frame_header));
            const uint8_t frame_size = frame_header == USB_CONTROL_FRAME_HEADER ? USB_CONTROL_FRAME_SIZE
                : frame_header == USB_SENTRY_FRAME_HEADER ? USB_SENTRY_FRAME_SIZE : 0U;
            uint8_t consumed = 1U;

            if (frame_size != 0U)
            {
                if (receive_frame_length < frame_size)
                {
                    break;
                }
                const bool valid = frame_header == USB_CONTROL_FRAME_HEADER
                    ? (receive_frame[offsetof(receive_cmd_t, eof)] == 0xA5U
                       && receive_frame[offsetof(receive_cmd_t, data.chassis_spin)] <= 1U
                       && receive_frame[offsetof(receive_cmd_t, data.gimabal_model)] <= 2U
                       && receive_frame[offsetof(receive_cmd_t, data.activate_energy)] <= 1U)
                    : receive_frame[offsetof(receive_sentry_t, eof)] == 0xA5U;
                if (valid)
                {
                    taskENTER_CRITICAL();
                    const bool still_connected = !receive_reset_pending && hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED
                        && hUsbDeviceFS.pClassData != NULL;
                    if (still_connected && frame_header == USB_CONTROL_FRAME_HEADER)
                    {
                        memcpy(&received_command, receive_frame, sizeof(received_command));
                        command_pending = true;
                    }
                    taskEXIT_CRITICAL();
                    if (!still_connected)
                    {
                        receive_frame_length = 0U;
                        return;
                    }
                    if (frame_header == USB_SENTRY_FRAME_HEADER)
                    {
                        /* 登记为 0x0301/0x0120 请求，由 Judge_Task 按裁判串口限频发出；只保留最新一条。 */
                        judge_send_sentry((const sentry_cmd_t *)&receive_frame[offsetof(receive_sentry_t, data)]);
                    }
                    consumed = frame_size;
                }
            }
            /* 未知帧头或校验失败只前移一个字节，保留其中可能包含的下一帧。 */
            receive_frame_length -= consumed;
            memmove(receive_frame, &receive_frame[consumed], receive_frame_length);
        }
    }
    if (length != 0U)
    {
        last_receive_tick = now_tick;
    }
}

bool usb_read_command(receive_cmd_t *command)
{
    bool available = false;

    taskENTER_CRITICAL();
    if (command_pending && !receive_reset_pending && hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED
        && hUsbDeviceFS.pClassData != NULL)
    {
        *command = received_command;
        command_pending = false;
        available = true;
    }
    taskEXIT_CRITICAL();
    return available;
}
