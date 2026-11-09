#ifndef SENTRY_CHASSIS_USB_COMMON_H
#define SENTRY_CHASSIS_USB_COMMON_H

#include <stdint.h>

#define USB_SEND_MIN_INTERVAL_MS  2U   /* USB 成功提交包之间的最小间隔，单位 ms。 */
#define USB_IMU_SEND_INTERVAL_MS 10U   /* IMU 周期发送间隔，单位 ms。 */
#define USB_JUDGE_SEND_INTERVAL_MS 50U /* 裁判包之间的最小间隔，单位 ms；限制为最高 20 Hz。 */

/* INS 单一写入，USB 发送读取；读写仅在复制时短暂锁调度，单位 deg。 */
extern volatile float usb_chassis_yaw_deg;

/* USB 收发任务正常调度调用，不在外层加锁；内部只保护快照复制及 USB 状态检查与提交。 */
void usb_send_update(uint32_t now_tick);



/* 沿用参考姿态包：0x5A、LEN=5、ID=0x12、4 字节小端时间戳、以下载荷、0xA5，共 13 字节。 */
typedef enum
{
    USB_POSTURE_ATTACK = 1,
    USB_POSTURE_DEFEND = 2,
    USB_POSTURE_MOVE = 3,
} usb_posture_t;

typedef struct
{
    uint8_t posture;
    uint8_t follow_gimbal_big;
    uint8_t track_status;
    uint8_t perception_status;
    uint8_t start_gimbal_big_spin;
} usb_posture_data_t;

/* 已解码命令，time_stamp 来自主机，仅作附带信息；超时依据本机 received_tick。 */
typedef struct
{
    usb_posture_data_t data;
    uint32_t time_stamp;
    uint32_t received_tick;
} usb_posture_command_t;

typedef enum
{
    USB_POSTURE_EMPTY = 0,
    USB_POSTURE_READY = 1,
    USB_POSTURE_OVERFLOW = 2,
    USB_POSTURE_EXPIRED = 4,
    USB_POSTURE_RESET = 8,
    USB_POSTURE_RX_ERROR = 16,
} usb_posture_result_t;


/* 仅供 CDC 初始化、反初始化和接收回调使用，不解析协议、不调用任务侧锁。
 * 回调复制到单包 RX 缓冲区后不重装端点，收发任务取走数据后重装；待处理包受端点 NAK 保护。 */
void usb_receive_reset(void);
void usb_receive_from_isr(uint8_t *buffer, uint32_t length);

/* 收发任务正常调度调用；只在交接单包和更新命令队列时短暂保护，拼帧校验在临界区外。
 * 两个超时参数的单位为当前 1kHz 内核 tick。 */
void usb_receive_process(uint32_t now_tick, uint32_t frame_timeout_ticks, uint32_t command_max_age_ticks);
/* 消费任务直接调用，内部只保护出队和事件读取；ISR 禁用。FIFO 深度 4，满时丢新。
 * 仅 READY 时写入 command；错误位可组合返回，先报告一次事件，再读取仍有效的命令。
 * 超时只丢弃尚未消费的旧命令，不自动改变已经执行的姿态，也不要求命令周期重发。 */
usb_posture_result_t usb_read_posture(usb_posture_command_t *command);

#endif
