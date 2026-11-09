//
// Created by mot on 2026/11/7.
//

#ifndef SENTRY_CHASSIS_CAN_COMMON_H
#define SENTRY_CHASSIS_CAN_COMMON_H

#include <stdbool.h>
#include <stdint.h>

#define CAN_COMMON_RC_FRAME_ID_BASE       0x300U  /* ch0～ch3，小端 int16。 */
#define CAN_COMMON_IMU_FRAME_ID_BASE      0x310U  /* IMU 最终解算数据，按 float 小端发送。 */

#define CAN_COMMON_YAW_FEEDBACK_ID        0x205U  /* yaw 电机反馈，使用标准 8 字节电机反馈帧。 */
#define CAN_COMMON_YAW_TIMEOUT_MS         100U    /* yaw 电机反馈超时时间，单位 ms。 */

typedef struct
{
    uint16_t angle;
    int16_t speed;
    int16_t current;
    uint8_t temperature;
    uint32_t tick;
    bool online;
} can_common_yaw_data_t;

bool can_common_init(void);
bool can_common_read_yaw_data(can_common_yaw_data_t *data);
/* 遥控有效帧触发 0x300～0x302；周期任务触发 0x310～0x312。 */
void can_common_send_remote(void);
void can_common_send_imu(void);
void can_common_process(void);

#endif //SENTRY_CHASSIS_CAN_COMMON_H
