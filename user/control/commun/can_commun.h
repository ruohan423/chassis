//
// Created by mot on 2026/11/7.
//

#ifndef SENTRY_CHASSIS_CAN_COMMUN_H
#define SENTRY_CHASSIS_CAN_COMMUN_H

#include <stdbool.h>
#include <stdint.h>

#define CAN_COMMUN_RC_FRAME_ID_BASE       0x110U  /* 遥控器单帧标识：ch0～ch3 按 11 位打包，s1、s2 各占 2 位。 */
#define CAN_COMMUN_RC_DATA_LENGTH         6U      /* 遥控器单帧载荷长度，单位：字节。 */
#define CAN_COMMUN_IMU_FRAME_ID_BASE      0x111U  /* 底盘 yaw（rad）与 yaw 角速度（rad/s），各按 float 小端发送。 */

#define CAN_COMMUN_GIMBAL_FEEDBACK_ID     0x100U  /* 云台相对 yaw 角与 yaw 角速度，两个小端 float。 */
#define CAN_COMMUN_GIMBAL_TIMEOUT_MS      10U    /* 云台反馈超时时间，单位 ms。 */

typedef struct
{
    float yaw_angle_rad;
    float yaw_rate_rad_s;
    uint32_t tick;
    bool online;
} gimbal_data_t;

bool can_commun_init(void);
bool can_commun_read_gimbal_data(gimbal_data_t *data);
void can_commun_update_imu(float yaw_rad, float yaw_rate_rad_s);
/* 遥控有效帧触发 0x100 单帧；周期任务触发 0x110 单帧。 */
void can_commun_send_remote(void);
void can_commun_send_imu(void);

#endif //SENTRY_CHASSIS_CAN_COMMUN_H
