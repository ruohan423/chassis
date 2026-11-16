//
// Created by mot on 2026/11/7.
//

#ifndef SENTRY_CHASSIS_CAN_COMMUN_H
#define SENTRY_CHASSIS_CAN_COMMUN_H

#include <stdbool.h>
#include <stdint.h>

/* CAN1 板间通信：每条消息使用独立的标准帧标识，帧内容直接为消息载荷，多字节量均为小端。 */
#define CAN_COMMUN_REMOTE_ID                  0x311U   /* 底盘板发往云台板的遥控器帧标识。 */
#define CAN_COMMUN_IMU_ID                     0x312U   /* 底盘板发往云台板的姿态帧标识。 */
#define CAN_COMMUN_GIMBAL_ACTIVE_ID           0x313U   /* 底盘板发往云台板的指令帧标识。 */
#define CAN_COMMUN_GIMBAL_FEEDBACK_ID         0x301U   /* 云台板发往底盘板的反馈帧标识。 */

#define CAN_COMMUN_REMOTE_LENGTH              6U       /* 遥控器载荷长度，单位：字节；ch0～ch3 各 11 位依次打包，随后 s2、s1 各 2 位。 */
#define CAN_COMMUN_IMU_LENGTH                 4U       /* 底盘姿态载荷长度，单位：字节；依次为 int16 yaw 和 int16 yaw 角速度。 */
#define CAN_COMMUN_GIMBAL_ACTIVE_LENGTH       5U       /* 云台指令载荷长度，单位：字节；依次为状态位、int16 目标角和 uint16 巡逻范围。 */
#define CAN_COMMUN_GIMBAL_FEEDBACK_LENGTH     5U       /* 云台反馈载荷长度，单位：字节；依次为 int16 相对 yaw 角、int16 yaw 角速度和状态位。 */

#define CAN_COMMUN_ANGLE_LSB_RAD              0.0001f  /* 角度量化单位，单位 rad/LSB。int16 角度由发送方归一化到 -π～π；uint16 巡逻范围表示 0～6.5535 rad，超出时饱和。 */
#define CAN_COMMUN_RATE_LSB_RAD_S             0.001f   /* 角速度量化单位，单位 (rad/s)/LSB。int16 表示 ±32.767 rad/s，超出时饱和。 */
#define CAN_COMMUN_GIMBAL_TIMEOUT_MS          10U      /* 云台反馈超时时间，单位 ms。 */
#define CAN_COMMUN_GIMBAL_ACTIVE_TIMEOUT_MS   200U     /* 上位机云台指令超时时间，单位 ms；超时后云台指令帧恢复为全零默认值。 */

typedef struct
{
    float yaw_angle_rad;
    float yaw_rate_rad_s;
    bool pitch_status;//小云台当前状态，0升起状态，1降低云台
    bool energy_mechanis_done; //是否完成开符动作，0未完成1完成，仅指当前开符动作有没有结束，不代表有没有开符成功
    uint32_t tick;
    bool online;
} gimbal_data_t;

typedef struct {
    bool pitch_cmd;             //0升起云台，1降低云台
    uint8_t gimabal_model;      //0 跟随底盘，1 固定目标角，2 目标角巡逻。
    float gimabal_tar_angle;    // 云台目标角度，单位 rad。
    float gimabal_pat_angle;    // 云台巡逻范围，单位 rad。
    bool activate_energy;       // 是否准备打能量机关 0 关闭，1 请求开启。

}gimbal_active_t;

bool can_commun_init(void);
/* 任务上下文调用；data 为调用方输出对象，仅在调用期间写入且不保存指针。
 * 内部以任务/ISR 临界区复制快照，调用方不再加锁；云台反馈超时返回 false。 */
bool can_commun_read_gimbal_data(gimbal_data_t *data);
void can_commun_update_imu(float yaw_rad, float yaw_rate_rad_s);
/* active 仅在调用期间只读借用并复制；内部以调度锁保护，调用方不再加锁。 */
void can_commun_update_gimbal_active(const gimbal_active_t *active);
/* 以下三个发送函数由 CanCom_Task 调用：遥控器帧随有效遥控帧触发，底盘姿态帧和云台指令帧按周期触发。 */
void can_commun_send_remote(void);
void can_commun_send_imu(void);
void can_commun_send_gimbal_active(void);

#endif //SENTRY_CHASSIS_CAN_COMMUN_H
