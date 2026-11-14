#ifndef SENTRY_CHASSIS_CHASSIS_PORT_H
#define SENTRY_CHASSIS_CHASSIS_PORT_H

#include <stdbool.h>
#include <stdint.h>

#include "chassis_config.h"

/* 给控制器提供归一化电机反馈及输出通道。 按逻辑轮序（左前、左后、右后、右前）和安装方向归一化 */
typedef struct
{
    volatile float speed_rad_s;      /* 轮侧角速度 rad/s */
    float output;                    /* 方向归一化后的电流原始指令 */
    volatile uint32_t feedback_tick; /* HAL 毫秒时钟，仅 port 层解释 */
    volatile bool has_feedback;
} chassis_motor_t;

extern chassis_motor_t chassis_motors[CHASSIS_MOTOR_COUNT];

bool chassis_port_init(void);
/* 每个控制周期调用一次：下发四轮指令并更新电容充电功率上限（单位 W）。离线电机的指令在此清零；发送失败返回 false。 */
bool chassis_port_send(float supercap_charge_power_limit_w);
bool chassis_motor_is_online(const chassis_motor_t *motor);

/* 裁判、电容与姿态反馈快照，已做在线判断和单位换算。
 * 云台相对角约定：云台正前方与底盘正前方（x 轴）重合时为 0，俯视云台相对底盘逆时针转动为正；
 * 零位和方向由云台侧标定后下发，port 层只归一化到下列范围。 */
typedef struct
{
    float power_limit_w;          /* 裁判底盘功率上限 W，referee_online 为真时有效 */
    float buffer_energy_j;        /* 裁判缓冲能量 J，referee_online 为真时有效 */
    float chassis_power_w;        /* 电容实测底盘功率 W，supercap_online 为真时有效 */
    float supercap_voltage_v;     /* 电容组电压 V，supercap_online 为真时有效 */
    float yaw_rate_rad_s;         /* 底盘 IMU 的 yaw 角速度 rad/s，yaw_rate_valid 为真时有效 */
    float gimbal_yaw_angle;       /* 云台相对底盘的 yaw 角 rad，范围 0～2π；云台在线时有效 */
    float gimbal_yaw_rate_rad_s;  /* 云台 yaw 角速度 rad/s；云台在线时有效 */
    float gimbal_yaw_angle_error; /* 云台 yaw 控制误差（目标减实测）rad，范围 -π～π，正方向同相对角；来源未接入，恒为 0 */
    bool referee_online;          /* 机器人性能与功率热量数据均未超时 */
    bool supercap_online;         /* 电容在线，实测功率和电压可信 */
    bool supercap_enabled;        /* 电容允许放电，离线时为假 */
    bool yaw_rate_valid;          /* 姿态解算已开始发布角速度 */
    bool gimbal_online;           /* 云台角度与角速度反馈未超时且数值有效 */
} chassis_feedback_t;

/* 仅任务上下文调用；内部以调度锁保护裁判、电容和姿态数据的快照复制，调用方不再加锁。 */
void chassis_port_read_feedback(uint32_t now_tick, chassis_feedback_t *feedback);

#endif
