#ifndef SENTRY_CHASSIS_USB_COMMUN_H
#define SENTRY_CHASSIS_USB_COMMUN_H

#include <stdbool.h>
#include <stdint.h>

#include "judge.h"

#define USB_SEND_MIN_INTERVAL_MS  2U   /* USB 成功提交包之间的最小间隔，单位 ms。 */
#define USB_STATE_SEND_INTERVAL_MS 10U /* 底盘状态帧发送间隔，单位 ms。 */

/* 底盘 yaw 单位 rad、yaw 角速度单位 rad/s*/
extern volatile float usb_chassis_yaw_rad;
extern volatile float usb_chassis_wz_rad_s;
extern volatile bool usb_chassis_imu_valid;
void usb_commun_update_imu(float yaw_rad, float yaw_rate_rad_s);

void usb_send_update(uint32_t now_tick);



typedef struct
{
    uint32_t frame_header; //0x1002
    uint32_t frame_timestamp;
    struct
    {
        float x;                    //x 轴速度，向前为正，单位 m/s
        float y;                    //y 轴速度，向左为正，单位 m/s
        float w;                    // 底盘 yaw 角速度，逆时针为正，单位 rad/s
        bool chassis_spin;          //0不开小陀螺，1 为快速小陀螺 SPIN_FAST
        uint8_t gimabal_model;      //0 跟随底盘，1 固定目标角，2 目标角巡逻。
        float gimabal_tar_angle;    // 云台目标角度，单位 rad。
        float gimabal_pat_angle;    // 云台巡逻范围，单位 rad。
        bool activate_energy;       // 0 关闭能量机关，1 请求开启。
    } data;
    uint8_t eof; //固定为 0xA5
} receive_cmd_t;

/* 上位机下发的哨兵自主决策指令帧，共 16 字节（eof 后 3 字节为对齐填充，不校验）。
 * USB 任务收到后调用 judge_send_sentry()，转发为裁判系统 0x0301 子内容 0x0120。 */
typedef struct
{
    uint32_t frame_header; //固定为 0x1003
    uint32_t frame_timestamp;
    sentry_cmd_t data;     //哨兵自主决策指令，位定义同裁判协议 0x0120
    uint8_t eof; //固定为 0xA5
} receive_sentry_t;


typedef struct
{
    uint32_t frame_header; //固定为 0x1001
    uint32_t frame_timestamp;
    struct
    {
        float chassis_yaw; /* 底盘当前 yaw，单位 rad。 */
        float chassis_wz;  /* 底盘当前 yaw 角速度，单位 rad/s。 */
    } data;
    uint8_t eof; /* 固定为 0xA5。 */
} send_cmd_t;



void usb_receive_reset(void);
void usb_receive_from_isr(uint8_t *buffer, uint32_t length);

void usb_receive_process(uint32_t now_tick, uint32_t frame_timeout_ticks);

bool usb_read_command(receive_cmd_t *command);

#endif
