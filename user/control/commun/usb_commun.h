#ifndef SENTRY_CHASSIS_USB_COMMUN_H
#define SENTRY_CHASSIS_USB_COMMUN_H

#include <stdbool.h>
#include <stdint.h>

#include "judge.h"

#define USB_SEND_MIN_INTERVAL_MS  2U   /* USB 成功提交包之间的最小间隔，单位 ms。 */
#define USB_STATE_SEND_INTERVAL_MS 10U /* 底盘状态帧发送间隔，单位 ms。 */
#define USB_CHASSIS_BEHAVIOR_MAX 9U  /* 控制帧 chassis_behavior 的最大合法编号，须等于 chassis_behavior_t 的最后一项；超出的帧按无效帧丢弃。 */

/* 底盘 yaw 单位 rad、yaw 角速度单位 rad/s*/
extern volatile float usb_chassis_yaw_rad;
extern volatile float usb_chassis_wz_rad_s;
extern volatile bool usb_chassis_imu_valid;
void usb_commun_update_imu(float yaw_rad, float yaw_rate_rad_s);

void usb_send_update(uint32_t now_tick);

/* 上位机下发的控制帧，共 44 字节，小端：data 起始偏移 8，target_yaw 偏移 20，chassis_behavior 偏移 24，pitch_cmd 偏移 25，
 * gimabal_model 偏移 26，gimabal_tar_angle 偏移 28，gimabal_pat_angle 偏移 32，activate_energy 偏移 36，eof 偏移 40；
 * 偏移 27、37～39、41～43 为对齐填充，不校验。底盘字段由 Chassis_Task 读取，云台字段经 CAN 转发给云台板。
 * 上电世界系：底盘姿态解算启动时刻的底盘坐标系，此后固定不动；状态帧的 chassis_yaw 即底盘在该系中的朝向。 */
typedef struct
{
    uint32_t frame_header; //0x1002
    uint32_t frame_timestamp;
    struct
    {
        float x;                    //x 轴速度，向前为正，单位 m/s；直控时在底盘坐标系，跟随云台时在云台坐标系，其余行为在上电世界系
        float y;                    //y 轴速度，向左为正，单位 m/s；坐标系同 x
        float w;                    // 底盘 yaw 角速度，逆时针为正，单位 rad/s；仅直控和世界系直控采用
        float target_yaw;           // 底盘目标朝向，上电世界系，与状态帧 chassis_yaw 同一基准，单位 rad；仅两面对齐、单面对齐采用
        uint8_t chassis_behavior;   // 底盘行为编号，同 chassis_behavior_t：0 停止，1 直控，2 世界系直控，3 慢速小陀螺，4 快速小陀螺，5 四面对齐，6 两面对齐，7 单面对齐，8 跟随云台，9 爬坡（爬坡输入无效时按直控）
        bool pitch_cmd;             //0升起云台，1降低云台
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

/* 上报给上位机的底盘状态帧，共 24 字节，小端：chassis_yaw 偏移 8，chassis_wz 偏移 12，pitch_status 偏移 16，
 * energy_mechanis_done 偏移 17，eof 偏移 20；偏移 18～19、21～23 为对齐填充，固定为 0。
 * 两个云台状态位取自云台板 CAN 反馈，云台板反馈超时时为 0。 */
typedef struct
{
    uint32_t frame_header; //固定为 0x1001
    uint32_t frame_timestamp;
    struct
    {
        float chassis_yaw; /* 底盘 IMU 的实时 yaw，即底盘在上电世界系中的朝向，逆时针为正，范围 -π～π，单位 rad。 */
        float chassis_wz;  /* 底盘当前 yaw 角速度，单位 rad/s。 */
        bool pitch_status;//小云台当前状态，0升起状态，1降低云台
        bool energy_mechanis_done; //是否完成开符动作，0未完成1完成，仅指当前开符动作有没有结束，不代表有没有开符成功
    } data;
    uint8_t eof; /* 固定为 0xA5。 */
} send_cmd_t;

void usb_receive_reset(void);
/* buffer 由 USB CDC 驱动持有且必须可写；本函数保存端点缓冲区指针，直到任务完成复制并重新装载端点。 */
void usb_receive_from_isr(uint8_t *buffer, uint32_t length);

void usb_receive_process(uint32_t now_tick, uint32_t frame_timeout_ticks);

/* command 为调用方输出对象，仅在调用期间写入；成功读取后模块清除待处理标志，不保存该指针。 */
bool usb_read_command(receive_cmd_t *command);

#endif
