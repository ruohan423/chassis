#ifndef SENTRY_CHASSIS_SUPERCAPACITOR_H
#define SENTRY_CHASSIS_SUPERCAPACITOR_H

#include <stdbool.h>
#include <stdint.h>

#define SUPER_CAPACITOR_STATUS_LENGTH  8U  /* 状态帧数据长度，单位字节 */
#define SUPER_CAPACITOR_COMMAND_LENGTH 8U  /* 指令帧数据长度，单位字节 */
#define SUPER_CAPACITOR_RX_ID          0x51U  /* 电容板发往底盘板的状态帧标识 */
#define SUPER_CAPACITOR_TX_ID          0x61U  /* 底盘板发往电容板的指令帧标识 */

/* 超级电容状态与指令，帧格式沿用 exter_add/Sentry_Chassis 的电容板协议。
 * 状态帧（0x51）：字节 0 为错误码，1～4 为 float 底盘功率，5～6 为 uint16 上报功率上限（不解析），7 为 uint8 电量。
 * 指令帧（0x61）：字节 0 为 enableDCDC 与 systemRestart 位，1～2 为 uint16 裁判功率上限，3～4 为 uint16 裁判缓冲能量，5～7 保留。
 * 电容板自行充电，主控只转发裁判上限和缓冲能量；未收到状态帧时 online 为假，功率控制按电容离线处理。 */
typedef struct
{
    bool online;           /* 状态帧未超时，由 super_capacitor_get_data 按最近更新时间判定 */
    bool error;            /* 错误码非零 */
    float chassis_power_w; /* 电容测得的底盘功率，单位 W */
    uint8_t cap_energy;    /* 电容电量，原始 0～255，量程待按电容板手册核对 */
} super_capacitor_data_t;

/* 解析一帧状态数据并刷新在线时间；data 在调用期间保持有效，不会被保存。仅由 CAN 接收中断调用，
 * 内部以任务/ISR 临界区保护共享状态，chassisPower 非有限值的帧丢弃且不刷新在线时间。 */
void super_capacitor_update_status(const uint8_t *data);
/* 任务上下文调用；复制模块内部状态到 data 并判定是否在线，内部以临界区保护，调用方不再加锁。 */
void super_capacitor_get_data(super_capacitor_data_t *data);

/* 生成常规指令帧：按 enable_dcdc 设置 enableDCDC、systemRestart 清零，转发裁判功率上限和缓冲能量。
 * data 由调用方提供，至少有 SUPER_CAPACITOR_COMMAND_LENGTH 字节，仅在调用期间写入。 */
void super_capacitor_build_command(uint8_t *data, uint16_t power_limit_w, uint16_t buffer_energy_j, bool enable_dcdc);
/* 生成上电使能帧：enableDCDC 与 systemRestart 均置位，其余字段为零。 */
void super_capacitor_build_enable_command(uint8_t *data);

#endif //SENTRY_CHASSIS_SUPERCAPACITOR_H
