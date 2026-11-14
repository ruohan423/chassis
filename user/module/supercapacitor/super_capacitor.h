#ifndef SENTRY_CHASSIS_SUPERCAPACITOR_H
#define SENTRY_CHASSIS_SUPERCAPACITOR_H

#include <stdbool.h>
#include <stdint.h>

#define SUPER_CAPACITOR_STATUS_LENGTH  8U  /* 状态帧数据长度，单位字节 */
#define SUPER_CAPACITOR_COMMAND_LENGTH 2U  /* 指令载荷长度，单位字节；RMCS 中占用 0x1FE 帧的最后两个字节 */

/* 超级电容状态与指令，帧格式移植自 RMCS-main 的 Supercap 器件（状态帧 0x300）。
 * 电容通讯尚未接线：总线、帧标识和发送周期按实车确定后，由接收方调用 super_capacitor_update_status，
 * 由发送方调用 super_capacitor_build_command。未收到状态帧时 online 为假，功率控制按电容离线处理。 */
typedef struct
{
    bool online;              /* 状态帧未超时，由 super_capacitor_get_data 按最近更新时间判定 */
    bool enabled;             /* 电容允许放电 */
    float chassis_power_w;    /* 电容测得的底盘功率，单位 W */
    float supercap_voltage_v; /* 电容组电压，单位 V */
    float chassis_voltage_v;  /* 底盘母线电压，单位 V */
} super_capacitor_data_t;

/* 解析一帧状态数据并刷新在线时间。调用上下文待接线时确定；
 * 若在 CAN 接收中断中调用，读取方须改用任务/ISR 临界区保护，调度锁不能覆盖中断。 */
void super_capacitor_update_status(const uint8_t data[SUPER_CAPACITOR_STATUS_LENGTH]);
const super_capacitor_data_t *super_capacitor_get_data(void);

/* 保存控制层给出的充电功率上限（单位 W）和输出使能，供组帧使用。 */
void super_capacitor_set_command(float charge_power_limit_w, bool enabled);
/* 按当前保存的指令生成载荷：字节 0 为充电功率上限（0～255 W），字节 1 为使能。 */
void super_capacitor_build_command(uint8_t data[SUPER_CAPACITOR_COMMAND_LENGTH]);

#endif //SENTRY_CHASSIS_SUPERCAPACITOR_H
