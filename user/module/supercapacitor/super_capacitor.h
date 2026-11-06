#ifndef SENTRY_CHASSIS_SUPERCAPACITOR_H
#define SENTRY_CHASSIS_SUPERCAPACITOR_H

#include <stdbool.h>
#include <stdint.h>

/* 超级电容状态存储：接收层收到有效帧后调用 super_capacitor_update 刷新。
 * 电容通讯协议尚未接入，未收到帧时 online 为假，功率控制按电容离线处理。 */
typedef struct
{
    bool online;          /* 通讯在线，由 super_capacitor_get_data 按最近更新时间判定 */
    uint8_t dcdc_enabled; /* DCDC 使能状态 */
    uint8_t error_code;   /* 错误码，0 为无错误 */
    uint8_t energy;       /* 剩余能量原始值 0~255 */
    float chassis_power;  /* 电容测得的底盘功率 W */
} super_capacitor_data_t;

void super_capacitor_update(uint8_t dcdc_enabled, uint8_t error_code, uint8_t energy, float chassis_power);
const super_capacitor_data_t *super_capacitor_get_data(void);

#endif //SENTRY_CHASSIS_SUPERCAPACITOR_H
