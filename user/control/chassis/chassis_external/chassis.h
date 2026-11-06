#ifndef SENTRY_CHASSIS_CHASSIS_H
#define SENTRY_CHASSIS_CHASSIS_H

#include <stdbool.h>
#include <stdint.h>

#include "../chassis_config.h"

typedef enum
{
    CHASSIS_STOP = 0,
    CHASSIS_NORMAL,
    CHASSIS_FOLLOW_GIMBAL,
    CHASSIS_GYROSCOPE,
    CHASSIS_ROSHAN,
    CHASSIS_SZUPUP,
} chassis_mode_t;

/* 唯一写入者为遥控器任务；初始化保持 NORMAL，兼容既有停止态遥控复位流程。
 * 控制器上电输出仍为零，电机必须收到有效反馈才允许输出。 */
extern volatile chassis_mode_t actChassis;

/* 给任务提供初始化、行为目标生成和控制周期接口。
 * 输入已换算为 m/s、deg，时钟来自 CMSIS-OS2。
 * 初始化/下发失败返回 false，调用任务负责延时和重试。 */
bool chassis_init(uint32_t now_tick, uint32_t tick_frequency_hz);
bool chassis_update(float input_x, float input_y, float gimbal_angle_deg, bool gimbal_valid, uint32_t now_tick);

/* 创建任务前初始化功率状态；功率任务启动延时后重新设置积分起点。 */
void chassis_power_init(uint32_t now_tick, uint32_t tick_frequency_hz);
void chassis_power_start(uint32_t now_tick);
void chassis_power_update(uint32_t now_tick);

#endif
