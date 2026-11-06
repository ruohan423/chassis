#ifndef SENTRY_CHASSIS_CHASSIS_PORT_H
#define SENTRY_CHASSIS_CHASSIS_PORT_H

#include <stdbool.h>
#include <stdint.h>

#include "chassis_config.h"

/* 给两个控制器提供归一化电机反馈及输出通道。
 * 均按逻辑轮序和安装方向归一化；控制器不接触 HAL 句柄或电机协议结构。 */
typedef struct
{
    volatile float speed_rad_s;             /* 轮侧角速度 rad/s */
    volatile float torque_nm;               /* 轮侧力矩 Nm，含减速比 */
    float output;                  /* 方向归一化后的电流原始指令 */
    volatile uint32_t feedback_tick;         /* HAL 毫秒时钟，仅 port 层解释 */
    volatile bool has_feedback;
} chassis_motor_t;

extern chassis_motor_t chassis_motors[CHASSIS_MOTOR_COUNT];

bool chassis_port_init(void);
bool chassis_port_send(void);
bool chassis_motor_is_online(const chassis_motor_t *motor);

#if USE_POWER_CONTROLLER
/* 移植功率控制器使用的输入格式，区别于设备模块的原始协议结构。 */
typedef struct {
    uint16_t bufferEnergy;   /* J */
    float    chassisPower;   /* W */
    uint8_t  chassisPowerValid; /* 必须有真实测量来源，保留字段不能作为功率反馈。 */
} RefereePowerHeatMessageData;

typedef struct {
    uint16_t chassis_power_limit;
    uint8_t  robot_level;
} RefereeRobotStatusMessageData;

typedef struct {
    uint8_t isConnected;
    struct { uint8_t enableDCDC; } capacitorTx;
    struct { uint8_t errorCode; uint8_t capEnergy; float chassisPower; } capacitorRx;  /* capEnergy 0~255 */
} CapacitorStatus;

typedef struct {
    const RefereePowerHeatMessageData   *powerMessage;
    const RefereeRobotStatusMessageData *robotMessage;
    const CapacitorStatus               *capStatus;      /* 未用电容时可为 NULL */
    uint8_t (*refereeIsConnected)(void);
} PowerFeedbackSources;

/* 控制器只读取适配后的稳定对象，协议快照在 port 内部转换。 */
extern const PowerFeedbackSources chassis_power_sources;
void chassis_port_update_feedback(uint32_t now_tick);
#endif

#endif
