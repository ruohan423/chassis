/* 移植功率控制器：功率分配、能量环、RLS 与离线降级。
 * 电机由 chassis_port 提供轮侧反馈；所有实车可配置参数见 chassis_config.h。 */
#ifndef SENTRY_CHASSIS_POWERCONTROLLER_H
#define SENTRY_CHASSIS_POWERCONTROLLER_H

#include <stdint.h>

#include "pid.h"
#include "rls.h"
#include "chassis_port.h"

#if USE_POWER_CONTROLLER

extern const uint8_t HeroChassisPowerLimit_HP_FIRST[CHASSIS_POWER_LEVEL_COUNT];
extern const uint8_t InfantryChassisPowerLimit_HP_FIRST[CHASSIS_POWER_LEVEL_COUNT];

typedef enum { INFANTRY = 0, HERO, SENTRY } Division;

/* ---------------- Manager ---------------- */
typedef enum { Disable = 0, Enable = 1 } RLSEnabled;

enum ErrorFlags {
    MotorDisconnect   = 1U,
    RefereeDisConnect = 2U,
    CAPDisConnect     = 4U
};

typedef struct {
    RLSEnabled rlsEnabled;
    uint8_t    error;

    Division division;

    float powerBuff;
    float fullBuffSet;
    float baseBuffSet;
    float fullMaxPower;
    float baseMaxPower;

    float powerUpperLimit;
    float refereeMaxPower;

    float userConfiguredMaxPower;
    float (*callback)(void);

    float measuredPower;
    float estimatedPower;
    float estimatedCapEnergy;

    float torqueConst;

    float k1;
    float k2;
    float k3;

    uint32_t lastUpdateTick;
    uint32_t tick_frequency_hz;
    uint32_t motor_hold_ticks;
    float sample_period_s;
    RLS rls;
} Manager;

/* lambda_ 为实际使用的遗忘因子；初始协方差由统一配置中的 delta 给出。 */
void Manager_construct(Manager *m, Division division_, RLSEnabled rlsEnabled_,
                       float k1_, float k2_, float k3_, float lambda_);

typedef struct {
    float pidOutput;     /* torque current command, [-maxOutput, maxOutput], no unit */
    float curAv;         /* Measured angular velocity, rad/s */
    float setAv;         /* target angular velocity, rad/s */
    float pidMaxOutput;  /* pid max output */
} PowerObj;

typedef struct {
    float userConfiguredMaxPower;
    float maxPowerLimited;
    float sumPowerCmd_before_clamp;
    float effectivePower;
    float powerLoss;
    float efficiency;
    uint8_t estimatedCapEnergy;
    uint8_t error;   /* ErrorFlags 位域 */
} PowerStatus;

/* ---------------- 接口---------------- */
float  Power_getLatestFeedbackJudgePowerLimit(void);
float *Power_getControlledOutput(PowerObj *objs[4]);
const volatile PowerStatus *Power_getPowerStatus(void);
void   Power_init(const Manager *manager, const PowerFeedbackSources *sources);
void   Power_setMaxPowerConfigured(float maxPower);
void   Power_setMode(uint8_t mode);
void   Power_registerPowerCallbackFunc(float (*callback)(void));
void   Power_setRLSEnabled(uint8_t isUpdate);

/* 启动后设置积分起点，之后由任务按统一配置周期调用。 */
void   Power_daemonInit(uint32_t nowTick);
void   Power_daemonStep(uint32_t nowTick);

#endif /* USE_POWER_CONTROLLER */
#endif //SENTRY_CHASSIS_POWERCONTROLLER_H
