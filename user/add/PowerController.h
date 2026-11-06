/**
 * @file PowerController.h
 * @version 2.0 (C port of Mecanum_and_Omni/PowerController.hpp)
 * @note The estimated power formula: P = τΩ + k1|Ω| + k2τ^2
 *
 * 与原工程的对应关系:
 *   namespace Core::Control::Power  ->  前缀 Power_
 *   struct Manager / PowerObj / PowerStatus  -> 同名 struct
 *   void powerDaemon(void*)  ->  Power_daemonInit() 一次 + Power_daemonStep(nowTick) 每 1 ms
 *   RefereeSystem / SuperCapacitor 数据  ->  PowerFeedbackSources 里的指针, 由你的通讯层维护
 */
#ifndef POWER_CONTROLLER_H
#define POWER_CONTROLLER_H

#include <stdint.h>
#include "AbstractFeedbackMotor.h"
#include "PID.h"
#include "RLS.h"

/* ---------------- 编译开关 (对应 AppConfig.h) ---------------- */
#ifndef USE_POWER_CONTROLLER
#define USE_POWER_CONTROLLER 1
#endif
#ifndef USE_SUPER_CAPACITOR
#define USE_SUPER_CAPACITOR 1
#endif
#ifndef USE_REFEREE_SYSTEM_COMM
#define USE_REFEREE_SYSTEM_COMM 1
#endif
#ifndef USE_DEBUG
#define USE_DEBUG 0
#endif
#if !(USE_REFEREE_SYSTEM_COMM)
#error Referee System Communication is not defined!
#endif
/* 对应 configTICK_RATE_HZ */
#ifndef POWER_TICK_RATE_HZ
#define POWER_TICK_RATE_HZ 1000U
#endif

#if USE_POWER_CONTROLLER

/* ---------------- 常量, 与 PowerController.hpp 一致 ---------------- */
#define refereeFullBuffSet          60.0f
#define refereeBaseBuffSet          50.0f
#define capFullBuffSet              230.0f
#define capBaseBuffSet              30.0f
#define error_powerDistribution_set 20.0f
#define prop_powerDistribution_set  15.0f

#define MAX_CAP_POWER_OUT                         300.0f
#define CAP_OFFLINE_ENERGY_RUNOUT_POWER_THRESHOLD 43.0f
#define CAP_OFFLINE_ENERGY_TARGET_POWER           37.0f
#define MAX_POEWR_REFEREE_BUFF                    60.0f
#define REFEREE_GG_COE                            0.95f
#define CAP_REFEREE_BOTH_GG_COE                   0.85f

#define maxLevel 10U
extern const uint8_t HeroChassisPowerLimit_HP_FIRST[maxLevel];
extern const uint8_t InfantryChassisPowerLimit_HP_FIRST[maxLevel];
#define SentryChassisPowerLimit 100U

typedef enum { INFANTRY = 0, HERO, SENTRY } Division;

/* ---------------- 外部反馈数据 (字段名沿用裁判系统/电容协议) ---------------- */
typedef struct {
    uint16_t bufferEnergy;   /* J */
    float    chassisPower;   /* W */
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

/* ---------------- Manager ---------------- */
typedef enum { Disable = 0, Enable = 1 } RLSEnabled;

enum ErrorFlags {
    MotorDisconnect   = 1U,
    RefereeDisConnect = 2U,
    CAPDisConnect     = 4U
};

typedef struct {
    const AbstractFeedbackMotor *motorRf;
    const AbstractFeedbackMotor *motorLf;
    const AbstractFeedbackMotor *motorLb;
    const AbstractFeedbackMotor *motorRb;
} Motors;
/* 对应 Motors::operator[] : 0 Rf, 1 Lf, 2 Lb, 3 Rb */
const AbstractFeedbackMotor *Motors_at(const Motors *m, int index);

typedef struct {
    RLSEnabled rlsEnabled;
    uint8_t    error;

    Motors   motors;
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
    RLS rls;
} Manager;

/* 对应 Manager::Manager(motors, division, rlsEnabled, k1, k2, k3, lambda)
 * 默认值: rlsEnabled=Enable, k1=0.22, k2=1.2, k3=2.78, lambda=0.9999
 * 注: 原工程 lambda_ 实参未被使用, RLS 固定用 (delta=1e-5, lambda=0.99999), 此处保持一致 */
void Manager_construct(Manager *m, const Motors *motors_, Division division_, RLSEnabled rlsEnabled_,
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

/* ---------------- 接口, 与原工程同名 ---------------- */
float  Power_getLatestFeedbackJudgePowerLimit(void);
float *Power_getControlledOutput(PowerObj *objs[4]);
const volatile PowerStatus *Power_getPowerStatus(void);
void   Power_init(const Manager *manager, const PowerFeedbackSources *sources);
void   Power_setMaxPowerConfigured(float maxPower);
void   Power_setMode(uint8_t mode);
void   Power_registerPowerCallbackFunc(float (*callback)(void));
void   Power_setRLSEnabled(uint8_t isUpdate);

/* 对应 powerDaemon: 原任务结构为
 *     torqueConst = KA * reductionRatio; isInitialized = true; vTaskDelay(1000); lastUpdateTick = now;
 *     while (true) { <循环体>; vTaskDelay(1ms); }
 * 移植为: 任务启动后调一次 Power_daemonInit(nowTick), 之后每 1 ms 调一次 Power_daemonStep(nowTick) */
void   Power_daemonInit(uint32_t nowTick);
void   Power_daemonStep(uint32_t nowTick);

/* 调试: 暴露内部 manager 便于看波形 */
const Manager *Power_getManager(void);

#endif /* USE_POWER_CONTROLLER */
#endif
