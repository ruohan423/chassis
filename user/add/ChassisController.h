/**
 * @file ChassisController.h  (C port of Mecanum_and_Omni/ChassisController.hpp)
 * @brief The first version of chassis controller, coupled with power controller
 *
 * The coordinate system for chassis
 *           ^ Y
 * 1         |        0
 *           |
 * ----------|----------> X
 *           |
 * 2         |        3
 * vw is positive for anti-clockwise rotate
 */
#ifndef CHASSIS_CONTROLLER_H
#define CHASSIS_CONTROLLER_H

#include <stdint.h>
#include "AbstractFeedbackMotor.h"
#include "PID.h"

#ifndef USE_CHASSIS_CONTROLLER
#define USE_CHASSIS_CONTROLLER 1
#endif
#if (!defined USE_OMNI_CHASSIS) && (!defined USE_MECANUM_CHASSIS)
#define USE_MECANUM_CHASSIS 1
#endif
#ifndef USE_MECANUM_CHASSIS
#define USE_MECANUM_CHASSIS 0
#endif
#ifndef CHASSIS_TICK_RATE_HZ
#define CHASSIS_TICK_RATE_HZ 1000U
#endif

#if USE_CHASSIS_CONTROLLER

/* Structure that stores the measured property datas of the controled chassis
 *  chassisWheelDiameters       wheel diameter (m)
 *  chassisRightLeftLength      distance between right/left wheel contact points   (mecanum)
 *  chassisFrontBackLength      distance between front/back wheel contact points   (mecanum)
 *  chassisWheelToCentreLength  wheel contact point to chassis centre              (omni, assumes circle) */
typedef struct {
    float chassisWheelDiameters;
#if USE_MECANUM_CHASSIS
    float chassisRightLeftLength;
    float chassisFrontBackLength;
#else
    float chassisWheelToCentreLength;
#endif
} ChassisPropertyConfig;

#if USE_MECANUM_CHASSIS
void ChassisPropertyConfig_construct(ChassisPropertyConfig *c, float wheelDiameters, float fbLength, float rlLength);
#else
void ChassisPropertyConfig_construct(ChassisPropertyConfig *c, float wheelDiameters, float wheelToCentre);
#endif

typedef struct { float x; float y; float w; } Dynamics;

/* Main chassis configurations data sheets. motors/pids 顺序: 0 Rf, 1 Lf, 2 Lb, 3 Rb */
typedef struct {
    ChassisPropertyConfig  chassisPropertyConfig;
    AbstractFeedbackMotor *chassisMotors[4];
    PID                   *chassisPIDs[4];
    uint32_t lastUpdate;
    float    maxAcceleration;
    float    maxAngularAcceleration;
    Dynamics maxSpeed;
} ChassisConfigs;

void ChassisConfigs_construct(ChassisConfigs *cfg,
                              AbstractFeedbackMotor *pMotorRf, AbstractFeedbackMotor *pMotorLf,
                              AbstractFeedbackMotor *pMotorLb, AbstractFeedbackMotor *pMotorRb,
                              PID *pPIDRf, PID *pPIDLf, PID *pPIDLb, PID *pPIDRb,
                              const ChassisPropertyConfig *property,
                              float maxAcceleration_, float maxAngularAccelertion_,
                              const Dynamics *maxSpeed_, uint32_t nowTick);

typedef struct {
    float wheelsTargetAv[4];
    float wheelsOutput[4];
    float currentAv[4];
    ChassisConfigs chassisConfigs;

    Dynamics rawTargetVelocity;
    Dynamics lastRawTargetVelocity;
    Dynamics targetVelocity;
    Dynamics lastTargetVelocity;
    Dynamics measuredVelocity;
    Dynamics lastMeasuredVelocity;
    Dynamics measuredAcceleration;
    Dynamics measuredDisplacement;

    float diameters;
#if USE_MECANUM_CHASSIS
    float sum;
#else
    float length;
#endif
    float wFactor;
    float xyFactor;
} ChassisController;

void ChassisController_construct(ChassisController *ctl, const ChassisConfigs *chassisConfig_);
void ChassisController_settargetVelocity(ChassisController *ctl, float x, float y, float w);
void ChassisController_settargetVelocityArr(ChassisController *ctl, const float speeds[3]);
void ChassisController_setTargetPercentageSpeed(ChassisController *ctl, float vxPercentage, float vyPercentage, float vwPercentage);
const Dynamics *ChassisController_getMeasuredAccleration(ChassisController *ctl);
const Dynamics *ChassisController_getMeasuredVelocity(ChassisController *ctl);
const Dynamics *ChassisController_getMeasuredDisplacement(ChassisController *ctl);
void ChassisController_setMaxVelocity(ChassisController *ctl, const Dynamics *newMaxSpeeds);
void ChassisController_setMaxAngularAcceleration(ChassisController *ctl, float maxAngularAcceleration);
void ChassisController_setMaxAcceleration(ChassisController *ctl, float maxAcceleration);
/* 原 update() 无参; nowTick 替代 xTaskGetTickCount() */
void ChassisController_update(ChassisController *ctl, uint32_t nowTick);
void ChassisController_stop(ChassisController *ctl, uint32_t nowTick);
const ChassisConfigs *ChassisController_getConfig(ChassisController *ctl);

#endif /* USE_CHASSIS_CONTROLLER */
#endif
