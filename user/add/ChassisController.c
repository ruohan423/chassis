/* ChassisController.c - C port of Mecanum_and_Omni/ChassisController.cpp */
#include "ChassisController.h"

#if USE_CHASSIS_CONTROLLER

#include <math.h>
#include <assert.h>

#include "PowerController.h"

#define ABS(x) ((x > 0) ? x : -x)
#define SQRT2 1.41421356237f
#define RPM_TO_ANGULAR_SPEED_DIV (30.0f / (float)M_PI)

#define CHASSIS_CLAMP(x, min, max) ((x < min) ? min : (x > max) ? max : x)
#define SIGN(x, y) (((x < 0 && y < 0) || (x > 0 && y > 0)) ? 1 : -1)

static inline float maxAccClamp(float velocity, float lastVelocity, float maxAcc, float timeDiff)
{
    float currentAcc = (velocity - lastVelocity) / timeDiff;
    return (currentAcc < -maxAcc) ? lastVelocity - timeDiff * maxAcc : (currentAcc > maxAcc) ? lastVelocity + timeDiff * maxAcc : velocity;
}

#if USE_MECANUM_CHASSIS
void ChassisPropertyConfig_construct(ChassisPropertyConfig *c, float wheelDiameters, float fbLength, float rlLength)
{
    c->chassisWheelDiameters  = wheelDiameters;
    c->chassisRightLeftLength = rlLength;
    c->chassisFrontBackLength = fbLength;
}
#else
void ChassisPropertyConfig_construct(ChassisPropertyConfig *c, float wheelDiameters, float wheelToCentre)
{
    c->chassisWheelDiameters       = wheelDiameters;
    c->chassisWheelToCentreLength  = wheelToCentre;
}
#endif

void ChassisConfigs_construct(ChassisConfigs *cfg,
                              AbstractFeedbackMotor *pMotorRf, AbstractFeedbackMotor *pMotorLf,
                              AbstractFeedbackMotor *pMotorLb, AbstractFeedbackMotor *pMotorRb,
                              PID *pPIDRf, PID *pPIDLf, PID *pPIDLb, PID *pPIDRb,
                              const ChassisPropertyConfig *property,
                              float maxAcceleration_, float maxAngularAccelertion_,
                              const Dynamics *maxSpeed_, uint32_t nowTick)
{
    cfg->chassisPropertyConfig = *property;
    cfg->chassisMotors[0] = pMotorRf; cfg->chassisMotors[1] = pMotorLf;
    cfg->chassisMotors[2] = pMotorLb; cfg->chassisMotors[3] = pMotorRb;
    cfg->chassisPIDs[0] = pPIDRf; cfg->chassisPIDs[1] = pPIDLf;
    cfg->chassisPIDs[2] = pPIDLb; cfg->chassisPIDs[3] = pPIDRb;
    cfg->lastUpdate             = nowTick;
    cfg->maxAcceleration        = maxAcceleration_;
    cfg->maxAngularAcceleration = maxAngularAccelertion_;
    cfg->maxSpeed               = *maxSpeed_;
}

void ChassisController_construct(ChassisController *ctl, const ChassisConfigs *chassisConfig_)
{
    int i;
    static const Dynamics zero = {0.0f, 0.0f, 0.0f};
    for (i = 0; i < 4; i++) { ctl->wheelsTargetAv[i] = 0.0f; ctl->wheelsOutput[i] = 0.0f; ctl->currentAv[i] = 0.0f; }
    ctl->chassisConfigs = *chassisConfig_;
    ctl->rawTargetVelocity = ctl->lastRawTargetVelocity = ctl->targetVelocity = ctl->lastTargetVelocity = zero;
    ctl->measuredVelocity = ctl->lastMeasuredVelocity = ctl->measuredAcceleration = ctl->measuredDisplacement = zero;

    ctl->diameters = ctl->chassisConfigs.chassisPropertyConfig.chassisWheelDiameters;
#if USE_MECANUM_CHASSIS
    ctl->sum      = ctl->chassisConfigs.chassisPropertyConfig.chassisFrontBackLength + ctl->chassisConfigs.chassisPropertyConfig.chassisRightLeftLength;
    ctl->wFactor  = -ctl->diameters / (4.0f * ctl->sum);
    ctl->xyFactor = ctl->diameters / (8.0f);
#else
    ctl->length   = ctl->chassisConfigs.chassisPropertyConfig.chassisWheelToCentreLength;
    ctl->wFactor  = -ctl->diameters / (8.0f * ctl->length);
    ctl->xyFactor = ctl->diameters / (4.0f * SQRT2);
#endif
}

void ChassisController_settargetVelocity(ChassisController *ctl, float x, float y, float w)
{
    ctl->lastRawTargetVelocity.x = ctl->rawTargetVelocity.x;
    ctl->lastRawTargetVelocity.y = ctl->rawTargetVelocity.y;
    ctl->lastRawTargetVelocity.w = ctl->rawTargetVelocity.w;
    ctl->rawTargetVelocity.x     = x;
    ctl->rawTargetVelocity.y     = y;
    ctl->rawTargetVelocity.w     = w;
    ctl->targetVelocity.x        = CHASSIS_CLAMP(x, -ctl->chassisConfigs.maxSpeed.x, ctl->chassisConfigs.maxSpeed.x);
    ctl->targetVelocity.y        = CHASSIS_CLAMP(y, -ctl->chassisConfigs.maxSpeed.y, ctl->chassisConfigs.maxSpeed.y);
    ctl->targetVelocity.w        = CHASSIS_CLAMP(w, -ctl->chassisConfigs.maxSpeed.w, ctl->chassisConfigs.maxSpeed.w);
}

void ChassisController_settargetVelocityArr(ChassisController *ctl, const float speeds[3])
{
    ChassisController_settargetVelocity(ctl, speeds[0], speeds[1], speeds[2]);
}

void ChassisController_setTargetPercentageSpeed(ChassisController *ctl, float vxPercentage, float vyPercentage, float vwPercentage)
{
    ChassisController_settargetVelocity(ctl, vxPercentage * ctl->chassisConfigs.maxSpeed.x,
                                        vyPercentage * ctl->chassisConfigs.maxSpeed.y,
                                        vwPercentage * ctl->chassisConfigs.maxSpeed.w);
}

const Dynamics *ChassisController_getMeasuredAccleration(ChassisController *ctl) { return &ctl->measuredAcceleration; }
const Dynamics *ChassisController_getMeasuredVelocity(ChassisController *ctl) { return &ctl->measuredVelocity; }
const Dynamics *ChassisController_getMeasuredDisplacement(ChassisController *ctl) { return &ctl->measuredDisplacement; }
void ChassisController_setMaxVelocity(ChassisController *ctl, const Dynamics *newMaxSpeeds) { ctl->chassisConfigs.maxSpeed = *newMaxSpeeds; }
void ChassisController_setMaxAngularAcceleration(ChassisController *ctl, float maxAngularAcceleration) { ctl->chassisConfigs.maxAngularAcceleration = maxAngularAcceleration; }
void ChassisController_setMaxAcceleration(ChassisController *ctl, float maxAcceleration) { ctl->chassisConfigs.maxAcceleration = maxAcceleration; }

static void solveMeasuredDynamics(ChassisController *ctl, uint32_t nowTick)
{
    int i;
    /* Update tick count */
    float timeDiff = (float)nowTick - (float)ctl->chassisConfigs.lastUpdate;
    ctl->chassisConfigs.lastUpdate = nowTick;

    /* Update last measured speeds */
    ctl->lastMeasuredVelocity.w = ctl->measuredVelocity.w;
    ctl->lastMeasuredVelocity.y = ctl->measuredVelocity.y;
    ctl->lastMeasuredVelocity.x = ctl->measuredVelocity.x;
    ctl->measuredVelocity.w     = 0.0f;
    for (i = 0; i < 4; i++) {
        AbstractFeedbackMotor *m = ctl->chassisConfigs.chassisMotors[i];
        ctl->currentAv[i] = m->getRPMFeedback(m) / RPM_TO_ANGULAR_SPEED_DIV;
    }
#if USE_MECANUM_CHASSIS
    ctl->measuredVelocity.x = (ctl->currentAv[2] + ctl->currentAv[3] - ctl->currentAv[1] - ctl->currentAv[0]) * ctl->xyFactor;
#else
    ctl->measuredVelocity.x = -(ctl->currentAv[2] + ctl->currentAv[3] - ctl->currentAv[0] - ctl->currentAv[1]) * ctl->xyFactor;
#endif
    ctl->measuredVelocity.y = (ctl->currentAv[1] + ctl->currentAv[2] - ctl->currentAv[0] - ctl->currentAv[3]) * ctl->xyFactor;
    ctl->measuredVelocity.w = (ctl->currentAv[0] + ctl->currentAv[1] + ctl->currentAv[2] + ctl->currentAv[3]) * ctl->wFactor;

    /* Get displacements data */
    ctl->measuredDisplacement.w += (ctl->measuredVelocity.w * timeDiff / (float)(CHASSIS_TICK_RATE_HZ));
    ctl->measuredDisplacement.y += (ctl->measuredVelocity.y * timeDiff / (float)(CHASSIS_TICK_RATE_HZ));
    ctl->measuredDisplacement.x += (ctl->measuredVelocity.x * timeDiff / (float)(CHASSIS_TICK_RATE_HZ));

    /* Get acceleration data */
    ctl->measuredAcceleration.w = (ctl->measuredVelocity.w - ctl->lastMeasuredVelocity.w) / (timeDiff / (float)(CHASSIS_TICK_RATE_HZ));
    ctl->measuredAcceleration.y = (ctl->measuredVelocity.y - ctl->lastMeasuredVelocity.y) / (timeDiff / (float)(CHASSIS_TICK_RATE_HZ));
    ctl->measuredAcceleration.x = (ctl->measuredVelocity.x - ctl->lastMeasuredVelocity.x) / (timeDiff / (float)(CHASSIS_TICK_RATE_HZ));
}

void ChassisController_update(ChassisController *ctl, uint32_t nowTick)
{
    int i;
    float pidOutput[4];
    const Dynamics *tv = &ctl->targetVelocity;

    solveMeasuredDynamics(ctl, nowTick);

    /* Calculate target RPM */
#if USE_MECANUM_CHASSIS
    ctl->wheelsTargetAv[0] = (-tv->y - tv->x - tv->w * (ctl->sum / 2.0f)) * 2.0f / ctl->diameters;
    ctl->wheelsTargetAv[1] = ( tv->y - tv->x - tv->w * (ctl->sum / 2.0f)) * 2.0f / ctl->diameters;
    ctl->wheelsTargetAv[2] = ( tv->y + tv->x - tv->w * (ctl->sum / 2.0f)) * 2.0f / ctl->diameters;
    ctl->wheelsTargetAv[3] = (-tv->y + tv->x - tv->w * (ctl->sum / 2.0f)) * 2.0f / ctl->diameters;
#else
    ctl->wheelsTargetAv[0] = (-SQRT2 / 2.0f * tv->x - SQRT2 / 2.0f * tv->y - tv->w * ctl->length) * 2.0f / ctl->diameters;
    ctl->wheelsTargetAv[1] = (-SQRT2 / 2.0f * tv->x + SQRT2 / 2.0f * tv->y - tv->w * ctl->length) * 2.0f / ctl->diameters;
    ctl->wheelsTargetAv[2] = ( SQRT2 / 2.0f * tv->x + SQRT2 / 2.0f * tv->y - tv->w * ctl->length) * 2.0f / ctl->diameters;
    ctl->wheelsTargetAv[3] = ( SQRT2 / 2.0f * tv->x - SQRT2 / 2.0f * tv->y - tv->w * ctl->length) * 2.0f / ctl->diameters;
#endif

    /* Calculate power and do power limitation */
#if USE_POWER_CONTROLLER
    {
        static PowerObj  objs[4];
        static PowerObj *pObjs[4] = {&objs[0], &objs[1], &objs[2], &objs[3]};
        float *controlledOutput;
        /* Update Power Objects */
        for (i = 0; i < 4; i++) {
            objs[i].curAv     = ctl->currentAv[i];
            objs[i].setAv     = ctl->wheelsTargetAv[i];
            objs[i].pidOutput = pidOutput[i] = PID_run(ctl->chassisConfigs.chassisPIDs[i], ctl->wheelsTargetAv[i], ctl->currentAv[i]);
            objs[i].pidMaxOutput             = ctl->chassisConfigs.chassisPIDs[i]->outputMax;
        }
        controlledOutput = Power_getControlledOutput(pObjs);
        for (i = 0; i < 4; i++) {
            ctl->wheelsOutput[i] = controlledOutput[i];
        }
    }
#else
    for (i = 0; i < 4; i++) {
        pidOutput[i]         = PID_run(ctl->chassisConfigs.chassisPIDs[i], ctl->wheelsTargetAv[i], ctl->currentAv[i]);
        ctl->wheelsOutput[i] = pidOutput[i];
    }
#endif
    for (i = 0; i < 4; i++) {
        AbstractFeedbackMotor *m = ctl->chassisConfigs.chassisMotors[i];
        m->setOutput(m, ctl->wheelsOutput[i]);
    }
}

void ChassisController_stop(ChassisController *ctl, uint32_t nowTick)
{
    int i;
    solveMeasuredDynamics(ctl, nowTick);
    for (i = 0; i < 4; i++) {
        AbstractFeedbackMotor *m = ctl->chassisConfigs.chassisMotors[i];
        PID_reset(ctl->chassisConfigs.chassisPIDs[i]);
        m->setOutput(m, 0);
    }
}

const ChassisConfigs *ChassisController_getConfig(ChassisController *ctl) { return &ctl->chassisConfigs; }

#undef ABS
#undef SQRT2
#undef RPM_TO_ANGULAR_SPEED_DIV
#undef CHASSIS_CLAMP
#undef SIGN

#endif /* USE_CHASSIS_CONTROLLER */
