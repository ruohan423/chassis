/* PowerController.c - C port of Mecanum_and_Omni/PowerController.cpp, 逐段对应原实现 */
#include "PowerController.h"

#if USE_POWER_CONTROLLER

#include <math.h>
#include <stddef.h>
#include <assert.h>

#define POWER_PD_KP 50.0f

/* 原工程: static_cast<float>((now - lastUpdateTick) / configTICK_RATE_HZ)
 * 这是整数除法, 1 kHz 下恒为 0, 电容离线能量积分实际不生效.
 * 置 POWER_KEEP_ORIGINAL_INT_DIV=1 完全复刻原行为; 默认按意图改为浮点秒. */
#ifndef POWER_KEEP_ORIGINAL_INT_DIV
#define POWER_KEEP_ORIGINAL_INT_DIV 0
#endif
#if POWER_KEEP_ORIGINAL_INT_DIV
#define TICKS_TO_SEC(t) ((float)((t) / POWER_TICK_RATE_HZ))
#else
#define TICKS_TO_SEC(t) ((float)(t) / (float)POWER_TICK_RATE_HZ)
#endif

const uint8_t HeroChassisPowerLimit_HP_FIRST[maxLevel]     = {55U, 60U, 65U, 70U, 75U, 80U, 85U, 90U, 100U, 120U};
const uint8_t InfantryChassisPowerLimit_HP_FIRST[maxLevel] = {45U, 50U, 55U, 60U, 65U, 70U, 75U, 80U, 90U, 100U};

static Manager manager;
static PowerStatus powerStatus;
static uint8_t  LATEST_FEEDBACK_JUDGE_ROBOT_LEVEL;
static uint8_t  isCapEnergyOut                = 0;
static uint16_t motorDisconnectCounter[4]     = {0U, 0U, 0U, 0U};
static float    MIN_MAXPOWER_CONFIGURED       = 30.0f;
static uint8_t  isInitialized                 = 0;
static PowerFeedbackSources sources;

/* Core::Control::PID::Param powerPDParam(POWER_PD_KP, 0.0f, 0.2f, 0.0f, MAX_CAP_POWER_OUT, 0.0f, 0.0f, 0.2f, 100UL); */
static PID powerPD_base;  /* ensure the system does not die */
static PID powerPD_full;  /* ensure the capacitor's energy keeps under 90% */

/* ---------------- helpers (与原文件 static inline 一一对应) ---------------- */
static inline float   clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
static inline uint8_t floatEqual(float a, float b) { return fabsf(a - b) < 1e-5f; }
static inline float   rpm2av(float rpm) { return rpm * (float)M_PI / 30.0f; }
static inline float   av2rpm(float av) { return av * 30.0f / (float)M_PI; }
static inline void    setErrorFlag(uint8_t *curFlag, enum ErrorFlags setFlag) { *curFlag |= (uint8_t)setFlag; }
static inline void    clearErrorFlag(uint8_t *curFlag, enum ErrorFlags clearFlag) { *curFlag &= (uint8_t)(~(uint8_t)clearFlag); }
static inline uint8_t isFlagged(uint8_t curFlag, enum ErrorFlags flag) { return (curFlag & (uint8_t)flag) != 0; }
static inline uint8_t isMotorConnected(const AbstractFeedbackMotor *motor)
{
    return motor != NULL && motor->getDisconnectCounter(motor) < 10;
}
static inline uint8_t refereeIsConnected(void) { return sources.refereeIsConnected ? sources.refereeIsConnected() : 0; }

const AbstractFeedbackMotor *Motors_at(const Motors *m, int index)
{
    switch (index) {
    case 0: return m->motorRf;
    case 1: return m->motorLf;
    case 2: return m->motorLb;
    case 3: return m->motorRb;
    default: return m->motorRf;
    }
}

static uint8_t isAllMotorConnected(void)
{
    int i;
    for (i = 0; i < 4; i++)
        if (!isMotorConnected(Motors_at(&manager.motors, i)))
            return 0;
    return 1;
}

/* ---------------- Manager::Manager ---------------- */
void Manager_construct(Manager *m, const Motors *motors_, Division division_, RLSEnabled rlsEnabled_,
                       float k1_, float k2_, float k3_, float lambda_)
{
    float initParams[2];
    (void)lambda_;  /* 原工程亦未使用该实参 */
    m->rlsEnabled             = rlsEnabled_;
    m->error                  = 0U;
    m->motors                 = *motors_;
    m->division               = division_;
    m->powerBuff              = 0.0f;
    m->fullBuffSet            = 0.0f;
    m->baseBuffSet            = 0.0f;
    m->fullMaxPower           = 0.0f;
    m->baseMaxPower           = 0.0f;
    m->powerUpperLimit        = 0.0f;
    m->refereeMaxPower        = 0.0f;
    m->userConfiguredMaxPower = 0.0f;
    m->callback               = NULL;
    m->measuredPower          = 0.0f;
    m->estimatedPower         = 0.0f;
    m->estimatedCapEnergy     = 0.0f;
    m->torqueConst            = 0.0f;
    m->k1                     = k1_;
    m->k2                     = k2_;
    m->k3                     = k3_;
    m->lastUpdateTick         = 0;
    RLS_construct(&m->rls, 1e-5f, 0.99999f);

    assert(k1_ >= 0);
    assert(k2_ >= 0);
    assert(k3_ >= 0);

    initParams[0] = k1_;
    initParams[1] = k2_;
    RLS_setParamVector(&m->rls, initParams);
}

/* ---------------- 小接口 ---------------- */
void Power_setMaxPowerConfigured(float maxPower)
{
    manager.userConfiguredMaxPower = clampf(maxPower, MIN_MAXPOWER_CONFIGURED, manager.powerUpperLimit);
}

void Power_setMode(uint8_t mode) { Power_setMaxPowerConfigured(mode == 1 ? manager.powerUpperLimit : manager.refereeMaxPower); }

void Power_registerPowerCallbackFunc(float (*callback)(void)) { manager.callback = callback; }

void Power_setRLSEnabled(uint8_t enable) { manager.rlsEnabled = (RLSEnabled)enable; }

const volatile PowerStatus *Power_getPowerStatus(void) { return (const volatile PowerStatus *)&powerStatus; }

float Power_getLatestFeedbackJudgePowerLimit(void) { return manager.refereeMaxPower; }

const Manager *Power_getManager(void) { return &manager; }

/* ---------------- getControlledOutput ---------------- */
float *Power_getControlledOutput(PowerObj *objs[4])
{
    const AbstractFeedbackMotor *m0 = Motors_at(&manager.motors, 0);
    const float k0 = manager.torqueConst * m0->getCurrentLimit(m0) /
                     m0->getOutputLimit(m0);  /* torque current rate of the motor, defined as Nm/Output */

    static float newTorqueCurrent[4];

    float sumCmdPower = 0.0f;
    float cmdPower[4];

    float sumError = 0.0f;
    float error[4];

    float maxPower = clampf(manager.userConfiguredMaxPower, manager.fullMaxPower, manager.baseMaxPower);

    float allocatablePower = maxPower;
    float sumPowerRequired = 0.0f;
#if USE_DEBUG
    static float newCmdPower;
#endif
    int i;

    for (i = 0; i < 4; i++) {
        const AbstractFeedbackMotor *motor = Motors_at(&manager.motors, i);
        if (isMotorConnected(motor)) {
            PowerObj *p = objs[i];
            cmdPower[i] = p->pidOutput * k0 * p->curAv + fabsf(p->curAv) * manager.k1 +
                          p->pidOutput * k0 * p->pidOutput * k0 * manager.k2 + manager.k3 / (float)4;
            sumCmdPower += cmdPower[i];
            error[i] = fabsf(p->setAv - p->curAv);
            if (floatEqual(cmdPower[i], 0.0f) || cmdPower[i] < 0.0f) {
                allocatablePower += -cmdPower[i];
            } else {
                sumError += error[i];
                sumPowerRequired += cmdPower[i];
            }
        } else if (motorDisconnectCounter[i] < 1000U && motor != NULL) {
            float tq = motor->getTorqueFeedback(motor);
            float av = rpm2av(motor->getRPMFeedback(motor));
            cmdPower[i] = tq * av + fabsf(av) * manager.k1 + tq * tq * manager.k2 + manager.k3 / 4.0f;
            error[i]    = 0.0f;
        } else {
            cmdPower[i] = 0.0f;
            error[i]    = 0.0f;
        }
    }

    /* update power status */
    powerStatus.maxPowerLimited          = maxPower;
    powerStatus.sumPowerCmd_before_clamp = sumCmdPower;

    if (sumCmdPower > maxPower) {
        float errorConfidence;
        if (sumError > error_powerDistribution_set) {
            errorConfidence = 1.0f;
        } else if (sumError > prop_powerDistribution_set) {
            errorConfidence = clampf((sumError - prop_powerDistribution_set) / (error_powerDistribution_set - prop_powerDistribution_set), 0.0f, 1.0f);
        } else {
            errorConfidence = 0.0f;
        }
        for (i = 0; i < 4; i++) {
            PowerObj *p = objs[i];
            if (isMotorConnected(Motors_at(&manager.motors, i))) {
                float powerWeight_Error, powerWeight_Prop, powerWeight, delta;
                if (floatEqual(cmdPower[i], 0.0f) || cmdPower[i] < 0.0f) {
                    newTorqueCurrent[i] = p->pidOutput;
                    continue;
                }
                powerWeight_Error = fabsf(p->setAv - p->curAv) / sumError;
                powerWeight_Prop  = cmdPower[i] / sumPowerRequired;
                powerWeight       = errorConfidence * powerWeight_Error + (1.0f - errorConfidence) * powerWeight_Prop;
                delta             = p->curAv * p->curAv -
                        4.0f * manager.k2 * (manager.k1 * fabsf(p->curAv) + manager.k3 / (float)4 - powerWeight * allocatablePower);
                if (floatEqual(delta, 0.0f)) {  /* repeat roots */
                    newTorqueCurrent[i] = -p->curAv / (2.0f * manager.k2) / k0;
                } else if (delta > 0.0f) {  /* distinct roots */
                    newTorqueCurrent[i] = p->pidOutput > 0.0f ? (-p->curAv + sqrtf(delta)) / (2.0f * manager.k2) / k0
                                                              : (-p->curAv - sqrtf(delta)) / (2.0f * manager.k2) / k0;
                } else {  /* imaginary roots */
                    newTorqueCurrent[i] = -p->curAv / (2.0f * manager.k2) / k0;
                }
                newTorqueCurrent[i] = clampf(newTorqueCurrent[i], -p->pidMaxOutput, p->pidMaxOutput);
            } else {
                newTorqueCurrent[i] = 0.0f;
            }
        }
    } else {
        for (i = 0; i < 4; i++) {
            if (isMotorConnected(Motors_at(&manager.motors, i))) {
                newTorqueCurrent[i] = objs[i]->pidOutput;
            } else {
                newTorqueCurrent[i] = 0.0f;
            }
        }
    }

#if USE_DEBUG
    newCmdPower = 0.0f;
    for (i = 0; i < 4; i++) {
        PowerObj *p = objs[i];
        newCmdPower += newTorqueCurrent[i] * k0 * p->curAv + fabsf(p->curAv) * manager.k1 +
                       newTorqueCurrent[i] * k0 * newTorqueCurrent[i] * k0 * manager.k2 + manager.k3 / 4.0f;
    }
#endif

    return newTorqueCurrent;
}

/* ---------------- setErrorFlag() 无参重载 ---------------- */
static void setErrorFlags(void)
{
    /*Judge the error status*/
#if USE_SUPER_CAPACITOR
    {
        const CapacitorStatus *capStatus = sources.capStatus;
        if (capStatus == NULL || !capStatus->isConnected || !capStatus->capacitorTx.enableDCDC || capStatus->capacitorRx.errorCode != 0)
            setErrorFlag(&manager.error, CAPDisConnect);
        else
            clearErrorFlag(&manager.error, CAPDisConnect);
    }
#else
    setErrorFlag(&manager.error, CAPDisConnect);
#endif

    if (!refereeIsConnected())
        setErrorFlag(&manager.error, RefereeDisConnect);
    else
        clearErrorFlag(&manager.error, RefereeDisConnect);

    if (!isAllMotorConnected())
        setErrorFlag(&manager.error, MotorDisconnect);
    else
        clearErrorFlag(&manager.error, MotorDisconnect);
}

/* ---------------- powerDaemon: 任务启动段 ---------------- */
void Power_daemonInit(uint32_t nowTick)
{
    const AbstractFeedbackMotor *m0 = Motors_at(&manager.motors, 0);
    manager.torqueConst = m0->getKA(m0) * m0->getReductionRatio(m0);
    isInitialized       = 1;
    /* 原工程此处 vTaskDelay(1000); 由调用方在任务里自行延时 */
    manager.lastUpdateTick = nowTick;
}

/* ---------------- powerDaemon: while(true) 循环体 ---------------- */
void Power_daemonStep(uint32_t now)
{
    static float samples[2];
    static const float *params;
    static float effectivePower = 0;
    const RefereePowerHeatMessageData   *powerMessage = sources.powerMessage;
    const RefereeRobotStatusMessageData *robotMessage = sources.robotMessage;
#if USE_SUPER_CAPACITOR
    const CapacitorStatus *capStatus = sources.capStatus;
#endif
    int i;

    setErrorFlags();

    /* update rls state and check whether cap energy is out even when cap disconnect to utilize credible data from referee system
     * estimate the cap energy if cap disconnect; estimated cap energy = cap energy feedback when cap is connected */
#if USE_SUPER_CAPACITOR
    if (isFlagged(manager.error, CAPDisConnect)) {
        if (!isFlagged(manager.error, RefereeDisConnect)) {
            if (powerMessage->bufferEnergy < MAX_POEWR_REFEREE_BUFF && powerMessage->chassisPower > CAP_OFFLINE_ENERGY_RUNOUT_POWER_THRESHOLD) {
                isCapEnergyOut             = 1;
                manager.estimatedCapEnergy = 0.0f;
            } else {
                isCapEnergyOut     = 0;
                manager.rlsEnabled = Disable;
                if (powerMessage->chassisPower < MIN_MAXPOWER_CONFIGURED && powerMessage->bufferEnergy == 60U) {
                    manager.estimatedCapEnergy = 2100.0f;
                } else {
                    manager.estimatedCapEnergy += (powerMessage->chassisPower - manager.estimatedPower) * TICKS_TO_SEC(now - manager.lastUpdateTick);
                    manager.estimatedCapEnergy = clampf(manager.estimatedCapEnergy, 0.0f, 2100.0f);
                }
            }
        } else {
            isCapEnergyOut     = 0;
            manager.rlsEnabled = Disable;
            manager.estimatedCapEnergy += (CAP_OFFLINE_ENERGY_TARGET_POWER - manager.estimatedPower) * TICKS_TO_SEC(now - manager.lastUpdateTick);
            manager.estimatedCapEnergy = clampf(manager.estimatedCapEnergy, 0.0f, 2100.0f);
        }
    } else {
        isCapEnergyOut             = 0;
        manager.estimatedCapEnergy = capStatus->capacitorRx.capEnergy / 255.0f * 2100.0f;
    }
#else /* Only Use Referee System, disable the rls update if referee data is invalid */
    if (isFlagged(manager.error, RefereeDisConnect)) {
        manager.rlsEnabled = Disable;
    }
    isCapEnergyOut             = 0;
    manager.estimatedCapEnergy = 0.0f;
#endif

    /* Set the energy feedback based on the current error status: cap first, then referee buffer */
#if USE_SUPER_CAPACITOR
    if (!isFlagged(manager.error, CAPDisConnect))
        manager.powerBuff = capStatus->capacitorRx.capEnergy;
    else if (!isFlagged(manager.error, RefereeDisConnect))
        manager.powerBuff = powerMessage->bufferEnergy;
#else
    if (!isFlagged(manager.error, RefereeDisConnect))
        manager.powerBuff = powerMessage->bufferEnergy;
#endif

    /* Set the energy target based on the current error status */
#if USE_SUPER_CAPACITOR
    if (!isFlagged(manager.error, CAPDisConnect)) {
        manager.fullBuffSet = capFullBuffSet;
        manager.baseBuffSet = capBaseBuffSet;
    } else {
        manager.fullBuffSet = refereeFullBuffSet;
        manager.baseBuffSet = refereeBaseBuffSet;
    }
#else
    manager.fullBuffSet = refereeFullBuffSet;
    manager.baseBuffSet = refereeBaseBuffSet;
#endif

    /* Update the referee maximum power limit and user configured power limit
     * If disconnected, then restore the last robot level and find corresponding chassis power limit */
    if (!isFlagged(manager.error, RefereeDisConnect)) {
        manager.refereeMaxPower = fmaxf((float)robotMessage->chassis_power_limit, CAP_OFFLINE_ENERGY_RUNOUT_POWER_THRESHOLD);
        if (robotMessage->robot_level > 10U)
            LATEST_FEEDBACK_JUDGE_ROBOT_LEVEL = 1U;
        else
            LATEST_FEEDBACK_JUDGE_ROBOT_LEVEL = (uint8_t)fmaxf(1.0f, (float)robotMessage->robot_level);
#if USE_SUPER_CAPACITOR
        if (isFlagged(manager.error, CAPDisConnect))
            manager.powerUpperLimit = manager.refereeMaxPower + POWER_PD_KP * (sqrtf(refereeFullBuffSet) - sqrtf(refereeBaseBuffSet));
        else
            manager.powerUpperLimit = manager.refereeMaxPower + MAX_CAP_POWER_OUT;
#else
        manager.powerUpperLimit = manager.refereeMaxPower;
#endif
    } else {
        switch (manager.division) {
        case HERO:     manager.refereeMaxPower = HeroChassisPowerLimit_HP_FIRST[LATEST_FEEDBACK_JUDGE_ROBOT_LEVEL - 1U]; break;
        case INFANTRY: manager.refereeMaxPower = InfantryChassisPowerLimit_HP_FIRST[LATEST_FEEDBACK_JUDGE_ROBOT_LEVEL - 1U]; break;
        case SENTRY:   manager.refereeMaxPower = SentryChassisPowerLimit; break;
        default:       assert(0); break;
        }
        /* Since we have less available feedback, we constrain the power conservatively */
#if USE_SUPER_CAPACITOR
        if (isFlagged(manager.error, CAPDisConnect)) {
            manager.powerUpperLimit = manager.refereeMaxPower * CAP_REFEREE_BOTH_GG_COE;
        } else {
            manager.powerUpperLimit = manager.refereeMaxPower + MAX_CAP_POWER_OUT;
        }
#else
        manager.powerUpperLimit = manager.refereeMaxPower * CAP_REFEREE_BOTH_GG_COE;
#endif
    }

    MIN_MAXPOWER_CONFIGURED = manager.refereeMaxPower * 0.8f;

    /* energy loop
     * if cap and referee both gg, set the max power to latest power limit * 0.85 and disable energy loop
     * if referee gg, set the max power to latest power limit * 0.95, enable energy loop when cap energy out */
    if (isFlagged(manager.error, CAPDisConnect) && isFlagged(manager.error, RefereeDisConnect)) {
        manager.baseMaxPower = manager.fullMaxPower = manager.refereeMaxPower * CAP_REFEREE_BOTH_GG_COE;
        PID_reset(&powerPD_base);
        PID_reset(&powerPD_full);
    } else {
        manager.baseMaxPower = fmaxf(manager.refereeMaxPower - PID_run(&powerPD_base, sqrtf(manager.baseBuffSet), sqrtf(manager.powerBuff)), MIN_MAXPOWER_CONFIGURED);
        manager.fullMaxPower = fmaxf(manager.refereeMaxPower - PID_run(&powerPD_full, sqrtf(manager.fullBuffSet), sqrtf(manager.powerBuff)), MIN_MAXPOWER_CONFIGURED);
    }

    /* if user has self defined power curve, use it */
    if (manager.callback != NULL)
        Power_setMaxPowerConfigured(manager.callback());

    /* Estimate the power based on the current model */
    effectivePower = 0;
    samples[0]     = 0;
    samples[1]     = 0;
    for (i = 0; i < 4; i++) {
        const AbstractFeedbackMotor *motor = Motors_at(&manager.motors, i);
        if (isMotorConnected(motor)) {
            motorDisconnectCounter[i] = 0U;
        } else {
            motorDisconnectCounter[i]++;
        }
        if (motorDisconnectCounter[i] < 1000U && motor != NULL) {
            /* We consider motor that is just disconnected as still using power, by assuming the motor keep
             * latest output and rpm by 1 second, otherwise it is not safe if we only use energy loop */
            float tq = motor->getTorqueFeedback(motor);
            float av = rpm2av(motor->getRPMFeedback(motor));
            effectivePower += tq * av;
            samples[0] += fabsf(av);
            samples[1] += tq * tq;
        } else {
            motorDisconnectCounter[i] = 1000U;
        }
    }
    manager.estimatedPower = manager.k1 * samples[0] + manager.k2 * samples[1] + effectivePower + manager.k3;

    /* Get the measured power from cap; if cap is disconnected, from referee if cap energy is out; otherwise estimated */
#if USE_SUPER_CAPACITOR
    if (!isFlagged(manager.error, CAPDisConnect)) {
        manager.measuredPower = capStatus->capacitorRx.chassisPower;
    } else if (!isFlagged(manager.error, RefereeDisConnect) && isCapEnergyOut) {
        manager.measuredPower = powerMessage->chassisPower;
    } else {
        manager.measuredPower = manager.estimatedPower;
    }
#else
    if (!isFlagged(manager.error, RefereeDisConnect)) {
        manager.measuredPower = powerMessage->chassisPower;
    } else {
        manager.measuredPower = manager.estimatedPower;
    }
#endif

    /* update power status */
    powerStatus.userConfiguredMaxPower = manager.userConfiguredMaxPower;
    powerStatus.effectivePower         = effectivePower;
    powerStatus.powerLoss              = manager.measuredPower - effectivePower;
    powerStatus.efficiency             = clampf(effectivePower / manager.measuredPower, 0.0f, 1.0f);
    powerStatus.estimatedCapEnergy     = (uint8_t)(manager.estimatedCapEnergy / 2100.0f * 255.0f);
    powerStatus.error                  = manager.error;

    /* Update the RLS parameters AND add dead zone AND
     * The Referee System could not detect negative power, so use estimated power to evaluate this situation */
    if (manager.rlsEnabled == Enable && fabsf(manager.measuredPower) > 5.0f &&
        !(isFlagged(manager.error, CAPDisConnect) && manager.estimatedPower < 0)) {
        params     = RLS_update(&manager.rls, samples, manager.measuredPower - effectivePower - manager.k3, now);
        manager.k1 = fmaxf(params[0], 1e-5f);  /* In case the k1 diverge to negative number */
        manager.k2 = fmaxf(params[1], 1e-5f);  /* In case the k2 diverge to negative number */
    }

    manager.lastUpdateTick = now;
    /* 原工程: vTaskDelay(pdMS_TO_TICKS(1)); 由调用方任务负责 */
}

/* ---------------- init ---------------- */
void Power_init(const Manager *mana, const PowerFeedbackSources *src)
{
    if (isInitialized)
        return;
    manager = *mana;
    sources = *src;

    /* default value */
    LATEST_FEEDBACK_JUDGE_ROBOT_LEVEL = manager.division == SENTRY ? 10U : 1U;
    MIN_MAXPOWER_CONFIGURED           = 30.0f;
    manager.powerUpperLimit           = CAP_OFFLINE_ENERGY_RUNOUT_POWER_THRESHOLD + MAX_CAP_POWER_OUT;

    /* PID::Param(POWER_PD_KP, 0.0f, 0.2f, 0.0f, MAX_CAP_POWER_OUT, 0.0f, 0.0f, 0.2f, 100UL) */
    PID_init(&powerPD_base, POWER_PD_KP, 0.0f, 0.2f, 0.0f, MAX_CAP_POWER_OUT, 0.2f, 1.0f / (float)POWER_TICK_RATE_HZ);
    PID_init(&powerPD_full, POWER_PD_KP, 0.0f, 0.2f, 0.0f, MAX_CAP_POWER_OUT, 0.2f, 1.0f / (float)POWER_TICK_RATE_HZ);

    /* 原工程: subscribeMessage(&powerMessage/&robotMessage) -> 由 sources 指针代替
     *         xTaskCreateStatic(powerDaemon, "power", 1024, nullptr, 10, ...) -> 由调用方创建任务 */
}

#endif /* USE_POWER_CONTROLLER */
