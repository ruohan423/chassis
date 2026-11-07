#include "power_controller.h"

#include "chassis_port.h"

#if USE_POWER_CONTROLLER

#include <assert.h>
#include <math.h>
#include <stddef.h>

/* 所有时间差使用调用方传入的 tick 频率，避免原移植版本的整数秒截断。 */
#define TICKS_TO_SEC(t) ((float)(t) / (float)manager.tick_frequency_hz)

const uint8_t HeroChassisPowerLimit_HP_FIRST[CHASSIS_POWER_LEVEL_COUNT] = CHASSIS_POWER_HERO_LIMITS_W;
const uint8_t InfantryChassisPowerLimit_HP_FIRST[CHASSIS_POWER_LEVEL_COUNT] = CHASSIS_POWER_INFANTRY_LIMITS_W;

static Manager manager;
static PowerStatus powerStatus;
static uint8_t  LATEST_FEEDBACK_JUDGE_ROBOT_LEVEL;
static uint8_t  isCapEnergyOut                = 0;
static uint32_t motorDisconnectCounter[4]     = {0U, 0U, 0U, 0U};
static float    MIN_MAXPOWER_CONFIGURED       = CHASSIS_POWER_INITIAL_MIN_W;
static uint8_t  isInitialized                 = 0;
static PowerFeedbackSources sources;

static pid_position_t powerPD_base;  /* ensure the system does not die */
static pid_position_t powerPD_full;  /* ensure the capacitor's energy keeps under 90% */

/* ---------------- helpers ---------------- */
static inline uint8_t floatEqual(float a, float b) { return fabsf(a - b) < 1e-5f; }
static inline void    setErrorFlag(uint8_t *curFlag, enum ErrorFlags setFlag) { *curFlag |= (uint8_t)setFlag; }
static inline void    clearErrorFlag(uint8_t *curFlag, enum ErrorFlags clearFlag) { *curFlag &= (uint8_t)(~(uint8_t)clearFlag); }
static inline uint8_t isFlagged(uint8_t curFlag, enum ErrorFlags flag) { return (curFlag & (uint8_t)flag) != 0; }
static inline uint8_t refereeIsConnected(void) { return sources.refereeIsConnected ? sources.refereeIsConnected() : 0; }


static void power_pd_reset(void)
{
    pid_position_init(&powerPD_base, CHASSIS_POWER_ENERGY_KP, 0.0f, CHASSIS_POWER_ENERGY_KD, 0.0f, manager.sample_period_s);
    pid_position_init(&powerPD_full, CHASSIS_POWER_ENERGY_KP, 0.0f, CHASSIS_POWER_ENERGY_KD, 0.0f, manager.sample_period_s);
}

/* 工程 PID 参数顺序为 (feedback, reference), 输出限幅 ±CHASSIS_POWER_CAP_MAX_OUTPUT_W */
static float power_pd_run(pid_position_t *pid, float set, float feedback)
{
    return float_constrain(pid_position_calculate(pid, feedback, set), -CHASSIS_POWER_CAP_MAX_OUTPUT_W, CHASSIS_POWER_CAP_MAX_OUTPUT_W);
}

static uint8_t isAllMotorConnected(void)
{
    for (uint32_t index = 0U; index < CHASSIS_MOTOR_COUNT; index++)
    {
        if (!chassis_motor_is_online(&chassis_motors[index])){return 0U;}
    }
    return 1U;
}

/* ---------------- 小接口 ---------------- */
void Power_setMaxPowerConfigured(float maxPower)
{
    manager.userConfiguredMaxPower = float_constrain(maxPower, MIN_MAXPOWER_CONFIGURED, manager.powerUpperLimit);
}

void Power_setMode(uint8_t mode) { Power_setMaxPowerConfigured(mode == 1 ? manager.powerUpperLimit : manager.refereeMaxPower); }

void Power_registerPowerCallbackFunc(float (*callback)(void)) { manager.callback = callback; }

void Power_setRLSEnabled(uint8_t enable) { manager.rlsEnabled = (RLSEnabled)enable; }

const volatile PowerStatus *Power_getPowerStatus(void) { return (const volatile PowerStatus *)&powerStatus; }

float Power_getLatestFeedbackJudgePowerLimit(void) { return manager.refereeMaxPower; }

/* ---------------- getControlledOutput ---------------- */
float *Power_getControlledOutput(PowerObj *objs[4])
{
    const float k0 = manager.torqueConst * CHASSIS_MOTOR_CURRENT_LIMIT_A /
                     CHASSIS_MOTOR_OUTPUT_LIMIT;  /* torque current rate of the motor, defined as Nm/Output */

    static float newTorqueCurrent[4];

    float sumCmdPower = 0.0f;
    float cmdPower[4];

    float sumError = 0.0f;
    float error[4];

    float maxPower = float_constrain(manager.userConfiguredMaxPower, manager.fullMaxPower, manager.baseMaxPower);

    float allocatablePower = maxPower;
    float sumPowerRequired = 0.0f;
    int i;

    for (i = 0; i < 4; i++) {
        const chassis_motor_t *motor = &chassis_motors[i];
        if (chassis_motor_is_online(motor)) {
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
        } else if (motorDisconnectCounter[i] < manager.motor_hold_ticks) {
            float tq = motor->torque_nm;
            float av = motor->speed_rad_s;
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
        if (sumError > CHASSIS_POWER_ERROR_DISTRIBUTION_RAD_S) {
            errorConfidence = 1.0f;
        } else if (sumError > CHASSIS_POWER_PROPORTIONAL_DISTRIBUTION_RAD_S) {
            errorConfidence = float_constrain((sumError - CHASSIS_POWER_PROPORTIONAL_DISTRIBUTION_RAD_S) / (CHASSIS_POWER_ERROR_DISTRIBUTION_RAD_S - CHASSIS_POWER_PROPORTIONAL_DISTRIBUTION_RAD_S), 0.0f, 1.0f);
        } else {
            errorConfidence = 0.0f;
        }
        for (i = 0; i < 4; i++) {
            PowerObj *p = objs[i];
            if (chassis_motor_is_online(&chassis_motors[i])) {
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
                newTorqueCurrent[i] = float_constrain(newTorqueCurrent[i], -p->pidMaxOutput, p->pidMaxOutput);
            } else {
                newTorqueCurrent[i] = 0.0f;
            }
        }
    } else {
        for (i = 0; i < 4; i++) {
            if (chassis_motor_is_online(&chassis_motors[i])) {
                newTorqueCurrent[i] = objs[i]->pidOutput;
            } else {
                newTorqueCurrent[i] = 0.0f;
            }
        }
    }


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
    manager.torqueConst = CHASSIS_MOTOR_KA_NM_PER_A * CHASSIS_MOTOR_REDUCTION_RATIO;
    isInitialized       = 1;
    /* vTaskDelay(1000); 由调用方在任务里自行延时 */
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
        if (!isFlagged(manager.error, RefereeDisConnect) && powerMessage->chassisPowerValid) {
            if (powerMessage->bufferEnergy < CHASSIS_POWER_REFEREE_FULL_ENERGY_J && powerMessage->chassisPower > CHASSIS_POWER_CAP_RUNOUT_THRESHOLD_W) {
                isCapEnergyOut             = 1;
                manager.estimatedCapEnergy = 0.0f;
            } else {
                isCapEnergyOut     = 0;
                manager.rlsEnabled = Disable;
                if (powerMessage->chassisPower < MIN_MAXPOWER_CONFIGURED && powerMessage->bufferEnergy == (uint16_t)CHASSIS_POWER_REFEREE_FULL_ENERGY_J) {
                    manager.estimatedCapEnergy = CHASSIS_POWER_CAP_CAPACITY_J;
                } else {
                    manager.estimatedCapEnergy += (powerMessage->chassisPower - manager.estimatedPower) * TICKS_TO_SEC(now - manager.lastUpdateTick);
                    manager.estimatedCapEnergy = float_constrain(manager.estimatedCapEnergy, 0.0f, CHASSIS_POWER_CAP_CAPACITY_J);
                }
            }
        } else {
            isCapEnergyOut     = 0;
            manager.rlsEnabled = Disable;
            manager.estimatedCapEnergy += (CHASSIS_POWER_CAP_OFFLINE_TARGET_W - manager.estimatedPower) * TICKS_TO_SEC(now - manager.lastUpdateTick);
            manager.estimatedCapEnergy = float_constrain(manager.estimatedCapEnergy, 0.0f, CHASSIS_POWER_CAP_CAPACITY_J);
        }
    } else {
        isCapEnergyOut             = 0;
        manager.estimatedCapEnergy = capStatus->capacitorRx.capEnergy / CHASSIS_POWER_CAP_ENERGY_RAW_MAX * CHASSIS_POWER_CAP_CAPACITY_J;
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
        manager.fullBuffSet = CHASSIS_POWER_CAP_FULL_ENERGY_RAW;
        manager.baseBuffSet = CHASSIS_POWER_CAP_BASE_ENERGY_RAW;
    } else {
        manager.fullBuffSet = CHASSIS_POWER_REFEREE_FULL_ENERGY_J;
        manager.baseBuffSet = CHASSIS_POWER_REFEREE_BASE_ENERGY_J;
    }
#else
    manager.fullBuffSet = CHASSIS_POWER_REFEREE_FULL_ENERGY_J;
    manager.baseBuffSet = CHASSIS_POWER_REFEREE_BASE_ENERGY_J;
#endif

    /* Update the referee maximum power limit and user configured power limit
     * If disconnected, then restore the last robot level and find corresponding chassis power limit */
    if (!isFlagged(manager.error, RefereeDisConnect)) {
        manager.refereeMaxPower = fmaxf((float)robotMessage->chassis_power_limit, CHASSIS_POWER_CAP_RUNOUT_THRESHOLD_W);
        if (robotMessage->robot_level > CHASSIS_POWER_LEVEL_COUNT)
            LATEST_FEEDBACK_JUDGE_ROBOT_LEVEL = 1U;
        else
            LATEST_FEEDBACK_JUDGE_ROBOT_LEVEL = (uint8_t)fmaxf(1.0f, (float)robotMessage->robot_level);
#if USE_SUPER_CAPACITOR
        if (isFlagged(manager.error, CAPDisConnect))
            manager.powerUpperLimit = manager.refereeMaxPower + CHASSIS_POWER_ENERGY_KP * (sqrtf(CHASSIS_POWER_REFEREE_FULL_ENERGY_J) - sqrtf(CHASSIS_POWER_REFEREE_BASE_ENERGY_J));
        else
            manager.powerUpperLimit = manager.refereeMaxPower + CHASSIS_POWER_CAP_MAX_OUTPUT_W;
#else
        manager.powerUpperLimit = manager.refereeMaxPower;
#endif
    } else {
        switch (manager.division) {
        case HERO:     manager.refereeMaxPower = HeroChassisPowerLimit_HP_FIRST[LATEST_FEEDBACK_JUDGE_ROBOT_LEVEL - 1U]; break;
        case INFANTRY: manager.refereeMaxPower = InfantryChassisPowerLimit_HP_FIRST[LATEST_FEEDBACK_JUDGE_ROBOT_LEVEL - 1U]; break;
        case SENTRY:   manager.refereeMaxPower = CHASSIS_POWER_SENTRY_LIMIT_W; break;
        default:       assert(0); break;
        }
        /* Since we have less available feedback, we constrain the power conservatively */
#if USE_SUPER_CAPACITOR
        if (isFlagged(manager.error, CAPDisConnect)) {
            manager.powerUpperLimit = manager.refereeMaxPower * CHASSIS_POWER_BOTH_OFFLINE_RATIO;
        } else {
            manager.powerUpperLimit = manager.refereeMaxPower + CHASSIS_POWER_CAP_MAX_OUTPUT_W;
        }
#else
        manager.powerUpperLimit = manager.refereeMaxPower * CHASSIS_POWER_BOTH_OFFLINE_RATIO;
#endif
    }

    MIN_MAXPOWER_CONFIGURED = manager.refereeMaxPower * CHASSIS_POWER_MIN_LIMIT_RATIO;

    /* energy loop
     * if cap and referee both gg, set the max power to latest power limit * 0.85 and disable energy loop
     * if referee gg, set the max power to latest power limit * 0.95, enable energy loop when cap energy out */
    if (isFlagged(manager.error, CAPDisConnect) && isFlagged(manager.error, RefereeDisConnect)) {
        manager.baseMaxPower = manager.fullMaxPower = manager.refereeMaxPower * CHASSIS_POWER_BOTH_OFFLINE_RATIO;
        power_pd_reset();
    } else {
        manager.baseMaxPower = fmaxf(manager.refereeMaxPower - power_pd_run(&powerPD_base, sqrtf(manager.baseBuffSet), sqrtf(manager.powerBuff)), MIN_MAXPOWER_CONFIGURED);
        manager.fullMaxPower = fmaxf(manager.refereeMaxPower - power_pd_run(&powerPD_full, sqrtf(manager.fullBuffSet), sqrtf(manager.powerBuff)), MIN_MAXPOWER_CONFIGURED);
    }

    /* if user has self defined power curve, use it */
    if (manager.callback != NULL)
        Power_setMaxPowerConfigured(manager.callback());

    /* Estimate the power based on the current model */
    effectivePower = 0;
    samples[0]     = 0;
    samples[1]     = 0;
    for (i = 0; i < 4; i++) {
        const chassis_motor_t *motor = &chassis_motors[i];
        if (chassis_motor_is_online(motor)) {
            motorDisconnectCounter[i] = 0U;
        } else {
            motorDisconnectCounter[i] += now - manager.lastUpdateTick;
        }
        if (motorDisconnectCounter[i] < manager.motor_hold_ticks) {
            /* 配置的离线记忆窗口内继续计入末次力矩/速度，避免低估尚未消失的耗能。 */
            float tq = motor->torque_nm;
            float av = motor->speed_rad_s;
            effectivePower += tq * av;
            samples[0] += fabsf(av);
            samples[1] += tq * tq;
        } else {
            motorDisconnectCounter[i] = manager.motor_hold_ticks;
        }
    }
    manager.estimatedPower = manager.k1 * samples[0] + manager.k2 * samples[1] + effectivePower + manager.k3;

    /* 无真实功率来源时只作模型估计，不把占位值或估计值送回 RLS。 */
    bool has_measured_power = false;
#if USE_SUPER_CAPACITOR
    if (!isFlagged(manager.error, CAPDisConnect)) {
        manager.measuredPower = capStatus->capacitorRx.chassisPower;
        has_measured_power = true;
    } else if (!isFlagged(manager.error, RefereeDisConnect) && powerMessage->chassisPowerValid && isCapEnergyOut) {
        manager.measuredPower = powerMessage->chassisPower;
        has_measured_power = true;
    } else {
        manager.measuredPower = manager.estimatedPower;
    }
#else
    if (!isFlagged(manager.error, RefereeDisConnect) && powerMessage->chassisPowerValid) {
        manager.measuredPower = powerMessage->chassisPower;
        has_measured_power = true;
    } else {
        manager.measuredPower = manager.estimatedPower;
    }
#endif

    /* update power status */
    powerStatus.userConfiguredMaxPower = manager.userConfiguredMaxPower;
    powerStatus.effectivePower         = effectivePower;
    powerStatus.powerLoss              = manager.measuredPower - effectivePower;
    powerStatus.efficiency             = manager.measuredPower > 0.0f ? float_constrain(effectivePower / manager.measuredPower, 0.0f, 1.0f) : 0.0f;
    powerStatus.estimatedCapEnergy     = (uint8_t)(manager.estimatedCapEnergy / CHASSIS_POWER_CAP_CAPACITY_J * CHASSIS_POWER_CAP_ENERGY_RAW_MAX);
    powerStatus.error                  = manager.error;

    /* Update the RLS parameters AND add dead zone AND
     * The Referee System could not detect negative power, so use estimated power to evaluate this situation */
    if (manager.rlsEnabled == Enable && has_measured_power && fabsf(manager.measuredPower) > CHASSIS_POWER_RLS_MIN_MEASURED_W &&
        !(isFlagged(manager.error, CAPDisConnect) && manager.estimatedPower < 0)) {
        params     = RLS_update(&manager.rls, samples, manager.measuredPower - effectivePower - manager.k3, now);
        manager.k1 = fmaxf(params[0], CHASSIS_POWER_RLS_MIN_COEFFICIENT);  /* 避免辨识系数变为负数。 */
        manager.k2 = fmaxf(params[1], CHASSIS_POWER_RLS_MIN_COEFFICIENT);  /* 避免辨识系数变为负数。 */
    }

    manager.lastUpdateTick = now;
    /* vTaskDelay(pdMS_TO_TICKS(1)); 由调用方任务负责 */
}

/* ---------------- 初始化 ---------------- */
void power_controller_init(uint32_t tick_frequency_hz, uint32_t period_ticks, const PowerFeedbackSources *src)
{
    if (isInitialized)
    {
        return;
    }

    manager.rlsEnabled = (RLSEnabled)CHASSIS_POWER_RLS_ENABLED;
    manager.error = 0U;
    manager.division = (Division)CHASSIS_POWER_DIVISION;
    manager.powerBuff = 0.0f;
    manager.fullBuffSet = 0.0f;
    manager.baseBuffSet = 0.0f;
    manager.fullMaxPower = 0.0f;
    manager.baseMaxPower = 0.0f;
    manager.refereeMaxPower = 0.0f;
    manager.userConfiguredMaxPower = 0.0f;
    manager.callback = NULL;
    manager.measuredPower = 0.0f;
    manager.estimatedPower = 0.0f;
    manager.estimatedCapEnergy = 0.0f;
    manager.torqueConst = 0.0f;
    manager.k1 = CHASSIS_POWER_K1;
    manager.k2 = CHASSIS_POWER_K2;
    manager.k3 = CHASSIS_POWER_K3_W;
    manager.lastUpdateTick = 0U;
    manager.tick_frequency_hz = tick_frequency_hz;
    manager.motor_hold_ticks = (tick_frequency_hz * CHASSIS_POWER_MOTOR_HOLD_MS + 999U) / 1000U;
    manager.sample_period_s = (float)period_ticks / (float)tick_frequency_hz;
    RLS_construct(&manager.rls, CHASSIS_POWER_RLS_DELTA, CHASSIS_POWER_RLS_LAMBDA);
    const float init_params[2] = {CHASSIS_POWER_K1, CHASSIS_POWER_K2};
    RLS_setParamVector(&manager.rls, init_params);
    sources = *src;

    /* 设置裁判等级与功率预算的初始值。 */
    LATEST_FEEDBACK_JUDGE_ROBOT_LEVEL = manager.division == SENTRY ? CHASSIS_POWER_LEVEL_COUNT : 1U;
    MIN_MAXPOWER_CONFIGURED           = CHASSIS_POWER_INITIAL_MIN_W;
    manager.powerUpperLimit           = CHASSIS_POWER_CAP_RUNOUT_THRESHOLD_W + CHASSIS_POWER_CAP_MAX_OUTPUT_W;

    /* 能量环参数与实际功率任务周期来自统一配置。 */
    power_pd_reset();

    /* subscribeMessage(&powerMessage/&robotMessage) -> 由 sources 指针代替
     *         xTaskCreateStatic(powerDaemon, "power", 1024, nullptr, 10, ...) -> 由调用方创建任务 */
}

#endif /* USE_POWER_CONTROLLER */
