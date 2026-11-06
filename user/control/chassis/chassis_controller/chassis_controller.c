/* 移植自 user/add/ChassisController.c；工程 PID、斜坡与轮侧反馈由下层适配。
 * 轮序固定为右前、左前、左后、右后；接线与安装方向只在 port 层处理。 */
#include "chassis_controller.h"

#include <math.h>

#include "chassis_port.h"
#include "power_controller.h"
#include "ramp.h"

#define SQRT2 1.41421356237f
#define CHASSIS_CLAMP(x, min, max) ((x < min) ? min : (x > max) ? max : x)
#define CHASSIS_DEG_TO_RAD (3.14159265f / 180.0f)

static float elapsed_seconds(ChassisController *ctl, uint32_t nowTick);
static void solveMeasuredDynamics(ChassisController *ctl, float timeDiff_s);

/* 旋转方向约定：按纵向(y)、横向(x)旋转 */
void chassis_rotate(const Dynamics *in, float angle_deg, Dynamics *out)
{
    const float angle_rad = angle_deg * CHASSIS_DEG_TO_RAD;
    const float cos_angle = cosf(angle_rad);
    const float sin_angle = sinf(angle_rad);

    out->y = in->y * cos_angle - in->x * sin_angle;
    out->x = in->y * sin_angle + in->x * cos_angle;
    out->w = in->w;
}

#if USE_MECANUM_CHASSIS
void ChassisPropertyConfig_construct(ChassisPropertyConfig *c, float wheelDiameters, float fbLength, float rlLength)
{
    c->chassisWheelDiameters = wheelDiameters;
    c->chassisRightLeftLength = rlLength;
    c->chassisFrontBackLength = fbLength;
}
#else
void ChassisPropertyConfig_construct(ChassisPropertyConfig *c, float wheelDiameters, float wheelToCentre)
{
    c->chassisWheelDiameters = wheelDiameters;
    c->chassisWheelToCentreLength = wheelToCentre;
}
#endif

void ChassisConfigs_construct(ChassisConfigs *cfg,
                              pid_position_t *pPIDRf, pid_position_t *pPIDLf, pid_position_t *pPIDLb, pid_position_t *pPIDRb,
                              const ChassisPropertyConfig *property,
                              float maxAcceleration_, float maxAngularAccelertion_,
                              const Dynamics *maxSpeed_, uint32_t nowTick, uint32_t tick_frequency_hz)
{
    cfg->chassisPropertyConfig = *property;
    cfg->chassisPIDs[0] = pPIDRf;
    cfg->chassisPIDs[1] = pPIDLf;
    cfg->chassisPIDs[2] = pPIDLb;
    cfg->chassisPIDs[3] = pPIDRb;
    cfg->lastUpdate = nowTick;
    cfg->tick_frequency_hz = tick_frequency_hz;
    cfg->maxAcceleration = maxAcceleration_;
    cfg->maxAngularAcceleration = maxAngularAccelertion_;
    cfg->maxSpeed = *maxSpeed_;
}

void ChassisController_construct(ChassisController *ctl, const ChassisConfigs *chassisConfig_)
{
    static const Dynamics zero = {0.0f, 0.0f, 0.0f};

    for (int i = 0; i < 4; i++)
    {
        ctl->wheelsTargetAv[i] = 0.0f;
        ctl->wheelsOutput[i] = 0.0f;
        ctl->currentAv[i] = 0.0f;
    }
    ctl->chassisConfigs = *chassisConfig_;
    ctl->rawTargetVelocity = ctl->lastRawTargetVelocity = ctl->targetVelocity = ctl->lastTargetVelocity = zero;
    ctl->measuredVelocity = ctl->lastMeasuredVelocity = ctl->measuredAcceleration = ctl->measuredDisplacement = zero;

    ctl->diameters = ctl->chassisConfigs.chassisPropertyConfig.chassisWheelDiameters;
#if USE_MECANUM_CHASSIS
    ctl->sum = ctl->chassisConfigs.chassisPropertyConfig.chassisFrontBackLength + ctl->chassisConfigs.chassisPropertyConfig.chassisRightLeftLength;
    ctl->wFactor = -ctl->diameters / (4.0f * ctl->sum);
    ctl->xyFactor = ctl->diameters / 8.0f;
#else
    ctl->length = ctl->chassisConfigs.chassisPropertyConfig.chassisWheelToCentreLength;
    ctl->wFactor = -ctl->diameters / (8.0f * ctl->length);
    ctl->xyFactor = ctl->diameters / (4.0f * SQRT2);
#endif
}

void ChassisController_settargetVelocity(ChassisController *ctl, float x, float y, float w)
{
    ctl->lastRawTargetVelocity = ctl->rawTargetVelocity;
    ctl->rawTargetVelocity.x = x;
    ctl->rawTargetVelocity.y = y;
    ctl->rawTargetVelocity.w = w;
    ctl->targetVelocity.x = CHASSIS_CLAMP(x, -ctl->chassisConfigs.maxSpeed.x, ctl->chassisConfigs.maxSpeed.x);
    ctl->targetVelocity.y = CHASSIS_CLAMP(y, -ctl->chassisConfigs.maxSpeed.y, ctl->chassisConfigs.maxSpeed.y);
    ctl->targetVelocity.w = CHASSIS_CLAMP(w, -ctl->chassisConfigs.maxSpeed.w, ctl->chassisConfigs.maxSpeed.w);
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

const Dynamics *ChassisController_getMeasuredAccleration(ChassisController *ctl)
{
    return &ctl->measuredAcceleration;
}

const Dynamics *ChassisController_getMeasuredVelocity(ChassisController *ctl)
{
    return &ctl->measuredVelocity;
}

const Dynamics *ChassisController_getMeasuredDisplacement(ChassisController *ctl)
{
    return &ctl->measuredDisplacement;
}

void ChassisController_setMaxVelocity(ChassisController *ctl, const Dynamics *newMaxSpeeds)
{
    ctl->chassisConfigs.maxSpeed = *newMaxSpeeds;
}

void ChassisController_setMaxAngularAcceleration(ChassisController *ctl, float maxAngularAcceleration)
{
    ctl->chassisConfigs.maxAngularAcceleration = maxAngularAcceleration;
}

void ChassisController_setMaxAcceleration(ChassisController *ctl, float maxAcceleration)
{
    ctl->chassisConfigs.maxAcceleration = maxAcceleration;
}

void ChassisController_update(ChassisController *ctl, uint32_t nowTick)
{
    const float timeDiff_s = elapsed_seconds(ctl, nowTick);
    const ChassisConfigs *cfg = &ctl->chassisConfigs;
    Dynamics *tv = &ctl->targetVelocity;

    solveMeasuredDynamics(ctl, timeDiff_s);

    /* 目标速度加速度限幅，承接参考实现未启用的 maxAccClamp() */
    tv->x = ramp_limit(tv->x, ctl->lastTargetVelocity.x, cfg->maxAcceleration, timeDiff_s);
    tv->y = ramp_limit(tv->y, ctl->lastTargetVelocity.y, cfg->maxAcceleration, timeDiff_s);
    tv->w = ramp_limit(tv->w, ctl->lastTargetVelocity.w, cfg->maxAngularAcceleration, timeDiff_s);
    ctl->lastTargetVelocity = *tv;

    /* 轮侧目标 rad/s：X 向右、Y 向前、w 逆时针为正。 */
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

    /* 轮速 PID 与功率分配使用相同的轮侧单位。 */
#if USE_POWER_CONTROLLER
    {
        /* 功率模块为单实例，PowerObj 静态复用 */
        static PowerObj objs[4];
        static PowerObj *pObjs[4] = {&objs[0], &objs[1], &objs[2], &objs[3]};
        const float *controlledOutput;

        for (int i = 0; i < 4; i++)
        {
            objs[i].curAv = ctl->currentAv[i];
            objs[i].setAv = ctl->wheelsTargetAv[i];
            /* 工程 PID 不限幅输出，按电机指令上限限幅后交给功率环 */
            objs[i].pidOutput = float_constrain(pid_position_calculate(cfg->chassisPIDs[i], ctl->currentAv[i], ctl->wheelsTargetAv[i]), -CHASSIS_MOTOR_OUTPUT_LIMIT, CHASSIS_MOTOR_OUTPUT_LIMIT);
            objs[i].pidMaxOutput = CHASSIS_MOTOR_OUTPUT_LIMIT;
        }
        controlledOutput = Power_getControlledOutput(pObjs);
        for (int i = 0; i < 4; i++)
        {
            ctl->wheelsOutput[i] = controlledOutput[i];
        }
    }
#else
    for (int i = 0; i < 4; i++)
    {
        ctl->wheelsOutput[i] = float_constrain(pid_position_calculate(cfg->chassisPIDs[i], ctl->currentAv[i], ctl->wheelsTargetAv[i]), -CHASSIS_MOTOR_OUTPUT_LIMIT, CHASSIS_MOTOR_OUTPUT_LIMIT);
    }
#endif

    for (int i = 0; i < 4; i++)
    {
        if (!chassis_motor_is_online(&chassis_motors[i]))
        {
            pid_position_t *pid = cfg->chassisPIDs[i];
            pid_position_init(pid, pid->kp, pid->ki, pid->kd, pid->integral_limit, pid->sample_period_s);
            ctl->wheelsOutput[i] = 0.0f;
        }
        chassis_motors[i].output = ctl->wheelsOutput[i];
    }
}

void ChassisController_stop(ChassisController *ctl, uint32_t nowTick)
{
    solveMeasuredDynamics(ctl, elapsed_seconds(ctl, nowTick));
    for (int i = 0; i < 4; i++)
    {
        pid_position_t *pid = ctl->chassisConfigs.chassisPIDs[i];

        /* 工程 PID 无复位接口，用原参数重新初始化清零积分与上次误差 */
        pid_position_init(pid, pid->kp, pid->ki, pid->kd, pid->integral_limit, pid->sample_period_s);
        ctl->wheelsTargetAv[i] = 0.0f;
        ctl->wheelsOutput[i] = 0.0f;
        chassis_motors[i].output = 0.0f;
    }
    ctl->rawTargetVelocity = (Dynamics){0.0f, 0.0f, 0.0f};
    ctl->lastRawTargetVelocity = ctl->rawTargetVelocity;
    ctl->targetVelocity = ctl->rawTargetVelocity;
    ctl->lastTargetVelocity = ctl->rawTargetVelocity;
}

const ChassisConfigs *ChassisController_getConfig(ChassisController *ctl)
{
    return &ctl->chassisConfigs;
}

static float elapsed_seconds(ChassisController *ctl, uint32_t nowTick)
{
    const float timeDiff_s = (float)(nowTick - ctl->chassisConfigs.lastUpdate) / (float)ctl->chassisConfigs.tick_frequency_hz;

    ctl->chassisConfigs.lastUpdate = nowTick;
    return timeDiff_s;
}

static void solveMeasuredDynamics(ChassisController *ctl, float timeDiff_s)
{
    ctl->lastMeasuredVelocity = ctl->measuredVelocity;
    for (int i = 0; i < 4; i++)
    {
        ctl->currentAv[i] = chassis_motors[i].speed_rad_s;
    }
    ctl->measuredVelocity.x = (ctl->currentAv[2] + ctl->currentAv[3] - ctl->currentAv[1] - ctl->currentAv[0]) * ctl->xyFactor;
    ctl->measuredVelocity.y = (ctl->currentAv[1] + ctl->currentAv[2] - ctl->currentAv[0] - ctl->currentAv[3]) * ctl->xyFactor;
    ctl->measuredVelocity.w = (ctl->currentAv[0] + ctl->currentAv[1] + ctl->currentAv[2] + ctl->currentAv[3]) * ctl->wFactor;

    /* 位移在底盘机体坐标系积分，不作为世界坐标里程计。 */
    ctl->measuredDisplacement.w += ctl->measuredVelocity.w * timeDiff_s;
    ctl->measuredDisplacement.y += ctl->measuredVelocity.y * timeDiff_s;
    ctl->measuredDisplacement.x += ctl->measuredVelocity.x * timeDiff_s;

    /* 初始化后的首次调用或同一 tick 内停止时，时间差可能为零。 */
    if (timeDiff_s > 0.0f)
    {
        ctl->measuredAcceleration.w = (ctl->measuredVelocity.w - ctl->lastMeasuredVelocity.w) / timeDiff_s;
        ctl->measuredAcceleration.y = (ctl->measuredVelocity.y - ctl->lastMeasuredVelocity.y) / timeDiff_s;
        ctl->measuredAcceleration.x = (ctl->measuredVelocity.x - ctl->lastMeasuredVelocity.x) / timeDiff_s;
    }
    else
    {
        ctl->measuredAcceleration = (Dynamics){0.0f, 0.0f, 0.0f};
    }
}
