/* 移植底盘控制器：运动学、加速度约束、轮速 PID 与功率分配。
 * 只依赖归一化的 chassis_port 接口，不处理 CAN 或任务输入。 */
#ifndef SENTRY_CHASSIS_CHASSISCONTROLLER_H
#define SENTRY_CHASSIS_CHASSISCONTROLLER_H

#include <stdint.h>

#include "pid.h"
#include "chassis_config.h"

/* 底盘机体速度：x 横向(右正)、y 纵向(前正)，单位 m/s；w 自转，逆时针为正，单位 rad/s */
typedef struct {
    float x;
    float y;
    float w;
} Dynamics;

/* 把云台参考系速度旋转到底盘参考系，angle_deg 为云台相对底盘角，与轮系无关。
 * in 与 out 不能是同一对象 */
void chassis_rotate(const Dynamics *in, float angle_deg, Dynamics *out);

/* 轮径、前后/左右轮距及全向轮自转力臂，单位均为 m。 */
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

/* 底盘控制配置。 pids 顺序: 0 Rf, 1 Lf, 2 Lb, 3 Rb，与 chassis_motors[] 一致
 * maxAcceleration / maxAngularAcceleration 为目标速度的变化率限幅，单位 m/s^2、rad/s^2 */
typedef struct {
    ChassisPropertyConfig  chassisPropertyConfig;
    pid_position_t        *chassisPIDs[4];
    uint32_t lastUpdate;
    uint32_t tick_frequency_hz;
    float    maxAcceleration;
    float    maxAngularAcceleration;
    Dynamics maxSpeed;
} ChassisConfigs;

void ChassisConfigs_construct(ChassisConfigs *cfg,
                              pid_position_t *pPIDRf, pid_position_t *pPIDLf, pid_position_t *pPIDLb, pid_position_t *pPIDRb,
                              const ChassisPropertyConfig *property,
                              float maxAcceleration_, float maxAngularAccelertion_,
                              const Dynamics *maxSpeed_, uint32_t nowTick, uint32_t tick_frequency_hz);

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
/* nowTick 与初始化传入的 tick 频率一致，每个控制周期调用一次：
 * 解算实测速度 -> 目标速度加速度限幅 -> 逆解 -> 轮速 PID -> 功率限制 -> chassis_motors[].output */
void ChassisController_update(ChassisController *ctl, uint32_t nowTick);
/* 复位轮速 PID 并输出零 */
void ChassisController_stop(ChassisController *ctl, uint32_t nowTick);
const ChassisConfigs *ChassisController_getConfig(ChassisController *ctl);

#endif //SENTRY_CHASSIS_CHASSISCONTROLLER_H
