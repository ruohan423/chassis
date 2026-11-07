#include "chassis.h"

#include "chassis_controller.h"
#include "chassis_port.h"
#include "power_controller.h"

volatile chassis_mode_t actChassis = CHASSIS_NORMAL;

static pid_position_t speed_pid[CHASSIS_MOTOR_COUNT];
static ChassisController controller;
static pid_position_t follow_pid;
#if USE_POWER_CONTROLLER
/* 功率任务完成首轮计算前，底盘保持零输出，避免依赖任务创建和运行顺序。 */
static volatile bool power_ready;
#endif

static bool update_target(float input_x, float input_y, float gimbal_angle_deg, bool gimbal_valid, Dynamics *target);

bool chassis_init(uint32_t now_tick, uint32_t tick_frequency_hz, uint32_t period_ticks)
{
    if (!chassis_port_init())
    {
        return false;
    }

    const float sample_period_s = (float)period_ticks / (float)tick_frequency_hz;
    for (uint32_t index = 0U; index < CHASSIS_MOTOR_COUNT; index++)
    {
        pid_position_init(&speed_pid[index], CHASSIS_SPEED_KP, CHASSIS_SPEED_KI, CHASSIS_SPEED_KD,
                          CHASSIS_SPEED_INTEGRAL_LIMIT, sample_period_s);
        chassis_motors[index].output = 0.0f;
    }
    pid_position_init(&follow_pid, CHASSIS_FOLLOW_KP, CHASSIS_FOLLOW_KI, CHASSIS_FOLLOW_KD,
                      CHASSIS_FOLLOW_INTEGRAL_LIMIT, sample_period_s);

    ChassisPropertyConfig property;
    ChassisConfigs configs;
    const Dynamics max_speed = {CHASSIS_MAX_SPEED_X_MPS, CHASSIS_MAX_SPEED_Y_MPS, CHASSIS_MAX_SPIN_RAD_S};
#if USE_MECANUM_CHASSIS
    ChassisPropertyConfig_construct(&property, CHASSIS_WHEEL_DIAMETER_M,
                                    CHASSIS_FRONT_BACK_LENGTH_M, CHASSIS_RIGHT_LEFT_LENGTH_M);
#else
    ChassisPropertyConfig_construct(&property, CHASSIS_WHEEL_DIAMETER_M, CHASSIS_WHEEL_TO_CENTRE_M);
#endif
    ChassisConfigs_construct(&configs, &speed_pid[0], &speed_pid[1], &speed_pid[2], &speed_pid[3], &property,
                             CHASSIS_MAX_ACC_MPS2, CHASSIS_MAX_SPIN_ACC_RAD_S2, &max_speed, now_tick, tick_frequency_hz);
    ChassisController_construct(&controller, &configs);
    return true;
}

bool chassis_update(float input_x, float input_y, float gimbal_angle_deg, bool gimbal_valid, uint32_t now_tick)
{
#if USE_POWER_CONTROLLER
    if (!power_ready)
    {
        ChassisController_stop(&controller, now_tick);
        return chassis_port_send();
    }
#endif
    Dynamics target;
    if (update_target(input_x, input_y, gimbal_angle_deg, gimbal_valid, &target))
    {
        ChassisController_settargetVelocity(&controller, target.x, target.y, target.w);
        ChassisController_update(&controller, now_tick);
    }
    else
    {
        ChassisController_stop(&controller, now_tick);
    }

    if (!chassis_port_send())
    {
        ChassisController_stop(&controller, now_tick);
        return false;
    }
    return true;
}

void chassis_power_init(uint32_t now_tick, uint32_t tick_frequency_hz, uint32_t period_ticks)
{
#if USE_POWER_CONTROLLER
    power_ready = false;
    power_controller_init(tick_frequency_hz, period_ticks, &chassis_power_sources);
    Power_setMode(CHASSIS_POWER_LIMIT_MODE);
    Power_daemonInit(now_tick);
#else
    (void)now_tick;
    (void)tick_frequency_hz;
    (void)period_ticks;
#endif
}

void chassis_power_start(uint32_t now_tick)
{
#if USE_POWER_CONTROLLER
    Power_daemonInit(now_tick);
#else
    (void)now_tick;
#endif
}

void chassis_power_update(uint32_t now_tick)
{
#if USE_POWER_CONTROLLER
    chassis_port_update_feedback(now_tick);
    Power_daemonStep(now_tick);
    power_ready = true;
#else
    (void)now_tick;
#endif
}

static bool update_target(float input_x, float input_y, float gimbal_angle_deg, bool gimbal_valid, Dynamics *target)
{
    float frame_angle_deg = gimbal_valid ? gimbal_angle_deg : 0.0f;
    Dynamics frame_command = {input_x, input_y, 0.0f};
    bool active = true;

    switch (actChassis)
    {
        case CHASSIS_FOLLOW_GIMBAL:
            if (!gimbal_valid)
            {
                active = false;
                break;
            }
            frame_angle_deg = 0.0f;
            frame_command.w = float_constrain(-pid_position_calculate(&follow_pid, gimbal_angle_deg, 0.0f),
                                              -CHASSIS_FOLLOW_VW_LIMIT, CHASSIS_FOLLOW_VW_LIMIT);
            break;
        case CHASSIS_GYROSCOPE:
            frame_command.w = CHASSIS_GYRO_SPIN_RAD_S;
            break;
        case CHASSIS_NORMAL:
            break;
        case CHASSIS_STOP:
        default:
            active = false;
            break;
    }

    if (!active)
    {
        pid_position_init(&follow_pid, CHASSIS_FOLLOW_KP, CHASSIS_FOLLOW_KI, CHASSIS_FOLLOW_KD,
                          CHASSIS_FOLLOW_INTEGRAL_LIMIT, follow_pid.sample_period_s);
        *target = (Dynamics){0.0f, 0.0f, 0.0f};
        return false;
    }

    chassis_rotate(&frame_command, frame_angle_deg, target);
    return true;
}
