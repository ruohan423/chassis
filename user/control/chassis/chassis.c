/* 轮序固定为左前、左后、右后、右前；接线与安装方向只在 port 层处理。
 * 控制链路移植自 RMCS-main 全向轮底盘：行为模式与速度目标取自 ChassisController，
 * 功率预算取自 ChassisPowerController，力矩分配取自 OmniWheelController。 */
#include "chassis.h"

#include <math.h>

#include "chassis_port.h"
#include "pid.h"
#include "qcp_solver.h"

#define TWO_PI 6.28318530718f
#define SQRT2  1.41421356237f
/* 轮速与底盘速度、加速度与轮侧力矩之间的公共换算系数，单位 m。 */
#define VELOCITY_SCALE (-SQRT2 / 4.0f * CHASSIS_WHEEL_RADIUS_M)
/* 自转力臂：45° 对称布局下为两个方向轮距之和，单位 m。 */
#define SPIN_LEVER     (CHASSIS_RADIUS_X_M + CHASSIS_RADIUS_Y_M)
/* 不受功率上限约束的模式使用的功率上限，单位 W。RMCS 用无穷大表示不限功率，
 * 这里取足够大的有限值，使功率约束不起作用且不把无穷大带入力矩优化。 */
#define POWER_UNLIMITED_W 1.0e6f

volatile chassis_mode_t actChassis = CHASSIS_FREE;

static pid_calculator_t translation_pid[2];
static pid_calculator_t angular_pid;
static pid_calculator_t wheel_pid[CHASSIS_MOTOR_COUNT];
static pid_calculator_t following_pid;
static float control_period_s;
static float virtual_buffer_energy_j;
static float expected_power_limit_w;
static chassis_mode_t last_mode;
static bool spinning_forward = true;
static uint32_t spin_stuck_count;
static uint32_t spin_reverse_cooldown;
static uint32_t spin_stuck_confirm_cycles;
static uint32_t spin_reverse_cooldown_cycles;

static void reset_control(void);
static void update_spin_stuck_watchdog(chassis_behavior_t behavior, const chassis_feedback_t *feedback);
static void update_target_velocity(chassis_behavior_t behavior, const float input[3], const chassis_command_t *command,
                                   const chassis_feedback_t *feedback, float target[3]);
static float update_power_limit(bool power_unlimited, bool boost_request, const chassis_feedback_t *feedback);

bool chassis_init(float sample_period_s)
{
    control_period_s = sample_period_s;
    spin_stuck_confirm_cycles = (uint32_t)(CHASSIS_SPIN_STUCK_CONFIRM_S / sample_period_s + 0.5f);
    spin_reverse_cooldown_cycles = (uint32_t)(CHASSIS_SPIN_REVERSE_COOLDOWN_S / sample_period_s + 0.5f);

    pid_calculator_init(&translation_pid[0], CHASSIS_TRANSLATION_KP, CHASSIS_TRANSLATION_KI, CHASSIS_TRANSLATION_KD);
    pid_calculator_init(&translation_pid[1], CHASSIS_TRANSLATION_KP, CHASSIS_TRANSLATION_KI, CHASSIS_TRANSLATION_KD);
    pid_calculator_init(&angular_pid, CHASSIS_ANGULAR_KP, CHASSIS_ANGULAR_KI, CHASSIS_ANGULAR_KD);
    for (uint32_t index = 0U; index < CHASSIS_MOTOR_COUNT; index++)
    {
        pid_calculator_init(&wheel_pid[index], CHASSIS_WHEEL_KP, CHASSIS_WHEEL_KI, CHASSIS_WHEEL_KD);
    }
    pid_calculator_init(&following_pid, CHASSIS_FOLLOW_KP, CHASSIS_FOLLOW_KI, CHASSIS_FOLLOW_KD);
    following_pid.output_min = -CHASSIS_MAX_ANGULAR_RAD_S;
    following_pid.output_max = CHASSIS_MAX_ANGULAR_RAD_S;

    reset_control();
    return chassis_port_init();
}

bool chassis_update(const chassis_command_t *command, uint32_t now_tick)
{
    const chassis_mode_t mode = actChassis;
    float w[CHASSIS_MOTOR_COUNT];
    bool motors_online = true;

    for (uint32_t index = 0U; index < CHASSIS_MOTOR_COUNT; index++)
    {
        w[index] = chassis_motors[index].speed_rad_s;
        motors_online = motors_online && chassis_motor_is_online(&chassis_motors[index]);
    }

    chassis_feedback_t feedback;
    chassis_port_read_feedback(now_tick, &feedback);

    /* 电容充电功率上限只由裁判功率上限和缓冲能量决定，与底盘是否使能无关，每周期更新：
     * 缓冲能量低于基准线时按比例压低裁判上限，高于基准线后逐步放开额外充电功率。裁判离线时不充电。 */
    float charge_power_limit_w = 0.0f;
    if (feedback.referee_online)
    {
        charge_power_limit_w = feedback.power_limit_w
            * float_constrain((feedback.buffer_energy_j - CHASSIS_SUPERCAP_CHARGE_BUFFER_DEAD_J)
                              / (CHASSIS_SUPERCAP_CHARGE_BUFFER_BASE_J - CHASSIS_SUPERCAP_CHARGE_BUFFER_DEAD_J), 0.0f, 1.0f)
            + CHASSIS_SUPERCAP_CHARGE_EXCESS_W
            * float_constrain((feedback.buffer_energy_j - CHASSIS_SUPERCAP_CHARGE_BUFFER_BASE_J)
                              / (CHASSIS_SUPERCAP_CHARGE_BUFFER_CONTROL_J - CHASSIS_SUPERCAP_CHARGE_BUFFER_BASE_J), 0.0f, 1.0f);
    }

    /* 遥控器切入快速小陀螺时换向；全自动下上位机指定的小陀螺不改变方向。 */
    if (mode == CHASSIS_SPIN_FAST && last_mode != CHASSIS_SPIN_FAST)
    {
        spinning_forward = !spinning_forward;
    }
    last_mode = mode;

    /* 底盘速度由四轮共同解算，任一轮无反馈时估计失效，四轮一起停止。 */
    if (mode == CHASSIS_STOP || !motors_online)
    {
        reset_control();
        return chassis_port_send(charge_power_limit_w);
    }

    /* 行为和输入来源由模式唯一决定，两种来源不叠加：遥控器模式各固定一种行为，只取摇杆和键盘；
     * 全自动只取上位机指令及其指定的子行为，链路离线时为直控零速度。
     * input 依次为 x、y 向平移速度（m/s）和直控自转角速度（rad/s）。 */
    chassis_behavior_t behavior = CHASSIS_BEHAVIOR_DIRECT;
    float input[3] = {0.0f, 0.0f, 0.0f};
    bool boost_request = false;
    if (mode != CHASSIS_AUTO)
    {
        if (mode == CHASSIS_GIMBAL_FOLLOW_POWERED)
        {
            behavior = CHASSIS_BEHAVIOR_GIMBAL_FOLLOW;
        }
        else if (mode == CHASSIS_SPIN_FAST)
        {
            behavior = CHASSIS_BEHAVIOR_SPIN_FAST;
        }
        input[0] = (command->joystick_x + command->keyboard_x) * CHASSIS_MAX_TRANSLATION_MPS;
        input[1] = (command->joystick_y + command->keyboard_y) * CHASSIS_MAX_TRANSLATION_MPS;
        input[2] = command->joystick_w * CHASSIS_MAX_ANGULAR_RAD_S;
        boost_request = command->boost_request;
    }
    else
    {
        if (command->navigation_enabled)
        {
            behavior = command->navigation_behavior;
            input[0] = command->navigation_vx;
            input[1] = command->navigation_vy;
            input[2] = command->navigation_wz;
            boost_request = command->navigation_supercap;
        }
        /* 爬坡输入有效时进入爬坡；上位机指定爬坡而输入无效时退回直控。 */
        if (isfinite(command->climb_direction) && isfinite(command->climb_speed) && isfinite(command->climb_measure_yaw))
        {
            behavior = CHASSIS_BEHAVIOR_CLIMB;
        }
        else if (behavior == CHASSIS_BEHAVIOR_CLIMB)
        {
            behavior = CHASSIS_BEHAVIOR_DIRECT;
        }
    }

    update_spin_stuck_watchdog(behavior, &feedback);

    float target[3];
    update_target_velocity(behavior, input, command, &feedback, target);
    if (!isfinite(target[0]) || !isfinite(target[1]) || !isfinite(target[2]))
    {
        /* 目标无效时与停止态同样处理，对应 RMCS 对 NaN 速度目标的复位。 */
        reset_control();
        return chassis_port_send(charge_power_limit_w);
    }
    const float power_limit_w = update_power_limit(mode == CHASSIS_GIMBAL_FOLLOW_POWERED, boost_request, &feedback);

    /* 由轮速解算底盘实际速度。 */
    const float vx = (-w[0] - w[1] + w[2] + w[3]) * VELOCITY_SCALE;
    const float vy = (w[0] - w[1] - w[2] + w[3]) * VELOCITY_SCALE;
    const float wz = (w[0] + w[1] + w[2] + w[3]) / SPIN_LEVER * VELOCITY_SCALE;

    /* 平移、自转速度环，输出折算为轮侧力矩；x 分量为平移合力矩，y 分量为自转力矩。 */
    const float translation_x = VELOCITY_SCALE * CHASSIS_MASS_KG * pid_calculator_update(&translation_pid[0], target[0] - vx);
    const float translation_y = VELOCITY_SCALE * CHASSIS_MASS_KG * pid_calculator_update(&translation_pid[1], target[1] - vy);
    const float translation_torque = hypotf(translation_x, translation_y);
    const float angular_torque = VELOCITY_SCALE * (CHASSIS_INERTIA_KGM2 / SPIN_LEVER)
        * pid_calculator_update(&angular_pid, target[2] - wz);
    const float direction_x = translation_torque > 0.0f ? translation_x / translation_torque : 1.0f;
    const float direction_y = translation_torque > 0.0f ? translation_y / translation_torque : 0.0f;
    const float lambda_1 = -direction_x + direction_y;
    const float lambda_2 = -direction_x - direction_y;

    /* 轮速环：目标为实测底盘速度反解的轮速，只修正各轮相对整车运动的偏差。 */
    const float wheel_reference[CHASSIS_MOTOR_COUNT] = {
        -vx + vy + SPIN_LEVER * wz,
        -vx - vy + SPIN_LEVER * wz,
        +vx - vy + SPIN_LEVER * wz,
        +vx + vy + SPIN_LEVER * wz,
    };
    float t[CHASSIS_MOTOR_COUNT];
    float torque_sum = 0.0f;
    float speed_sum = 0.0f;
    float torque_square_sum = 0.0f;
    float speed_square_sum = 0.0f;
    float mechanical_power = 0.0f;
    for (uint32_t index = 0U; index < CHASSIS_MOTOR_COUNT; index++)
    {
        t[index] = pid_calculator_update(&wheel_pid[index],
                                         wheel_reference[index] * (-1.0f / (SQRT2 * CHASSIS_WHEEL_RADIUS_M)) - w[index]);
        torque_sum += t[index];
        speed_sum += w[index];
        torque_square_sum += t[index] * t[index];
        speed_square_sum += w[index] * w[index];
        mechanical_power += t[index] * w[index];
    }

    /* 在摩擦菱形和功率二次约束内，最大化平移与自转力矩。 */
    const float angular_sign = angular_torque > 0.0f ? 1.0f : -1.0f;
    const float rhombus_top = CHASSIS_FRICTION_COEFFICIENT * CHASSIS_MASS_KG * CHASSIS_GRAVITY_MPS2 * CHASSIS_WHEEL_RADIUS_M / 4.0f;
    const float quadratic[6] = {
        4.0f * CHASSIS_POWER_K1,
        0.0f,
        4.0f * CHASSIS_POWER_K1,
        lambda_1 * (w[0] - w[2] + 2.0f * CHASSIS_POWER_K1 * (t[0] - t[2]))
            + lambda_2 * (w[1] - w[3] + 2.0f * CHASSIS_POWER_K1 * (t[1] - t[3])),
        angular_sign * (2.0f * CHASSIS_POWER_K1 * torque_sum + speed_sum),
        CHASSIS_POWER_K1 * torque_square_sum + mechanical_power + CHASSIS_POWER_K2 * speed_square_sum
            - CHASSIS_POWER_NO_LOAD_W - power_limit_w,
    };
    const qcp_point_t constrained = qcp_solve((qcp_point_t){1.0f, 1.0f}, translation_torque, fabsf(angular_torque),
                                              rhombus_top / fmaxf(fabsf(lambda_1), fabsf(lambda_2)), rhombus_top, quadratic);
    const float constrained_angular = constrained.y * angular_sign;

    chassis_motors[0].output = (+lambda_1 * constrained.x + constrained_angular + t[0]) * CHASSIS_MOTOR_RAW_PER_NM;
    chassis_motors[1].output = (+lambda_2 * constrained.x + constrained_angular + t[1]) * CHASSIS_MOTOR_RAW_PER_NM;
    chassis_motors[2].output = (-lambda_1 * constrained.x + constrained_angular + t[2]) * CHASSIS_MOTOR_RAW_PER_NM;
    chassis_motors[3].output = (-lambda_2 * constrained.x + constrained_angular + t[3]) * CHASSIS_MOTOR_RAW_PER_NM;

    if (!chassis_port_send(charge_power_limit_w))
    {
        reset_control();
        return false;
    }
    return true;
}

/* 停止态：清空环路状态、卡死检测和输出，虚拟缓冲能量恢复为上限；小陀螺方向保持。 */
static void reset_control(void)
{
    pid_calculator_reset(&translation_pid[0]);
    pid_calculator_reset(&translation_pid[1]);
    pid_calculator_reset(&angular_pid);
    pid_calculator_reset(&following_pid);
    for (uint32_t index = 0U; index < CHASSIS_MOTOR_COUNT; index++)
    {
        pid_calculator_reset(&wheel_pid[index]);
        chassis_motors[index].output = 0.0f;
    }
    spin_stuck_count = 0U;
    spin_reverse_cooldown = 0U;
    virtual_buffer_energy_j = CHASSIS_POWER_BUFFER_LIMIT_J;
    expected_power_limit_w = 0.0f;
}

/* 小陀螺卡死换向：实测 yaw 角速度持续远低于目标自转时反转方向，换向后的冷却期内不再检测。 */
static void update_spin_stuck_watchdog(chassis_behavior_t behavior, const chassis_feedback_t *feedback)
{
    if (spin_reverse_cooldown > 0U)
    {
        spin_reverse_cooldown--;
        spin_stuck_count = 0U;
        return;
    }

    /* 没有角速度数据时不判定，避免姿态解算启动前误换向。 */
    const float expected = (behavior == CHASSIS_BEHAVIOR_SPIN_FAST ? CHASSIS_SPIN_FAST_RATIO : CHASSIS_SPIN_SLOW_RATIO) * CHASSIS_MAX_ANGULAR_RAD_S;
    if ((behavior != CHASSIS_BEHAVIOR_SPIN_FAST && behavior != CHASSIS_BEHAVIOR_SPIN_SLOW) || !feedback->yaw_rate_valid
        || fabsf(feedback->yaw_rate_rad_s) >= CHASSIS_SPIN_STUCK_SPEED_RATIO * expected)
    {
        spin_stuck_count = 0U;
        return;
    }

    if (++spin_stuck_count < spin_stuck_confirm_cycles)
    {
        return;
    }
    spinning_forward = !spinning_forward;
    spin_reverse_cooldown = spin_reverse_cooldown_cycles;
    spin_stuck_count = 0U;
}

/* 行为目标：平移输入按行为确定坐标系并整体限幅，自转按行为生成。input 的来源已由模式选定。
 * target 依次为底盘坐标系 x、y 向速度（m/s）和自转角速度（rad/s）。 */
static void update_target_velocity(chassis_behavior_t behavior, const float input[3], const chassis_command_t *command,
                                   const chassis_feedback_t *feedback, float target[3])
{
    if (behavior == CHASSIS_BEHAVIOR_CLIMB)
    {
        /* 爬坡速度以底盘正向为正向：上坡为正前进，下坡为负倒车。 */
        target[0] = command->climb_speed;
        target[1] = 0.0f;
    }
    else
    {
        /* 直控的平移输入已在底盘坐标系；其余行为在云台坐标系，按云台相对角转到底盘坐标系。 */
        target[0] = input[0];
        target[1] = input[1];
        if (behavior != CHASSIS_BEHAVIOR_DIRECT)
        {
            const float cos_yaw = cosf(feedback->gimbal_yaw_angle);
            const float sin_yaw = sinf(feedback->gimbal_yaw_angle);
            target[0] = cos_yaw * input[0] - sin_yaw * input[1];
            target[1] = sin_yaw * input[0] + cos_yaw * input[1];
        }
        const float speed = hypotf(target[0], target[1]);
        if (speed > CHASSIS_MAX_TRANSLATION_MPS)
        {
            target[0] *= CHASSIS_MAX_TRANSLATION_MPS / speed;
            target[1] *= CHASSIS_MAX_TRANSLATION_MPS / speed;
        }
    }

    switch (behavior)
    {
    case CHASSIS_BEHAVIOR_DIRECT:
        target[2] = float_constrain(input[2], -CHASSIS_MAX_ANGULAR_RAD_S, CHASSIS_MAX_ANGULAR_RAD_S);
        break;

    case CHASSIS_BEHAVIOR_SPIN_SLOW:
        target[2] = CHASSIS_SPIN_SLOW_RATIO * (spinning_forward ? CHASSIS_MAX_ANGULAR_RAD_S : -CHASSIS_MAX_ANGULAR_RAD_S);
        break;

    case CHASSIS_BEHAVIOR_SPIN_FAST:
        target[2] = CHASSIS_SPIN_FAST_RATIO * (spinning_forward ? CHASSIS_MAX_ANGULAR_RAD_S : -CHASSIS_MAX_ANGULAR_RAD_S);
        break;

    case CHASSIS_BEHAVIOR_GIMBAL_FOLLOW:
    {
        /* 底盘相对云台目标朝向的无符号角误差，范围 0～2π。 */
        float error = feedback->gimbal_yaw_angle_error;
        if (error < 0.0f)
        {
            error += TWO_PI;
        }
        error += feedback->gimbal_yaw_angle;
        if (error >= TWO_PI)
        {
            error -= TWO_PI;
        }
        /* 正前方对齐按 2π 周期折算到半圈以内。 */
        error = fmodf(error, TWO_PI);
        if (error > TWO_PI / 2.0f)
        {
            error -= TWO_PI;
        }
        target[2] = pid_calculator_update(&following_pid, error);
        break;
    }

    case CHASSIS_BEHAVIOR_CLIMB:
        /* 跟随爬坡朝向，角误差折算到 -π～π。 */
        target[2] = pid_calculator_update(&following_pid,
                                          remainderf(command->climb_direction - command->climb_measure_yaw, TWO_PI));
        break;
    }
}

/* 功率预算：裁判上限叠加 boost 后按电容电压折减，再加额外功率并按虚拟缓冲能量比例折减，
 * 返回本周期底盘可用功率 W。power_unlimited 为真时不受上述上限约束，虚拟缓冲能量仍照常积分。 */
static float update_power_limit(bool power_unlimited, bool boost_request, const chassis_feedback_t *feedback)
{
    if (!feedback->referee_online)
    {
        /* 裁判离线时缓冲能量未知，使用固定保守上限，不叠加额外功率。 */
        virtual_buffer_energy_j = CHASSIS_POWER_BUFFER_LIMIT_J;
        expected_power_limit_w = CHASSIS_POWER_OFFLINE_LIMIT_W;
        return power_unlimited ? POWER_UNLIMITED_W : CHASSIS_POWER_OFFLINE_LIMIT_W;
    }

    /* 电容离线时没有实测功率，不积分，仅由裁判缓冲能量限制虚拟缓冲。 */
    if (feedback->supercap_online)
    {
        virtual_buffer_energy_j += control_period_s * (expected_power_limit_w - feedback->chassis_power_w);
    }
    virtual_buffer_energy_j = float_constrain(virtual_buffer_energy_j, 0.0f,
                                              fminf(feedback->buffer_energy_j, CHASSIS_POWER_BUFFER_LIMIT_J));

    /* boost：有请求且电容允许放电时，在裁判上限之上放开固定的额外功率，仍属受限。 */
    float power_limit_w = feedback->power_limit_w;
    if (boost_request && feedback->supercap_enabled)
    {
        power_limit_w += CHASSIS_POWER_BOOST_EXCESS_W;
    }
    expected_power_limit_w = power_limit_w;

    /* 超出裁判上限的部分随电容电压从基准线到控制线线性放开，电压不足时回落到裁判上限。 */
    power_limit_w = feedback->power_limit_w + (power_limit_w - feedback->power_limit_w)
        * float_constrain((feedback->supercap_voltage_v - CHASSIS_SUPERCAP_VOLTAGE_BASE_V)
                          / (CHASSIS_SUPERCAP_VOLTAGE_CONTROL_V - CHASSIS_SUPERCAP_VOLTAGE_BASE_V), 0.0f, 1.0f);

    return power_unlimited ? POWER_UNLIMITED_W
        : (power_limit_w + CHASSIS_POWER_EXCESS_W) * virtual_buffer_energy_j / CHASSIS_POWER_BUFFER_LIMIT_J;
}
