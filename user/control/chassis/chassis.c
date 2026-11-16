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

static pid_calculator_t translation_x_pid;
static pid_calculator_t translation_y_pid;
static pid_calculator_t angular_pid;
static pid_calculator_t wheel_pid[CHASSIS_MOTOR_COUNT];
static pid_calculator_t following_pid;
static float control_period_seconds;
static float virtual_buffer_energy_j;
static float expected_power_limit_w;
static chassis_mode_t previous_mode;
static bool spinning_forward = true;
static uint32_t spin_stuck_cycle_count;
static uint32_t spin_reverse_cooldown_cycles_remaining;
static uint32_t spin_stuck_confirmation_cycles;
static uint32_t spin_reverse_cooldown_duration_cycles;

static void reset_control(void);
static void update_spin_stuck_watchdog(chassis_behavior_t chassis_behavior,
                                       const chassis_feedback_t *chassis_feedback);
static void update_target_velocity(chassis_behavior_t chassis_behavior,
                                   float requested_velocity_x, float requested_velocity_y,
                                   float requested_angular_velocity, const chassis_command_t *control_command,
                                   const chassis_feedback_t *chassis_feedback, float *target_velocity);
static float update_power_limit(bool power_limit_bypassed, bool boost_request,
                                const chassis_feedback_t *chassis_feedback);

bool chassis_init(float sample_period_seconds)
{
    control_period_seconds = sample_period_seconds;
    spin_stuck_confirmation_cycles = (uint32_t)(CHASSIS_SPIN_STUCK_CONFIRM_S / sample_period_seconds + 0.5f);
    spin_reverse_cooldown_duration_cycles = (uint32_t)(CHASSIS_SPIN_REVERSE_COOLDOWN_S / sample_period_seconds + 0.5f);

    pid_calculator_init(&translation_x_pid, CHASSIS_TRANSLATION_KP, CHASSIS_TRANSLATION_KI, CHASSIS_TRANSLATION_KD);
    pid_calculator_init(&translation_y_pid, CHASSIS_TRANSLATION_KP, CHASSIS_TRANSLATION_KI, CHASSIS_TRANSLATION_KD);
    pid_calculator_init(&angular_pid, CHASSIS_ANGULAR_KP, CHASSIS_ANGULAR_KI, CHASSIS_ANGULAR_KD);
    for (uint32_t motor_index = 0U; motor_index < CHASSIS_MOTOR_COUNT; motor_index++)
    {
        pid_calculator_init(&wheel_pid[motor_index], CHASSIS_WHEEL_KP, CHASSIS_WHEEL_KI, CHASSIS_WHEEL_KD);
    }
    pid_calculator_init(&following_pid, CHASSIS_FOLLOW_KP, CHASSIS_FOLLOW_KI, CHASSIS_FOLLOW_KD);
    following_pid.output_min = -CHASSIS_MAX_ANGULAR_RAD_S;
    following_pid.output_max = CHASSIS_MAX_ANGULAR_RAD_S;

    reset_control();
    return chassis_port_init();
}

bool chassis_update(const chassis_command_t *control_command, uint32_t current_tick)
{
    const chassis_mode_t chassis_mode = actChassis;
    /* 底盘只由一个周期任务调用；控制周期工作区使用静态存储，避免占用任务栈。 */
    static chassis_feedback_t chassis_feedback;
    static float wheel_speed_rad_s[CHASSIS_MOTOR_COUNT];
    static float target_velocity[3];
    static float wheel_torque[CHASSIS_MOTOR_COUNT];
    static float power_constraint_coefficients[6];
    static qcp_workspace_t power_constraint_workspace;
    bool all_motors_online = true;

    for (uint32_t motor_index = 0U; motor_index < CHASSIS_MOTOR_COUNT; motor_index++)
    {
        wheel_speed_rad_s[motor_index] = chassis_motors[motor_index].speed_rad_s;
        if (!chassis_motor_is_online(&chassis_motors[motor_index]))
        {
            all_motors_online = false;
        }
    }

    chassis_port_read_feedback(current_tick, &chassis_feedback);
    target_velocity[0] = 0.0f;
    target_velocity[1] = 0.0f;
    target_velocity[2] = 0.0f;

    /* 电容充电功率上限只由裁判功率上限和缓冲能量决定，与底盘是否使能无关，每周期更新：
     * 缓冲能量低于基准线时按比例压低裁判上限，高于基准线后逐步放开额外充电功率。裁判离线时不充电。 */
    float supercap_charge_power_limit_w = 0.0f;
    if (chassis_feedback.referee_online)
    {
        supercap_charge_power_limit_w = chassis_feedback.power_limit_w
            * float_constrain((chassis_feedback.buffer_energy_j - CHASSIS_SUPERCAP_CHARGE_STOP_J)
            / (CHASSIS_SUPERCAP_CHARGE_BASE_J - CHASSIS_SUPERCAP_CHARGE_STOP_J), 0.0f, 1.0f)
            + CHASSIS_SUPERCAP_CHARGE_EXCESS_W
            * float_constrain((chassis_feedback.buffer_energy_j - CHASSIS_SUPERCAP_CHARGE_BASE_J)
            / (CHASSIS_SUPERCAP_CHARGE_FULL_J - CHASSIS_SUPERCAP_CHARGE_BASE_J), 0.0f, 1.0f);
    }

    /* 遥控器切入快速小陀螺时换向；全自动下上位机指定的小陀螺不改变方向。 */
    if (chassis_mode == CHASSIS_SPIN_FAST && previous_mode != CHASSIS_SPIN_FAST)
    {
        spinning_forward = !spinning_forward;
    }
    previous_mode = chassis_mode;

    /* 底盘速度由四轮共同解算，任一轮无反馈时估计失效，四轮一起停止。 */
    if (chassis_mode == CHASSIS_STOP || !all_motors_online)
    {
        reset_control();
        return chassis_port_send(supercap_charge_power_limit_w);
    }

    /* 行为和输入来源由模式唯一决定，两种来源不叠加：遥控器模式各固定一种行为，只取摇杆；
     * 全自动只取上位机指令及其指定的子行为，链路离线时为直控零速度。
     * target_velocity 先暂存选定的 x、y 向平移速度（m/s）和自转角速度（rad/s），再原地转换为最终目标。 */
    chassis_behavior_t chassis_behavior = CHASSIS_BEHAVIOR_DIRECT;
    if (chassis_mode != CHASSIS_AUTO)
    {
        if (chassis_mode == CHASSIS_GIMBAL_FOLLOW_POWERED)
        {
            chassis_behavior = CHASSIS_BEHAVIOR_GIMBAL_FOLLOW;
        }
        else if (chassis_mode == CHASSIS_SPIN_FAST)
        {
            chassis_behavior = CHASSIS_BEHAVIOR_SPIN_FAST;
        }
        target_velocity[0] = control_command->joystick_x * CHASSIS_MAX_TRANSLATION_MPS;
        target_velocity[1] = control_command->joystick_y * CHASSIS_MAX_TRANSLATION_MPS;
        target_velocity[2] = control_command->joystick_angular * CHASSIS_MAX_ANGULAR_RAD_S;
    }
    else
    {
        if (control_command->navigation_enabled)
        {
            chassis_behavior = control_command->navigation_behavior;
            target_velocity[0] = control_command->navigation_velocity_x;
            target_velocity[1] = control_command->navigation_velocity_y;
            target_velocity[2] = control_command->navigation_angular_velocity;
        }
        /* 爬坡输入有效时进入爬坡；上位机指定爬坡而输入无效时退回直控。 */
        if (isfinite(control_command->climb_direction) && isfinite(control_command->climb_speed)
            && isfinite(control_command->climb_measure_yaw))
        {
            chassis_behavior = CHASSIS_BEHAVIOR_CLIMB;
        }
        else if (chassis_behavior == CHASSIS_BEHAVIOR_CLIMB)
        {
            chassis_behavior = CHASSIS_BEHAVIOR_DIRECT;
        }
    }

    update_spin_stuck_watchdog(chassis_behavior, &chassis_feedback);

    update_target_velocity(chassis_behavior, target_velocity[0], target_velocity[1], target_velocity[2],
                           control_command, &chassis_feedback, target_velocity);
    if (!isfinite(target_velocity[0]) || !isfinite(target_velocity[1]) || !isfinite(target_velocity[2]))
    {
        /* 目标无效时与停止态同样处理，对应 RMCS 对 NaN 速度目标的复位。 */
        reset_control();
        return chassis_port_send(supercap_charge_power_limit_w);
    }
    const float available_power_limit_w = update_power_limit(
        chassis_mode == CHASSIS_GIMBAL_FOLLOW_POWERED,
        chassis_mode == CHASSIS_AUTO && control_command->navigation_enabled && control_command->boost_request,
        &chassis_feedback);

    /* 由轮速解算底盘实际速度。 */
    const float chassis_velocity_x = (-wheel_speed_rad_s[0] - wheel_speed_rad_s[1]
                                      + wheel_speed_rad_s[2] + wheel_speed_rad_s[3]) * VELOCITY_SCALE;
    const float chassis_velocity_y = (wheel_speed_rad_s[0] - wheel_speed_rad_s[1]
                                      - wheel_speed_rad_s[2] + wheel_speed_rad_s[3]) * VELOCITY_SCALE;
    const float chassis_angular_velocity = (wheel_speed_rad_s[0] + wheel_speed_rad_s[1]
                                            + wheel_speed_rad_s[2] + wheel_speed_rad_s[3]) / SPIN_LEVER * VELOCITY_SCALE;

    /* 平移、自转速度环，输出折算为轮侧力矩；x 分量为平移合力矩，y 分量为自转力矩。 */
    const float translation_torque_x = VELOCITY_SCALE * CHASSIS_MASS_KG
        * pid_calculator_update(&translation_x_pid, target_velocity[0] - chassis_velocity_x);
    const float translation_torque_y = VELOCITY_SCALE * CHASSIS_MASS_KG
        * pid_calculator_update(&translation_y_pid, target_velocity[1] - chassis_velocity_y);
    const float translation_torque_magnitude = hypotf(translation_torque_x, translation_torque_y);
    const float angular_torque = VELOCITY_SCALE * (CHASSIS_INERTIA_KGM2 / SPIN_LEVER)
        * pid_calculator_update(&angular_pid, target_velocity[2] - chassis_angular_velocity);
    const float wheel_translation_factor_1 = translation_torque_magnitude > 0.0f
        ? (-translation_torque_x + translation_torque_y) / translation_torque_magnitude : -1.0f;
    const float wheel_translation_factor_2 = translation_torque_magnitude > 0.0f
        ? (-translation_torque_x - translation_torque_y) / translation_torque_magnitude : -1.0f;

    /* 轮速环：目标为实测底盘速度反解的轮速，只修正各轮相对整车运动的偏差。 */
    float wheel_torque_square_sum = 0.0f;
    float wheel_speed_square_sum = 0.0f;
    float wheel_mechanical_power_w = 0.0f;
    for (uint32_t motor_index = 0U; motor_index < CHASSIS_MOTOR_COUNT; motor_index++)
    {
        float wheel_speed_reference;
        switch (motor_index)
        {
        case 0U:
            wheel_speed_reference = -chassis_velocity_x + chassis_velocity_y
                + SPIN_LEVER * chassis_angular_velocity;
            break;
        case 1U:
            wheel_speed_reference = -chassis_velocity_x - chassis_velocity_y
                + SPIN_LEVER * chassis_angular_velocity;
            break;
        case 2U:
            wheel_speed_reference = chassis_velocity_x - chassis_velocity_y
                + SPIN_LEVER * chassis_angular_velocity;
            break;
        default:
            wheel_speed_reference = chassis_velocity_x + chassis_velocity_y
                + SPIN_LEVER * chassis_angular_velocity;
            break;
        }
        wheel_torque[motor_index] = pid_calculator_update(&wheel_pid[motor_index],
            wheel_speed_reference * (-1.0f / (SQRT2 * CHASSIS_WHEEL_RADIUS_M))
            - wheel_speed_rad_s[motor_index]);
        wheel_torque_square_sum += wheel_torque[motor_index] * wheel_torque[motor_index];
        wheel_speed_square_sum += wheel_speed_rad_s[motor_index] * wheel_speed_rad_s[motor_index];
        wheel_mechanical_power_w += wheel_torque[motor_index] * wheel_speed_rad_s[motor_index];
    }

    /* 在摩擦菱形和功率二次约束内，最大化平移与自转力矩。 */
    const float friction_torque_limit = CHASSIS_FRICTION_COEFFICIENT * CHASSIS_MASS_KG * CHASSIS_GRAVITY_MPS2
        * CHASSIS_WHEEL_RADIUS_M / 4.0f;
    power_constraint_coefficients[0] = 4.0f * CHASSIS_POWER_K1;
    power_constraint_coefficients[1] = 0.0f;
    power_constraint_coefficients[2] = 4.0f * CHASSIS_POWER_K1;
    power_constraint_coefficients[3] = wheel_translation_factor_1 * (wheel_speed_rad_s[0] - wheel_speed_rad_s[2]
                                                                       + 2.0f * CHASSIS_POWER_K1 * (wheel_torque[0] - wheel_torque[2]))
        + wheel_translation_factor_2 * (wheel_speed_rad_s[1] - wheel_speed_rad_s[3]
                                        + 2.0f * CHASSIS_POWER_K1 * (wheel_torque[1] - wheel_torque[3]));
    power_constraint_coefficients[4] = (angular_torque > 0.0f ? 1.0f : -1.0f)
        * (2.0f * CHASSIS_POWER_K1 * (wheel_torque[0] + wheel_torque[1]
                                      + wheel_torque[2] + wheel_torque[3])
           + wheel_speed_rad_s[0] + wheel_speed_rad_s[1]
           + wheel_speed_rad_s[2] + wheel_speed_rad_s[3]);
    power_constraint_coefficients[5] = CHASSIS_POWER_K1 * wheel_torque_square_sum + wheel_mechanical_power_w
        + CHASSIS_POWER_K2 * wheel_speed_square_sum - CHASSIS_POWER_NO_LOAD_W - available_power_limit_w;
    const qcp_point_t optimized_torque = qcp_solve((qcp_point_t){1.0f, 1.0f}, translation_torque_magnitude,
                                                   fabsf(angular_torque),
                                                   friction_torque_limit / fmaxf(fabsf(wheel_translation_factor_1),
                                                                                 fabsf(wheel_translation_factor_2)),
                                                   friction_torque_limit, power_constraint_coefficients,
                                                   &power_constraint_workspace);
    const float optimized_angular_torque = optimized_torque.y
        * (angular_torque > 0.0f ? 1.0f : -1.0f);

    chassis_motors[0].output = (+wheel_translation_factor_1 * optimized_torque.x
                                + optimized_angular_torque + wheel_torque[0]) * CHASSIS_MOTOR_RAW_PER_NM;
    chassis_motors[1].output = (+wheel_translation_factor_2 * optimized_torque.x
                                + optimized_angular_torque + wheel_torque[1]) * CHASSIS_MOTOR_RAW_PER_NM;
    chassis_motors[2].output = (-wheel_translation_factor_1 * optimized_torque.x
                                + optimized_angular_torque + wheel_torque[2]) * CHASSIS_MOTOR_RAW_PER_NM;
    chassis_motors[3].output = (-wheel_translation_factor_2 * optimized_torque.x
                                + optimized_angular_torque + wheel_torque[3]) * CHASSIS_MOTOR_RAW_PER_NM;

    if (!chassis_port_send(supercap_charge_power_limit_w))
    {
        reset_control();
        return false;
    }
    return true;
}

/* 停止态：清空环路状态、卡死检测和输出，虚拟缓冲能量恢复为上限；小陀螺方向保持。 */
static void reset_control(void)
{
    pid_calculator_reset(&translation_x_pid);
    pid_calculator_reset(&translation_y_pid);
    pid_calculator_reset(&angular_pid);
    pid_calculator_reset(&following_pid);
    for (uint32_t motor_index = 0U; motor_index < CHASSIS_MOTOR_COUNT; motor_index++)
    {
        pid_calculator_reset(&wheel_pid[motor_index]);
        chassis_motors[motor_index].output = 0.0f;
    }
    spin_stuck_cycle_count = 0U;
    spin_reverse_cooldown_cycles_remaining = 0U;
    virtual_buffer_energy_j = CHASSIS_POWER_BUFFER_LIMIT_J;
    expected_power_limit_w = 0.0f;
}

/* 小陀螺卡死换向：实测 yaw 角速度持续远低于目标自转时反转方向，换向后的冷却期内不再检测。 */
static void update_spin_stuck_watchdog(chassis_behavior_t chassis_behavior,
                                       const chassis_feedback_t *chassis_feedback)
{
    if (spin_reverse_cooldown_cycles_remaining > 0U)
    {
        spin_reverse_cooldown_cycles_remaining--;
        spin_stuck_cycle_count = 0U;
        return;
    }

    /* 没有角速度数据时不判定，避免姿态解算启动前误换向。 */
    const float target_angular_speed = (chassis_behavior == CHASSIS_BEHAVIOR_SPIN_FAST
                                        ? CHASSIS_SPIN_FAST_RATIO : CHASSIS_SPIN_SLOW_RATIO)
        * CHASSIS_MAX_ANGULAR_RAD_S;
    if ((chassis_behavior != CHASSIS_BEHAVIOR_SPIN_FAST && chassis_behavior != CHASSIS_BEHAVIOR_SPIN_SLOW)
        || !chassis_feedback->yaw_rate_valid
        || fabsf(chassis_feedback->yaw_rate_rad_s) >= CHASSIS_SPIN_STUCK_SPEED_RATIO * target_angular_speed)
    {
        spin_stuck_cycle_count = 0U;
        return;
    }

    if (++spin_stuck_cycle_count < spin_stuck_confirmation_cycles)
    {
        return;
    }
    spinning_forward = !spinning_forward;
    spin_reverse_cooldown_cycles_remaining = spin_reverse_cooldown_duration_cycles;
    spin_stuck_cycle_count = 0U;
}

/* 行为目标：平移输入按行为确定坐标系并整体限幅，自转按行为生成。
 * requested_velocity_x、requested_velocity_y 和 requested_angular_velocity 的来源已由模式选定；
 * target_velocity 依次为底盘坐标系 x、y 向速度（m/s）和自转角速度（rad/s）。 */
static void update_target_velocity(chassis_behavior_t chassis_behavior,
                                   float requested_velocity_x, float requested_velocity_y,
                                   float requested_angular_velocity, const chassis_command_t *control_command,
                                   const chassis_feedback_t *chassis_feedback, float *target_velocity)
{
    if (chassis_behavior == CHASSIS_BEHAVIOR_CLIMB)
    {
        /* 爬坡速度以底盘正向为正向：上坡为正前进，下坡为负倒车。 */
        target_velocity[0] = control_command->climb_speed;
        target_velocity[1] = 0.0f;
    }
    else
    {
        /* 直控的平移输入已在底盘坐标系；其余行为在云台坐标系，按云台相对角转到底盘坐标系。 */
        target_velocity[0] = requested_velocity_x;
        target_velocity[1] = requested_velocity_y;
        if (chassis_behavior != CHASSIS_BEHAVIOR_DIRECT)
        {
            const float cos_gimbal_yaw = cosf(chassis_feedback->gimbal_yaw_angle);
            const float sin_gimbal_yaw = sinf(chassis_feedback->gimbal_yaw_angle);
            target_velocity[0] = cos_gimbal_yaw * requested_velocity_x - sin_gimbal_yaw * requested_velocity_y;
            target_velocity[1] = sin_gimbal_yaw * requested_velocity_x + cos_gimbal_yaw * requested_velocity_y;
        }
        const float translation_speed = hypotf(target_velocity[0], target_velocity[1]);
        if (translation_speed > CHASSIS_MAX_TRANSLATION_MPS)
        {
            target_velocity[0] *= CHASSIS_MAX_TRANSLATION_MPS / translation_speed;
            target_velocity[1] *= CHASSIS_MAX_TRANSLATION_MPS / translation_speed;
        }
    }

    switch (chassis_behavior)
    {
    case CHASSIS_BEHAVIOR_DIRECT:
        target_velocity[2] = float_constrain(requested_angular_velocity,
                                               -CHASSIS_MAX_ANGULAR_RAD_S, CHASSIS_MAX_ANGULAR_RAD_S);
        break;

    case CHASSIS_BEHAVIOR_SPIN_SLOW:
        target_velocity[2] = CHASSIS_SPIN_SLOW_RATIO
            * (spinning_forward ? CHASSIS_MAX_ANGULAR_RAD_S : -CHASSIS_MAX_ANGULAR_RAD_S);
        break;

    case CHASSIS_BEHAVIOR_SPIN_FAST:
        target_velocity[2] = CHASSIS_SPIN_FAST_RATIO
            * (spinning_forward ? CHASSIS_MAX_ANGULAR_RAD_S : -CHASSIS_MAX_ANGULAR_RAD_S);
        break;

    case CHASSIS_BEHAVIOR_GIMBAL_FOLLOW:
    {
        /* 底盘相对云台目标朝向的无符号角误差，范围 0～2π。 */
        float yaw_alignment_error = chassis_feedback->gimbal_yaw_angle_error;
        if (yaw_alignment_error < 0.0f)
        {
            yaw_alignment_error += TWO_PI;
        }
        yaw_alignment_error += chassis_feedback->gimbal_yaw_angle;
        if (yaw_alignment_error >= TWO_PI)
        {
            yaw_alignment_error -= TWO_PI;
        }
        /* 正前方对齐按 2π 周期折算到半圈以内。 */
        yaw_alignment_error = fmodf(yaw_alignment_error, TWO_PI);
        if (yaw_alignment_error > TWO_PI / 2.0f)
        {
            yaw_alignment_error -= TWO_PI;
        }
        target_velocity[2] = pid_calculator_update(&following_pid, yaw_alignment_error);
        break;
    }

    case CHASSIS_BEHAVIOR_CLIMB:
        /* 跟随爬坡朝向，角误差折算到 -π～π。 */
        target_velocity[2] = pid_calculator_update(&following_pid,
                                                   remainderf(control_command->climb_direction
                                                               - control_command->climb_measure_yaw, TWO_PI));
        break;
    }
}

/* 功率预算：裁判上限叠加 boost 后按电容电压折减，再加额外功率并按虚拟缓冲能量比例折减，
 * 返回本周期底盘可用功率 W。power_limit_bypassed 为真时不受上述上限约束，虚拟缓冲能量仍照常积分。 */
static float update_power_limit(bool power_limit_bypassed, bool boost_request,
                                const chassis_feedback_t *chassis_feedback)
{
    if (!chassis_feedback->referee_online)
    {
        /* 裁判离线时缓冲能量未知，使用固定保守上限，不叠加额外功率。 */
        virtual_buffer_energy_j = CHASSIS_POWER_BUFFER_LIMIT_J;
        expected_power_limit_w = CHASSIS_POWER_OFFLINE_LIMIT_W;
        return power_limit_bypassed ? POWER_UNLIMITED_W : CHASSIS_POWER_OFFLINE_LIMIT_W;
    }

    /* 电容离线时没有实测功率，不积分，仅由裁判缓冲能量限制虚拟缓冲。 */
    if (chassis_feedback->supercap_online)
    {
        virtual_buffer_energy_j += control_period_seconds
            * (expected_power_limit_w - chassis_feedback->chassis_power_w);
    }
    virtual_buffer_energy_j = float_constrain(virtual_buffer_energy_j, 0.0f,
                                              fminf(chassis_feedback->buffer_energy_j, CHASSIS_POWER_BUFFER_LIMIT_J));

    /* boost：有请求且电容允许放电时，在裁判上限之上放开固定的额外功率，仍属受限。 */
    float available_power_limit_w = chassis_feedback->power_limit_w;
    if (boost_request && chassis_feedback->supercap_enabled)
    {
        available_power_limit_w += CHASSIS_POWER_BOOST_EXCESS_W;
    }
    expected_power_limit_w = available_power_limit_w;

    /* 超出裁判上限的部分随电容电压从启动阈值到全开阈值线性放开，电压不足时回落到裁判上限。 */
    available_power_limit_w = chassis_feedback->power_limit_w
        + (available_power_limit_w - chassis_feedback->power_limit_w)
        * float_constrain((chassis_feedback->supercap_voltage_v - CHASSIS_SUPERCAP_BOOST_START_V)
                          / (CHASSIS_SUPERCAP_BOOST_FULL_V - CHASSIS_SUPERCAP_BOOST_START_V), 0.0f, 1.0f);

    return power_limit_bypassed ? POWER_UNLIMITED_W
        : (available_power_limit_w + CHASSIS_POWER_EXCESS_W) * virtual_buffer_energy_j / CHASSIS_POWER_BUFFER_LIMIT_J;
}
