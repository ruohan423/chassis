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

/* 底盘坐标系速度：vx 向前、vy 向左，单位 m/s；wz 逆时针为正，单位 rad/s。 */
typedef struct
{
    float vx;
    float vy;
    float wz;
} chassis_velocity_t;

volatile chassis_mode_t actChassis = CHASSIS_FREE;

static pid_calculator_t vx_pid;
static pid_calculator_t vy_pid;
static pid_calculator_t wz_pid;
static pid_calculator_t wheel_pid[CHASSIS_MOTOR_COUNT];
static pid_calculator_t follow_pid;
static float control_period_s;
static float virtual_buffer_j;         /* 虚拟缓冲能量 J */
static float expected_power_w;         /* 上一周期期望的功率上限 W，作为虚拟缓冲能量的积分基准 */
static bool spin_forward = true;       /* 小陀螺方向，真为逆时针 */
static bool remote_spin_active;        /* 上一周期处于遥控器小陀螺子模式 */
static uint32_t spin_stuck_cycles;     /* 小陀螺低速已持续的周期数 */
static uint32_t spin_cooldown_cycles;  /* 换向后剩余的检测冷却周期数 */
static uint32_t spin_stuck_confirm_cycles;
static uint32_t spin_cooldown_reload_cycles;

static void reset_control(void);
static void update_spin_watchdog(chassis_behavior_t behavior, const chassis_feedback_t *feedback);
static void resolve_target(chassis_behavior_t behavior, const chassis_command_t *command,
                           const chassis_feedback_t *feedback, chassis_velocity_t *target);
static float update_power_limit(bool power_unlimited, bool boost, const chassis_feedback_t *feedback);

bool chassis_init(float period_s)
{
    control_period_s = period_s;
    spin_stuck_confirm_cycles = (uint32_t)(CHASSIS_SPIN_STUCK_CONFIRM_S / period_s + 0.5f);
    spin_cooldown_reload_cycles = (uint32_t)(CHASSIS_SPIN_REVERSE_COOLDOWN_S / period_s + 0.5f);

    pid_calculator_init(&vx_pid, CHASSIS_TRANSLATION_KP, CHASSIS_TRANSLATION_KI, CHASSIS_TRANSLATION_KD);
    pid_calculator_init(&vy_pid, CHASSIS_TRANSLATION_KP, CHASSIS_TRANSLATION_KI, CHASSIS_TRANSLATION_KD);
    pid_calculator_init(&wz_pid, CHASSIS_ANGULAR_KP, CHASSIS_ANGULAR_KI, CHASSIS_ANGULAR_KD);
    for (uint32_t i = 0U; i < CHASSIS_MOTOR_COUNT; i++)
    {
        pid_calculator_init(&wheel_pid[i], CHASSIS_WHEEL_KP, CHASSIS_WHEEL_KI, CHASSIS_WHEEL_KD);
    }
    pid_calculator_init(&follow_pid, CHASSIS_FOLLOW_KP, CHASSIS_FOLLOW_KI, CHASSIS_FOLLOW_KD);
    follow_pid.output_min = -CHASSIS_MAX_ANGULAR_RAD_S;
    follow_pid.output_max = CHASSIS_MAX_ANGULAR_RAD_S;

    reset_control();
    return chassis_port_init();
}

bool chassis_update(const chassis_command_t *command, uint32_t tick)
{
    /* 各轮速度参考中 x、y 向底盘速度的符号，按逻辑轮序排列。 */
    static const float wheel_sign_x[CHASSIS_MOTOR_COUNT] = {-1.0f, -1.0f, 1.0f, 1.0f};
    static const float wheel_sign_y[CHASSIS_MOTOR_COUNT] = {1.0f, -1.0f, -1.0f, 1.0f};
    /* 底盘只由一个周期任务调用；控制周期工作区使用静态存储，避免占用任务栈。 */
    static chassis_feedback_t feedback;
    static float wheel_speed[CHASSIS_MOTOR_COUNT];  /* 轮侧角速度 rad/s */
    static float wheel_torque[CHASSIS_MOTOR_COUNT]; /* 轮速环输出的轮侧力矩 Nm */
    static float power_coeffs[6];
    static qcp_workspace_t qcp_workspace;
    static chassis_velocity_t target;

    const chassis_mode_t mode = actChassis;
    const bool power_unlimited = mode == CHASSIS_FREE_UNLIMITED || mode == CHASSIS_FOLLOW_UNLIMITED
        || mode == CHASSIS_SPIN_UNLIMITED;
    const bool remote_spin = mode == CHASSIS_SPIN || mode == CHASSIS_SPIN_UNLIMITED;
    bool motors_online = true;

    for (uint32_t i = 0U; i < CHASSIS_MOTOR_COUNT; i++)
    {
        wheel_speed[i] = chassis_motors[i].speed_rad_s;
        if (!chassis_motor_is_online(&chassis_motors[i]))
        {
            motors_online = false;
        }
    }
    chassis_port_read_feedback(tick, &feedback);

    /* 电容充电功率上限只由裁判功率上限和缓冲能量决定，与底盘是否使能无关，每周期更新：
     * 缓冲能量低于基准线时按比例压低裁判上限，高于基准线后逐步放开额外充电功率。裁判离线时不充电。 */
    float charge_limit_w = 0.0f;
    if (feedback.referee_online)
    {
        charge_limit_w = feedback.power_limit_w
            * float_constrain((feedback.buffer_energy_j - CHASSIS_SUPERCAP_CHARGE_STOP_J)
                              / (CHASSIS_SUPERCAP_CHARGE_BASE_J - CHASSIS_SUPERCAP_CHARGE_STOP_J), 0.0f, 1.0f)
            + CHASSIS_SUPERCAP_CHARGE_EXCESS_W
            * float_constrain((feedback.buffer_energy_j - CHASSIS_SUPERCAP_CHARGE_BASE_J)
                              / (CHASSIS_SUPERCAP_CHARGE_FULL_J - CHASSIS_SUPERCAP_CHARGE_BASE_J), 0.0f, 1.0f);
    }

    /* 遥控器切入小陀螺子模式时换向，仅切换功率约束时不换向；全自动下上位机指定的小陀螺不改变方向。 */
    if (remote_spin && !remote_spin_active)
    {
        spin_forward = !spin_forward;
    }
    remote_spin_active = remote_spin;

    /* 底盘速度由四轮共同解算，任一轮无反馈时估计失效，四轮一起停止。 */
    if (mode == CHASSIS_STOP || !motors_online)
    {
        reset_control();
        return chassis_port_send(charge_limit_w);
    }

    /* 行为和输入来源由模式唯一决定，两种来源不叠加：遥控器子模式各固定一种行为，只取摇杆；
     * 全自动只取上位机指令及其指定的子行为，链路离线时为直控零速度。
     * target 先存选定来源的平移和自转请求，再由 resolve_target 原地换算为底盘坐标系下的最终目标。 */
    chassis_behavior_t behavior = CHASSIS_BEHAVIOR_DIRECT;
    target = (chassis_velocity_t){0};
    if (mode != CHASSIS_AUTO)
    {
        if (mode == CHASSIS_FOLLOW_UNLIMITED)
        {
            behavior = CHASSIS_BEHAVIOR_GIMBAL_FOLLOW;
        }
        else if (remote_spin)
        {
            behavior = CHASSIS_BEHAVIOR_SPIN_FAST;
        }
        target.vx = command->stick_x * CHASSIS_MAX_TRANSLATION_MPS;
        target.vy = command->stick_y * CHASSIS_MAX_TRANSLATION_MPS;
        target.wz = command->stick_wz * CHASSIS_MAX_ANGULAR_RAD_S;
    }
    else
    {
        if (command->nav_online)
        {
            behavior = command->nav_behavior;
            target.vx = command->nav_vx;
            target.vy = command->nav_vy;
            target.wz = command->nav_wz;
        }
        /* 爬坡输入有效时进入爬坡；上位机指定爬坡而输入无效时退回直控。 */
        if (isfinite(command->climb_direction) && isfinite(command->climb_speed) && isfinite(command->climb_yaw))
        {
            behavior = CHASSIS_BEHAVIOR_CLIMB;
        }
        else if (behavior == CHASSIS_BEHAVIOR_CLIMB)
        {
            behavior = CHASSIS_BEHAVIOR_DIRECT;
        }
    }

    update_spin_watchdog(behavior, &feedback);
    resolve_target(behavior, command, &feedback, &target);
    if (!isfinite(target.vx) || !isfinite(target.vy) || !isfinite(target.wz))
    {
        /* 目标无效时与停止态同样处理，对应 RMCS 对 NaN 速度目标的复位。 */
        reset_control();
        return chassis_port_send(charge_limit_w);
    }
    const float power_limit_w = update_power_limit(
        power_unlimited, mode == CHASSIS_AUTO && command->nav_online && command->boost, &feedback);

    /* 由轮速解算底盘实际速度。 */
    const float measured_vx = (-wheel_speed[0] - wheel_speed[1] + wheel_speed[2] + wheel_speed[3]) * VELOCITY_SCALE;
    const float measured_vy = (wheel_speed[0] - wheel_speed[1] - wheel_speed[2] + wheel_speed[3]) * VELOCITY_SCALE;
    const float measured_wz = (wheel_speed[0] + wheel_speed[1] + wheel_speed[2] + wheel_speed[3])
        / SPIN_LEVER * VELOCITY_SCALE;

    /* 平移、自转速度环，输出折算为轮侧力矩。lf_ratio、lb_ratio 是单位平移力矩在左前、左后轮上的分量，
     * 右后、右前轮分别与之反号。 */
    const float torque_x = VELOCITY_SCALE * CHASSIS_MASS_KG * pid_calculator_update(&vx_pid, target.vx - measured_vx);
    const float torque_y = VELOCITY_SCALE * CHASSIS_MASS_KG * pid_calculator_update(&vy_pid, target.vy - measured_vy);
    const float translation_torque = hypotf(torque_x, torque_y);
    const float spin_torque = VELOCITY_SCALE * (CHASSIS_INERTIA_KGM2 / SPIN_LEVER)
        * pid_calculator_update(&wz_pid, target.wz - measured_wz);
    const float spin_sign = spin_torque > 0.0f ? 1.0f : -1.0f;
    const float lf_ratio = translation_torque > 0.0f ? (-torque_x + torque_y) / translation_torque : -1.0f;
    const float lb_ratio = translation_torque > 0.0f ? (-torque_x - torque_y) / translation_torque : -1.0f;

    /* 轮速环：目标为实测底盘速度反解的轮速，只修正各轮相对整车运动的偏差。 */
    float torque_square_sum = 0.0f;
    float speed_square_sum = 0.0f;
    float mechanical_power_w = 0.0f;
    for (uint32_t i = 0U; i < CHASSIS_MOTOR_COUNT; i++)
    {
        const float speed_reference = wheel_sign_x[i] * measured_vx + wheel_sign_y[i] * measured_vy
            + SPIN_LEVER * measured_wz;
        wheel_torque[i] = pid_calculator_update(&wheel_pid[i],
            speed_reference * (-1.0f / (SQRT2 * CHASSIS_WHEEL_RADIUS_M)) - wheel_speed[i]);
        torque_square_sum += wheel_torque[i] * wheel_torque[i];
        speed_square_sum += wheel_speed[i] * wheel_speed[i];
        mechanical_power_w += wheel_torque[i] * wheel_speed[i];
    }

    /* 在摩擦菱形和功率二次约束内，最大化平移与自转力矩。 */
    const float friction_torque = CHASSIS_FRICTION_COEFFICIENT * CHASSIS_MASS_KG * CHASSIS_GRAVITY_MPS2
        * CHASSIS_WHEEL_RADIUS_M / 4.0f;
    power_coeffs[0] = 4.0f * CHASSIS_POWER_K1;
    power_coeffs[1] = 0.0f;
    power_coeffs[2] = 4.0f * CHASSIS_POWER_K1;
    power_coeffs[3] = lf_ratio * (wheel_speed[0] - wheel_speed[2]
                                  + 2.0f * CHASSIS_POWER_K1 * (wheel_torque[0] - wheel_torque[2]))
        + lb_ratio * (wheel_speed[1] - wheel_speed[3]
                      + 2.0f * CHASSIS_POWER_K1 * (wheel_torque[1] - wheel_torque[3]));
    power_coeffs[4] = spin_sign
        * (2.0f * CHASSIS_POWER_K1 * (wheel_torque[0] + wheel_torque[1] + wheel_torque[2] + wheel_torque[3])
           + wheel_speed[0] + wheel_speed[1] + wheel_speed[2] + wheel_speed[3]);
    power_coeffs[5] = CHASSIS_POWER_K1 * torque_square_sum + mechanical_power_w
        + CHASSIS_POWER_K2 * speed_square_sum - CHASSIS_POWER_NO_LOAD_W - power_limit_w;
    const qcp_point_t solved = qcp_solve((qcp_point_t){1.0f, 1.0f}, translation_torque, fabsf(spin_torque),
                                         friction_torque / fmaxf(fabsf(lf_ratio), fabsf(lb_ratio)),
                                         friction_torque, power_coeffs, &qcp_workspace);
    const float solved_spin_torque = solved.y * spin_sign;

    chassis_motors[0].output = (lf_ratio * solved.x + solved_spin_torque + wheel_torque[0]) * CHASSIS_MOTOR_RAW_PER_NM;
    chassis_motors[1].output = (lb_ratio * solved.x + solved_spin_torque + wheel_torque[1]) * CHASSIS_MOTOR_RAW_PER_NM;
    chassis_motors[2].output = (-lf_ratio * solved.x + solved_spin_torque + wheel_torque[2]) * CHASSIS_MOTOR_RAW_PER_NM;
    chassis_motors[3].output = (-lb_ratio * solved.x + solved_spin_torque + wheel_torque[3]) * CHASSIS_MOTOR_RAW_PER_NM;

    /* 不限功率子模式的硬件保护。电源：上面的功率约束已换成供电功率上限；
     * 电机：单轮电流上限随最热电机的反馈温度线性压低，到截止温度为零；
     * 电调与电流：最大的单轮指令超过该上限时四轮同比例缩小，保持合力方向。
     * 受功率约束的模式维持原有输出，只由 port 层逐轮限幅。 */
    if (power_unlimited)
    {
        float peak_output = 0.0f;
        uint8_t peak_temperature_c = 0U;
        for (uint32_t i = 0U; i < CHASSIS_MOTOR_COUNT; i++)
        {
            peak_output = fmaxf(peak_output, fabsf(chassis_motors[i].output));
            if (chassis_motors[i].temperature_c > peak_temperature_c)
            {
                peak_temperature_c = chassis_motors[i].temperature_c;
            }
        }
        const float output_limit = CHASSIS_MOTOR_OUTPUT_LIMIT
            * float_constrain((CHASSIS_MOTOR_TEMP_CUTOFF_C - (float)peak_temperature_c)
                              / (CHASSIS_MOTOR_TEMP_CUTOFF_C - CHASSIS_MOTOR_TEMP_DERATE_START_C), 0.0f, 1.0f);
        if (peak_output > output_limit)
        {
            for (uint32_t i = 0U; i < CHASSIS_MOTOR_COUNT; i++)
            {
                chassis_motors[i].output *= output_limit / peak_output;
            }
        }
    }

    if (!chassis_port_send(charge_limit_w))
    {
        reset_control();
        return false;
    }
    return true;
}

/* 停止态：清空环路状态、卡死检测和输出，虚拟缓冲能量恢复为上限；小陀螺方向保持。 */
static void reset_control(void)
{
    pid_calculator_reset(&vx_pid);
    pid_calculator_reset(&vy_pid);
    pid_calculator_reset(&wz_pid);
    pid_calculator_reset(&follow_pid);
    for (uint32_t i = 0U; i < CHASSIS_MOTOR_COUNT; i++)
    {
        pid_calculator_reset(&wheel_pid[i]);
        chassis_motors[i].output = 0.0f;
    }
    spin_stuck_cycles = 0U;
    spin_cooldown_cycles = 0U;
    virtual_buffer_j = CHASSIS_POWER_BUFFER_LIMIT_J;
    expected_power_w = 0.0f;
}

/* 小陀螺卡死换向：实测 yaw 角速度持续远低于目标自转时反转方向，换向后的冷却期内不再检测。 */
static void update_spin_watchdog(chassis_behavior_t behavior, const chassis_feedback_t *feedback)
{
    if (spin_cooldown_cycles > 0U)
    {
        spin_cooldown_cycles--;
        spin_stuck_cycles = 0U;
        return;
    }

    /* 没有角速度数据时不判定，避免姿态解算启动前误换向。 */
    const float spin_speed = (behavior == CHASSIS_BEHAVIOR_SPIN_FAST ? CHASSIS_SPIN_FAST_RATIO : CHASSIS_SPIN_SLOW_RATIO)
        * CHASSIS_MAX_ANGULAR_RAD_S;
    if ((behavior != CHASSIS_BEHAVIOR_SPIN_FAST && behavior != CHASSIS_BEHAVIOR_SPIN_SLOW)
        || !feedback->yaw_rate_valid
        || fabsf(feedback->yaw_rate_rad_s) >= CHASSIS_SPIN_STUCK_SPEED_RATIO * spin_speed)
    {
        spin_stuck_cycles = 0U;
        return;
    }

    if (++spin_stuck_cycles < spin_stuck_confirm_cycles)
    {
        return;
    }
    spin_forward = !spin_forward;
    spin_cooldown_cycles = spin_cooldown_reload_cycles;
    spin_stuck_cycles = 0U;
}

/* 行为目标：平移请求按行为确定坐标系并整体限幅，自转按行为生成。
 * 调用时 target 为模式选定来源的平移和自转请求，返回时为底盘坐标系下的速度目标。 */
static void resolve_target(chassis_behavior_t behavior, const chassis_command_t *command,
                           const chassis_feedback_t *feedback, chassis_velocity_t *target)
{
    if (behavior == CHASSIS_BEHAVIOR_CLIMB)
    {
        /* 爬坡速度以底盘正向为正向：上坡为正前进，下坡为负倒车。 */
        target->vx = command->climb_speed;
        target->vy = 0.0f;
    }
    else
    {
        /* 直控的平移请求已在底盘坐标系；其余行为在云台坐标系，按云台相对角转到底盘坐标系。 */
        if (behavior != CHASSIS_BEHAVIOR_DIRECT)
        {
            const float cos_yaw = cosf(feedback->gimbal_yaw_angle);
            const float sin_yaw = sinf(feedback->gimbal_yaw_angle);
            const float gimbal_vx = target->vx;
            target->vx = cos_yaw * gimbal_vx - sin_yaw * target->vy;
            target->vy = sin_yaw * gimbal_vx + cos_yaw * target->vy;
        }
        const float speed = hypotf(target->vx, target->vy);
        if (speed > CHASSIS_MAX_TRANSLATION_MPS)
        {
            target->vx *= CHASSIS_MAX_TRANSLATION_MPS / speed;
            target->vy *= CHASSIS_MAX_TRANSLATION_MPS / speed;
        }
    }

    switch (behavior)
    {
    case CHASSIS_BEHAVIOR_DIRECT:
        target->wz = float_constrain(target->wz, -CHASSIS_MAX_ANGULAR_RAD_S, CHASSIS_MAX_ANGULAR_RAD_S);
        break;

    case CHASSIS_BEHAVIOR_SPIN_SLOW:
        target->wz = CHASSIS_SPIN_SLOW_RATIO * (spin_forward ? CHASSIS_MAX_ANGULAR_RAD_S : -CHASSIS_MAX_ANGULAR_RAD_S);
        break;

    case CHASSIS_BEHAVIOR_SPIN_FAST:
        target->wz = CHASSIS_SPIN_FAST_RATIO * (spin_forward ? CHASSIS_MAX_ANGULAR_RAD_S : -CHASSIS_MAX_ANGULAR_RAD_S);
        break;

    case CHASSIS_BEHAVIOR_GIMBAL_FOLLOW:
    {
        /* 底盘相对云台目标朝向的无符号角误差，范围 0～2π。 */
        float yaw_error = feedback->gimbal_yaw_angle_error;
        if (yaw_error < 0.0f)
        {
            yaw_error += TWO_PI;
        }
        yaw_error += feedback->gimbal_yaw_angle;
        if (yaw_error >= TWO_PI)
        {
            yaw_error -= TWO_PI;
        }
        /* 正前方对齐按 2π 周期折算到半圈以内。 */
        yaw_error = fmodf(yaw_error, TWO_PI);
        if (yaw_error > TWO_PI / 2.0f)
        {
            yaw_error -= TWO_PI;
        }
        target->wz = pid_calculator_update(&follow_pid, yaw_error);
        break;
    }

    case CHASSIS_BEHAVIOR_CLIMB:
        /* 跟随爬坡朝向，角误差折算到 -π～π。 */
        target->wz = pid_calculator_update(&follow_pid,
                                           remainderf(command->climb_direction - command->climb_yaw, TWO_PI));
        break;
    }
}

/* 功率预算：裁判上限叠加 boost 后按电容电压折减，再加额外功率并按虚拟缓冲能量比例折减，
 * 返回本周期底盘可用功率 W。power_unlimited 为真时返回供电功率上限，虚拟缓冲能量仍照常积分，
 * 切回受约束模式时预算连续。 */
static float update_power_limit(bool power_unlimited, bool boost, const chassis_feedback_t *feedback)
{
    if (!feedback->referee_online)
    {
        /* 裁判离线时缓冲能量未知，使用固定保守上限，不叠加额外功率。 */
        virtual_buffer_j = CHASSIS_POWER_BUFFER_LIMIT_J;
        expected_power_w = CHASSIS_POWER_OFFLINE_LIMIT_W;
        return power_unlimited ? CHASSIS_SUPPLY_POWER_LIMIT_W : CHASSIS_POWER_OFFLINE_LIMIT_W;
    }

    /* 电容离线时没有实测功率，不积分，仅由裁判缓冲能量限制虚拟缓冲。 */
    if (feedback->supercap_online)
    {
        virtual_buffer_j += control_period_s * (expected_power_w - feedback->chassis_power_w);
    }
    virtual_buffer_j = float_constrain(virtual_buffer_j, 0.0f,
                                       fminf(feedback->buffer_energy_j, CHASSIS_POWER_BUFFER_LIMIT_J));

    /* boost：有请求且电容允许放电时，在裁判上限之上放开固定的额外功率，仍属受限。 */
    float power_limit_w = feedback->power_limit_w;
    if (boost && feedback->supercap_enabled)
    {
        power_limit_w += CHASSIS_POWER_BOOST_EXCESS_W;
    }
    expected_power_w = power_limit_w;

    /* 超出裁判上限的部分随电容电压从启动阈值到全开阈值线性放开，电压不足时回落到裁判上限。 */
    power_limit_w = feedback->power_limit_w + (power_limit_w - feedback->power_limit_w)
        * float_constrain((feedback->supercap_voltage_v - CHASSIS_SUPERCAP_BOOST_START_V)
                          / (CHASSIS_SUPERCAP_BOOST_FULL_V - CHASSIS_SUPERCAP_BOOST_START_V), 0.0f, 1.0f);

    return power_unlimited ? CHASSIS_SUPPLY_POWER_LIMIT_W
        : (power_limit_w + CHASSIS_POWER_EXCESS_W) * virtual_buffer_j / CHASSIS_POWER_BUFFER_LIMIT_J;
}
