



/* 2027 协议的 0x0202 无底盘实测功率，不能把保留字段或占位零值当成测量。 */
//power_message.chassisPower = 0.0f;
//power_message.chassisPowerValid = 0U;
#include "chassis_port.h"

#include "cmsis_os2.h"

#include "can.h"
#include "can_commun.h"
#include "judge.h"
#include "motor.h"
#include "pid.h"
#include "super_capacitor.h"
#include "usb_commun.h"
#include "user_can.h"

#include <math.h>

#if CHASSIS_CAN_BUS == 1U
#define CHASSIS_CAN_HANDLE (&hcan1)
#elif CHASSIS_CAN_BUS == 2U
#define CHASSIS_CAN_HANDLE (&hcan2)
#else
#error 底盘电机总线只支持 CAN1 或 CAN2
#endif

#define TWO_PI 6.28318530718f

_Static_assert(CHASSIS_MOTOR_COUNT == 4U, "底盘控制器按四轮实现");
_Static_assert(CHASSIS_LF_MOTOR_ID >= CHASSIS_MOTOR_FIRST_ID && CHASSIS_LF_MOTOR_ID < CHASSIS_MOTOR_FIRST_ID + 4U, "左前轮电机编号不在控制组内");
_Static_assert(CHASSIS_LB_MOTOR_ID >= CHASSIS_MOTOR_FIRST_ID && CHASSIS_LB_MOTOR_ID < CHASSIS_MOTOR_FIRST_ID + 4U, "左后轮电机编号不在控制组内");
_Static_assert(CHASSIS_RB_MOTOR_ID >= CHASSIS_MOTOR_FIRST_ID && CHASSIS_RB_MOTOR_ID < CHASSIS_MOTOR_FIRST_ID + 4U, "右后轮电机编号不在控制组内");
_Static_assert(CHASSIS_RF_MOTOR_ID >= CHASSIS_MOTOR_FIRST_ID && CHASSIS_RF_MOTOR_ID < CHASSIS_MOTOR_FIRST_ID + 4U, "右前轮电机编号不在控制组内");
_Static_assert(CHASSIS_RF_MOTOR_ID != CHASSIS_LF_MOTOR_ID && CHASSIS_RF_MOTOR_ID != CHASSIS_LB_MOTOR_ID
    && CHASSIS_RF_MOTOR_ID != CHASSIS_RB_MOTOR_ID && CHASSIS_LF_MOTOR_ID != CHASSIS_LB_MOTOR_ID
    && CHASSIS_LF_MOTOR_ID != CHASSIS_RB_MOTOR_ID && CHASSIS_LB_MOTOR_ID != CHASSIS_RB_MOTOR_ID, "底盘电机编号重复");

static const uint8_t motor_ids[CHASSIS_MOTOR_COUNT] = {
    CHASSIS_LF_MOTOR_ID, CHASSIS_LB_MOTOR_ID, CHASSIS_RB_MOTOR_ID, CHASSIS_RF_MOTOR_ID,
};
static const float motor_directions[CHASSIS_MOTOR_COUNT] = {
    CHASSIS_LF_MOTOR_DIRECTION, CHASSIS_LB_MOTOR_DIRECTION, CHASSIS_RB_MOTOR_DIRECTION, CHASSIS_RF_MOTOR_DIRECTION,
};
static RM3508_TypeDef motor_raw[CHASSIS_MOTOR_COUNT];
chassis_motor_t chassis_motors[CHASSIS_MOTOR_COUNT];
/* 上电使能帧是否已成功发出；发送成功后切换为常规指令帧。 */
static bool supercap_enable_sent;

static void on_can_frame(CAN_HandleTypeDef *can_handle, const user_can_frame_t *frame);

bool chassis_port_init(void)
{
    user_can_register_rx_callback(CHASSIS_CAN_HANDLE, on_can_frame);
    return user_can_start(CHASSIS_CAN_HANDLE);
}

bool chassis_port_send(void)
{
    int16_t commands[CHASSIS_MOTOR_COUNT] = {0};

    for (uint32_t i = 0U; i < CHASSIS_MOTOR_COUNT; i++)
    {
        const float output = chassis_motor_is_online(&chassis_motors[i]) ? chassis_motors[i].output : 0.0f;
        commands[motor_ids[i] - CHASSIS_MOTOR_FIRST_ID] = (int16_t)float_rounding(
            float_constrain(output * motor_directions[i], -CHASSIS_MOTOR_OUTPUT_LIMIT, CHASSIS_MOTOR_OUTPUT_LIMIT));
    }

    /* bus-off 期间控制器不收不发，按发送失败上报，任务延时后经 chassis_port_init 重启总线。 */
    return user_can_is_online(CHASSIS_CAN_HANDLE)
        && MotorSend(CHASSIS_CAN_HANDLE, CHASSIS_MOTOR_COMMAND_ID, commands) == HAL_OK;
}

void chassis_port_send_supercap(uint16_t power_limit_w, uint16_t buffer_energy_j, bool enable_dcdc)
{
    uint8_t frame[SUPER_CAPACITOR_COMMAND_LENGTH];

    if (!supercap_enable_sent && enable_dcdc)
    {
        /* 裁判允许 chassis 输出后才发使能加重启帧；发送成功后切到常规帧。 */
        super_capacitor_build_enable_command(frame);
        supercap_enable_sent = user_can_send(&hcan1, SUPER_CAPACITOR_TX_ID, frame, SUPER_CAPACITOR_COMMAND_LENGTH) == HAL_OK;
        return;
    }

    super_capacitor_build_command(frame, power_limit_w, buffer_energy_j, enable_dcdc);
    (void)user_can_send(&hcan1, SUPER_CAPACITOR_TX_ID, frame, SUPER_CAPACITOR_COMMAND_LENGTH);
}

bool chassis_motor_is_online(const chassis_motor_t *motor)
{
    /* 先固定中断写入的时间戳再取当前时刻，避免接收中断插在两次读取之间使差值回绕而误判离线。 */
    const uint32_t feedback_tick = motor->feedback_tick;
    return motor->has_feedback && HAL_GetTick() - feedback_tick <= CHASSIS_MOTOR_FEEDBACK_TIMEOUT_MS;
}

void chassis_port_read_feedback(uint32_t tick, chassis_feedback_t *feedback)
{
    static gimbal_data_t gimbal;
    const bool gimbal_online = can_commun_read_gimbal_data(&gimbal);
    super_capacitor_data_t supercap;
    super_capacitor_get_data(&supercap);

    *feedback = (chassis_feedback_t){0};

    const int32_t lock_state = osKernelLock();
    if (lock_state < 0)
    {
        /* 未获得保护时不读取共享数据，按裁判和姿态数据均无效处理；电容快照已取，随反馈一起清零。 */
        return;
    }

    feedback->referee_online = judge_data_is_fresh(&judge_data, JUDGE_DATA_ROBOT_STATUS, tick)
        && judge_data_is_fresh(&judge_data, JUDGE_DATA_POWER_HEAT, tick);
    feedback->supercap_output_enabled = feedback->referee_online
        && judge_data.robot_status.power_management_chassis_output != 0U;
    feedback->power_limit_w = (float)judge_data.robot_status.chassis_power_limit;
    feedback->buffer_energy_j = (float)judge_data.power_heat.buffer_energy;
    /* 2027 协议的 0x0202 无底盘实测功率，实测功率只取自电容。 */
    feedback->supercap_online = supercap.online;
    feedback->supercap_enabled = supercap.online && !supercap.error;
    feedback->chassis_power_w = supercap.chassis_power_w;
    feedback->cap_energy = supercap.cap_energy;
    feedback->imu_valid = usb_chassis_imu_valid;
    feedback->chassis_yaw = usb_chassis_yaw_rad;
    feedback->yaw_rate_rad_s = usb_chassis_wz_rad_s;
    feedback->gimbal_online = gimbal_online;
    if (gimbal_online)
    {
        feedback->gimbal_yaw_angle = fmodf(gimbal.yaw_angle_rad, TWO_PI);
        if (feedback->gimbal_yaw_angle < 0.0f)
        {
            feedback->gimbal_yaw_angle += TWO_PI;
        }
    }

    if (lock_state == 0)
    {
        (void)osKernelRestoreLock(lock_state);
    }
}

static void on_can_frame(CAN_HandleTypeDef *can_handle, const user_can_frame_t *frame)
{
    (void)can_handle;
    if (frame->extended || frame->length != 8U)
    {
        return;
    }

    for (uint32_t i = 0U; i < CHASSIS_MOTOR_COUNT; i++)
    {
        if (frame->id != CHASSIS_MOTOR_FEEDBACK_ID_BASE + motor_ids[i] - CHASSIS_MOTOR_FIRST_ID)
        {
            continue;
        }

        RM3508_Receive(&motor_raw[i], frame->data);
        chassis_motors[i].speed_rad_s = (float)motor_raw[i].Speed * motor_directions[i]
            / CHASSIS_MOTOR_REDUCTION_RATIO * (3.14159265f / 30.0f);
        chassis_motors[i].temperature_c = motor_raw[i].temp;
        chassis_motors[i].feedback_tick = HAL_GetTick();
        chassis_motors[i].has_feedback = true;
        return;
    }
}
