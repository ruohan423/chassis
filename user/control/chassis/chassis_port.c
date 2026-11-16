



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
#error Unsupported chassis CAN bus
#endif

_Static_assert(CHASSIS_MOTOR_COUNT == 4U, "底盘控制器按四轮实现");
_Static_assert(CHASSIS_LF_MOTOR_ID >= CHASSIS_MOTOR_FIRST_ID && CHASSIS_LF_MOTOR_ID < CHASSIS_MOTOR_FIRST_ID + 4U, "LF motor outside command group");
_Static_assert(CHASSIS_LB_MOTOR_ID >= CHASSIS_MOTOR_FIRST_ID && CHASSIS_LB_MOTOR_ID < CHASSIS_MOTOR_FIRST_ID + 4U, "LB motor outside command group");
_Static_assert(CHASSIS_RB_MOTOR_ID >= CHASSIS_MOTOR_FIRST_ID && CHASSIS_RB_MOTOR_ID < CHASSIS_MOTOR_FIRST_ID + 4U, "RB motor outside command group");
_Static_assert(CHASSIS_RF_MOTOR_ID >= CHASSIS_MOTOR_FIRST_ID && CHASSIS_RF_MOTOR_ID < CHASSIS_MOTOR_FIRST_ID + 4U, "RF motor outside command group");
_Static_assert(CHASSIS_RF_MOTOR_ID != CHASSIS_LF_MOTOR_ID && CHASSIS_RF_MOTOR_ID != CHASSIS_LB_MOTOR_ID
    && CHASSIS_RF_MOTOR_ID != CHASSIS_RB_MOTOR_ID && CHASSIS_LF_MOTOR_ID != CHASSIS_LB_MOTOR_ID
    && CHASSIS_LF_MOTOR_ID != CHASSIS_RB_MOTOR_ID && CHASSIS_LB_MOTOR_ID != CHASSIS_RB_MOTOR_ID, "Duplicate chassis motor IDs");

static const uint8_t chassis_motor_ids[CHASSIS_MOTOR_COUNT] = {
    CHASSIS_LF_MOTOR_ID, CHASSIS_LB_MOTOR_ID, CHASSIS_RB_MOTOR_ID, CHASSIS_RF_MOTOR_ID,
};
static const float chassis_motor_directions[CHASSIS_MOTOR_COUNT] = {
    CHASSIS_LF_MOTOR_DIRECTION, CHASSIS_LB_MOTOR_DIRECTION, CHASSIS_RB_MOTOR_DIRECTION, CHASSIS_RF_MOTOR_DIRECTION,
};
static RM3508_TypeDef motor_feedback_data[CHASSIS_MOTOR_COUNT];
chassis_motor_t chassis_motors[CHASSIS_MOTOR_COUNT];
/* 裁判电源管理 chassis 口输出状态，随反馈快照更新，作为电容指令的使能位。 */
static bool supercap_output_enabled;

static void chassis_on_can_frame(CAN_HandleTypeDef *can_handle, const user_can_frame_t *can_frame);

bool chassis_port_init(void)
{
    user_can_register_rx_callback(CHASSIS_CAN_HANDLE, chassis_on_can_frame);
    return user_can_start(CHASSIS_CAN_HANDLE);
}

bool chassis_port_send(float supercap_charge_power_limit_w)
{
    int16_t motor_commands[CHASSIS_MOTOR_COUNT] = {0};

    /* 电容通讯未接线：指令先交给电容模块保存，接线后在这里用 super_capacitor_build_command 组帧发送。 */
    super_capacitor_set_command(supercap_charge_power_limit_w, supercap_output_enabled);

    for (uint32_t motor_index = 0U; motor_index < CHASSIS_MOTOR_COUNT; motor_index++)
    {
        const float motor_output = chassis_motor_is_online(&chassis_motors[motor_index])
            ? chassis_motors[motor_index].output : 0.0f;
        motor_commands[chassis_motor_ids[motor_index] - CHASSIS_MOTOR_FIRST_ID] = (int16_t)float_rounding(
            float_constrain(motor_output * chassis_motor_directions[motor_index],
                            -CHASSIS_MOTOR_OUTPUT_LIMIT, CHASSIS_MOTOR_OUTPUT_LIMIT));
    }

    return MotorSend(CHASSIS_CAN_HANDLE, CHASSIS_MOTOR_COMMAND_ID, motor_commands) == HAL_OK;
}

bool chassis_motor_is_online(const chassis_motor_t *chassis_motor)
{
    return chassis_motor->has_feedback
        && HAL_GetTick() - chassis_motor->feedback_tick <= CHASSIS_MOTOR_FEEDBACK_TIMEOUT_MS;
}

void chassis_port_read_feedback(uint32_t current_tick, chassis_feedback_t *chassis_feedback)
{
    static gimbal_data_t gimbal_feedback;
    const bool gimbal_feedback_valid = can_commun_read_gimbal_data(&gimbal_feedback);

    *chassis_feedback = (chassis_feedback_t){0};

    const int32_t previous_kernel_lock = osKernelLock();
    if (previous_kernel_lock < 0)
    {
        /* 未获得保护时不读取共享数据，按裁判、电容和姿态数据均无效处理。 */
        supercap_output_enabled = false;
        return;
    }

    chassis_feedback->referee_online = judge_data_is_fresh(&judge_data, JUDGE_DATA_ROBOT_STATUS, current_tick)
        && judge_data_is_fresh(&judge_data, JUDGE_DATA_POWER_HEAT, current_tick);
    chassis_feedback->power_limit_w = (float)judge_data.robot_status.chassis_power_limit;
    chassis_feedback->buffer_energy_j = (float)judge_data.power_heat.buffer_energy;
    supercap_output_enabled = chassis_feedback->referee_online
        && judge_data.robot_status.power_management_chassis_output != 0U;
    /* 2027 协议的 0x0202 无底盘实测功率，实测功率只取自电容。 */
    const super_capacitor_data_t *supercap_data = super_capacitor_get_data();
    chassis_feedback->supercap_online = supercap_data->online;
    chassis_feedback->supercap_enabled = supercap_data->online && supercap_data->enabled;
    chassis_feedback->chassis_power_w = supercap_data->chassis_power_w;
    chassis_feedback->supercap_voltage_v = supercap_data->supercap_voltage_v;
    chassis_feedback->yaw_rate_valid = usb_chassis_imu_valid;
    chassis_feedback->yaw_rate_rad_s = usb_chassis_wz_rad_s;
    chassis_feedback->gimbal_online = gimbal_feedback_valid;
    if (gimbal_feedback_valid)
    {
        chassis_feedback->gimbal_yaw_angle = fmodf(gimbal_feedback.yaw_angle_rad, 6.28318530718f);
        if (chassis_feedback->gimbal_yaw_angle < 0.0f)
        {
            chassis_feedback->gimbal_yaw_angle += 6.28318530718f;
        }
        chassis_feedback->gimbal_yaw_rate_rad_s = gimbal_feedback.yaw_rate_rad_s;
    }

    if (previous_kernel_lock == 0)
    {
        (void)osKernelRestoreLock(previous_kernel_lock);
    }
}

static void chassis_on_can_frame(CAN_HandleTypeDef *can_handle, const user_can_frame_t *can_frame)
{
    (void)can_handle;
    if (can_frame->extended || can_frame->length != 8U)
    {
        return;
    }

    for (uint32_t motor_index = 0U; motor_index < CHASSIS_MOTOR_COUNT; motor_index++)
    {
        if (can_frame->id != CHASSIS_MOTOR_FEEDBACK_ID_BASE + chassis_motor_ids[motor_index]
            - CHASSIS_MOTOR_FIRST_ID)
        {
            continue;
        }

        RM3508_Receive(&motor_feedback_data[motor_index], can_frame->data);
        chassis_motors[motor_index].speed_rad_s = (float)motor_feedback_data[motor_index].Speed
            * chassis_motor_directions[motor_index]
            / CHASSIS_MOTOR_REDUCTION_RATIO * (3.14159265f / 30.0f);
        chassis_motors[motor_index].feedback_tick = HAL_GetTick();
        chassis_motors[motor_index].has_feedback = true;
        return;
    }
}
