



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
_Static_assert(CHASSIS_LF_MOTOR_ID >= CHASSIS_MOTOR_GROUP_FIRST_ID && CHASSIS_LF_MOTOR_ID < CHASSIS_MOTOR_GROUP_FIRST_ID + 4U, "LF motor outside command group");
_Static_assert(CHASSIS_LB_MOTOR_ID >= CHASSIS_MOTOR_GROUP_FIRST_ID && CHASSIS_LB_MOTOR_ID < CHASSIS_MOTOR_GROUP_FIRST_ID + 4U, "LB motor outside command group");
_Static_assert(CHASSIS_RB_MOTOR_ID >= CHASSIS_MOTOR_GROUP_FIRST_ID && CHASSIS_RB_MOTOR_ID < CHASSIS_MOTOR_GROUP_FIRST_ID + 4U, "RB motor outside command group");
_Static_assert(CHASSIS_RF_MOTOR_ID >= CHASSIS_MOTOR_GROUP_FIRST_ID && CHASSIS_RF_MOTOR_ID < CHASSIS_MOTOR_GROUP_FIRST_ID + 4U, "RF motor outside command group");
_Static_assert(CHASSIS_RF_MOTOR_ID != CHASSIS_LF_MOTOR_ID && CHASSIS_RF_MOTOR_ID != CHASSIS_LB_MOTOR_ID
    && CHASSIS_RF_MOTOR_ID != CHASSIS_RB_MOTOR_ID && CHASSIS_LF_MOTOR_ID != CHASSIS_LB_MOTOR_ID
    && CHASSIS_LF_MOTOR_ID != CHASSIS_RB_MOTOR_ID && CHASSIS_LB_MOTOR_ID != CHASSIS_RB_MOTOR_ID, "Duplicate chassis motor IDs");

static const uint8_t motor_ids[CHASSIS_MOTOR_COUNT] = {
    CHASSIS_LF_MOTOR_ID, CHASSIS_LB_MOTOR_ID, CHASSIS_RB_MOTOR_ID, CHASSIS_RF_MOTOR_ID,
};
static const float motor_directions[CHASSIS_MOTOR_COUNT] = {
    CHASSIS_LF_MOTOR_DIRECTION, CHASSIS_LB_MOTOR_DIRECTION, CHASSIS_RB_MOTOR_DIRECTION, CHASSIS_RF_MOTOR_DIRECTION,
};
static RM3508_TypeDef motor_feedback[CHASSIS_MOTOR_COUNT];
chassis_motor_t chassis_motors[CHASSIS_MOTOR_COUNT];
/* 裁判电源管理 chassis 口输出状态，随反馈快照更新，作为电容指令的使能位。 */
static bool supercap_output_enabled;

static void chassis_on_can_frame(CAN_HandleTypeDef *handle, const user_can_frame_t *frame);

bool chassis_port_init(void)
{
    user_can_register_rx_callback(CHASSIS_CAN_HANDLE, chassis_on_can_frame);
    return user_can_start(CHASSIS_CAN_HANDLE);
}

bool chassis_port_send(float supercap_charge_power_limit_w)
{
    int16_t commands[CHASSIS_MOTOR_COUNT] = {0};

    /* 电容通讯未接线：指令先交给电容模块保存，接线后在这里用 super_capacitor_build_command 组帧发送。 */
    super_capacitor_set_command(supercap_charge_power_limit_w, supercap_output_enabled);

    for (uint32_t index = 0U; index < CHASSIS_MOTOR_COUNT; index++)
    {
        const float output = chassis_motor_is_online(&chassis_motors[index]) ? chassis_motors[index].output : 0.0f;
        commands[motor_ids[index] - CHASSIS_MOTOR_GROUP_FIRST_ID] = (int16_t)float_rounding(
            float_constrain(output * motor_directions[index], -CHASSIS_MOTOR_OUTPUT_LIMIT, CHASSIS_MOTOR_OUTPUT_LIMIT));
    }

    return MotorSend(CHASSIS_CAN_HANDLE, CHASSIS_MOTOR_COMMAND_ID, commands) == HAL_OK;
}

bool chassis_motor_is_online(const chassis_motor_t *motor)
{
    return motor->has_feedback && HAL_GetTick() - motor->feedback_tick <= CHASSIS_MOTOR_FEEDBACK_TIMEOUT_MS;
}

void chassis_port_read_feedback(uint32_t now_tick, chassis_feedback_t *feedback)
{
    gimbal_data_t gimbal_data;
    const bool gimbal_online = can_commun_read_gimbal_data(&gimbal_data);

    *feedback = (chassis_feedback_t){0};

    const int32_t previous_lock = osKernelLock();
    if (previous_lock < 0)
    {
        /* 未获得保护时不读取共享数据，按裁判、电容和姿态数据均无效处理。 */
        supercap_output_enabled = false;
        return;
    }

    feedback->referee_online = judge_data_is_fresh(&judge_data, JUDGE_DATA_ROBOT_STATUS, now_tick)
        && judge_data_is_fresh(&judge_data, JUDGE_DATA_POWER_HEAT, now_tick);
    feedback->power_limit_w = (float)judge_data.robot_status.chassis_power_limit;
    feedback->buffer_energy_j = (float)judge_data.power_heat.buffer_energy;
    supercap_output_enabled = feedback->referee_online && judge_data.robot_status.power_management_chassis_output != 0U;
    /* 2027 协议的 0x0202 无底盘实测功率，实测功率只取自电容。 */
    const super_capacitor_data_t *capacitor = super_capacitor_get_data();
    feedback->supercap_online = capacitor->online;
    feedback->supercap_enabled = capacitor->online && capacitor->enabled;
    feedback->chassis_power_w = capacitor->chassis_power_w;
    feedback->supercap_voltage_v = capacitor->supercap_voltage_v;
    feedback->yaw_rate_valid = usb_chassis_imu_valid;
    feedback->yaw_rate_rad_s = usb_chassis_wz_rad_s;
    feedback->gimbal_online = gimbal_online;
    if (gimbal_online)
    {
        feedback->gimbal_yaw_angle = fmodf(gimbal_data.yaw_angle_rad, 6.28318530718f);
        if (feedback->gimbal_yaw_angle < 0.0f)
        {
            feedback->gimbal_yaw_angle += 6.28318530718f;
        }
        feedback->gimbal_yaw_rate_rad_s = gimbal_data.yaw_rate_rad_s;
    }

    if (previous_lock == 0)
    {
        (void)osKernelRestoreLock(previous_lock);
    }
}

static void chassis_on_can_frame(CAN_HandleTypeDef *handle, const user_can_frame_t *frame)
{
    (void)handle;
    if (frame->extended || frame->length != 8U)
    {return;}

    for (uint32_t index = 0U; index < CHASSIS_MOTOR_COUNT; index++)
    {
        if (frame->id != CHASSIS_MOTOR_FEEDBACK_ID_BASE + motor_ids[index] - CHASSIS_MOTOR_GROUP_FIRST_ID)
        {continue;}

        RM3508_Receive(&motor_feedback[index], frame->data);
        chassis_motors[index].speed_rad_s = (float)motor_feedback[index].Speed * motor_directions[index]
            / CHASSIS_MOTOR_REDUCTION_RATIO * (3.14159265f / 30.0f);
        chassis_motors[index].feedback_tick = HAL_GetTick();
        chassis_motors[index].has_feedback = true;
        return;
    }
}
