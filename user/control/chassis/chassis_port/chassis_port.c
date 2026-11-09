#include "chassis_port.h"

#include "can.h"
#include "motor.h"
#include "pid.h"
#include "user_can.h"

#if USE_POWER_CONTROLLER
#include "judge.h"
#include "super_capacitor.h"
#endif

#if CHASSIS_CAN_BUS == 1U
#define CHASSIS_CAN_HANDLE (&hcan1)
#elif CHASSIS_CAN_BUS == 2U
#define CHASSIS_CAN_HANDLE (&hcan2)
#else
#error Unsupported chassis CAN bus
#endif

_Static_assert(CHASSIS_MOTOR_COUNT == 4U, "Chassis controller requires four wheels");
_Static_assert(CHASSIS_RF_MOTOR_ID >= CHASSIS_MOTOR_GROUP_FIRST_ID && CHASSIS_RF_MOTOR_ID < CHASSIS_MOTOR_GROUP_FIRST_ID + 4U, "RF motor outside command group");
_Static_assert(CHASSIS_LF_MOTOR_ID >= CHASSIS_MOTOR_GROUP_FIRST_ID && CHASSIS_LF_MOTOR_ID < CHASSIS_MOTOR_GROUP_FIRST_ID + 4U, "LF motor outside command group");
_Static_assert(CHASSIS_LB_MOTOR_ID >= CHASSIS_MOTOR_GROUP_FIRST_ID && CHASSIS_LB_MOTOR_ID < CHASSIS_MOTOR_GROUP_FIRST_ID + 4U, "LB motor outside command group");
_Static_assert(CHASSIS_RB_MOTOR_ID >= CHASSIS_MOTOR_GROUP_FIRST_ID && CHASSIS_RB_MOTOR_ID < CHASSIS_MOTOR_GROUP_FIRST_ID + 4U, "RB motor outside command group");
_Static_assert(CHASSIS_RF_MOTOR_ID != CHASSIS_LF_MOTOR_ID && CHASSIS_RF_MOTOR_ID != CHASSIS_LB_MOTOR_ID
    && CHASSIS_RF_MOTOR_ID != CHASSIS_RB_MOTOR_ID && CHASSIS_LF_MOTOR_ID != CHASSIS_LB_MOTOR_ID
    && CHASSIS_LF_MOTOR_ID != CHASSIS_RB_MOTOR_ID && CHASSIS_LB_MOTOR_ID != CHASSIS_RB_MOTOR_ID, "Duplicate chassis motor IDs");

static const uint8_t motor_ids[CHASSIS_MOTOR_COUNT] = {
    CHASSIS_RF_MOTOR_ID, CHASSIS_LF_MOTOR_ID, CHASSIS_LB_MOTOR_ID, CHASSIS_RB_MOTOR_ID,
};
static const float motor_directions[CHASSIS_MOTOR_COUNT] = {
    CHASSIS_RF_MOTOR_DIRECTION, CHASSIS_LF_MOTOR_DIRECTION, CHASSIS_LB_MOTOR_DIRECTION, CHASSIS_RB_MOTOR_DIRECTION,
};
static RM3508_TypeDef motor_feedback[CHASSIS_MOTOR_COUNT];
chassis_motor_t chassis_motors[CHASSIS_MOTOR_COUNT];

static void chassis_on_can_frame(CAN_HandleTypeDef *handle, const user_can_frame_t *frame);

#if USE_POWER_CONTROLLER
static RefereePowerHeatMessageData power_message;
static RefereeRobotStatusMessageData robot_message;
static CapacitorStatus capacitor_status;
static bool referee_connected;

const PowerFeedbackSources chassis_power_sources = {
    .powerMessage = &power_message,
    .robotMessage = &robot_message,
    .capStatus = &capacitor_status,
    .referee_connected = &referee_connected,
};
#endif

bool chassis_port_init(void)
{
    user_can_register_rx_callback(CHASSIS_CAN_HANDLE, chassis_on_can_frame);
    return user_can_start(CHASSIS_CAN_HANDLE);
}

bool chassis_port_send(void)
{
    int16_t commands[CHASSIS_MOTOR_COUNT] = {0};

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

static void chassis_on_can_frame(CAN_HandleTypeDef *handle, const user_can_frame_t *frame)
{
    (void)handle;
    if (frame->extended || frame->length != 8U)
    {
        return;
    }

    for (uint32_t index = 0U; index < CHASSIS_MOTOR_COUNT; index++)
    {
        if (frame->id != CHASSIS_MOTOR_FEEDBACK_ID_BASE + motor_ids[index] - CHASSIS_MOTOR_GROUP_FIRST_ID)
        {
            continue;
        }

        RM3508_Receive(&motor_feedback[index], frame->data);
        chassis_motors[index].speed_rad_s = (float)motor_feedback[index].Speed * motor_directions[index]
            / CHASSIS_MOTOR_REDUCTION_RATIO * (3.14159265f / 30.0f);
        chassis_motors[index].torque_nm = (float)motor_feedback[index].TorqueCurrent * motor_directions[index]
            * CHASSIS_MOTOR_CURRENT_PER_RAW * CHASSIS_MOTOR_KA_NM_PER_A * CHASSIS_MOTOR_REDUCTION_RATIO;
        chassis_motors[index].feedback_tick = HAL_GetTick();
        chassis_motors[index].has_feedback = true;
        return;
    }
}

#if USE_POWER_CONTROLLER
void chassis_port_update_feedback(uint32_t now_tick)
{
    const bool robot_online = judge_data_is_fresh(&judge_data, JUDGE_DATA_ROBOT_STATUS, now_tick);
    const bool heat_online = judge_data_is_fresh(&judge_data, JUDGE_DATA_POWER_HEAT, now_tick);
    const super_capacitor_data_t *capacitor = super_capacitor_get_data();

    robot_message.chassis_power_limit = robot_online ? judge_data.robot_status.chassis_power_limit : 0U;
    robot_message.robot_level = robot_online ? judge_data.robot_status.robot_level : 0U;
    referee_connected = robot_online && heat_online;
    power_message.bufferEnergy = heat_online ? judge_data.power_heat.buffer_energy : 0U;
    /* 2027 协议的 0x0202 无底盘实测功率，不能把保留字段或占位零值当成测量。 */
    power_message.chassisPower = 0.0f;
    power_message.chassisPowerValid = 0U;

    capacitor_status.isConnected = capacitor->online ? 1U : 0U;
    capacitor_status.capacitorTx.enableDCDC = capacitor->dcdc_enabled;
    capacitor_status.capacitorRx.errorCode = capacitor->error_code;
    capacitor_status.capacitorRx.capEnergy = capacitor->energy;
    capacitor_status.capacitorRx.chassisPower = capacitor->chassis_power;
}
#endif
