/* CubeMX 创建说明：仅上板电机调试时由用户创建，不用于常规底盘运行。
 * 名称 motor_test_task，入口 MotorTest_Task，优先级 osPriorityAboveNormal，栈 1024 字节。
 * 若界面以 32 位字填写栈大小，填 256；当前周期要求内核时钟为 1000 Hz。
 * 无独立信号量；入口初始化电机和 CAN，不再提供自动创建线程的启动函数。
 * 与底盘共用同一 CAN 接收分发或电机控制组时，禁止同时启用两个任务。
 */
#include "motor_test_task.h"

#include "can.h"
#include "cmsis_os2.h"
#include "motor.h"
#include "pid.h"
#include "user_can.h"

#define MOTOR_TEST_PERIOD_MS 2U        /* 电机上板调试控制周期，单位 ms。 */
#define MOTOR_TEST_RETRY_MS 100U       /* CAN 启动失败后的重试间隔，单位 ms。 */

static void receive_motor_feedback(CAN_HandleTypeDef *handle, const user_can_frame_t *frame);

volatile test_config_t motor_config = {
    .can_handle = &hcan1,
    .feedback_id = 0x201U,
    .command_id = 0x200U,
    .feedback_timeout_ms = 100U,
    .type = test_rm3508,
    .target_position = 0.0f,
    .position_kp = 50.0f,
    .position_ki = 0.0f,
    .position_kd = 12.0f,
    .position_integral_limit = 5000.0f,
    .sample_period_s = (float)MOTOR_TEST_PERIOD_MS * 0.001f,
    .output_limit = RM3508_LIMIT,
};

typedef struct
{
    float position;
    float speed;
    pid_dom_t position_pid;
    uint32_t feedback_count;
    uint32_t last_feedback_tick;
} motor_state_t;

static motor_state_t motor_state;
static RM3508_TypeDef rm3508_feedback;
static GM6020_TypeDef gm6020_feedback;
static M2006_TypeDef m2006_feedback;
static RM3510_TypeDef rm3510_feedback;
static GM3510_TypeDef gm3510_feedback;
static RM6623_TypeDef rm6623_feedback;
static DM4310_TypeDef dm4310_feedback;



void MotorTest_Task(void *argument)
{
    (void)argument;

    pid_dom_init(&motor_state.position_pid, motor_config.position_kp,
                 motor_config.position_ki, motor_config.position_kd,
                 motor_config.position_integral_limit, motor_config.sample_period_s);

    motor_state.position = 0.0f;
    motor_state.speed = 0.0f;
    motor_state.feedback_count = 0U;
    motor_state.last_feedback_tick = 0U;

    rm3508_feedback = (RM3508_TypeDef){0};
    gm6020_feedback = (GM6020_TypeDef){0};
    m2006_feedback = (M2006_TypeDef){0};
    rm3510_feedback = (RM3510_TypeDef){0};
    gm3510_feedback = (GM3510_TypeDef){0};
    rm6623_feedback = (RM6623_TypeDef){0};
    dm4310_feedback = (DM4310_TypeDef){0};

    user_can_register_rx_callback(motor_config.can_handle, receive_motor_feedback);

    while (!user_can_start(motor_config.can_handle))
    {
        osDelay(MOTOR_TEST_RETRY_MS);
    }
    uint32_t next_tick = osKernelGetTickCount();

    for (;;)
    {
        (void)MotorTest_Work();
        next_tick += MOTOR_TEST_PERIOD_MS;
        osDelayUntil(next_tick);
    }
}

HAL_StatusTypeDef MotorTest_Work(void)
{
    int16_t command[4] = {0};

    if (motor_state.feedback_count > 0U&& HAL_GetTick() - motor_state.last_feedback_tick<= motor_config.feedback_timeout_ms)
    {
        float current_reference = pid_dom_calculate(&motor_state.position_pid,motor_state.position,motor_config.target_position);
        current_reference = float_constrain(current_reference,-(float)motor_config.output_limit,(float)motor_config.output_limit);
        command[0] = (int16_t)float_rounding(current_reference);
    }

    return MotorSend(motor_config.can_handle, motor_config.command_id, command);
}

HAL_StatusTypeDef MotorTest_Stop(void)
{
    int16_t command[4] = {0};
    return MotorSend(motor_config.can_handle, motor_config.command_id, command);
}

static void receive_motor_feedback(CAN_HandleTypeDef *handle, const user_can_frame_t *frame)
{
    if (handle != motor_config.can_handle || frame->extended
        || frame->id != motor_config.feedback_id || frame->length < USER_CAN_DATA_LENGTH){return;}

    switch (motor_config.type)
    {
        case test_rm3508:
            RM3508_Receive(&rm3508_feedback, frame->data);
            motor_state.position = (float)rm3508_feedback.Angle;
            motor_state.speed = (float)rm3508_feedback.Speed;
            break;
        case test_gm6020:
            GM6020_Receive(&gm6020_feedback, frame->data);
            motor_state.position = (float)gm6020_feedback.Angle;
            motor_state.speed = (float)gm6020_feedback.Speed;
            break;
        case test_m2006:
            M2006_Receive(&m2006_feedback, frame->data);
            motor_state.position = (float)m2006_feedback.Angle;
            motor_state.speed = (float)m2006_feedback.Speed;
            break;
        case test_rm3510:
            RM3510_Receive(&rm3510_feedback, frame->data);
            motor_state.position = (float)rm3510_feedback.MchanicalAngle;
            motor_state.speed = (float)rm3510_feedback.Speed;
            break;
        case test_gm3510:
            GM3510_Receive(&gm3510_feedback, frame->data);
            motor_state.position = (float)gm3510_feedback.Angle;
            motor_state.speed = (float)gm3510_feedback.OutputTorque;
            break;
        case test_rm6623:
            RM6623_Receive(&rm6623_feedback, frame->data);
            motor_state.position = (float)rm6623_feedback.Angle;
            motor_state.speed = (float)rm6623_feedback.TorqueCurrent;
            break;
        case test_dm4310:
            DM4310_Receive(&dm4310_feedback, frame->data);
            motor_state.position = (float)dm4310_feedback.Angle;
            motor_state.speed = (float)dm4310_feedback.Speed;
            break;
        default:
            return;
    }
    motor_state.feedback_count++;
    motor_state.last_feedback_tick = HAL_GetTick();
}
