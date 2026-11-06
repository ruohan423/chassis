#ifndef SENTRY_CHASSIS_MOTOR_TEST_TASK_H
#define SENTRY_CHASSIS_MOTOR_TEST_TASK_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"
#include "user_can.h"

typedef enum
{
    test_rm3508 = 0,
    test_gm6020,
    test_m2006,
    test_rm3510,
    test_gm3510,
    test_rm6623,
    test_dm4310
} motor_type_t;

typedef struct
{
    CAN_HandleTypeDef *can_handle;
    uint32_t feedback_id;
    uint32_t command_id;
    uint32_t feedback_timeout_ms;
    motor_type_t type;
    float target_position;
    float position_kp;
    float position_ki;
    float position_kd;
    float position_integral_limit;
    float sample_period_s;
    int16_t output_limit;
} test_config_t;

extern volatile test_config_t motor_config;

bool MotorTest_TaskStart(void);
void MotorTest_Task(void *argument);
HAL_StatusTypeDef MotorTest_Work(void);
HAL_StatusTypeDef MotorTest_Stop(void);

#endif
