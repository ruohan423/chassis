#ifndef CALIB_TASK_H
#define CALIB_TASK_H

#include <stdbool.h>

#include "calibration.h"

void Calib_Task(void *argument);
/* 通知用户已创建的校准线程，不创建线程；仅由 INS 启动流程调用一次。 */
bool Calib_TaskStart(void);
bool Calib_IsReady(void);
bool Calib_IsFailed(void);
void Calib_ReadImu(float gyro[CAL_AXES], float accel[CAL_AXES], float *temperature_c);
bool Calib_IsAccelReady(void);
void Calib_GetBias(float bias[CAL_AXES], float *sigma);

#endif
