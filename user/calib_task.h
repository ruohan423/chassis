#ifndef CALIB_TASK_H
#define CALIB_TASK_H

#include <stdbool.h>

#include "calibration.h"

bool Calib_TaskStart(void);
bool Calib_IsReady(void);
bool Calib_IsFailed(void);
bool Calib_ReadImu(float gyro[CAL_AXES], float accel[CAL_AXES], float *temperature_c);
bool Calib_IsAccelReady(void);
bool Calib_GetBias(float bias[CAL_AXES], float *sigma);

#endif
