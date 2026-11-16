#ifndef CALIB_TASK_H
#define CALIB_TASK_H

#include <stdbool.h>

#include "calibration.h"

void Calib_Task(void *argument);
/* 通知用户已创建的校准线程，不创建线程；仅由 INS 启动流程调用一次。 */
bool Calib_TaskStart(void);
bool Calib_IsReady(void);
bool Calib_IsFailed(void);
/* gyro、accel 分别至少有三个 float，temperature_c 指向一个 float；函数仅在调用期间写入，不保存指针。 */
void Calib_ReadImu(float *gyro, float *accel, float *temperature_c);
bool Calib_IsAccelReady(void);
/* bias 至少有三个 float，sigma 指向一个 float；函数仅在调用期间写入，不保存指针。 */
void Calib_GetBias(float *bias, float *sigma);

#endif
