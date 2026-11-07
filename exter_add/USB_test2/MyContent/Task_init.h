#ifndef __TASK_INIT_H__
#define __TASK_INIT_H__

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "Usb.h"
#include "WatchDog_Task.h"
#include "cmsis_os.h"

void Start_Task (void);

#endif
