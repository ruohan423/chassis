#ifndef __WATCHDOG_TASK_H__
#define __WATCHDOG_TASK_H__

#include "Task_Init.h"
#include "WatchDog.h"
#include "FreeRTOS.h"
#include "task.h"

typedef enum{
	Device_OK,
	Device_ERROR,
}Device_State_e;

typedef struct{
	Device_State_e CMD_DATA_State;
	Device_State_e POSTURE_DATA_State;
}Device_State_t;
extern Device_State_t Device_State;

typedef struct{
	WatchDog_TypeDef CMD_DATA_WatchDog;
	WatchDog_TypeDef POSTURE_DATA_WatchDog;

}WatchDog_t;
extern WatchDog_t WatchDog;

extern TaskHandle_t WatchDog_Task_Handle;

void WatchDog_Task(void *pvParameters);
void FeedDog_CallBack(WatchDogp handle);
void WatchDog_CallBack(WatchDogp handle);

#endif

