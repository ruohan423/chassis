#include "Task_Init.h"


void Start_Task (void)
{
	WatchDog_Init(&WatchDog.CMD_DATA_WatchDog, 20);
	WatchDog_Init(&WatchDog.POSTURE_DATA_WatchDog, 20);
	
	xTaskCreate(WatchDog_Task,  "WatchDog_Task",			256,	NULL, osPriorityNormal,			&WatchDog_Task_Handle);
	xTaskCreate(usb_task, 	     "usb_task",				1028,	NULL, osPriorityNormal, 		&usb_task_handle);
}