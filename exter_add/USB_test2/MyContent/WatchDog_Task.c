#include "WatchDog_Task.h"
#include "main.h"

WatchDog_t WatchDog;
Device_State_t Device_State;

TaskHandle_t WatchDog_Task_Handle;
void WatchDog_Task(void *pvParameters)
{
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	for(;;)
	{
		WatchDog_Polling();
		
		vTaskDelayUntil(&xLastWakeTime, 2);
	}
}

void FeedDog_CallBack(WatchDogp handle)
{	
/*******Communication*******/
	if(IS_Dog(handle, WatchDog.CMD_DATA_WatchDog))
	{
		Device_State.CMD_DATA_State = Device_OK;
	}
	if(IS_Dog(handle, WatchDog.POSTURE_DATA_WatchDog))
	{
		Device_State.POSTURE_DATA_State = Device_OK;
	}
	if(Device_State.CMD_DATA_State == Device_OK && Device_State.POSTURE_DATA_State == Device_OK)
	{
		HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
	}
}

void WatchDog_CallBack(WatchDogp handle)
{	
/*******Communication*******/
	if(IS_Dog(handle, WatchDog.CMD_DATA_WatchDog))
	{
		Device_State.CMD_DATA_State = Device_ERROR;
	}

	if(IS_Dog(handle, WatchDog.POSTURE_DATA_WatchDog))
	{
		Device_State.POSTURE_DATA_State = Device_ERROR;
	}
	
	if(Device_State.CMD_DATA_State == Device_ERROR || Device_State.POSTURE_DATA_State == Device_ERROR)
	{
		HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
	}
}


