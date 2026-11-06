/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "power_task.h"
#include "../../user/control/chassis/chassis_external/chassis.h"
#include "judge_task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osSemaphoreId_t JudgeSemHandle;
static const osSemaphoreAttr_t judge_semaphore_attributes = {
  .name = "JudgeSem",
};
static const osThreadAttr_t judge_task_attributes = {
  .name = "judge_task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* 底盘控制与功率守护任务不由 CubeMX 生成，在用户区创建 */
static const osThreadAttr_t chassis_task_attributes = {
  .name = "chassis_task",
  .stack_size = CHASSIS_TASK_STACK_SIZE_BYTES,
  .priority = (osPriority_t) CHASSIS_TASK_PRIORITY,
};
static const osThreadAttr_t power_task_attributes = {
  .name = "power_task",
  .stack_size = CHASSIS_POWER_TASK_STACK_SIZE_BYTES,
  .priority = (osPriority_t) CHASSIS_POWER_TASK_PRIORITY,
};
/* USER CODE END Variables */
/* Definitions for ins_task */
osThreadId_t ins_taskHandle;
const osThreadAttr_t ins_task_attributes = {
  .name = "ins_task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for remote_task */
osThreadId_t remote_taskHandle;
const osThreadAttr_t remote_task_attributes = {
  .name = "remote_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for watchdog_task */
osThreadId_t watchdog_taskHandle;
const osThreadAttr_t watchdog_task_attributes = {
  .name = "watchdog_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for RemoteSem */
osSemaphoreId_t RemoteSemHandle;
const osSemaphoreAttr_t RemoteSem_attributes = {
  .name = "RemoteSem"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void Chassis_Task(void *argument);
/* USER CODE END FunctionPrototypes */

void INS_Task(void *argument);
void Remote_Task(void *argument);
void Watchdog_Task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of RemoteSem */
  RemoteSemHandle = osSemaphoreNew(1, 0, &RemoteSem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  JudgeSemHandle = osSemaphoreNew(1, 0, &judge_semaphore_attributes);
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of ins_task */
  ins_taskHandle = osThreadNew(INS_Task, NULL, &ins_task_attributes);

  /* creation of remote_task */
  remote_taskHandle = osThreadNew(Remote_Task, NULL, &remote_task_attributes);

  /* creation of watchdog_task */
  watchdog_taskHandle = osThreadNew(Watchdog_Task, NULL, &watchdog_task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  (void)osThreadNew(Judge_Task, NULL, &judge_task_attributes);
  /* 创建任务前完成底盘功率模块初始化。 */
  chassis_power_init(osKernelGetTickCount(), osKernelGetTickFreq());
  (void)osThreadNew(Chassis_Task, NULL, &chassis_task_attributes);
  (void)osThreadNew(Power_Task, NULL, &power_task_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_INS_Task */
/**
  * @brief  Function implementing the ins_task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_INS_Task */
__weak void INS_Task(void *argument)
{
  /* USER CODE BEGIN INS_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END INS_Task */
}

/* USER CODE BEGIN Header_Remote_Task */
/**
* @brief Function implementing the remote_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Remote_Task */
__weak void Remote_Task(void *argument)
{
  /* USER CODE BEGIN Remote_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Remote_Task */
}

/* USER CODE BEGIN Header_Watchdog_Task */
/**
* @brief Function implementing the watchdog_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Watchdog_Task */
__weak void Watchdog_Task(void *argument)
{
  /* USER CODE BEGIN Watchdog_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Watchdog_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

