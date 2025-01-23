/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "usart.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
	uint8_t received_id;
/* USER CODE END Variables */
/* Definitions for Task01 */
osThreadId_t Task01Handle;
const osThreadAttr_t Task01_attributes = {
  .name = "Task01",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Task02 */
osThreadId_t Task02Handle;
const osThreadAttr_t Task02_attributes = {
  .name = "Task02",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task03 */
osThreadId_t Task03Handle;
const osThreadAttr_t Task03_attributes = {
  .name = "Task03",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Hangdoi_ngat */
osMessageQueueId_t Hangdoi_ngatHandle;
const osMessageQueueAttr_t Hangdoi_ngat_attributes = {
  .name = "Hangdoi_ngat"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Task01_Function(void *argument);
void Task02_Function(void *argument);
void Task03_Function(void *argument);

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

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of Hangdoi_ngat */
  Hangdoi_ngatHandle = osMessageQueueNew (16, sizeof(uint8_t), &Hangdoi_ngat_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Task01 */
  Task01Handle = osThreadNew(Task01_Function, NULL, &Task01_attributes);

  /* creation of Task02 */
  Task02Handle = osThreadNew(Task02_Function, NULL, &Task02_attributes);

  /* creation of Task03 */
  Task03Handle = osThreadNew(Task03_Function, NULL, &Task03_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_Task01_Function */
/**
  * @brief  Function implementing the Task01 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Task01_Function */
void Task01_Function(void *argument)
{
	 // Biến lưu trữ ID được nhận từ hàng đợi
	    for (;;) {
	        // Chờ ID từ hàng đợi (chờ mãi cho đến khi có dữ liệu)
	    		osMessageQueueGet(Hangdoi_ngatHandle, &received_id, NULL, osWaitForever);

	            // Xử lý ID nhận được
	            switch (received_id) {
	                case 1: // Task xử lý cho ID1
	                	const char* start_task_ID1="Task interrupt 01 is starting.............\r\n";
	                	const char* process_task_ID1="Task interrupt 01 is processing.............\r\n";
	                	const char* end_task_ID1="Task interrupt 01 is Ending.............\r\n\n";
	                	HAL_UART_Transmit(&huart1, (uint8_t*)start_task_ID1, strlen(start_task_ID1), HAL_MAX_DELAY);
	                	HAL_UART_Transmit(&huart1, (uint8_t*)process_task_ID1, strlen(process_task_ID1), HAL_MAX_DELAY);
	                	HAL_Delay(3000);
	                	HAL_UART_Transmit(&huart1, (uint8_t*)end_task_ID1, strlen(end_task_ID1), HAL_MAX_DELAY);
	                    break;
	                case 2: // Task xử lý cho ID2
	                	const char* start_task_ID2="Task interrupt 02 is starting.............\r\n";
	                	const char* process_task_ID2="Task interrupt 02 is processing.............\r\n";
	                	const char* end_task_ID2="Task interrupt 02 is Ending.............\r\n\n";
	                	HAL_UART_Transmit(&huart1, (uint8_t*)start_task_ID2, strlen(start_task_ID2), HAL_MAX_DELAY);
	                	HAL_UART_Transmit(&huart1, (uint8_t*)process_task_ID2, strlen(process_task_ID2), HAL_MAX_DELAY);
	                	HAL_Delay(1000);
	                	HAL_UART_Transmit(&huart1, (uint8_t*)end_task_ID2, strlen(end_task_ID2), HAL_MAX_DELAY);
	                	break;
	                case 3: // Task xử lý cho ID3
	                	const char* start_task_ID3="Task interrupt 03 is starting.............\r\n";
	                	const char* process_task_ID3="Task interrupt 03 is processing.............\r\n";
	                	const char* end_task_ID3="Task interrupt 03 is Ending.............\r\n\n";
	                	HAL_UART_Transmit(&huart1, (uint8_t*)start_task_ID3, strlen(start_task_ID3), HAL_MAX_DELAY);
	                	HAL_UART_Transmit(&huart1, (uint8_t*)process_task_ID3, strlen(process_task_ID3), HAL_MAX_DELAY);
	                	HAL_Delay(1000);
	                	HAL_UART_Transmit(&huart1, (uint8_t*)end_task_ID3, strlen(end_task_ID3), HAL_MAX_DELAY);
	                	break;
	            }
	        }
  /* USER CODE END Task01_Function */
}

/* USER CODE BEGIN Header_Task02_Function */
/**
* @brief Function implementing the Task02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task02_Function */
void Task02_Function(void *argument)
{
  /* USER CODE BEGIN Task02_Function */
	const char* start_task2="High_Task is starting...\r\n";
	const char* end_task2="High_Task is ending...\r\n\n";
  /* Infinite loop */
  for(;;)
  {
	  HAL_UART_Transmit(&huart1, (uint8_t*)start_task2, strlen(start_task2), HAL_MAX_DELAY);
	  HAL_UART_Transmit(&huart1, (uint8_t*)end_task2, strlen(end_task2), HAL_MAX_DELAY);
	  osDelay(5000);
  }
  /* USER CODE END Task02_Function */
}

/* USER CODE BEGIN Header_Task03_Function */
/**
* @brief Function implementing the Task03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task03_Function */
void Task03_Function(void *argument)
{
  /* USER CODE BEGIN Task03_Function */
	const char* start_task3="Low_Task is starting...\r\n";
	const char* end_task3="Low_Task is ending...\r\n\n";
  /* Infinite loop */
  for(;;)
  {
	HAL_UART_Transmit(&huart1, (uint8_t*)start_task3, strlen(start_task3), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart1, (uint8_t*)end_task3, strlen(end_task3), HAL_MAX_DELAY);
	osDelay(3000);
  }

  /* USER CODE END Task03_Function */
}


/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

