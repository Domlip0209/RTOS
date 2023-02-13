/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
static volatile uint16_t delay = 1000;
static const uint8_t len = 0xFF;
static volatile char buff[len];
/* USER CODE END Variables */
osThreadId LEDHandle;
osThreadId UART_RxHandle;
osThreadId UART_TxHandle;
osMessageQId QueueUARTHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void blink(void const * argument);
void Rx(void const * argument);
void Tx(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

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
  /* definition and creation of QueueUART */
  osMessageQDef(QueueUART, 16, uint16_t);
  QueueUARTHandle = osMessageCreate(osMessageQ(QueueUART), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of LED */
  osThreadDef(LED, blink, osPriorityNormal, 0, 128);
  LEDHandle = osThreadCreate(osThread(LED), NULL);

  /* definition and creation of UART_Rx */
  osThreadDef(UART_Rx, Rx, osPriorityIdle, 0, 128);
  UART_RxHandle = osThreadCreate(osThread(UART_Rx), NULL);

  /* definition and creation of UART_Tx */
  osThreadDef(UART_Tx, Tx, osPriorityIdle, 0, 128);
  UART_TxHandle = osThreadCreate(osThread(UART_Tx), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_blink */
/**
  * @brief  Function implementing the LED thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_blink */
void blink(void const * argument)
{
  /* USER CODE BEGIN blink */
  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	  osDelay(delay);
  }
  /* USER CODE END blink */
}

/* USER CODE BEGIN Header_Rx */
/**
* @brief Function implementing the UART_Rx thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Rx */
void Rx(void const * argument)
{
  /* USER CODE BEGIN Rx */
	char c;
	uint8_t idx = 0;
	memset(buf, 0, len);
  /* Infinite loop */
  for(;;)
  {
	  //if something is on the UART RX you send it to Queue
	  if(serial2.available() > 0){
		  c = serial2.read();
		  if(idx< len-1){
			  buf[idx] = c;
			  idx++;
		  }
		  if (xQueueSend(QueueUARTHandle, (void *)&buff, 10) != pdTRUE) {
		              printf("ERROR: Could not put item on delay queue.");
		            }
		  xQueueSend(QueueUARTHandle, (void *)&buff,10);
	  memset(buf, 0, len);
	  }
	  else{
		  printf("%c", c);
	  }
    osDelay(1);
  }
  /* USER CODE END Rx */
}

/* USER CODE BEGIN Header_Tx */
/**
* @brief Function implementing the UART_Tx thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Tx */
void Tx(void const * argument)
{
  /* USER CODE BEGIN Tx */
  /* Infinite loop */
  for(;;)
  {
	  if(xQueueReceive(&QueueUARTHandle, (void *)&buff, 0) == pdTRUE){
		  delay = &buff;
		  printf("Delay wynosi aktualnie %d\n", delay);
	  }
	  //HAL_UART_Transmit(&huart2, (int*) &buff, Size, HAL_MAX_DELAY);
    osDelay(1);
  }
  /* USER CODE END Tx */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

