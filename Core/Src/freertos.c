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

/* USER CODE END Variables */
/* Definitions for UART_rx */
osThreadId_t UART_rxHandle;
const osThreadAttr_t UART_rx_attributes = {
  .name = "UART_rx",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for blink */
osThreadId_t blinkHandle;
const osThreadAttr_t blink_attributes = {
  .name = "blink",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for EEPROM */
osThreadId_t EEPROMHandle;
const osThreadAttr_t EEPROM_attributes = {
  .name = "EEPROM",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void RX(void *argument);
void led_blk(void *argument);
void R_W(void *argument);

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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of UART_rx */
  UART_rxHandle = osThreadNew(RX, NULL, &UART_rx_attributes);

  /* creation of blink */
  blinkHandle = osThreadNew(led_blk, NULL, &blink_attributes);

  /* creation of EEPROM */
  EEPROMHandle = osThreadNew(R_W, NULL, &EEPROM_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_RX */
/**
  * @brief  Function implementing the UART_rx thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_RX */
void RX(void *argument)
{
  /* USER CODE BEGIN RX */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END RX */
}

/* USER CODE BEGIN Header_led_blk */
/**
* @brief Function implementing the blink thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_led_blk */
void led_blk(void *argument)
{
  /* USER CODE BEGIN led_blk */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END led_blk */
}

/* USER CODE BEGIN Header_R_W */
/**
* @brief Function implementing the EEPROM thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_R_W */
void R_W(void *argument)
{
  /* USER CODE BEGIN R_W */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END R_W */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

