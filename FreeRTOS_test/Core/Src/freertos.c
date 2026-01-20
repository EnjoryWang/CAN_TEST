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
#include "common_def.h"
#include "led_control.h"
#include "can_process.h"
#include "uart_process.h"
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
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CAN */
osThreadId_t CANHandle;
const osThreadAttr_t CAN_attributes = {
  .name = "CAN",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task_Breathing_ */
osThreadId_t Task_Breathing_Handle;
const osThreadAttr_t Task_Breathing__attributes = {
  .name = "Task_Breathing_",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for TASK_UART_Proce */
osThreadId_t TASK_UART_ProceHandle;
const osThreadAttr_t TASK_UART_Proce_attributes = {
  .name = "TASK_UART_Proce",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Queue_CAN_Data */
osMessageQueueId_t Queue_CAN_DataHandle;
const osMessageQueueAttr_t Queue_CAN_Data_attributes = {
  .name = "Queue_CAN_Data"
};
/* Definitions for Queue_UART_Data */
osMessageQueueId_t Queue_UART_DataHandle;
const osMessageQueueAttr_t Queue_UART_Data_attributes = {
  .name = "Queue_UART_Data"
};
/* Definitions for Queue_CAN_Process */
osMessageQueueId_t Queue_CAN_ProcessHandle;
const osMessageQueueAttr_t Queue_CAN_Process_attributes = {
  .name = " Queue_CAN_Process"
};
/* Definitions for BinarySem_LED_Freq */
osSemaphoreId_t BinarySem_LED_FreqHandle;
const osSemaphoreAttr_t BinarySem_LED_Freq_attributes = {
  .name = "BinarySem_LED_Freq"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask_CAN_Process(void *argument);
void StartTask_BreathingLED(void *argument);
void StartTask_USART_Process(void *argument);

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
  /* creation of BinarySem_LED_Freq */
  BinarySem_LED_FreqHandle = osSemaphoreNew(1, 1, &BinarySem_LED_Freq_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of Queue_CAN_Data */
  Queue_CAN_DataHandle = osMessageQueueNew (6, sizeof(uint16_t), &Queue_CAN_Data_attributes);

  /* creation of Queue_UART_Data */
  Queue_UART_DataHandle = osMessageQueueNew (3, sizeof(uint16_t), &Queue_UART_Data_attributes);

  /* creation of Queue_CAN_Process */
   Queue_CAN_ProcessHandle = osMessageQueueNew (6, sizeof(uint16_t), & Queue_CAN_Process_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of CAN */
  CANHandle = osThreadNew(StartTask_CAN_Process, NULL, &CAN_attributes);

  /* creation of Task_Breathing_ */
  Task_Breathing_Handle = osThreadNew(StartTask_BreathingLED, NULL, &Task_Breathing__attributes);

  /* creation of TASK_UART_Proce */
  TASK_UART_ProceHandle = osThreadNew(StartTask_USART_Process, NULL, &TASK_UART_Proce_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask_CAN_Process */
/**
* @brief Function implementing the CAN thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_CAN_Process */
void StartTask_CAN_Process(void *argument)
{
  /* USER CODE BEGIN StartTask_CAN_Process */
  CAN_Message_t can_msg;
  BreathingLED_Param_t led_params;
  DEBUG_PRINT("CAN Process Task Started.\n");
  /* Infinite loop */
  for(;;)
  {
    if(osMessageQueueGet(Queue_CAN_DataHandle, &can_msg, NULL, osWaitForever) == osOK)
    {
        DEBUG_PRINT("Processing CAN Msg ID=0x%03X\n", can_msg.std_id);
        /*?查CAN帧有效?? */
        if(CAN_CheckFrame(&can_msg) == SYS_OK)
        {
            /* 提取控制信息 */
            led_params = CAN_ExtractControlInfo(&can_msg);
            DEBUG_PRINT("Extracted LED Params: Period=%dms MinDuty=%d%% MaxDuty=%d%%\n",
                        led_params.period_ms, led_params.min_duty, led_params.max_duty);
            /* 发�?�到呼吸灯任务队�? */
            if(osMessageQueuePut(Queue_CAN_ProcessHandle, &led_params, 0, 0) != osOK)
            {
                DEBUG_PRINT("Breathing LED Queue Full!\n");
            }
        }
        else
        {
            DEBUG_PRINT("Invalid CAN Frame Received!\n");
        }
    }
    osDelay(1);
  }
  /* USER CODE END StartTask_CAN_Process */
}

/* USER CODE BEGIN Header_StartTask_BreathingLED */
/**
* @brief Function implementing the Task_Breathing_ thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_BreathingLED */
void StartTask_BreathingLED(void *argument)
{
  /* USER CODE BEGIN StartTask_BreathingLED */
  BreathingLED_Param_t led_params;
  /* Infinite loop */
  for(;;)
  {
    if(osMessageQueueGet(Queue_CAN_ProcessHandle, &led_params, NULL, osWaitForever) == osOK)
    {
        DEBUG_PRINT("Processing Breathing LED Params: Period=%dms MinDuty=%d%% MaxDuty=%d%%\n",
                    led_params.period_ms, led_params.min_duty, led_params.max_duty);
        /* 调用呼吸灯控制函�? */
        LED_UpdatePWM(&led_params);
        LED_BreathingUpdate();
    }
    osDelay(1);
  }
  /* USER CODE END StartTask_BreathingLED */
}

/* USER CODE BEGIN Header_StartTask_USART_Process */
/**
* @brief Function implementing the TASK_UART_Proce thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_USART_Process */
void StartTask_USART_Process(void *argument)
{
  /* USER CODE BEGIN StartTask_USART_Process */
    UART_Message_t uart_msg;
    char temp_buffer[UART_TX_BUFFER_SIZE+1];
    DEBUG_PRINT("UART Process Task Started.\n");
  /* Infinite loop */
  for(;;)
  {
    if(osMessageQueueGet(Queue_UART_DataHandle, &uart_msg, NULL, osWaitForever) == osOK)
    {
        DEBUG_PRINT("Processing UART Msg Length=%d\n", uart_msg.length);
        /* 处理UART数据，这里简单打印接收到的数�? */
        memset(temp_buffer, 0, sizeof(temp_buffer));
        memcpy(temp_buffer, uart_msg.buffer, uart_msg.length);
        DEBUG_PRINT("Received Data: %s\n", temp_buffer);
    }

    osDelay(1);
  }
  /* USER CODE END StartTask_USART_Process */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

