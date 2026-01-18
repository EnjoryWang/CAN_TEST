#ifndef __COMMON_DEF_H
#define __COMMON_DEF_H

#include"main.h"
#include"cmsis_os2.h"
#include"string.h"
#include"stdio.h"
#define UART_RX_BUFFER_SIZE 256
#define UART_TX_BUFFER_SIZE 256
typedef struct 
{
    uint32_t std_id;        // Standard Identifier
    uint32_t ext_id;        // Extended Identifier
    uint8_t ide;           // Identifier Type (Standard/Extended)
    uint8_t rtr;           // Remote Transmission Request
    uint8_t dlc;           // Data Length Code
    uint8_t data[8];      // Data Field (0-8 bytes)
    uint32_t timestamp;    // Timestamp of the message
} CAN_Message_t;

typedef struct
{
    uint8_t *buffer;      // Pointer to data buffer
    uint16_t length;      // Length of data
    uint32_t timestamp; // Timestamp of the message
} UART_Message_t;

typedef struct
{
    uint32_t period_ms;      // Period of the breathing cycle in milliseconds
    uint8_t min_duty;   // Minimum duty cycle percentage
    uint8_t max_duty;   // Maximum duty cycle percentage
    uint8_t update_rate_ms;   // Update rate in milliseconds
} BreathingLED_Param_t;

typedef enum
{
    SYS_OK = 0,
    SYS_ERROR,
    SYS_INVALID_PARAM,
    SYS_BUSY,
    SYS_QUEUE_FULL,
    SYS_QUEUE_EMPTY,
    SYS_TIMEOUT
} System_Status_t;


extern osMessageQueueId_t Queue_CAN_Handle;//CAN消息队列句柄
extern osMessageQueueId_t Queue_UART_Handle;//UART消息队列句柄

extern osThreadId_t Task_CAN_ProcessHandle;//   CAN处理任务句柄
extern osThreadId_t Task_UART_ProcessHandle;//  UART处理任务句柄
extern osThreadId_t Task_BreathingLED_Handle;// 呼吸灯处理任务句柄

extern osSemaphoreId_t BinarySem_LED_FreqHandle;//呼吸灯频率修改二值信号量句柄
#endif /* __COMMON_DEF_H */
