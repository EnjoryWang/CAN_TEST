#ifndef __UART_PROCESS_H
#define __UART_PROCESS_H

#include "common_def.h"

typedef struct{
   uint8_t buffer[2][UART_RX_BUFFER_SIZE]; // 双缓冲区
   volatile uint8_t current_buffer;        // 当前使用的缓冲区索引
   volatile uint16_t data_length;     // 当前接收到的数据长度
}UART_RxBuffer_t;
void UART_Init(void); // 初始化UART接收
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart); // UART接收完成回调函数
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart); // UART发送完成回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);// UART接收完成回调函数
#endif /* __UART_PROCESS_H */
