#include "uart_process.h"
#include "usart.h"
#include "string.h"
#include "common_def.h"
#include "FreeRTOS.h"
#include "task.h"
/* 串口接收缓冲区 */
static UART_RxBuffer_t uart_rx_buffer;
static uint8_t uart_tx_buffer[UART_TX_BUFFER_SIZE];
static volatile uint8_t uart_tx_busy = 0;

/* 串口初始化 */
void UART_Init(void)
{
    /* 初始化接收缓冲区 */
    memset(uart_rx_buffer.buffer, 0, sizeof(uart_rx_buffer.buffer));
    uart_rx_buffer.current_buffer = 0;
    uart_rx_buffer.data_length = 0;
    
    /* 启动DMA接收 */
    HAL_UART_Receive_DMA(&huart1, 
                         uart_rx_buffer.buffer[uart_rx_buffer.current_buffer], 
                         UART_RX_BUFFER_SIZE);
    
    /* 启用串口空闲中断 */
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    
    DEBUG_PRINT("UART Initialized with DMA\n");
}

/* 串口发送响应 */
System_Status_t UART_SendResponse(const char *data)
{
    if(data == NULL || strlen(data) == 0)
        return SYS_INVALID_PARAM;
    
    if(uart_tx_busy)
        return SYS_BUSY;
    
    /* 格式化响应 */
    int len = snprintf((char*)uart_tx_buffer, UART_TX_BUFFER_SIZE, 
                       "Receive Data: (%s) \n", data);
    
    if(len <= 0 || len >= UART_TX_BUFFER_SIZE)
        return SYS_ERROR;
    
    /* 设置发送忙标志 */
    uart_tx_busy = 1;
    
    /* 启动DMA发送 */
    HAL_UART_Transmit_DMA(&huart1, uart_tx_buffer, len);
    
    return SYS_OK;
}

/* 串口空闲中断回调 */
void USART_IDLE_Callback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {
        /* 计算接收到的数据长度 */
        uint16_t received_len = UART_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);
        
        if(received_len > 0)
        {
            UART_Message_t uart_msg;
            
            /* 分配内存存储数据 */
            uart_msg.buffer = (uint8_t*)pvPortMalloc(received_len);
            if(uart_msg.buffer != NULL)
            {
                /* 复制数据 */
                memcpy(uart_msg.buffer, 
                       uart_rx_buffer.buffer[uart_rx_buffer.current_buffer], 
                       received_len);
                uart_msg.length = received_len;
                uart_msg.timestamp = HAL_GetTick();
                
                /* 发送到队列 */
                if(osMessageQueuePut(Queue_UART_DataHandle, &uart_msg, 0, 0) != osOK)
                {
                    /* 队列已满，释放内存 */
                    vPortFree(uart_msg.buffer);
                    DEBUG_PRINT("UART Queue Full!\n");
                }
            }
            
            /* 切换到另一个缓冲区 */
            uart_rx_buffer.current_buffer ^= 1;
            uart_rx_buffer.data_length = 0;
            
            /* 重新启动DMA接收 */
            HAL_UART_Receive_DMA(&huart1, 
                                 uart_rx_buffer.buffer[uart_rx_buffer.current_buffer], 
                                 UART_RX_BUFFER_SIZE);
        }
    }
}

/* 串口发送完成回调 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {
        /* 清除发送忙标志 */
        uart_tx_busy = 0;
    }
}

/* 串口接收完成回调 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {
        /* 更新数据长度 */
        uart_rx_buffer.data_length = UART_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);
    }
}

/* 数据验证函数（可在任务中调用） */
System_Status_t UART_ValidateData(const uint8_t *data, uint16_t length)
{
    if(data == NULL || length == 0 || length > UART_RX_BUFFER_SIZE)
        return SYS_INVALID_PARAM;
    
    /* 检查是否有不可打印字符（可选） */
    for(uint16_t i = 0; i < length; i++)
    {
        /* 只允许打印字符（包括空格和换行） */
        if(data[i] < 32 && data[i] != '\r' && data[i] != '\n' && data[i] != '\t')
            return SYS_INVALID_PARAM;
    }
    
    return SYS_OK;
}
