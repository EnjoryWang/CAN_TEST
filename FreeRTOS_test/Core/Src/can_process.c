#include "can_process.h"
#include "led_control.h"

/* CAN接收回调函数 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if(hcan->Instance == CAN1)
    {
        CAN_Message_t can_msg;
        CAN_RxHeaderTypeDef rx_header;
        
        /* 读取CAN数据 */
        if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, can_msg.data) == HAL_OK)
        {
            /* 打包数据 */
            can_msg.std_id = rx_header.StdId;
            can_msg.dlc = rx_header.DLC;
            can_msg.timestamp = HAL_GetTick();
            
            /* 发送到队列 */
           if(osMessageQueuePut(Queue_CAN_Handle, &can_msg, 0, 0) != osOK)
            {
                /* 队列已满，丢弃数据 */
                DEBUG_PRINT("CAN Queue Full!\n");
            }
            else
            {
                /* 成功接收CAN消息 */
                DEBUG_PRINT("CAN Msg Received: ID=0x%03X DLC=%d\n", can_msg.std_id, can_msg.dlc);
            }
        }
    }
}

/* CAN帧检查函数 */
System_Status_t CAN_CheckFrame(const CAN_Message_t *msg)
{
    /* 检查数据长度 */
    if(msg->dlc < 3 || msg->dlc > 8)
    {
        return SYS_INVALID_PARAM;
    }
    
    /* 检查头帧（头帧0xAA，尾帧0x55） */
    if(msg->data[0] != 0xAA)
    {
        return SYS_INVALID_PARAM;
    }
    
    /* 检查尾帧 */
    if(msg->data[msg->dlc - 1] != 0x55)
    {
        return SYS_INVALID_PARAM;
    }
    
    /* 简单的CRC检查（效验位） */
    uint8_t checksum = 0;
    for(uint8_t i = 1; i < msg->dlc - 2; i++)
    {
        checksum += msg->data[i];
    }
    
    if(checksum != msg->data[msg->dlc - 2])
    {
        return SYS_INVALID_PARAM;
    }
    
    return SYS_OK;
}

/* 提取控制信息函数 */
BreathingLED_Param_t CAN_ExtractControlInfo(const CAN_Message_t *msg)
{
    BreathingLED_Param_t param;
    
    /* 从CAN数据中提取控制信息 */
    /* 数据格式：[0xAA][命令][周期高][周期低][最小占空比][最大占空比][CRC][0x55] */
    param.period_ms = (msg->data[2] << 8) | msg->data[3];//从第三个数据开始，把高八位左移8位，与低八位进行或运算，得到完整的周期值
    param.min_duty = msg->data[4];
    param.max_duty = msg->data[5];
    param.update_rate_ms = 10;  /* 默认10ms更新一次 */
    
    /* 参数有效性检查 */
    if(param.period_ms < 100) param.period_ms = 100;
    if(param.period_ms > 10000) param.period_ms = 10000;
    if(param.min_duty > 90) param.min_duty = 10;
    if(param.max_duty < 20) param.max_duty = 90;
    if(param.min_duty >= param.max_duty)
    {
        param.min_duty = 10;
        param.max_duty = 90;
    }
    
    return param;
}