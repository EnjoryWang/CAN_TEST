#include "CAN_define.h"
#include "main.h"
#include<stdio.h>

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
    {
        CAN_MessageTypeDef can_msg;
        can_msg.StdId = RxHeader.StdId;
        can_msg.ExtId = RxHeader.ExtId;
        can_msg.DLC = RxHeader.DLC;
        can_msg.Timestamp = HAL_GetTick();
        // 打包数据
    }
    /*else
    {
        // Process the received message
        printf("Received CAN message: ID=0x%X, Data=", RxHeader.StdId);
        for (int i = 0; i < RxHeader.DLC; i++)
        {
            printf(" %02X", RxData[i]);
        }
        printf("\n");
    }*/
}