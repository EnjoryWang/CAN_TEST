#ifndef __CAN_PROCESS_H
#define __CAN_PROCESS_H

#include "common_def.h"
System_Status_t CAN_CheckFrame(const CAN_Message_t* msg);
BreathingLED_Param_t CAN_ExtractControlInfo(const CAN_Message_t* msg);// 从CAN消息中提取呼吸灯控制信息
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);// CAN接收回调函数

#endif /* __CAN_PROCESS_H */
