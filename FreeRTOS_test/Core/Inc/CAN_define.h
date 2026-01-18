#ifndef CAN_DEFINE_H
#define CAN_DEFINE_H

typedef struct
{
    uint32_t StdId;        // Standard Identifier
    uint32_t ExtId;        // Extended Identifier
    uint8_t IDE;           // Identifier Type (Standard/Extended)
    uint8_t RTR;           // Remote Transmission Request
    uint8_t DLC;           // Data Length Code
    uint8_t Data[8];      // Data Field (0-8 bytes)
    uint32_t Timestamp;    // Timestamp of the message
} CAN_MessageTypeDef;
#endif // CAN_DEFINE_H
