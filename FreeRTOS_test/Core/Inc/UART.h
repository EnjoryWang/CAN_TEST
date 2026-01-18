#ifndef UART_H
#define UART_H

typedef struct
{
    uint8_t *buffer;      // Pointer to data buffer
    uint16_t length;      // Length of data
    uint32_t __TIMESTAMP__; // Timestamp of the message
} UART_MessageTypeDef;
#endif // UART_H
