#ifndef __BREATHING_LED_H
#define __BREATHING_LED_H

typedef struct
{
    uint32_t period_ms;      // Period of the breathing cycle in milliseconds
    uint8_t min_duty;   // Minimum duty cycle percentage
    uint8_t max_duty;   // Maximum duty cycle percentage
}BreathingLEDConfig;
#endif /* __BREATHING_LED_H */
