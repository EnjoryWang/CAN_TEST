#ifndef __LED_CONTROL_H
#define __LED_CONTROL_H

#include"common_def.h"
typedef struct
{
    uint32_t pwm_period;      // PWM周期，单位为计数器时钟周期数
    uint32_t pwm_duty;      // 占空比，范围0到pwm
    int8_t direction;     // 变化方向，1表示增加，-1表示减少
    BreathingLED_Param_t params; // 呼吸灯参数
}LED_State_t;

void LED_Init(void);// 初始化LED控制
void LED_UpdatePWM(const BreathingLED_Param_t  *params);// 更新LED的PWM设置
void LED_BreathingUpdate(void);// 呼吸灯状态更新函数
#endif /* __LED_CONTROL_H */

