#include "led_control.h"
static LED_State_t led_state;
static uint32_t last_update_time = 0;

void LED_Init() {
    // 初始化LED GPIO和PWM设置
    led_state.pwm_period = 1000; // 假设PWM周期为1000个计数器时钟周期
    led_state.pwm_duty = 0;
    led_state.direction = 1; // 初始方向为增加
    // 设置默认呼吸灯参数
    led_state.params.period_ms = 2000; // 2秒一个呼吸周期
    led_state.params.min_duty = 10;    // 最小占空比10%
    led_state.params.max_duty = 90;    // 最大占空比90%
    led_state.params.update_rate_ms = 20; // 每20ms更新一次

    uint32_t timer_freq = HAL_RCC_GetPCLK1Freq() * 2; // 获取定时器时钟频率
    uint32_t prescaler = htim3.Init.Prescaler + 1;
    uint32_t arr=(led_state.params.update_rate_ms * (timer_freq / prescaler)) / 1000;
    led_state.pwm_period = arr;// 计算PWM周期
    __HAL_TIM_SET_AUTORELOAD(&htim3, led_state.pwm_period);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0); // 初始占空比为0
}

void LED_UpdatePWM(const BreathingLED_Param_t *params) {
    if(params == NULL) return;
    led_state.params = *params;
    // 重新计算PWM周期
    uint32_t timer_freq = HAL_RCC_GetPCLK1Freq() * 2; // 获取定时器时钟频率
    uint32_t prescaler = htim3.Init.Prescaler + 1;
    uint32_t arr=(led_state.params.update_rate_ms * (timer_freq / prescaler)) / 1000;
    led_state.pwm_period = arr;
    __HAL_TIM_SET_AUTORELOAD(&htim3, led_state.pwm_period);// 设置新的PWM周期
    if(led_state.pwm_duty > led_state.params.max_duty)
        led_state.pwm_duty = led_state.params.max_duty;
    if(led_state.pwm_duty < led_state.params.min_duty)
        led_state.pwm_duty = led_state.params.min_duty;// 调整当前占空比在新范围内

}

void LED_BreathingUpdate(void) {
    uint32_t current_time = HAL_GetTick();
    if (current_time - last_update_time >= led_state.params.update_rate_ms) {
        last_update_time = current_time;

        uint32_t duty_range = led_state.params.max_duty - led_state.params.min_duty;
        uint32_t duty_step = (duty_range * led_state.params.update_rate_ms) / (led_state.params.period_ms / 2);

        if (led_state.direction == 1) {
            // 增加占空比
            if (led_state.pwm_duty + duty_step >= led_state.params.max_duty) {
                led_state.pwm_duty = led_state.params.max_duty;
                led_state.direction = -1; // 改变方向为减少
            } else {
                led_state.pwm_duty += duty_step;
            }
        } else {
            // 减少占空比
            if (led_state.pwm_duty <= led_state.params.min_duty + duty_step) {
                led_state.pwm_duty = led_state.params.min_duty;
                led_state.direction = 1; // 改变方向为增加
            } else {
                led_state.pwm_duty -= duty_step;
            }
        }

        // 更新PWM占空比
        uint32_t compare_value = (led_state.pwm_duty * led_state.pwm_period) / 100;
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, compare_value);
    }
}