#ifndef _dc_motor_h_
#define _dc_motor_h_

#include "zf_driver_pwm.h"
#include "zf_driver_gpio.h"

/**
 * @brief 直流电机控制结构体定义
 * @note 该结构体封装了单电机的所有控制参数和状态信息
 *       支持通过方向引脚和 PWM 通道实现电机的正转、反转和速度调节
 */
typedef struct DCMotor
{
    // GPIO 引脚配置成员变量
    gpio_pin_enum dir_pin;          // 方向控制引脚
    pwm_channel_enum pwm_channel;   // PWM 通道
    
    // 电机运行参数成员变量
    uint32 pwm_freq;                // PWM 频率 (Hz)
    int8 speed_percent;             // 当前速度百分比 (-100~100)
} DCMotor;

void DCMotor_Init(DCMotor *motor, gpio_pin_enum dir_pin, pwm_channel_enum pwm_channel, uint32 freq);
void DCMotor_SetSpeed(DCMotor *motor, float speed);
void DCMotor_Stop(DCMotor *motor);

#endif
