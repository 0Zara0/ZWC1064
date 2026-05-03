#include "dc_motor.h"

/**
 * @brief 直流电机初始化函数
 * @param motor 指向 DCMotor 结构体的指针，用于存储电机配置参数
 * @param dir_pin 方向控制 GPIO 引脚，通过电平高低控制电机正反转
 * @param pwm_channel PWM 通道号，用于输出 PWM 波形控制电机速度
 * @param freq PWM 信号频率，单位为 Hz，决定 PWM 波形的周期
 * @note 初始化后会设置方向引脚为 GPIO 输出模式，PWM 通道占空比为 0
 */
void DCMotor_Init(DCMotor *motor, gpio_pin_enum dir_pin, pwm_channel_enum pwm_channel, uint32 freq)
{
    // 参数有效性检查
    if (motor == NULL)
    {
        return;
    }
    
    // 将用户配置的方向引脚、PWM 通道和频率参数保存到电机结构体中
    motor->dir_pin = dir_pin;
    motor->pwm_channel = pwm_channel;
    motor->pwm_freq = freq;
    motor->speed_percent = 0;  // 初始化速度百分比为 0，表示电机初始状态为停止
    
    // 配置方向控制引脚为 GPIO 输出模式，初始电平为 0
    gpio_init(dir_pin, GPO, 0, GPO_PUSH_PULL);
    
    // 配置 PWM 通道，设置工作频率和初始占空比为 0%
    pwm_init(pwm_channel, freq, 0);
}

/**
 * @brief 设置电机运行速度
 * @param motor 指向 DCMotor 结构体的指针
 * @param speed 速度百分比参数，取值范围 -100 到 100
 *              正值表示电机正转，负值表示电机反转，0 表示停止电机
 * @note 函数内部会自动限制速度范围在 -100 到 100 之间，超出范围的值会被截断
 */
void DCMotor_SetSpeed(DCMotor *motor, float speed)
{
    if (motor == NULL)
    {
        return;
    }

    if (speed > 100.0f)
    {
        speed = 100.0f;
    }
    else if (speed < -100.0f)
    {
        speed = -100.0f;
    }

    motor->speed_percent = (int8)speed;

    if (speed > 0.0f)
    {
        gpio_set_level(motor->dir_pin, 1);
        pwm_set_duty(motor->pwm_channel, (uint32)(speed * 100.0f));
    }
    else if (speed < 0.0f)
    {
        gpio_set_level(motor->dir_pin, 0);
        pwm_set_duty(motor->pwm_channel, (uint32)(-speed * 100.0f));
    }
    else
    {
        pwm_set_duty(motor->pwm_channel, 0);
    }
}

/**
 * @brief 停止电机运行
 * @param motor 指向 DCMotor 结构体的指针
 * @note 该函数将速度设置为 0，调用 DCMotor_SetSpeed 实现电机制动
 */
void DCMotor_Stop(DCMotor *motor)
{
    // 参数有效性检查
    if (motor == NULL)
    {
        return;
    }
    
    DCMotor_SetSpeed(motor, 0);
}
