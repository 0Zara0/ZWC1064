
#ifndef MOVE_MODE_H
#define MOVE_MODE_H

#include "zf_common_typedef.h"

/**
 * @brief 小车运动模式枚举
 */
typedef enum
{
    STOP = 0,           // 停止
    FORWARD,            // 前进
    BACKWARD,           // 后退
    STRAFE_LEFT,        // 左平移
    STRAFE_RIGHT        // 右平移
} MoveMode_t;

/**
 * @brief 初始化运动控制系统
 * @note 调用此函数会初始化所有传感器、电机和定时器
 *       必须在调用其他运动控制函数前执行
 */
void MoveMode_Init(void);

/**
 * @brief 设置小车运动模式和速度
 * @param mode 运动模式（前进/后退/左移/右移/停止）
 * @param speed 目标速度，单位：脉冲/秒
 *              正值表示正向运动，负值表示反向运动
 * @note 示例：
 *       MoveMode_SetSpeed(FORWARD, 100.0f);     // 以 100 脉冲/秒前进
 *       MoveMode_SetSpeed(BACKWARD, 50.0f);     // 以 50 脉冲/秒后退
 *       MoveMode_SetSpeed(STRAFE_LEFT, 80.0f);  // 以 80 脉冲/秒左移
 *       MoveMode_SetSpeed(STOP, 0.0f);          // 停止
 */
void MoveMode_SetSpeed(MoveMode_t mode, float speed);

/**
 * @brief 控制小车前进
 * @param speed 前进速度，单位：脉冲/秒
 */
void MoveMode_Forward(float speed);

/**
 * @brief 控制小车后退
 * @param speed 后退速度，单位：脉冲/秒
 */
void MoveMode_Backward(float speed);

/**
 * @brief 控制小车左平移
 * @param speed 左移速度，单位：脉冲/秒
 */
void MoveMode_StrafeLeft(float speed);

/**
 * @brief 控制小车右平移
 * @param speed 右移速度，单位：脉冲/秒
 */
void MoveMode_StrafeRight(float speed);

/**
 * @brief 停止小车运动
 */
void MoveMode_Stop(void);

#endif // MOVE_MODE_H
