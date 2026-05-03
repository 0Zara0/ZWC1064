
#include "move_mode.h"
#include "motion_PID.h"
#include "zf_driver_delay.h"

// 外部声明系统初始化标志
extern uint8 g_system_initialized;

/**
 * @brief 电机索引定义（假设小车为四轮驱动，麦克纳姆轮布局）
 * 电机布局：
 *       车头 (前方)
 *   ┌─────────────┐
 *   │  0       1  │
 *   │  LF     RF  │
 *   │             │
 *   │  LR     RR  │
 *   │  3       2  │
 *   └─────────────┘
 *   
 *   LF: 左前轮，RF: 右前轮
 *   LR: 左后轮，RR: 右后轮
 */
#define MOTOR_LEFT_FRONT    0   // 左前轮电机索引
#define MOTOR_RIGHT_FRONT   1   // 右前轮电机索引
#define MOTOR_RIGHT_REAR    2   // 右后轮电机索引
#define MOTOR_LEFT_REAR     3   // 左后轮电机索引

/**
 * @brief 初始化运动控制系统
 * @note 按顺序初始化传感器、电机和定时器
 *       具有防重复初始化机制，多次调用只执行一次
 */
void MoveMode_Init(void)
{
    // 防止重复初始化：如果系统已经初始化过，直接返回
    if (g_system_initialized)
    {
        return;
    }
    
    // 初始化传感器系统（编码器 + IMU）
    MotionPID_Sensor_Init();
    
    // 初始化电机控制器
    MotionPID_Motor_Init();
    
    // 初始化定时器（启动自动 PID 控制循环）
    MotionPID_Timer_Init();
    
    // 短暂延时确保系统稳定
    system_delay_ms(50);
}

/**
 * @brief 初始化运动控制系统（开环模式）
 * @note 适用于编码器未连接或故障的情况，电机将以开环方式运行
 *       具有防重复初始化机制，多次调用只执行一次
 */
void MoveMode_InitOpenLoop(void)
{
    // 防止重复初始化：如果系统已经初始化过，直接返回
    if (g_system_initialized)
    {
        return;
    }
    
    // 初始化传感器系统（编码器 + IMU）
    MotionPID_Sensor_Init();
    
    // 初始化电机控制器
    MotionPID_Motor_Init();
    
    // 切换到开环控制模式
    MotionPID_SetOpenLoopMode(1);
    
    // 初始化定时器（启动自动控制循环）
    MotionPID_Timer_Init();
    
    // 短暂延时确保系统稳定
    system_delay_ms(50);
}

/**
 * @brief 设置小车运动模式和速度
 * @param mode 运动模式
 * @param speed 目标速度
 */
void MoveMode_SetSpeed(MoveMode_t mode, float speed)
{
    switch (mode)
    {
        case FORWARD:
            // 前进：所有 4 个轮子同时正转（速度相同）
            MotionPID_SetAllMotorsSpeed(speed);
            break;
            
        case BACKWARD:
            // 后退：所有 4 个轮子同时反转（速度相同）
            MotionPID_SetAllMotorsSpeed(-speed);
            break;
            
        case STRAFE_LEFT:
            // 左平移：对角同向，邻角反向
            // 左前轮 (0) 与右后轮 (2) 反转
            MotionPID_SetTargetSpeed(MOTOR_LEFT_FRONT, -speed);
            MotionPID_SetTargetSpeed(MOTOR_RIGHT_REAR, -speed);
            
            // 右前轮 (1) 与左后轮 (3) 正转
            MotionPID_SetTargetSpeed(MOTOR_RIGHT_FRONT, speed);
            MotionPID_SetTargetSpeed(MOTOR_LEFT_REAR, speed);
            break;
            
        case STRAFE_RIGHT:
            // 右平移：对角同向，邻角反向
            // 左前轮 (0) 与右后轮 (2) 正转
            MotionPID_SetTargetSpeed(MOTOR_LEFT_FRONT, speed);
            MotionPID_SetTargetSpeed(MOTOR_RIGHT_REAR, speed);
            
            // 右前轮 (1) 与左后轮 (3) 反转
            MotionPID_SetTargetSpeed(MOTOR_RIGHT_FRONT, -speed);
            MotionPID_SetTargetSpeed(MOTOR_LEFT_REAR, -speed);
            break;
            
        case STOP:
        default:
            // 停止：所有电机停止转动
            MotionPID_SetAllMotorsSpeed(0.0f);
            break;
    }
}

/**
 * @brief 控制小车前进
 * @param speed 前进速度，单位：脉冲/秒
 */
void MoveMode_Forward(float speed)
{
    MoveMode_SetSpeed(FORWARD, speed);
}

/**
 * @brief 控制小车后退
 * @param speed 后退速度，单位：脉冲/秒
 */
void MoveMode_Backward(float speed)
{
    MoveMode_SetSpeed(BACKWARD, speed);
}

/**
 * @brief 控制小车左平移
 * @param speed 左移速度，单位：脉冲/秒
 */
void MoveMode_StrafeLeft(float speed)
{
    MoveMode_SetSpeed(STRAFE_LEFT, speed);
}

/**
 * @brief 控制小车右平移
 * @param speed 右移速度，单位：脉冲/秒
 */
void MoveMode_StrafeRight(float speed)
{
    MoveMode_SetSpeed(STRAFE_RIGHT, speed);
}

/**
 * @brief 停止小车运动
 */
void MoveMode_Stop(void)
{
    MoveMode_SetSpeed(STOP, 0.0f);
}
