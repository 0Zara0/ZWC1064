

#ifndef MOTION_PID_H
#define MOTION_PID_H

#include "imu963.h"
#include "zf_driver_encoder.h"
#include "zf_driver_pit.h"
#include "dc_motor.h"

// 编码器数量定义（4 个电机）
#define ENCODER_COUNT   (4)

/**
 * @brief 电机索引定义（四轮驱动，麦克纳姆轮布局，实测验证）
 * 电机布局：
 *       车头 (前方)
 *   ┌─────────────┐
 *   │  1       0  │
 *   │  LF     RF  │
 *   │             │
 *   │  LR     RR  │
 *   │  3       2  │
 *   └─────────────┘
 *
 *   LF: 左前轮，RF: 右前轮
 *   LR: 左后轮，RR: 右后轮
 */
#define MOTOR_LEFT_FRONT    1   // 左前轮电机索引 (实测: 电机1=LF)
#define MOTOR_RIGHT_FRONT   0   // 右前轮电机索引 (实测: 电机0=RF)
#define MOTOR_RIGHT_REAR    2   // 右后轮电机索引
#define MOTOR_LEFT_REAR     3   // 左后轮电机索引

// 定时器配置
#define SENSOR_TIMER_CH         PIT_CH0               // 使用 PIT 通道 0
#define SENSOR_UPDATE_PERIOD_MS 2                     // 传感器更新周期 2ms(500Hz)

/**
 * @brief 传感器数据结构体定义
 * @note 该结构体集中存储从正交编码器和 IMU963RA 采集的所有原始数据和融合数据
 *       包括：编码器脉冲计数、三轴加速度、三轴角速度、三轴磁场强度以及融合后的偏航角
 */

 /* @par 使用示例:
 * @code
 * // ========== 系统初始化阶段 ==========
 * MotionPID_Sensor_Init();   // 初始化传感器硬件（编码器 + IMU）
 * MotionPID_Motor_Init();     // 初始化电机驱动控制器
 * MotionPID_Timer_Init();     // 初始化 PIT 定时器（自动使能中断，2ms 周期）
 * 
 * // ========== 主循环运行阶段 ==========
 * // 使用中断方式时的主循环（推荐方案）
 * while(1) {
 *     // 传感器数据由定时器中断自动更新，无需手动调用
 *     // 直接使用全局变量 g_sensor_data 中的数据即可
 *     int16 speed = g_sensor_data.encoder_speed[0];    // 获取电机 1 速度
 *     float gyro = g_sensor_data.gyro_z;              // 获取 Z 轴角速度
 *     
 *     // 在此添加上层控制算法和决策逻辑
 * }
 * @endcode
 */
typedef struct
{
    // 编码器原始数据（来自正交编码器计数器）
    int16 encoder_speed[ENCODER_COUNT];     // 编码器计数（用于速度计算）
    
    // IMU963RA 九轴传感器原始数据
    float acc_x, acc_y, acc_z;              // 三轴加速度（单位：g）
    float gyro_x, gyro_y, gyro_z;           // 三轴角速度（单位：度/秒）
    float mag_x, mag_y, mag_z;              // 三轴磁场（单位：Gs）
    
    // 通过传感器融合算法解算出的姿态角度数据
    float yaw;                              // 偏航角（单位：度）
} SensorData_t;

/**
 * @brief 电机控制器结构体定义
 * @note 该结构体管理 4 个直流电机实例的运行状态和控制参数
 */
typedef struct
{
    DCMotor motor[ENCODER_COUNT];           // 4 个直流电机实例
    float target_speed[ENCODER_COUNT];      // 目标速度（脉冲/秒），供 PID 直接使用
    uint8 initialized;                      // 初始化标志
} MotorController_t;

// 全局变量外部声明，供其他文件访问
extern SensorData_t g_sensor_data;          // 全局传感器数据
extern MotorController_t g_motor_controller; // 全局电机控制器
extern uint8 g_system_initialized;          // 系统初始化标志（防止重复初始化）

// 传感器初始化和数据采集函数声明
void MotionPID_Sensor_Init(void);                       // 传感器初始化（编码器和 IMU）
void MotionPID_Motor_Init(void);                        // 电机控制器初始化
void MotionPID_ReadEncoderData(void);                   // 读取所有编码器数据
void MotionPID_ReadIMUData(void);                       // 读取 IMU 数据
void MotionPID_UpdateSensorData(void);                  // 更新所有传感器数据
void MotionPID_Timer_Init(void);                        // 定时器初始化
void MotionPID_ResetTimer(void);                        // 重置定时器

// 简化的运动控制 API（供上层应用调用）
void MotionPID_SetTargetSpeed(uint8 motor_index, float target_speed);  // 设置单个电机目标速度（脉冲/秒）
void MotionPID_SetAllMotorsSpeed(float target_speed);                  // 设置所有电机相同的目标速度
float MotionPID_GetActualSpeed(uint8 motor_index);                     // 获取单个电机的实际速度

void MotionPID_EnableHeadingHold(void);
void MotionPID_DisableHeadingHold(void);
void MotionPID_InitHeadingHold(void);
extern uint8 g_heading_hold_enabled;
extern float g_heading_target;

#endif //MOTION_PID_H
