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
#define MOTOR_LEFT_FRONT    1   // 左前轮电机索引
#define MOTOR_RIGHT_FRONT   0   // 右前轮电机索引
#define MOTOR_RIGHT_REAR    2   // 右后轮电机索引
#define MOTOR_LEFT_REAR     3   // 左后轮电机索引

#define STRAFE_COMPENSATION_GAIN_LEFT    (2.20f)//向左平移速度补偿
#define STRAFE_COMPENSATION_GAIN_RIGHT    (1.80f)//向右平移速度补偿

// 定时器配置
#define SENSOR_TIMER_CH         PIT_CH0
#define SENSOR_UPDATE_PERIOD_MS 1
#define IMU_TIMER_CH            PIT_CH1

/**
 * @brief 传感器数据结构体定义
 */
typedef struct
{
    // 编码器原始数据 / 速度数据
    int16 encoder_speed[ENCODER_COUNT];

    // IMU963RA 六轴传感器原始数据（磁力计已禁用）
    float acc_x, acc_y, acc_z;
    float gyro_x, gyro_y, gyro_z;

    // 融合后的偏航角
    float yaw;
} SensorData_t;

/**
 * @brief 电机控制器结构体定义
 */
typedef struct
{
    DCMotor motor[ENCODER_COUNT];
    float target_speed[ENCODER_COUNT];
    uint8 initialized;
} MotorController_t;

// ==================================================== 全局变量外部声明 ====================================================

extern SensorData_t g_sensor_data;
extern MotorController_t g_motor_controller;
extern uint8 g_system_initialized;

extern uint8 g_heading_hold_enabled;
extern float g_heading_target;
extern float g_heading_target_rate;

// ==================================================== 传感器、电机、定时器初始化 ====================================================

void MotionPID_Encoder_Init(void);
void MotionPID_IMU_Init(void);
void MotionPID_Sensor_Init(void);
void MotionPID_Motor_Init(void);
void MotionPID_Timer_Init(void);
void MotionPID_IMU_Timer_Init(void);
void MotionPID_ResetTimer(void);

// ==================================================== 传感器数据更新 ====================================================

void MotionPID_ReadEncoderData(void);
void MotionPID_ReadIMUData(void);
void MotionPID_UpdateSensorData(void);

// ==================================================== PIT 中断回调 ====================================================

void pit_handler(void);
void imu_timer_handler(void);

// ==================================================== 电机目标速度控制 ====================================================

void MotionPID_SetTargetSpeed(uint8 motor_index, float target_speed);
void MotionPID_SetAllMotorsSpeed(float target_speed);

/**
 * @brief 设置麦克纳姆底盘速度
 * @param vx 前后速度分量，正值=前进，负值=后退
 * @param vy 左右速度分量，正值=左移，负值=右移
 * @note 与当前电机编号匹配：
 *       RF = vx + vy
 *       LF = vx - vy
 *       RR = vx - vy
 *       LR = vx + vy
 */
void MotionPID_SetMecanumSpeed(float vx, float vy);

/**
 * @brief 获取方向归一化后的实际电机速度
 * @param motor_index 电机索引
 * @return float 实际速度，正值表示与目标正方向一致
 */
float MotionPID_GetActualSpeed(uint8 motor_index);

// ==================================================== 航向保持 ====================================================

void MotionPID_InitHeadingHold(void);
void MotionPID_EnableHeadingHold(void);
void MotionPID_DisableHeadingHold(void);
void MotionPID_ApplyHeadingCorrection(void);

#endif
