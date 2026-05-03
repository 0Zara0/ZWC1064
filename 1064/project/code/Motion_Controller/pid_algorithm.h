

#ifndef PID_ALGORITHM_H
#define PID_ALGORITHM_H

#include "zf_common_typedef.h"
#include "dc_motor.h"

// 编码器数量定义（与 motion_PID.h 保持一致）
#ifndef ENCODER_COUNT
#define ENCODER_COUNT   (4)
#endif

// ==================================================== 配置参数 ====================================================

/** @brief 编码器低通滤波系数 (alpha = dt / (RC + dt)) */
/** @note alpha 越小滤波效果越强，但响应越慢；alpha 越大响应越快，但滤波效果弱 */
#define ENCODER_FILTER_ALPHA          (0.2f)

/** @brief 编码器脉冲数溢出阈值 (用于处理 16 位计数器溢出) */
#define ENCODER_COUNT_OVERFLOW_THRESHOLD  (32000)

/** @brief 编码器最大计数值 (16 位) */
#define ENCODER_MAX_COUNT             (65536)

// ==================================================== PID 参数配置 ================================================

/** @brief 速度环 PID 比例系数 (Kp) */
/** @note 增大 Kp 可以加快响应速度，但过大会导致系统震荡 */
#define VELOCITY_PID_KP               (1.0f)

/** @brief 速度环 PID 积分系数 (Ki) */
/** @note 增大 Ki 可以减小稳态误差，但过大会导致超调和震荡 */
#define VELOCITY_PID_KI               (0.01f)

/** @brief 速度环 PID 微分系数 (Kd) */
/** @note 增大 Kd 可以抑制震荡和超调，但过大会放大噪声 */
#define VELOCITY_PID_KD               (0.05f)

/** @brief 积分分离阈值 */
/** @note 当误差超过此阈值时，取消积分作用，防止积分饱和 */
#define VELOCITY_PID_INTEGRAL_LIMIT   (100.0f)

/** @brief 积分项限幅值 */
/** @note 防止积分累积过大导致积分饱和 */
#define VELOCITY_PID_INTEGRAL_MAX     (1000.0f)

/** @brief 输出限幅值 (PWM 占空比百分比 -100~100) */
#define VELOCITY_PID_OUTPUT_MAX       (100.0f)
#define VELOCITY_PID_OUTPUT_MIN       (-100.0f)

// ==================================================== 数据结构定义 ==================================================

/**
 * @brief 编码器速度计算器结构体
 * @note 用于存储每个编码器的速度计算状态
 */
typedef struct
{
    int16 last_count;           // 上次编码器计数值
    float speed_filtered;       // 滤波后的速度 (脉冲/秒)
    float speed_raw;            // 原始瞬时速度 (脉冲/秒)
    uint8 initialized;          // 初始化标志
} EncoderSpeedCalculator_t;

/**
 * @brief 速度环 PID 控制器结构体
 * @note 使用位置式 PID 算法，每个电机独立控制
 */
typedef struct
{
    // PID 参数
    float Kp;                   // 比例系数
    float Ki;                   // 积分系数
    float Kd;                   // 微分系数
    
    // PID 中间变量
    float error;                // 当前误差
    float error_last;           // 上次误差
    float error_sum;            // 误差累积 (积分项)
    
    // PID 输出
    float output;               // PID 控制器输出 (-100~100 表示 PWM 占空比百分比)
    
    // 目标与实际值
    float target_speed;         // 目标速度 (脉冲/秒)
    float current_speed;        // 当前实际速度 (脉冲/秒)
    
    // 状态标志
    uint8 initialized;          // 初始化标志
    uint8 enabled;              // 使能标志
} VelocityPIDController_t;

// ==================================================== 全局变量声明 ================================================

extern EncoderSpeedCalculator_t g_encoder_speed_calc[ENCODER_COUNT];
extern VelocityPIDController_t g_velocity_pid_controller[ENCODER_COUNT];

// ==================================================== 函数声明 ====================================================

/**
 * @brief 初始化编码器速度计算器
 * @note 在系统初始化时调用一次
 */
void EncoderSpeedCalc_Init(void);

/**
 * @brief 计算并滤波编码器速度
 * @param encoder_data 编码器原始数据数组
 * @param encoder_count 编码器数量
 * @param dt 采样时间间隔 (秒)
 * @note 此函数应在 MotionPID_UpdateSensorData() 中被调用
 *       将编码器位置转换为速度并进行低通滤波
 */
void EncoderSpeedCalc_Update(int16* encoder_data, uint8 encoder_count, float dt);

/**
 * @brief 获取滤波后的编码器速度
 * @param encoder_index 编码器索引 (0~ENCODER_COUNT-1)
 * @return float 滤波后的速度值 (脉冲/秒)
 */
float EncoderSpeedCalc_GetFilteredSpeed(uint8 encoder_index);

// ==================================================== PID 速度环函数声明 ===========================================

/**
 * @brief 初始化速度环 PID 控制器
 * @param pid_index PID 控制器索引 (对应电机编号)
 * @param Kp 比例系数
 * @param Ki 积分系数
 * @param Kd 微分系数
 * @note 在系统初始化阶段调用，为每个电机配置独立的 PID 参数
 */
void VelocityPID_Init(uint8 pid_index, float Kp, float Ki, float Kd);

/**
 * @brief 初始化所有速度环 PID 控制器（使用默认参数）
 * @note 便捷函数，一次性初始化 4 个电机的 PID 控制器
 */
void VelocityPID_InitAll(void);

/**
 * @brief 执行速度环 PID 计算
 * @param pid_index PID 控制器索引 (0~ENCODER_COUNT-1)
 * @param target_speed 目标速度 (脉冲/秒)
 * @param current_speed 当前实际速度 (脉冲/秒)
 * @param dt 采样时间间隔 (秒)
 * @return float PID 控制器输出值 (-100~100 表示 PWM 占空比百分比)
 * @note 使用位置式 PID 算法：output = Kp*e + Ki*∑e + Kd*(e-e_last)
 */
float VelocityPID_Calculate(uint8 pid_index, float target_speed, float current_speed, float dt);

/**
 * @brief 获取 PID 控制器输出
 * @param pid_index PID 控制器索引 (0~ENCODER_COUNT-1)
 * @return float PID 输出值 (-100~100)
 */
float VelocityPID_GetOutput(uint8 pid_index);

/**
 * @brief 获取当前速度误差
 * @param pid_index PID 控制器索引 (0~ENCODER_COUNT-1)
 * @return float 速度误差 (脉冲/秒)
 */
float VelocityPID_GetError(uint8 pid_index);

/**
 * @brief 重置 PID 控制器状态
 * @param pid_index PID 控制器索引 (0~ENCODER_COUNT-1)
 * @note 清除积分累积和误差历史，用于避免积分饱和或重新启动
 */
void VelocityPID_Reset(uint8 pid_index);

/**
 * @brief 根据 PID 输出执行电机控制
 * @param pid_index PID 控制器索引 (对应电机编号)
 * @param motor 指向电机控制器结构体的指针
 * @note 将 PID 输出 (-100~100) 转换为电机速度控制信号
 *       自动处理正反转和 PWM 占空比设置
 */
void VelocityPID_ExecuteMotorControl(uint8 pid_index, DCMotor *motor);

#endif //PID_ALGORITHM_H
