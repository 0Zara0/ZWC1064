

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
/** @note 1ms周期: α=0.02 → τ≈50ms，与 2ms/α=0.04 保持相同滤波强度 */
#define ENCODER_FILTER_ALPHA          (0.02f)

/** @brief 编码器脉冲数溢出阈值 (用于处理 16 位计数器溢出) */
#define ENCODER_COUNT_OVERFLOW_THRESHOLD  (32000)

/** @brief 编码器最大计数值 (16 位) */
#define ENCODER_MAX_COUNT             (65536)

// ==================================================== PID 参数配置 ================================================

/** @brief 速度环 PID 比例系数 (Kp) */
/** @note 增大 Kp 可以加快响应速度，但过大会导致系统震荡 */
/** @note Kp=0.06 时比例带约 1667pps，占 5000pps 工作范围的 33%，正常调节不饱和 */
/** @note 30pps 角速度环修正时 P 项贡献 1.8%PWM，配合 I 项快速越过 3% 死区 */
#define VELOCITY_PID_KP               (0.06f)

/** @brief 速度环 PID 积分系数 (Ki) */
/** @note 增大 Ki 可以减小稳态误差，但过大会导致超调和震荡 */
/** @note 修正依据：Ki=0.005 时，稳态误差 20 脉冲/s 下需 50 秒才累积 5% PWM，实质无效 */
/** @note Ki=0.08 时，稳态误差 20 脉冲/s 下约 3 秒可累积 5% PWM，能有效消除稳态偏差 */
#define VELOCITY_PID_KI               (0.08f)

/** @brief 速度环 PID 微分系数 (Kd) */
/** @note 增大 Kd 可以抑制震荡和超调，但过大会放大噪声 */
/** @note 修正依据：原 Kd=0.002 与 dt=1ms 组合导致 Kd/dt=2.0，D项直接等于裸Δerror，引发剧烈振荡 */
/** @note Kd=0.0002 使 Kd/dt=0.2，进一步衰减编码器跳变影响，抑制高频抖动 */
#define VELOCITY_PID_KD               (0.00f)

/** @brief D 项低通滤波系数 (alpha) */
/** @note 对微分项 (Δerror/dt) 施加一阶低通滤波，抑制编码器噪声引起的 D 项尖峰 */
/** @note alpha=0.1 在 1ms 周期下对应时间常数约 10ms，平衡滤波效果和响应速度 */
#define VELOCITY_PID_D_FILTER_ALPHA   (0.1f)

/** @brief 积分分离阈值 */
/** @note 当误差超过此阈值时，取消积分作用，防止积分饱和 */
/** @note 修正依据：原阈值 100 恰好等于目标速度，堵转时 error=100 导致积分被永久锁死 */
/** @note 增大到 200 确保堵转时 (error=100) 积分仍能持续累积以克服静摩擦 */
#define VELOCITY_PID_INTEGRAL_LIMIT   (200.0f)

/** @brief 积分项限幅值 */
/** @note 防止积分累积过大导致积分饱和 */
#define VELOCITY_PID_INTEGRAL_MAX     (1000.0f)

/** @brief 输出限幅值 (PWM 占空比百分比 -95~95, 预留 5% 给 H 桥自举电容充电) */
#define VELOCITY_PID_OUTPUT_MAX       (30.0f)
#define VELOCITY_PID_OUTPUT_MIN       (-30.0f)

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
    float error_derivative_filtered; // D 项低通滤波后的误差变化率 (Δerror/dt)
    
    // PID 输出
    float output;               // PID 控制器输出 (-100~100 表示 PWM 占空比百分比)
    
    // 目标与实际值
    float target_speed;         // 目标速度 (脉冲/秒)
    float current_speed;        // 当前实际速度 (脉冲/秒)
    
    // 状态标志
    uint8 initialized;          // 初始化标志
    uint8 enabled;              // 使能标志
} VelocityPIDController_t;

// ==================================================== 角度环 PID 配置参数 ========================================

#define ANGLE_PID_KP                    (10.0f)
#define ANGLE_PID_KI                    (4.0f)
#define ANGLE_PID_OUTPUT_LIMIT          (1000.0f)
#define ANGLE_PID_INTEGRAL_LIMIT        (5000.0f)

typedef struct
{
    float Kp, Ki;
    float error, error_sum;
    float output;
    float target_angle, current_angle;
    float output_limit;
    float integral_limit;
    uint8 initialized, enabled;
} AnglePIDController_t;

// ==================================================== 全局变量声明 ================================================

extern EncoderSpeedCalculator_t g_encoder_speed_calc[ENCODER_COUNT];
extern VelocityPIDController_t g_velocity_pid_controller[ENCODER_COUNT];
extern AnglePIDController_t g_angle_pid_controller;

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

void AnglePID_Init(AnglePIDController_t *pid, float Kp, float Ki, float output_limit, float integral_limit);
float AnglePID_Calculate(AnglePIDController_t *pid, float target_angle, float current_angle, float dt);
void AnglePID_Reset(AnglePIDController_t *pid);

/* ============================== 角速度环 PID 配置 ============================== */

#define ANGULAR_RATE_PID_KP                    (3.0f)
#define ANGULAR_RATE_PID_KI                    (0.5f)
#define ANGULAR_RATE_PID_KD                    (0.1f)
#define ANGULAR_RATE_PID_OUTPUT_LIMIT          (30.0f)
#define ANGULAR_RATE_PID_INTEGRAL_LIMIT        (100.0f)

typedef struct
{
    float Kp, Ki, Kd;
    float error, error_last, error_sum;
    float output;
    float target_rate;
    float current_rate;
    float output_limit;
    float integral_limit;
    uint8 initialized, enabled;
} AngularRatePIDController_t;

extern AngularRatePIDController_t g_angular_rate_pid;

void AngularRatePID_Init(AngularRatePIDController_t *pid, float Kp, float Ki, float Kd, float output_limit, float integral_limit);
float AngularRatePID_Calculate(AngularRatePIDController_t *pid, float target_rate, float current_rate, float dt);
void AngularRatePID_Reset(AngularRatePIDController_t *pid);

#endif //PID_ALGORITHM_H
