

#include "pid_algorithm.h"
#include "motion_PID.h"  // 需要访问 g_sensor_data 全局变量
#include "zf_driver_delay.h"
#include <math.h>

// ==================================================== 全局变量定义 ====================================================

/** @brief 编码器速度计算器数组 */
EncoderSpeedCalculator_t g_encoder_speed_calc[ENCODER_COUNT] = {0};

/** @brief 速度环 PID 控制器数组 (4 个电机) */
VelocityPIDController_t g_velocity_pid_controller[ENCODER_COUNT] = {0};

// ==================================================== 静态函数声明 ================================================

/**
 * @brief 处理编码器计数溢出
 * @param current_count 当前计数值
 * @param last_count 上次计数值
 * @return int16 修正后的差值
 * @note 当编码器计数器溢出时 (从 32767→-32768 或反之),需要修正差值
 */
static int16 EncoderSpeedCalc_HandleOverflow(int16 current_count, int16 last_count);

// ==================================================== PID 速度环静态函数声明 =======================================

/**
 * @brief 限制输出值在指定范围内
 * @param value 待限制的值
 * @param min_val 最小值
 * @param max_val 最大值
 * @return float 限制后的值
 */
static float VelocityPID_LimitOutput(float value, float min_val, float max_val);

// ==================================================== 函数实现 ======================================================

/**
 * @brief 初始化编码器速度计算器
 * @note 在系统初始化时调用一次
 */
void EncoderSpeedCalc_Init(void)
{
    for (uint8 i = 0; i < ENCODER_COUNT; i++)
    {
        g_encoder_speed_calc[i].last_count = 0;
        g_encoder_speed_calc[i].speed_filtered = 0.0f;
        g_encoder_speed_calc[i].speed_raw = 0.0f;
        g_encoder_speed_calc[i].initialized = 0;
    }
}

/**
 * @brief 处理编码器计数溢出
 * @param current_count 当前计数值
 * @param last_count 上次计数值
 * @return int16 修正后的差值
 */
static int16 EncoderSpeedCalc_HandleOverflow(int16 current_count, int16 last_count)
{
    int16 delta = current_count - last_count;
    
    // 检测正向溢出 (从负最大值跳变到正最大值)
    if (delta > ENCODER_COUNT_OVERFLOW_THRESHOLD)
    {
        delta -= ENCODER_MAX_COUNT;
    }
    // 检测负向溢出 (从正最大值跳变到负最大值)
    else if (delta < -ENCODER_COUNT_OVERFLOW_THRESHOLD)
    {
        delta += ENCODER_MAX_COUNT;
    }
    
    return delta;
}

/**
 * @brief 计算并滤波编码器速度
 * @param encoder_data 编码器原始数据数组
 * @param encoder_count 编码器数量
 * @param dt 采样时间间隔 (秒)
 * @note 此函数应在 MotionPID_UpdateSensorData() 中被调用
 *       将编码器位置转换为速度并进行低通滤波
 */
void EncoderSpeedCalc_Update(int16* encoder_data, uint8 encoder_count, float dt)
{
    // 参数验证
    if (encoder_data == NULL || encoder_count == 0 || dt <= 0.0f)
    {
        return;
    }
    
    // 限制最大编码器数量
    if (encoder_count > ENCODER_COUNT)
    {
        encoder_count = ENCODER_COUNT;
    }
    
    for (uint8 i = 0; i < encoder_count; i++)
    {
        // 第一次运行时，仅初始化，不计算速度
        if (!g_encoder_speed_calc[i].initialized)
        {
            g_encoder_speed_calc[i].last_count = encoder_data[i];
            g_encoder_speed_calc[i].speed_filtered = 0.0f;
            g_encoder_speed_calc[i].speed_raw = 0.0f;
            g_encoder_speed_calc[i].initialized = 1;
            continue;
        }
        
        // 1. 计算脉冲差值 (处理溢出)
        int16 delta = EncoderSpeedCalc_HandleOverflow(encoder_data[i], g_encoder_speed_calc[i].last_count);
        
        // 2. 计算瞬时速度 (脉冲/秒)
        g_encoder_speed_calc[i].speed_raw = (float)delta / dt;
        
        // 3. 一阶低通滤波 (指数平滑)
        // 公式：y[n] = α * x[n] + (1-α) * y[n-1]
        g_encoder_speed_calc[i].speed_filtered = 
            ENCODER_FILTER_ALPHA * g_encoder_speed_calc[i].speed_raw + 
            (1.0f - ENCODER_FILTER_ALPHA) * g_encoder_speed_calc[i].speed_filtered;
        
        // 4. 更新上次计数值
        g_encoder_speed_calc[i].last_count = encoder_data[i];
        
        // 5. 同步更新到全局传感器数据结构体
        g_sensor_data.encoder_speed[i] = (int16)g_encoder_speed_calc[i].speed_filtered;
    }
}

/**
 * @brief 获取滤波后的编码器速度
 * @param encoder_index 编码器索引 (0~ENCODER_COUNT-1)
 * @return float 滤波后的速度值 (脉冲/秒)
 */
float EncoderSpeedCalc_GetFilteredSpeed(uint8 encoder_index)
{
    if (encoder_index >= ENCODER_COUNT)
    {
        return 0.0f;
    }
    
    return g_encoder_speed_calc[encoder_index].speed_filtered;
}

// ==================================================== PID 速度环函数实现 ===========================================

/**
 * @brief 限制输出值在指定范围内
 * @param value 待限制的值
 * @param min_val 最小值
 * @param max_val 最大值
 * @return float 限制后的值
 */
static float VelocityPID_LimitOutput(float value, float min_val, float max_val)
{
    if (value > max_val)
    {
        return max_val;
    }
    else if (value < min_val)
    {
        return min_val;
    }
    return value;
}

/**
 * @brief 初始化速度环 PID 控制器
 * @param pid_index PID 控制器索引 (对应电机编号)
 * @param Kp 比例系数
 * @param Ki 积分系数
 * @param Kd 微分系数
 * @note 在系统初始化阶段调用，为每个电机配置独立的 PID 参数
 */
void VelocityPID_Init(uint8 pid_index, float Kp, float Ki, float Kd)
{
    if (pid_index >= ENCODER_COUNT)
    {
        return;
    }
    
    // 设置 PID 参数
    g_velocity_pid_controller[pid_index].Kp = Kp;
    g_velocity_pid_controller[pid_index].Ki = Ki;
    g_velocity_pid_controller[pid_index].Kd = Kd;
    
    // 清零中间变量
    g_velocity_pid_controller[pid_index].error = 0.0f;
    g_velocity_pid_controller[pid_index].error_last = 0.0f;
    g_velocity_pid_controller[pid_index].error_sum = 0.0f;
    g_velocity_pid_controller[pid_index].output = 0.0f;
    
    // 目标与实际值清零
    g_velocity_pid_controller[pid_index].target_speed = 0.0f;
    g_velocity_pid_controller[pid_index].current_speed = 0.0f;
    
    // 设置状态标志
    g_velocity_pid_controller[pid_index].initialized = 1;
    g_velocity_pid_controller[pid_index].enabled = 1;  // 默认使能
}

/**
 * @brief 初始化所有速度环 PID 控制器（使用默认参数）
 * @note 便捷函数，一次性初始化 4 个电机的 PID 控制器
 */
void VelocityPID_InitAll(void)
{
    for (uint8 i = 0; i < ENCODER_COUNT; i++)
    {
        VelocityPID_Init(i, VELOCITY_PID_KP, VELOCITY_PID_KI, VELOCITY_PID_KD);
    }
}

/**
 * @brief 执行速度环 PID 计算
 * @param pid_index PID 控制器索引 (0~ENCODER_COUNT-1)
 * @param target_speed 目标速度 (脉冲/秒)
 * @param current_speed 当前实际速度 (脉冲/秒)
 * @param dt 采样时间间隔 (秒)
 * @return float PID 控制器输出值 (-100~100 表示 PWM 占空比百分比)
 * @note 使用位置式 PID 算法：output = Kp*e + Ki*∑e + Kd*(e-e_last)
 */
float VelocityPID_Calculate(uint8 pid_index, float target_speed, float current_speed, float dt)
{
    // 参数验证
    if (pid_index >= ENCODER_COUNT || dt <= 0.0f)
    {
        return 0.0f;
    }
    
    // 检查是否已初始化和使能
    if (!g_velocity_pid_controller[pid_index].initialized || 
        !g_velocity_pid_controller[pid_index].enabled)
    {
        return 0.0f;
    }
    
    // 1. 计算误差 e(t) = target - current
    g_velocity_pid_controller[pid_index].error = target_speed - current_speed;
    
    // 2. 比例项 P = Kp * e
    float proportional = g_velocity_pid_controller[pid_index].Kp * g_velocity_pid_controller[pid_index].error;
    
    // 3. 积分项 I = Ki * ∑e (带限幅和积分分离)
    // 只有当误差小于阈值时才进行积分累积（积分分离）
    if (fabs(g_velocity_pid_controller[pid_index].error) < VELOCITY_PID_INTEGRAL_LIMIT)
    {
        g_velocity_pid_controller[pid_index].error_sum += g_velocity_pid_controller[pid_index].error * dt;
        // 积分项限幅
        g_velocity_pid_controller[pid_index].error_sum = VelocityPID_LimitOutput(
            g_velocity_pid_controller[pid_index].error_sum,
            -VELOCITY_PID_INTEGRAL_MAX,
            VELOCITY_PID_INTEGRAL_MAX
        );
    }
    float integral = g_velocity_pid_controller[pid_index].Ki * g_velocity_pid_controller[pid_index].error_sum;
    
    // 4. 微分项 D = Kd * (e - e_last) / dt
    float derivative = g_velocity_pid_controller[pid_index].Kd * 
                       (g_velocity_pid_controller[pid_index].error - g_velocity_pid_controller[pid_index].error_last) / dt;
    
    // 5. 计算总输出 output = P + I + D
    g_velocity_pid_controller[pid_index].output = proportional + integral + derivative;
    
    // 6. 输出限幅 (-100~100 对应 PWM 占空比百分比)
    g_velocity_pid_controller[pid_index].output = VelocityPID_LimitOutput(
        g_velocity_pid_controller[pid_index].output,
        VELOCITY_PID_OUTPUT_MIN,
        VELOCITY_PID_OUTPUT_MAX
    );
    
    // 7. 更新误差历史
    g_velocity_pid_controller[pid_index].error_last = g_velocity_pid_controller[pid_index].error;
    
    // 8. 更新当前速度
    g_velocity_pid_controller[pid_index].current_speed = current_speed;
    
    // 9. 更新目标速度
    g_velocity_pid_controller[pid_index].target_speed = target_speed;
    
    return g_velocity_pid_controller[pid_index].output;
}

/**
 * @brief 获取 PID 控制器输出
 * @param pid_index PID 控制器索引 (0~ENCODER_COUNT-1)
 * @return float PID 输出值 (-100~100)
 */
float VelocityPID_GetOutput(uint8 pid_index)
{
    if (pid_index >= ENCODER_COUNT)
    {
        return 0.0f;
    }
    
    return g_velocity_pid_controller[pid_index].output;
}

/**
 * @brief 获取当前速度误差
 * @param pid_index PID 控制器索引 (0~ENCODER_COUNT-1)
 * @return float 速度误差 (脉冲/秒)
 */
float VelocityPID_GetError(uint8 pid_index)
{
    if (pid_index >= ENCODER_COUNT)
    {
        return 0.0f;
    }
    
    return g_velocity_pid_controller[pid_index].error;
}

/**
 * @brief 重置 PID 控制器状态
 * @param pid_index PID 控制器索引 (0~ENCODER_COUNT-1)
 * @note 清除积分累积和误差历史，用于避免积分饱和或重新启动
 */
void VelocityPID_Reset(uint8 pid_index)
{
    if (pid_index >= ENCODER_COUNT)
    {
        return;
    }
    
    g_velocity_pid_controller[pid_index].error = 0.0f;
    g_velocity_pid_controller[pid_index].error_last = 0.0f;
    g_velocity_pid_controller[pid_index].error_sum = 0.0f;
    g_velocity_pid_controller[pid_index].output = 0.0f;
    g_velocity_pid_controller[pid_index].current_speed = 0.0f;
}

/**
 * @brief 根据 PID 输出执行电机控制
 * @param pid_index PID 控制器索引 (对应电机编号)
 * @param motor 指向电机控制器结构体的指针
 * @note 将 PID 输出 (-100~100) 转换为电机速度控制信号
 */
void VelocityPID_ExecuteMotorControl(uint8 pid_index, DCMotor *motor)
{
    if (pid_index >= ENCODER_COUNT || motor == NULL)
    {
        return;
    }

    if (!g_velocity_pid_controller[pid_index].initialized || 
        !g_velocity_pid_controller[pid_index].enabled)
    {
        DCMotor_Stop(motor);
        return;
    }

    float pid_output = g_velocity_pid_controller[pid_index].output;

    if (pid_output > 100.0f) pid_output = 100.0f;
    if (pid_output < -100.0f) pid_output = -100.0f;

    if (pid_output > 0.0f && pid_output < 1.0f)
    {
        pid_output = 1.0f;
    }
    else if (pid_output < 0.0f && pid_output > -1.0f)
    {
        pid_output = -1.0f;
    }

    DCMotor_SetSpeed(motor, pid_output);
}
