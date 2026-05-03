
#include "motion_PID.h"
#include "zf_driver_delay.h"
#include "zf_driver_pit.h"
#include "pid_algorithm.h"

// ==================================================== 全局变量定义 ====================================================

/** @brief 全局传感器数据结构体，存储所有传感器采集的数据 */
SensorData_t g_sensor_data = {0};

/** @brief 全局电机控制器结构体，管理所有电机的控制状态 */
MotorController_t g_motor_controller = {0};

/** @brief 系统初始化标志，防止重复初始化（0=未初始化，1=已初始化） */
uint8 g_system_initialized = 0;

// ==================================================== 编码器配置 ====================================================
// 正交编码器与 MCU 引脚的硬件连接定义
// 编码器 1 连接到 QTIMER1_ENCODER1 通道，使用 GPIO 引脚 C0 和 C1
// 编码器 2 连接到 QTIMER1_ENCODER2 通道，使用 GPIO 引脚 C2 和 C24
// 编码器 3 连接到 QTIMER2_ENCODER1 通道，使用 GPIO 引脚 C3 和 C4
// 编码器 4 连接到 QTIMER2_ENCODER2 通道，使用 GPIO 引脚 C5 和 C25

#define ENCODER1_CH1_PIN    QTIMER1_ENCODER1_CH1_C0
#define ENCODER1_CH2_PIN    QTIMER1_ENCODER1_CH2_C1
#define ENCODER2_CH1_PIN    QTIMER1_ENCODER2_CH1_C2
#define ENCODER2_CH2_PIN    QTIMER1_ENCODER2_CH2_C24
#define ENCODER3_CH1_PIN    QTIMER2_ENCODER1_CH1_C3
#define ENCODER3_CH2_PIN    QTIMER2_ENCODER1_CH2_C4
#define ENCODER4_CH1_PIN    QTIMER2_ENCODER2_CH1_C5
#define ENCODER4_CH2_PIN    QTIMER2_ENCODER2_CH2_C25

// ==================================================== 电机配置 ======================================================
// 直流电机与 MCU 引脚的硬件连接定义
// 电机 1 方向控制引脚为 C7，PWM 输出通道为 PWM2_MODULE0_CHA_C6
// 电机 2 方向控制引脚为 C9，PWM 输出通道为 PWM2_MODULE1_CHA_C8
// 电机 3 方向控制引脚为 C11，PWM 输出通道为 PWM2_MODULE2_CHA_C10
// 电机 4 方向控制引脚为 D3，PWM 输出通道为 PWM2_MODULE3_CHA_D2

#define MOTOR1_DIR_PIN      C7
#define MOTOR1_PWM_CH       PWM2_MODULE0_CHA_C6
#define MOTOR2_DIR_PIN      C9
#define MOTOR2_PWM_CH       PWM2_MODULE1_CHA_C8
#define MOTOR3_DIR_PIN      C11
#define MOTOR3_PWM_CH       PWM2_MODULE2_CHA_C10
#define MOTOR4_DIR_PIN      D3
#define MOTOR4_PWM_CH       PWM2_MODULE3_CHA_D2

#define MOTOR_PWM_FREQ      17000   // PWM 信号工作频率设置为 17kHz

// ==================================================== 定时器相关变量 ================================================

/** @brief 定时器上次更新的时间戳计数器，单位为毫秒，用于记录控制周期时间 */
static uint32 g_timer_last_count = 0;

// ==================================================== 函数实现 ======================================================

/**
 * @brief 初始化正交编码器模块
 * @note 配置 4 个 QTIMER 定时器通道为正交编码器模式
 *       用于采集电机转速和位置信息，提供速度反馈和位置反馈
 */
void MotionPID_Encoder_Init(void)
{
    // 初始化编码器 1 的正交解码通道，配置 A/B 相输入引脚
    encoder_quad_init(QTIMER1_ENCODER1, ENCODER1_CH1_PIN, ENCODER1_CH2_PIN);
    encoder_quad_init(QTIMER1_ENCODER2, ENCODER2_CH1_PIN, ENCODER2_CH2_PIN);
    encoder_quad_init(QTIMER2_ENCODER1, ENCODER3_CH1_PIN, ENCODER3_CH2_PIN);
    encoder_quad_init(QTIMER2_ENCODER2, ENCODER4_CH1_PIN, ENCODER4_CH2_PIN);
}

/**
 * @brief 初始化 IMU963RA 六轴惯性测量单元
 * @return uint8 初始化状态码，0 表示初始化成功，非 0 表示失败
 * @note IMU 提供加速度、角速度和磁场强度数据，用于姿态解算
 */
void MotionPID_IMU_Init(void)
{
    imu963ra_init();
}

/**
 * @brief 传感器系统总初始化函数
 * @note 按顺序初始化编码器和 IMU 两大类传感器
 *       具有防重复初始化机制，多次调用只执行一次
 */
void MotionPID_Sensor_Init(void)
{
    // 防止重复初始化：如果已经初始化过，直接返回
    if (g_system_initialized)
    {
        return;
    }
    
    // 调用编码器初始化函数，配置 4 个正交编码器通道
    MotionPID_Encoder_Init();
    
    // 调用 IMU963RA 初始化函数，配置六轴传感器参数
    MotionPID_IMU_Init();
    
    // 初始化编码器速度计算器，为后续速度解算做准备
    EncoderSpeedCalc_Init();
    
    // 初始化速度环 PID 控制器，加载默认 PID 参数配置
    VelocityPID_InitAll();
    
    // 延时 100ms 等待传感器上电稳定并完成内部自检
    system_delay_ms(100);
}

/**
 * @brief 读取所有编码器通道的计数值
 * @note 将 4 个编码器的原始计数数据存储到全局传感器数据结构体 g_sensor_data 中
 */
void MotionPID_ReadEncoderData(void)
{
    // 从 QTIMER1_ENCODER1 通道读取编码器 1 的脉冲计数值
    g_sensor_data.encoder_speed[0] = encoder_get_count(QTIMER1_ENCODER1);
    g_sensor_data.encoder_speed[1] = encoder_get_count(QTIMER1_ENCODER2);
    g_sensor_data.encoder_speed[2] = encoder_get_count(QTIMER2_ENCODER1);
    g_sensor_data.encoder_speed[3] = encoder_get_count(QTIMER2_ENCODER2);
}

/**
 * @brief 读取 IMU963RA 九轴传感器数据
 * @note 依次读取加速度计、陀螺仪和磁力计数据并存储到全局结构体
 */
void MotionPID_ReadIMUData(void)
{
    // 调用底层驱动读取三轴加速度原始数据
    imu963ra_get_acc();
    g_sensor_data.acc_x = imu963ra_acc_transition(imu963ra_acc_x);
    g_sensor_data.acc_y = imu963ra_acc_transition(imu963ra_acc_y);
    g_sensor_data.acc_z = imu963ra_acc_transition(imu963ra_acc_z);
    
    // 读取陀螺仪数据
    imu963ra_get_gyro();
    g_sensor_data.gyro_x = imu963ra_gyro_transition(imu963ra_gyro_x);
    g_sensor_data.gyro_y = imu963ra_gyro_transition(imu963ra_gyro_y);
    g_sensor_data.gyro_z = imu963ra_gyro_transition(imu963ra_gyro_z);
    
    // 读取磁力计数据
    imu963ra_get_mag();
    g_sensor_data.mag_x = imu963ra_mag_transition(imu963ra_mag_x);
    g_sensor_data.mag_y = imu963ra_mag_transition(imu963ra_mag_y);
    g_sensor_data.mag_z = imu963ra_mag_transition(imu963ra_mag_z);
}

/**
 * @brief 更新所有传感器数据的综合函数
 * @note 周期性调用此函数刷新全局传感器数据
 *       自动完成编码器速度解算和数字滤波处理
 */
void MotionPID_UpdateSensorData(void)
{
    // 第一步：采集原始传感器数据，包括编码器计数和 IMU 九轴数据
    MotionPID_ReadEncoderData();
    MotionPID_ReadIMUData();
    
    // 第二步：执行编码器速度解算和滤波处理核心算法
    // 将编码器脉冲计数转换为速度单位（脉冲/秒），应用一阶低通滤波器平滑数据
    EncoderSpeedCalc_Update(g_sensor_data.encoder_speed, ENCODER_COUNT, SENSOR_UPDATE_PERIOD_MS / 1000.0f);
}

/**
 * @brief 初始化 4 个直流电机实例
 * @note 根据预定义的引脚配置逐一初始化每个电机
 *       具有防重复初始化机制，多次调用只执行一次
 */
void MotionPID_Motor_Init(void)
{
    // 防止重复初始化：如果已经初始化过，直接返回
    if (g_motor_controller.initialized)
    {
        return;
    }
    
    // 配置电机 1 的方向引脚和 PWM 通道，设置 PWM 频率
    DCMotor_Init(&g_motor_controller.motor[0], MOTOR1_DIR_PIN, MOTOR1_PWM_CH, MOTOR_PWM_FREQ);
    
    // 初始化电机 2
    DCMotor_Init(&g_motor_controller.motor[1], MOTOR2_DIR_PIN, MOTOR2_PWM_CH, MOTOR_PWM_FREQ);
    
    // 初始化电机 3
    DCMotor_Init(&g_motor_controller.motor[2], MOTOR3_DIR_PIN, MOTOR3_PWM_CH, MOTOR_PWM_FREQ);
    
    // 初始化电机 4
    DCMotor_Init(&g_motor_controller.motor[3], MOTOR4_DIR_PIN, MOTOR4_PWM_CH, MOTOR_PWM_FREQ);
    
    // 初始化目标速度数组为 0（停止状态）
    for (uint8 i = 0; i < ENCODER_COUNT; i++)
    {
        g_motor_controller.target_speed[i] = 0.0f;
        g_motor_controller.requested_speed[i] = 0.0f;
        g_motor_controller.ramp_start_speed[i] = 0.0f;
        g_motor_controller.ramp_cycle_count[i] = 0;
    }
    
    // 设置初始化标志
    g_motor_controller.initialized = 1;
    
    // 默认启用闭环模式，但允许切换到开环
    g_motor_controller.open_loop_mode = 0;
    
    // 添加延时确保 PWM 完全启动稳定
    system_delay_ms(10);
}

/**
 * @brief 初始化 PIT 定时器用于周期性传感器数据更新
 * @note 配置 PIT_CH1 通道，设定中断周期为 SENSOR_UPDATE_PERIOD_MS 毫秒
 *       定时器启动后会自动产生周期性中断，触发数据采集任务
 */
void MotionPID_Timer_Init(void)
{
    // 初始化 PIT 通道 1，配置定时时间为 SENSOR_UPDATE_PERIOD_MS 毫秒（默认 5ms）
    pit_ms_init(SENSOR_TIMER_CH, SENSOR_UPDATE_PERIOD_MS);
    
    // 启动 PIT 定时器开始计时
    pit_enable(SENSOR_TIMER_CH);
    
    // 清除可能存在的历史中断标志位，避免误触发
    pit_flag_clear(SENSOR_TIMER_CH);
    
    // 初始化时间戳计数器为 0，作为第一次定时的基准点
    g_timer_last_count = 0;
    
    // 标记系统已完成所有初始化
    g_system_initialized = 1;
}


/**
 * @brief 重置 PIT 定时器计时器
 * @note 强制定时器重新开始计时，用于同步多个控制任务的执行周期
 */
void MotionPID_ResetTimer(void)
{
    // 停止 PIT 定时器运行
    pit_disable(SENSOR_TIMER_CH);
    
    // 清除定时器溢出标志位
    pit_flag_clear(SENSOR_TIMER_CH);
    
    // 重新启动 PIT 定时器开始新的计时周期
    pit_enable(SENSOR_TIMER_CH);
    
    // 将时间戳计数器清零，重新从 0 开始计时
    g_timer_last_count = 0;
}


/**
 * @brief PIT 定时器中断服务回调函数
 * @note 该函数由 isr.c 中的 PIT_IRQHandler 每 5ms 自动调用一次
 *       执行传感器数据采集、速度解算、PID 控制和电机驱动等实时任务
 */
void pit_handler(void)
{
    // 第一步：读取并更新所有传感器数据到全局变量 g_sensor_data
    MotionPID_UpdateSensorData();

    // 第二步：更新 S 曲线加速，实现电机平滑启动/停止
    // 使用 smoothstep 函数: f(x) = 3x² - 2x³，在 x=0 和 x=1 处导数均为零
    for (uint8 i = 0; i < ENCODER_COUNT; i++)
    {
        if (g_motor_controller.ramp_cycle_count[i] > 0)
        {
            float progress = (float)g_motor_controller.ramp_cycle_count[i] / MOTION_ACCEL_CYCLES;

            if (progress >= 1.0f)
            {
                g_motor_controller.target_speed[i] = g_motor_controller.requested_speed[i];
                g_motor_controller.ramp_cycle_count[i] = 0;
            }
            else
            {
                float s_curve = (3.0f - 2.0f * progress) * progress * progress;
                float speed_diff = g_motor_controller.requested_speed[i] - g_motor_controller.ramp_start_speed[i];
                g_motor_controller.target_speed[i] = g_motor_controller.ramp_start_speed[i] + speed_diff * s_curve;
                g_motor_controller.ramp_cycle_count[i]++;
            }
        }
    }

    // 第三步：根据控制模式执行不同的控制策略
    if (g_motor_controller.open_loop_mode)
    {
        // 开环模式：直接将目标速度转换为PWM输出，不使用PID
        for (uint8 i = 0; i < ENCODER_COUNT; i++)
        {
            float target_speed = g_motor_controller.target_speed[i];
            
            // 将目标速度（脉冲/秒）线性映射到PWM占空比（-100~100）
            // 假设最大速度对应100%占空比，可根据实际情况调整比例系数
            // 示例：10脉冲/秒 = 1%占空比，即 1000脉冲/秒 = 100%占空比
            float pwm_duty = target_speed / 10.0f;
            
            // 限制PWM范围在 -100~100
            if (pwm_duty > 100.0f) pwm_duty = 100.0f;
            if (pwm_duty < -100.0f) pwm_duty = -100.0f;
            
            // 直接设置电机速度（开环控制）
            DCMotor_SetSpeed(&g_motor_controller.motor[i], pwm_duty);
        }
    }
    else
    {
        // 闭环模式：执行速度闭环 PID 控制算法
        for (uint8 i = 0; i < ENCODER_COUNT; i++)
        {
            // 获取经过滤波处理后的当前实际速度值
            float current_speed = EncoderSpeedCalc_GetFilteredSpeed(i);
            
            // 获取目标速度
            float target_speed = g_motor_controller.target_speed[i];
            
            // 调用速度环 PID 计算函数，计算控制量输出
            float pid_output = VelocityPID_Calculate(i, target_speed, current_speed, SENSOR_UPDATE_PERIOD_MS / 1000.0f);
            
            // 根据 PID 运算结果驱动对应电机运转
            VelocityPID_ExecuteMotorControl(i, &g_motor_controller.motor[i]);
        }
    }
}

/**
 * @brief 设置单个电机的目标速度
 * @param motor_index 电机索引（0-3 对应 4 个电机）
 * @param target_speed 目标速度值，单位：脉冲/秒
 *                   正值表示正转，负值表示反转
 * @note 示例：MotionPID_SetTargetSpeed(0, 100.0f);  // 设置电机 1 以 100 脉冲/秒正转
 *           MotionPID_SetTargetSpeed(1, -50.0f);  // 设置电机 2 以 50 脉冲/秒反转
 */
void MotionPID_SetTargetSpeed(uint8 motor_index, float target_speed)
{
    if (motor_index < ENCODER_COUNT)
    {
        if (g_motor_controller.requested_speed[motor_index] != target_speed)
        {
            g_motor_controller.ramp_start_speed[motor_index] = g_motor_controller.target_speed[motor_index];
            g_motor_controller.requested_speed[motor_index] = target_speed;
            g_motor_controller.ramp_cycle_count[motor_index] = 1;
        }
    }
}

/**
 * @brief 设置所有电机的目标速度
 * @param target_speed 目标速度值，单位：脉冲/秒
 *                   正值表示正转，负值表示反转
 * @note 示例：MotionPID_SetAllMotorsSpeed(100.0f);  // 所有电机以 100 脉冲/秒正转
 */
void MotionPID_SetAllMotorsSpeed(float target_speed)
{
    for (uint8 i = 0; i < ENCODER_COUNT; i++)
    {
        MotionPID_SetTargetSpeed(i, target_speed);
    }
}

/**
 * @brief 获取单个电机的实际运行速度
 * @param motor_index 电机索引（0-3 对应 4 个电机）
 * @return float 实际速度值，单位：脉冲/秒
 *              正值表示正转，负值表示反转
 * @note 示例：float speed = MotionPID_GetActualSpeed(0);  // 获取电机 1 的实际速度
 */
float MotionPID_GetActualSpeed(uint8 motor_index)
{
    if (motor_index < ENCODER_COUNT)
    {
        return EncoderSpeedCalc_GetFilteredSpeed(motor_index);
    }
    return 0.0f;
}

/**
 * @brief 设置开环/闭环控制模式
 * @param enable_open_loop 1=启用开环模式，0=启用闭环模式
 * @note 开环模式下电机直接根据目标速度运行，不依赖编码器反馈
 *       适用于编码器故障或未连接的情况
 *       切换时会自动重置PID控制器状态，避免积分饱和
 */
void MotionPID_SetOpenLoopMode(uint8 enable_open_loop)
{
    g_motor_controller.open_loop_mode = enable_open_loop;
    
    // 切换到开环模式时，重置所有PID控制器以避免积分饱和
    if (enable_open_loop)
    {
        for (uint8 i = 0; i < ENCODER_COUNT; i++)
        {
            VelocityPID_Reset(i);
        }
    }
}

/**
 * @brief 获取当前控制模式
 * @return uint8 1=开环模式，0=闭环模式
 */
uint8 MotionPID_GetOpenLoopMode(void)
{
    return g_motor_controller.open_loop_mode;
}




