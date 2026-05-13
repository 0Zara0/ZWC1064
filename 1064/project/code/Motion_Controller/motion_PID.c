
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

/** @brief 航向保持使能标志：1=使能（直行时锁定目标偏航角），0=关闭 */
uint8 g_heading_hold_enabled = 0;

/** @brief 航向保持的目标偏航角（单位：度，0~360） */
float g_heading_target = 0.0f;

/** @brief 上一次角度PID修正量缓存（用于IMU未更新周期时保持输出） */
static float g_heading_last_correction = 0.0f;

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
 * @brief 初始化 IMU963RA 六轴惯性测量单元（含磁力计校准和卡尔曼滤波融合）
 * @note IMU 提供加速度、角速度和磁场强度数据，用于姿态解算
 */
void MotionPID_IMU_Init(void)
{
    imu963_init_with_calibration(200, 10);
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

    // 初始化角度环 PID 控制器（航向保持用）
    MotionPID_InitHeadingHold();

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
 * @brief 读取 IMU963RA 九轴传感器数据（从 imu963_data 全局结构体读取物理值）
 * @note imu963_data 由 imu963_get_angle() 内部的 imu963_update() 定期刷新
 */
void MotionPID_ReadIMUData(void)
{
    g_sensor_data.acc_x = imu963_data.acc_x;
    g_sensor_data.acc_y = imu963_data.acc_y;
    g_sensor_data.acc_z = imu963_data.acc_z;
    
    g_sensor_data.gyro_x = imu963_data.gyro_x;
    g_sensor_data.gyro_y = imu963_data.gyro_y;
    g_sensor_data.gyro_z = imu963_data.gyro_z;
    
    g_sensor_data.mag_x = imu963_data.mag_x;
    g_sensor_data.mag_y = imu963_data.mag_y;
    g_sensor_data.mag_z = imu963_data.mag_z;
    
    g_sensor_data.yaw = imu963_data.angle_deg;
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
    g_motor_controller.motor[0].dir_inverted = 1;  // 电机 1 物理安装方向相反

    // 初始化电机 2
    DCMotor_Init(&g_motor_controller.motor[1], MOTOR2_DIR_PIN, MOTOR2_PWM_CH, MOTOR_PWM_FREQ);

    // 初始化电机 3
    DCMotor_Init(&g_motor_controller.motor[2], MOTOR3_DIR_PIN, MOTOR3_PWM_CH, MOTOR_PWM_FREQ);
    g_motor_controller.motor[2].dir_inverted = 1;  // 电机 3 物理安装方向相反

    // 初始化电机 4
    DCMotor_Init(&g_motor_controller.motor[3], MOTOR4_DIR_PIN, MOTOR4_PWM_CH, MOTOR_PWM_FREQ);
    
    // 初始化目标速度数组为 0（停止状态）
    for (uint8 i = 0; i < ENCODER_COUNT; i++)
    {
        g_motor_controller.target_speed[i] = 0.0f;
    }
    
    // 设置初始化标志
    g_motor_controller.initialized = 1;
    
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
    // 初始化 PIT 通道 1，配置定时时间为 SENSOR_UPDATE_PERIOD_MS 毫秒（当前 2ms）
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
 * @note 该函数由 isr.c 中的 PIT_IRQHandler 每 2ms 自动调用一次
 *       执行传感器数据采集、速度解算、PID 控制和电机驱动等实时任务
 *       目标速度由外部直接设置，不再经过 S 曲线加速过渡
 */
void pit_handler(void)
{
    MotionPID_UpdateSensorData();

    float angle_correction = 0.0f;
    if (g_heading_hold_enabled)
    {
        static uint8 imu_divider = 0;
        imu_divider++;
        if (imu_divider >= 5)
        {
            imu_divider = 0;
            float current_angle = imu963_get_angle();
            angle_correction = AnglePID_Calculate(&g_angle_pid_controller, g_heading_target, current_angle, 0.010f);
            g_heading_last_correction = angle_correction;
        }
        else
        {
            angle_correction = g_heading_last_correction;
        }
    }

    for (uint8 i = 0; i < ENCODER_COUNT; i++)
    {
        float current_speed = EncoderSpeedCalc_GetFilteredSpeed(i);

        if (g_motor_controller.motor[i].dir_inverted)
        {
            current_speed = -current_speed;
        }

        float target_speed = g_motor_controller.target_speed[i];

        if (g_heading_hold_enabled && angle_correction != 0.0f)
        {
            if (i == MOTOR_RIGHT_FRONT || i == MOTOR_RIGHT_REAR)
            {
                target_speed -= angle_correction;
            }
            else
            {
                target_speed += angle_correction;
            }
        }

        float pid_output = VelocityPID_Calculate(i, target_speed, current_speed, SENSOR_UPDATE_PERIOD_MS / 1000.0f);
        VelocityPID_ExecuteMotorControl(i, &g_motor_controller.motor[i]);
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
        g_motor_controller.target_speed[motor_index] = target_speed;
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

void MotionPID_InitHeadingHold(void)
{
    AnglePID_Init(&g_angle_pid_controller,
                  ANGLE_PID_KP, ANGLE_PID_KI, ANGLE_PID_KD,
                  ANGLE_PID_OUTPUT_LIMIT, ANGLE_PID_INTEGRAL_LIMIT);
    g_heading_hold_enabled = 0;
    g_heading_target = 0.0f;
    g_heading_last_correction = 0.0f;
}

void MotionPID_EnableHeadingHold(void)
{
    if (!g_angle_pid_controller.initialized)
        return;

    float current_angle = imu963_get_angle();
    g_heading_target = current_angle;
    g_heading_hold_enabled = 1;
    AnglePID_Reset(&g_angle_pid_controller);
}

void MotionPID_DisableHeadingHold(void)
{
    g_heading_hold_enabled = 0;
    AnglePID_Reset(&g_angle_pid_controller);
}




