#include "motion_PID.h"
#include "zf_driver_delay.h"
#include "zf_driver_pit.h"
#include "pid_algorithm.h"
#include "imu963.h"
// ==================================================== 全局变量定义 ====================================================

/** @brief 全局传感器数据结构体，存储所有传感器采集的数据 */
SensorData_t g_sensor_data = {0};

/** @brief 全局电机控制器结构体，管理所有电机的控制状态 */
MotorController_t g_motor_controller = {0};

/** @brief 系统初始化标志，防止重复初始化（0=未初始化，1=已初始化） */
uint8 g_system_initialized = 0;

/** @brief 航向保持使能标志：1=使能（直行/平移时锁定目标偏航角），0=关闭 */
uint8 g_heading_hold_enabled = 0;

/** @brief 航向保持的目标偏航角（单位：度，0~360） */
float g_heading_target = 0.0f;

/** @brief 角度环输出的目标角速度，供角速度环消费（单位：°/s） */
float g_heading_target_rate = 0.0f;

/** @brief 上一次 pit_handler 计算的航向修正量，供 MoveMode_DistanceUpdate 恢复（单位：脉冲/秒） */
static float g_heading_correction_result = 0.0f;

// ==================================================== 编码器配置 ====================================================
// 正交编码器与 MCU 引脚的硬件连接定义
// ENC0(RF) → QTIMER2_ENCODER1, C3/C4
// ENC1(LF) → QTIMER1_ENCODER2, C2/C24
// ENC2(RR) → QTIMER2_ENCODER2, C5/C25
// ENC3(LR) → QTIMER1_ENCODER1, C0/C1

#define ENCODER0_CH1_PIN    QTIMER2_ENCODER1_CH1_C3
#define ENCODER0_CH2_PIN    QTIMER2_ENCODER1_CH2_C4
#define ENCODER1_CH1_PIN    QTIMER1_ENCODER2_CH1_C2
#define ENCODER1_CH2_PIN    QTIMER1_ENCODER2_CH2_C24
#define ENCODER2_CH1_PIN    QTIMER2_ENCODER2_CH1_C5
#define ENCODER2_CH2_PIN    QTIMER2_ENCODER2_CH2_C25
#define ENCODER3_CH1_PIN    QTIMER1_ENCODER1_CH1_C0
#define ENCODER3_CH2_PIN    QTIMER1_ENCODER1_CH2_C1

// ==================================================== 电机配置 ======================================================
// 直流电机与 MCU 引脚的硬件连接定义
// 电机方向：RF(0)和RR(2)方向取反，LF(1)和LR(3)不取反（根据实际接线测试确认）
// MOTOR0(RF): DIR=C11, PWM=PWM2_MODULE2_CHA_C10
// MOTOR1(LF): DIR=D3, PWM=PWM2_MODULE3_CHA_D2
// MOTOR2(RR): DIR=C9, PWM=PWM2_MODULE1_CHA_C8
// MOTOR3(LR): DIR=C7, PWM=PWM2_MODULE0_CHA_C6

#define MOTOR0_DIR_PIN      C11
#define MOTOR0_PWM_CH       PWM2_MODULE2_CHA_C10
#define MOTOR1_DIR_PIN      D3
#define MOTOR1_PWM_CH       PWM2_MODULE3_CHA_D2
#define MOTOR2_DIR_PIN      C9
#define MOTOR2_PWM_CH       PWM2_MODULE1_CHA_C8
#define MOTOR3_DIR_PIN      C7
#define MOTOR3_PWM_CH       PWM2_MODULE0_CHA_C6

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
    encoder_quad_init(QTIMER2_ENCODER1, ENCODER0_CH1_PIN, ENCODER0_CH2_PIN);
    encoder_quad_init(QTIMER1_ENCODER2, ENCODER1_CH1_PIN, ENCODER1_CH2_PIN);
    encoder_quad_init(QTIMER2_ENCODER2, ENCODER2_CH1_PIN, ENCODER2_CH2_PIN);
    encoder_quad_init(QTIMER1_ENCODER1, ENCODER3_CH1_PIN, ENCODER3_CH2_PIN);
}

/**
 * @brief 初始化 IMU963RA 六轴惯性测量单元（含磁力计校准和卡尔曼滤波融合）
 * @note IMU 提供加速度、角速度和磁场强度数据，用于姿态解算
 */
void MotionPID_IMU_Init(void)
{
    //imu963_init_with_calibration(200, 10);
}

/**
 * @brief 传感器系统总初始化函数
 * @note 按顺序初始化编码器和 IMU 两大类传感器
 *       具有防重复初始化机制，多次调用只执行一次
 */
void MotionPID_Sensor_Init(void)
{
    if (g_system_initialized)
    {
        return;
    }

    MotionPID_Encoder_Init();
    MotionPID_IMU_Init();

    EncoderSpeedCalc_Init();
    VelocityPID_InitAll();

    MotionPID_InitHeadingHold();

    system_delay_ms(100);
}

/**
 * @brief 读取所有编码器通道的计数值
 * @note 将 4 个编码器的原始计数数据存储到全局传感器数据结构体 g_sensor_data 中
 */
void MotionPID_ReadEncoderData(void)
{
    g_sensor_data.encoder_speed[0] = encoder_get_count(QTIMER2_ENCODER1);
    g_sensor_data.encoder_speed[1] = encoder_get_count(QTIMER1_ENCODER2);
    g_sensor_data.encoder_speed[2] = encoder_get_count(QTIMER2_ENCODER2);
    g_sensor_data.encoder_speed[3] = encoder_get_count(QTIMER1_ENCODER1);
}

/**
 * @brief 读取 IMU963RA 六轴传感器数据（从 imu963_data 全局结构体读取物理值，磁力计已禁用）
 * @note imu963_data 由 imu_timer_handler (PIT_CH1) 每 1ms 调用 imu963_get_angle() 定期刷新
 */
void MotionPID_ReadIMUData(void)
{
    g_sensor_data.acc_x = imu963_data.acc_x;
    g_sensor_data.acc_y = imu963_data.acc_y;
    g_sensor_data.acc_z = imu963_data.acc_z;

    g_sensor_data.gyro_x = imu963_data.gyro_x;
    g_sensor_data.gyro_y = imu963_data.gyro_y;
    g_sensor_data.gyro_z = imu963_data.gyro_z;

    g_sensor_data.yaw = imu963_data.angle_deg;
}

/**
 * @brief 更新所有传感器数据的综合函数
 * @note 周期性调用此函数刷新全局传感器数据
 *       自动完成编码器速度解算和数字滤波处理
 */
void MotionPID_UpdateSensorData(void)
{
    MotionPID_ReadEncoderData();
    MotionPID_ReadIMUData();

    EncoderSpeedCalc_Update(
        g_sensor_data.encoder_speed,
        ENCODER_COUNT,
        SENSOR_UPDATE_PERIOD_MS / 1000.0f
    );
}

/**
 * @brief 初始化 4 个直流电机实例
 * @note 根据预定义的引脚配置逐一初始化每个电机
 *       具有防重复初始化机制，多次调用只执行一次
 */
void MotionPID_Motor_Init(void)
{
    if (g_motor_controller.initialized)
    {
        return;
    }

    DCMotor_Init(&g_motor_controller.motor[0], MOTOR0_DIR_PIN, MOTOR0_PWM_CH, MOTOR_PWM_FREQ);
    g_motor_controller.motor[0].dir_inverted = 1;

    DCMotor_Init(&g_motor_controller.motor[1], MOTOR1_DIR_PIN, MOTOR1_PWM_CH, MOTOR_PWM_FREQ);

    DCMotor_Init(&g_motor_controller.motor[2], MOTOR2_DIR_PIN, MOTOR2_PWM_CH, MOTOR_PWM_FREQ);
    g_motor_controller.motor[2].dir_inverted = 1;

    DCMotor_Init(&g_motor_controller.motor[3], MOTOR3_DIR_PIN, MOTOR3_PWM_CH, MOTOR_PWM_FREQ);

    for (uint8 i = 0; i < ENCODER_COUNT; i++)
    {
        g_motor_controller.target_speed[i] = 0.0f;
    }

    g_motor_controller.initialized = 1;

    system_delay_ms(10);
}

/**
 * @brief 初始化 PIT 定时器用于周期性传感器数据更新
 * @note 配置 PIT 通道，设定中断周期为 SENSOR_UPDATE_PERIOD_MS 毫秒
 */
void MotionPID_Timer_Init(void)
{
    pit_ms_init(SENSOR_TIMER_CH, SENSOR_UPDATE_PERIOD_MS);
    pit_enable(SENSOR_TIMER_CH);
    pit_flag_clear(SENSOR_TIMER_CH);

    g_timer_last_count = 0;

    g_system_initialized = 1;
}

/**
 * @brief 重置 PIT 定时器计时器
 * @note 强制定时器重新开始计时，用于同步多个控制任务的执行周期
 */
void MotionPID_ResetTimer(void)
{
    pit_disable(SENSOR_TIMER_CH);
    pit_flag_clear(SENSOR_TIMER_CH);
    pit_enable(SENSOR_TIMER_CH);

    g_timer_last_count = 0;
}

/**
 * @brief 初始化 IMU 专用 PIT 定时器（独立于电机控制中断）
 * @note 使用 PIT_CH1，周期 1ms，由 isr.c 中的 PIT_IRQHandler 调用 imu_timer_handler
 */
void MotionPID_IMU_Timer_Init(void)
{
    pit_ms_init(IMU_TIMER_CH, SENSOR_UPDATE_PERIOD_MS);
    pit_enable(IMU_TIMER_CH);
    pit_flag_clear(IMU_TIMER_CH);
    imu963_set_dt_ms((float)SENSOR_UPDATE_PERIOD_MS);
}

/**
 * @brief IMU 定时器中断服务回调函数
 * @note 由 isr.c 中的 PIT_IRQHandler(PIT_CH1) 每 1ms 调用一次
 *       读取九轴传感器原始数据、计算磁力计偏航角、执行 Kalman 融合滤波
 *       完成后同步更新 g_sensor_data，供 pit_handler 消费
 */
void imu_timer_handler(void)
{
    imu963_get_angle();
    MotionPID_ReadIMUData();
}

/**
 * @brief PIT 定时器中断服务回调函数（1kHz 级联 PID 控制）
 * @note 该函数由 isr.c 中的 PIT_IRQHandler(PIT_CH0) 每 1ms 自动调用一次
 *       角度环 (AnglePID) → 角速度环 (AngularRatePID) → 速度环 (VelocityPID×4)
 */
void pit_handler(void)
{
    MotionPID_UpdateSensorData();

    float angle_correction = 0.0f;

    if (g_heading_hold_enabled && g_angle_pid_controller.enabled)
    {
        g_heading_target_rate = AnglePID_Calculate(
            &g_angle_pid_controller,
            g_heading_target,
            g_sensor_data.yaw,
            SENSOR_UPDATE_PERIOD_MS / 1000.0f
        );
    }

    if (g_heading_hold_enabled && g_angular_rate_pid.enabled)
    {
        angle_correction = AngularRatePID_Calculate(
            &g_angular_rate_pid,
            g_heading_target_rate,
            g_sensor_data.gyro_z,
            SENSOR_UPDATE_PERIOD_MS / 1000.0f
        );
    }

    g_heading_correction_result = angle_correction;

    for (uint8 i = 0; i < ENCODER_COUNT; i++)
    {
        float current_speed = MotionPID_GetActualSpeed(i);
        float target_speed = g_motor_controller.target_speed[i];

        // 航向保持：右侧轮减，左侧轮加，形成旋转修正
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

        VelocityPID_Calculate(
            i,
            target_speed,
            current_speed,
            SENSOR_UPDATE_PERIOD_MS / 1000.0f
        );
        VelocityPID_ExecuteMotorControl(i, &g_motor_controller.motor[i]);
    }
}

/**
 * @brief 设置单个电机的目标速度
 * @param motor_index 电机索引（0-3 对应 4 个电机）
 * @param target_speed 目标速度值，单位：脉冲/秒
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
 */
void MotionPID_SetAllMotorsSpeed(float target_speed)
{
    for (uint8 i = 0; i < ENCODER_COUNT; i++)
    {
        MotionPID_SetTargetSpeed(i, target_speed);
    }
}

/**
 * @brief 设置麦克纳姆底盘速度
 * @param vx 前后速度分量，正值=前进，负值=后退
 * @param vy 左右速度分量，正值=左移，负值=右移
 */
void MotionPID_SetMecanumSpeed(float vx, float vy)
{
		if(vy>1.0f)//向左
		{
    MotionPID_SetTargetSpeed(MOTOR_RIGHT_FRONT, (vx + vy));
    MotionPID_SetTargetSpeed(MOTOR_LEFT_FRONT,  (vx - vy));
    MotionPID_SetTargetSpeed(MOTOR_RIGHT_REAR,  (vx - vy));
    MotionPID_SetTargetSpeed(MOTOR_LEFT_REAR,   (vx + vy));
		}
		
		else if(vy<-1.0f)//向右
		{
    MotionPID_SetTargetSpeed(MOTOR_RIGHT_FRONT, (vx + vy));
    MotionPID_SetTargetSpeed(MOTOR_LEFT_FRONT,  (vx - vy));
    MotionPID_SetTargetSpeed(MOTOR_RIGHT_REAR,  (vx - vy));
    MotionPID_SetTargetSpeed(MOTOR_LEFT_REAR,   (vx + vy));
		}
		
		else if(vy==0)
		{
		MotionPID_SetTargetSpeed(MOTOR_RIGHT_FRONT, (vx + vy));
    MotionPID_SetTargetSpeed(MOTOR_LEFT_FRONT,  vx - vy);
    MotionPID_SetTargetSpeed(MOTOR_RIGHT_REAR,  (vx - vy));
    MotionPID_SetTargetSpeed(MOTOR_LEFT_REAR,   vx + vy);
		}
		else
		{
    MotionPID_SetTargetSpeed(MOTOR_RIGHT_FRONT, (vx + vy));
    MotionPID_SetTargetSpeed(MOTOR_LEFT_FRONT,  (vx - vy));
    MotionPID_SetTargetSpeed(MOTOR_RIGHT_REAR,  (vx - vy));
    MotionPID_SetTargetSpeed(MOTOR_LEFT_REAR,   (vx + vy));
		}
}

/**
 * @brief 获取单个电机的方向归一化实际运行速度
 * @param motor_index 电机索引（0-3 对应 4 个电机）
 * @return float 实际速度值，单位：脉冲/秒
 */
float MotionPID_GetActualSpeed(uint8 motor_index)
{
    if (motor_index < ENCODER_COUNT)
    {
        float speed = EncoderSpeedCalc_GetFilteredSpeed(motor_index);

        if (g_motor_controller.motor[motor_index].dir_inverted)
        {
            speed = -speed;
        }

        return speed;
    }

    return 0.0f;
}

void MotionPID_InitHeadingHold(void)
{
    AnglePID_Init(
        &g_angle_pid_controller,
        ANGLE_PID_KP,
        ANGLE_PID_KI,
        ANGLE_PID_OUTPUT_LIMIT,
        ANGLE_PID_INTEGRAL_LIMIT
    );

    AngularRatePID_Init(
        &g_angular_rate_pid,
        ANGULAR_RATE_PID_KP,
        ANGULAR_RATE_PID_KI,
        ANGULAR_RATE_PID_KD,
        ANGULAR_RATE_PID_OUTPUT_LIMIT,
        ANGULAR_RATE_PID_INTEGRAL_LIMIT
    );

    g_heading_hold_enabled = 0;
    g_heading_target = 0.0f;
    g_heading_target_rate = 0.0f;
}

/**
 * @brief 开启航向保持
 * @note 只有从关闭切换到开启时才锁定当前 yaw；
 *       已经开启时重复调用不会刷新目标角，避免距离环每次更新都重置航向目标。
 */
void MotionPID_EnableHeadingHold(void)
{
    if (!g_angle_pid_controller.initialized)
    {
        return;
    }

    if (!g_heading_hold_enabled)
    {
        float current_angle = imu963_data.angle_deg;

        g_heading_target = current_angle;
        g_heading_target_rate = 0.0f;
        AnglePID_Reset(&g_angle_pid_controller);
        g_angle_pid_controller.enabled = 1;
        AngularRatePID_Reset(&g_angular_rate_pid);
        g_angular_rate_pid.enabled = 1;
    }

    g_heading_hold_enabled = 1;
}

void MotionPID_DisableHeadingHold(void)
{
    g_heading_hold_enabled = 0;
    AnglePID_Reset(&g_angle_pid_controller);
    g_angle_pid_controller.enabled = 0;
    AngularRatePID_Reset(&g_angular_rate_pid);
    g_angular_rate_pid.enabled = 0;
    g_heading_correction_result = 0.0f;
}

void MotionPID_ApplyHeadingCorrection(void)
{
    if (!g_heading_hold_enabled)
        return;
    if (g_heading_correction_result == 0.0f)
        return;

    for (uint8 i = 0; i < ENCODER_COUNT; i++)
    {
        if (i == MOTOR_RIGHT_FRONT || i == MOTOR_RIGHT_REAR)
        {
            g_motor_controller.target_speed[i] -= g_heading_correction_result;
        }
        else
        {
            g_motor_controller.target_speed[i] += g_heading_correction_result;
        }
    }
}
