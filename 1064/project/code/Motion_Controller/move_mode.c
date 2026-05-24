#include "move_mode.h"
#include "motion_PID.h"
#include "pid_algorithm.h"
#include "zf_driver_delay.h"
#include "zf_driver_encoder.h"
#include <string.h>
#include <stdlib.h>

// 外部声明系统初始化标志
extern uint8 g_system_initialized;

// ==================================================== 距离移动状态变量 ====================================================

/** @brief 当前距离移动状态 */
static MoveState_t g_move_state = MOVE_STATE_IDLE;

/** @brief 当前移动模式 */
static MoveMode_t g_current_mode = STOP;

/** @brief 目标移动距离（脉冲数） */
static int32 g_target_pulse_count = 0;

/** @brief 移动开始时的编码器基准值 */
static int16 g_start_encoder_count[ENCODER_COUNT] = {0};

/** @brief 编码器累积位置（int32，已处理 int16 溢出），初始为 0 */
static int32 g_unwrapped_count[ENCODER_COUNT] = {0};

/** @brief 上次编码器原始读数（int16），用于增量计算 */
static int16 g_last_raw_count[ENCODER_COUNT] = {0};

/** @brief 当前移动速度 */
static float g_move_speed = 0.0f;

// ==================================================== 内部工具函数 ====================================================

static int32 MoveMode_AbsInt32(int32 value)
{
    return (value < 0) ? -value : value;
}

static float MoveMode_LimitFloat(float value, float min_value, float max_value)
{
    if (value > max_value)
    {
        return max_value;
    }
    if (value < min_value)
    {
        return min_value;
    }
    return value;
}

/**
 * @brief 平移时加入前后偏移修正
 * @param mode STRAFE_LEFT 或 STRAFE_RIGHT
 * @param strafe_speed 平移速度，正值
 * @param forward_pos 已累计的前后串动位置，正值=向前串，负值=向后串
 */
static void MoveMode_SetStrafeSpeedWithForwardCorrection(MoveMode_t mode, float strafe_speed, int32 forward_pos)
{
    float vy = 0.0f;
    float vx_correction = 0.0f;

    if (mode == STRAFE_LEFT)
    {
        vy = strafe_speed;
    }
    else if (mode == STRAFE_RIGHT)
    {
        vy = -strafe_speed;
    }
    else
    {
        return;
    }

    // forward_pos > 0 表示平移过程中向前串，需要叠加后退速度分量
    // forward_pos < 0 表示向后串，需要叠加前进速度分量
    if (MoveMode_AbsInt32(forward_pos) > MOVE_STRAFE_FORWARD_DEADBAND)
    {
        vx_correction = -MOVE_STRAFE_FORWARD_CORRECT_KP * (float)forward_pos;
        vx_correction = MoveMode_LimitFloat(
            vx_correction,
            -MOVE_STRAFE_FORWARD_CORRECT_LIMIT,
            MOVE_STRAFE_FORWARD_CORRECT_LIMIT
        );
    }

    MotionPID_SetMecanumSpeed(vx_correction, vy);
}

// ==================================================== 基础运动控制函数 ====================================================

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

    // 初始化距离移动状态
    g_move_state = MOVE_STATE_IDLE;
    g_current_mode = STOP;
    g_target_pulse_count = 0;
    g_move_speed = 0.0f;
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
            MotionPID_EnableHeadingHold();
            MotionPID_SetMecanumSpeed(speed, 0.0f);
            break;

        case BACKWARD:
            MotionPID_EnableHeadingHold();
            MotionPID_SetMecanumSpeed(-speed, 0.0f);
            break;

        case STRAFE_LEFT:
            // 平移也建议打开航向保持，避免车头角度变化造成斜走
            MotionPID_EnableHeadingHold();
            MotionPID_SetMecanumSpeed(0.0f, speed);
            break;

        case STRAFE_RIGHT:
            MotionPID_EnableHeadingHold();
            MotionPID_SetMecanumSpeed(0.0f, -speed);
            break;

        case STOP:
        default:
            MotionPID_DisableHeadingHold();
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
    for (uint8 i = 0; i < ENCODER_COUNT; i++)
    {
        VelocityPID_Reset(i);
    }
}

// ==================================================== 距离移动控制核心函数 ====================================================

/**
 * @brief 启动距离移动（内部通用函数）
 * @param mode 运动模式
 * @param distance 移动距离（单位：格，正数表示正向，负数表示反向）
 * @param speed 移动速度（正值）
 */
static void MoveMode_StartDistance(MoveMode_t mode, int32 distance, float speed)
{
    // 记录当前编码器位置作为基准
    g_start_encoder_count[0] = encoder_get_count(QTIMER1_ENCODER1);
    g_start_encoder_count[1] = encoder_get_count(QTIMER1_ENCODER2);
    g_start_encoder_count[2] = encoder_get_count(QTIMER2_ENCODER1);
    g_start_encoder_count[3] = encoder_get_count(QTIMER2_ENCODER2);

    for (uint8 i = 0; i < ENCODER_COUNT; i++)
    {
        g_last_raw_count[i] = g_start_encoder_count[i];
        g_unwrapped_count[i] = 0;
    }

    int32 pulse_per_unit = 0;
    switch (mode)
    {
        case FORWARD:
        case BACKWARD:
            pulse_per_unit = MOVE_DISTANCE_FORWARD_PULSE;
            break;

        case STRAFE_LEFT:
        case STRAFE_RIGHT:
            pulse_per_unit = MOVE_DISTANCE_STRAFE_PULSE;
            break;

        default:
            return; // 无效模式
    }

    // 计算目标脉冲数
    g_target_pulse_count = distance * pulse_per_unit;
    g_move_speed = (speed > 0.0f) ? speed : MOVE_DEFAULT_SPEED;
    g_move_state = MOVE_STATE_RUNNING;

    // 启动电机运动（同时确定实际运动方向）
    if (distance > 0)
    {
        g_current_mode = mode;
        MoveMode_SetSpeed(mode, g_move_speed);
    }
    else if (distance < 0)
    {
        // 反向移动：根据模式选择反向，并记录实际运动方向
        switch (mode)
        {
            case FORWARD:      g_current_mode = BACKWARD;     break;
            case BACKWARD:     g_current_mode = FORWARD;      break;
            case STRAFE_LEFT:  g_current_mode = STRAFE_RIGHT; break;
            case STRAFE_RIGHT: g_current_mode = STRAFE_LEFT;  break;
            default:           g_current_mode = STOP;         break;
        }

        MoveMode_SetSpeed(g_current_mode, g_move_speed);
        g_target_pulse_count = -g_target_pulse_count; // 取绝对值
    }
    else
    {
        // distance = 0，直接完成
        g_current_mode = STOP;
        g_move_state = MOVE_STATE_FINISHED;
        MoveMode_Stop();
    }
}

/**
 * @brief 控制小车前进指定距离
 * @param distance 移动距离（单位：格，1格 = 2470脉冲）
 * @param speed 移动速度，单位：脉冲/秒
 */
void MoveMode_ForwardDistance(int32 distance, float speed)
{
    MoveMode_StartDistance(FORWARD, distance, speed);
}

/**
 * @brief 控制小车后退指定距离
 * @param distance 移动距离（单位：格，1格 = 2470脉冲）
 * @param speed 移动速度，单位：脉冲/秒
 */
void MoveMode_BackwardDistance(int32 distance, float speed)
{
    MoveMode_StartDistance(BACKWARD, distance, speed);
}

/**
 * @brief 控制小车左平移指定距离
 * @param distance 移动距离（单位：格，1格 = 2470脉冲）
 * @param speed 移动速度，单位：脉冲/秒
 */
void MoveMode_StrafeLeftDistance(int32 distance, float speed)
{
    MoveMode_StartDistance(STRAFE_LEFT, distance, speed);
}

/**
 * @brief 控制小车右平移指定距离
 * @param distance 移动距离（单位：格，1格 = 2470脉冲）
 * @param speed 移动速度，单位：脉冲/秒
 */
void MoveMode_StrafeRightDistance(int32 distance, float speed)
{
    MoveMode_StartDistance(STRAFE_RIGHT, distance, speed);
}

/**
 * @brief 查询当前距离移动是否完成
 * @return uint8 1=移动完成，0=正在移动或未开始
 */
uint8 MoveMode_IsFinished(void)
{
    return (g_move_state == MOVE_STATE_FINISHED) ? 1 : 0;
}

/**
 * @brief 获取当前距离移动状态
 * @return MoveState_t 当前状态（空闲/运行中/已完成）
 */
MoveState_t MoveMode_GetState(void)
{
    return g_move_state;
}

/**
 * @brief 距离移动控制更新函数（需在主循环中周期性调用）
 * @note 此函数负责检查编码器位置并决定是否停止电机
 *       建议每 10~100ms 调用一次
 */
void MoveMode_DistanceUpdate(void)
{
    if (g_move_state != MOVE_STATE_RUNNING)
    {
        return;
    }

    MotionPID_UpdateHeadingControl(0.010f);

    // 读取当前编码器值
    int16 current_count[ENCODER_COUNT];
    current_count[0] = encoder_get_count(QTIMER1_ENCODER1);
    current_count[1] = encoder_get_count(QTIMER1_ENCODER2);
    current_count[2] = encoder_get_count(QTIMER2_ENCODER1);
    current_count[3] = encoder_get_count(QTIMER2_ENCODER2);

    // 计算各电机已移动的脉冲数（增量累积，处理 int16 溢出）
    int32 delta[ENCODER_COUNT];
    for (uint8 i = 0; i < ENCODER_COUNT; i++)
    {
        int32 raw_delta = (int32)current_count[i] - (int32)g_last_raw_count[i];

        // 处理 int16 计数器溢出：超过阈值说明编码器发生了缠绕
        if (raw_delta > 32000)
        {
            raw_delta -= 65536;
        }
        else if (raw_delta < -32000)
        {
            raw_delta += 65536;
        }

        g_unwrapped_count[i] += raw_delta;
        g_last_raw_count[i] = current_count[i];

        delta[i] = g_unwrapped_count[i];
    }

    // 编码器方向归一化：
    // 根据前进测试数据，RF(0)、RR(2) 原始编码器方向与期望前进方向相反
    int32 p0 = -delta[MOTOR_RIGHT_FRONT];  // RF
    int32 p1 =  delta[MOTOR_LEFT_FRONT];   // LF
    int32 p2 = -delta[MOTOR_RIGHT_REAR];   // RR
    int32 p3 =  delta[MOTOR_LEFT_REAR];    // LR

    // 底盘位移分量估计
    // forward_pos > 0 表示整体向前
    // strafe_pos  > 0 表示整体向左
    int32 forward_pos = (p0 + p1 + p2 + p3) / 4;
    int32 strafe_pos  = (p0 - p1 - p2 + p3) / 4;

    // 根据移动模式选择距离判定分量
    int32 avg_delta = 0;
    switch (g_current_mode)
    {
        case FORWARD:
        case BACKWARD:
            avg_delta = MoveMode_AbsInt32(forward_pos);
            break;

        case STRAFE_LEFT:
        case STRAFE_RIGHT:
            avg_delta = MoveMode_AbsInt32(strafe_pos);
            break;

        default:
            avg_delta = 0;
            break;
    }

    int32 remaining = g_target_pulse_count - avg_delta;
    if (remaining < 0)
    {
        remaining = 0;
    }

    if (remaining > MOVE_POSITION_TOLERANCE)
    {
        float dynamic_speed = MOVE_POSITION_KP * (float)remaining;

        if (dynamic_speed > g_move_speed)
        {
            dynamic_speed = g_move_speed;
        }

        if (dynamic_speed < MOVE_POSITION_MIN_SPEED)
        {
            dynamic_speed = MOVE_POSITION_MIN_SPEED;
        }

        if (g_current_mode == STRAFE_LEFT || g_current_mode == STRAFE_RIGHT)
        {
            MoveMode_SetStrafeSpeedWithForwardCorrection(g_current_mode, dynamic_speed, forward_pos);
        }
        else
        {
            MoveMode_SetSpeed(g_current_mode, dynamic_speed);
        }
    }
    else
    {
        MoveMode_Stop();
        g_move_state = MOVE_STATE_FINISHED;
    }
}

/**
 * @brief 重置距离移动状态
 * @note 清除目标位置和当前状态，强制回到空闲状态
 */
void MoveMode_ResetDistanceState(void)
{
    g_move_state = MOVE_STATE_IDLE;
    g_current_mode = STOP;
    g_target_pulse_count = 0;
    g_move_speed = 0.0f;
    MoveMode_Stop();
}

static uint8 runpath_is_observe(char ch)
{
    return (ch == '1' || ch == '2' || ch == '3' || ch == '4');
}

static void runpath_wait_finish(void)
{
    while (1)
    {
        MoveMode_DistanceUpdate();
        if (MoveMode_IsFinished())
        {
            break;
        }
        system_delay_ms(MOVE_RUNPATH_WAIT_DELAY_MS);
    }
}

static uint8 runpath_step(char dir, int32 count, float speed)
{
#if DEBUG_MOTOR_BYPASS
    Debug_MotorBypass_LogPathStep(dir, count);
    system_delay_ms(500);
    return 1;
#else
    switch (dir)
    {
        case 'W': MoveMode_ForwardDistance(count, speed);     break;
        case 'S': MoveMode_BackwardDistance(count, speed);    break;
        case 'A': MoveMode_StrafeLeftDistance(count, speed);  break;
        case 'D': MoveMode_StrafeRightDistance(count, speed); break;
        default:  return 0;
    }

    runpath_wait_finish();
    return 1;
#endif
}

uint8 MoveMode_RunPath(const char *path, float speed)
{
    uint16 i;
    uint8 result;

    if (path == NULL)
    {
        return 0;
    }

    if (speed <= 0.0f)
    {
        speed = MOVE_RUNPATH_DEFAULT_SPEED;
    }

    i = 0;
    result = 1;

    while (path[i] != '\0')
    {
        char ch = path[i];

        if (runpath_is_observe(ch))
        {
            i++;
            continue;
        }

        if (ch == 'W' || ch == 'S' || ch == 'A' || ch == 'D')
        {
            char dir = ch;
            int32 count = 1;

            while (path[i + 1] == dir)
            {
                count++;
                i++;
            }

            if (!runpath_step(dir, count, speed))
            {
                result = 0;
            }
        }

        i++;
    }

    MoveMode_Stop();
    return result;
}
