#ifndef MOVE_MODE_H
#define MOVE_MODE_H

#include "zf_common_typedef.h"

// ==================================================== 底盘物理尺寸参数 ====================================================

/** @brief 前后轴距 (mm) — 前后轮轴线间的距离 */
#define CHASSIS_WHEELBASE_MM            (186.0f)

/** @brief 左右轮距 (mm) — 左右轮接地中心间的距离 */
#define CHASSIS_TRACK_WIDTH_MM          (183.0f)

/** @brief 麦克纳姆轮直径 (mm) */
#define CHASSIS_WHEEL_DIAMETER_MM       (52.0f)

/** @brief 半轴距 Lx = 前后轴距 / 2 (mm)，用于运动学模型 */
#define CHASSIS_HALF_WHEELBASE_MM       (CHASSIS_WHEELBASE_MM / 2.0f)

/** @brief 半轮距 Ly = 左右轮距 / 2 (mm)，用于运动学模型 */
#define CHASSIS_HALF_TRACK_WIDTH_MM     (CHASSIS_TRACK_WIDTH_MM / 2.0f)

/** @brief 轮子半径 R (mm) */
#define CHASSIS_WHEEL_RADIUS_MM         (CHASSIS_WHEEL_DIAMETER_MM / 2.0f)

/** @brief 编码器每圈脉冲数 (PPR) */
#define ENCODER_PULSES_PER_REV          (2367)

// ==================================================== 距离移动配置参数 ====================================================

/** @brief 前进/后退移动200mm的理论脉冲数
 *  @note 计算: 200 / (π×52) × 2367 ≈ 2898
 *  @note 上次经验值: 2898 */
#define MOVE_DISTANCE_FORWARD_PULSE     (3018)

/** @brief 左/右平移移动200mm的理论脉冲数
 *  @note 计算: 同前向 ≈ 2898（麦轮平移滑动大，理论值仅供参考）
 *  @note 上次经验值: 3492 */
#define MOVE_DISTANCE_STRAFE_PULSE      (3470)

/** @brief 原地左/右转向90°的理论脉冲数
 *  @note 计算: (Lx+Ly) / (2×D) × PPR = 184.5 / 104 × 2367 ≈ 4200
 *  @note 上次经验值: 4200 */
#define MOVE_DISTANCE_TURN_QUARTER_PULSE (4395)

/** @brief 默认移动速度（脉冲/秒） */
#define MOVE_DEFAULT_SPEED              (5000.0f)

/** @brief 距离移动完成判定阈值（脉冲误差容限） */
#define MOVE_POSITION_TOLERANCE         (20)

/** @brief 位置 P 控制增益：dynamic_speed = Kp * remaining */
#define MOVE_POSITION_KP                (10.0f)

/** @brief 位置 P 控制输出的最低速度（脉冲/秒），确保能克服摩擦力 */
#define MOVE_POSITION_MIN_SPEED         (500.0f)

/**
 * @brief 平移时前后偏移修正增益
 * @note 左/右平移时，如果车体向前或向后串动，根据 forward_pos 自动叠加反向 vx 分量
 *       值越大修正越强，过大会抖动；建议从 0.3~1.0 调试
 */
#define MOVE_STRAFE_FORWARD_CORRECT_KP      (0.8f)

/** @brief 平移前后偏移修正输出限幅（脉冲/秒） */
#define MOVE_STRAFE_FORWARD_CORRECT_LIMIT   (1000.0f)

/** @brief 平移前后偏移死区（脉冲），小误差不修正，避免低速抖动 */
#define MOVE_STRAFE_FORWARD_DEADBAND        (10)

/** @brief 路径执行默认速度（脉冲/秒），统一使用 MOVE_DEFAULT_SPEED */
#define MOVE_RUNPATH_DEFAULT_SPEED      MOVE_DEFAULT_SPEED
#define MOVE_RUNPATH_WAIT_DELAY_MS      (10)

// ==================================================== 运动模式枚举与状态 ====================================================

/**
 * @brief 小车运动模式枚举
 */
typedef enum
{
    STOP = 0,           // 停止
    FORWARD,            // 前进
    BACKWARD,           // 后退
    STRAFE_LEFT,        // 左平移
    STRAFE_RIGHT,       // 右平移
    TURN_LEFT,          // 原地左转（CCW）
    TURN_RIGHT          // 原地右转（CW）
} MoveMode_t;

/**
 * @brief 距离移动状态枚举
 */
typedef enum
{
    MOVE_STATE_IDLE = 0,        // 空闲状态
    MOVE_STATE_RUNNING,         // 正在移动
    MOVE_STATE_FINISHED         // 移动完成
} MoveState_t;

// ==================================================== 函数声明 ====================================================

/**
 * @brief 初始化运动控制系统
 * @note 调用此函数会初始化所有传感器、电机和定时器
 *       必须在调用其他运动控制函数前执行
 */
void MoveMode_Init(void);

/**
 * @brief 设置小车运动模式和速度
 * @param mode 运动模式（前进/后退/左移/右移/停止）
 * @param speed 目标速度，单位：脉冲/秒
 */
void MoveMode_SetSpeed(MoveMode_t mode, float speed);

/**
 * @brief 控制小车前进
 * @param speed 前进速度，单位：脉冲/秒
 */
void MoveMode_Forward(float speed);

/**
 * @brief 控制小车后退
 * @param speed 后退速度，单位：脉冲/秒
 */
void MoveMode_Backward(float speed);

/**
 * @brief 控制小车左平移
 * @param speed 左移速度，单位：脉冲/秒
 */
void MoveMode_StrafeLeft(float speed);

/**
 * @brief 控制小车右平移
 * @param speed 右移速度，单位：脉冲/秒
 */
void MoveMode_StrafeRight(float speed);

/**
 * @brief 停止小车运动
 */
void MoveMode_Stop(void);

// ==================================================== 距离移动控制函数 ====================================================

/**
 * @brief 控制小车前进指定距离
 * @param distance 移动距离（单位：格，1格 = 2470脉冲）
 * @param speed 移动速度，单位：脉冲/秒
 */
void MoveMode_ForwardDistance(int32 distance, float speed);

/**
 * @brief 控制小车后退指定距离
 * @param distance 移动距离（单位：格，1格 = 2470脉冲）
 * @param speed 移动速度，单位：脉冲/秒
 */
void MoveMode_BackwardDistance(int32 distance, float speed);

/**
 * @brief 控制小车左平移指定距离
 * @param distance 移动距离（单位：格，1格 = 2470脉冲）
 * @param speed 移动速度，单位：脉冲/秒
 */
void MoveMode_StrafeLeftDistance(int32 distance, float speed);

/**
 * @brief 控制小车右平移指定距离
 * @param distance 移动距离（单位：格，1格 = 2470脉冲）
 * @param speed 移动速度，单位：脉冲/秒
 */
void MoveMode_StrafeRightDistance(int32 distance, float speed);

/**
 * @brief 查询当前距离移动是否完成
 * @return uint8 1=移动完成，0=正在移动或未开始
 */
uint8 MoveMode_IsFinished(void);

/**
 * @brief 获取当前距离移动状态
 * @return MoveState_t 当前状态（空闲/运行中/已完成）
 */
MoveState_t MoveMode_GetState(void);

/**
 * @brief 距离移动控制更新函数（需在主循环中周期性调用）
 * @note 建议每 10~100ms 调用一次
 */
void MoveMode_DistanceUpdate(void);

/**
 * @brief 重置距离移动状态
 */
void MoveMode_ResetDistanceState(void);

/**
 * @brief 原地左转指定角度（CCW，逆时针）
 * @param quarter_turns 转向角度（单位：1/4 周 = 90°，正数 CCW，负数 CW）
 * @param speed 转向速度，单位：脉冲/秒
 * @note 左侧两轮向后，右侧两轮向前，航向保持自动禁用
 */
void MoveMode_TurnLeftDistance(int32 quarter_turns, float speed);

/**
 * @brief 原地右转指定角度（CW，顺时针）
 * @param quarter_turns 转向角度（单位：1/4 周 = 90°，正数 CW，负数 CCW）
 * @param speed 转向速度，单位：脉冲/秒
 * @note 右侧两轮向后，左侧两轮向前，航向保持自动禁用
 */
void MoveMode_TurnRightDistance(int32 quarter_turns, float speed);

/**
 * @brief 原地 180° 后转
 * @param speed 转向速度，单位：脉冲/秒
 */
void MoveMode_TurnBack(float speed);

/**
 * @brief 执行路径字符串
 * @param path 路径字符串，W=前进，S=后退，A=左移，D=右移，1/2/3/4=观察点忽略
 * @param speed 执行速度，单位：脉冲/秒
 * @return uint8 1=成功，0=路径非法或执行失败
 */
uint8 MoveMode_RunPath(const char *path, float speed);

#endif
