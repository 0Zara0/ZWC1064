

#ifndef MOVE_MODE_H
#define MOVE_MODE_H

#include "zf_common_typedef.h"

// ==================================================== 距离移动配置参数 ====================================================

/** @brief 前进/后退移动单位距离对应的编码器脉冲数 */
#define MOVE_DISTANCE_FORWARD_PULSE     (2470)

/** @brief 左/右平移单位距离对应的编码器脉冲数 */
#define MOVE_DISTANCE_STRAFE_PULSE      (2470)

/** @brief 默认移动速度（脉冲/秒） */
#define MOVE_DEFAULT_SPEED              (5000.0f)

/** @brief 距离移动完成判定阈值（脉冲误差容限） */
#define MOVE_POSITION_TOLERANCE         (20)

/** @brief 减速触发阈值：剩余脉冲数低于此值时降速避免惯性过冲 */
#define MOVE_DECEL_THRESHOLD            (500)

/** @brief 减速阶段的目标速度（脉冲/秒） */
#define MOVE_DECEL_SPEED                (1000.0f)

#define MOVE_RUNPATH_DEFAULT_SPEED      (5000.0f)
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
    STRAFE_RIGHT        // 右平移
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
 *              正值表示正向运动，负值表示反向运动
 * @note 示例：
 *       MoveMode_SetSpeed(FORWARD, 100.0f);     // 以 100 脉冲/秒前进
 *       MoveMode_SetSpeed(BACKWARD, 50.0f);     // 以 50 脉冲/秒后退
 *       MoveMode_SetSpeed(STRAFE_LEFT, 80.0f);  // 以 80 脉冲/秒左移
 *       MoveMode_SetSpeed(STOP, 0.0f);          // 停止
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
 * @param speed 移动速度，单位：脉冲/秒（默认可用 MOVE_DEFAULT_SPEED）
 * @note 此函数为异步调用，启动移动后立即返回
 *       需配合 MoveMode_IsFinished() 查询是否完成
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
 * @brief 距离移动控制更新函数（需在 PIT 中断或主循环中周期性调用）
 * @note 此函数负责检查编码器位置并决定是否停止电机
 *       建议每 10~100ms 调用一次
 */
void MoveMode_DistanceUpdate(void);

/**
 * @brief 重置距离移动状态
 * @note 清除目标位置和当前状态，强制回到空闲状态
 */
void MoveMode_ResetDistanceState(void);

uint8 MoveMode_RunPath(const char *path, float speed);

#endif // MOVE_MODE_H
