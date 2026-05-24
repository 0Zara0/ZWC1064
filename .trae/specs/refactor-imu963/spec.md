# IMU963 系统性重构 Spec

## Why
当前 `imu963.c/.h` 经上轮修复后仍存在结构性问题：卡尔曼滤波预测/更新未分离导致陀螺仪积分在磁力计未就绪帧被跳过、`imu963_keep_angle` 三段重复代码缺乏静止检测、椭圆校准 static buffer 大量常驻内存、倾斜补偿无开关控制可能在加速时引入新误差。需要以**可靠性**为首要目标进行系统性重构，同时兼顾准确性和较低的计算复杂度，最终输出适合 PID 控制流程的偏航角。

## What Changes
- **BREAKING**: 重构 `imu963_get_angle()` 调用语义，分离陀螺仪积分预测步与磁力计测量修正步，确保每帧都执行陀螺仪积分
- **BREAKING**: 重构 `imu963_keep_angle()`，增加静止态检测（总加速度幅值 ≈1g），合并三段重复代码
- 新增 `IMU963_ENABLE_TILT_COMPENSATION` 宏控制倾斜补偿开关（默认关闭）
- 删减 `imu963_calib_params_t` 中冗余调试字段，减小结构体
- 将运行时校准参数从 `static` 变量迁入 `imu963_data_t`，消除隐式状态
- 删除输出角度一阶滤波（`IMU963_ANGLE_FILTER_ALPHA`），卡尔曼已提供足够平滑
- 将椭圆拟合临时 buffer 改为仅在 `imu963_calib_calculate_params()` 调用期间分配（若平台支持），或标注为条件编译以减少常驻内存
- 保持全部现有公开 API 函数签名不减少，仅内部实现重构
- 保持对 `zf_device_imu963ra` 的依赖不变

## Impact
- Affected specs: 无（首次重构 spec）
- Affected code:
  - `project/code/Motion_Controller/imu963.h` — 宏配置、数据结构调整
  - `project/code/Motion_Controller/imu963.c` — 核心算法重构
  - `project/code/Motion_Controller/motion_PID.c` — 可能需要适配 yaw 输出语义（仅当 `imu963_get_angle()` 返回值范围从 0~360° 变为 ±180° 时）
  - `libraries/zf_device/zf_device_imu963ra.h/.c` — 不修改

---

## ADDED Requirements

### Requirement: 卡尔曼滤波每帧预测、条件更新
系统 SHALL 在每个采样周期内执行陀螺仪积分预测步（状态外推），仅在磁力计数据更新时执行测量修正步。

#### Scenario: 磁力计数据未更新的帧
- **GIVEN** 上一个磁力计数据与当前读取完全一致
- **WHEN** `imu963_get_angle()` 被调用
- **THEN** 系统仍然用陀螺仪 Z 轴角速度 `gyro_z` 积分推进偏航角估计
- **AND** 系统不执行磁力计测量修正

#### Scenario: 磁力计数据已更新的帧
- **GIVEN** 磁力计原始值发生变化
- **WHEN** `imu963_get_angle()` 被调用
- **THEN** 系统先执行陀螺仪积分预测，再执行磁力计测量修正更新

### Requirement: 上电零点校准增加静止检测
`imu963_keep_angle()` SHALL 在采集加速度偏置和偏航角偏置之前，检测 IMU 是否处于静止状态（总加速度幅值偏差 ≤10% of 1g），静止检测失败时 SHALL 返回非零错误码。

#### Scenario: IMU 静止时初始化
- **GIVEN** IMU 静态放置，总加速度幅值在 0.9g~1.1g 之间
- **WHEN** `imu963_keep_angle()` 被调用
- **THEN** 系统成功采集加速度偏置和偏航角偏置
- **AND** 返回 0

#### Scenario: IMU 运动时初始化
- **GIVEN** IMU 在运动/振动中，总加速度幅值偏离 1g 超过 10%
- **WHEN** `imu963_keep_angle()` 被调用
- **THEN** 系统 SHALL 在 IMU963_KEEP_MAX_ATTEMPT 次尝试后返回非零错误码
- **AND** 不写入错误的偏置值

### Requirement: 倾斜补偿可通过宏开关控制
系统 SHALL 提供 `IMU963_ENABLE_TILT_COMPENSATION` 宏（默认 0），当设为 1 时 `imu963_get_yaw_mag()` 执行加速度计辅助的磁力计倾斜补偿；当设为 0 时直接对磁力计 XY 做 atan2 求偏航角。

#### Scenario: 平坦赛道场景
- **GIVEN** `IMU963_ENABLE_TILT_COMPENSATION = 0`
- **WHEN** `imu963_get_yaw_mag()` 被调用
- **THEN** 系统直接使用 `atan2f(y_cal, x_cal)` 计算偏航角

#### Scenario: 颠簸赛道场景
- **GIVEN** `IMU963_ENABLE_TILT_COMPENSATION = 1`
- **WHEN** `imu963_get_yaw_mag()` 被调用
- **THEN** 系统利用加速度数据将磁力计向量投影到水平面后再求偏航角

### Requirement: 输出角度语义清晰化
`imu963_get_angle()` SHALL 返回相对于上电初始航向的偏航偏差，默认范围 SHALL 为 `-180.0° ~ +180.0°`（带符号），由 `IMU963_OUTPUT_SIGNED` 宏控制。当 `IMU963_OUTPUT_SIGNED = 0` 时返回 `0~360°`。

#### Scenario: 带符号输出用于 PID
- **GIVEN** `IMU963_OUTPUT_SIGNED = 1`
- **WHEN** 车辆向右偏航 5°
- **THEN** `imu963_get_angle()` 返回约 `+5.0°`

#### Scenario: 无符号输出
- **GIVEN** `IMU963_OUTPUT_SIGNED = 0`
- **WHEN** 车辆向右偏航 5°
- **THEN** `imu963_get_angle()` 返回约 `5.0°`

---

## MODIFIED Requirements

### Requirement: 磁力计椭圆校准功能保持
`imu963_calib_calculate_params()` 保持全部核心算法（Levenberg-Marquardt 椭圆拟合 + 3σ 离群过滤 + fallback 估计），但 `imu963_calib_params_t` 中的冗余调试字段 `common_k`、`kx`、`ky`、`ellipse_a`、`ellipse_b` SHALL 移除。

### Requirement: 偏航角输出滤波移除
原有的 `IMU963_ANGLE_FILTER_ALPHA` 一阶低通输出滤波 SHALL 移除，因为卡尔曼滤波器已提供自适应平滑。相关宏定义和 `imu963_output_filter_valid`、`imu963_filtered_angle` 变量 SHALL 删除。

---

## REMOVED Requirements

### Requirement: 旧的三段循环式 `imu963_keep_angle` 实现
**Reason**: 三段独立循环代码重复度过高（50+ 行几乎相同的逻辑），缺少静止态验证。
**Migration**: 合并为单段循环，每次迭代同时采集加速度和角度的稳定样本，并校验静止条件。

### Requirement: `imu963_calib_params_t` 中的冗余调试字段
**Reason**: `common_k`/`kx`/`ky` 可由 `x_offset/y_offset/sx/sy` 和 `ellipse_a/ellipse_b` 计算推导，`ellipse_a/ellipse_b` 和 `radius_mean` 仅调试用途。
**Migration**: 通过 `imu963_calib_print_debug_params()` 改用椭圆拟合内部变量直接输出，不存入结构体。
