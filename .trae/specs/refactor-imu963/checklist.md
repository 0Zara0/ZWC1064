# Checklist

## Task 1: imu963.h 宏配置与数据结构
- [ ] `IMU963_ENABLE_TILT_COMPENSATION` 宏已定义，默认值为 0，且包含注释说明
- [ ] `IMU963_OUTPUT_SIGNED` 宏已定义，默认值为 1，且包含注释说明
- [ ] `IMU963_ANGLE_FILTER_ALPHA` 宏已移除，无残留引用
- [ ] `imu963_data_t` 包含 `mag_x_offset`、`mag_y_offset`、`mag_x_scale`、`mag_y_scale` 字段
- [ ] `imu963_calib_params_t` 不再包含 `common_k`、`kx`、`ky`、`ellipse_a`、`ellipse_b`、`radius_mean`
- [ ] `imu963_data_t.angle_deg` 注释更新，说明范围由 `IMU963_OUTPUT_SIGNED` 决定
- [ ] 所有原有宏（`IMU963_SAMPLE_PERIOD_MS`、换算系数、校准参数、卡尔曼 Q/R 等）保留不变

## Task 2: 卡尔曼滤波预测/更新分离
- [ ] `imu963_kalman_predict(gyro_zdps)` 函数存在，仅执行状态预测和 P 矩阵传播
- [ ] `imu963_kalman_update(mag_angle_deg)` 函数存在，仅执行测量更新和 P 矩阵修正
- [ ] `imu963_get_angle()` 中 `kalman_predict` 每帧必调用（无论 mag_updated）
- [ ] `imu963_get_angle()` 中 `kalman_update` 仅在 `mag_updated == 1` 时调用
- [ ] `imu963_kalman_angle`（卡尔曼内部角度状态）与 `imu963_data.angle_deg`（最终输出）职责清晰分离
- [ ] `imu963_reset_filter()` 正确重置卡尔曼状态和 P 矩阵对角元

## Task 3: imu963_keep_angle 静止检测 + 三段合一
- [ ] `imu963_is_stationary()` 函数存在，返回 uint8（1=静止）
- [ ] 静止判定逻辑：`sqrt(acc_x²+acc_y²+acc_z²)` 在 `[0.9g, 1.1g]` 内
- [ ] `imu963_keep_angle()` 为单段循环，不再复制三份
- [ ] 循环内同时采集加速度样本和角度样本，共享跳变过滤逻辑
- [ ] 静止检测失败时（非静止），valid_count 重置为 0 重新采集
- [ ] 达到 `IMU963_KEEP_MAX_ATTEMPT` 次仍失败时返回非 0
- [ ] 成功时返回 0，`yaw_bias_deg`、`acc_x_bias`、`acc_y_bias` 正确写入

## Task 4: 倾斜补偿开关化
- [ ] `#if IMU963_ENABLE_TILT_COMPENSATION` 包裹倾斜补偿代码路径
- [ ] `#else` 分支使用 `atan2f(y_cal, x_cal)` 直接计算
- [ ] 磁力计校准参数读写全部通过 `imu963_data` 结构体，无 `static` 变量
- [ ] `imu963_load_default_mag_calibration()` 正确初始化 `imu963_data` 中的校准字段
- [ ] `imu963_set_mag_calibration()` 和 `imu963_get_mag_calibration()` 从 `imu963_data` 读写

## Task 5: 移除输出角度一阶滤波
- [ ] 无 `imu963_output_filter_valid` 残留
- [ ] 无 `imu963_filtered_angle` 残留
- [ ] `imu963_get_angle()` 中无低通滤波代码
- [ ] `imu963_base_start()` 和 `imu963_reset_filter()` 中无相关初始化
- [ ] 编译通过，无未定义符号警告

## Task 6: 椭圆校准 buffer 内存优化
- [ ] 六个 `static double s_*[IMU963_CALIB_MAX_POINTS]` 数组已移除或包裹在条件编译下
- [ ] `imu963_calib_calculate_params()` 内部使用 C99 VLA 分配临时数组
- [ ] `imu963_calib_print_debug_params()` 不再引用已移除的 `calib_params_t` 字段
- [ ] 校准功能（采集、拟合、应用参数）端到端可正常工作

## Task 7: motion_PID.c 适配
- [ ] `MotionPID_ReadIMUData()` 中对 `imu963_get_angle()` 返回值的处理与 `IMU963_OUTPUT_SIGNED` 语义一致
- [ ] `g_sensor_data.yaw` 写入方式已适配（如果之前有硬编码的 0~360° 假设）
- [ ] `g_heading_hold_enabled` 相关逻辑验证：偏航修正的误差计算使用短角差或等效逻辑
- [ ] 编译通过，无 `motion_PID.c` 相关类型/符号错误

## 全局检查
- [ ] `imu963.h` 和 `imu963.c` 无编译错误和警告
- [ ] 所有公开 API 函数签名与重构前一致（无删除）
- [ ] `zf_device_imu963ra.h` 头文件仍被正确包含
- [ ] 项目中引用 `imu963_data` 或 `imu963_mag_calib_params` 的其他文件编译通过
