# Tasks

- [ ] Task 1: 重构 `imu963.h` — 宏配置与数据结构调整
  - [ ] 新增 `IMU963_ENABLE_TILT_COMPENSATION` 宏（默认 0）
  - [ ] 新增 `IMU963_OUTPUT_SIGNED` 宏（默认 1）
  - [ ] 移除 `IMU963_ANGLE_FILTER_ALPHA` 及相关注释
  - [ ] 将运行时磁力计校准参数（`mag_x_offset/y_offset/x_scale/y_scale`）加入 `imu963_data_t`
  - [ ] 删减 `imu963_calib_params_t` 冗余字段：`common_k`、`kx`、`ky`、`ellipse_a`、`ellipse_b`、`radius_mean`（保留 `x_offset/y_offset/sx/sy/input_count/valid_count/filtered_count/fit_success`）
  - [ ] 更新 `imu963_data_t` 中 `angle_deg` 的注释：范围取决于 `IMU963_OUTPUT_SIGNED`

- [ ] Task 2: 重构 `imu963.c` — 卡尔曼滤波预测/更新分离
  - [ ] 将 `imu963_kalman_filter()` 拆分为 `imu963_kalman_predict(gyro_zdps)` 和 `imu963_kalman_update(mag_angle_deg)`
  - [ ] 修改 `imu963_get_angle()`：每帧必调 `kalman_predict`，仅磁力计数据变化时调 `kalman_update`
  - [ ] 修正 `imu963_get_angle()` 中角度偏差计算：输出受 `IMU963_OUTPUT_SIGNED` 控制

- [ ] Task 3: 重构 `imu963.c` — `imu963_keep_angle()` 增加静止检测并合并三段循环
  - [ ] 新增 `imu963_is_stationary()` 辅助函数：计算 `sqrt(acc_x²+acc_y²+acc_z²)` 是否在 `0.9g~1.1g` 范围内
  - [ ] 合并原三段独立循环为单段：同时采集加速度稳定样本和角度稳定样本
  - [ ] 加入静止态检测：每次采集前校验 IMU 静止，非静止则重新计数
  - [ ] 保持跳变过滤逻辑不变

- [ ] Task 4: 重构 `imu963.c` — `imu963_get_yaw_mag()` 倾斜补偿开关化
  - [ ] 用 `#if IMU963_ENABLE_TILT_COMPENSATION` 包裹倾斜补偿逻辑
  - [ ] `#else` 分支使用直接 `atan2f(y_cal, x_cal)`
  - [ ] 移除运行时校准参数的 `static` 变量，改为从 `imu963_data` 结构体读写

- [ ] Task 5: 重构 `imu963.c` — 移除输出角度一阶滤波
  - [ ] 删除 `imu963_output_filter_valid` 和 `imu963_filtered_angle` static 变量
  - [ ] 从 `imu963_get_angle()` 中删除低通滤波代码块
  - [ ] 从 `imu963_base_start()` 中删除相关初始化
  - [ ] 从 `imu963_reset_filter()` 中删除相关初始化

- [ ] Task 6: 重构 `imu963.c` — 椭圆校准 buffer 内存优化
  - [ ] 将 `s_x`/`s_y`/`s_x_filter`/`s_y_filter`/`s_x_norm`/`s_y_norm` 六个 static double 数组包裹在 `#if IMU963_CALIB_PRINT_ENABLE` 或专用条件编译下
  - [ ] 在 `imu963_calib_calculate_params()` 内部改用局部 VLA 或动态分配的临时数组（如果编译器支持 C99 VLA）以复用栈空间
  - [ ] 更新 `imu963_calib_print_debug_params()` 使其不依赖已移除的 `calib_params_t` 字段，改为直接从函数内部变量计算后打印

- [ ] Task 7: 调整 `motion_PID.c` 中 IMU 角度读取逻辑
  - [ ] 检查 `MotionPID_ReadIMUData()` 中是否对 `imu963_get_angle()` 返回值范围做了硬编码假设
  - [ ] 若原代码假设 0~360° 而新输出为 ±180°，添加适配逻辑（如归一化或短角差计算）
  - [ ] 验证 `g_sensor_data.yaw` 在 `g_heading_hold_enabled` 为真时的消费逻辑不受影响

# Task Dependencies
- Task 2 依赖 Task 1（需新宏和结构体定义）
- Task 3 依赖 Task 1（需静止检测阈值宏）
- Task 4 依赖 Task 1（需 `IMU963_ENABLE_TILT_COMPENSATION` 和 `imu963_data_t` 新字段）
- Task 5 依赖 Task 1（需删除 `IMU963_ANGLE_FILTER_ALPHA`）
- Task 6 不依赖其他任务，可并行
- Task 7 依赖 Task 2（需确认新输出语义）
- Task 3/4/5/6 相互独立，可并行
