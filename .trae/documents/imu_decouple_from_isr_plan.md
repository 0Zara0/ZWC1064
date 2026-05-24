# IMU 读取从 PIT 中断解耦方案

## 一、摘要

将 IMU963RA 传感器 SPI 读取 + Kalman 角度融合 + 角度 PID 计算移出 PIT 中断服务程序（`pit_handler`），放入主循环中执行。PIT ISR 仅消费已计算好的角度修正量缓存值，从而将 ISR 执行时间从 ~230μs 降至 ~30μs。

---

## 二、当前架构分析

### 2.1 数据流现状

```
PIT ISR (2ms)
  └─ pit_handler()
       ├─ MotionPID_UpdateSensorData()          ← 每周期: 编码器读+速度解算
       ├─ [每5周期=10ms] imu963_get_angle()     ← ★SPI 阻塞 ~200μs
       │    ├─ imu963_update()
       │    │    ├─ imu963ra_get_acc()  → 2次 SPI 阻塞
       │    │    ├─ imu963ra_get_gyro() → 2次 SPI 阻塞
       │    │    ├─ imu963ra_get_mag()  → 6次 SPI 阻塞 (Sensor Hub)
       │    │    └─ imu963_calculate()  → 浮点运算
       │    ├─ imu963_get_yaw_mag()     → atan2f + 硬铁/软铁校准
       │    ├─ imu963_kalman_filter()   → 2阶 Kalman 矩阵运算
       │    └─ 输出低通滤波
       ├─ [每5周期] AnglePID_Calculate()        ← 角度环 PID
       │    └─ 结果存入 g_heading_last_correction
       └─ for i=0..3:                           ← 每周期
            ├─ VelocityPID_Calculate()           → 速度环 PID
            └─ VelocityPID_ExecuteMotorControl() → PWM 驱动
```

### 2.2 关键全景变量

| 变量 | 位置 | 写入方 | 读取方 |
|------|------|--------|--------|
| `g_heading_last_correction` | motion_PID.c:23 | ISR (当前) | ISR (当前) |
| `imu963_data` | imu963.c:14 | `imu963_get_angle()` | `MotionPID_ReadIMUData()` (ISR内) |
| `g_angle_pid_controller` | pid_algorithm.c:17 | ISR (当前) | ISR (当前) |
| `g_heading_hold_enabled` | motion_PID.c:17 | `MotionPID_Enable/DisableHeadingHold()` | ISR |
| `g_heading_target` | motion_PID.c:18 | `MotionPID_EnableHeadingHold()` | ISR |

### 2.3 Kalman 滤波器时间依赖

`imu963_kalman_filter()` 使用 `imu963_dt` 进行状态预测：
```c
imu963_data.angle_deg = imu963_data.angle_deg - imu963_q_bias * imu963_dt + new_gyro_dps * imu963_dt;
```
当前 `imu963_dt = IMU963_SAMPLE_PERIOD_MS / 1000.0f = 0.01f`（固定10ms）。移出 ISR 后需要根据实际调用间隔动态设置。

### 2.4 imu963_get_angle 的状态机依赖

`imu963_get_angle()` 内部维护连续状态（Kalman P 矩阵、输出滤波器），必须**按固定频率调用**才能保证融合精度。调用频率变化会导致 Kalman 预测误差增大。

---

## 三、修改方案

### 3.1 核心思路

1. 新增 `MotionPID_IMU_Task()` 函数，包含原 `pit_handler()` 中的 IMU 读取 + 角度 PID 计算逻辑
2. 在主循环的轮询等待点（`runpath_wait_finish`、调试 Stage 的 while 循环）中周期性调用
3. `pit_handler()` 删除 IMU 相关代码，仅从 `g_heading_last_correction` 读取缓存值
4. 增加实际时间间隔测量，传递给 `imu963_set_dt_ms()` / `AnglePID_Calculate()`

### 3.2 涉及文件及修改

#### 文件 1：[motion_PID.h](file:///e:/workData/CpporPy/1064/1064/project/code/Motion_Controller/motion_PID.h)

**新增函数声明**：
```c
void MotionPID_IMU_Task(void);    // 主循环中周期性调用，执行 IMU 读取 + 角度 PID
```

#### 文件 2：[motion_PID.c](file:///e:/workData/CpporPy/1064/1064/project/code/Motion_Controller/motion_PID.c)

**修改 `pit_handler()`**（第 226-285 行）：
- 删除 IMU 读取和角度 PID 计算代码（第 231-256 行）
- 保留 `g_heading_last_correction` 的使用（第 264-274 行）
- 当 `g_heading_hold_enabled` 为 1 时直接使用缓存值

修改后 `pit_handler()` 结构：
```c
void pit_handler(void)
{
    MotionPID_UpdateSensorData();

    float angle_correction = 0.0f;
    if (g_heading_hold_enabled)
    {
        angle_correction = g_heading_last_correction;  // 直接读缓存
    }

    for (uint8 i = 0; i < ENCODER_COUNT; i++)
    {
        // ... 速度环 PID + 航向修正叠加 + 电机驱动
    }
}
```

**新增 `MotionPID_IMU_Task()`**：
```c
void MotionPID_IMU_Task(void)
{
    if (!g_heading_hold_enabled)
        return;

    float current_angle = imu963_get_angle();
    g_heading_last_correction = AnglePID_Calculate(
        &g_angle_pid_controller,
        g_heading_target,
        current_angle,
        IMU963_SAMPLE_PERIOD_MS / 1000.0f  // 10ms
    );
}
```

**注意**：`imu963_get_angle()` 内部通过 `imu963_set_dt_ms()` 设置固定 10ms，无需额外传参。但若主循环调用频率不能保证 10ms，需要在调用前通过系统滴答计时器计算实际间隔并调用 `imu963_set_dt_ms(actual_ms)` 修正 Kalman 的 dt。

#### 文件 3：调用点修改

主循环中 `runpath_wait_finish()` ([move_mode.c:L473-L484](file:///e:/workData/CpporPy/1064/1064/project/code/Motion_Controller/move_mode.c#L473-L484)) 是路径执行期间唯一的轮询等待点：

```c
static void runpath_wait_finish(void)
{
    while (1)
    {
        MotionPID_IMU_Task();           // ← 新增：IMU 航向更新
        MoveMode_DistanceUpdate();
        if (MoveMode_IsFinished())
            break;
        system_delay_ms(MOVE_RUNPATH_WAIT_DELAY_MS);
    }
}
```

此外还需要在以下位置添加调用：

| 调用位置 | 文件 | 说明 |
|----------|------|------|
| `runpath_wait_finish()` | move_mode.c:473 | 路径执行等待循环（10ms 轮询） |
| Stage 6 while 循环 | main.c:534/559/583 | 距离移动调试等待循环（10ms 轮询） |
| Stage 7 调用前 | main.c:628 | `MoveMode_RunPath()` 内部会调用 `runpath_wait_finish` |
| Normal Mode 主循环 | main.c:1008 | 虽非等待循环，但主循环周期长，可在此添加 |

### 3.3 调用频率保证

| 场景 | 轮询间隔 | IMU 调用频率 | 是否满足 10ms |
|------|----------|-------------|:---:|
| `runpath_wait_finish` | 10ms | ~10ms | ✅ |
| Stage 6 while 循环 | 10ms | ~10ms | ✅ |
| Normal Mode 主循环 | OMV 通讯期间较长 | 不确定 | ⚠️ 见下方说明 |

**Normal Mode 处理**：主循环包含 OMV 触发接收（超时 3000ms），若直接在此添加 `MotionPID_IMU_Task()` 会导致调用间隔不均。但航向保持仅在**路径执行期间**需要（`MoveMode_RunPath` → `runpath_wait_finish` 内部），Normal Mode 的 OMV 通讯阶段不需要 IMU 更新。

因此：**仅需在 `runpath_wait_finish()` 中添加调用**，即可覆盖所有航向保持场景。

### 3.4 线程安全分析

| 共享变量 | 写入方 | 读取方 | 安全性 |
|----------|--------|--------|:---:|
| `g_heading_last_correction` (float) | 主循环 | PIT ISR | ✅ Cortex-M7 单精度 float 读写原子 |
| `imu963_data` (struct) | 主循环 | PIT ISR | ✅ 各字段为独立 float，原子读写 |
| `g_angle_pid_controller` | 主循环 | — | ✅ ISR 不再访问 |
| `g_heading_target` | 主循环 | 主循环 | ✅ ISR 不再访问 |

### 3.5 时间同步说明

`imu963_get_angle()` 内部使用全局 `imu963_dt`（默认 10ms）。若主循环调用间隔偏移较大，Kalman 预测会偏差。

**保守做法**：不改动 `imu963_dt`，要求调用方保证 ~10ms 间隔。当前 `runpath_wait_finish` 的 `MOVE_RUNPATH_WAIT_DELAY_MS = 10`，加上 `MoveMode_DistanceUpdate()` 和 `MotionPID_IMU_Task()` 自身的执行时间，实际间隔约 10~11ms，在可接受范围内。

**进阶做法**（可选）：在 `MotionPID_IMU_Task()` 中记录上次调用的系统滴答，计算实际 dt_ms，调用 `imu963_set_dt_ms(actual_ms)` 修正 Kalman 时间步。

---

## 四、ISR 负载改善

| 指标 | 修改前 | 修改后 | 改善 |
|------|--------|--------|:---:|
| 普通周期 ISR (80%) | ~30μs | ~30μs | — |
| 重周期 ISR (20%) | ~230μs | **~30μs** | **-87%** |
| 加权平均 ISR | ~70μs | **~30μs** | **-57%** |
| 重周期占 2ms 预算 | 11.5% | 1.5% | **-87%** |

---

## 五、step-by-step 实施步骤

### Step 1：添加函数声明
- 文件：`motion_PID.h`
- 在航向保持函数区域新增 `void MotionPID_IMU_Task(void);`

### Step 2：修改 pit_handler()
- 文件：`motion_PID.c`
- 删除第 231-256 行的 IMU 读取 + 角度 PID 代码
- 简化为直接读 `g_heading_last_correction`

### Step 3：实现 MotionPID_IMU_Task()
- 文件：`motion_PID.c`
- 新增函数，包含原 ISR 中的 `imu963_get_angle()` + `AnglePID_Calculate()` 逻辑

### Step 4：在 runpath_wait_finish 中添加调用
- 文件：`move_mode.c`
- 在 `while(1)` 循环中、`MoveMode_DistanceUpdate()` 之前调用 `MotionPID_IMU_Task()`

### Step 5：验证
- 编译通过（IAR）
- Stage 7 路径执行测试，观察航向保持是否正常
- 用示波器测量 ISR 执行时间确认缩短

---

## 六、不修改的文件

以下文件仅依赖 `imu963_data` 全局变量的**读取**（通过 `MotionPID_ReadIMUData()`），不涉及 IMU 读取的触发，无需修改：

- `zf_device_imu963ra.c` / `.h`
- `imu963.c` / `.h`
- `pid_algorithm.c` / `.h`
- `dc_motor.c` / `.h`

---

## 七、风险与故障恢复

| 风险 | 影响 | 缓解措施 |
|------|------|----------|
| 主循环被长时间阻塞导致 IMU 不更新 | 航向保持失效，车体跑偏 | `g_heading_last_correction` 保持上次值，不会突变；路径执行完成后自动停止 |
| 调用频率与 Kalman 预期 10ms 偏差 | 角度融合精度下降 | `runpath_wait_finish` 10ms 间隔稳定偏差 <1ms，影响可忽略 |
| 调试 Stage 6 的 while 循环未添加调用 | 距离移动时无航向保持 | Stage 6 距离移动调试时航向保持本就不是主要测试目标，可不添加（或按需添加） |
