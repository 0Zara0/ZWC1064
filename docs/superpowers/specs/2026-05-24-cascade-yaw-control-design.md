# 偏航级联控制设计文档

> 状态：已批准 | 日期：2026-05-24

**目标：** 以级联控制（角度环→角速度环→速度环）替换现有的"角度环→速度环"单层航向修正，实现 ±1° 偏航保持精度。

**架构：** 角度环在主循环运行（输出目标角速度），角速度环在 PIT ISR 中每 2ms 运行（用 gyro_z 直接对抗角加速度），imu963_get_angle() 移出 ISR。

**平台：** NXP MIMXRT1064, IAR EWARM, 裸机无 RTOS

---

## 1. 文件变更

| 文件 | 变更 | 说明 |
|------|------|------|
| `pid_algorithm.h` | 新增 ~15 行 | 角速度环 PID 结构体 + 配置宏 + 函数声明 |
| `pid_algorithm.c` | 新增 ~30 行 | 角速度环 Init/Calculate/Reset |
| `motion_PID.h` | 新增 ~4 行 | `g_heading_target_rate` 声明 + `MotionPID_UpdateHeadingControl` 声明 |
| `motion_PID.c` | 修改 ~20 行 | ISR 替换旧逻辑; 新增角度环控制函数 |
| `move_mode.c` | 修改 ~3 行 | `MoveMode_DistanceUpdate` 尾部调用角度环函数 |

**不涉及:** imu963.c/h, dc_motor.c/h, main.c, 速度环 PID。

---

## 2. 新增数据结构

```c
#define ANGULAR_RATE_PID_KP       (3.0f)
#define ANGULAR_RATE_PID_KI       (0.5f)
#define ANGULAR_RATE_PID_KD       (0.1f)
#define ANGULAR_RATE_PID_OUTPUT_LIMIT   (30.0f)
#define ANGULAR_RATE_PID_INTEGRAL_LIMIT (100.0f)

typedef struct {
    float Kp, Ki, Kd;
    float error, error_last, error_sum;
    float output;
    float target_rate;
    float current_rate;
    float output_limit;
    float integral_limit;
    uint8 initialized, enabled;
} AngularRatePIDController_t;
```

---

## 3. 数据流

```
主循环 (≥10ms)                      PIT ISR (2ms)
──────────────                      ─────────────
imu963_get_angle()                 g_sensor_data.gyro_z (已拷贝)
       │                                    │
       ▼                                    ▼
角度环 PID                           角速度环 PID
target vs yaw → target_rate         target_rate vs gyro_z → angle_correction
       │                                    │
写入 g_heading_target_rate                 ▼
                                   叠加到4电机 target_speed
                                          │
                                          ▼
                                   速度环 PID × 4 → PWM
```

### ISR pit_handler() 核心逻辑

```c
if (g_heading_hold_enabled && g_angular_rate_pid.enabled) {
    angle_correction = AngularRatePID_Calculate(
        &g_angular_rate_pid,
        g_heading_target_rate,          // 角度环写入
        g_sensor_data.gyro_z,           // 实时角速度
        SENSOR_UPDATE_PERIOD_MS / 1000.0f
    );
}
```

- 移除 `imu_divider` 分频器
- 移除 `imu963_get_angle()` 调用
- `angle_correction` 叠加到电机速度的方式不变

### 角度环函数（主循环调用）

```c
void MotionPID_UpdateHeadingControl(float dt) {
    if (!g_heading_hold_enabled) return;
    if (!imu963_data.initialized) return;  // 保护

    float yaw = imu963_get_angle();
    g_heading_target_rate = AnglePID_Calculate(
        &g_angle_pid_controller,
        g_heading_target, yaw, dt
    );
}
```

### 调用点

`MoveMode_DistanceUpdate()` 末尾添加：

```c
MotionPID_UpdateHeadingControl(0.010f);
```

---

## 4. 初始化

```
MotionPID_InitHeadingHold()
  ├─ AnglePID_Init(...)
  └─ AngularRatePID_Init(...)    ← 新增
```

---

## 5. 参数整定

| 步骤 | 内容 | 验证方式 |
|------|------|----------|
| 1 | 角速度环 P 从 1.0 往上加 | 掰车应感到阻尼，不抖 |
| 2 | 角速度环 I | 直行时 gyro_z 偏置应被消除 |
| 3 | 角度环 P | 偏 10°→期望 20~30°/s 修正 |
| 4 | 角度环 I | 固定差速下偏航长期归零 |

所有 PID 参数为 `#define` 宏，重新编译即可调参。

---

## 6. 边缘情况

| 场景 | 处理方式 |
|------|----------|
| IMU 未初始化时调用 | `imu963_data.initialized` 检查，直接 return |
| 航向保持未使能 | `g_heading_hold_enabled` 检查，correction=0 |
| 角速度环积分饱和 | INTEGRAL_LIMIT=100 硬限幅 |
| 角度环输出过大 | OUTPUT_LIMIT=50 (已有) |
| yaw 0°/360° 边界 | `AnglePID_Calculate` 已有 ±180° 归一化 |

---

## 7. 调试观察点（Stage 5）

```
Time  Yaw_Tgt  Yaw_Cur  AngErr  TargetRate  GyroZ  RateErr  RateCorr  Speed0~3
```

- 直行: GyroZ≈0, RateCorr小
- 受扰: GyroZ跳→RateErr扩大→RateCorr响应→GyroZ拉回
- 长期: Yaw_Cur应在Yaw_Tgt ±1°内
