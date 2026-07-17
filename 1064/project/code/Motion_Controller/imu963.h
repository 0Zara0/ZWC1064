#ifndef _IMU963_H_
#define _IMU963_H_

#include "zf_common_typedef.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * IMU963RA 高层读取与陀螺仪积分偏航角解算模块
 *
 * 依赖文件：
 *   工程中必须已经添加 zf_device_imu963ra.c / zf_device_imu963ra.h。
 *
 * 常规使用：
 *   imu963_init();
 *   angle = imu963_get_angle();
 *
 * 注意：磁力计已禁用（电机金属部件干扰），偏航角仅由陀螺仪积分得到。
 */

#ifndef IMU963_SAMPLE_PERIOD_MS
#define IMU963_SAMPLE_PERIOD_MS              (10.0f)
#endif

#ifndef IMU963_ACC_RAW_SCALE
#define IMU963_ACC_RAW_SCALE                 (4096.0f)
#endif

#ifndef IMU963_GYRO_RAW_SCALE
#define IMU963_GYRO_RAW_SCALE                (16.4f)
#endif

#ifndef IMU963_KEEP_SAMPLE_COUNT
#define IMU963_KEEP_SAMPLE_COUNT             (30u)
#endif

#ifndef IMU963_KEEP_MAX_ATTEMPT
#define IMU963_KEEP_MAX_ATTEMPT              (300u)
#endif

#ifndef IMU963_ACC_JUMP_THRESHOLD_G
#define IMU963_ACC_JUMP_THRESHOLD_G          (0.01f)
#endif

#ifndef IMU963_GYRO_JUMP_THRESHOLD_DPS
#define IMU963_GYRO_JUMP_THRESHOLD_DPS       (0.5f)
#endif

typedef struct
{
    int16 raw_acc_x;
    int16 raw_acc_y;
    int16 raw_acc_z;
    int16 raw_gyro_x;
    int16 raw_gyro_y;
    int16 raw_gyro_z;

    float acc_x;
    float acc_y;
    float acc_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;

    float angle_deg;

    float yaw_bias_deg;
    float acc_x_bias;
    float acc_y_bias;

    uint8 initialized;
} imu963_data_t;

extern imu963_data_t imu963_data;

uint8 imu963_init(void);
void  imu963_set_dt_ms(float dt_ms);
void  imu963_update_raw(void);
void  imu963_calculate(void);
void  imu963_update(void);
float imu963_get_angle(void);
uint8 imu963_keep_angle(void);
float imu963_normalize_360(float angle_deg);

#ifdef __cplusplus
}
#endif

#endif
