#ifndef _IMU963_H_
#define _IMU963_H_

#include "zf_common_typedef.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * IMU963RA 高层读取、偏航角解算与磁力计校准模块
 *
 * 依赖文件：
 *   工程中必须已经添加 zf_device_imu963ra.c / zf_device_imu963ra.h。
 *
 * 常规使用：
 *   imu963_init();
 *   angle = imu963_get_angle();
 *
 * 一体化校准使用：
 *   imu963_init_with_calibration(300, 100);
 *   angle = imu963_get_angle();
 */

#ifndef IMU963_SAMPLE_PERIOD_MS
#define IMU963_SAMPLE_PERIOD_MS              (10.0f)     /* 正常读取周期，单位 ms */
#endif

/*
 * 数据换算来源：
 *   0：使用 Python 程序中的换算系数，acc/4096，gyro/16.4，mag/12。
 *   1：使用逐飞 zf_device_imu963ra.h 中的 transition 宏。
 *
 * 注意：下面的磁力计校准参数是基于 Python 换算方式得到的，所以默认使用 0。
 */
#ifndef IMU963_USE_ZF_TRANSITION
#define IMU963_USE_ZF_TRANSITION             (0)
#endif

#ifndef IMU963_ACC_RAW_SCALE
#define IMU963_ACC_RAW_SCALE                 (4096.0f)   /* 加速度原始值转 g 的比例 */
#endif

#ifndef IMU963_GYRO_RAW_SCALE
#define IMU963_GYRO_RAW_SCALE                (16.4f)     /* 陀螺仪原始值转 °/s 的比例 */
#endif

#ifndef IMU963_MAG_RAW_SCALE
#define IMU963_MAG_RAW_SCALE                 (12.0f)     /* 磁力计原始值转 uT 的比例 */
#endif

/* 磁力计默认校准参数，重新校准后可以把串口输出的四个宏复制到这里 */
#ifndef IMU963_MAG_X_OFFSET_UT
#define IMU963_MAG_X_OFFSET_UT               (-3.5546f)
#endif

#ifndef IMU963_MAG_Y_OFFSET_UT
#define IMU963_MAG_Y_OFFSET_UT               (82.6659f)
#endif

#ifndef IMU963_MAG_X_SCALE
#define IMU963_MAG_X_SCALE                   (1.0354f)
#endif

#ifndef IMU963_MAG_Y_SCALE
#define IMU963_MAG_Y_SCALE                   (0.9669f)
#endif

#ifndef IMU963_KEEP_SAMPLE_COUNT
#define IMU963_KEEP_SAMPLE_COUNT             (30u)       /* 上电保持角度校准有效采样数 */
#endif

#ifndef IMU963_KEEP_MAX_ATTEMPT
#define IMU963_KEEP_MAX_ATTEMPT              (300u)      /* 上电保持角度校准最大尝试次数 */
#endif

#ifndef IMU963_ACC_JUMP_THRESHOLD_G
#define IMU963_ACC_JUMP_THRESHOLD_G          (0.01f)     /* 加速度跳变过滤阈值，单位 g */
#endif

#ifndef IMU963_ANGLE_JUMP_THRESHOLD_DEG
#define IMU963_ANGLE_JUMP_THRESHOLD_DEG      (1.5f)      /* 角度跳变过滤阈值，单位 ° */
#endif

/* ============================== 磁力计校准相关参数 ============================== */

#ifndef IMU963_CALIB_MAX_POINTS
#define IMU963_CALIB_MAX_POINTS              (800u)      /* 最大校准采样点数 */
#endif

#ifndef IMU963_CALIB_MIN_VALID_POINTS
#define IMU963_CALIB_MIN_VALID_POINTS        (20u)       /* 最少有效校准点数 */
#endif

#ifndef IMU963_CALIB_SIGMA
#define IMU963_CALIB_SIGMA                   (3.0)       /* 离群点过滤阈值，默认 3σ */
#endif

#ifndef IMU963_CALIB_MAX_ITER
#define IMU963_CALIB_MAX_ITER                (120u)      /* 椭圆拟合最大迭代次数 */
#endif

#ifndef IMU963_CALIB_SKIP_ZERO_POINTS
#define IMU963_CALIB_SKIP_ZERO_POINTS        (1u)        /* 是否跳过 0,0 无效点 */
#endif

#ifndef IMU963_CALIB_DEFAULT_COUNT
#define IMU963_CALIB_DEFAULT_COUNT           (300u)      /* 一体化校准默认采样点数 */
#endif

#ifndef IMU963_CALIB_DEFAULT_INTERVAL_MS
#define IMU963_CALIB_DEFAULT_INTERVAL_MS     (100u)      /* 一体化校准默认采样间隔 */
#endif

#ifndef IMU963_CALIB_PRINT_ENABLE
#define IMU963_CALIB_PRINT_ENABLE            (1u)        /* 一体化校准后是否自动打印四个参数 */
#endif

/* ============================== Kalman 可靠性参数 ============================== */

#ifndef IMU963_MAG_REF_DEVIATION
#define IMU963_MAG_REF_DEVIATION             (0.5f)      /* 磁场模量相对参考值的允许偏差比例，0.5=±50% */
#endif

#ifndef IMU963_KALMAN_RESIDUAL_THRESHOLD_DEG
#define IMU963_KALMAN_RESIDUAL_THRESHOLD_DEG (30.0f)     /* Kalman 残差阈值，超此值累计一次异常 */
#endif

#ifndef IMU963_KALMAN_RESIDUAL_MAX_COUNT
#define IMU963_KALMAN_RESIDUAL_MAX_COUNT     (8u)        /* 连续异常次数达此值时自动重置滤波器 */
#endif

/* ============================== 数据结构定义 ============================== */

typedef struct
{
    int16 raw_acc_x;
    int16 raw_acc_y;
    int16 raw_acc_z;
    int16 raw_gyro_x;
    int16 raw_gyro_y;
    int16 raw_gyro_z;
    int16 raw_mag_x;
    int16 raw_mag_y;
    int16 raw_mag_z;

    float acc_x;                         /* 加速度 X，单位 g */
    float acc_y;                         /* 加速度 Y，单位 g */
    float acc_z;                         /* 加速度 Z，单位 g */
    float gyro_x;                        /* 陀螺仪 X，单位 °/s */
    float gyro_y;                        /* 陀螺仪 Y，单位 °/s */
    float gyro_z;                        /* 陀螺仪 Z，单位 °/s */
    float mag_x;                         /* 磁力计 X，单位 uT */
    float mag_y;                         /* 磁力计 Y，单位 uT */
    float mag_z;                         /* 磁力计 Z，单位 uT */

    float yaw_deg;                       /* 仅由磁力计计算得到的偏航角，范围 0~360° */
    float angle_deg;                     /* 校准并滤波后的最终偏航角，范围 0~360° */

    float yaw_bias_deg;                  /* 上电零点偏航角偏置 */
    float acc_x_bias;                    /* 上电 X 轴加速度偏置 */
    float acc_y_bias;                    /* 上电 Y 轴加速度偏置 */

    uint8 initialized;                   /* 1 表示已经初始化完成 */
} imu963_data_t;

typedef struct
{
    double x;                            /* 磁力计 X 数据，单位与 imu963_data.mag_x 一致，默认 uT */
    double y;                            /* 磁力计 Y 数据，单位与 imu963_data.mag_y 一致，默认 uT */
} imu963_calib_point_t;

typedef struct
{
    double x_offset;                     /* 输出参数 1：X_offset，硬铁 X 轴零偏 */
    double y_offset;                     /* 输出参数 2：Y_offset，硬铁 Y 轴零偏 */
    double sx;                           /* 输出参数 3：Sx，X 轴比例补偿 */
    double sy;                           /* 输出参数 4：Sy，Y 轴比例补偿 */

    double common_k;                     /* 通用比例因子 K，仅用于调试对照 */
    double kx;                           /* X 轴比例因子，等价于 sx */
    double ky;                           /* Y 轴比例因子，等价于 sy */
    double ellipse_a;                    /* 拟合椭圆 X 半轴 */
    double ellipse_b;                    /* 拟合椭圆 Y 半轴 */
    double radius_mean;                  /* 椭圆平均半径 */

    uint16 input_count;                  /* 输入点数 */
    uint16 valid_count;                  /* 去除无效点后的点数 */
    uint16 filtered_count;               /* 3σ 过滤后的点数 */

    uint8 fit_success;                   /* 1 表示椭圆拟合成功，0 表示使用备用估计 */
} imu963_calib_params_t;

extern imu963_data_t imu963_data;
extern imu963_calib_params_t imu963_mag_calib_params;

/* ============================== 普通初始化与读取 ============================== */

uint8 imu963_init(void);
void  imu963_set_dt_ms(float dt_ms);
void  imu963_update_raw(void);
void  imu963_calculate(void);
void  imu963_update(void);
float imu963_get_yaw_mag(void);
float imu963_get_angle(void);
uint8 imu963_keep_angle(void);
void  imu963_reset_filter(float angle_deg);
float imu963_normalize_360(float angle_deg);

/* ============================== 一体化校准初始化 ============================== */

/*
 * 函数简介：初始化 IMU，采集磁力计 XY 数据，计算校准参数，应用参数，再完成角度零点初始化
 * 参数说明：collect_count 采样点数，建议 200~500；传 0 使用 IMU963_CALIB_DEFAULT_COUNT
 * 参数说明：interval_ms   采样间隔，建议 50~150ms；传 0 使用 IMU963_CALIB_DEFAULT_INTERVAL_MS
 * 返回参数：0 成功，非 0 失败
 * 使用方法：上电后调用本函数，同时缓慢水平旋转小车或 IMU 模块 1~2 圈
 */
uint8 imu963_init_with_calibration(uint16 collect_count, uint16 interval_ms);

/*
 * 函数简介：使用已经准备好的 XY 数据完成校准初始化
 * 备注信息：适合把串口采集好的数据写成数组后，直接在板子上计算四个参数并进入正常读取
 */
uint8 imu963_init_with_calib_data(const imu963_calib_point_t *data, uint16 count);

/* ============================== 校准采集与计算接口 ============================== */

uint8  imu963_calib_collect_init(void);
uint8  imu963_calib_collect_point(imu963_calib_point_t *point);
uint16 imu963_calib_collect_points(imu963_calib_point_t *buffer, uint16 max_count, uint16 interval_ms);
uint16 imu963_calib_collect_internal(uint16 collect_count, uint16 interval_ms);
const imu963_calib_point_t *imu963_calib_get_buffer(void);
uint16 imu963_calib_get_count(void);

uint8 imu963_calib_calculate_params(const imu963_calib_point_t *data,
                                    uint16 count,
                                    imu963_calib_params_t *out);
void imu963_calib_get_four_params(const imu963_calib_params_t *params,
                                  double *x_offset,
                                  double *y_offset,
                                  double *sx,
                                  double *sy);
void imu963_calib_apply_params(const imu963_calib_params_t *params);
void imu963_set_mag_calibration(float x_offset, float y_offset, float x_scale, float y_scale);
void imu963_get_mag_calibration(float *x_offset, float *y_offset, float *x_scale, float *y_scale);
void imu963_load_default_mag_calibration(void);

/* ============================== 串口打印辅助函数 ============================== */

void imu963_calib_print_point(const imu963_calib_point_t *point);
void imu963_calib_print_four_params(const imu963_calib_params_t *params);
void imu963_calib_print_debug_params(const imu963_calib_params_t *params);
void imu963_calib_print_imu963_macros(const imu963_calib_params_t *params);

#ifdef __cplusplus
}
#endif

#endif
