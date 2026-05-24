#include "imu963.h"

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "zf_driver_delay.h"
#include "zf_device_imu963ra.h"

#ifndef IMU963_PI
#define IMU963_PI                              (3.14159265358979323846f)
#endif

imu963_data_t imu963_data = {0};
imu963_calib_params_t imu963_mag_calib_params = {0};

static float imu963_dt = (IMU963_SAMPLE_PERIOD_MS / 1000.0f);

static float imu963_q_bias = 0.0f;
static float imu963_q_angle = 0.001f;
static float imu963_q_gyro = 0.001f;
static float imu963_r_angle = 0.08f;
static float imu963_p[2][2] = {{1.0f, 0.0f}, {0.0f, 1.0f}};
static uint8 imu963_kalman_valid = 0;
static float imu963_mag_ref_magnitude = 0.0f;
static uint16 imu963_residual_count = 0u;

/* 运行时磁力计校准参数。普通初始化时来自 imu963.h 中的四个宏；校准后会被新参数覆盖。 */
static float imu963_mag_x_offset = IMU963_MAG_X_OFFSET_UT;
static float imu963_mag_y_offset = IMU963_MAG_Y_OFFSET_UT;
static float imu963_mag_x_scale = IMU963_MAG_X_SCALE;
static float imu963_mag_y_scale = IMU963_MAG_Y_SCALE;

/* 一体化校准内部缓存。 */
static imu963_calib_point_t imu963_calib_buffer[IMU963_CALIB_MAX_POINTS];
static uint16 imu963_calib_buffer_count = 0u;

/* 椭圆拟合临时缓存。 */
static double s_x[IMU963_CALIB_MAX_POINTS];
static double s_y[IMU963_CALIB_MAX_POINTS];
static double s_x_filter[IMU963_CALIB_MAX_POINTS];
static double s_y_filter[IMU963_CALIB_MAX_POINTS];
static double s_x_norm[IMU963_CALIB_MAX_POINTS];
static double s_y_norm[IMU963_CALIB_MAX_POINTS];

typedef struct
{
    double x0;
    double y0;
    double a;
    double b;
} imu963_ellipse_param_t;

static uint8 imu963_base_start(void);
static float imu963_shortest_angle_delta(float target_deg, float current_deg);
static int imu963_is_finite_double(double value);
static double imu963_abs_double(double value);
static double imu963_max_abs_double(const double *data, uint16 count);
static double imu963_mean_double(const double *data, uint16 count);
static double imu963_std_double(const double *data, uint16 count);
static uint16 imu963_prepare_valid_points(const imu963_calib_point_t *data, uint16 count);
static uint16 imu963_filter_outliers(uint16 valid_count);
static double imu963_ellipse_sse(const imu963_ellipse_param_t *p, const double *x, const double *y, uint16 count);
static uint8 imu963_solve_4x4(double a[4][4], double b[4], double x[4]);
static void imu963_build_normal_equation(const imu963_ellipse_param_t *p,
                                         const double *x,
                                         const double *y,
                                         uint16 count,
                                         double lambda,
                                         double h[4][4],
                                         double g[4]);
static uint8 imu963_fit_ellipse_lm(const double *x, const double *y, uint16 count, imu963_ellipse_param_t *p);
static void imu963_fallback_estimate(const double *x, const double *y, uint16 count, imu963_ellipse_param_t *p);

float imu963_normalize_360(float angle_deg)
{
    while(angle_deg >= 360.0f)
    {
        angle_deg -= 360.0f;
    }
    while(angle_deg < 0.0f)
    {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

static float imu963_shortest_angle_delta(float target_deg, float current_deg)
{
    float delta = target_deg - current_deg;

    if(delta > 180.0f)
    {
        delta -= 360.0f;
    }
    else if(delta < -180.0f)
    {
        delta += 360.0f;
    }

    return delta;
}

void imu963_set_dt_ms(float dt_ms)
{
    if(dt_ms > 0.0f)
    {
        imu963_dt = dt_ms / 1000.0f;
    }
}

void imu963_load_default_mag_calibration(void)
{
    imu963_mag_x_offset = IMU963_MAG_X_OFFSET_UT;
    imu963_mag_y_offset = IMU963_MAG_Y_OFFSET_UT;
    imu963_mag_x_scale = IMU963_MAG_X_SCALE;
    imu963_mag_y_scale = IMU963_MAG_Y_SCALE;
}

void imu963_set_mag_calibration(float x_offset, float y_offset, float x_scale, float y_scale)
{
    imu963_mag_x_offset = x_offset;
    imu963_mag_y_offset = y_offset;

    if(fabsf(x_scale) > 1e-6f)
    {
        imu963_mag_x_scale = x_scale;
    }
    else
    {
        imu963_mag_x_scale = 1.0f;
    }

    if(fabsf(y_scale) > 1e-6f)
    {
        imu963_mag_y_scale = y_scale;
    }
    else
    {
        imu963_mag_y_scale = 1.0f;
    }
}

void imu963_get_mag_calibration(float *x_offset, float *y_offset, float *x_scale, float *y_scale)
{
    if(x_offset != 0) *x_offset = imu963_mag_x_offset;
    if(y_offset != 0) *y_offset = imu963_mag_y_offset;
    if(x_scale != 0) *x_scale = imu963_mag_x_scale;
    if(y_scale != 0) *y_scale = imu963_mag_y_scale;
}

void imu963_reset_filter(float angle_deg)
{
    imu963_data.angle_deg = imu963_normalize_360(angle_deg);
    imu963_q_bias = 0.0f;
    imu963_p[0][0] = 1.0f;
    imu963_p[0][1] = 0.0f;
    imu963_p[1][0] = 0.0f;
    imu963_p[1][1] = 1.0f;
    imu963_kalman_valid = 1u;
}

void imu963_update_raw(void)
{
    imu963ra_get_acc();
    imu963ra_get_gyro();
    imu963ra_get_mag();

    imu963_data.raw_acc_x = imu963ra_acc_x;
    imu963_data.raw_acc_y = imu963ra_acc_y;
    imu963_data.raw_acc_z = imu963ra_acc_z;
    imu963_data.raw_gyro_x = imu963ra_gyro_x;
    imu963_data.raw_gyro_y = imu963ra_gyro_y;
    imu963_data.raw_gyro_z = imu963ra_gyro_z;
    imu963_data.raw_mag_x = imu963ra_mag_x;
    imu963_data.raw_mag_y = imu963ra_mag_y;
    imu963_data.raw_mag_z = imu963ra_mag_z;
}

void imu963_calculate(void)
{
#if IMU963_USE_ZF_TRANSITION
    imu963_data.acc_x = imu963ra_acc_transition(imu963_data.raw_acc_x);
    imu963_data.acc_y = imu963ra_acc_transition(imu963_data.raw_acc_y);
    imu963_data.acc_z = imu963ra_acc_transition(imu963_data.raw_acc_z);

    imu963_data.gyro_x = imu963ra_gyro_transition(imu963_data.raw_gyro_x);
    imu963_data.gyro_y = imu963ra_gyro_transition(imu963_data.raw_gyro_y);
    imu963_data.gyro_z = imu963ra_gyro_transition(imu963_data.raw_gyro_z);

    imu963_data.mag_x = imu963ra_mag_transition(imu963_data.raw_mag_x) * 100.0f;
    imu963_data.mag_y = imu963ra_mag_transition(imu963_data.raw_mag_y) * 100.0f;
    imu963_data.mag_z = imu963ra_mag_transition(imu963_data.raw_mag_z) * 100.0f;
#else
    imu963_data.acc_x = (float)imu963_data.raw_acc_x / IMU963_ACC_RAW_SCALE;
    imu963_data.acc_y = (float)imu963_data.raw_acc_y / IMU963_ACC_RAW_SCALE;
    imu963_data.acc_z = (float)imu963_data.raw_acc_z / IMU963_ACC_RAW_SCALE;

    imu963_data.gyro_x = (float)imu963_data.raw_gyro_x / IMU963_GYRO_RAW_SCALE;
    imu963_data.gyro_y = (float)imu963_data.raw_gyro_y / IMU963_GYRO_RAW_SCALE;
    imu963_data.gyro_z = (float)imu963_data.raw_gyro_z / IMU963_GYRO_RAW_SCALE;

    imu963_data.mag_x = (float)imu963_data.raw_mag_x / IMU963_MAG_RAW_SCALE;
    imu963_data.mag_y = (float)imu963_data.raw_mag_y / IMU963_MAG_RAW_SCALE;
    imu963_data.mag_z = (float)imu963_data.raw_mag_z / IMU963_MAG_RAW_SCALE;
#endif
}

void imu963_update(void)
{
    imu963_update_raw();
    imu963_calculate();
}

float imu963_get_yaw_mag(void)
{
    float mag_x_corr;
    float mag_y_corr;
    float mag_z_corr;
    float x_cal;
    float y_cal;
    float z_cal;
    float pitch_rad;
    float roll_rad;
    float cos_pitch;
    float sin_pitch;
    float cos_roll;
    float sin_roll;
    float mag_x_horiz;
    float mag_y_horiz;
    float yaw_rad;

    mag_x_corr = imu963_data.mag_x - imu963_mag_x_offset;
    mag_y_corr = imu963_data.mag_y - imu963_mag_y_offset;
    mag_z_corr = imu963_data.mag_z;

    x_cal = mag_x_corr * imu963_mag_x_scale;
    y_cal = mag_y_corr * imu963_mag_y_scale;
    z_cal = mag_z_corr;

    pitch_rad = asinf(-imu963_data.acc_x);
    roll_rad  = asinf(imu963_data.acc_y);

    cos_pitch = cosf(pitch_rad);
    sin_pitch = sinf(pitch_rad);
    cos_roll  = cosf(roll_rad);
    sin_roll  = sinf(roll_rad);

    mag_x_horiz = x_cal * cos_pitch + z_cal * sin_pitch;
    mag_y_horiz = x_cal * sin_roll * sin_pitch + y_cal * cos_roll - z_cal * sin_roll * cos_pitch;

    yaw_rad = atan2f(-mag_y_horiz, mag_x_horiz);
    imu963_data.yaw_deg = yaw_rad * 180.0f / IMU963_PI;
    imu963_data.yaw_deg = imu963_normalize_360(imu963_data.yaw_deg);

    return imu963_data.yaw_deg;
}

static float imu963_kalman_filter(float mag_yaw, float gyro_z)
{
    float s;
    float k0;
    float k1;
    float angle_error;
    float p00_temp;
    float p01_temp;
    float mag_mag2;
    uint8 mag_ok;

    if(!imu963_kalman_valid)
    {
        imu963_reset_filter(mag_yaw);
    }

    imu963_data.angle_deg += (gyro_z - imu963_q_bias) * imu963_dt;

    imu963_p[0][0] += imu963_q_angle - (imu963_p[0][1] + imu963_p[1][0]) * imu963_dt;
    imu963_p[0][1] -= imu963_p[1][1] * imu963_dt;
    imu963_p[1][0] -= imu963_p[1][1] * imu963_dt;
    imu963_p[1][1] += imu963_q_gyro;

    mag_mag2 = imu963_data.mag_x * imu963_data.mag_x
             + imu963_data.mag_y * imu963_data.mag_y
             + imu963_data.mag_z * imu963_data.mag_z;

    if(imu963_mag_ref_magnitude > 1e-6f)
    {
        float dev = IMU963_MAG_REF_DEVIATION;
        float lo = imu963_mag_ref_magnitude * imu963_mag_ref_magnitude * (1.0f - dev) * (1.0f - dev);
        float hi = imu963_mag_ref_magnitude * imu963_mag_ref_magnitude * (1.0f + dev) * (1.0f + dev);
        mag_ok = (mag_mag2 > lo) && (mag_mag2 < hi);
    }
    else
    {
        mag_ok = 1u;
    }

    if(mag_ok)
    {
        s = imu963_p[0][0] + imu963_r_angle;
        k0 = imu963_p[0][0] / s;
        k1 = imu963_p[1][0] / s;

        angle_error = mag_yaw - imu963_data.angle_deg;
        if(angle_error > 180.0f)
        {
            angle_error -= 360.0f;
        }
        else if(angle_error < -180.0f)
        {
            angle_error += 360.0f;
        }

        if(fabsf(angle_error) > IMU963_KALMAN_RESIDUAL_THRESHOLD_DEG)
        {
            imu963_residual_count++;
            if(imu963_residual_count > IMU963_KALMAN_RESIDUAL_MAX_COUNT)
            {
                imu963_reset_filter(mag_yaw);
                imu963_residual_count = 0u;
                return imu963_data.angle_deg;
            }
        }
        else
        {
            if(imu963_residual_count > 0u) imu963_residual_count--;
        }

        imu963_data.angle_deg += k0 * angle_error;
        imu963_q_bias += k1 * angle_error;

        p00_temp = imu963_p[0][0];
        p01_temp = imu963_p[0][1];

        imu963_p[0][0] -= k0 * p00_temp;
        imu963_p[0][1] -= k0 * p01_temp;
        imu963_p[1][0] -= k1 * p00_temp;
        imu963_p[1][1] -= k1 * p01_temp;
    }

    imu963_data.angle_deg = imu963_normalize_360(imu963_data.angle_deg);
    return imu963_data.angle_deg;
}

float imu963_get_angle(void)
{
    imu963_update();
    imu963_get_yaw_mag();
    imu963_kalman_filter(imu963_data.yaw_deg, imu963_data.gyro_z);

    imu963_data.angle_deg = imu963_shortest_angle_delta(imu963_data.angle_deg, imu963_data.yaw_bias_deg);
    imu963_data.angle_deg = imu963_normalize_360(imu963_data.angle_deg);

    return imu963_data.angle_deg;
}

uint8 imu963_keep_angle(void)
{
    uint16 valid_count;
    uint16 attempt_count;
    float yaw_sum = 0.0f;
    float accx_sum = 0.0f;
    float accy_sum = 0.0f;
    float prev_angle_1 = 0.0f;
    float prev_angle_2 = 0.0f;
    float prev_accx_1 = 0.0f;
    float prev_accx_2 = 0.0f;
    float prev_accy_1 = 0.0f;
    float prev_accy_2 = 0.0f;
    float current_angle;
    uint8 prev_ready;

    valid_count = 0u;
    attempt_count = 0u;
    prev_ready = 0u;
    while((valid_count < IMU963_KEEP_SAMPLE_COUNT) && (attempt_count < IMU963_KEEP_MAX_ATTEMPT))
    {
        attempt_count++;
        imu963_update();

        if(prev_ready)
        {
            if((fabsf(imu963_data.acc_x - prev_accx_1) > IMU963_ACC_JUMP_THRESHOLD_G) ||
               (fabsf(imu963_data.acc_x - prev_accx_2) > IMU963_ACC_JUMP_THRESHOLD_G))
            {
                prev_accx_2 = prev_accx_1;
                prev_accx_1 = imu963_data.acc_x;
                system_delay_ms((uint32)IMU963_SAMPLE_PERIOD_MS);
                continue;
            }
        }

        accx_sum += imu963_data.acc_x;
        valid_count++;
        prev_accx_2 = prev_accx_1;
        prev_accx_1 = imu963_data.acc_x;
        if(valid_count >= 2u)
        {
            prev_ready = 1u;
        }
        system_delay_ms((uint32)IMU963_SAMPLE_PERIOD_MS);
    }

    if(valid_count < IMU963_KEEP_SAMPLE_COUNT)
    {
        return 1u;
    }

    valid_count = 0u;
    attempt_count = 0u;
    prev_ready = 0u;
    while((valid_count < IMU963_KEEP_SAMPLE_COUNT) && (attempt_count < IMU963_KEEP_MAX_ATTEMPT))
    {
        attempt_count++;
        imu963_update();

        if(prev_ready)
        {
            if((fabsf(imu963_data.acc_y - prev_accy_1) > IMU963_ACC_JUMP_THRESHOLD_G) ||
               (fabsf(imu963_data.acc_y - prev_accy_2) > IMU963_ACC_JUMP_THRESHOLD_G))
            {
                prev_accy_2 = prev_accy_1;
                prev_accy_1 = imu963_data.acc_y;
                system_delay_ms((uint32)IMU963_SAMPLE_PERIOD_MS);
                continue;
            }
        }

        accy_sum += imu963_data.acc_y;
        valid_count++;
        prev_accy_2 = prev_accy_1;
        prev_accy_1 = imu963_data.acc_y;
        if(valid_count >= 2u)
        {
            prev_ready = 1u;
        }
        system_delay_ms((uint32)IMU963_SAMPLE_PERIOD_MS);
    }

    if(valid_count < IMU963_KEEP_SAMPLE_COUNT)
    {
        return 1u;
    }

    imu963_reset_filter(0.0f);
    valid_count = 0u;
    attempt_count = 0u;
    prev_ready = 0u;
    while((valid_count < IMU963_KEEP_SAMPLE_COUNT) && (attempt_count < IMU963_KEEP_MAX_ATTEMPT))
    {
        attempt_count++;
        imu963_update();
        imu963_get_yaw_mag();
        current_angle = imu963_kalman_filter(imu963_data.yaw_deg, imu963_data.gyro_z);

        if(prev_ready)
        {
            if((fabsf(imu963_shortest_angle_delta(current_angle, prev_angle_1)) > IMU963_ANGLE_JUMP_THRESHOLD_DEG) ||
               (fabsf(imu963_shortest_angle_delta(current_angle, prev_angle_2)) > IMU963_ANGLE_JUMP_THRESHOLD_DEG))
            {
                prev_angle_2 = prev_angle_1;
                prev_angle_1 = current_angle;
                system_delay_ms((uint32)IMU963_SAMPLE_PERIOD_MS);
                continue;
            }
        }

        yaw_sum += current_angle;
        valid_count++;
        prev_angle_2 = prev_angle_1;
        prev_angle_1 = current_angle;
        if(valid_count >= 2u)
        {
            prev_ready = 1u;
        }
        system_delay_ms((uint32)IMU963_SAMPLE_PERIOD_MS);
    }

    if(valid_count < IMU963_KEEP_SAMPLE_COUNT)
    {
        return 1u;
    }

    imu963_data.yaw_bias_deg = yaw_sum / (float)valid_count;
    imu963_data.acc_x_bias = accx_sum / (float)IMU963_KEEP_SAMPLE_COUNT;
    imu963_data.acc_y_bias = accy_sum / (float)IMU963_KEEP_SAMPLE_COUNT;

    imu963_mag_ref_magnitude = sqrtf(imu963_data.mag_x * imu963_data.mag_x
                                   + imu963_data.mag_y * imu963_data.mag_y
                                   + imu963_data.mag_z * imu963_data.mag_z);

    imu963_reset_filter(imu963_data.yaw_bias_deg);
    return 0u;
}

static uint8 imu963_base_start(void)
{
    uint8 state;

    memset(&imu963_data, 0, sizeof(imu963_data));
    imu963_data.initialized = 0u;
    imu963_kalman_valid = 0u;
    imu963_q_bias = 0.0f;
    imu963_mag_ref_magnitude = 0.0f;
    imu963_residual_count = 0u;
    imu963_load_default_mag_calibration();

    state = imu963ra_init();
    if(state != 0u)
    {
        return state;
    }

    system_delay_ms(100u);
    imu963_set_dt_ms(IMU963_SAMPLE_PERIOD_MS);
    return 0u;
}

uint8 imu963_init(void)
{
    uint8 state;

    state = imu963_base_start();
    if(state != 0u)
    {
        return state;
    }

    state = imu963_keep_angle();
    if(state != 0u)
    {
        return state;
    }

    imu963_data.initialized = 1u;
    return 0u;
}

uint8 imu963_init_with_calibration(uint16 collect_count, uint16 interval_ms)
{
    uint8 state;
    uint16 actual_count;

    if(collect_count == 0u)
    {
        collect_count = IMU963_CALIB_DEFAULT_COUNT;
    }
    if(interval_ms == 0u)
    {
        interval_ms = IMU963_CALIB_DEFAULT_INTERVAL_MS;
    }

    state = imu963_calib_collect_init();
    if(state != 0u)
    {
        return state;
    }

    actual_count = imu963_calib_collect_internal(collect_count, interval_ms);
    if(actual_count < IMU963_CALIB_MIN_VALID_POINTS)
    {
        return 2u;
    }

    state = imu963_calib_calculate_params(imu963_calib_get_buffer(), actual_count, &imu963_mag_calib_params);
    if(state != 0u)
    {
        return (uint8)(10u + state);
    }

    imu963_calib_apply_params(&imu963_mag_calib_params);

#if IMU963_CALIB_PRINT_ENABLE
    imu963_calib_print_imu963_macros(&imu963_mag_calib_params);
#endif

    state = imu963_keep_angle();
    if(state != 0u)
    {
        return state;
    }

    imu963_data.initialized = 1u;
    return 0u;
}

uint8 imu963_init_with_calib_data(const imu963_calib_point_t *data, uint16 count)
{
    uint8 state;

    state = imu963_base_start();
    if(state != 0u)
    {
        return state;
    }

    state = imu963_calib_calculate_params(data, count, &imu963_mag_calib_params);
    if(state != 0u)
    {
        return (uint8)(10u + state);
    }

    imu963_calib_apply_params(&imu963_mag_calib_params);

#if IMU963_CALIB_PRINT_ENABLE
    imu963_calib_print_imu963_macros(&imu963_mag_calib_params);
#endif

    state = imu963_keep_angle();
    if(state != 0u)
    {
        return state;
    }

    imu963_data.initialized = 1u;
    return 0u;
}

uint8 imu963_calib_collect_init(void)
{
    uint8 state;

    state = imu963_base_start();
    if(state != 0u)
    {
        return state;
    }

    imu963_calib_buffer_count = 0u;
    return 0u;
}

uint8 imu963_calib_collect_point(imu963_calib_point_t *point)
{
    if(point == 0)
    {
        return 1u;
    }

    imu963_update();
    point->x = (double)imu963_data.mag_x;
    point->y = (double)imu963_data.mag_y;
    return 0u;
}

uint16 imu963_calib_collect_points(imu963_calib_point_t *buffer, uint16 max_count, uint16 interval_ms)
{
    uint16 i;
    uint16 actual_count = 0u;

    if((buffer == 0) || (max_count == 0u))
    {
        return 0u;
    }

    for(i = 0u; i < max_count; i++)
    {
        if(imu963_calib_collect_point(&buffer[actual_count]) == 0u)
        {
            actual_count++;
        }
        if(interval_ms > 0u)
        {
            system_delay_ms((uint32)interval_ms);
        }
    }

    return actual_count;
}

uint16 imu963_calib_collect_internal(uint16 collect_count, uint16 interval_ms)
{
    uint16 max_count;

    max_count = collect_count;
    if(max_count > IMU963_CALIB_MAX_POINTS)
    {
        max_count = IMU963_CALIB_MAX_POINTS;
    }

    imu963_calib_buffer_count = imu963_calib_collect_points(imu963_calib_buffer, max_count, interval_ms);
    return imu963_calib_buffer_count;
}

const imu963_calib_point_t *imu963_calib_get_buffer(void)
{
    return imu963_calib_buffer;
}

uint16 imu963_calib_get_count(void)
{
    return imu963_calib_buffer_count;
}

/* ============================== 磁力计 XY 椭圆拟合校准算法 ============================== */

static int imu963_is_finite_double(double value)
{
    return (value == value) && (value < DBL_MAX) && (value > -DBL_MAX);
}

static double imu963_abs_double(double value)
{
    return (value >= 0.0) ? value : -value;
}

static double imu963_max_abs_double(const double *data, uint16 count)
{
    uint16 i;
    double max_value = 0.0;

    for(i = 0u; i < count; i++)
    {
        double abs_value = imu963_abs_double(data[i]);
        if(abs_value > max_value)
        {
            max_value = abs_value;
        }
    }

    return max_value;
}

static double imu963_mean_double(const double *data, uint16 count)
{
    uint16 i;
    double sum = 0.0;

    if(count == 0u)
    {
        return 0.0;
    }

    for(i = 0u; i < count; i++)
    {
        sum += data[i];
    }

    return sum / (double)count;
}

static double imu963_std_double(const double *data, uint16 count)
{
    uint16 i;
    double mean_value;
    double sum = 0.0;

    if(count <= 1u)
    {
        return 0.0;
    }

    mean_value = imu963_mean_double(data, count);
    for(i = 0u; i < count; i++)
    {
        double diff = data[i] - mean_value;
        sum += diff * diff;
    }

    return sqrt(sum / (double)count);
}

static uint16 imu963_prepare_valid_points(const imu963_calib_point_t *data, uint16 count)
{
    uint16 i;
    uint16 valid_count = 0u;

    if(count > IMU963_CALIB_MAX_POINTS)
    {
        count = IMU963_CALIB_MAX_POINTS;
    }

    for(i = 0u; i < count; i++)
    {
        double x = data[i].x;
        double y = data[i].y;

        if(!imu963_is_finite_double(x) || !imu963_is_finite_double(y))
        {
            continue;
        }

#if IMU963_CALIB_SKIP_ZERO_POINTS
        if((imu963_abs_double(x) < 1e-12) && (imu963_abs_double(y) < 1e-12))
        {
            continue;
        }
#endif

        s_x[valid_count] = x;
        s_y[valid_count] = y;
        valid_count++;
    }

    return valid_count;
}

static uint16 imu963_filter_outliers(uint16 valid_count)
{
    uint16 i;
    uint16 filtered_count = 0u;
    double r_mean;
    double r_std;
    double low;
    double high;
    double radius[IMU963_CALIB_MAX_POINTS];

    if(valid_count == 0u)
    {
        return 0u;
    }

    for(i = 0u; i < valid_count; i++)
    {
        radius[i] = sqrt(s_x[i] * s_x[i] + s_y[i] * s_y[i]);
    }

    r_mean = imu963_mean_double(radius, valid_count);
    r_std = imu963_std_double(radius, valid_count);
    low = r_mean - IMU963_CALIB_SIGMA * r_std;
    high = r_mean + IMU963_CALIB_SIGMA * r_std;

    for(i = 0u; i < valid_count; i++)
    {
        if((radius[i] >= low) && (radius[i] <= high))
        {
            s_x_filter[filtered_count] = s_x[i];
            s_y_filter[filtered_count] = s_y[i];
            filtered_count++;
        }
    }

    return filtered_count;
}

static double imu963_ellipse_sse(const imu963_ellipse_param_t *p, const double *x, const double *y, uint16 count)
{
    uint16 i;
    double sse = 0.0;
    double a2;
    double b2;

    if((p->a <= 1e-12) || (p->b <= 1e-12))
    {
        return DBL_MAX;
    }

    a2 = p->a * p->a;
    b2 = p->b * p->b;

    for(i = 0u; i < count; i++)
    {
        double dx = x[i] - p->x0;
        double dy = y[i] - p->y0;
        double r = (dx * dx / a2) + (dy * dy / b2) - 1.0;
        sse += r * r;
    }

    return sse;
}

static uint8 imu963_solve_4x4(double a[4][4], double b[4], double x[4])
{
    int i;
    int j;
    int k;
    double aug[4][5];

    for(i = 0; i < 4; i++)
    {
        for(j = 0; j < 4; j++)
        {
            aug[i][j] = a[i][j];
        }
        aug[i][4] = b[i];
    }

    for(i = 0; i < 4; i++)
    {
        int pivot = i;
        double pivot_abs = imu963_abs_double(aug[i][i]);

        for(j = i + 1; j < 4; j++)
        {
            double value_abs = imu963_abs_double(aug[j][i]);
            if(value_abs > pivot_abs)
            {
                pivot = j;
                pivot_abs = value_abs;
            }
        }

        if(pivot_abs < 1e-18)
        {
            return 1u;
        }

        if(pivot != i)
        {
            for(k = i; k < 5; k++)
            {
                double temp = aug[i][k];
                aug[i][k] = aug[pivot][k];
                aug[pivot][k] = temp;
            }
        }

        {
            double div = aug[i][i];
            for(k = i; k < 5; k++)
            {
                aug[i][k] /= div;
            }
        }

        for(j = 0; j < 4; j++)
        {
            if(j != i)
            {
                double factor = aug[j][i];
                for(k = i; k < 5; k++)
                {
                    aug[j][k] -= factor * aug[i][k];
                }
            }
        }
    }

    for(i = 0; i < 4; i++)
    {
        x[i] = aug[i][4];
    }

    return 0u;
}

static void imu963_build_normal_equation(const imu963_ellipse_param_t *p,
                                         const double *x,
                                         const double *y,
                                         uint16 count,
                                         double lambda,
                                         double h[4][4],
                                         double g[4])
{
    uint16 i;
    int row;
    int col;
    double a2 = p->a * p->a;
    double b2 = p->b * p->b;
    double a3 = a2 * p->a;
    double b3 = b2 * p->b;

    memset(h, 0, sizeof(double) * 16u);
    memset(g, 0, sizeof(double) * 4u);

    for(i = 0u; i < count; i++)
    {
        double dx = x[i] - p->x0;
        double dy = y[i] - p->y0;
        double residual = (dx * dx / a2) + (dy * dy / b2) - 1.0;
        double jv[4];

        jv[0] = -2.0 * dx / a2;
        jv[1] = -2.0 * dy / b2;
        jv[2] = -2.0 * dx * dx / a3;
        jv[3] = -2.0 * dy * dy / b3;

        for(row = 0; row < 4; row++)
        {
            g[row] += -jv[row] * residual;
            for(col = 0; col < 4; col++)
            {
                h[row][col] += jv[row] * jv[col];
            }
        }
    }

    for(row = 0; row < 4; row++)
    {
        double diag = h[row][row];
        if(diag < 1e-18)
        {
            diag = 1.0;
        }
        h[row][row] += lambda * diag;
    }
}

static uint8 imu963_fit_ellipse_lm(const double *x, const double *y, uint16 count, imu963_ellipse_param_t *p)
{
    uint16 iter;
    double lambda = 1e-3;
    double old_sse;

    p->x0 = imu963_mean_double(x, count);
    p->y0 = imu963_mean_double(y, count);
    p->a = imu963_std_double(x, count) * 2.0;
    p->b = imu963_std_double(y, count) * 2.0;

    if(p->a < 1e-6)
    {
        p->a = 1.0;
    }
    if(p->b < 1e-6)
    {
        p->b = 1.0;
    }

    old_sse = imu963_ellipse_sse(p, x, y, count);

    for(iter = 0u; iter < IMU963_CALIB_MAX_ITER; iter++)
    {
        double h[4][4];
        double g[4];
        double delta[4];
        imu963_ellipse_param_t trial;
        double new_sse;

        imu963_build_normal_equation(p, x, y, count, lambda, h, g);

        if(imu963_solve_4x4(h, g, delta) != 0u)
        {
            lambda *= 10.0;
            continue;
        }

        trial.x0 = p->x0 + delta[0];
        trial.y0 = p->y0 + delta[1];
        trial.a = p->a + delta[2];
        trial.b = p->b + delta[3];

        if(trial.a < 0.0)
        {
            trial.a = -trial.a;
        }
        if(trial.b < 0.0)
        {
            trial.b = -trial.b;
        }

        if((trial.a < 1e-8) || (trial.b < 1e-8))
        {
            lambda *= 10.0;
            continue;
        }

        new_sse = imu963_ellipse_sse(&trial, x, y, count);

        if(new_sse < old_sse)
        {
            double step_sum = imu963_abs_double(delta[0]) +
                              imu963_abs_double(delta[1]) +
                              imu963_abs_double(delta[2]) +
                              imu963_abs_double(delta[3]);

            *p = trial;
            old_sse = new_sse;
            lambda *= 0.3;

            if(step_sum < 1e-10)
            {
                return 0u;
            }
        }
        else
        {
            lambda *= 10.0;
        }
    }

    return 0u;
}

static void imu963_fallback_estimate(const double *x, const double *y, uint16 count, imu963_ellipse_param_t *p)
{
    uint16 i;
    double xmin = x[0];
    double xmax = x[0];
    double ymin = y[0];
    double ymax = y[0];

    for(i = 1u; i < count; i++)
    {
        if(x[i] < xmin) xmin = x[i];
        if(x[i] > xmax) xmax = x[i];
        if(y[i] < ymin) ymin = y[i];
        if(y[i] > ymax) ymax = y[i];
    }

    p->x0 = (xmin + xmax) * 0.5;
    p->y0 = (ymin + ymax) * 0.5;
    p->a = (xmax - xmin) * 0.5;
    p->b = (ymax - ymin) * 0.5;

    if(p->a < 1e-6) p->a = 1.0;
    if(p->b < 1e-6) p->b = 1.0;
}

uint8 imu963_calib_calculate_params(const imu963_calib_point_t *data, uint16 count, imu963_calib_params_t *out)
{
    uint16 i;
    uint16 valid_count;
    uint16 filtered_count;
    double max_abs_x;
    double max_abs_y;
    imu963_ellipse_param_t fit_norm;
    imu963_ellipse_param_t fit_real;
    double radius_sum = 0.0;

    if((data == 0) || (out == 0) || (count == 0u))
    {
        return 1u;
    }

    memset(out, 0, sizeof(*out));
    out->input_count = count;

    valid_count = imu963_prepare_valid_points(data, count);
    out->valid_count = valid_count;

    if(valid_count < IMU963_CALIB_MIN_VALID_POINTS)
    {
        return 2u;
    }

    filtered_count = imu963_filter_outliers(valid_count);
    out->filtered_count = filtered_count;

    if(filtered_count < IMU963_CALIB_MIN_VALID_POINTS)
    {
        return 3u;
    }

    max_abs_x = imu963_max_abs_double(s_x_filter, filtered_count);
    max_abs_y = imu963_max_abs_double(s_y_filter, filtered_count);

    if((max_abs_x < 1e-12) || (max_abs_y < 1e-12))
    {
        return 4u;
    }

    for(i = 0u; i < filtered_count; i++)
    {
        s_x_norm[i] = s_x_filter[i] / max_abs_x;
        s_y_norm[i] = s_y_filter[i] / max_abs_y;
    }

    if(imu963_fit_ellipse_lm(s_x_norm, s_y_norm, filtered_count, &fit_norm) == 0u)
    {
        out->fit_success = 1u;
    }
    else
    {
        imu963_fallback_estimate(s_x_norm, s_y_norm, filtered_count, &fit_norm);
        out->fit_success = 0u;
    }

    fit_real.x0 = fit_norm.x0 * max_abs_x;
    fit_real.y0 = fit_norm.y0 * max_abs_y;
    fit_real.a = fit_norm.a * max_abs_x;
    fit_real.b = fit_norm.b * max_abs_y;

    if((fit_real.a < 1e-12) || (fit_real.b < 1e-12) ||
       !imu963_is_finite_double(fit_real.x0) || !imu963_is_finite_double(fit_real.y0) ||
       !imu963_is_finite_double(fit_real.a) || !imu963_is_finite_double(fit_real.b))
    {
        imu963_fallback_estimate(s_x_filter, s_y_filter, filtered_count, &fit_real);
        out->fit_success = 0u;
    }

    out->radius_mean = (fit_real.a + fit_real.b) * 0.5;

    for(i = 0u; i < filtered_count; i++)
    {
        double dx = s_x_filter[i] - fit_real.x0;
        double dy = s_y_filter[i] - fit_real.y0;
        radius_sum += sqrt(dx * dx + dy * dy);
    }

    if(radius_sum > 1e-12)
    {
        out->common_k = out->radius_mean / (radius_sum / (double)filtered_count);
    }
    else
    {
        out->common_k = 1.0;
    }

    out->kx = out->radius_mean / fit_real.a;
    out->ky = out->radius_mean / fit_real.b;

    if(!imu963_is_finite_double(out->common_k)) out->common_k = 1.0;
    if(!imu963_is_finite_double(out->kx)) out->kx = 1.0;
    if(!imu963_is_finite_double(out->ky)) out->ky = 1.0;

    out->x_offset = fit_real.x0;
    out->y_offset = fit_real.y0;
    out->sx = out->kx;
    out->sy = out->ky;
    out->ellipse_a = fit_real.a;
    out->ellipse_b = fit_real.b;

    return 0u;
}

void imu963_calib_get_four_params(const imu963_calib_params_t *params,
                                  double *x_offset,
                                  double *y_offset,
                                  double *sx,
                                  double *sy)
{
    if(params == 0)
    {
        return;
    }

    if(x_offset != 0) *x_offset = params->x_offset;
    if(y_offset != 0) *y_offset = params->y_offset;
    if(sx != 0) *sx = params->sx;
    if(sy != 0) *sy = params->sy;
}

void imu963_calib_apply_params(const imu963_calib_params_t *params)
{
    if(params == 0)
    {
        return;
    }

    imu963_set_mag_calibration((float)params->x_offset,
                               (float)params->y_offset,
                               (float)params->sx,
                               (float)params->sy);
}

void imu963_calib_print_point(const imu963_calib_point_t *point)
{
    if(point == 0)
    {
        return;
    }

    printf("%.6f,%.6f\r\n", point->x, point->y);
}

void imu963_calib_print_four_params(const imu963_calib_params_t *params)
{
    if(params == 0)
    {
        return;
    }

    printf("X_offset = %.4f\r\n", params->x_offset);
    printf("Y_offset = %.4f\r\n", params->y_offset);
    printf("Sx = %.4f\r\n", params->sx);
    printf("Sy = %.4f\r\n", params->sy);
}

void imu963_calib_print_debug_params(const imu963_calib_params_t *params)
{
    if(params == 0)
    {
        return;
    }

    printf("============================================================\r\n");
    printf("IMU963RA 磁力计 XY 校准参数\r\n");
    printf("============================================================\r\n");
    printf("输入点数: %u\r\n", (unsigned int)params->input_count);
    printf("有效点数: %u\r\n", (unsigned int)params->valid_count);
    printf("过滤后点数: %u\r\n", (unsigned int)params->filtered_count);
    printf("拟合状态: %s\r\n", params->fit_success ? "椭圆拟合成功" : "使用备用估计");
    printf("硬铁偏差: X_offset = %.4f, Y_offset = %.4f\r\n", params->x_offset, params->y_offset);
    printf("软铁补偿: Sx = %.4f, Sy = %.4f\r\n", params->sx, params->sy);
    printf("对照参数: K = %.4f, Kx = %.4f, Ky = %.4f\r\n", params->common_k, params->kx, params->ky);
    printf("椭圆半轴: a = %.4f, b = %.4f\r\n", params->ellipse_a, params->ellipse_b);
    printf("============================================================\r\n");
}

void imu963_calib_print_imu963_macros(const imu963_calib_params_t *params)
{
    if(params == 0)
    {
        return;
    }

    printf("/* Copy these four lines to imu963.h */\r\n");
    printf("#define IMU963_MAG_X_OFFSET_UT               (%.4ff)\r\n", params->x_offset);
    printf("#define IMU963_MAG_Y_OFFSET_UT               (%.4ff)\r\n", params->y_offset);
    printf("#define IMU963_MAG_X_SCALE                   (%.4ff)\r\n", params->sx);
    printf("#define IMU963_MAG_Y_SCALE                   (%.4ff)\r\n", params->sy);
}
