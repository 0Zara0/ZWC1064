#include "imu963.h"

#include <math.h>
#include <string.h>
#include "zf_driver_delay.h"
#include "zf_device_imu963ra.h"

#ifndef IMU963_PI
#define IMU963_PI                              (3.14159265358979323846f)
#endif

imu963_data_t imu963_data = {0};

static float imu963_dt = (IMU963_SAMPLE_PERIOD_MS / 1000.0f);
static float imu963_q_bias = 0.0f;

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

void imu963_update_raw(void)
{
    imu963ra_get_acc();
    imu963ra_get_gyro();

    imu963_data.raw_acc_x = imu963ra_acc_x;
    imu963_data.raw_acc_y = imu963ra_acc_y;
    imu963_data.raw_acc_z = imu963ra_acc_z;
    imu963_data.raw_gyro_x = imu963ra_gyro_x;
    imu963_data.raw_gyro_y = imu963ra_gyro_y;
    imu963_data.raw_gyro_z = imu963ra_gyro_z;
}

void imu963_calculate(void)
{
    imu963_data.acc_x = (float)imu963_data.raw_acc_x / IMU963_ACC_RAW_SCALE;
    imu963_data.acc_y = (float)imu963_data.raw_acc_y / IMU963_ACC_RAW_SCALE;
    imu963_data.acc_z = (float)imu963_data.raw_acc_z / IMU963_ACC_RAW_SCALE;

    imu963_data.gyro_x = (float)imu963_data.raw_gyro_x / IMU963_GYRO_RAW_SCALE;
    imu963_data.gyro_y = (float)imu963_data.raw_gyro_y / IMU963_GYRO_RAW_SCALE;
    imu963_data.gyro_z = (float)imu963_data.raw_gyro_z / IMU963_GYRO_RAW_SCALE;
}

void imu963_update(void)
{
    imu963_update_raw();
    imu963_calculate();
}

float imu963_get_angle(void)
{
    imu963_update();

    imu963_data.angle_deg += (imu963_data.gyro_z - imu963_q_bias) * imu963_dt;

    imu963_data.angle_deg = imu963_normalize_360(imu963_data.angle_deg);

    return imu963_data.angle_deg;
}

uint8 imu963_keep_angle(void)
{
    uint16 valid_count;
    uint16 attempt_count;
    float gyro_z_sum = 0.0f;
    float accx_sum = 0.0f;
    float accy_sum = 0.0f;
    float prev_gyro_z_1 = 0.0f;
    float prev_gyro_z_2 = 0.0f;
    float prev_accx_1 = 0.0f;
    float prev_accx_2 = 0.0f;
    float prev_accy_1 = 0.0f;
    float prev_accy_2 = 0.0f;
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

    imu963_data.angle_deg = 0.0f;
    valid_count = 0u;
    attempt_count = 0u;
    prev_ready = 0u;
    while((valid_count < IMU963_KEEP_SAMPLE_COUNT) && (attempt_count < IMU963_KEEP_MAX_ATTEMPT))
    {
        attempt_count++;
        imu963_update();

        if(prev_ready)
        {
            if((fabsf(imu963_data.gyro_z - prev_gyro_z_1) > IMU963_GYRO_JUMP_THRESHOLD_DPS) ||
               (fabsf(imu963_data.gyro_z - prev_gyro_z_2) > IMU963_GYRO_JUMP_THRESHOLD_DPS))
            {
                prev_gyro_z_2 = prev_gyro_z_1;
                prev_gyro_z_1 = imu963_data.gyro_z;
                system_delay_ms((uint32)IMU963_SAMPLE_PERIOD_MS);
                continue;
            }
        }

        gyro_z_sum += imu963_data.gyro_z;
        valid_count++;
        prev_gyro_z_2 = prev_gyro_z_1;
        prev_gyro_z_1 = imu963_data.gyro_z;
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

    imu963_q_bias = gyro_z_sum / (float)valid_count;
    imu963_data.acc_x_bias = accx_sum / (float)IMU963_KEEP_SAMPLE_COUNT;
    imu963_data.acc_y_bias = accy_sum / (float)IMU963_KEEP_SAMPLE_COUNT;
    imu963_data.yaw_bias_deg = 0.0f;

    return 0u;
}

static uint8 imu963_base_start(void)
{
    uint8 state;

    memset(&imu963_data, 0, sizeof(imu963_data));
    imu963_data.initialized = 0u;
    imu963_q_bias = 0.0f;

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
