#include "mpu6050.h"
#include "iic.h"
#include "main.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define FILTER_ALPHA             0.995f
#define RAD_TO_DEG               57.29578f
#define BIAS_COUNT               200
#define INIT_ACCEL_NORM_MIN_SQ   0.49f
#define INIT_ACCEL_NORM_MAX_SQ   1.69f

static float pitch_bias = 0.0f;
static float pitch_angle = 0.0f;
static float pure_gyro_pitch = 0.0f;

static uint8_t MPU6050_Write_Byte(uint8_t reg, uint8_t data)
{
    IIC_Start();
    IIC_Send_Byte(MPU6050_ADDR << 1);
    if (IIC_Wait_Ack()) { IIC_Stop(); return MPU6050_FAIL; }

    IIC_Send_Byte(reg);
    if (IIC_Wait_Ack()) { IIC_Stop(); return MPU6050_FAIL; }

    IIC_Send_Byte(data);
    if (IIC_Wait_Ack()) { IIC_Stop(); return MPU6050_FAIL; }

    IIC_Stop();
    return MPU6050_OK;
}

static uint8_t MPU6050_Read_Bytes(uint8_t reg, uint8_t *data, uint8_t len)
{
    uint8_t i;

    if (data == 0 || len == 0) return MPU6050_FAIL;

    IIC_Start();
    IIC_Send_Byte(MPU6050_ADDR << 1);
    if (IIC_Wait_Ack()) { IIC_Stop(); return MPU6050_FAIL; }

    IIC_Send_Byte(reg);
    if (IIC_Wait_Ack()) { IIC_Stop(); return MPU6050_FAIL; }

    IIC_Start();
    IIC_Send_Byte((MPU6050_ADDR << 1) | 0x01);
    if (IIC_Wait_Ack()) { IIC_Stop(); return MPU6050_FAIL; }

    for (i = 0; i < len; i++) {
        data[i] = (i < (len - 1U)) ? IIC_Read_Byte(1) : IIC_Read_Byte(0);
    }

    IIC_Stop();
    return MPU6050_OK;
}

uint8_t MPU6050_Read_RawData(MPU6050_RawData *raw)
{
    uint8_t buffer[14];

    if (raw == 0) return MPU6050_FAIL;

    if (MPU6050_Read_Bytes(MPU6050_ACCEL_XOUT_H, buffer, 14) != MPU6050_OK) {
        return MPU6050_FAIL;
    }

    raw->ax_raw = (int16_t)((buffer[0] << 8) | buffer[1]);
    raw->ay_raw = (int16_t)((buffer[2] << 8) | buffer[3]);
    raw->az_raw = (int16_t)((buffer[4] << 8) | buffer[5]);
    raw->temp_raw = (int16_t)((buffer[6] << 8) | buffer[7]);
    raw->gx_raw = (int16_t)((buffer[8] << 8) | buffer[9]);
    raw->gy_raw = (int16_t)((buffer[10] << 8) | buffer[11]);
    raw->gz_raw = (int16_t)((buffer[12] << 8) | buffer[13]);

    return MPU6050_OK;
}

uint8_t MPU6050_Init(void)
{
    uint8_t who_am_i = 0;
    int32_t gyro_y_sum = 0;
    MPU6050_RawData raw;
    float ax_g, ay_g, az_g, accel_norm_sq;
    int i;

    IIC_Init();

    if (MPU6050_Write_Byte(MPU6050_PWR_MGMT_1, MPU6050_PWR_MGMT_1_VALUE) != MPU6050_OK)
        return MPU6050_FAIL;
    delay_ms(10);

    if (MPU6050_Read_Bytes(MPU6050_WHO_AM_I, &who_am_i, 1) != MPU6050_OK)
        return MPU6050_FAIL;
    if (who_am_i != MPU6050_ADDR) return MPU6050_FAIL;

    if (MPU6050_Write_Byte(MPU6050_SMPLRT_DIV, MPU6050_SMPLRT_DIV_VALUE) != MPU6050_OK)
        return MPU6050_FAIL;
    if (MPU6050_Write_Byte(MPU6050_CONFIG, MPU6050_CONFIG_VALUE) != MPU6050_OK)
        return MPU6050_FAIL;
    if (MPU6050_Write_Byte(MPU6050_ACCEL_CONFIG, MPU6050_ACCEL_CONFIG_VALUE) != MPU6050_OK)
        return MPU6050_FAIL;
    if (MPU6050_Write_Byte(MPU6050_GYRO_CONFIG, MPU6050_GYRO_CONFIG_VALUE) != MPU6050_OK)
        return MPU6050_FAIL;
    if (MPU6050_Write_Byte(MPU6050_PWR_MGMT_2, MPU6050_PWR_MGMT_2_VALUE) != MPU6050_OK)
        return MPU6050_FAIL;

    delay_ms(50);

    for (i = 0; i < BIAS_COUNT; i++) {
        if (MPU6050_Read_RawData(&raw) != MPU6050_OK) return MPU6050_FAIL;
        gyro_y_sum += raw.gy_raw;
        delay_ms(2);
    }

    pitch_bias = (float)gyro_y_sum / (float)BIAS_COUNT / MPU6050_GYRO_SENSITIVITY_LSB_PER_DPS;

    if (MPU6050_Read_RawData(&raw) != MPU6050_OK) return MPU6050_FAIL;

    ax_g = (float)raw.ax_raw / MPU6050_ACCEL_SENSITIVITY_LSB_PER_G;
    ay_g = (float)raw.ay_raw / MPU6050_ACCEL_SENSITIVITY_LSB_PER_G;
    az_g = (float)raw.az_raw / MPU6050_ACCEL_SENSITIVITY_LSB_PER_G;
    accel_norm_sq = ax_g * ax_g + ay_g * ay_g + az_g * az_g;
    (void)accel_norm_sq;

    pitch_angle = atan2f(-ax_g, az_g) * RAD_TO_DEG;
    pure_gyro_pitch = pitch_angle;

    delay_ms(10);
    return MPU6050_OK;
}

uint8_t MPU6050_Read_Data(MPU6050_Data *data)
{
    MPU6050_RawData raw;

    if (data == 0) return MPU6050_FAIL;
    if (MPU6050_Read_RawData(&raw) != MPU6050_OK) return MPU6050_FAIL;

    data->ax = (float)raw.ax_raw / MPU6050_ACCEL_SENSITIVITY_LSB_PER_G;
    data->ay = (float)raw.ay_raw / MPU6050_ACCEL_SENSITIVITY_LSB_PER_G;
    data->az = (float)raw.az_raw / MPU6050_ACCEL_SENSITIVITY_LSB_PER_G;
    data->temp = (float)raw.temp_raw / 340.0f + 36.53f;
    data->gx = (float)raw.gx_raw / MPU6050_GYRO_SENSITIVITY_LSB_PER_DPS;
    data->gy = (float)raw.gy_raw / MPU6050_GYRO_SENSITIVITY_LSB_PER_DPS;
    data->gz = (float)raw.gz_raw / MPU6050_GYRO_SENSITIVITY_LSB_PER_DPS;

    return MPU6050_OK;
}

float MPU6050_Get_Accel_Pitch(MPU6050_Data *data)
{
    return atan2f(-data->ax, data->az) * RAD_TO_DEG;
}

void MPU6050_Calculate_Attitude(MPU6050_Data *data, Attitude_Data *attitude, float dt)
{
    float gyro_pitch;
    float accel_pitch;

    gyro_pitch = data->gy - pitch_bias;
    pitch_angle += gyro_pitch * dt;
    pure_gyro_pitch += gyro_pitch * dt;

    accel_pitch = atan2f(-data->ax, data->az) * RAD_TO_DEG;
    pitch_angle = FILTER_ALPHA * pitch_angle + (1.0f - FILTER_ALPHA) * accel_pitch;

    attitude->pitch = pitch_angle;
    attitude->roll = 0.0f;
    attitude->yaw = 0.0f;
}

void MPU6050_Get_Attitude(Attitude_Data *attitude)
{
    attitude->pitch = pitch_angle;
    attitude->roll = 0.0f;
    attitude->yaw = 0.0f;
}

float MPU6050_Get_Gyro_Pitch(MPU6050_Data *data)
{
    return data->gy - pitch_bias;
}

float MPU6050_Get_Gyro_Only_Pitch(void)
{
    return pure_gyro_pitch;
}
