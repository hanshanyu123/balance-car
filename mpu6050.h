#ifndef __MPU6050_H
#define __MPU6050_H

#include "main.h"

#define MPU6050_ADDR            0x68
#define MPU6050_OK              0U
#define MPU6050_FAIL            1U

#define MPU6050_PWR_MGMT_1      0x6B
#define MPU6050_PWR_MGMT_2      0x6C
#define MPU6050_WHO_AM_I        0x75
#define MPU6050_SMPLRT_DIV      0x19
#define MPU6050_CONFIG          0x1A
#define MPU6050_GYRO_CONFIG     0x1B
#define MPU6050_ACCEL_CONFIG    0x1C

#define MPU6050_ACCEL_XOUT_H    0x3B
#define MPU6050_ACCEL_XOUT_L    0x3C
#define MPU6050_ACCEL_YOUT_H    0x3D
#define MPU6050_ACCEL_YOUT_L    0x3E
#define MPU6050_ACCEL_ZOUT_H    0x3F
#define MPU6050_ACCEL_ZOUT_L    0x40

#define MPU6050_TEMP_OUT_H      0x41
#define MPU6050_TEMP_OUT_L      0x42

#define MPU6050_GYRO_XOUT_H     0x43
#define MPU6050_GYRO_XOUT_L     0x44
#define MPU6050_GYRO_YOUT_H     0x45
#define MPU6050_GYRO_YOUT_L     0x46
#define MPU6050_GYRO_ZOUT_H     0x47
#define MPU6050_GYRO_ZOUT_L     0x48

#define MPU6050_PWR_MGMT_1_VALUE             0x01
#define MPU6050_SMPLRT_DIV_VALUE             0x07
#define MPU6050_CONFIG_VALUE                 0x00
#define MPU6050_ACCEL_CONFIG_VALUE           0x18
#define MPU6050_GYRO_CONFIG_VALUE            0x18
#define MPU6050_PWR_MGMT_2_VALUE             0x00

#define MPU6050_ACCEL_SENSITIVITY_LSB_PER_G  2048.0f
#define MPU6050_GYRO_SENSITIVITY_LSB_PER_DPS 16.4f
#define MPU6050_ACCEL_FULL_SCALE_G           16.0f
#define MPU6050_GYRO_FULL_SCALE_DPS          2000.0f
#define MPU6050_GYRO_VALID_LIMIT_DPS         MPU6050_GYRO_FULL_SCALE_DPS

typedef struct {
    int16_t ax_raw;
    int16_t ay_raw;
    int16_t az_raw;
    int16_t gx_raw;
    int16_t gy_raw;
    int16_t gz_raw;
    int16_t temp_raw;
} MPU6050_RawData;

typedef struct {
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
    float temp;
} MPU6050_Data;

typedef struct {
    float pitch;
    float roll;
    float yaw;
} Attitude_Data;

uint8_t MPU6050_Init(void);
uint8_t MPU6050_Read_RawData(MPU6050_RawData *raw);
uint8_t MPU6050_Read_Data(MPU6050_Data *data);
void MPU6050_Calculate_Attitude(MPU6050_Data *data, Attitude_Data *attitude, float dt);
void MPU6050_Get_Attitude(Attitude_Data *attitude);
float MPU6050_Get_Accel_Pitch(MPU6050_Data *data);
float MPU6050_Get_Gyro_Pitch(MPU6050_Data *data);
float MPU6050_Get_Gyro_Only_Pitch(void);

#endif
