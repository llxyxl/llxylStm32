#ifndef __MPU6050_H
#define __MPU6050_H
#include <stdint.h>
#define MPU6050_ADDR    0x68

// ¼Ä´æÆ÷µØÖ·
#define MPU6050_PWR_MGMT_1     0x6B
#define MPU6050_ACCEL_XOUT_H   0x3B
#define MPU6050_TEMP_OUT_H     0x41
#define MPU6050_GYRO_XOUT_H    0x43
#define MPU6050_WHO_AM_I       0x75

typedef struct {
    float accel_x, accel_y, accel_z;
    float gyro_x,  gyro_y,  gyro_z;
    float temp;
    float pitch, roll, yaw;
} MPU6050_Data;

extern float gyro_bias[3];
void MPU6050_CalibrateGyro(void);

uint8_t MPU6050_Init(void);
void MPU6050_ReadAll(MPU6050_Data *data);

#endif
