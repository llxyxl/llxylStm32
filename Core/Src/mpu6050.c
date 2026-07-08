#include "mpu6050.h"
#include "main.h"  // 包含HAL库和hi2c1句柄
#include <math.h>

extern I2C_HandleTypeDef hi2c1;

// MPU6050寄存器地址
#define MPU6050_SMPLRT_DIV     0x19		//采样器分频寄存器
#define MPU6050_CONFIG         0x1A		//配置寄存器
#define MPU6050_GYRO_CONFIG    0x1B		//陀螺仪配置寄存器
#define MPU6050_ACCEL_CONFIG   0x1C		//加速度计配置寄存器

// 灵敏度转换系数
#define ACCEL_SENSITIVITY      16384.0f  // ±2g: 16384 LSB/g
#define GYRO_SENSITIVITY       131.0f    // ±250°/s: 131 LSB/(°/s)
#define TEMP_OFFSET            36.53f    // 温度偏移
#define TEMP_SENSITIVITY       340.0f    // 温度灵敏度 340 LSB/°C

// 互补滤波系数
#define ALPHA                  0.98f     // 陀螺仪权重

static uint32_t last_tick = 0;

// 静态变量用于陀螺仪积分
static float gyro_angle_x = 0.0f;
static float gyro_angle_y = 0.0f;
static float gyro_angle_z = 0.0f;

float gyro_bias[3] = {0.0f, 0.0f, 0.0f};
static uint8_t gyro_calibrated = 0;

/**
  * @brief  MPU6050初始化
  * @retval 0: 初始化成功, 1: 初始化失败
  */
uint8_t MPU6050_Init(void)
{
    uint8_t who_am_i = 0;
    
    // 复位设备
    uint8_t reset_cmd[2] = {MPU6050_PWR_MGMT_1, 0x80};
    HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR << 1, reset_cmd, 2, 100);
    
    // 延时 100ms
    HAL_Delay(100);
    
    // 唤醒
    uint8_t wake_cmd[2] = {MPU6050_PWR_MGMT_1, 0x00};
    HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR << 1, wake_cmd, 2, 100);
    
    // 设置采样率1.6kHZ
    uint8_t rate_cmd[2] = {MPU6050_SMPLRT_DIV, 0x04};
    HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR << 1, rate_cmd, 2, 100);
    
    // DLPF 42Hz
    uint8_t dlpf_cmd[2] = {MPU6050_CONFIG, 0x03};
    HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR << 1, dlpf_cmd, 2, 100);
    
    // 加速度计量程 ±2g
    uint8_t accel_cmd[2] = {MPU6050_ACCEL_CONFIG, 0x00};
    HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR << 1, accel_cmd, 2, 100);
    
    // 陀螺仪量程 ±250°/s
    uint8_t gyro_cmd[2] = {MPU6050_GYRO_CONFIG, 0x00};
    HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR << 1, gyro_cmd, 2, 100);
    
    // 读 WHO_AM_I
    // 先发送寄存器地址
    uint8_t reg_addr = MPU6050_WHO_AM_I;
    HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR << 1, &reg_addr, 1, 100);
    // 再读取WHO_AM_I值
    HAL_I2C_Master_Receive(&hi2c1, MPU6050_ADDR << 1, &who_am_i, 1, 100);
    
    if (who_am_i != 0x68)
    {
        return 1;  // 初始化失败，设备ID不匹配
    }
    
    return 0;  // 初始化成功
}

/**
  * @brief  校准陀螺仪零偏（必须保证传感器完全静止）
  */
void MPU6050_CalibrateGyro(void)
{
    int32_t sum[3] = {0, 0, 0};
    uint8_t buf[6];
    uint8_t reg_addr = MPU6050_GYRO_XOUT_H;
    
    // 采集 200 个样本
    for (int i = 0; i < 200; i++) 
		{
        HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR << 1, &reg_addr, 1, 100);
        HAL_I2C_Master_Receive(&hi2c1, MPU6050_ADDR << 1, buf, 6, 100);
        
        sum[0] += (int16_t)((buf[0] << 8) | buf[1]);
        sum[1] += (int16_t)((buf[2] << 8) | buf[3]);
        sum[2] += (int16_t)((buf[4] << 8) | buf[5]);
        
        HAL_Delay(5);
    }
    
    // 计算平均值并转换为 °/s
    gyro_bias[0] = (float)sum[0] / 200.0f / GYRO_SENSITIVITY;
    gyro_bias[1] = (float)sum[1] / 200.0f / GYRO_SENSITIVITY;
    gyro_bias[2] = (float)sum[2] / 200.0f / GYRO_SENSITIVITY;
    
    gyro_calibrated = 1;
    
    // 将积分角度归零
    gyro_angle_x = 0.0f;
    gyro_angle_y = 0.0f;
    gyro_angle_z = 0.0f;
}


void MPU6050_ReadAll(MPU6050_Data *data)
{
		uint32_t now = HAL_GetTick();
		float DT = (now - last_tick) / 1000.0f;
		last_tick = now;

		// 限制 DT 范围，防止异常
		if (DT < 0.001f) DT = 0.001f;
		if (DT > 0.05f)  DT = 0.005f;
	
    uint8_t buf[14];
    int16_t raw_accel_x, raw_accel_y, raw_accel_z;
    int16_t raw_temp;
    int16_t raw_gyro_x, raw_gyro_y, raw_gyro_z;
    
    uint8_t reg_addr = MPU6050_ACCEL_XOUT_H;
    HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR << 1, &reg_addr, 1, 100);
    HAL_I2C_Master_Receive(&hi2c1, MPU6050_ADDR << 1, buf, 14, 100);
    
    raw_accel_x = (int16_t)((buf[0] << 8) | buf[1]);
    raw_accel_y = (int16_t)((buf[2] << 8) | buf[3]);
    raw_accel_z = (int16_t)((buf[4] << 8) | buf[5]);
    
    raw_temp = (int16_t)((buf[6] << 8) | buf[7]);
    raw_gyro_x = (int16_t)((buf[8] << 8) | buf[9]);
    raw_gyro_y = (int16_t)((buf[10] << 8) | buf[11]);
    raw_gyro_z = (int16_t)((buf[12] << 8) | buf[13]);
    
    // 加速度（单位：g）
    data->accel_x = raw_accel_x / ACCEL_SENSITIVITY;
    data->accel_y = raw_accel_y / ACCEL_SENSITIVITY;
    data->accel_z = raw_accel_z / ACCEL_SENSITIVITY;
    
    // 温度（单位：°C）
    data->temp = raw_temp / TEMP_SENSITIVITY + TEMP_OFFSET;
    
    // 陀螺仪（单位：°/s），减去零偏
    data->gyro_x = raw_gyro_x / GYRO_SENSITIVITY - gyro_bias[0];
    data->gyro_y = raw_gyro_y / GYRO_SENSITIVITY - gyro_bias[1];
    data->gyro_z = raw_gyro_z / GYRO_SENSITIVITY - gyro_bias[2];
    
    // 姿态角计算
    
    // 加速度计计算角度（静态参考）
    float accel_angle_roll  = atan2f(data->accel_y, data->accel_z) * 57.2958f;
    float accel_angle_pitch = atan2f(-data->accel_x, 
			sqrtf(data->accel_y * data->accel_y + data->accel_z * data->accel_z)) * 57.2958f;
    
    // 陀螺仪积分
    gyro_angle_x += data->gyro_x * DT;
    gyro_angle_y += data->gyro_y * DT;
    gyro_angle_z += data->gyro_z * DT;
    
    // 角度复位保护：如果陀螺仪积分与加速度角度偏差超过 30°，强制同步
    if (fabsf(gyro_angle_x - accel_angle_roll) > 30.0f) 
		{
        gyro_angle_x = accel_angle_roll;
    }
    if (fabsf(gyro_angle_y - accel_angle_pitch) > 30.0f) 
		{
        gyro_angle_y = accel_angle_pitch;
    }
    
    // 互补滤波融合
    data->roll  = ALPHA * gyro_angle_x + (1.0f - ALPHA) * accel_angle_roll;
    data->pitch = ALPHA * gyro_angle_y + (1.0f - ALPHA) * accel_angle_pitch;
    data->yaw   = gyro_angle_z;  
}
