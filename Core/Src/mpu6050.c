#include "mpu6050.h"
#include "main.h"
#include <math.h>
#include <stdio.h>

extern I2C_HandleTypeDef hi2c1;

void mpu6050_init()
{
  uint8_t set_gyro_range = GYRO_RANGE_500;
  uint8_t set_accel_range = ACCEL_RANGE_4G;
  uint8_t set_sleep_mode = SLEEP_MODE_OFF;
  uint8_t dlpf_cfg = 0x03; // DLPF = 42Hz
  HAL_I2C_IsDeviceReady(&hi2c1, SENSOR_ADDR, 1, 100);
  HAL_I2C_Mem_Write(&hi2c1, SENSOR_ADDR, GYRO_CONFIG_REG, 1, &set_gyro_range, 1, 100);
  HAL_I2C_Mem_Write(&hi2c1, SENSOR_ADDR, ACCEL_CONFIG_REG, 1, &set_accel_range, 1, 100);
  HAL_I2C_Mem_Write(&hi2c1, SENSOR_ADDR, SENSOR_CONFIG_REG, 1, &dlpf_cfg, 1, 100);
  HAL_I2C_Mem_Write(&hi2c1, SENSOR_ADDR, SLEEP_MODE_REG, 1, &set_sleep_mode, 1, 100);
}

void mpu6050_read_gyro(gyro_data * gyro)
{
    HAL_I2C_Mem_Read(&hi2c1, SENSOR_ADDR, GYRO_XOUT_HIGH, 1, &gyro->x_data[0], 1, 100);
    HAL_I2C_Mem_Read(&hi2c1, SENSOR_ADDR, GYRO_XOUT_LOW, 1, &gyro->x_data[1], 1, 100);
    gyro->x_val_raw = ((int16_t)gyro->x_data[0] << 8) | gyro->x_data[1];
    gyro->x_val = ((float)gyro->x_val_raw)/GYRO_SENSITIVITY; // deg/s

    HAL_I2C_Mem_Read(&hi2c1, SENSOR_ADDR, GYRO_YOUT_HIGH, 1, &gyro->y_data[0], 1, 100);
    HAL_I2C_Mem_Read(&hi2c1, SENSOR_ADDR, GYRO_YOUT_LOW, 1, &gyro->y_data[1], 1, 100);
    gyro->y_val_raw = ((int16_t)gyro->y_data[0] << 8) | gyro->y_data[1];
    gyro->y_val = ((float)gyro->y_val_raw)/GYRO_SENSITIVITY;

    HAL_I2C_Mem_Read(&hi2c1, SENSOR_ADDR, GYRO_ZOUT_HIGH, 1, &gyro->z_data[0], 1, 100);
    HAL_I2C_Mem_Read(&hi2c1, SENSOR_ADDR, GYRO_ZOUT_LOW, 1, &gyro->z_data[1], 1, 100);
    gyro->z_val_raw = ((int16_t)gyro->z_data[0] << 8) | gyro->z_data[1];
    gyro->z_val = ((float)gyro->z_val_raw)/GYRO_SENSITIVITY;
}

void mpu6050_read_accel(accel_data * accel)
{
    HAL_I2C_Mem_Read(&hi2c1, SENSOR_ADDR, ACCEL_XOUT_HIGH, 1, &accel->x_data[0], 1, 100);
    HAL_I2C_Mem_Read(&hi2c1, SENSOR_ADDR, ACCEL_XOUT_LOW, 1, &accel->x_data[1], 1, 100);
    accel->x_val_raw = ((int16_t)accel->x_data[0] << 8) | accel->x_data[1];
    accel->x_val = ((float)accel->x_val_raw)/ACCEL_SENSITIVITY;

    HAL_I2C_Mem_Read(&hi2c1, SENSOR_ADDR, ACCEL_YOUT_HIGH, 1, &accel->y_data[0], 1, 100);
    HAL_I2C_Mem_Read(&hi2c1, SENSOR_ADDR, ACCEL_YOUT_LOW, 1, &accel->y_data[1], 1, 100);
    accel->y_val_raw = ((int16_t)accel->y_data[0] << 8) | accel->y_data[1];
    accel->y_val = ((float)accel->y_val_raw)/ACCEL_SENSITIVITY;

    HAL_I2C_Mem_Read(&hi2c1, SENSOR_ADDR, ACCEL_ZOUT_HIGH, 1, &accel->z_data[0], 1, 100);
    HAL_I2C_Mem_Read(&hi2c1, SENSOR_ADDR, ACCEL_ZOUT_LOW, 1, &accel->z_data[1], 1, 100);
    accel->z_val_raw = ((int16_t)accel->z_data[0] << 8) | accel->z_data[1];
    accel->z_val = ((float)accel->z_val_raw)/ACCEL_SENSITIVITY;

    accel->roll_angle  = atan2f(accel->y_val, accel->z_val) * RAD_TO_DEG;
	accel->pitch_angle = atan2f(-accel->x_val, accel->z_val) * RAD_TO_DEG;
}
