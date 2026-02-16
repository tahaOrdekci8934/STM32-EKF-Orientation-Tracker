#ifndef MPU6050_DRIVER_H
#define MPU6050_DRIVER_H

#include "stm32f1xx.h"

/**
 * @brief Initialize MPU6050 to wake up from sleep mode.
 * @param I2Cx I2C peripheral (e.g., I2C1)
 * @return 0 if successful
 */
uint8_t MPU6050_Init(I2C_TypeDef *I2Cx);

/**
 * @brief Read raw Accelerometer and Gyroscope data.
 * @param I2Cx I2C peripheral
 * @param acc Array to store [ax, ay, az]
 * @param gyro Array to store [gx, gy, gz]
 */
void MPU6050_Read_Raw_Data(I2C_TypeDef *I2Cx, float *ax, float *ay, float *az, float *gx, float *gy, float *gz);

#endif
