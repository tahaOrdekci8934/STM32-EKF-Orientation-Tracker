#include <mpu6050_driver.h>
#include <stdint.h>
#define I2C_TIMEOUT_MAX 0x2000

/**
 * @brief  Initializes MPU6050 and wakes it up from sleep mode.
 * @param  I2Cx: I2C instance (e.g. I2C1)
 * @retval 0 if success, 1 if error
 */
uint8_t MPU6050_Init(I2C_TypeDef *I2Cx)
{
    uint32_t timeout;

    /* Generate START condition */
    I2Cx->CR1 |= (1 << 8);
    timeout = I2C_TIMEOUT_MAX;
    while (!(I2Cx->SR1 & (1 << 0)) && --timeout);

    /* Send device address with write bit */
    I2Cx->DR = 0xD0;
    timeout = I2C_TIMEOUT_MAX;
    while (!(I2Cx->SR1 & (1 << 1)) && --timeout);

    /* Clear ADDR flag by reading SR1 and SR2 */
    (void)I2Cx->SR1;
    (void)I2Cx->SR2;

    /* Send PWR_MGMT_1 register address */
    I2Cx->DR = 0x6B;
    timeout = I2C_TIMEOUT_MAX;
    while (!(I2Cx->SR1 & (1 << 7)) && --timeout);

    /* Write 0x00 to wake up the MPU6050 */
    I2Cx->DR = 0x00;
    timeout = I2C_TIMEOUT_MAX;
    while (!(I2Cx->SR1 & (1 << 7)) && --timeout);

    /* Generate STOP condition */
    I2Cx->CR1 |= (1 << 9);

    if (timeout == 0) return 1; // Error
        return 0; // Success
}

/**
 * @brief Performs a software reset of the I2C peripheral.
 * @param I2Cx: I2C instance
 */
static void MPU6050_I2C_HardwareReset(I2C_TypeDef *I2Cx)
{
    /* Software reset of the I2C peripheral */
    I2Cx->CR1 |=  (1 << 15);
    for (volatile int i = 0; i < 100; i++);
    I2Cx->CR1 &= ~(1 << 15);

    /* Disable and re-enable peripheral to reset internal state machines */
    I2Cx->CR1 &= ~(1 << 0);
    for (volatile int i = 0; i < 100; i++);
    I2Cx->CR1 |=  (1 << 0);
}

/**
 * @brief Reads raw accelerometer and gyroscope data from MPU6050.
 *        Resets I2C if a timeout occurs.
 */
void MPU6050_Read_Raw_Data(I2C_TypeDef *I2Cx,
                           float *ax, float *ay, float *az,
                           float *gx, float *gy, float *gz)
{
    uint8_t  buffer[14];
    uint32_t timeout;

    /* Generate START condition */
    I2Cx->CR1 |= (1 << 8);
    timeout = I2C_TIMEOUT_MAX;
    while (!(I2Cx->SR1 & (1 << 0)) && --timeout);
    if (timeout == 0) { MPU6050_I2C_HardwareReset(I2Cx); return; }

    /* Send device address with write bit */
    I2Cx->DR = 0xD0;
    timeout = I2C_TIMEOUT_MAX;
    while (!(I2Cx->SR1 & (1 << 1)) && --timeout);
    if (timeout == 0) { MPU6050_I2C_HardwareReset(I2Cx); return; }

    /* Clear ADDR flag */
    (void)I2Cx->SR1;
    (void)I2Cx->SR2;

    /* Send start register address (ACCEL_XOUT_H) */
    I2Cx->DR = 0x3B;
    timeout = I2C_TIMEOUT_MAX;
    while (!(I2Cx->SR1 & (1 << 7)) && --timeout);
    if (timeout == 0) return;

    /* Generate repeated START condition */
    I2Cx->CR1 |= (1 << 8);
    timeout = I2C_TIMEOUT_MAX;
    while (!(I2Cx->SR1 & (1 << 0)) && --timeout);
    if (timeout == 0) return;

    /* Send device address with read bit */
    I2Cx->DR = 0xD1;
    timeout = I2C_TIMEOUT_MAX;
    while (!(I2Cx->SR1 & (1 << 1)) && --timeout);
    if (timeout == 0) return;

    /* Clear ADDR flag */
    (void)I2Cx->SR1;
    (void)I2Cx->SR2;

    /* Read 14 consecutive data bytes */
    for (int i = 0; i < 14; i++) {

        if (i == 13) {
            /* Last byte: send NACK and STOP */
            I2Cx->CR1 &= ~(1 << 10);
            I2Cx->CR1 |=  (1 << 9);
        } else {
            /* More bytes expected: keep ACK enabled */
            I2Cx->CR1 |= (1 << 10);
        }

        /* Wait until RXNE flag is set */
        timeout = I2C_TIMEOUT_MAX;
        while (!(I2Cx->SR1 & (1 << 6)) && --timeout);

        if (timeout == 0)
            break;

        /* Read received byte */
        buffer[i] = I2Cx->DR;
    }

    /* Combine high and low bytes */
    int16_t raw_ax = (int16_t)((buffer[0]  << 8) | buffer[1]);
    int16_t raw_ay = (int16_t)((buffer[2]  << 8) | buffer[3]);
    int16_t raw_az = (int16_t)((buffer[4]  << 8) | buffer[5]);
    int16_t raw_gx = (int16_t)((buffer[8]  << 8) | buffer[9]);
    int16_t raw_gy = (int16_t)((buffer[10] << 8) | buffer[11]);
    int16_t raw_gz = (int16_t)((buffer[12] << 8) | buffer[13]);

    /* Convert raw values to physical units */
    *ax = raw_ax / 16384.0f;
    *ay = raw_ay / 16384.0f;
    *az = raw_az / 16384.0f;
    *gx = raw_gx / 131.0f;
    *gy = raw_gy / 131.0f;
    *gz = raw_gz / 131.0f;
}
