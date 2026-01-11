#ifndef MPU6050_H
#define MPU6050_H

#include "esp_err.h"
#include <stdint.h>

// MPU6050 I2C Address
#define MPU6050_ADDR                0x68

// MPU6050 Register Map
#define MPU6050_REG_WHO_AM_I        0x75
#define MPU6050_REG_PWR_MGMT_1      0x6B
#define MPU6050_REG_PWR_MGMT_2      0x6C
#define MPU6050_REG_SMPLRT_DIV      0x19
#define MPU6050_REG_CONFIG          0x1A
#define MPU6050_REG_GYRO_CONFIG     0x1B
#define MPU6050_REG_ACCEL_CONFIG    0x1C
#define MPU6050_REG_INT_ENABLE      0x38
#define MPU6050_REG_ACCEL_XOUT_H    0x3B
#define MPU6050_REG_ACCEL_XOUT_L    0x3C
#define MPU6050_REG_ACCEL_YOUT_H    0x3D
#define MPU6050_REG_ACCEL_YOUT_L    0x3E
#define MPU6050_REG_ACCEL_ZOUT_H    0x3F
#define MPU6050_REG_ACCEL_ZOUT_L    0x40
#define MPU6050_REG_TEMP_OUT_H      0x41
#define MPU6050_REG_TEMP_OUT_L      0x42
#define MPU6050_REG_GYRO_XOUT_H     0x43
#define MPU6050_REG_GYRO_XOUT_L     0x44
#define MPU6050_REG_GYRO_YOUT_H     0x45
#define MPU6050_REG_GYRO_YOUT_L     0x46
#define MPU6050_REG_GYRO_ZOUT_H     0x47
#define MPU6050_REG_GYRO_ZOUT_L     0x48

// WHO_AM_I value
#define MPU6050_WHO_AM_I_VAL        0x68

// Power Management bits
#define MPU6050_PWR1_DEVICE_RESET   0x80
#define MPU6050_PWR1_SLEEP          0x40
#define MPU6050_PWR1_CYCLE          0x20
#define MPU6050_PWR1_TEMP_DIS       0x08
#define MPU6050_PWR1_CLKSEL_MASK    0x07

// Clock Source
#define MPU6050_CLOCK_INTERNAL      0x00
#define MPU6050_CLOCK_PLL_XGYRO     0x01
#define MPU6050_CLOCK_PLL_YGYRO     0x02
#define MPU6050_CLOCK_PLL_ZGYRO     0x03

// Accelerometer Full Scale Range
#define MPU6050_ACCEL_FS_2G         0x00
#define MPU6050_ACCEL_FS_4G         0x08
#define MPU6050_ACCEL_FS_8G         0x10
#define MPU6050_ACCEL_FS_16G        0x18

// Gyroscope Full Scale Range
#define MPU6050_GYRO_FS_250         0x00
#define MPU6050_GYRO_FS_500         0x08
#define MPU6050_GYRO_FS_1000        0x10
#define MPU6050_GYRO_FS_2000        0x18

// Data structures
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} mpu6050_accel_t;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} mpu6050_gyro_t;

typedef struct {
    int16_t temp_raw;
    float temp_celsius;
} mpu6050_temp_t;

// Function prototypes
esp_err_t mpu6050_init(void);
esp_err_t mpu6050_who_am_i(uint8_t *who_am_i);
esp_err_t mpu6050_read_accel(mpu6050_accel_t *accel);
esp_err_t mpu6050_read_gyro(mpu6050_gyro_t *gyro);
esp_err_t mpu6050_read_temp(mpu6050_temp_t *temp);
esp_err_t mpu6050_read_all(mpu6050_accel_t *accel, mpu6050_gyro_t *gyro, mpu6050_temp_t *temp);

#endif // MPU6050_H