#include "mpu6050.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_MASTER_NUM          I2C_NUM_0
#define I2C_MASTER_TIMEOUT_MS   1000

static const char *TAG = "MPU6050";

/**
 * @brief Write a byte to MPU6050 register
 */
static esp_err_t mpu6050_write_byte(uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};
    
    return i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_ADDR, 
                                      write_buf, sizeof(write_buf),
                                      I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/**
 * @brief Read bytes from MPU6050 register
 */
static esp_err_t mpu6050_read_bytes(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, MPU6050_ADDR,
                                        &reg_addr, 1, data, len,
                                        I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/**
 * @brief Read a single byte from MPU6050 register
 */
static esp_err_t mpu6050_read_byte(uint8_t reg_addr, uint8_t *data)
{
    return mpu6050_read_bytes(reg_addr, data, 1);
}

/**
 * @brief Initialize MPU6050 sensor
 */
esp_err_t mpu6050_init(void)
{
    esp_err_t ret;

    ESP_LOGI(TAG, "Initializing MPU6050...");

    // Reset device
    ret = mpu6050_write_byte(MPU6050_REG_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset device");
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(100));  // Wait for reset to complete

    // Wake up device and set clock source to PLL with X-axis gyroscope
    ret = mpu6050_write_byte(MPU6050_REG_PWR_MGMT_1, MPU6050_CLOCK_PLL_XGYRO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wake up device");
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    // Disable sleep mode in PWR_MGMT_2
    ret = mpu6050_write_byte(MPU6050_REG_PWR_MGMT_2, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure PWR_MGMT_2");
        return ret;
    }

    // Set sample rate divider to 0 (1kHz sample rate)
    ret = mpu6050_write_byte(MPU6050_REG_SMPLRT_DIV, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set sample rate");
        return ret;
    }

    // Configure DLPF (Digital Low Pass Filter) - bandwidth 94Hz
    ret = mpu6050_write_byte(MPU6050_REG_CONFIG, 0x02);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure DLPF");
        return ret;
    }

    // Set accelerometer full scale range to ±2g
    ret = mpu6050_write_byte(MPU6050_REG_ACCEL_CONFIG, MPU6050_ACCEL_FS_2G);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure accelerometer");
        return ret;
    }

    // Set gyroscope full scale range to ±250°/s
    ret = mpu6050_write_byte(MPU6050_REG_GYRO_CONFIG, MPU6050_GYRO_FS_250);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure gyroscope");
        return ret;
    }

    ESP_LOGI(TAG, "MPU6050 initialization complete");
    return ESP_OK;
}

/**
 * @brief Read WHO_AM_I register
 */
esp_err_t mpu6050_who_am_i(uint8_t *who_am_i)
{
    return mpu6050_read_byte(MPU6050_REG_WHO_AM_I, who_am_i);
}

/**
 * @brief Read accelerometer data using burst read
 */
esp_err_t mpu6050_read_accel(mpu6050_accel_t *accel)
{
    uint8_t data[6];
    esp_err_t ret;

    // Burst read 6 bytes starting from ACCEL_XOUT_H
    ret = mpu6050_read_bytes(MPU6050_REG_ACCEL_XOUT_H, data, 6);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read accelerometer data");
        return ret;
    }

    // Combine high and low bytes
    accel->x = (int16_t)((data[0] << 8) | data[1]);
    accel->y = (int16_t)((data[2] << 8) | data[3]);
    accel->z = (int16_t)((data[4] << 8) | data[5]);

    return ESP_OK;
}

/**
 * @brief Read gyroscope data using burst read
 */
esp_err_t mpu6050_read_gyro(mpu6050_gyro_t *gyro)
{
    uint8_t data[6];
    esp_err_t ret;

    // Burst read 6 bytes starting from GYRO_XOUT_H
    ret = mpu6050_read_bytes(MPU6050_REG_GYRO_XOUT_H, data, 6);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read gyroscope data");
        return ret;
    }

    // Combine high and low bytes
    gyro->x = (int16_t)((data[0] << 8) | data[1]);
    gyro->y = (int16_t)((data[2] << 8) | data[3]);
    gyro->z = (int16_t)((data[4] << 8) | data[5]);

    return ESP_OK;
}

/**
 * @brief Read temperature data
 */
esp_err_t mpu6050_read_temp(mpu6050_temp_t *temp)
{
    uint8_t data[2];
    esp_err_t ret;

    // Read 2 bytes starting from TEMP_OUT_H
    ret = mpu6050_read_bytes(MPU6050_REG_TEMP_OUT_H, data, 2);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read temperature data");
        return ret;
    }

    // Combine high and low bytes
    temp->temp_raw = (int16_t)((data[0] << 8) | data[1]);
    
    // Convert to Celsius: Temp_degC = (TEMP_OUT / 340) + 36.53
    temp->temp_celsius = (temp->temp_raw / 340.0f) + 36.53f;

    return ESP_OK;
}

/**
 * @brief Read all sensor data (accel + gyro + temp) using burst read
 */
esp_err_t mpu6050_read_all(mpu6050_accel_t *accel, mpu6050_gyro_t *gyro, mpu6050_temp_t *temp)
{
    uint8_t data[14];
    esp_err_t ret;

    // Burst read 14 bytes starting from ACCEL_XOUT_H
    // Order: ACCEL_X(2), ACCEL_Y(2), ACCEL_Z(2), TEMP(2), GYRO_X(2), GYRO_Y(2), GYRO_Z(2)
    ret = mpu6050_read_bytes(MPU6050_REG_ACCEL_XOUT_H, data, 14);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read all sensor data");
        return ret;
    }

    // Parse accelerometer data
    accel->x = (int16_t)((data[0] << 8) | data[1]);
    accel->y = (int16_t)((data[2] << 8) | data[3]);
    accel->z = (int16_t)((data[4] << 8) | data[5]);

    // Parse temperature data
    temp->temp_raw = (int16_t)((data[6] << 8) | data[7]);
    temp->temp_celsius = (temp->temp_raw / 340.0f) + 36.53f;

    // Parse gyroscope data
    gyro->x = (int16_t)((data[8] << 8) | data[9]);
    gyro->y = (int16_t)((data[10] << 8) | data[11]);
    gyro->z = (int16_t)((data[12] << 8) | data[13]);

    return ESP_OK;
}