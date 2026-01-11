#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "mpu6050.h"
#include "ssd1306.h"

#define I2C_MASTER_SCL_IO           22      /*!< GPIO number for I2C master clock */
#define I2C_MASTER_SDA_IO           21      /*!< GPIO number for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0   /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ          100000      /*!< I2C master clock frequency */

static const char *TAG = "MPU6050_OLED";

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        return err;
    }

    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting MPU6050 + OLED Display Application");

    // Initialize I2C bus
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    // Initialize SSD1306 OLED Display
    if (ssd1306_init() != ESP_OK) {
        ESP_LOGE(TAG, "SSD1306 initialization failed!");
        return;
    }
    ESP_LOGI(TAG, "SSD1306 initialized successfully");

    // Clear display and show startup message
    ssd1306_clear();
    ssd1306_draw_string(0, 0, "MPU6050 Ready", 1);
    ssd1306_update();
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Initialize MPU6050
    if (mpu6050_init() != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 initialization failed!");
        ssd1306_clear();
        ssd1306_draw_string(0, 0, "MPU6050 FAIL!", 1);
        ssd1306_update();
        return;
    }
    ESP_LOGI(TAG, "MPU6050 initialized successfully");

    // Verify MPU6050 connection
    uint8_t who_am_i = 0;
    if (mpu6050_who_am_i(&who_am_i) != ESP_OK || who_am_i != MPU6050_WHO_AM_I_VAL) {
        ESP_LOGE(TAG, "MPU6050 WHO_AM_I check failed! Got: 0x%02X", who_am_i);
        ssd1306_clear();
        ssd1306_draw_string(0, 0, "WHO_AM_I FAIL!", 1);
        ssd1306_update();
        return;
    }
    ESP_LOGI(TAG, "MPU6050 WHO_AM_I: 0x%02X - OK", who_am_i);

    mpu6050_accel_t accel_data;
    char line_buffer[32];

    while (1) {
        // Read accelerometer data
        if (mpu6050_read_accel(&accel_data) == ESP_OK) {
            ESP_LOGI(TAG, "Accel: X=%d, Y=%d, Z=%d", 
                     accel_data.x, accel_data.y, accel_data.z);

            // Clear display
            ssd1306_clear();

            // Display title
            ssd1306_draw_string(10, 0, "Accelerometer", 1);
            ssd1306_draw_line(0, 10, 127, 10, 1);

            // Display X axis
            snprintf(line_buffer, sizeof(line_buffer), "X: %6d", accel_data.x);
            ssd1306_draw_string(0, 16, line_buffer, 1);

            // Display Y axis
            snprintf(line_buffer, sizeof(line_buffer), "Y: %6d", accel_data.y);
            ssd1306_draw_string(0, 28, line_buffer, 1);

            // Display Z axis
            snprintf(line_buffer, sizeof(line_buffer), "Z: %6d", accel_data.z);
            ssd1306_draw_string(0, 40, line_buffer, 1);

            // Calculate and display tilt angle (bonus feature)
            float angle_x = atan2(accel_data.y, accel_data.z) * 180.0 / 3.14159;
            float angle_y = atan2(accel_data.x, accel_data.z) * 180.0 / 3.14159;
            
            snprintf(line_buffer, sizeof(line_buffer), "Tilt: %.1f/%.1f", angle_x, angle_y);
            ssd1306_draw_string(0, 52, line_buffer, 1);

            // Update display
            ssd1306_update();
        } else {
            ESP_LOGE(TAG, "Failed to read MPU6050 data");
        }

        vTaskDelay(pdMS_TO_TICKS(100));  // Update every 100ms
    }
}