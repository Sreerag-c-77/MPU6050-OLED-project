#include "ssd1306.h"
#include "font5x7.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdlib.h>

#define I2C_MASTER_NUM          I2C_NUM_0
#define I2C_MASTER_TIMEOUT_MS   1000

static const char *TAG = "SSD1306";

// Frame buffer
static uint8_t framebuffer[SSD1306_WIDTH * SSD1306_PAGES];

static esp_err_t ssd1306_write_cmd(uint8_t cmd)
{
    uint8_t data[2] = {SSD1306_CONTROL_CMD_SINGLE, cmd};
    return i2c_master_write_to_device(I2C_MASTER_NUM, SSD1306_ADDR,
                                      data, sizeof(data),
                                      I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t ssd1306_write_cmds(const uint8_t *cmds, size_t len)
{
    esp_err_t ret;
    for (size_t i = 0; i < len; i++) {
        ret = ssd1306_write_cmd(cmds[i]);
        if (ret != ESP_OK) return ret;
    }
    return ESP_OK;
}

static esp_err_t ssd1306_write_data(const uint8_t *data, size_t len)
{
    uint8_t *buf = malloc(len + 1);
    if (!buf) {
        ESP_LOGE(TAG, "Failed to allocate memory");
        return ESP_ERR_NO_MEM;
    }
    buf[0] = SSD1306_CONTROL_DATA_STREAM;
    memcpy(buf + 1, data, len);
    esp_err_t ret = i2c_master_write_to_device(I2C_MASTER_NUM, SSD1306_ADDR,
                                               buf, len + 1,
                                               I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    free(buf);
    return ret;
}

esp_err_t ssd1306_init(void)
{
    ESP_LOGI(TAG, "Initializing SSD1306...");
    
    const uint8_t init_cmds[] = {
        SSD1306_CMD_DISPLAY_OFF,
        SSD1306_CMD_SET_DISPLAY_CLOCK, 0x80,
        SSD1306_CMD_SET_MULTIPLEX_RATIO, 0x3F,
        SSD1306_CMD_SET_DISPLAY_OFFSET, 0x00,
        SSD1306_CMD_SET_START_LINE | 0x00,
        SSD1306_CMD_CHARGE_PUMP, 0x14,
        SSD1306_CMD_SET_MEMORY_MODE, SSD1306_MEMORY_MODE_HORIZONTAL,
        SSD1306_CMD_SET_SEGMENT_REMAP | 0x01,
        SSD1306_CMD_SET_COM_SCAN_DEC,
        SSD1306_CMD_SET_COM_PINS, 0x12,
        SSD1306_CMD_SET_CONTRAST, 0xCF,
        SSD1306_CMD_SET_PRECHARGE_PERIOD, 0xF1,
        SSD1306_CMD_SET_VCOMH_DESELECT, 0x40,
        SSD1306_CMD_DISPLAY_ALL_ON_RESUME,
        SSD1306_CMD_NORMAL_DISPLAY,
        SSD1306_CMD_DISPLAY_ON
    };
    
    esp_err_t ret = ssd1306_write_cmds(init_cmds, sizeof(init_cmds));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SSD1306");
        return ret;
    }
    
    ssd1306_clear();
    ssd1306_update();
    
    ESP_LOGI(TAG, "SSD1306 initialization complete");
    return ESP_OK;
}

void ssd1306_clear(void)
{
    memset(framebuffer, 0, sizeof(framebuffer));
}

void ssd1306_update(void)
{
    ssd1306_write_cmd(SSD1306_CMD_SET_COLUMN_ADDR);
    ssd1306_write_cmd(SSD1306_COL_OFFSET);
    ssd1306_write_cmd(SSD1306_COL_OFFSET + SSD1306_WIDTH - 1);
    
    ssd1306_write_cmd(SSD1306_CMD_SET_PAGE_ADDR);
    ssd1306_write_cmd(0);
    ssd1306_write_cmd(SSD1306_PAGES - 1);
    
    const size_t chunk_size = 64;
    for (size_t i = 0; i < sizeof(framebuffer); i += chunk_size) {
        size_t remaining = sizeof(framebuffer) - i;
        size_t size = (remaining < chunk_size) ? remaining : chunk_size;
        ssd1306_write_data(&framebuffer[i], size);
    }
}

void ssd1306_draw_pixel(uint8_t x, uint8_t y, bool on)
{
    if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) return;
    
    uint16_t index = x + (y / 8) * SSD1306_WIDTH;
    uint8_t bit = y % 8;
    
    if (on) {
        framebuffer[index] |= (1 << bit);
    } else {
        framebuffer[index] &= ~(1 << bit);
    }
}

void ssd1306_draw_line(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, bool on)
{
    int16_t dx = abs(x1 - x0);
    int16_t dy = abs(y1 - y0);
    int16_t sx = (x0 < x1) ? 1 : -1;
    int16_t sy = (y0 < y1) ? 1 : -1;
    int16_t err = dx - dy;
    
    while (1) {
        ssd1306_draw_pixel(x0, y0, on);
        if (x0 == x1 && y0 == y1) break;
        
        int16_t e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }
    }
}

void ssd1306_draw_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, bool on)
{
    ssd1306_draw_line(x, y, x + w - 1, y, on);
    ssd1306_draw_line(x + w - 1, y, x + w - 1, y + h - 1, on);
    ssd1306_draw_line(x + w - 1, y + h - 1, x, y + h - 1, on);
    ssd1306_draw_line(x, y + h - 1, x, y, on);
}

void ssd1306_fill_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, bool on)
{
    for (uint8_t i = x; i < x + w && i < SSD1306_WIDTH; i++) {
        for (uint8_t j = y; j < y + h && j < SSD1306_HEIGHT; j++) {
            ssd1306_draw_pixel(i, j, on);
        }
    }
}

void ssd1306_draw_char(uint8_t x, uint8_t y, char c, bool on)
{
    if (c < 32 || c > 126) c = '?';
    uint8_t char_index = c - 32;
    
    for (uint8_t col = 0; col < FONT_WIDTH; col++) {
        uint8_t char_col = font5x7[char_index][col];
        for (uint8_t row = 0; row < FONT_HEIGHT; row++) {
            if (char_col & (1 << row)) {
                ssd1306_draw_pixel(x + col, y + row, on);
            }
        }
    }
}

void ssd1306_draw_string(uint8_t x, uint8_t y, const char *str, bool on)
{
    while (*str) {
        if (*str == '\n') {
            y += FONT_HEIGHT + 2;
            x = 0;
        } else {
            ssd1306_draw_char(x, y, *str, on);
            x += FONT_WIDTH + 1;
        }
        str++;
        
        if (x + FONT_WIDTH >= SSD1306_WIDTH) {
            x = 0;
            y += FONT_HEIGHT + 2;
        }
        if (y + FONT_HEIGHT >= SSD1306_HEIGHT) break;
    }
}

void ssd1306_set_contrast(uint8_t contrast)
{
    ssd1306_write_cmd(SSD1306_CMD_SET_CONTRAST);
    ssd1306_write_cmd(contrast);
}

void ssd1306_display_on(bool on)
{
    ssd1306_write_cmd(on ? SSD1306_CMD_DISPLAY_ON : SSD1306_CMD_DISPLAY_OFF);
}

void ssd1306_invert_display(bool invert)
{
    ssd1306_write_cmd(invert ? SSD1306_CMD_INVERT_DISPLAY : SSD1306_CMD_NORMAL_DISPLAY);
}