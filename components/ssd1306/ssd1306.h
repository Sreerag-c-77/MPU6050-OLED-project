#ifndef SSD1306_H
#define SSD1306_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

// SSD1306 I2C Address (can be 0x3C or 0x3D depending on SA0 pin)
#define SSD1306_ADDR            0x3C

// Display dimensions
#define SSD1306_WIDTH           128
#define SSD1306_HEIGHT          64
#define SSD1306_PAGES           (SSD1306_HEIGHT / 8)

// Column offset for some displays (usually 0 or 2)
#define SSD1306_COL_OFFSET      0

// Control byte flags
#define SSD1306_CONTROL_CMD_SINGLE   0x80
#define SSD1306_CONTROL_CMD_STREAM   0x00
#define SSD1306_CONTROL_DATA_STREAM  0x40

// Fundamental Commands
#define SSD1306_CMD_SET_CONTRAST            0x81
#define SSD1306_CMD_DISPLAY_ALL_ON_RESUME   0xA4
#define SSD1306_CMD_DISPLAY_ALL_ON          0xA5
#define SSD1306_CMD_NORMAL_DISPLAY          0xA6
#define SSD1306_CMD_INVERT_DISPLAY          0xA7
#define SSD1306_CMD_DISPLAY_OFF             0xAE
#define SSD1306_CMD_DISPLAY_ON              0xAF

// Addressing Setting Commands
#define SSD1306_CMD_SET_MEMORY_MODE         0x20
#define SSD1306_CMD_SET_COLUMN_ADDR         0x21
#define SSD1306_CMD_SET_PAGE_ADDR           0x22
#define SSD1306_CMD_SET_PAGE_START_ADDR     0xB0

// Hardware Configuration Commands
#define SSD1306_CMD_SET_START_LINE          0x40
#define SSD1306_CMD_SET_SEGMENT_REMAP       0xA0
#define SSD1306_CMD_SET_MULTIPLEX_RATIO     0xA8
#define SSD1306_CMD_SET_COM_SCAN_INC        0xC0
#define SSD1306_CMD_SET_COM_SCAN_DEC        0xC8
#define SSD1306_CMD_SET_DISPLAY_OFFSET      0xD3
#define SSD1306_CMD_SET_COM_PINS            0xDA

// Timing & Driving Scheme Commands
#define SSD1306_CMD_SET_DISPLAY_CLOCK       0xD5
#define SSD1306_CMD_SET_PRECHARGE_PERIOD    0xD9
#define SSD1306_CMD_SET_VCOMH_DESELECT      0xDB
#define SSD1306_CMD_NOP                     0xE3

// Charge Pump Commands
#define SSD1306_CMD_CHARGE_PUMP             0x8D

// Memory addressing modes
#define SSD1306_MEMORY_MODE_HORIZONTAL      0x00
#define SSD1306_MEMORY_MODE_VERTICAL        0x01
#define SSD1306_MEMORY_MODE_PAGE            0x02

// Function prototypes
esp_err_t ssd1306_init(void);
void ssd1306_clear(void);
void ssd1306_update(void);
void ssd1306_draw_pixel(uint8_t x, uint8_t y, bool on);
void ssd1306_draw_line(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, bool on);
void ssd1306_draw_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, bool on);
void ssd1306_fill_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, bool on);
void ssd1306_draw_char(uint8_t x, uint8_t y, char c, bool on);
void ssd1306_draw_string(uint8_t x, uint8_t y, const char *str, bool on);
void ssd1306_set_contrast(uint8_t contrast);
void ssd1306_display_on(bool on);
void ssd1306_invert_display(bool invert);

#endif // SSD1306_H