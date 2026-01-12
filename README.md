# MPU6050 + SSD1306 OLED Display Project

Complete register-level driver implementation for ESP32 with MPU6050 accelerometer and SSD1306 OLED display.

---

## 📋 Table of Contents

- [Hardware Requirements](#hardware-requirements)
- [Pin Configuration](#pin-configuration)
- [Software Setup](#software-setup)
- [Project Structure](#project-structure)
- [Building and Flashing](#building-and-flashing)
- [Expected Output](#expected-output)
- [Troubleshooting](#troubleshooting)
- [Technical Details](#technical-details)
- [Features Implemented](#features-implemented)

---

## 🔧 Hardware Requirements

| Component | Specification | Quantity |
|-----------|--------------|----------|
| ESP32 | Any variant (ESP32-WROOM, DevKit, etc.) | 1 |
| MPU6050 | 6-axis Accelerometer + Gyroscope | 1 |
| SSD1306 | 128x64 OLED Display (I2C) | 1 |
| Jumper Wires | Male-to-Female or Male-to-Male | 8 |
| Breadboard | Optional | 1 |
| USB Cable | Micro-USB or USB-C (depending on ESP32) | 1 |

---

## 🔌 Pin Configuration

### Wiring Diagram


┌──────────┐
│  ESP32   │
├──────────┤                 ┌─────────────┐         ┌─────────────┐
│ GPIO 22  │────────────────→│ SCL         │         │ SCL         │
│          │                 │   MPU6050   │         │  SSD1306    │
│ GPIO 21  │────────────────→│ SDA         │         │  OLED       │
│          │                 │             │         │             │
│   3.3V   │────────┬───────→│ VCC         │         │ VCC         │
│          │        │        └─────────────┘         └─────────────┘
│   GND    │────────┴────────→ GND                    GND
└──────────┘                    │                       │
                                └───────────────────────┘


### Pin Mapping Table

| ESP32 Pin | Function | MPU6050 Pin | SSD1306 Pin |
|-----------|----------|-------------|-------------|
| GPIO 22   | I2C SCL  | SCL         | SCL         |
| GPIO 21   | I2C SDA  | SDA         | SDA         |
| 3.3V      | Power    | VCC         | VCC         |
| GND       | Ground   | GND         | GND         |

### I2C Addresses

- *MPU6050*: 0x68 (AD0 pin connected to GND)
- *SSD1306*: 0x3C (default) or 0x3D (if SA0 pin is high)

---

## 💻 Software Setup

### Prerequisites

1. *Espressif-IDE* installed (Eclipse-based IDE)
2. *ESP-IDF 4.4 or later* (usually comes with Espressif-IDE)
3. *USB drivers* for your ESP32 board:
   - CH340 driver (for most Chinese boards)
   - CP210x driver (for official boards)
   - FTDI driver (for some development boards)

### Installing USB Drivers

#### Check if Driver is Needed:
1. Connect ESP32 to computer
2. Open *Device Manager* (Win+X → Device Manager)
3. Check *"Ports (COM & LPT)"* section
4. If you see your ESP32 listed with a COM port, drivers are installed
5. If you see "Unknown Device" or yellow warning, install drivers:

*CH340 Driver*: https://www.wch.cn/downloads/CH341SER_EXE.html
*CP210x Driver*: https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers

---

## 📁 Project Structure

Create this exact folder structure in your Espressif-IDE workspace:


mpu6050oled/
├── CMakeLists.txt                    # Root build configuration
├── README.md                         # This file
├── main/
│   ├── CMakeLists.txt               # Main component configuration
│   └── app_main.c                   # Main application code
└── components/
    ├── mpu6050/
    │   ├── CMakeLists.txt           # MPU6050 component configuration
    │   ├── mpu6050.h                # MPU6050 header file
    │   └── mpu6050.c                # MPU6050 driver implementation
    └── ssd1306/
        ├── CMakeLists.txt           # SSD1306 component configuration
        ├── ssd1306.h                # SSD1306 header file
        ├── ssd1306.c                # SSD1306 driver implementation
        └── font5x7.h                # Font data for display


### File Contents

#### Root CMakeLists.txt
cmake
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(mpu6050oled)


#### main/CMakeLists.txt
cmake
idf_component_register(SRCS "app_main.c"
                    INCLUDE_DIRS "."
                    REQUIRES mpu6050 ssd1306)


#### components/mpu6050/CMakeLists.txt
cmake
idf_component_register(SRCS "mpu6050.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver)


#### components/ssd1306/CMakeLists.txt
cmake
idf_component_register(SRCS "ssd1306.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver)


---

## 🚀 Building and Flashing

### Method 1: Using ESP-IDF Terminal (Recommended)

This is the *easiest and most reliable* method.

#### Step 1: Open ESP-IDF Terminal

1. In Espressif-IDE, look at the *bottom panel*
2. Click the *"Terminal"* tab
3. Click the *dropdown arrow* or *+* icon next to terminal icon
4. Select *"ESP-IDF 5.x CMD"* or *"ESP-IDF PowerShell"*

If Terminal tab is not visible:
- Go to Window → Show View → Terminal

#### Step 2: Navigate to Project Directory

bash
cd C:\Users\YourUsername\workspace\mpu6050oled


Replace YourUsername with your actual Windows username.

#### Step 3: Build the Project

bash
idf.py build


*Expected output:*

...
Project build complete. To flash, run:
idf.py -p (PORT) flash
or
idf.py -p (PORT) flash monitor
to build, flash and monitor output.
Total time taken to build the project: 9.845 ms


#### Step 4: Find Your COM Port

*Method A: Device Manager*
1. Press Win + X
2. Select *Device Manager*
3. Expand *"Ports (COM & LPT)"*
4. Look for your ESP32 device (e.g., "USB-SERIAL CH340 (COM3)")
5. Note the COM port number (e.g., COM3, COM5, COM7)

*Method B: Terminal Command*
bash
idf.py -p list


#### Step 5: Flash to ESP32

Replace COM3 with your actual COM port:

bash
idf.py -p COM3 flash monitor


*What this does:*
- Flashes the firmware to ESP32
- Automatically opens serial monitor
- Shows real-time output from ESP32

#### Step 6: Exit Monitor

To exit the serial monitor:
- Press Ctrl + ] (Control + Right Bracket)

---

### Method 2: Using Espressif-IDE GUI

#### Step 1: Import/Create Project

*Option A: Create New Project*
1. File → New → Espressif IDF Project
2. Project Name: mpu6050oled
3. Click Finish

*Option B: Import Existing*
1. File → Import... → Espressif → Existing IDF Project
2. Browse to project location
3. Click Finish

#### Step 2: Add Source Files

1. Right-click project → New → Folder → Create structure as shown above
2. Copy all .c, .h, and CMakeLists.txt files to respective folders
3. Right-click project → Refresh (or press F5)

#### Step 3: Build

*Using Build Button:*
- Click the *🔨 Build* icon in toolbar
- Or press Ctrl + B
- Or right-click project → Build Project

*Watch Console Output:*
- Check the *Console* tab at bottom
- Wait for "Build complete" message

#### Step 4: Configure Flash Target

1. Right-click project → Select ESP-IDF
2. Hover to see submenu
3. Look for flash options

*If ESP-IDF submenu doesn't work:*
Use the terminal method instead (Method 1).

#### Step 5: Flash

In Terminal within IDE:
bash
idf.py -p COM3 flash monitor


---

### Method 3: Using External ESP-IDF Command Prompt

#### Step 1: Open ESP-IDF CMD

1. Press Win key
2. Type *"ESP-IDF"*
3. Click *"ESP-IDF 5.x CMD"* (not regular CMD)

This opens a command prompt with ESP-IDF environment pre-configured.

#### Step 2: Navigate to Project

bash
cd C:\Users\YourUsername\workspace\mpu6050oled


#### Step 3: Build and Flash

bash
idf.py build
idf.py -p COM3 flash monitor


---

## 📊 Expected Output

### Serial Monitor Output


ESP-ROM:esp32-20200911
Build:Sep 11 2020
rst:0x1 (POWERON),boot:0x13 (SPI_FAST_FLASH_BOOT)
...

I (312) cpu_start: Starting scheduler on PRO CPU.
I (0) cpu_start: Starting scheduler on APP CPU.

I (323) MPU6050_OLED: Starting MPU6050 + OLED Display Application
I (334) MPU6050_OLED: I2C initialized successfully
I (456) SSD1306: Initializing SSD1306...
I (567) SSD1306: SSD1306 initialization complete
I (578) MPU6050_OLED: SSD1306 initialized successfully
I (689) MPU6050: Initializing MPU6050...
I (801) MPU6050: MPU6050 initialization complete
I (812) MPU6050_OLED: MPU6050 initialized successfully
I (823) MPU6050: MPU6050 WHO_AM_I: 0x68 - OK
I (834) MPU6050_OLED: Accel: X=123, Y=-456, Z=16384
I (934) MPU6050_OLED: Accel: X=125, Y=-454, Z=16382
I (1034) MPU6050_OLED: Accel: X=122, Y=-458, Z=16386
...


### OLED Display Output


┌────────────────────────┐
│   Accelerometer        │
├────────────────────────┤
│ X:    123             │
│                        │
│ Y:   -456             │
│                        │
│ Z:  16384             │
│                        │
│ Tilt: 12.3/-5.7       │
└────────────────────────┘


*Values update every 100ms (10 Hz)*

### Understanding the Values

- *X, Y, Z*: Raw accelerometer readings (±2g range)
  - Each axis: -32768 to +32767
  - At rest (flat): Z ≈ 16384 (1g), X ≈ 0, Y ≈ 0
  
- *Tilt*: Calculated angles in degrees
  - First value: X-axis tilt
  - Second value: Y-axis tilt

*Test Your Setup:*
- Board flat: Z ≈ 16384, X ≈ 0, Y ≈ 0
- Tilt forward: Y increases (positive)
- Tilt right: X increases (positive)
- Upside down: Z becomes negative

---

## 🔧 Troubleshooting

### Issue 1: Nothing on OLED Display

#### Symptom:
OLED remains completely black, no display at all.

#### Possible Causes & Solutions:

*A. Wrong I2C Address*

Some SSD1306 displays use 0x3D instead of 0x3C.

*Fix:*
Edit components/ssd1306/ssd1306.h:
c
// Line 8 - Try changing this
#define SSD1306_ADDR    0x3D    // Change from 0x3C to 0x3D


*B. Verify I2C Connection*

Add I2C scanner code to check what devices are connected.

In main/app_main.c, add this function before app_main():

c
void i2c_scanner(void)
{
    printf("\nScanning I2C bus...\n");
    for (uint8_t addr = 1; addr < 127; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        
        esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(50));
        i2c_cmd_link_delete(cmd);
        
        if (ret == ESP_OK) {
            printf("Device found at address 0x%02X\n", addr);
        }
    }
    printf("Scan complete\n\n");
}


Then call it in app_main() after I2C initialization:
c
ESP_ERROR_CHECK(i2c_master_init());
ESP_LOGI(TAG, "I2C initialized successfully");

// Add this line
i2c_scanner();


*Expected output:*

Scanning I2C bus...
Device found at address 0x3C
Device found at address 0x68
Scan complete


If you don't see 0x3C or 0x3D, check your wiring.

*C. Column Offset Issue*

Some displays need a column offset adjustment.

*Fix:*
Edit components/ssd1306/ssd1306.h:
c
// Line 14 - Try different values
#define SSD1306_COL_OFFSET    2    // Try 0, 2, or 4


*D. Wrong Display Configuration*

For 128x32 displays (not 128x64):

Edit components/ssd1306/ssd1306.c, in ssd1306_init() function:
c
// Change these two lines in the init_cmds array:
SSD1306_CMD_SET_MULTIPLEX_RATIO, 0x1F,    // Instead of 0x3F
SSD1306_CMD_SET_COM_PINS, 0x02,           // Instead of 0x12


---

### Issue 2: MPU6050 WHO_AM_I Check Fails

#### Symptom:

E (823) MPU6050_OLED: MPU6050 WHO_AM_I check failed! Got: 0xFF


#### Possible Causes & Solutions:

*A. Wrong I2C Address*

MPU6050 can be at 0x68 or 0x69 depending on AD0 pin.

*Check AD0 Pin:*
- AD0 → GND = Address 0x68 (default)
- AD0 → VCC = Address 0x69

*Fix:*
Edit components/mpu6050/mpu6050.h:
c
// Line 7 - Change if AD0 is connected to VCC
#define MPU6050_ADDR    0x69    // Change from 0x68 to 0x69


*B. Connection Issue*

Run I2C scanner (see Issue 1B) to verify MPU6050 is detected.

*C. Power Supply Problem*

- Verify 3.3V power is stable
- Some MPU6050 modules have voltage regulator and can use 5V
- Check your module's specification

---

### Issue 3: Build Errors

#### Symptom:

ninja: error: loading 'build.ninja': The system cannot find the file specified.


#### Solution:

*Clean and rebuild:*
bash
idf.py fullclean
idf.py build


*Or delete build folder manually:*
1. In Espressif-IDE: Right-click build folder → Delete
2. Build again: idf.py build

---

### Issue 4: Display Shows Garbage/Random Pixels

#### Symptom:
Display shows random pixels or noise instead of text.

#### Solutions:

*A. Reduce I2C Speed*

Edit main/app_main.c:
c
// Line 14 - Reduce from 100000 to 50000
#define I2C_MASTER_FREQ_HZ    50000    // 50 kHz instead of 100 kHz


*B. Add Delays*

Edit components/ssd1306/ssd1306.c, in ssd1306_update() function:
c
// Add small delay between chunks
for (size_t i = 0; i < sizeof(framebuffer); i += chunk_size) {
    size_t remaining = sizeof(framebuffer) - i;
    size_t size = (remaining < chunk_size) ? remaining : chunk_size;
    ssd1306_write_data(&framebuffer[i], size);
    vTaskDelay(pdMS_TO_TICKS(1));  // Add 1ms delay
}


*C. Force Clear on Init*

Edit main/app_main.c, after ssd1306_init():
c
ssd1306_init();
vTaskDelay(pdMS_TO_TICKS(100));
ssd1306_clear();
ssd1306_update();
vTaskDelay(pdMS_TO_TICKS(100));


---

### Issue 5: Flash Failed - Serial Port Not Found

#### Symptom:

serial.serialutil.SerialException: could not open port 'COM3'


#### Solutions:

*A. Check Device Manager*
1. Press Win + X → Device Manager
2. Look under "Ports (COM & LPT)"
3. Verify your ESP32 is listed with a COM port
4. Use that exact COM port number

*B. Close Other Programs*
- Close any serial monitor programs
- Close Arduino IDE if open
- Close Putty or other terminal programs

*C. Reconnect USB*
1. Unplug ESP32
2. Wait 5 seconds
3. Plug back in
4. Check Device Manager for new COM port

*D. Try Different USB Port*
Sometimes USB ports have issues. Try a different port.

---

### Issue 6: Permission Denied (Linux/Mac)

#### Symptom:

serial.serialutil.SerialException: [Errno 13] Permission denied: '/dev/ttyUSB0'


#### Solution:

Add your user to dialout group:
bash
sudo usermod -a -G dialout $USER


Then logout and login again.

---

### Issue 7: Display Flickering

#### Symptom:
Display flickers or updates appear slow.

#### Solutions:

*A. Increase Update Interval*

Edit main/app_main.c:
c
// At end of while loop - change from 100 to 200
vTaskDelay(pdMS_TO_TICKS(200));  // Update every 200ms instead of 100ms


*B. Increase I2C Speed* (if wiring is good)

Edit main/app_main.c:
c
#define I2C_MASTER_FREQ_HZ    400000  // 400 kHz (fast mode)


---

### Issue 8: Text is Inverted or Upside Down

#### Solution:

Edit components/ssd1306/ssd1306.c, in ssd1306_init() function:

c
// Try different combinations of these two lines:
SSD1306_CMD_SET_SEGMENT_REMAP | 0x00,  // Normal (default is | 0x01)
SSD1306_CMD_SET_COM_SCAN_INC,          // Normal scan (default is DEC)


---

## 🔬 Technical Details

### I2C Configuration

| Parameter | Value | Notes |
|-----------|-------|-------|
| Speed | 100 kHz | Standard mode |
| Pull-ups | Internal enabled | 4.7kΩ external recommended |
| Timeout | 1000 ms | For all I2C operations |
| Bus | I2C_NUM_0 | ESP32 I2C port 0 |

### MPU6050 Configuration

| Parameter | Value | Register |
|-----------|-------|----------|
| Clock Source | PLL with X-axis gyro | PWR_MGMT_1 |
| Accel Range | ±2g | ACCEL_CONFIG |
| Gyro Range | ±250°/s | GYRO_CONFIG |
| Sample Rate | 1 kHz | SMPLRT_DIV |
| DLPF | 94 Hz bandwidth | CONFIG |

*Accelerometer Sensitivity:*
- ±2g range: 16384 LSB/g
- 1g ≈ 16384 raw value
- Formula: accel_g = raw_value / 16384.0

### SSD1306 Configuration

| Parameter | Value | Command |
|-----------|-------|---------|
| Resolution | 128x64 pixels | - |
| Memory Mode | Horizontal | 0x20, 0x00 |
| Multiplex Ratio | 64 | 0xA8, 0x3F |
| COM Pins | Alternative | 0xDA, 0x12 |
| Contrast | 0xCF | 0x81, 0xCF |
| Charge Pump | Enabled | 0x8D, 0x14 |

*Display Memory:*
- Framebuffer: 1024 bytes (128 × 64 ÷ 8)
- Update method: Full refresh
- Font: 5×7 pixels per character
- Characters: ASCII 32-126 (95 characters)

---

## ✨ Features Implemented

### Phase A: MPU6050 Driver (Register-Level)

✅ *Complete Initialization Sequence*
- Device reset via PWR_MGMT_1 register
- Wake from sleep mode
- Clock source: PLL with X-axis gyroscope
- Configurable accelerometer range (±2g default)
- Configurable gyroscope range (±250°/s default)
- Sample rate: 1 kHz
- Digital Low Pass Filter: 94 Hz bandwidth

✅ *Burst Read Implementation*
- Reads 6 bytes in single I2C transaction
- Efficient register access
- Atomic data capture
- Reduces I2C overhead from 3 transactions to 1

✅ *WHO_AM_I Validation*
- Verifies device ID (0x68)
- Ensures proper I2C communication
- Pre-flight check before main loop

✅ *Register-Level Access*
- No high-level libraries used
- Direct register manipulation
- Full control over hardware
- Educational and professional approach

---

### Phase B: SSD1306 Driver (Register-Level)

✅ *Primitive Implementation*
- Full register-level initialization (17 commands)
- Framebuffer management (1024 bytes)
- Pixel-level drawing with coordinate validation
- Line drawing using Bresenham's algorithm
- Rectangle drawing (outline and filled)
- Display control (on/off, contrast, invert)

✅ *Font Engine*
- 5×7 bitmap font
- ASCII characters 32-126 (95 total)
- Vertical byte format storage
- Character rendering function
- String rendering with automatic line wrap
- Support for newline characters

✅ *Bus Management*
- Proper I2C command/data differentiation
- Control byte protocol:
  - 0x00: Command stream
  - 0x40: Data stream
  - 0x80: Single command
- Chunked data transfer (64 bytes per chunk)
- Horizontal addressing mode for efficiency
- Error handling for all I2C operations

✅ *Drawing Functions*
- ssd1306_draw_pixel(): Individual pixel control
- ssd1306_draw_line(): Bresenham's line algorithm
- ssd1306_draw_rect(): Rectangle outline
- ssd1306_fill_rect(): Filled rectangle
- ssd1306_draw_char(): Single character
- ssd1306_draw_string(): Multi-character strings

---

### Phase C: Integration & Display

✅ *Live Sensor Display*
- Real-time accelerometer values (X, Y, Z)
- 10 Hz update rate (100ms interval)
- Clear text labeling
- Professional UI layout:
  - Title bar: "Accelerometer"
  - Separator line
  - Formatted value display
  - Tilt angle calculation

✅ *Data Processing*
- 16-bit signed integer parsing
- Big-endian byte order handling
- Real-time value updates
- Smooth display rendering

✅ *Error Handling*
- All functions return esp_err_t status codes
- Initialization failure detection
- WHO_AM_I validation
- Graceful error messages on display
- Serial logging for debugging

---

### Bonus: Tilt Angle Calculation

✅ *Implemented*
- Calculates tilt angles using atan2()
- X-axis tilt: atan2(Y, Z) × 180/π
- Y-axis tilt: atan2(X, Z) × 180/π
- Displayed on OLED bottom line
- Ready for spirit level UI enhancement

*Future Enhancement - Spirit Level UI:*
- Graphical bubble representation
- Circular bubble that moves with tilt
- Smooth animation
- Partial screen updates for efficiency
- Low-pass filter for smooth movement

---

## 📚 Driver Architecture

### Design Decisions

*1. Polling vs Interrupt-Driven*

*Current: Polling-based*

*Rationale:*
- ✅ Simpler implementation
- ✅ Easier to debug
- ✅ Predictable timing
- ✅ No interrupt priority conflicts
- ✅ Sufficient for 10 Hz display updates

*Trade-offs:*
- Higher CPU usage (mitigated by FreeRTOS delays)
- Slightly higher latency (acceptable for this application)

*Future: Interrupt-driven option*
- Use MPU6050 INT pin
- Data-ready interrupt
- Lower CPU usage
- Faster response time
- More complex code

---

*2. Framebuffer Strategy*

*Current: Full framebuffer update*

*Rationale:*
- ✅ Simple to implement
- ✅ No complexity in tracking changes
- ✅ Consistent update timing
- ✅ Works well at 10 Hz

*Trade-offs:*
- Transfers 1024 bytes per update
- Slightly higher I2C bus usage

*Future: Partial update option*
- Track dirty rectangles
- Update only changed areas
- Reduce I2C traffic
- Enable faster update rates

---

*3. I2C Bus Sharing*

Both devices share the same I2C bus:
- Sequential access (no contention)
- Proper timeout handling
- Error recovery mechanisms
- Stable operation

---

## 🎯 Assessment Criteria Met

### Phase A Requirements: ✅ Complete

- [x] Initialization routine implemented
- [x] Wake sensor from sleep
- [x] Configure clock source (PLL with X-gyro)
- [x] Set accelerometer full-scale range (±2g)
- [x] Burst-read sequence for 6 bytes
- [x] WHO_AM_I validation before main loop
- [x] All at register level (no high-level libraries)

### Phase B Requirements: ✅ Complete

- [x] Display initialization (17 register commands)
- [x] Clear screen function
- [x] Draw individual pixels
- [x] Draw simple shapes (lines, rectangles)
- [x] Font engine (5×7 bitmap, 95 characters)
- [x] Render alphanumeric text
- [x] Proper I2C bus management
- [x] Control byte differentiation (command/data)

### Phase C Requirements: ✅ Complete

- [x] Display live X, Y, Z values
- [x] Clear text labeling on display
- [x] Real-time updates (10 Hz)
- [x] Integration of both components
- [x] Professional UI layout

### Bonus Task: 🔄 Partially Complete

- [x] Tilt angle calculation
- [x] Display tilt values
- [ ] Graphical bubble UI (ready to implement)
- [ ] Smooth animation (ready to implement)
- [ ] Optimized screen updates (ready to implement)

### Code Quality: ✅ Complete

- [x] Clean, modular code structure
- [x] Comprehensive comments
- [x] Logical component separation
- [x] Error handling throughout
- [x] Register-level documentation
- [x] Professional naming conventions

---

## 📖 Code Walkthrough Guide

For the technical review, be prepared to explain:

### 1. I2C Communication

*Write Operation:*
c
// In mpu6050.c - mpu6050_write_byte()
uint8_t write_buf[2] = {reg_addr, data};
return i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_ADDR, 
                                  write_buf, sizeof(write_buf),
                                  timeout);


*Burst Read Operation:*
c
// In mpu6050.c - mpu6050_read_bytes()
return i2c_master_write_read_device(I2C_MASTER_NUM, MPU6050_ADDR,
                                   &reg_addr, 1, data, len,
                                   timeout);


*Explanation:*
- Write: Send register address + data byte
- Read: Send register address, then read multiple bytes
- Burst read: More efficient than individual reads

---

### 2. MPU6050 Initialization Sequence

c
// 1. Reset device (bit 7 of PWR_MGMT_1)
mpu6050_write_byte(MPU6050_REG_PWR_MGMT_1, 0x80);
vTaskDelay(100ms);  // Wait for reset

// 2. Wake up, set clock source (bits 0-2 of PWR_MGMT_1)
mpu6050_write_byte(MPU6050_REG_PWR_MGMT_1, 0x01);  // PLL with X-gyro

// 3. Disable sleep in PWR_MGMT_2
mpu6050_write_byte(MPU6050_REG_PWR_MGMT_2, 0x00);

// 4. Configure sample rate (SMPLRT_DIV)
mpu6050_write_byte(MPU6050_REG_SMPLRT_DIV, 0x00);  // 1kHz

// 5. Configure DLPF (CONFIG register)
mpu6050_write_byte(MPU6050_REG_CONFIG, 0x02);  // 94Hz bandwidth

// 6. Set accelerometer range (ACCEL_CONFIG)
mpu6050_write_byte(MPU6050_REG_ACCEL_CONFIG, 0x00);  // ±2g

// 7. Set gyroscope range (GYRO_CONFIG)
mpu6050_write_byte(MPU6050_REG_GYRO_CONFIG, 0x00);  // ±250°/s


---

### 3. SSD1306 Initialization

c
// Essential commands from ssd1306_init()
SSD1306_CMD_DISPLAY_OFF,                    // Turn off while configuring
SSD1306_CMD_SET_MULTIPLEX_RATIO, 0x3F,      // 64 rows
SSD1306_CMD_CHARGE_PUMP, 0x14,              // Enable charge pump (required)
SSD1306_CMD_SET_MEMORY_MODE, 0x00,          // Horizontal addressing
SSD1306_CMD_SET_SEGMENT_REMAP | 0x01,       // Flip horizontally
SSD1306_CMD_SET_COM_SCAN_DEC,               // Flip vertically
SSD1306_CMD_DISPLAY_ON                      // Turn on display


---

### 4. Framebuffer to Display Mapping

c
// Framebuffer organization:
// - 1024 bytes total (128 columns × 8 pages)
// - Each page = 8 vertical pixels
// - Byte format: LSB = top pixel, MSB = bottom pixel

// Draw pixel formula:
uint16_t index = x + (y / 8) * 128;  // Which byte in framebuffer
uint8_t bit = y % 8;                 // Which bit in that byte

if (on) {
    framebuffer[index] |= (1 << bit);   // Set bit
} else {
    framebuffer[index] &= ~(1 << bit);  // Clear bit
}


---

### 5. Handling Trade-offs

*Why polling instead of interrupts?*
- 10 Hz update rate doesn't require interrupts
- Simpler code, easier to maintain
- More predictable for demonstration

*Why full framebuffer updates?*
- Simple and reliable
- Consistent timing
- Good enough for 10 Hz refresh rate
- Can optimize later if needed

*Why horizontal addressing mode?*
- Efficient for full-screen updates
- Sequential memory access
- Matches framebuffer organization

---

## 🔄 Next Steps: Implementing Spirit Level

### Algorithm

c
// 1. Calculate angles
float pitch = atan2(accel.y, sqrt(accel.x*accel.x + accel.z*accel.z));
float roll = atan2(-accel.x, accel.z);

// 2. Map to screen coordinates (center = 64, 32)
int bubble_x = 64 + (int)(roll * 30);    // Scale factor: 30 pixels per radian
int bubble_y = 32 + (int)(pitch * 30);

// 3. Clamp to screen bounds
if (bubble_x < 5) bubble_x = 5;
if (bubble_x > 123) bubble_x = 123;
if (bubble_y < 5) bubble_y = 5;
if (bubble_y > 59) bubble_y = 59;

// 4. Draw circular bubble (radius = 5 pixels)
for (int i = -5; i <= 5; i++) {
    for (int j = -5; j <= 5; j++) {
        if (i*i + j*j <= 25) {  // Circle equation: r² = 25
            ssd1306_draw_pixel(bubble_x + i, bubble_y + j, 1);
        }
    }
}

// 5. Draw reference circle at center (target)
for (int angle = 0; angle < 360; angle += 10) {
    float rad = angle * 3.14159 / 180.0;
    int x = 64 + (int)(10 * cos(rad));
    int y = 32 + (int)(10 * sin(rad));
    ssd1306_draw_pixel(x, y, 1);
}


### Optimization for Smooth Movement

c
// Low-pass filter for smooth bubble movement
static float filtered_roll = 0;
static float filtered_pitch = 0;
const float alpha = 0.3;  // Smoothing factor (0-1)

filtered_roll = alpha * roll + (1 - alpha) * filtered_roll;
filtered_pitch = alpha * pitch + (1 - alpha) * filtered_pitch;

// Use filtered values for bubble position
int bubble_x = 64 + (int)(filtered_roll * 30);
int bubble_y = 32 + (int)(filtered_pitch * 30);


---

## 📞 Support & Resources

### Documentation References

- [MPU6050 Register Map](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf)
- [SSD1306 Datasheet](https://cdn-shop.adafruit.com/datasheets/SSD1306.pdf)
- [ESP-IDF I2C Driver](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/i2c.html)
- [ESP-IDF Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/index.html)

### Common Commands Reference

bash
# Build
idf.py build

# Clean build
idf.py fullclean
idf.py build

# Flash only
idf.py -p COM3 flash

# Monitor only
idf.py -p COM3 monitor

# Flash and monitor
idf.py -p COM3 flash monitor

# Exit monitor
Ctrl + ]

# Project configuration
idf.py menuconfig

# Erase flash
idf.py -p COM3 erase-flash


---

## ✅ Final Checklist

Before submitting:

- [ ] All files in correct folder structure
- [ ] Code compiles without errors or warnings
- [ ] WHO_AM_I check passes
- [ ] Display shows live accelerometer data
- [ ] Values update smoothly at 10 Hz
- [ ] Tilt angles calculated correctly
- [ ] Code is well-commented
- [ ] Git repository initialized
- [ ] Commits show logical progression
- [ ] README.md is complete
- [ ] All test cases pass

---

## 📝 License

This project is created for educational purposes as part of the Estro Tech Robotics Embedded Driver Development Challenge.
