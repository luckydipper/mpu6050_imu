# ESP32 I2C MPU6050 Hardware Test

This is a standalone ESP-IDF project to test I2C communication with MPU6050 sensor.

## Quick Start

```bash
cd ~/Desktop/DEV/esp32_i2c_test

# Setup ESP-IDF environment (choose one):
# Option 1: Use micro-ROS toolchain
. ~/uros_ws/firmware/toolchain/esp-idf/export.sh

# Option 2: Use your ESP-IDF installation
# . $HOME/esp/esp-idf/export.sh

# Configure (first time only)
idf.py set-target esp32

# Build, flash, and monitor
idf.py build flash monitor
# Hold BOOT button when you see "Connecting..."
# Press Ctrl+] to exit monitor
```

## Wiring

```
MPU6050    →    ESP32
VCC        →    3.3V
GND        →    GND
SCL        →    GPIO19 (or GPIO22)
SDA        →    GPIO18 (or GPIO21)
AD0        →    GND (or leave floating)
```

## Troubleshooting

### Change GPIO Pins

Edit `main/main.c` lines 13-14:
```c
#define I2C_SDA_IO          GPIO_NUM_21  // Change this
#define I2C_SCL_IO          GPIO_NUM_22  // Change this
```

### Change I2C Address

Edit `main/main.c` line 21:
```c
#define MPU6050_ADDR        0x69  // Try 0x69 if 0x68 doesn't work
```

### Slower I2C Speed

Edit `main/main.c` line 20:
```c
#define I2C_MASTER_FREQ_HZ  50000  // Try 50kHz instead of 100kHz
```

After any changes, rebuild:
```bash
idf.py build flash monitor
```

## Expected Output

When working:
```
I (xxx) I2C_TEST: Found 1 device(s)
I (xxx) I2C_TEST: WHO_AM_I: 0x68
I (xxx) I2C_TEST: MPU6050 initialized successfully!
I (xxx) I2C_TEST: Accel: X=0.05, Y=0.12, Z=9.81 m/s²
I (xxx) I2C_TEST: ✓ Z-axis shows gravity! Sensor is working correctly!
```
