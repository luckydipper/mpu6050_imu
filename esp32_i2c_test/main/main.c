#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_err.h"

static const char *TAG = "I2C_TEST";

// ============ CONFIGURATION - TRY DIFFERENT COMBINATIONS ============
// Configuration 1: GPIO 18/19
#define I2C_SDA_IO          GPIO_NUM_18
#define I2C_SCL_IO          GPIO_NUM_19

// Configuration 2: GPIO 21/22 (uncomment to try)
// #define I2C_SDA_IO          GPIO_NUM_21
// #define I2C_SCL_IO          GPIO_NUM_22

#define I2C_MASTER_NUM      I2C_NUM_0
#define I2C_MASTER_FREQ_HZ  100000        // 100kHz - try 50000 if this fails
#define MPU6050_ADDR        0x68          // Try 0x69 if 0x68 doesn't work

// MPU6050 Registers
#define MPU6050_WHO_AM_I    0x75
#define MPU6050_PWR_MGMT_1  0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B

// ============ I2C INITIALIZATION ============

static esp_err_t i2c_master_init(void)
{
    ESP_LOGI(TAG, "Initializing I2C Master");
    ESP_LOGI(TAG, "SDA: GPIO%d, SCL: GPIO%d", I2C_SDA_IO, I2C_SCL_IO);
    
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_IO,
        .scl_io_num = I2C_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C config failed: %s", esp_err_to_name(err));
        return err;
    }
    
    err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(err));
        return err;
    }
    
    ESP_LOGI(TAG, "I2C initialized successfully");
    return ESP_OK;
}

// ============ I2C SCANNER ============

static void i2c_scanner(void)
{
    ESP_LOGI(TAG, "Starting I2C bus scan...");
    printf("\n");
    printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
    printf("00:         ");
    
    int devices_found = 0;
    
    for (int addr = 0; addr < 0x78; addr++) {
        if (addr % 16 == 0) {
            printf("\n%.2x:", addr);
        }
        
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 50 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        
        if (ret == ESP_OK) {
            printf(" %.2x", addr);
            devices_found++;
        } else {
            printf(" --");
        }
    }
    
    printf("\n\n");
    
    if (devices_found == 0) {
        ESP_LOGE(TAG, "No I2C devices found!");
        ESP_LOGW(TAG, "Check your wiring:");
        ESP_LOGW(TAG, "  VCC -> 3.3V");
        ESP_LOGW(TAG, "  GND -> GND");
        ESP_LOGW(TAG, "  SDA -> GPIO%d", I2C_SDA_IO);
        ESP_LOGW(TAG, "  SCL -> GPIO%d", I2C_SCL_IO);
    } else {
        ESP_LOGI(TAG, "Found %d device(s)", devices_found);
    }
}

// ============ MPU6050 FUNCTIONS ============

static esp_err_t mpu6050_read_register(uint8_t reg_addr, uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
    
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

static esp_err_t mpu6050_write_register(uint8_t reg_addr, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

static esp_err_t mpu6050_init(void)
{
    ESP_LOGI(TAG, "Initializing MPU6050 at address 0x%02X", MPU6050_ADDR);
    
    uint8_t who_am_i;
    esp_err_t ret = mpu6050_read_register(MPU6050_WHO_AM_I, &who_am_i, 1);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "WHO_AM_I: 0x%02X (expected: 0x68 or 0x72)", who_am_i);
    
    if (who_am_i != 0x68 && who_am_i != 0x72 && who_am_i != 0x70) {
        ESP_LOGW(TAG, "Unexpected WHO_AM_I value, but continuing...");
    }
    
    // Wake up MPU6050
    ret = mpu6050_write_register(MPU6050_PWR_MGMT_1, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wake up MPU6050: %s", esp_err_to_name(ret));
        return ret;
    }
    
    vTaskDelay(100 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "MPU6050 initialized successfully!");
    
    return ESP_OK;
}

static void mpu6050_read_accel(void)
{
    uint8_t data[6];
    esp_err_t ret = mpu6050_read_register(MPU6050_ACCEL_XOUT_H, data, 6);
    
    if (ret == ESP_OK) {
        int16_t accel_x = (data[0] << 8) | data[1];
        int16_t accel_y = (data[2] << 8) | data[3];
        int16_t accel_z = (data[4] << 8) | data[5];
        
        float ax = (accel_x / 16384.0) * 9.81;
        float ay = (accel_y / 16384.0) * 9.81;
        float az = (accel_z / 16384.0) * 9.81;
        
        ESP_LOGI(TAG, "Accel: X=%.2f, Y=%.2f, Z=%.2f m/s²", ax, ay, az);
        
        // Check if Z-axis shows gravity (sensor is working)
        if (az > 8.0 && az < 11.0) {
            ESP_LOGI(TAG, "✓ Z-axis shows gravity! Sensor is working correctly!");
        }
    } else {
        ESP_LOGE(TAG, "Failed to read accelerometer: %s", esp_err_to_name(ret));
    }
}

// ============ MAIN APPLICATION ============

void app_main(void)
{
    printf("\n\n");
    printf("================================================\n");
    printf("   ESP32 I2C MPU6050 Hardware Test\n");
    printf("================================================\n\n");
    
    // Step 1: Initialize I2C
    ESP_LOGI(TAG, "Step 1: Initialize I2C bus");
    if (i2c_master_init() != ESP_OK) {
        ESP_LOGE(TAG, "I2C initialization failed!");
        return;
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
    
    // Step 2: Scan I2C bus
    ESP_LOGI(TAG, "Step 2: Scan I2C bus for devices");
    i2c_scanner();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    // Step 3: Try to initialize MPU6050
    ESP_LOGI(TAG, "Step 3: Initialize MPU6050 sensor");
    if (mpu6050_init() == ESP_OK) {
        ESP_LOGI(TAG, "✓ MPU6050 is connected and responding!");
        
        // Step 4: Read data continuously
        ESP_LOGI(TAG, "Step 4: Reading accelerometer data...");
        ESP_LOGI(TAG, "Tip: Move the sensor to see values change!\n");
        
        while (1) {
            mpu6050_read_accel();
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    } else {
        ESP_LOGE(TAG, "✗ MPU6050 initialization failed!");
        ESP_LOGW(TAG, "\nTroubleshooting:");
        ESP_LOGW(TAG, "1. Check if device was found in I2C scan above");
        ESP_LOGW(TAG, "2. If found at different address, change MPU6050_ADDR in code");
        ESP_LOGW(TAG, "3. If not found, check physical wiring");
        ESP_LOGW(TAG, "4. Try different GPIO pins (edit I2C_SDA_IO and I2C_SCL_IO)");
        ESP_LOGW(TAG, "5. Measure VCC with multimeter (should be 3.3V)");
        
        // Keep scanning every 3 seconds
        while (1) {
            vTaskDelay(3000 / portTICK_PERIOD_MS);
            ESP_LOGI(TAG, "Scanning again...");
            i2c_scanner();
        }
    }
}
