#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <sensor_msgs/msg/imu.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"
#endif

// ============ CONFIGURATION ============
// IMPORTANT: Adjust these pins to match your wiring!
#define MPU_ADDR 0x68           // Try 0x69 if this doesn't work
#define I2C_NUM I2C_NUM_0       
#define I2C_SCL_IO GPIO_NUM_19  // Change to GPIO_NUM_19 if needed
#define I2C_SDA_IO GPIO_NUM_18  // Change to GPIO_NUM_18 if needed
#define I2C_FREQ_HZ 100000      
#define PUBLISH_FREQ_HZ 50      

// MPU6050 Registers
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_SMPLRT_DIV   0x19
#define MPU6050_CONFIG       0x1A
#define MPU6050_GYRO_CONFIG  0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_WHO_AM_I     0x75
#define MPU6050_ACCEL_XOUT_H 0x3B

// Scaling factors
#define ACCEL_SCALE 16384.0f    
#define GYRO_SCALE 131.0f       
#define GRAVITY 9.81f
#define DEG_TO_RAD 0.017453292519943295f

// Error handling macros
#define RCCHECK(fn) { \
    rcl_ret_t temp_rc = fn; \
    if((temp_rc != RCL_RET_OK)){ \
        printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); \
        vTaskDelete(NULL); \
    } \
}

#define RCSOFTCHECK(fn) { \
    rcl_ret_t temp_rc = fn; \
    if((temp_rc != RCL_RET_OK)){ \
        printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc); \
    } \
}

// ============ GLOBAL VARIABLES ============
rcl_publisher_t publisher;
sensor_msgs__msg__Imu imu_msg;

// ============ I2C FUNCTIONS ============

static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_IO,
        .scl_io_num = I2C_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };
    
    esp_err_t err = i2c_param_config(I2C_NUM, &conf);
    if (err != ESP_OK) {
        printf("I2C config failed: %d\n", err);
        return err;
    }
    
    err = i2c_driver_install(I2C_NUM, conf.mode, 0, 0, 0);
    if (err != ESP_OK) {
        printf("I2C driver install failed: %d\n", err);
        return err;
    }
    
    printf("I2C initialized: SDA=GPIO%d, SCL=GPIO%d\n", I2C_SDA_IO, I2C_SCL_IO);
    return ESP_OK;
}

static esp_err_t mpu_write_reg(uint8_t reg, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t mpu_read_reg(uint8_t reg, uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | I2C_MASTER_READ, true);
    
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// ============ I2C SCANNER ============

static void i2c_scanner(void)
{
    printf("\n=== I2C Scanner ===\n");
    printf("Scanning I2C bus...\n");
    int devices_found = 0;
    
    for (uint8_t addr = 0x03; addr < 0x78; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        
        esp_err_t ret = i2c_master_cmd_begin(I2C_NUM, cmd, 50 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        
        if (ret == ESP_OK) {
            printf("  Found device at address: 0x%02X\n", addr);
            devices_found++;
        }
    }
    
    if (devices_found == 0) {
        printf("  ERROR: No I2C devices found!\n");
        printf("  Check wiring and connections.\n");
    } else {
        printf("  Total devices found: %d\n", devices_found);
    }
    printf("===================\n\n");
}

// ============ MPU6050 FUNCTIONS ============

static esp_err_t mpu6050_init(void)
{
    uint8_t who_am_i;
    esp_err_t ret;
    
    printf("Initializing MPU6050...\n");
    
    // Check WHO_AM_I
    ret = mpu_read_reg(MPU6050_WHO_AM_I, &who_am_i, 1);
    if (ret != ESP_OK) {
        printf("Failed to read WHO_AM_I: %d\n", ret);
        return ret;
    }
    printf("WHO_AM_I: 0x%02X\n", who_am_i);
    
    // Wake up device
    ret = mpu_write_reg(MPU6050_PWR_MGMT_1, 0x00);
    if (ret != ESP_OK) {
        printf("Failed to wake up MPU6050: %d\n", ret);
        return ret;
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    // Configure sample rate (1kHz / (1 + 7) = 125Hz)
    mpu_write_reg(MPU6050_SMPLRT_DIV, 0x07);
    
    // Configure gyroscope (±250°/s)
    mpu_write_reg(MPU6050_GYRO_CONFIG, 0x00);
    
    // Configure accelerometer (±2g)
    mpu_write_reg(MPU6050_ACCEL_CONFIG, 0x00);
    
    // Configure DLPF (~44Hz bandwidth)
    mpu_write_reg(MPU6050_CONFIG, 0x03);
    
    printf("MPU6050 initialized successfully\n");
    return ESP_OK;
}

static esp_err_t read_imu_data(float *ax, float *ay, float *az,
                               float *gx, float *gy, float *gz)
{
    uint8_t data[14];
    
    esp_err_t ret = mpu_read_reg(MPU6050_ACCEL_XOUT_H, data, 14);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Parse raw data
    int16_t raw_ax = (data[0] << 8) | data[1];
    int16_t raw_ay = (data[2] << 8) | data[3];
    int16_t raw_az = (data[4] << 8) | data[5];
    int16_t raw_gx = (data[8] << 8) | data[9];
    int16_t raw_gy = (data[10] << 8) | data[11];
    int16_t raw_gz = (data[12] << 8) | data[13];
    
    // Convert to physical units
    *ax = (raw_ax / ACCEL_SCALE) * GRAVITY;
    *ay = (raw_ay / ACCEL_SCALE) * GRAVITY;
    *az = (raw_az / ACCEL_SCALE) * GRAVITY;
    
    *gx = (raw_gx / GYRO_SCALE) * DEG_TO_RAD;
    *gy = (raw_gy / GYRO_SCALE) * DEG_TO_RAD;
    *gz = (raw_gz / GYRO_SCALE) * DEG_TO_RAD;
    
    return ESP_OK;
}

// ============ ROS TIMER CALLBACK ============

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    static int msg_count = 0;
    
    if (timer != NULL) {
        float ax, ay, az, gx, gy, gz;
        
        if (read_imu_data(&ax, &ay, &az, &gx, &gy, &gz) == ESP_OK) {
            // Print every 50 messages (~1 second at 50Hz)
            if (++msg_count % 50 == 0) {
                printf("IMU #%d: Accel[%.2f, %.2f, %.2f] Gyro[%.3f, %.3f, %.3f]\n",
                       msg_count, ax, ay, az, gx, gy, gz);
            }
            
            // Update timestamp
            int64_t time_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
            imu_msg.header.stamp.sec = (int32_t)(time_ms / 1000);
            imu_msg.header.stamp.nanosec = (uint32_t)((time_ms % 1000) * 1000000);
            
            // Fill IMU data
            imu_msg.linear_acceleration.x = ax;
            imu_msg.linear_acceleration.y = ay;
            imu_msg.linear_acceleration.z = az;
            
            imu_msg.angular_velocity.x = gx;
            imu_msg.angular_velocity.y = gy;
            imu_msg.angular_velocity.z = gz;
            
            // Publish
            RCSOFTCHECK(rcl_publish(&publisher, &imu_msg, NULL));
        } else {
            printf("Failed to read IMU data\n");
        }
    }
}

// ============ MAIN APPLICATION ============

void appMain(void *arg)
{
    printf("\n========================================\n");
    printf("  MPU6050 micro-ROS Publisher\n");
    printf("========================================\n\n");
    
    // Initialize I2C
    if (i2c_master_init() != ESP_OK) {
        printf("ERROR: I2C initialization failed!\n");
        vTaskDelete(NULL);
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    // Scan I2C bus to find devices
    printf("Scanning for I2C devices...\n");
    i2c_scanner();
    
    // Initialize MPU6050
    if (mpu6050_init() != ESP_OK) {
        printf("ERROR: MPU6050 initialization failed!\n");
        printf("Check your wiring:\n");
        printf("  - VCC -> 3.3V\n");
        printf("  - GND -> GND\n");
        printf("  - SCL -> GPIO%d\n", I2C_SCL_IO);
        printf("  - SDA -> GPIO%d\n", I2C_SDA_IO);
        printf("  - AD0 -> GND (for address 0x68) or VCC (for 0x69)\n");
        printf("\nIf device was found but init failed, try changing MPU_ADDR\n");
        vTaskDelete(NULL);
    }
    
    // Test read
    float ax, ay, az, gx, gy, gz;
    if (read_imu_data(&ax, &ay, &az, &gx, &gy, &gz) == ESP_OK) {
        printf("Test read OK: Accel[%.2f, %.2f, %.2f] m/s²\n", ax, ay, az);
    } else {
        printf("ERROR: Failed to read sensor data!\n");
        vTaskDelete(NULL);
    }
    
    // Initialize micro-ROS
    printf("\nInitializing micro-ROS...\n");
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    
    // Create node
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "mpu6050_imu_node", "", &support));
    
    // Create publisher
    RCCHECK(rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu/data_raw"));
    
    // Initialize IMU message
    sensor_msgs__msg__Imu__init(&imu_msg);
    
    // Set frame_id
    static char frame_id[] = "imu_link";
    imu_msg.header.frame_id.data = frame_id;
    imu_msg.header.frame_id.size = strlen(frame_id);
    imu_msg.header.frame_id.capacity = sizeof(frame_id);
    
    // Set covariance (orientation not available from raw MPU6050)
    imu_msg.orientation_covariance[0] = -1.0;
    
    // Create timer
    rcl_timer_t timer;
    const unsigned int timer_timeout = 1000 / PUBLISH_FREQ_HZ;
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback));
    
    // Create executor
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    
    printf("micro-ROS ready!\n");
    printf("Publishing on topic: /imu/data_raw at %dHz\n", PUBLISH_FREQ_HZ);
    printf("========================================\n\n");
    
    // Main loop
    while(1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        usleep(10000);
    }
    
    // Cleanup (never reached)
    RCCHECK(rcl_publisher_fini(&publisher, &node));
    RCCHECK(rcl_node_fini(&node));
    vTaskDelete(NULL);
}