#include <stdio.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_MASTER_NUM           I2C_NUM_0
#define I2C_MASTER_SDA_IO        15
#define I2C_MASTER_SCL_IO        16
#define I2C_MASTER_FREQ_HZ       100000
#define I2C_MASTER_TIMEOUT_MS    1000

#define ITG3200_ADDR             0x68
#define ITG3200_REG_DEVID        0x00
#define ITG3200_REG_SAMPLE_RATE  0x15 // Fsample = Finternal / (divider+1)
#define ITG3200_REG_RANGE_FILTER 0x16 // B0-B2 Low Pass Filter B3-B4 Range
#define ITG3200_REG_INT          0x17
#define ITG3200_REG_DATA_RDY     0x1A // B0 Data Ready
#define ITG3200_REG_DATAX0       0x1D
#define ITG3200_REG_PWR          0x3E

static const char *TAG = "ITG3200";

esp_err_t itg3200_write_register(uint8_t reg, uint8_t value) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ITG3200_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, value, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t itg3200_read_register(uint8_t reg, uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ITG3200_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ITG3200_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return ret;
}

void i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

void itg3200_init() {
    uint8_t devid = 0;
    itg3200_read_register(ITG3200_REG_DEVID, &devid, 1);
    ESP_LOGI(TAG, "Device ID: 0x%02X", devid);
    itg3200_write_register(ITG3200_REG_PWR, 0b00000001); // wake up, xyz modes and ref clock selection 
    itg3200_write_register(ITG3200_REG_RANGE_FILTER, 0b00011000); // 256Hz ±2000°/sec 
}

void itg3200_read_xyz(int16_t *x, int16_t *y, int16_t *z) {
    uint8_t data[6];
    itg3200_read_register(ITG3200_REG_DATAX0, data, 6);
    *x = (int16_t)((data[0] << 8) | data[1]);
    *y = (int16_t)((data[2] << 8) | data[3]);
    *z = (int16_t)((data[4] << 8) | data[5]);
}

void app_main(void) {
    i2c_master_init();
    itg3200_init();

    while (1) {
        int16_t x, y, z;
        itg3200_read_xyz(&x, &y, &z);
        ESP_LOGI(TAG, "X: %d, Y: %d, Z: %d", x, y, z);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}