#include "bq27220.h"
#include "bq27220_regs.h"

#include "esp_err.h"



 esp_err_t i2c_bq27220_init(bq27220_t *cfg)
{
    // I2C bus config (new driver)
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = -1, // auto-select
        .sda_io_num = (gpio_num_t)GROVE_SDA,
        .scl_io_num = (gpio_num_t)GROVE_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false, // обычно внешние подтяжки на Grove уже есть
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &cfg->s_i2c_bus));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = BQ27220_I2C_ADDRESS, // 0x55 (7-bit)
        .scl_speed_hz    = 400000,              // 100k если будут ошибки
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(cfg->s_i2c_bus, &dev_cfg, &cfg->s_bq_dev));



    return ESP_OK;


}

// read 16-bit little-endian value from Standard Command register
 esp_err_t bq_read_u16(uint8_t reg, uint16_t *out, bq27220_t *cfg)
{
    uint8_t rx[2] = {0};
    esp_err_t err = i2c_master_transmit_receive(
        cfg->s_bq_dev,
        &reg, 1,
        rx, 2,
        50 /* timeout ms */
    );
    if (err != ESP_OK) return err;

    *out = (uint16_t)rx[0] | ((uint16_t)rx[1] << 8); // LSB,MSB
    return ESP_OK;
}

esp_err_t bq_write_subcmd(uint16_t subcmd, bq27220_t *cfg)
{
    // Формируем пакет: [регистр 0x3E] [LSB команды] [MSB команды]
    uint8_t tx[3];
    tx[0] = 0x3E; // Control() register
    tx[1] = (uint8_t)(subcmd & 0x00FF);
    tx[2] = (uint8_t)((subcmd >> 8) & 0x00FF);

    return i2c_master_transmit(cfg->s_bq_dev, tx, 3, 50);
}