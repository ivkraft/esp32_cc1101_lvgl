#include "driver/i2c_master.h"   // ESP-IDF 5.x new driver

#define GROVE_SDA 8
#define GROVE_SCL 18
#define BQ27220_I2C_ADDRESS 0x55

typedef struct {
    i2c_master_bus_handle_t s_i2c_bus;
    i2c_master_dev_handle_t s_bq_dev;
} bq27220_t;


#define BQ27220_REG_VOLTAGE        0x08
#define BQ27220_REG_SOC            0x2C

esp_err_t i2c_bq27220_init(bq27220_t *cfg);
esp_err_t bq_read_u16(uint8_t reg, uint16_t *out, bq27220_t *cfg);
esp_err_t bq_write_subcmd(uint16_t subcmd, bq27220_t *cfg);



