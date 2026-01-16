#include "bq25896.h"
#include "esp_check.h"

#define TAG "bq25896"

esp_err_t bq25896_init(bq25896_t *ctx, i2c_master_bus_handle_t bus)
{
    ESP_RETURN_ON_FALSE(ctx != NULL, ESP_ERR_INVALID_ARG, TAG, "ctx null");
    ESP_RETURN_ON_FALSE(bus != NULL, ESP_ERR_INVALID_ARG, TAG, "bus null");

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = BQ25896_SLAVE_ADDRESS,
        .scl_speed_hz     = 400000,
    };

    ctx->addr_7bit = BQ25896_SLAVE_ADDRESS;
    ctx->dev = NULL;

    return i2c_master_bus_add_device(bus, &dev_cfg, &ctx->dev);
}

esp_err_t bq25896_read_reg(bq25896_t *ctx, uint8_t reg, uint8_t *val)
{
    ESP_RETURN_ON_FALSE(ctx && ctx->dev && val, ESP_ERR_INVALID_ARG, TAG, "bad args");
    return i2c_master_transmit_receive(ctx->dev, &reg, 1, val, 1, 100);
}

esp_err_t bq25896_write_reg(bq25896_t *ctx, uint8_t reg, uint8_t val)
{
    ESP_RETURN_ON_FALSE(ctx && ctx->dev, ESP_ERR_INVALID_ARG, TAG, "bad args");

    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(ctx->dev, buf, sizeof(buf), 100);
}

esp_err_t bq25896_get_status(bq25896_t *ctx, bq25896_status_t *out)
{
    ESP_RETURN_ON_FALSE(out != NULL, ESP_ERR_INVALID_ARG, TAG, "out null");

    uint8_t v = 0;
    esp_err_t err = bq25896_read_reg(ctx, BQ25896_REG_SYS_STATUS, &v);
    if (err != ESP_OK) return err;

    // REG0B:
    // CHRG_STAT bits [4:3]
    // PG_STAT bit [2]
    out->raw_sys_status = v;
    out->chg_state = (bq25896_charge_state_t)((v >> 3) & 0x03);
    out->power_good = ((v >> 2) & 0x01) != 0;

    return ESP_OK;
}
