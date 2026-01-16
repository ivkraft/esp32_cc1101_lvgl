

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"



// Default 7-bit I2C address (as in your define)
#define BQ25896_SLAVE_ADDRESS 0x6B
#

// Registers (subset)
#define BQ25896_REG_SYS_STATUS   0x0B
#define BQ25896_REG_REG11        0x11

typedef enum {
    BQ25896_CHG_NOT_CHARGING = 0,  // 00b
    BQ25896_CHG_PRECHARGE    = 1,  // 01b
    BQ25896_CHG_FASTCHARGE   = 2,  // 10b
    BQ25896_CHG_DONE         = 3,  // 11b
} bq25896_charge_state_t;

typedef struct {
    i2c_master_dev_handle_t dev;
    uint8_t addr_7bit;
} bq25896_t;

typedef struct {
    bool power_good;                  // PG_STAT (REG0B bit2)
    bq25896_charge_state_t chg_state; // CHRG_STAT (REG0B bits4..3)
    uint8_t raw_sys_status;           // raw REG0B value (for debug)
} bq25896_status_t;

// Attach device to an existing I2C master bus
esp_err_t bq25896_init(bq25896_t *ctx, i2c_master_bus_handle_t bus);

// Basic register access
esp_err_t bq25896_read_reg(bq25896_t *ctx, uint8_t reg, uint8_t *val);
esp_err_t bq25896_write_reg(bq25896_t *ctx, uint8_t reg, uint8_t val);

// High-level: read REG0B and decode PG_STAT + CHRG_STAT
esp_err_t bq25896_get_status(bq25896_t *ctx, bq25896_status_t *out);

// Convenience helpers
static inline bool bq25896_is_charging_active(const bq25896_status_t *st)
{
    // “реально идёт заряд” = precharge или fastcharge
    return (st->chg_state == BQ25896_CHG_PRECHARGE) || (st->chg_state == BQ25896_CHG_FASTCHARGE);
}

static inline bool bq25896_is_power_present(const bq25896_status_t *st)
{
    // “внешнее питание присутствует” = power_good
    return st->power_good;
}

