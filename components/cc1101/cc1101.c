#include "cc1101.h"
#include "cc1101_regs.h"
#include <string.h>
#include <math.h>
#include "esp_log.h"

static const char *TAG = "cc1101";


static esp_err_t xfer(cc1101_t *cc, const uint8_t *tx, uint8_t *rx, int len)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = len * 8;
    t.tx_buffer = tx;
    t.rx_buffer = rx;
    return spi_device_polling_transmit(cc->dev, &t);
}

esp_err_t cc1101_init_dev(cc1101_t *cc, const cc1101_cfg_t *cfg)
{
    if (!cc || !cfg) return ESP_ERR_INVALID_ARG;
    memset(cc, 0, sizeof(*cc));
    cc->host = cfg->host;
    cc->pin_cs = cfg->pin_cs;

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = (cfg->clock_hz > 0) ? cfg->clock_hz : (2 * 1000 * 1000),
        .mode = 0,
        .spics_io_num = cfg->pin_cs, // CS управляет SPI driver
        .queue_size = 1,
        .flags = SPI_DEVICE_NO_DUMMY,
    };

    esp_err_t err = spi_bus_add_device(cfg->host, &devcfg, &cc->dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "CC1101 dev added host=%d cs=%d", (int)cfg->host, cfg->pin_cs);
    return ESP_OK;
}

esp_err_t cc1101_strobe(cc1101_t *cc, uint8_t strobe)
{
    uint8_t tx[1] = { strobe };
    return xfer(cc, tx, NULL, 1);
}

esp_err_t cc1101_read_status(cc1101_t *cc, uint8_t addr, uint8_t *outv)
{
    if (!outv) return ESP_ERR_INVALID_ARG;
    uint8_t tx[2] = { (uint8_t)(addr | CC1101_READ_BURST), 0x00 };
    uint8_t rx[2] = { 0, 0 };
    esp_err_t err = xfer(cc, tx, rx, 2);
    if (err == ESP_OK) *outv = rx[1];
    return err;
}

esp_err_t cc1101_write_reg(cc1101_t *cc, uint8_t addr, uint8_t val)
{
    uint8_t tx[2] = { addr, val };
    return xfer(cc, tx, NULL, 2);
}


esp_err_t cc1101_read_reg(cc1101_t *cc, uint8_t addr, uint8_t *outv)
{
    if (!outv) return ESP_ERR_INVALID_ARG;
    uint8_t tx[2] = { (uint8_t)(addr | CC1101_READ_SINGLE), 0x00 };
    uint8_t rx[2] = { 0, 0 };
    esp_err_t err = xfer(cc, tx, rx, 2);
    if (err == ESP_OK) *outv = rx[1];
    return err;
}

esp_err_t cc1101_set_freq_hz(cc1101_t *cc, uint32_t freq_hz)
{
    // FREQ = freq_hz * 2^16 / f_xosc
    uint32_t word = (uint32_t)llround((double)freq_hz * 65536.0 / (double)F_XOSC_HZ);

    esp_err_t err;
    err = cc1101_write_reg(cc, CC1101_FREQ2, (word >> 16) & 0xFF); if (err) return err;
    err = cc1101_write_reg(cc, CC1101_FREQ1, (word >>  8) & 0xFF); if (err) return err;
    err = cc1101_write_reg(cc, CC1101_FREQ0, (word >>  0) & 0xFF); return err;
}


esp_err_t cc1101_enter_rx(cc1101_t *cc)
{
    esp_err_t err = cc1101_strobe(cc, CC1101_SIDLE);
    if (err != ESP_OK) return err;
    return cc1101_strobe(cc, CC1101_SRX);
}


esp_err_t cc1101_read_rssi_dbm(cc1101_t *cc, int16_t *out_dbm)
{
    if (!out_dbm) return ESP_ERR_INVALID_ARG;
    uint8_t raw = 0;

    esp_err_t err = cc1101_read_status(cc, CC1101_RSSI, &raw);
    if (err != ESP_OK) return err;

    int rssi_dec = (raw >= 128) ? ((int)raw - 256) : (int)raw;
    // datasheet: RSSI_dBm ≈ rssi_dec/2 - 74
    *out_dbm = (int16_t)(rssi_dec / 2 - 74);
    return ESP_OK;
}

esp_err_t cc1101_write_burst_reg(cc1101_t *cc, uint8_t addr, const uint8_t *data, size_t len)
{
    if (!cc || !cc->dev || (!data && len)) return ESP_ERR_INVALID_ARG;
    if (len == 0) return ESP_OK;

    // tx = [addr|WRITE_BURST] + data[len]
    size_t tx_len = len + 1;

    // маленькие записи держим на стеке, большие — malloc
    uint8_t stack_buf[1 + 32];
    uint8_t *tx = stack_buf;

    if (tx_len > sizeof(stack_buf)) {
        tx = (uint8_t *)malloc(tx_len);
        if (!tx) return ESP_ERR_NO_MEM;
    }

    tx[0] = (uint8_t)(addr | CC1101_WRITE_BURST);
    memcpy(&tx[1], data, len);

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = tx_len * 8;
    t.tx_buffer = tx;

    esp_err_t err = spi_device_polling_transmit(cc->dev, &t);

    if (tx != stack_buf) free(tx);
    return err;
}

esp_err_t cc1101_read_burst_reg(cc1101_t *cc, uint8_t addr, uint8_t *out, size_t len)
{
    if (!cc || !cc->dev || (!out && len)) return ESP_ERR_INVALID_ARG;
    if (len == 0) return ESP_OK;

    size_t n = len + 1; // первый байт — статус, дальше данные

    uint8_t stack_tx[1 + 32];
    uint8_t stack_rx[1 + 32];
    uint8_t *tx = stack_tx;
    uint8_t *rx = stack_rx;

    if (n > sizeof(stack_tx)) {
        tx = (uint8_t *)malloc(n);
        rx = (uint8_t *)malloc(n);
        if (!tx || !rx) { free(tx); free(rx); return ESP_ERR_NO_MEM; }
    }

    memset(tx, 0, n);
    memset(rx, 0, n);
    tx[0] = (uint8_t)(addr | CC1101_READ_BURST);

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = n * 8;
    t.tx_buffer = tx;
    t.rx_buffer = rx;

    esp_err_t err = spi_device_polling_transmit(cc->dev, &t);
    if (err == ESP_OK) memcpy(out, &rx[1], len);

    if (tx != stack_tx) { free(tx); free(rx); }
    return err;
}

esp_err_t cc1101_apply_preset_pairs_then_patable(cc1101_t *cc, const uint8_t *preset)
{
    if (!cc || !preset) return ESP_ERR_INVALID_ARG;

    int i = 0;

    // пары (addr,val) до 0,0
    while (!(preset[i] == 0x00 && preset[i + 1] == 0x00)) {
        uint8_t reg = preset[i++];
        uint8_t val = preset[i++];
        esp_err_t err = cc1101_write_reg(cc, reg, val);
        if (err != ESP_OK) return err;
    }

    i += 2; // skip 0,0

    // PATABLE 8 bytes
    return cc1101_write_burst_reg(cc, CC1101_PATABLE, &preset[i], 8);
}

void cc1101_power_on(bool on)
{
    gpio_config_t pwr_cfg = {
        .pin_bit_mask = 1ULL << PIN_POWER_EN,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&pwr_cfg);
    gpio_set_level(PIN_POWER_EN, on);
    vTaskDelay(pdMS_TO_TICKS(50));
}
