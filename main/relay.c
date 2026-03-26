#include "relay.h"
#include "board.h"
#include "esp_log.h"
#include "driver/i2c_master.h"

static const char *TAG = "relay";

static i2c_master_bus_handle_t i2c_bus = NULL;
static i2c_master_dev_handle_t tca9554_dev = NULL;
static uint8_t relay_states = 0x00;

static esp_err_t tca9554_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(tca9554_dev, buf, sizeof(buf), -1);
}

static esp_err_t tca9554_read_reg(uint8_t reg, uint8_t *val)
{
    esp_err_t ret = i2c_master_transmit_receive(tca9554_dev, &reg, 1, val, 1, -1);
    return ret;
}

static esp_err_t relay_flush(void)
{
    return tca9554_write_reg(TCA9554_REG_OUTPUT, relay_states);
}

esp_err_t relay_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = BOARD_I2C_SDA_PIN,
        .scl_io_num = BOARD_I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t ret = i2c_new_master_bus(&bus_cfg, &i2c_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = TCA9554_ADDR,
        .scl_speed_hz = BOARD_I2C_FREQ_HZ,
    };
    ret = i2c_master_bus_add_device(i2c_bus, &dev_cfg, &tca9554_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "TCA9554 device add failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure all 8 pins as outputs (0 = output)
    ret = tca9554_write_reg(TCA9554_REG_CONFIG, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "TCA9554 config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // All relays off
    relay_states = 0x00;
    ret = relay_flush();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "TCA9554 output init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "8 relays initialized (TCA9554 @ 0x%02X)", TCA9554_ADDR);
    return ESP_OK;
}

esp_err_t relay_set(uint8_t channel, bool state)
{
    if (channel >= NUM_RELAYS) return ESP_ERR_INVALID_ARG;

    if (state) {
        relay_states |= (1 << channel);
    } else {
        relay_states &= ~(1 << channel);
    }

    ESP_LOGI(TAG, "CH%d %s", channel + 1, state ? "ON" : "OFF");
    return relay_flush();
}

esp_err_t relay_toggle(uint8_t channel)
{
    if (channel >= NUM_RELAYS) return ESP_ERR_INVALID_ARG;

    relay_states ^= (1 << channel);
    bool state = (relay_states >> channel) & 1;

    ESP_LOGI(TAG, "CH%d toggled %s", channel + 1, state ? "ON" : "OFF");
    return relay_flush();
}

esp_err_t relay_set_all(bool state)
{
    relay_states = state ? 0xFF : 0x00;
    ESP_LOGI(TAG, "All relays %s", state ? "ON" : "OFF");
    return relay_flush();
}

uint8_t relay_get_states(void)
{
    return relay_states;
}
