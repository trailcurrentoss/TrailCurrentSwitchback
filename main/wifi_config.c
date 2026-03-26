#include "wifi_config.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_timer.h"
#include <string.h>

static const char *TAG = "wifi_cfg";

#define NVS_NAMESPACE   "wifi_config"
#define NVS_KEY_SSID    "ssid"
#define NVS_KEY_PASS    "password"
#define CONFIG_TIMEOUT_US (5 * 1000 * 1000) // 5 seconds

static nvs_handle_t nvs_hdl;

// Multi-message receive state machine
static struct {
    bool receiving;
    uint8_t ssid_len;
    uint8_t pass_len;
    uint8_t ssid_chunks;
    uint8_t pass_chunks;
    uint8_t rx_ssid_chunks;
    uint8_t rx_pass_chunks;
    char ssid_buf[33];
    char pass_buf[64];
    int64_t last_msg_time;
} state;

esp_err_t wifi_config_init(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition truncated, erasing...");
        nvs_flash_erase();
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) return ret;

    ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_hdl);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed: %s", esp_err_to_name(ret));
        return ret;
    }

    memset(&state, 0, sizeof(state));
    ESP_LOGI(TAG, "NVS initialized");
    return ESP_OK;
}

bool wifi_config_load(char *ssid, size_t ssid_len, char *password, size_t pass_len)
{
    size_t s_len = ssid_len;
    size_t p_len = pass_len;

    if (nvs_get_str(nvs_hdl, NVS_KEY_SSID, ssid, &s_len) != ESP_OK) return false;
    if (nvs_get_str(nvs_hdl, NVS_KEY_PASS, password, &p_len) != ESP_OK) return false;
    if (s_len <= 1) return false;

    ESP_LOGI(TAG, "Loaded SSID: %s", ssid);
    return true;
}

static bool save_credentials(const char *ssid, const char *password)
{
    ESP_LOGI(TAG, "Saving SSID: %s", ssid);
    if (nvs_set_str(nvs_hdl, NVS_KEY_SSID, ssid) != ESP_OK) return false;
    if (nvs_set_str(nvs_hdl, NVS_KEY_PASS, password) != ESP_OK) return false;
    if (nvs_commit(nvs_hdl) != ESP_OK) return false;
    ESP_LOGI(TAG, "Credentials saved to NVS");
    return true;
}

static void handle_start(const uint8_t *data)
{
    ESP_LOGI(TAG, "Config start received");
    memset(&state, 0, sizeof(state));
    state.receiving = true;
    state.ssid_len = data[1];
    state.pass_len = data[2];
    state.ssid_chunks = data[3];
    state.pass_chunks = data[4];
    state.last_msg_time = esp_timer_get_time();

    ESP_LOGI(TAG, "Expecting SSID: %d bytes in %d chunks, Password: %d bytes in %d chunks",
             state.ssid_len, state.ssid_chunks, state.pass_len, state.pass_chunks);
}

static void handle_ssid_chunk(const uint8_t *data)
{
    if (!state.receiving) return;

    uint8_t idx = data[1];
    uint8_t offset = idx * 6;
    if (idx >= state.ssid_chunks) return;

    for (int i = 0; i < 6 && (offset + i) < state.ssid_len; i++) {
        state.ssid_buf[offset + i] = data[2 + i];
    }
    state.rx_ssid_chunks++;
    state.last_msg_time = esp_timer_get_time();
}

static void handle_pass_chunk(const uint8_t *data)
{
    if (!state.receiving) return;

    uint8_t idx = data[1];
    uint8_t offset = idx * 6;
    if (idx >= state.pass_chunks) return;

    for (int i = 0; i < 6 && (offset + i) < state.pass_len; i++) {
        state.pass_buf[offset + i] = data[2 + i];
    }
    state.rx_pass_chunks++;
    state.last_msg_time = esp_timer_get_time();
}

static void handle_end(const uint8_t *data)
{
    if (!state.receiving) return;

    if (state.rx_ssid_chunks != state.ssid_chunks ||
        state.rx_pass_chunks != state.pass_chunks) {
        ESP_LOGE(TAG, "Missing chunks (SSID: %d/%d, Pass: %d/%d)",
                 state.rx_ssid_chunks, state.ssid_chunks,
                 state.rx_pass_chunks, state.pass_chunks);
        state.receiving = false;
        return;
    }

    uint8_t checksum = 0;
    for (int i = 0; i < state.ssid_len; i++) checksum ^= state.ssid_buf[i];
    for (int i = 0; i < state.pass_len; i++) checksum ^= state.pass_buf[i];

    if (checksum != data[1]) {
        ESP_LOGE(TAG, "Checksum mismatch (expected 0x%02X, got 0x%02X)", data[1], checksum);
        state.receiving = false;
        return;
    }

    state.ssid_buf[state.ssid_len] = '\0';
    state.pass_buf[state.pass_len] = '\0';

    if (save_credentials(state.ssid_buf, state.pass_buf)) {
        ESP_LOGI(TAG, "Credentials saved successfully");
    }
    state.receiving = false;
}

void wifi_config_handle_can(const uint8_t *data, uint8_t length)
{
    if (length < 1) return;

    switch (data[0]) {
    case 0x01: handle_start(data); break;
    case 0x02: handle_ssid_chunk(data); break;
    case 0x03: handle_pass_chunk(data); break;
    case 0x04: handle_end(data); break;
    default:
        ESP_LOGW(TAG, "Unknown message type: 0x%02X", data[0]);
    }
}

void wifi_config_check_timeout(void)
{
    if (state.receiving && (esp_timer_get_time() - state.last_msg_time > CONFIG_TIMEOUT_US)) {
        ESP_LOGW(TAG, "Config timeout — resetting");
        memset(&state, 0, sizeof(state));
    }
}
