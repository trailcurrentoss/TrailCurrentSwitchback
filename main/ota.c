#include "ota.h"
#include "wifi_config.h"
#include "discovery.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_http_server.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "mdns.h"

static const char *TAG = "ota";

// ---------------------------------------------------------------------------
// OTA state
// ---------------------------------------------------------------------------

static volatile bool s_ota_running = false;
static volatile bool s_ota_complete = false;

// ---------------------------------------------------------------------------
// HTTP OTA server — accepts firmware upload at POST /ota
// ---------------------------------------------------------------------------

static esp_ota_handle_t s_ota_handle;
static const esp_partition_t *s_update_partition;

static esp_err_t ota_post_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "OTA upload started (%d bytes)", req->content_len);

    s_update_partition = esp_ota_get_next_update_partition(NULL);
    if (s_update_partition == NULL) {
        ESP_LOGE(TAG, "No OTA partition available");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No OTA partition");
        return ESP_FAIL;
    }

    esp_err_t err = esp_ota_begin(s_update_partition, OTA_WITH_SEQUENTIAL_WRITES, &s_ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA begin failed");
        return ESP_FAIL;
    }

    char buf[1024];
    int received = 0;
    int total = 0;

    while (total < req->content_len) {
        received = httpd_req_recv(req, buf, sizeof(buf));
        if (received <= 0) {
            if (received == HTTPD_SOCK_ERR_TIMEOUT) {
                continue;
            }
            ESP_LOGE(TAG, "OTA receive error");
            esp_ota_abort(s_ota_handle);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Receive failed");
            return ESP_FAIL;
        }

        err = esp_ota_write(s_ota_handle, buf, received);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_write failed: %s", esp_err_to_name(err));
            esp_ota_abort(s_ota_handle);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Write failed");
            return ESP_FAIL;
        }

        total += received;
        if ((total % (64 * 1024)) == 0 || total == req->content_len) {
            ESP_LOGI(TAG, "OTA progress: %d / %d bytes", total, req->content_len);
        }
    }

    err = esp_ota_end(s_ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA end failed");
        return ESP_FAIL;
    }

    err = esp_ota_set_boot_partition(s_update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Set boot partition failed");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "OTA upload complete, rebooting...");
    httpd_resp_sendstr(req, "OTA OK, rebooting...\n");

    s_ota_complete = true;
    return ESP_OK;
}

static httpd_handle_t start_ota_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server");
        return NULL;
    }

    httpd_uri_t ota_uri = {
        .uri       = "/ota",
        .method    = HTTP_POST,
        .handler   = ota_post_handler,
    };
    httpd_register_uri_handler(server, &ota_uri);

    ESP_LOGI(TAG, "OTA HTTP server started on port %d", config.server_port);
    ESP_LOGI(TAG, "Upload firmware: curl -X POST http://%s.local/ota --data-binary @build/switchback.bin",
             wifi_config_get_hostname());

    return server;
}

// ---------------------------------------------------------------------------
// OTA task — runs on its own FreeRTOS task to avoid blocking CAN
// ---------------------------------------------------------------------------

static void ota_task_fn(void *arg)
{
    ESP_LOGI(TAG, "=== Entering OTA mode ===");

    if (!wifi_connect()) {
        ESP_LOGE(TAG, "WiFi connection failed — aborting OTA");
        s_ota_running = false;
        vTaskDelete(NULL);
        return;
    }

    // Start mDNS for .local hostname resolution
    mdns_init();
    mdns_hostname_set(wifi_config_get_hostname());
    mdns_instance_name_set("TrailCurrent Switchback OTA");
    ESP_LOGI(TAG, "mDNS hostname: %s.local", wifi_config_get_hostname());

    httpd_handle_t server = start_ota_server();

    // Wait for upload or timeout
    s_ota_complete = false;
    int64_t start = esp_timer_get_time();

    while (!s_ota_complete) {
        vTaskDelay(pdMS_TO_TICKS(100));
        int64_t elapsed_ms = (esp_timer_get_time() - start) / 1000;
        if (elapsed_ms >= OTA_TIMEOUT_MS) {
            ESP_LOGW(TAG, "OTA timeout — no upload received");
            break;
        }
    }

    // Cleanup
    if (server) {
        httpd_stop(server);
    }
    mdns_free();
    wifi_disconnect();

    if (s_ota_complete) {
        ESP_LOGI(TAG, "Restarting with new firmware...");
        vTaskDelay(pdMS_TO_TICKS(500));
        esp_restart();
    }

    ESP_LOGI(TAG, "=== OTA mode exited, resuming normal operation ===");
    s_ota_running = false;
    vTaskDelete(NULL);
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void ota_init(void)
{
    ESP_LOGI(TAG, "OTA ready — will respond to CAN 0x00 trigger");
}

bool ota_is_running(void)
{
    return s_ota_running;
}

void ota_handle_trigger(const uint8_t *data, uint8_t len)
{
    if (len < 3) return;

    // Compare against this device's MAC
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);

    if (data[0] != mac[3] || data[1] != mac[4] || data[2] != mac[5]) {
        ESP_LOGD(TAG, "OTA trigger ignored (MAC mismatch)");
        return;
    }

    if (s_ota_running) {
        ESP_LOGW(TAG, "OTA already in progress — ignoring trigger");
        return;
    }
    if (discovery_is_running()) {
        ESP_LOGW(TAG, "Discovery in progress — cannot start OTA");
        return;
    }
    if (!wifi_config_has_credentials()) {
        ESP_LOGE(TAG, "OTA triggered but no WiFi credentials available");
        return;
    }

    s_ota_running = true;
    xTaskCreate(ota_task_fn, "ota", 8192, NULL, 3, NULL);
}
