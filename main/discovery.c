#include "discovery.h"
#include "wifi_config.h"
#include "board.h"

#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_app_desc.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "mdns.h"
#include "driver/gpio.h"

static const char *TAG = "discovery";

// ---------------------------------------------------------------------------
// Module identity
// ---------------------------------------------------------------------------

#define MODULE_TYPE "switchback"

#ifndef SWITCHBACK_ADDRESS
#define SWITCHBACK_ADDRESS 0
#endif
#if SWITCHBACK_ADDRESS < 0 || SWITCHBACK_ADDRESS > 2
#error "SWITCHBACK_ADDRESS must be 0-2"
#endif

// ---------------------------------------------------------------------------
// Discovery state
// ---------------------------------------------------------------------------

static volatile bool s_confirmed = false;
static volatile bool s_discovery_running = false;

// ---------------------------------------------------------------------------
// Status LED blink (WS2812 on GPIO38 — use simple GPIO toggle for blink)
// ---------------------------------------------------------------------------

static esp_timer_handle_t s_led_timer = NULL;
static bool s_led_state = false;

static void led_blink_cb(void *arg)
{
    s_led_state = !s_led_state;
    gpio_set_level(RGB_LED_PIN, s_led_state ? 1 : 0);
}

static void led_blink_start(void)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << RGB_LED_PIN,
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io);

    esp_timer_create_args_t args = {
        .callback = led_blink_cb,
        .name = "disc_led",
    };
    esp_timer_create(&args, &s_led_timer);
    esp_timer_start_periodic(s_led_timer, 125000);  // 125 ms = 4 Hz
}

static void led_blink_stop(void)
{
    if (s_led_timer) {
        esp_timer_stop(s_led_timer);
        esp_timer_delete(s_led_timer);
        s_led_timer = NULL;
    }
    gpio_set_level(RGB_LED_PIN, 0);
}

// ---------------------------------------------------------------------------
// mDNS service advertisement with TXT records
// ---------------------------------------------------------------------------

static void discovery_mdns_start(void)
{
    const char *hostname = wifi_config_get_hostname();

    // Get firmware version from app descriptor
    const esp_app_desc_t *app = esp_app_get_description();

    char addr_str[4];
    char canid_str[8];
    snprintf(addr_str, sizeof(addr_str), "%d", SWITCHBACK_ADDRESS);
    snprintf(canid_str, sizeof(canid_str), "0x%02X", CAN_ID_STATUS_BASE + SWITCHBACK_ADDRESS);

    mdns_init();
    mdns_hostname_set(hostname);
    mdns_instance_name_set("TrailCurrent Module");

    mdns_txt_item_t txt[] = {
        { "type",  MODULE_TYPE },
        { "addr",  addr_str },
        { "canid", canid_str },
        { "fw",    app->version },
    };

    mdns_service_add("TrailCurrent Discovery", "_trailcurrent", "_tcp",
                     80, txt, sizeof(txt) / sizeof(txt[0]));

    ESP_LOGI(TAG, "mDNS discovery: %s.local type=%s addr=%s canid=%s fw=%s",
             hostname, MODULE_TYPE, addr_str, canid_str, app->version);
}

// ---------------------------------------------------------------------------
// HTTP confirmation endpoint -- Headwaters calls this to acknowledge
// ---------------------------------------------------------------------------

static esp_err_t confirm_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Discovery confirmed by Headwaters");
    httpd_resp_sendstr(req, "confirmed\n");
    s_confirmed = true;
    return ESP_OK;
}

static httpd_handle_t discovery_start_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server");
        return NULL;
    }

    httpd_uri_t confirm_uri = {
        .uri     = "/discovery/confirm",
        .method  = HTTP_GET,
        .handler = confirm_handler,
    };
    httpd_register_uri_handler(server, &confirm_uri);

    return server;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void discovery_init(void)
{
    ESP_LOGI(TAG, "Discovery ready — will respond to CAN 0x02 trigger");
}

static void discovery_task_fn(void *arg)
{
    if (!wifi_config_has_credentials()) {
        ESP_LOGE(TAG, "Discovery triggered but no WiFi credentials — cannot respond");
        s_discovery_running = false;
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "=== Entering discovery mode ===");

    led_blink_start();
    if (!wifi_connect()) {
        ESP_LOGE(TAG, "WiFi connection failed — aborting discovery");
        led_blink_stop();
        s_discovery_running = false;
        vTaskDelete(NULL);
        return;
    }
    discovery_mdns_start();
    httpd_handle_t server = discovery_start_server();

    // Wait for confirmation or timeout
    s_confirmed = false;
    int64_t start = esp_timer_get_time();

    while (!s_confirmed) {
        vTaskDelay(pdMS_TO_TICKS(100));
        int64_t elapsed_ms = (esp_timer_get_time() - start) / 1000;
        if (elapsed_ms >= DISCOVERY_TIMEOUT_MS) {
            ESP_LOGW(TAG, "Discovery timeout — no confirmation received");
            break;
        }
    }

    // Cleanup
    if (server) {
        httpd_stop(server);
    }
    mdns_free();
    wifi_disconnect();
    led_blink_stop();

    if (s_confirmed) {
        ESP_LOGI(TAG, "=== Discovery complete — module registered ===");
    } else {
        ESP_LOGI(TAG, "=== Discovery timed out — will respond to next trigger ===");
    }

    s_discovery_running = false;
    vTaskDelete(NULL);
}

void discovery_handle_trigger(void)
{
    if (s_discovery_running) {
        ESP_LOGW(TAG, "Discovery already in progress — ignoring trigger");
        return;
    }
    s_discovery_running = true;
    xTaskCreate(discovery_task_fn, "discovery", 8192, NULL, 3, NULL);
}
