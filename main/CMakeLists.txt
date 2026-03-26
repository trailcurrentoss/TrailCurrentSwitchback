idf_component_register(
    SRCS "main.c" "relay.c" "can_handler.c" "wifi_config.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_timer nvs_flash esp_wifi esp_https_ota esp_netif esp_eth
)
