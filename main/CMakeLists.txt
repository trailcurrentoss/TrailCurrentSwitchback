idf_component_register(
    SRCS "ota.c" "discovery.c" "main.c" "relay.c" "can_handler.c" "wifi_config.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_timer nvs_flash esp_wifi esp_netif esp_eth esp_http_server esp_app_format app_update
)

# Module address (0-2): idf.py build -DSWITCHBACK_ADDRESS=1
if(DEFINED SWITCHBACK_ADDRESS)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE SWITCHBACK_ADDRESS=${SWITCHBACK_ADDRESS})
endif()
